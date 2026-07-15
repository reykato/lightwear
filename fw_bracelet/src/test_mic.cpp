/*
 * ATtiny1616 Mic -> LED Brightness Pipeline (v1, amplitude-only, no FFT)
 * --------------------------------------------------------------------
 * Goal: confirm the full signal chain works end-to-end --
 *   mic -> ADC -> DC removal -> AGC -> brightness mapping -> DAC -> strip
 *
 * This intentionally skips frequency analysis. It just reacts to overall
 * loudness, using a windowed peak-to-brightness mapping. Once this is
 * confirmed working on the bench, the FFT/band-splitting stage can be
 * inserted between AGC and brightness mapping.
 *
 * Board: ATtiny1616 (megaTinyCore)
 * Tools > Chip: ATtiny1616
 * Tools > Clock: 20MHz internal
 *
 * WIRING ASSUMPTION:
 *   - Mic module output -> PA4 (ADC0 AIN4) -- change ADC_PIN/MUXPOS if different
 *   - Mic module biased to ~VCC/2 (typical electret breakout w/ opamp)
 *   - dacSetVoltage(float) already implemented elsewhere, drives strip dimming
 */

#include <Arduino.h>

#define PIN_LED   PIN_PC1
#define PIN_nCHRG PIN_PC2
#define PIN_BTN   PIN_PA5
#define PIN_GATE  PIN_PA6 // (DAC output)


// ---------- Configuration ----------
#define ADC_PIN           PIN_PA1   // change to match your mic input pin
#define SAMPLE_WINDOW_MS  20        // how often we update brightness (ms) -- ~50Hz update rate

// AGC tuning
#define AGC_TARGET        400       // target peak amplitude (post-DC-removal, ADC counts)
#define AGC_MIN_GAIN      1         // gain stored as fixed point Q4 (i.e. divide by 16 later)
#define AGC_MAX_GAIN      255       // max ~16x gain in Q4
#define AGC_DECAY_SHIFT   7         // how fast AGC peak tracker decays (higher = slower)

// DC offset tracker tuning
#define DC_SHIFT          6         // running average shift factor (higher = slower/smoother)

// Brightness smoothing (decay) -- prevents flicker, gives a falling "tail"
#define BRIGHTNESS_ATTACK_SHIFT  1  // how fast brightness rises to a new peak (lower = faster)
#define BRIGHTNESS_DECAY_SHIFT   4  // how fast brightness falls after a peak (higher = slower)

// DAC output voltage range
#define DAC_VOLTAGE_MIN   1.2f
#define DAC_VOLTAGE_MAX   1.70f

// DAC resolution used by the ATtiny1616 DAC (8-bit)
#define DAC_MAX 255u


// ---------- Global state ----------
static int16_t  dcOffset       = 512;  // running DC offset estimate, init to expected mid-rail
static uint16_t agcPeak        = 1;    // running peak estimate used by AGC, avoid 0 (div by zero)
static uint8_t  agcGainQ4      = 16;   // current AGC gain in Q4 fixed point (16 = 1.0x)
static uint16_t windowPeak     = 0;    // peak seen during the current brightness update window
static float    smoothedLevel  = 0.0f; // smoothed 0.0-1.0 brightness level

static void configurePins(void) {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_nCHRG, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
}

// Setup DAC if available
static void setupDAC(void) {
		VREF.CTRLA = VREF.CTRLA | VREF_DAC0REFSEL_2V5_gc;

    // Configure PA6 as output
    PORTA.DIRSET = PIN6_bm;

    // Enable DAC0, enable output pin
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;

    // Start with 0V
    DAC0.DATA = 0;
}

// Convert desired voltage as float (0..2.5V) to 8-bit number
static inline uint8_t voltageToDac(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 2.5f) return (uint8_t)DAC_MAX;
  return (uint8_t)((v / 2.5f) * (float)DAC_MAX + 0.5f);
}

static inline void dacSetVoltage(float voltage) {
  DAC0.DATA = voltageToDac(voltage);
}

// ---------- ADC Setup ----------
void setupADC() {
	pinMode(ADC_PIN, INPUT);
	PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;

  ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
  ADC0.CTRLA = ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm;
  ADC0.CTRLB = ADC_SAMPNUM_ACC16_gc;   // 16x hardware oversampling
  ADC0.SAMPCTRL = 14;
  ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;    // <-- update to match ADC_PIN
}

uint16_t readADC() {
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  uint16_t raw = ADC0.RES;
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  return raw >> 4; // rescale 16x-accumulated sum back to ~10-bit range
}

// ---------- DC Offset Removal ----------
int16_t removeDC(uint16_t sample) {
  dcOffset += ((int16_t)sample - dcOffset) >> DC_SHIFT;
  return (int16_t)sample - dcOffset;
}

// ---------- AGC ----------
int16_t applyAGC(int16_t acSample) {
  uint16_t absSample = (acSample < 0) ? -acSample : acSample;

  if (absSample > agcPeak) {
    agcPeak = absSample;
  } else {
    agcPeak -= (agcPeak >> AGC_DECAY_SHIFT);
    if (agcPeak < 1) agcPeak = 1;
  }

  uint32_t desiredGainQ4 = ((uint32_t)AGC_TARGET * 16) / agcPeak;
  if (desiredGainQ4 > AGC_MAX_GAIN) desiredGainQ4 = AGC_MAX_GAIN;
  if (desiredGainQ4 < AGC_MIN_GAIN) desiredGainQ4 = AGC_MIN_GAIN;

  agcGainQ4 = agcGainQ4 + (((int16_t)desiredGainQ4 - (int16_t)agcGainQ4) >> 2);

  int32_t gained = ((int32_t)acSample * agcGainQ4) >> 4;
  if (gained > 32767) gained = 32767;
  if (gained < -32768) gained = -32768;

  return (int16_t)gained;
}

// ---------- Brightness Mapping ----------

// Maps a normalized 0.0-1.0 level to the configured DAC voltage range and
// sends it out. Kept separate so you can swap in gamma correction later.
void setLEDBrightness(float level) {
  if (level < 0.0f) level = 0.0f;
  if (level > 1.0f) level = 1.0f;

  float voltage = DAC_VOLTAGE_MIN + level * (DAC_VOLTAGE_MAX - DAC_VOLTAGE_MIN);
  dacSetVoltage(voltage);
}

// Converts the AGC'd peak-amplitude reading for this window into a
// smoothed 0.0-1.0 brightness level with fast attack / slow decay,
// then drives the DAC.
void updateBrightnessFromPeak(uint16_t peak) {
  // Normalize peak (post-AGC, so it should roughly track AGC_TARGET as "full scale")
  float target = (float)peak / (float)AGC_TARGET;
  if (target > 1.0f) target = 1.0f;

  if (target > smoothedLevel) {
    // Rise quickly toward new peak
    smoothedLevel += (target - smoothedLevel) / (1 << BRIGHTNESS_ATTACK_SHIFT);
  } else {
    // Fall slowly for a smooth trailing decay
    smoothedLevel += (target - smoothedLevel) / (1 << BRIGHTNESS_DECAY_SHIFT);
  }

  setLEDBrightness(smoothedLevel);
}

// ---------- Main Loop ----------
unsigned long windowStart = 0;

// void loop() {
//   uint16_t rawADC    = readADC();
//   int16_t  dcRemoved = removeDC(rawADC);
//   int16_t  agcSample = applyAGC(dcRemoved);

//   uint16_t absAGC = (agcSample < 0) ? -agcSample : agcSample;
//   if (absAGC > windowPeak) windowPeak = absAGC;

//   if (millis() - windowStart >= SAMPLE_WINDOW_MS) {
//     windowStart = millis();
//     updateBrightnessFromPeak(windowPeak);
//     windowPeak = 0;
//   }

//   // No delay -- sample as fast as practical within each window
// }

uint16_t restingRaw = 0;

void setup() {
	configurePins();
  setupADC();
  setupDAC();
  setLEDBrightness(1.0f);
	delay(200);
	setLEDBrightness(0.0f);
	delay(200);
	digitalWrite(PIN_LED, LOW);
	delay(200);
	digitalWrite(PIN_LED, HIGH);
	delay(200);
	digitalWrite(PIN_LED, LOW);


  // Average several readings at startup (assumes mic is quiet at power-on)
  uint32_t sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += readADC();
    delay(2);
  }
  restingRaw = sum / 32;
}

void loop() {
  uint16_t raw = readADC();
  int16_t deviation = (int16_t)raw - (int16_t)restingRaw;

  // Scale deviation to brightness -- adjust SENSITIVITY to taste
  const float SENSITIVITY = 8.0f; // try 4-20, higher = more amplified
  float level = 0.5f + (deviation / 1023.0f) * SENSITIVITY;

  setLEDBrightness(level);
}
