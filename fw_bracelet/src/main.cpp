#include <Arduino.h>
#include <avr/io.h>
#include <avr/sleep.h>

// Pin port bits (ATtiny1616 port naming)
#define PIN_nCHRG PIN_PC2
#define PIN_BTN   PIN_PA5
#define PIN_LED   PIN_PC1 // (LED cathode, active low)
#define PIN_GATE  PIN_PA6 // (DAC output)

// Gate voltages (volts) - must be <= 1.5V internal reference
// Adjust these values to the required GATE_V_{mode-name}
#define GATE_V_LOW  1.35f
#define GATE_V_MED  1.55f
#define GATE_V_HIGH 1.8f

// DAC resolution used by the ATtiny1616 DAC (8-bit)
#define DAC_MAX 255u

enum Mode { MODE_OFF = 0, MODE_LOW, MODE_MED, MODE_HIGH };

volatile Mode globalMode = MODE_OFF;

// Flag set by ISR to request wake handling in main loop
volatile bool wakeRequested = false;

const unsigned long debounceDelayMillis = 50;
const unsigned long longPressMillis = 2000;

void configurePins() {
  pinMode(PIN_nCHRG, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
}

// Setup DAC if available
void configureDAC() {
    // Configure PA6 as output
    PORTA.DIRSET = PIN6_bm;

    // Set VREF to 2.5v (raw bit mask 0x02)
    VREF.CTRLA = (VREF.CTRLA & ~0x07) | 0x02;

    // Enable DAC0, enable output pin
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;

    // Start with 0V
    DAC0.DATA = 0;
}

// --- VDD measurement via ADC (internal bandgap) -------------------------
// Note: The exact ADC mux/bitfield symbols depend on the device header.
// Adjust `ADC_MUXPOS_BANDGAP_gc` / `ADC_REFSEL_VDD_gc` if your toolchain
// uses slightly different names. The bandgap nominal is typically ~1.1V;
// use the datasheet or factory calibration for best accuracy.

static inline void configureADCForVddMeasurement(void) {
  // Enable ADC, keep other bits default
  ADC0.CTRLA = ADC_ENABLE_bm;

  // Use available reference and resolution macros
  #if defined(ADC_REFSEL_VDD_gc)
    #define _ADC_REF_VDD ADC_REFSEL_VDD_gc
  #elif defined(ADC_REFSEL_VDDREF_gc)
    #define _ADC_REF_VDD ADC_REFSEL_VDDREF_gc
  #else
    #define _ADC_REF_VDD 0
  #endif

  #if defined(ADC_RESSEL_12BIT_gc)
    #define _ADC_RES_12 ADC_RESSEL_12BIT_gc
  #elif defined(ADC_RESSEL_10BIT_gc)
    #define _ADC_RES_12 ADC_RESSEL_10BIT_gc
  #else
    #define _ADC_RES_12 0
  #endif

    ADC0.CTRLC = _ADC_REF_VDD | _ADC_RES_12;

    // Choose a moderate prescaler so ADC clock is within spec
  #if defined(ADC_PRESC_DIV64_gc)
    ADC0.CTRLB = ADC_PRESC_DIV64_gc;
  #elif defined(ADC_PRESC_DIV32_gc)
    ADC0.CTRLB = ADC_PRESC_DIV32_gc;
  #endif

  // Small sample time; increase if you see unstable readings
  ADC0.SAMPCTRL = 8;

  // Do not set MUXPOS here permanently; the read function will select bandgap
}

static inline uint16_t readBandgapADC(void) {
  // Select internal bandgap as positive input
  #ifdef ADC_MUXPOS_BANDGAP_gc
    ADC0.MUXPOS = ADC_MUXPOS_BANDGAP_gc;
  #elif defined(ADC_MUXPOS_INTREF_gc)
    ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  #else
    // Fallback: try value 0x1E which is commonly the internal reference selector
    ADC0.MUXPOS = 0x1E;
  #endif

  // Start single conversion
  ADC0.COMMAND = ADC_STCONV_bm;

  // Wait for conversion complete
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {}

  // Clear ready flag
  ADC0.INTFLAGS = ADC_RESRDY_bm;

  // Return result
  return ADC0.RES;
}

float measureVdd(void) {
  // nominal internal bandgap voltage (use datasheet/calibration)
  const float VBG = 1.1f;

  // Configure ADC peripheral
  configureADCForVddMeasurement();

  // Take several samples and average
  const int samples = 8;
  uint32_t sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += readBandgapADC();
    delay(2);
  }
  uint32_t avg = sum / samples;

  // ADC resolution: 12-bit -> max = 4095
  const float ADC_MAX = 4095.0f;
  if (avg == 0) return 0.0f;

  // VDD = Vbg * ADC_MAX / reading
  float vdd = VBG * (ADC_MAX / (float)avg);
  return vdd;
}

// Convert desired voltage (0..2.5V) to DAC code
uint8_t voltageToDac(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 2.5f) return (uint8_t)DAC_MAX;
  return (uint8_t)((v / 2.5f) * (float)DAC_MAX + 0.5f);
}

void dacSetVoltage(float voltage) {
  DAC0.DATA = voltageToDac(voltage);
}

void applyMode(Mode m) {
  globalMode = m;
  switch (m) {
    case MODE_OFF:
      // set DAC output to 0
      dacSetVoltage(0.0f);
      break;
    case MODE_LOW:
      dacSetVoltage(GATE_V_LOW);
      break;
    case MODE_MED:
      dacSetVoltage(GATE_V_MED);
      break;
    case MODE_HIGH:
      dacSetVoltage(GATE_V_HIGH);
      break;
  }
}

void blinkOnboardLED(int times, unsigned int delayMillis) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(delayMillis);
    digitalWrite(PIN_LED, HIGH);
    delay(delayMillis);
  }
}

void blinkStrip(Mode mode, int times, unsigned int delayMillis) {
  Mode beforeMode = globalMode;
  for (int i = 0; i < times; i++) {
    applyMode(MODE_OFF);
    delay(delayMillis);
    applyMode(mode);
    delay(delayMillis);
  }
  applyMode(beforeMode);
}

void showBatteryLevel() {
  float batteryV = measureVdd();
  if (batteryV <= 3.75) { // 0% - 25%
    blinkStrip(MODE_LOW, 1, 500);
  } else if (batteryV <= 3.85) { // 25% - 50%
    blinkStrip(MODE_LOW, 2, 300);
  } else if (batteryV <= 3.95) { // 50% - 75%
    blinkStrip(MODE_MED, 3, 250);
  } else { // 75% - 100%
    blinkStrip(MODE_HIGH, 4, 200);
  }
}

void startSleep() {
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  sleep_cpu();   // CPU sleeps here

  // Execution resumes here after wake-up
}

// ISR should be minimal: set a flag and return
void wakeISR() {
  wakeRequested = true;
  // sleep_disable();
}

// Handle the wake event in the main context (not in ISR)
void handleWake() {
  // clear request early to avoid re-entrancy while handling
  wakeRequested = false;

  delay(debounceDelayMillis);
  if (!digitalRead(PIN_BTN)) {
    unsigned long startTime = millis();
    unsigned long endTime = startTime;
    while (!digitalRead(PIN_BTN)) {
      endTime = millis();
      delay(1);
      if (endTime - startTime >= longPressMillis) {
        showBatteryLevel();
        return;
      }
    }
    globalMode = (Mode)((globalMode + 1) % 4);
    applyMode(globalMode);
  }
}

void setup() {
  configurePins();

  // Blink when code resets
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
  }

  attachInterrupt(
    digitalPinToInterrupt(PIN_BTN),
    wakeISR,
    CHANGE
  );
  configureDAC();
  applyMode(globalMode);

}

void loop() {
  startSleep();

  // If ISR requested wake handling, do it here (safe to use delays, millis, etc.)
  if (wakeRequested) {
    handleWake();
  }
}