#include <Arduino.h>
#include <avr/io.h>
#include <avr/sleep.h>

// Forward declarations
static void wakeISR(void);

// Pin port bits (ATtiny1616 port naming)
#define PIN_nCHRG PIN_PC2
#define PIN_BTN   PIN_PA5
#define PIN_LED   PIN_PC1 // (LED cathode, active low)
#define PIN_GATE  PIN_PA6 // (DAC output)

// Gate voltages (volts) - must be <= 2.5V internal reference
// Adjust these values to the required GATE_V_{mode-name}
#define GATE_V_LOW  1.35f
#define GATE_V_MED  1.55f
#define GATE_V_HIGH 1.7f

// DAC resolution used by the ATtiny1616 DAC (8-bit)
#define DAC_MAX 255u

enum Mode { MODE_OFF = 0, MODE_LOW, MODE_MED, MODE_HIGH };

static const float modeVoltages[] = {
    0.0f,
    GATE_V_LOW,
    GATE_V_MED,
    GATE_V_HIGH
};

volatile Mode globalMode = MODE_OFF;

// Flag set by ISR to request wake handling in main loop
volatile bool wakeRequested = false;

static constexpr unsigned long debounceDelayMillis = 50;
static constexpr long longPressMillis = 2000;

static void configurePins(void) {
  pinMode(PIN_nCHRG, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
}

static void configureVREF(void) {
  // Set VREF to 1.1v for ADC, 2.5v for DAC
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc | VREF_DAC0REFSEL_2V5_gc;
}

// Setup DAC if available
static void configureDAC(void) {
    // Configure PA6 as output
    PORTA.DIRSET = PIN6_bm;

    // Enable DAC0, enable output pin
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;

    // Start with 0V
    DAC0.DATA = 0;
}

static inline void configureADC(void) {
  // Configure ADC for measuring the internal bandgap (VDD as VREF).
  // Setup order: configure resolution and control fields first, then enable.

  ADC0.CTRLA = ADC_RESSEL_10BIT_gc; // Set ADC resolution to 10-bit
  ADC0.CTRLB = ADC_SAMPNUM_ACC32_gc; // Accumulate 32 samples for each measurement
  ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV32_gc;
  ADC0.CTRLD = ADC_INITDLY_DLY32_gc; // Initialization delay of 32 CLK_ADC cycles
  ADC0.SAMPCTRL = 20; // Extended sample length for internal bandgap (in CLK_ADC cycles)

  // Finally, enable ADC
  ADC0.CTRLA |= ADC_ENABLE_bm;
}

static inline uint16_t readBandgapADC(void) {
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;

  // Clear any pending result-ready flag, allow MUX/sample cap to settle
  ADC0.INTFLAGS = ADC_RESRDY_bm;

  // Allow the input MUX and sample cap to settle
  delayMicroseconds(10);

  // Start single conversion
  ADC0.COMMAND = ADC_STCONV_bm;

  // Wait for conversion complete
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {}

  // Read result first (reading can clear ready flag on some implementations)
  uint16_t res = ADC0.RES;

  // Ensure flag cleared
  ADC0.INTFLAGS = ADC_RESRDY_bm;

  return res;
}

static float measureVdd(void) {
  // nominal internal bandgap voltage (use datasheet/calibration)
  const float VBG = 1.1f;

  // Configure ADC peripheral
  configureADC();

  // Single conversion returns accumulated sum when ACC32 is used.
  // Divide by accumulation count to get average per-sample reading.
  const float ACCUM_COUNT = 32.0f;

  uint16_t res_sum = readBandgapADC();

  // Disable ADC after reading
  ADC0.CTRLA &= ~ADC_ENABLE_bm;

  float avg_reading = (float)res_sum / ACCUM_COUNT;

  // ADC resolution: 10-bit -> max = 1023
  const float ADC_MAX = 1023.0f;
  if (avg_reading <= 0.0f) return 0.0f;

  // VDD = Vbg * ADC_MAX / average_reading
  float vdd = VBG * (ADC_MAX / avg_reading);
  return vdd;
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

static inline void applyMode(Mode m) {
  globalMode = m;
  dacSetVoltage(modeVoltages[m]);
}

static void blinkOnboardLED(int times, unsigned int delayMillis) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(delayMillis);
    digitalWrite(PIN_LED, HIGH);
    delay(delayMillis);
  }
}

static void blinkStrip(Mode mode, int times, unsigned int delayMillis) {
  for (int i = 0; i < times; i++) {
    applyMode(mode);
    delay(delayMillis);
    applyMode(MODE_OFF);
    delay(delayMillis);
  }
}

static void showBatteryLevel(void) {
  Mode beforeMode = globalMode;
  applyMode(MODE_OFF);
  delay(500);
  float batteryV = measureVdd();
  if (batteryV <= 3.65) { // 0% - 25%
    blinkStrip(MODE_MED, 1, 400);
  } else if (batteryV <= 3.8) { // 25% - 50%
    blinkStrip(MODE_MED, 2, 300);
  } else if (batteryV <= 3.95) { // 50% - 75%
    blinkStrip(MODE_MED, 3, 250);
  } else { // 75% - 100%
    blinkStrip(MODE_HIGH, 4, 200);
  }
  delay(500);
  applyMode(beforeMode);
}

// Disable input buffers on all pins (except PA5) to minimise sleep current.
static void disableInputBuffers(void) {
  PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTA.PIN5CTRL = PORT_ISC_BOTHEDGES_gc;
  PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc; // PA6 = GATE output, input buffer not needed
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused

  PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused

  PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
  PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc; // PC1 = LED, set to INPUT before sleep
  PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; // PC2 = nCHRG, pull-up disabled, buffer disabled
  PORTC.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused
}

static void restoreInputBuffers(void) {
  // Reset all PINnCTRL registers to power-on default (input buffer on,
  // no pull-up, no interrupt). configurePins() and attachInterrupt()
  // called in restorePeripherals() will then set the correct modes.
  PORTA.PIN0CTRL = 0;
  PORTA.PIN1CTRL = 0;
  PORTA.PIN2CTRL = 0;
  PORTA.PIN3CTRL = 0;
  PORTA.PIN4CTRL = 0;
  PORTA.PIN5CTRL = 0; // attachInterrupt() will reconfigure PA5
  PORTA.PIN6CTRL = 0;
  PORTA.PIN7CTRL = 0;

  PORTB.PIN0CTRL = 0;
  PORTB.PIN1CTRL = 0;
  PORTB.PIN2CTRL = 0;
  PORTB.PIN3CTRL = 0;
  PORTB.PIN4CTRL = 0;
  PORTB.PIN5CTRL = 0;

  PORTC.PIN0CTRL = 0;
  PORTC.PIN1CTRL = 0;
  PORTC.PIN2CTRL = 0;
  PORTC.PIN3CTRL = 0;
}

static void startSleep(void) {
  set_sleep_mode(SLEEP_MODE_STANDBY);

  noInterrupts();       // cli()
  sleep_enable();

  interrupts();         // sei()
  sleep_cpu();          // go to sleep

  sleep_disable();
}

static void shutdownPeripherals(void) {
  // Ensure DAC output is zero, then disable the DAC
  DAC0.DATA = 0;
  DAC0.CTRLA &= ~DAC_ENABLE_bm;

  // Turn off onboard LED (active low) and set pin high-impedance
  digitalWrite(PIN_LED, HIGH);
  pinMode(PIN_LED, INPUT);

  // Disable ADC to save power
  ADC0.CTRLA &= ~ADC_ENABLE_bm;

  // Disable voltage references
  VREF.CTRLA = 0;
}

// Reconfigure peripherals after wake (VREF, pins, DAC, etc.)
static void restorePeripherals(void) {
  restoreInputBuffers();
  configureVREF();
  configurePins();
  attachInterrupt(digitalPinToInterrupt(PIN_BTN), wakeISR, CHANGE);
  configureDAC();
  applyMode(globalMode);
}

static void startDeepSleep(void) {
  // Use the deepest available sleep mode for ATtiny1616
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  noInterrupts();

  // Turn off outputs and peripherals we'll reconfigure after wake
  shutdownPeripherals();

  // Disable all input buffers and configure PA5 wake interrupt.
  // Must happen after shutdownPeripherals() (which calls pinMode/
  // digitalWrite and would overwrite PINnCTRL registers) and inside
  // the cli() guard so the ISC field is set before sei().
  disableInputBuffers();

  sleep_enable();

  interrupts();
  sleep_cpu();

  // Woke up here
  sleep_disable();

  // Restore peripherals needed after wake
  restorePeripherals();
}

static void wakeISR(void) {
  wakeRequested = true;
}

static void handleWake(void) {
  // clear request early to avoid re-entrancy while handling
  wakeRequested = false;

  delay(debounceDelayMillis);
  if (!digitalRead(PIN_BTN)) {
    unsigned long startTime = millis();
    unsigned long endTime = startTime;

    while (!digitalRead(PIN_BTN)) { // while button is held down
      endTime = millis();
      delay(1);
      if (endTime - startTime >= longPressMillis) { // long press identified
        showBatteryLevel();
        return;
      }
    }

    // short press
    applyMode((Mode)((globalMode + 1) % 4));
  }
}

void setup() {
  configurePins();
  attachInterrupt(digitalPinToInterrupt(PIN_BTN), wakeISR, CHANGE);
  configureVREF();
  configureDAC();

  blinkOnboardLED(2, 200);
  blinkStrip(MODE_HIGH, 2, 200);
  applyMode(globalMode);
}

void loop() {
  if (!wakeRequested) {
      if (globalMode == MODE_OFF)
        startDeepSleep();
      else
        startSleep();
  }

  if (wakeRequested)
      handleWake();
}