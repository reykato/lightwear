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
#define GATE_V_HIGH 1.7f

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

void configureVREF() {
  // Set VREF to 1.1v for ADC, 2.5v for DAC
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc | VREF_DAC0REFSEL_2V5_gc;
}

// Setup DAC if available
void configureDAC() {
    // Configure PA6 as output
    PORTA.DIRSET = PIN6_bm;

    // Enable DAC0, enable output pin
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;

    // Start with 0V
    DAC0.DATA = 0;
}

static inline void configureADCForVddMeasurement(void) {
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

float measureVdd(void) {
  // nominal internal bandgap voltage (use datasheet/calibration)
  const float VBG = 1.1f;

  // Configure ADC peripheral
  configureADCForVddMeasurement();

  // Single conversion returns accumulated sum when ACC32 is used.
  // Divide by accumulation count to get average per-sample reading.
  const float ACCUM_COUNT = 32.0f;

  uint16_t res_sum = readBandgapADC();
  float avg_reading = (float)res_sum / ACCUM_COUNT;

  // ADC resolution: 10-bit -> max = 1023
  const float ADC_MAX = 1023.0f;
  if (avg_reading <= 0.0f) return 0.0f;

  // VDD = Vbg * ADC_MAX / average_reading
  float vdd = VBG * (ADC_MAX / avg_reading);
  return vdd;
}

// Convert desired voltage as float (0..2.5V) to 8-bit number
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
  for (int i = 0; i < times; i++) {
    applyMode(mode);
    delay(delayMillis);
    applyMode(MODE_OFF);
    delay(delayMillis);
  }
}

void showBatteryLevel() {
  Mode beforeMode = globalMode;
  applyMode(MODE_OFF);
  delay(500);
  float batteryV = measureVdd();
  if (batteryV <= 3.75) { // 0% - 25%
    blinkStrip(MODE_LOW, 1, 400);
  } else if (batteryV <= 3.85) { // 25% - 50%
    blinkStrip(MODE_LOW, 2, 300);
  } else if (batteryV <= 3.95) { // 50% - 75%
    blinkStrip(MODE_MED, 3, 250);
  } else { // 75% - 100%
    blinkStrip(MODE_HIGH, 4, 200);
  }
  delay(500);
  applyMode(beforeMode);
}

void startSleep() {
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  sleep_cpu();
}

void wakeISR() {
  wakeRequested = true;
}

void handleWake() {
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
    globalMode = (Mode)((globalMode + 1) % 4);
    applyMode(globalMode);
  }
}

void setup() {
  configurePins();
  attachInterrupt(digitalPinToInterrupt(PIN_BTN), wakeISR, CHANGE);
  configureVREF();
  configureDAC();
  blinkOnboardLED(2, 200);

  applyMode(globalMode);
}

void loop() {
  startSleep();

  // If ISR requested wake handling, do it here (safe to use delays, millis, etc.)
  if (wakeRequested) {
    handleWake();
  }
}