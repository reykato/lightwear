#include <Arduino.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Forward declarations
static void wakeISR(void);

// Pin port bits (ATtiny1616 port naming)
#define PIN_nCHRG PIN_PC2
#define PIN_BTN   PIN_PA5
#define PIN_GATE  PIN_PA6 // (DAC output)

// Raw 8-bit DAC codes written directly to DAC0.DATA
#define DAC_LOW   170
#define DAC_HIGH  196

// Breathing mode ramps between these DAC codes
#define BREATHE_LOW   136
#define BREATHE_HIGH  196

enum Mode { MODE_OFF = 0, MODE_LOW, MODE_HIGH, MODE_BREATHE };
#define MODE_COUNT 4

// Raw DAC code for each static mode. MODE_BREATHE is driven dynamically,
// so its table entry is unused (kept only to keep the array in bounds).
static const uint8_t modeDacValues[] = {
    0,        // MODE_OFF
    DAC_LOW,  // MODE_LOW
    DAC_HIGH, // MODE_HIGH
    0         // MODE_BREATHE (unused)
};

volatile Mode globalMode = MODE_OFF;

// Flag set by button ISR to request wake handling in main loop
volatile bool wakeRequested = false;

// Flag set by the RTC PIT ISR (~every 15.6 ms) to advance the breathing effect
volatile bool breatheTick = false;

static constexpr unsigned long debounceDelayMillis = 50;
static constexpr long longPressMillis = 2000;

static void configurePins(void) {
  pinMode(PIN_nCHRG, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
}

static void configureVREF(void) {
  // Set VREF to 1.1v for ADC, 1.5v for DAC
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc | VREF_DAC0REFSEL_1V5_gc;
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

// --- RTC Periodic Interrupt Timer (breathing tick) --------------------------
// Runs off the internal 32.768 kHz ULP oscillator, which keeps running in
// standby sleep. CYC512 / 32768 Hz = 15.625 ms per tick.
// NOTE: if you selected "RTC" as the millis() timer source in the Tools menu,
// it will collide with this. megaTinyCore's default millis source is a TCB,
// which leaves the PIT free — keep it that way.
static void pitInit(void) {
  while (RTC.STATUS > 0) {}             // wait for any pending register syncs
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    // 32.768 kHz internal ULP
  RTC.PITINTCTRL = RTC_PI_bm;           // enable the periodic interrupt
}

static void pitEnable(void) {
  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
  RTC.PITCTRLA = RTC_PERIOD_CYC512_gc | RTC_PITEN_bm; // ~15.6 ms period
}

static void pitDisable(void) {
  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
  RTC.PITCTRLA = 0;                     // stop the periodic interrupt
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;          // clear the interrupt flag
  breatheTick = true;
}

// --- Breathing effect -------------------------------------------------------
// Ramp the raw DAC code up and down between BREATHE_LOW and BREATHE_HIGH. The
// PIT wakes the MCU every ~15.6 ms; we only step one DAC code every BREATHE_DIV
// ticks so the sweep is slow. Each code is simply held for BREATHE_DIV ticks.
// Full breath ~= (BREATHE_HIGH - BREATHE_LOW) * 2 * BREATHE_DIV * 15.6 ms.
// Note: a wider BREATHE window = more codes = longer breath at the same
// BREATHE_DIV, so drop BREATHE_DIV if the sweep feels too slow.
#define BREATHE_DIV 3

static uint8_t breatheVal       = BREATHE_LOW;
static int8_t  breatheDir       = +1;
static uint8_t breatheTickCount = 0;

static void breatheInit(void) {
  breatheVal = BREATHE_LOW;
  breatheDir = +1;
  breatheTickCount = 0;
}

static void breatheUpdate(void) {
  if (++breatheTickCount < BREATHE_DIV) return; // not time to step yet
  breatheTickCount = 0;

  int16_t v = (int16_t)breatheVal + breatheDir;
  if (v >= BREATHE_HIGH)     { v = BREATHE_HIGH; breatheDir = -1; } // top, reverse
  else if (v <= BREATHE_LOW) { v = BREATHE_LOW;  breatheDir = +1; } // bottom, reverse

  breatheVal = (uint8_t)v;
  DAC0.DATA = breatheVal;
}

static inline void applyMode(Mode m) {
  globalMode = m;
  if (m == MODE_BREATHE) {
    breatheInit();
    DAC0.DATA = BREATHE_LOW;  // start of the breath
    pitEnable();              // begin ~15.6 ms periodic wake
  } else {
    pitDisable();             // no periodic wake in static / OFF modes
    DAC0.DATA = modeDacValues[m];
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
  if (batteryV <= 3.55) { // 0% - 25%
    blinkStrip(MODE_HIGH, 1, 400);
  } else if (batteryV <= 3.7) { // 25% - 50%
    blinkStrip(MODE_HIGH, 2, 300);
  } else if (batteryV <= 3.9) { // 50% - 75%
    blinkStrip(MODE_HIGH, 3, 250);
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
  PORTA.PIN5CTRL = PORT_ISC_BOTHEDGES_gc;     // PA5 = button, sense enabled for both edges
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
  // PORTA.PIN0CTRL = 0;
  // PORTA.PIN1CTRL = 0;
  // PORTA.PIN2CTRL = 0;
  // PORTA.PIN3CTRL = 0;
  // PORTA.PIN4CTRL = 0;
  PORTA.PIN5CTRL = 0; // attachInterrupt() will reconfigure PA5
  PORTA.PIN6CTRL = 0;
  // PORTA.PIN7CTRL = 0;

  // PORTB.PIN0CTRL = 0;
  // PORTB.PIN1CTRL = 0;
  // PORTB.PIN2CTRL = 0;
  // PORTB.PIN3CTRL = 0;
  // PORTB.PIN4CTRL = 0;
  // PORTB.PIN5CTRL = 0;

  // PORTC.PIN0CTRL = 0;
  // PORTC.PIN1CTRL = 0;
  // PORTC.PIN2CTRL = 0;
  // PORTC.PIN3CTRL = 0;
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
  // digitalWrite(PIN_LED, HIGH);
  // pinMode(PIN_LED, INPUT);

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
  // Deepest sleep mode on ATtiny1616. The PIT is already stopped here because
  // applyMode() calls pitDisable() for every non-breathe mode, so nothing will
  // wake the MCU except the button.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();

  // Turn off outputs and peripherals which will be reconfigured after wake
  shutdownPeripherals();
  disableInputBuffers();

  sleep_enable();
  interrupts();
  sleep_cpu();

  // Woke up here
  sleep_disable();
  restorePeripherals();
}

static void wakeISR(void) {
  wakeRequested = true;
}

static void handleWake(void) {
  // Clear request early to avoid re-entrancy while handling
  wakeRequested = false;

  delay(debounceDelayMillis);
  if (!digitalRead(PIN_BTN)) {
    unsigned long startTime = millis();
    unsigned long endTime = startTime;

    while (!digitalRead(PIN_BTN)) { // While button is held down
      endTime = millis();
      delay(1);
      if (endTime - startTime >= longPressMillis) { // Long press identified
        showBatteryLevel();
        return;
      }
    }

    // Short press: advance to the next mode (OFF -> LOW -> HIGH -> BREATHE -> OFF)
    applyMode((Mode)((globalMode + 1) % MODE_COUNT));
  }
}

void setup() {
  configurePins();
  attachInterrupt(digitalPinToInterrupt(PIN_BTN), wakeISR, CHANGE);
  configureVREF();
  configureDAC();
  pitInit();

  blinkStrip(MODE_LOW, 2, 200);
  applyMode(globalMode);
}

void loop() {
  if (wakeRequested) {
    handleWake();
    return;               // re-check state on next loop() before sleeping
  }

  switch (globalMode) {
    case MODE_BREATHE:
      if (breatheTick) {
        breatheTick = false;
        breatheUpdate();
      }
      startSleep();       // STANDBY: DAC keeps driving, PIT + button both wake
      break;

    case MODE_OFF:
      startDeepSleep();   // POWER_DOWN: lowest power, only the button wakes
      break;

    default:              // MODE_LOW / MODE_HIGH
      startSleep();       // STANDBY: DAC holds the static level
      break;
  }
}