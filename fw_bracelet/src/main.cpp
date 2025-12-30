#include <Arduino.h>
#include <avr/io.h>

// Pin port bits (ATtiny1616 port naming)
#define PIN_nCHRG PIN_PC2
#define PIN_BTN   PIN_PC3
#define PIN_LED   PIN_PC1 // (LED cathode, active low)
#define PIN_GATE  PIN_PA6 // (DAC output)

// Gate voltages (volts) - must be <= 1.5V internal reference
// Adjust these values to the required GATE_V_{mode-name}
#define GATE_V_LOW  1.20f
#define GATE_V_MED  1.35f
#define GATE_V_HIGH 1.5f

// DAC resolution used by the ATtiny1616 DAC (8-bit)
#define DAC_MAX 255u

enum Mode { MODE_OFF = 0, MODE_LOW, MODE_MED, MODE_HIGH };

volatile Mode mode = MODE_OFF;

// Debounce variables
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
volatile bool lastBtnState = HIGH;
volatile bool btnState = HIGH;

// Debounce button
bool readButtonPressed() {
    bool reading = digitalRead(PIN_BTN);
    if (reading != lastBtnState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != btnState) {
            btnState = reading;
            if (btnState == LOW) {  // Button pressed (active low)
                lastBtnState = reading;
                return true;
            }
        }
    }
    lastBtnState = reading;
    return false;
}

void configurePins() {
  pinMode(PIN_nCHRG, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
}

// Setup DAC if available
void configureDAC() {
    // 1. Configure PA6 as output
    PORTA.DIRSET = PIN6_bm;

    // Set VREF = VDD (raw bits 0x3)
    VREF.CTRLA = (VREF.CTRLA & ~0x07) | 0x04; // set VREF to 1.5v

    // 2. Enable DAC0, enable output pin
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;

    // 3. Start with 0V
    DAC0.DATA = 0;
}

// Convert desired voltage (0..1.5V) to DAC code
uint8_t voltageToDac(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 1.5f) return (uint8_t)DAC_MAX;
  return (uint8_t)((v / 1.5f) * (float)DAC_MAX + 0.5f);
}

void dacSetVoltage(float voltage) {
  DAC0.DATA = voltageToDac(voltage);
}

void applyMode(Mode m) {
  mode = m;
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

void setup() {
  configurePins();
  configureDAC();
  applyMode(MODE_OFF);
}

void loop() {
  // button press cycles modes
  if (readButtonPressed()) {
    mode = (Mode)((mode + 1) % 4);
    applyMode(mode);
  }

  delay(20);
}