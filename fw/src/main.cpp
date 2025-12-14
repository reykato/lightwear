#include <Arduino.h>

// Pin definitions
#define DRV_nSHDN PIN_PA4  // PA4
#define ISET PIN_PA6       // PA6
#define USB_SBU1 PIN_PB4   // PB4
#define USB_SBU2 PIN_PB5   // PB5
#define BTN PIN_PC0        // PC0
#define LED_OFF PIN_PC2
#define LED_LOW PIN_PC1
#define LED_MED PIN_PA2
#define LED_HIGH PIN_PA1

// Modes
enum Mode { MODE_OFF, MODE_LOW, MODE_MED, MODE_HIGH };
Mode currentMode = MODE_OFF;

// Debouncing variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool lastBtnState = HIGH;
bool btnState = HIGH;

// Dithering variables (for fractional step between two adjacent 8-bit DAC values)
// Using the existing RC filter (4.7k + 100nF) the output will be the time-average
// of the DAC toggles between `low` and `high` values.
static float dith_acc = 0.0f;
static unsigned long lastToggleMicros = 0;
// Toggle period in microseconds (200us -> 5 kHz). Tune as needed.
static const unsigned long togglePeriodUs = 200UL;
// Desired effective 8-bit DAC value for the LOW mode. Set fractional part here.
// E.g. 170.37 will produce an average between 170 and 171.
static float desired_low_mode_value = 170.6f;

void dac_init() {
    // 1. Configure PA6 as output
    PORTA.DIRSET = PIN6_bm;

    // Set VREF = VDD (raw bits 0x3)
    VREF.CTRLA = (VREF.CTRLA & ~0x07) | 0x02; // clear DAC0REFSEL bits, set to 0x2

    // 2. Enable DAC0, enable output pin
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;

    // 3. Start with 0V
    DAC0.DATA = 0;
}

void dac_set_voltage(uint8_t value) {
    // 8-bit value: 0 = 0V, 255 = VREF
    DAC0.DATA = value;
}

// Non-blocking dither update: toggles between `low` and `high` based on fractional
// part of `desired`. Call frequently from `loop()`.
void dither_update(float desired, uint8_t low, uint8_t high) {
    unsigned long now = micros();
    if ((unsigned long)(now - lastToggleMicros) < togglePeriodUs) return;
    lastToggleMicros = now;

    float frac = desired - (float)low;
    if (frac <= 0.0f) {
        // always low
        dith_acc = 0.0f;
        dac_set_voltage(low);
        return;
    }
    if (frac >= 1.0f) {
        // always high
        dith_acc = 0.0f;
        dac_set_voltage(high);
        return;
    }

    dith_acc += frac;
    if (dith_acc >= 1.0f) {
        dith_acc -= 1.0f;
        dac_set_voltage(high);
    } else {
        dac_set_voltage(low);
    }
}

void setup() {
    // Configure pins
    pinMode(DRV_nSHDN, INPUT_PULLUP);  // Start with pull-up (high)
    pinMode(USB_SBU1, INPUT_PULLUP);
    pinMode(USB_SBU2, INPUT_PULLUP);
    pinMode(BTN, INPUT_PULLUP);
    pinMode(LED_OFF, OUTPUT);
    pinMode(LED_LOW, OUTPUT);
    pinMode(LED_MED, OUTPUT);
    pinMode(LED_HIGH, OUTPUT);
    dac_init();
}

void loop() {
    // Read USB signals
    bool usbHigh = (digitalRead(USB_SBU1) == HIGH) && (digitalRead(USB_SBU2) == HIGH);

    // Debounce button
    bool reading = digitalRead(BTN);
    if (reading != lastBtnState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != btnState) {
            btnState = reading;
            if (btnState == LOW) {  // Button pressed (active low)
                currentMode = (Mode)((currentMode + 1) % 4);
            }
        }
    }
    lastBtnState = reading;

    // Set mode indicators
    digitalWrite(LED_OFF, currentMode == MODE_OFF ? HIGH : LOW);
    digitalWrite(LED_LOW, currentMode == MODE_LOW ? HIGH : LOW);
    digitalWrite(LED_MED, currentMode == MODE_MED ? HIGH : LOW);
    digitalWrite(LED_HIGH, currentMode == MODE_HIGH ? HIGH : LOW);

    // Control DRV_nSHDN
    if (usbHigh || currentMode == MODE_OFF) {
        pinMode(DRV_nSHDN, OUTPUT);
        digitalWrite(DRV_nSHDN, LOW);
    } else {
        pinMode(DRV_nSHDN, INPUT_PULLUP);  // Release to high
    }

    // Control ISET (use dithering in LOW mode to synthesize fractional 8-bit values)
    uint16_t duty = 255;
    switch (currentMode) {
        case MODE_OFF:
            duty = 255; // DRV disabled / max
            // no dithering, hold high
            dac_set_voltage((uint8_t)duty);
            break;
        case MODE_LOW:
            // Use dithering between 170 and 171 to synthesize fractional values.
            // Set `desired_low_mode_value` above to change the exact fractional target.
            dither_update(desired_low_mode_value, 170u, 171u);
            break;
        case MODE_MED:
            duty = 168;
            dac_set_voltage((uint8_t)duty);
            break;
        case MODE_HIGH:
            duty = 165;
            dac_set_voltage((uint8_t)duty);
            break;
    }

    // Small non-blocking wait to avoid burning CPU; dithering timing uses micros().
    delayMicroseconds(50);
}
