// ESP32 PWM on pin 25 at 25 kHz, 10% duty cycle

const int pwmPin = 25;
const int pwmChannel = 0;
const int pwmFreq = 19000;     // 25 kHz
const int pwmResolution = 10;  // 10-bit resolution (0–1023)

void setup() {
  // Configure PWM functionality
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);

  // Attach the channel to GPIO 25
  ledcAttachPin(pwmPin, pwmChannel);

  // Set duty cycle to 10% → 10% of 1023 ≈ 102
  int duty = (1023 * 15) / 100;
  ledcWrite(pwmChannel, duty);
}

void loop() {
  // Nothing needed here
}
