#include <ESP32Servo.h>

Servo servoPitch;

int servoPitchPin = 19;   // choose a PWM-capable GPIO
int pos = 0;

void setup() {
  servoPitch.setPeriodHertz(50);          // standard 50 Hz servo
  servoPitch.attach(servoPitchPin, 500, 2400); // min/max pulse width in microseconds
}

void loop() {
  // Sweep 0 → 180
  for (pos = 0; pos <= 180; pos++) {
    servoPitch.write(pos);
    delay(15);
  }

  delay(500);

  // Sweep 180 → 0
  for (pos = 180; pos >= 0; pos--) {
    servoPitch.write(pos);
    delay(15);
  }

  delay(500);
}