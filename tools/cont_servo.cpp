#include <ESP32Servo.h>

Servo servoYaw;

// Servo signal pin (change as needed)
const int servoYawPin = 18;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // can map
  // pos = map(-100, 100, 0, 180)

  servoYaw.write(180);
}