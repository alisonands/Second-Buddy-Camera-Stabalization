#include "joystick.h"

// Calibration offsets
static int calibration_x_offset = 0;
static int calibration_y_offset = 0;

void joystick_init() {
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  
  // Auto-calibrate on startup
  joystick_calibrate();
  Serial.println("Joystick initialized");
}

void joystick_calibrate() {
  Serial.println("Calibrating joystick...");
  
  // Read center values
  int sum_x = 0, sum_y = 0;
  const int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    sum_x += analogRead(JOYSTICK_X_PIN);
    sum_y += analogRead(JOYSTICK_Y_PIN);
    delay(5);
  }
  
  calibration_x_offset = sum_x / samples - JOYSTICK_CENTER;
  calibration_y_offset = sum_y / samples - JOYSTICK_CENTER;
  
  Serial.print("Calibration X offset: ");
  Serial.println(calibration_x_offset);
  Serial.print("Calibration Y offset: ");
  Serial.println(calibration_y_offset);
}

void joystick_read(float &x_angle, float &y_angle) {
  // Read raw ADC values
  int raw_x = analogRead(JOYSTICK_X_PIN);
  int raw_y = analogRead(JOYSTICK_Y_PIN);
  
  // Apply calibration offset
  int centered_x = raw_x - JOYSTICK_CENTER - calibration_x_offset;
  int centered_y = raw_y - JOYSTICK_CENTER - calibration_y_offset;
  
  // Apply deadzone
  if (abs(centered_x) < JOYSTICK_DEADZONE) {
    centered_x = 0;
  }
  if (abs(centered_y) < JOYSTICK_DEADZONE) {
    centered_y = 0;
  }
  
  // Map to angle range [0, 180] for servo control
  // ADC range is approximately [-2048, +2047] from center
  x_angle = map(centered_x, -2048, 2047, 0, 180);
  y_angle = map(centered_y, -2048, 2047, 0, 180);
}

bool joystick_button_pressed() {
  return digitalRead(JOYSTICK_BUTTON_PIN) == LOW;
}
