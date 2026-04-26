#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

// ===== Joystick Pin Configuration =====
#define JOYSTICK_X_PIN 15      // X-axis analog input
#define JOYSTICK_Y_PIN 2      // Y-axis analog input
#define JOYSTICK_BUTTON_PIN 32 // Joystick button (optional)

// ===== Joystick Calibration =====
#define JOYSTICK_DEADZONE 200      // Deadzone to avoid drift
#define JOYSTICK_MAX_ANGLE 45      // Max angle output in degrees
#define JOYSTICK_CENTER 2048       // Center value for 12-bit ADC (4095/2)

// ===== Function Prototypes =====
void joystick_init();
void joystick_read(float &x_angle, float &y_angle);
bool joystick_button_pressed();
void joystick_calibrate();

#endif
