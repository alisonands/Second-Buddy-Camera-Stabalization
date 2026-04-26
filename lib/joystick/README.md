# Joystick Control Module

This module adds analog joystick control to the ESP32 servo system.

## Configuration

Edit `joystick.h` to match your hardware setup:

- **JOYSTICK_X_PIN** (default 34): GPIO pin for X-axis (yaw control)
- **JOYSTICK_Y_PIN** (default 35): GPIO pin for Y-axis (pitch control)  
- **JOYSTICK_BUTTON_PIN** (default 32): GPIO pin for joystick button
- **JOYSTICK_DEADZONE** (default 200): Analog threshold to ignore drift
- **JOYSTICK_MAX_ANGLE** (default 45): Maximum desired angle in degrees

## Usage

The joystick values are automatically read in the main loop and converted to desired pitch/yaw setpoints:

- Joystick **X-axis** → Desired **Yaw**
- Joystick **Y-axis** → Desired **Pitch**

The PID controller then stabilizes the servos to match these desired angles using IMU feedback.

## Calibration

Auto-calibration runs on startup. For best results:
1. Power on the ESP32
2. Keep joystick centered at startup (it's being calibrated)
3. After the calibration message appears, you can use full joystick range

## Typical Pinout (for reference)

```
Joystick     ESP32
VRx   -----> GPIO 34 (ADC1_CH6)
VRy   -----> GPIO 35 (ADC1_CH7)
SW    -----> GPIO 32 (or any available GPIO)
+5V   -----> 5V/3.3V (depending on joystick)
GND   -----> GND
```

Note: If using 5V joystick, use voltage divider for analog inputs to protect ESP32 (max 3.3V).
