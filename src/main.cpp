#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <ESP32Servo.h>
#include "pid_pos.h"

// ------ DEFINE I2C PINS -------
#define SDA_PIN 21
#define SCL_PIN 22
#define BNO_ADDR 0x4B

// --------- BNO080 GYRO ----------
BNO080 myIMU;

// Servo
Servo servoPitch;
Servo servoYaw;
int servoPitchPin = 19;
int servoYawPin = 18;

// desired values (to be replaced w joystick inputs in loop later)
float desired_yaw = 0.0;
float desired_pitch = 0.0;

// PID VALUES
float Kp_pitch = 1.0, Ki_pitch = 0.001, Kd_pitch = 0.1;             // position control for pitch
float Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.0;                   // rate control for yaw
float Kp_yaw_angle = 1.0, Ki_yaw_angle = 0.0, Kd_yaw_angle = 0.0; // outer loop position control for yaw
// float dt = 0.05;                                                  // 50ms delay
// int desiredOmegaX = 20;
// int desiredOmegaY = 20;
float prev_err_roll, prev_err_pitch, prev_err_yaw;
float prev_int_roll, prev_int_pitch, prev_int_yaw;

// other
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 50; // ms = 20 Hz
float dt = 0.05;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(100);

  // servo setup
  servoPitch.setPeriodHertz(50);
  servoYaw.setPeriodHertz(50);
  servoPitch.attach(servoPitchPin, 500, 2400);
  servoYaw.attach(servoYawPin, 500, 2400);

  // init PID
  reset_pid();

  // Start BNO080
  if (!myIMU.begin(BNO_ADDR, Wire))
  {
    Serial.println("BNO080 not found 😭 BAD BAD BAD");
    while (1);
    // delay(10);
  }

  Serial.println("Connected, initializing...");

  delay(500);
  myIMU.enableRotationVector(50);
  delay(200);
  myIMU.enableGyro(50); // 50ms = 20Hz
  Serial.println("IMU ready. End of setup.");
}

void loop()
{
  // unsigned long now = millis();

  // if (now - lastLoopTime < loopInterval) {
  //   return; // wait until 50ms has passed
  // }

  // lastLoopTime = now;

    if (myIMU.hasReset())
    {
      Serial.println("IMU reset!");
      delay(100);
      myIMU.enableRotationVector(50);
      myIMU.enableGyro(50); // 50ms = 20Hz
    }

    if (myIMU.dataAvailable())
    {
      // ------------------------------------
      // ------GET VALUES FROM SENSOR--------
      // ------------------------------------
      float roll = myIMU.getRoll() * 180.0 / PI;
      float pitch = myIMU.getPitch() * 180.0 / PI;
      float yaw = myIMU.getYaw() * 180.0 / PI;

      // Get gyro rates (rad/s)
      float gx_yaw = myIMU.getGyroX();
      float gy_roll = myIMU.getGyroY();
      float gz_pitch = myIMU.getGyroZ();

      // ------------------------------------
      // ----JOYSTICK INPUT VALUES HERE------
      // ------------------------------------

      // note: update desired inputs to joystick inputs
      // float joystick_x, joystick_y;
      // joystick_read(joystick_x, joystick_y);

      // ------------------------------------
      // ------------PID CONTROL-------------
      // ------------------------------------

      // PITCH POSITION CONTROL
      float error_pitch = desired_pitch - pitch;
      pid_pos_eqn(error_pitch, Kp_pitch, Ki_pitch, Kd_pitch, prev_err_pitch, prev_int_pitch, dt);
      float output_pitch = PIDReturn[0];
      prev_err_pitch = PIDReturn[1];
      prev_int_pitch = PIDReturn[2];

      // Positional servo: base position + PID output, constrained to [0, 90]
      int pitch_servo_cmd = output_pitch;

      // YAW RATE CONTROL (with outer cascading position control)
      float max_yaw_rate = 3.0; // limit max rotation speed

      float error_yaw = desired_yaw - yaw;
      float desired_yaw_rate = Kp_yaw_angle * error_yaw; // connect to rate of rotation
      desired_yaw_rate = constrain(desired_yaw_rate, -3.0, 3.0);

      // control rate
      float error_yaw_rate = desired_yaw_rate - gx_yaw;
      pid_pos_eqn(error_yaw_rate, Kp_yaw, Ki_yaw, Kd_yaw, prev_err_yaw, prev_int_yaw, dt);
      float output_yaw = PIDReturn[0];
      prev_err_yaw = PIDReturn[1];
      prev_int_yaw = PIDReturn[2];

      // constrained to 90 (for now. because wires. will remove later)
      int yaw_servo_cmd = constrain(90 + (int)output_yaw, 0, 180);

      // Set servos
      servoPitch.write(pitch_servo_cmd); // pitch PID output
      servoYaw.write(yaw_servo_cmd);     // yaw PID output

      Serial.print(roll);
      Serial.print(",");
      Serial.print(pitch);
      Serial.print(",");
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(gx_yaw, 1);
      Serial.print(",");
      Serial.print(gy_roll, 1);
      Serial.print(",");
      Serial.print(gz_pitch, 1);
      Serial.print(",");
      Serial.print(pitch_servo_cmd);
      Serial.print(",");
      Serial.println(yaw_servo_cmd);
      // Serial.print("DATA: ");
      // Serial.println(myIMU.dataAvailable());
    }
  }