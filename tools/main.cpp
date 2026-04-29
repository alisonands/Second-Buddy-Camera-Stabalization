// notes
// gyro is sideways -> +x points down, +y points forward, +z points outward
// update: I dont think we have to change anything 👀
// so yaw is x, pitch is z

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <ESP32Servo.h>
#include "pid_pos.h"
#include "joystick.h"

// --------- I2C PINS ----------
#define SDA_PIN 21
#define SCL_PIN 22

#define BNO_ADDR 0x4B // <-- I2C ADDRESS

// --------- BNO080 GYRO ----------
BNO080 myIMU;

// Servo
Servo servoPitch;
Servo servoYaw;
int servoPitchPin = 19;
int servoYawPin = 18;

float AccOffsetX, AccOffsetY, AccOffsetZ;
float roll, pitch, yaw;

// desired values (to be replaced w joystick inputs in loop later)
float desired_yaw = 0.0;
// float desired_pitch = 0.0;

// PID VALUES
float Kp_pitch = 2.0, Ki_pitch = 0.0, Kd_pitch = 0.1; // position control for pitch
float Kp_yaw = 2.0, Ki_yaw = 0.0, Kd_yaw = 0.1;       // rate control for yaw
float Kp_yaw_angle = 2.0, Ki_yaw_angle = 0.0, Kd_yaw_angle = 0.1;   // outer loop position control for yaw
float dt = 0.05; // 50ms delay
int desiredOmegaX = 20;
int desiredOmegaY = 20;

float prev_err_roll, prev_err_pitch, prev_err_yaw;
float prev_int_roll, prev_int_pitch, prev_int_yaw;

void setup()
{ 
  Serial.begin(115200);
  delay(1000);

  // Start I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // reduced from 400kHz to 100kHz for stability. adjust as required
  delay(100);

  // servo setup
  servoPitch.setPeriodHertz(50);
  servoYaw.setPeriodHertz(50);
  servoPitch.attach(servoPitchPin, 500, 2400);
  servoYaw.attach(servoYawPin, 500, 2400);

  // joystick
  joystick_init();

  reset_pid();

  // Start BNO080
  if (!myIMU.begin(BNO_ADDR, Wire))
  {
    Serial.println("BNO080 not found 😭");
    while (1)
      delay(10);
  }

  // Enable Euler angles (via Rotation Vector)
  myIMU.enableRotationVector(50); // 50ms (20Hz)
  myIMU.enableGyro(50);           // 50ms = 20Hz

  Serial.println("BNO080 Euler Angles (deg)");
  Serial.println("End setup");
}

void calibrate()
{
  const int samples = 1000;
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++)
  {

    Serial.println("Calibrating accel... keep flat");
    sumX += (float)myIMU.getRoll();
    sumY += (float)myIMU.getPitch();
    sumZ += (float)myIMU.getYaw();
    delay(2);
  }
  AccOffsetX = sumX / samples;
  AccOffsetY = sumY / samples;
  AccOffsetZ = sumZ / samples;
}

void loop()
{
  if (myIMU.dataAvailable())
  {
    // Convert radians to degrees
    yaw = (myIMU.getRoll() - AccOffsetX) * 180.0f / PI;
    roll = (myIMU.getPitch() - AccOffsetY) * 180.0f / PI;
    pitch = (myIMU.getYaw() - AccOffsetZ) * 180.0f / PI;

    // check for packet

    // Get gyro rates (rad/s)
    float gx_yaw = myIMU.getGyroX(); // 
    float gy_roll = myIMU.getGyroY(); //
    float gz_pitch = myIMU.getGyroZ(); //

    // read values from joystick (desired values)
    float joystick_x, joystick_y;
    joystick_read(joystick_x, joystick_y);
    
    // --- PITCH POSITION CONTROL (Positional Servo) ---
    // Map joystick_y [0, 180] to desired pitch [0, 90] degrees
    float desired_pitch = joystick_y / 2.0;
    
    // error pitch
    float error_pitch = desired_pitch - pitch;

    pid_pos_eqn(error_pitch, Kp_pitch, Ki_pitch, Kd_pitch, prev_err_pitch, prev_int_pitch, dt);
    float output_pitch = PIDReturn[0];
    prev_err_pitch = PIDReturn[1];
    prev_int_pitch = PIDReturn[2];

    // Positional servo: base position + PID output, constrained to [0, 90]
    int pitch_servo_cmd = constrain((int)(desired_pitch + output_pitch), 0, 90);

    // --- YAW RATE CONTROL (Continuous Servo) ---
    // Map joystick_x [0, 180] to desired yaw rate [-max_rate, +max_rate] in rad/s
    float max_yaw_rate = 3.0; // Adjust this to limit max rotation speed
    // float desired_yaw_rate = ((desired_pitch - 90.0) / 90.0) * max_yaw_rate;
    
    // yaw angle
    float error_yaw = desired_yaw - desired_yaw;
    float desired_yaw_rate = Kp_yaw_angle * error_yaw;  // connect to rate of rotation
    desired_yaw_rate = constrain(desired_yaw_rate, -3.0, 3.0);

    // Error yaw
    float error_yaw_rate = desired_yaw_rate - gx_yaw;

    pid_pos_eqn(error_yaw_rate, Kp_yaw, Ki_yaw, Kd_yaw, prev_err_yaw, prev_int_yaw, dt);
    float output_yaw = PIDReturn[0];
    prev_err_yaw = PIDReturn[1];
    prev_int_yaw = PIDReturn[2];

    int yaw_servo_cmd = constrain(90 + (int)output_yaw, 0, 180);

    // Set servos
    servoPitch.write(pitch_servo_cmd); // pitch PID output
    servoYaw.write(yaw_servo_cmd); // yaw PID output

    // debug
    // Serial.print(roll+90, 0);
    // Serial.print(",");
    // Serial.print(pitch+90, 0);
    // Serial.print(",");
    // Serial.print(yaw, 0);
    // Serial.print(",");
    // Serial.print(desired_yaw, 2);
    // Serial.print(",");
    Serial.print(desired_pitch, 2);
    // Serial.print(",");
    // Serial.print(gx_yaw, 4);
    // Serial.print(",");
    // Serial.print(gy_roll, 4);
    // Serial.print(",");
    // Serial.println(gz_pitch, 4);
  }

  delay(50);
}