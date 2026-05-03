#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <ESP32Servo.h>
#include "pid_pos.h"
#include <WiFi.h>
#include <esp_now.h>

// ------- DEFINE I2C PINS --------
#define SDA_PIN 21
#define SCL_PIN 22
#define BNO_ADDR 0x4B

// --------- BNO080 GYRO ----------
BNO080 myIMU;

// ------------ ESP-NOW -----------
struct ControlPacket {
  int16_t x;
  int16_t y;
  uint8_t btn;
  uint8_t mode;
};
ControlPacket pkt;
uint8_t btn_prev = 0;
bool laser_state = false;

// Servo
Servo servoPitch;
Servo servoYaw;
int servoPitchPin = 19; 
int servoYawPin = 18;

// desired values (to be replaced w joystick inputs in loop later)
float desired_yaw = 0.0;
float desired_pitch = 0.0;

// PID VALUES
float Kp_pitch = 1.0, Ki_pitch = 0.0, Kd_pitch = 0.001;             // position control for pitch
float Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.1;                   // rate control for yaw
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

// callback on receiving packet
void onDataRecv(const esp_now_recv_info * info, const uint8_t * incomingData, int len)
{
  memcpy(&pkt, incomingData, sizeof(pkt)); // update packet information
}

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
  
  // initialize esp-now protocol
  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW protocol");
    return;
  }
  else
  {
    Serial.println("Successfully initialized ESP-NOW protocol");
  }

  // set callback function when receiving data
  esp_now_register_recv_cb(onDataRecv);
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
      float gx_roll = myIMU.getGyroX();
      float gy_pitch = myIMU.getGyroY();
      float gz_yaw = myIMU.getGyroZ();

      // Quaternions
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();
      float qw = myIMU.getQuatReal();


      // low pass filter
      // static float pitch_f = 0;
      // static float yaw_f = 0;

      // float alpha = 0.9;

      // pitch = alpha * pitch_f + (1 - alpha) * pitch;
      // yaw   = alpha * yaw_f   + (1 - alpha) * yaw;

      // ------------------------------------
      // -------BASE STATION CONTROLS--------
      // ------------------------------------
      if (pkt.mode == 0) 
      {
        // open loop control; commands range from -1 to 1
        desired_yaw   += float(pkt.x) * 0.1f;
        desired_pitch += float(pkt.y) * 0.1f;
      }
      else
      {
        // openCV tracking mode; commands range from -1000 to 1000
        desired_yaw   += float(pkt.x) * (0.1f / 1000.0f);
        desired_pitch += float(pkt.y) * (0.1f / 1000.0f);
      }

      // edge detection for laser toggle
      if (pkt.btn == 0 && btn_prev == 1)
      {
        // falling edge (released)
        laser_state = !laser_state;
        // digitalWrite(laserPin, laser_state); // need to define our laser pin
      }

      btn_prev = pkt.btn;

      // ------------------------------------
      // ------------PID CONTROL-------------
      // ------------------------------------

      // PITCH POSITION CONTROL
      // float error_pitch = desired_pitch - pitch;
      float error_pitch = desired_pitch - pitch;

      if (abs(error_pitch) < 2.0) {   // 1 degree deadband
        error_pitch = 0;
      }
      pid_pos_eqn(error_pitch, Kp_pitch, Ki_pitch, Kd_pitch, gy_pitch, prev_int_pitch, dt);
      float output_pitch = PIDReturn[0];
      prev_err_pitch = PIDReturn[1];
      prev_int_pitch = PIDReturn[2];

      // Positional servo: base neutral position (90) - PID output (reversed)
      int pitch_servo_cmd = (int)output_pitch + 90;
      // int pitch_servo_cmd = 90 + output_pitch;

      // YAW RATE CONTROL (with outer cascading position control)
      float max_yaw_rate = 1.0; // limit max rotation speed

      // Convert position error from degrees to radians for consistent units
      float error_yaw_rad = (desired_yaw - yaw) * PI / 180.0;
      // The outer P-controller (Kp_yaw_angle) converts position error (rad) to a desired rate (rad/s)
      float desired_yaw_rate = Kp_yaw_angle * error_yaw_rad;
      desired_yaw_rate = constrain(desired_yaw_rate, -3.0, 3.0);

      // The inner PID controller corrects for the difference between desired rate and actual rate (gz_yaw)
      float error_yaw_rate = desired_yaw_rate - gz_yaw;
      pid_pos_eqn(error_yaw_rate, Kp_yaw, Ki_yaw, Kd_yaw, gz_yaw, prev_int_yaw, dt);
      float output_yaw = PIDReturn[0];
      prev_err_yaw = PIDReturn[1];
      prev_int_yaw = PIDReturn[2];

      // Scale PID output to overcome servo deadband. The '20.0' is a tuning factor you can adjust.
      const float yaw_output_scaling = 5.0;
      int yaw_servo_cmd = constrain(94 - lroundf(output_yaw * yaw_output_scaling), 0, 180);

      // Set servos
      servoPitch.write(pitch_servo_cmd); // pitch PID output
      servoYaw.write(yaw_servo_cmd);     // yaw PID output

      Serial.print(pitch);
      Serial.print(",");
      Serial.print(pitch_servo_cmd);
      Serial.print(",");
      Serial.print(output_pitch);
      Serial.print(",");
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(output_yaw);
      Serial.print(",");
      Serial.println(yaw_servo_cmd);
      // Serial.print("DATA: ");
      // Serial.println(myIMU.dataAvailable());
    }
  }