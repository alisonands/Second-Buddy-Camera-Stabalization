// adding joystick input

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <ESP32Servo.h>
#include "pid_pos.h"
#include <WiFi.h>
#include <esp_now.h>
#include "gimbal_target.h"

// ------- DEFINE I2C PINS --------
#define SDA_PIN 21
#define SCL_PIN 22
#define BNO_ADDR 0x4B

// --------- BNO080 GYRO ----------
BNO080 IMU;

// ------------ ESP-NOW -----------
struct ControlPacket
{
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

constexpr int PITCH_SERVO_MIN_US = 1000;
constexpr int PITCH_SERVO_MAX_US = 2000;

constexpr uint16_t IMU_REPORT_INTERVAL_MS = 20; // 50 Hz
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float DT_FALLBACK = 0.02f;
constexpr float DT_MIN = 0.001f;
constexpr float DT_MAX = 0.1f;
constexpr float PITCH_DEADBAND = 0.0f;
constexpr float YAW_DEADBAND_RAW = 0.02f;
constexpr float PITCH_OUTPUT_SLEW_DPS = 90.0f;
constexpr float YAW_OUTPUT_SLEW_DPS = 220.0f;
constexpr float PITCH_OUTPUT_LIMIT = 46.0f;
constexpr float YAW_OUTPUT_LIMIT = 8.0f;
constexpr float PITCH_ERROR_FILTER_ALPHA = 0.12f;
constexpr float PITCH_HOLD_BAND_DEG = 1.0f;         // for ~deg stabalization
constexpr float PITCH_HOLD_RATE_DPS = 8.0f;         // dont do anything if pitch rate is under 8 deg/s
constexpr float PITCH_HOLD_OUTPUT_BAND_DEG = 0.49f; // if requested servo move is under 0.49
// Set PITCH_USE_Y_AXIS to true if the serial x/y/z print shows pitch motion on y.
constexpr bool PITCH_USE_Y_AXIS = true;
constexpr float PITCH_ERROR_SIGN = 1.0f;
constexpr float PITCH_RATE_SIGN = -1.0f;
constexpr float PITCH_OUTPUT_SIGN = 1.0f;
constexpr float YAW_ERROR_SIGN = 1.0f;
constexpr float YAW_RATE_SIGN = 1.0f;
constexpr float DEG_TO_RAD_F = 0.01745329252f;

float pitch_center = 100.0;
float yaw_center = 94.0;

float pitch_range_down = 90;
float pitch_range_up = 45;

float desired_pitch = 0.0f;
float desired_yaw = 0.0f;

uint32_t last_sample_us = 0;
float pitch_output_state = 90.0f;
float yaw_output_state = 94.0f;
float filtered_pitch_error = 0.0f;

// Data structures
typedef struct
{
  float w, x, y, z;
} Quaternion;

typedef struct
{
  float kp, ki, kd;
  float integ;
  float integLimit;
} PID;
typedef struct
{
  float m[3][3];
} DCM;

typedef struct
{
  float yaw;
  float pitch;
  float roll;
} EulerDeg;

// axis alignment
// float pitch_error = err_x;
// float yaw_error   = err_z;

// Quaternions
void normalizeQuat(Quaternion *q)
{
  float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  q->w /= norm;
  q->x /= norm;
  q->y /= norm;
  q->z /= norm;
}

// inverse
Quaternion quatInverse(Quaternion q)
{
  Quaternion inv = {q.w, -q.x, -q.y, -q.z};
  return inv;
}

// multiply
Quaternion quatMultiply(Quaternion a, Quaternion b)
{
  Quaternion q;

  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return q;
}

// motors
void setPitchMotor(float cmd)
{
  // position motor
  // e.g. map to servo or position controller
}

void setYawMotor(float cmd)
{
  // continuous motor
  // e.g. PWM or velocity control
}

DCM quatToDCM(Quaternion q)
{
  normalizeQuat(&q);

  float q0 = q.w;
  float q1 = q.x;
  float q2 = q.y;
  float q3 = q.z;

  DCM dcm = {
      {
          {q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2.0f * (q1 * q2 + q3 * q0), 2.0f * (q1 * q3 - q2 * q0)},
          {2.0f * (q2 * q1 - q3 * q0), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2.0f * (q2 * q3 + q1 * q0)},
          {2.0f * (q3 * q1 + q2 * q0), 2.0f * (q3 * q2 - q1 * q0), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3},
      }};

  return dcm;
}

EulerDeg dcmToEulerDeg(DCM dcm)
{
  float yaw = atan2f(dcm.m[0][1], dcm.m[0][0]);
  float pitch = -asinf(constrain(dcm.m[0][2], -1.0f, 1.0f));
  float roll = atan2f(dcm.m[1][2], dcm.m[2][2]);

  EulerDeg euler = {
      yaw * RAD_TO_DEG_F,
      pitch * RAD_TO_DEG_F,
      roll * RAD_TO_DEG_F,
  };

  return euler;
}

// PID
float runPID(PID *pid, float error, float gyroRate, float dt)
{

  float p = pid->kp * error;

  // Integral
  pid->integ += pid->ki * error * dt;
  pid->integ = constrain(pid->integ, -pid->integLimit, pid->integLimit);

  // Derivative (gyro damping)
  float d = -pid->kd * gyroRate;

  return p + pid->integ + d;
}

// global variables
Quaternion q_current;
Quaternion q_home;
Quaternion q_target;

PID pid_pitch = {.kp = 2.0, .ki = 0.0, .kd = 0.001, .integ = 0, .integLimit = 8};
PID pid_yaw = {.kp = 35.0, .ki = 0.0, .kd = 0.001, .integ = 0, .integLimit = 10};

float gyro_x, gyro_y, gyro_z;

float computeSampleDt()
{
  uint32_t now = micros();
  if (last_sample_us == 0)
  {
    last_sample_us = now;
    return DT_FALLBACK;
  }

  float dt = (now - last_sample_us) * 1e-6f;
  last_sample_us = now;

  if (dt < DT_MIN || dt > DT_MAX)
  {
    return DT_FALLBACK;
  }

  return dt;
}

float lowPass(float state, float input, float alpha)
{
  return state + alpha * (input - state);
}

float slewLimit(float current, float target, float max_rate_deg_per_sec, float dt)
{
  float max_step = max_rate_deg_per_sec * dt;
  return current + constrain(target - current, -max_step, max_step);
}

int pitchDegreesToMicros(float pitch_deg)
{
  float clamped_pitch = constrain(pitch_deg, 0.0f, 180.0f);
  float span_us = float(PITCH_SERVO_MAX_US - PITCH_SERVO_MIN_US);
  return int(lroundf(PITCH_SERVO_MIN_US + (clamped_pitch / 180.0f) * span_us));
}

void writePitchPwm(float pitch_deg)
{
  servoPitch.writeMicroseconds(pitchDegreesToMicros(pitch_deg));
}

Quaternion quatFromAxisAngle(float ax, float ay, float az, float degrees)
{
  float half_angle = 0.5f * degrees * DEG_TO_RAD_F;
  float s = sinf(half_angle);
  Quaternion q = {cosf(half_angle), ax * s, ay * s, az * s};
  normalizeQuat(&q);
  return q;
}

void updateTargetFromDegrees()
{
  // Serial.print(desired_pitch);
  // Serial.print(", ");
  // Serial.println(desired_yaw);
  Quaternion q_pitch = PITCH_USE_Y_AXIS
                           ? quatFromAxisAngle(0.0f, 1.0f, 0.0f, desired_pitch)
                           : quatFromAxisAngle(1.0f, 0.0f, 0.0f, desired_pitch);
  Quaternion q_yaw = quatFromAxisAngle(0.0f, 0.0f, 1.0f, desired_yaw);
  Quaternion q_offset = quatMultiply(q_yaw, q_pitch);
  q_target = quatMultiply(q_offset, q_home);
  normalizeQuat(&q_target);
}

void resetPitchControlState()
{
  filtered_pitch_error = 0.0f;
  pid_pitch.integ = 0.0f;
}

void setDesiredAnglesDeg(float pitch_deg, float yaw_deg)
{
  desired_pitch = pitch_deg;
  desired_yaw = yaw_deg;
  updateTargetFromDegrees();
  resetPitchControlState();
}

void offsetDesiredAnglesDeg(float pitch_delta_deg, float yaw_delta_deg)
{
  setDesiredAnglesDeg(desired_pitch + pitch_delta_deg, desired_yaw + yaw_delta_deg);
}

GimbalTargetDegrees getDesiredAnglesDeg()
{
  return {desired_pitch, desired_yaw};
}

void zeroDesiredAnglesAtCurrentPose()
{
  q_home = q_current;
}

void handleSerialTargetInput()
{
  //if (!Serial.available())
  //{
    //return;
  //}

  String line = Serial.readStringUntil('\n');
  line.trim();
  // if (line.length() == 0)
  // {
  //   return;
  // }

  if (line.equalsIgnoreCase("zero"))
  {
    zeroDesiredAnglesAtCurrentPose();
    Serial.println("Target reset to 0,0 at current orientation");
    return;
  }

  if (pkt.mode == 0)
  {
    // open loop control; commands range from -1 to 1
    desired_yaw += float(pkt.x) * 0.05f;
    desired_pitch += float(pkt.y) * 0.05f;
  }
  else
  {
    // openCV tracking mode; commands range from -1000 to 1000
    desired_yaw += float(pkt.x) * (0.05f / 1000.0f);
    desired_pitch += float(pkt.y) * (0.05f / 1000.0f);
  }

  // edge detection for laser toggle
  if (pkt.btn == 0 && btn_prev == 1)
  {
    // falling edge (released)
    laser_state = !laser_state;
    // digitalWrite(laserPin, laser_state); // need to define our laser pin
  }

  btn_prev = pkt.btn;

  float pitch_deg = desired_pitch;

  float yaw_deg = desired_yaw;

  Serial.print(desired_pitch);
  Serial.print(", ");
  Serial.println(desired_yaw);
  int parsed = sscanf(line.c_str(), "%f %f", &pitch_deg, &yaw_deg);
  if (parsed == 2)
  {
    setDesiredAnglesDeg(pitch_deg, yaw_deg);
    Serial.print("Target degrees set to ");
    Serial.print(desired_pitch, 2);
    Serial.print(",");
    Serial.println(desired_yaw, 2);
    return;
  }

  //Serial.println("Send: <pitch_deg> <yaw_deg>  or  zero");
}

// callback on receiving packet
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  memcpy(&pkt, incomingData, sizeof(pkt)); // update packet information
}

// setup
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

  writePitchPwm(110.0f);

  if (!IMU.begin(BNO_ADDR, Wire))
  {
    Serial.println("BNO080 not found 😭 BAD BAD BAD");
    while (1)
      ;
  }

  Serial.println("Connected, initializing...");
  delay(500);

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

  IMU.enableRotationVector(IMU_REPORT_INTERVAL_MS);
  delay(200);

  IMU.enableGyro(IMU_REPORT_INTERVAL_MS);

  Serial.println("IMU ready");

  // wait for a few good samples
  for (int i = 0; i < 10; i++)
  {
    while (!IMU.dataAvailable())
      ;

    q_current.w = IMU.getQuatReal();
    q_current.x = IMU.getQuatI();
    q_current.y = IMU.getQuatJ();
    q_current.z = IMU.getQuatK();
  }

  normalizeQuat(&q_current);
  q_home = q_current;
  updateTargetFromDegrees();
  last_sample_us = micros();
  pitch_output_state = pitch_center;
  yaw_output_state = yaw_center;
  writePitchPwm(pitch_output_state);
  servoYaw.write(yaw_output_state);
  Serial.println("Send target as: <pitch_deg> <yaw_deg>");
  Serial.println("Example: 10 -15");
  Serial.println("Send 'zero' to redefine the current orientation as 0,0");
}

void loop()
{
  // handleSerialTargetInput();

  if (IMU.hasReset())
  {
    Serial.println("IMU reset!");
    delay(100);
  }
  if (IMU.dataAvailable())
  {
    q_current.w = IMU.getQuatReal();
    q_current.x = IMU.getQuatI();
    q_current.y = IMU.getQuatJ();
    q_current.z = IMU.getQuatK();

    normalizeQuat(&q_current);

    gyro_x = IMU.getGyroX();
    gyro_y = IMU.getGyroY();
    gyro_z = IMU.getGyroZ();
    float dt = computeSampleDt();

    Quaternion q_inv = quatInverse(q_current);
    Quaternion q_err = quatMultiply(q_target, q_inv);
    // Serial.print(q_current.y);
    // Serial.print(", ");
    // Serial.print(q_current.z);
    // Serial.print(", ");
    // Serial.print(q_target.y);
    // Serial.print(", ");
    // Serial.println(q_target.z);

    if (q_err.w < 0)
    {
      q_err.w *= -1;
      q_err.x *= -1;
      q_err.y *= -1;
      q_err.z *= -1;
    }

    DCM err_dcm = quatToDCM(q_err);
    EulerDeg err_euler = dcmToEulerDeg(err_dcm);

    float err_x_raw = 2.0f * q_err.x;
    float err_y_raw = 2.0f * q_err.y;
    float err_z_raw = 2.0f * q_err.z;

    Serial.print(err_x_raw);
    Serial.print(", ");
    Serial.print(err_y_raw);
    Serial.print(", ");
    Serial.println(err_z_raw);

    float err_x = RAD_TO_DEG_F * err_x_raw;
    float err_y = RAD_TO_DEG_F * err_y_raw;
    float err_z = RAD_TO_DEG_F * err_z_raw;

    float raw_pitch_error = PITCH_USE_Y_AXIS ? err_y : err_x;
    float raw_pitch_rate = RAD_TO_DEG_F * (PITCH_USE_Y_AXIS ? gyro_y : gyro_x);
    float pitch_error = PITCH_ERROR_SIGN * raw_pitch_error;
    float pitch_rate = PITCH_RATE_SIGN * raw_pitch_rate;
    float yaw_error = YAW_ERROR_SIGN * err_z_raw;
    float yaw_rate = YAW_RATE_SIGN * gyro_z;

    // ------------------------------------
    // -------BASE STATION CONTROLS--------
    // ------------------------------------
    // String line = Serial.readStringUntil('\n');
    // line.trim();

    if (pkt.mode == 0)
    {
      // open loop control; commands range from -1 to 1
      desired_yaw += float(pkt.x) * 0.05f;
      desired_pitch += float(pkt.y) * 0.05f;
    }
    else
    {
      // openCV tracking mode; commands range from -1000 to 1000
      desired_yaw += float(pkt.x) * (0.05f / 1000.0f);
      desired_pitch += float(pkt.y) * (0.05f / 1000.0f);
    }

    // // edge detection for laser toggle
    // if (pkt.btn == 0 && btn_prev == 1)
    // {
    //   // falling edge (released)
    //   laser_state = !laser_state;
    //   // digitalWrite(laserPin, laser_state); // need to define our laser pin
    // }

    btn_prev = pkt.btn;
    updateTargetFromDegrees();

    // float pitch_deg = desired_pitch;

    // float yaw_deg = desired_yaw;

    // Serial.print(pitch_deg);
    // Serial.print(", ");
    // Serial.println(yaw_deg);
    // int parsed = sscanf(line.c_str(), "%f %f", &pitch_deg, &yaw_deg);
    // if (parsed == 2)
    // {
    //   setDesiredAnglesDeg(pitch_deg, yaw_deg);
    //   Serial.print("Target degrees set to ");
    //   Serial.print(desired_pitch, 2);
    //   Serial.print(",");
    //   Serial.println(desired_yaw, 2);
    //   return;
    // }

    //Serial.println("Send: <pitch_deg> <yaw_deg>  or  zero");

    if (abs(pitch_error) < PITCH_DEADBAND)
    {
      pitch_error = 0.0f;
    }
    if (abs(yaw_error) < YAW_DEADBAND_RAW)
    {
      yaw_error = 0.0f;
    }

    filtered_pitch_error = lowPass(filtered_pitch_error, pitch_error, PITCH_ERROR_FILTER_ALPHA);

    float pitch_cmd = runPID(&pid_pitch, filtered_pitch_error, pitch_rate, dt);
    float yaw_cmd = runPID(&pid_yaw, yaw_error, yaw_rate, dt);

    pitch_cmd = constrain(pitch_cmd, -PITCH_OUTPUT_LIMIT, PITCH_OUTPUT_LIMIT);
    yaw_cmd = constrain(yaw_cmd, -YAW_OUTPUT_LIMIT, YAW_OUTPUT_LIMIT);

    float pitch_min_output = constrain(pitch_center - pitch_range_down, 0.0f, 180.0f);
    float pitch_max_output = constrain(pitch_center + pitch_range_up, 0.0f, 180.0f);
    float pitch_output_target = constrain(pitch_center + PITCH_OUTPUT_SIGN * pitch_cmd, pitch_min_output, pitch_max_output);
    float yaw_output_target = constrain(yaw_center - yaw_cmd, 0.0f, 180.0f);

    bool pitch_in_hold_zone =
        abs(pitch_error) < PITCH_HOLD_BAND_DEG &&
        abs(pitch_rate) < PITCH_HOLD_RATE_DPS &&
        abs(pitch_output_target - pitch_output_state) < PITCH_HOLD_OUTPUT_BAND_DEG;
    if (pitch_in_hold_zone)
    {
      pitch_output_target = pitch_output_state;
      pid_pitch.integ *= 0.98f;
    }

    pitch_output_state = slewLimit(pitch_output_state, pitch_output_target, PITCH_OUTPUT_SLEW_DPS, dt);
    yaw_output_state = slewLimit(yaw_output_state, yaw_output_target, YAW_OUTPUT_SLEW_DPS, dt);

    // writePitchPwm(pitch_output_state);
    servoPitch.write(pitch_output_state);
    servoYaw.write(yaw_output_state);

    // Serial.print(dt, 4);
    // Serial.print(pid_pitch);
    // Serial.print(",");
    // Serial.print(pid_yaw);
    int pitch_us = pitchDegreesToMicros(pitch_output_state);
    // Serial.print(pkt.x);
    // Serial.print(",");
    // Serial.print(pitch_output_state);
    // Serial.print(", ");
    // Serial.print(pkt.y);
    // Serial.print(", ");
    // Serial.print(pkt.mode);
    // Serial.print(", ");
    // Serial.print(desired_pitch);
    // Serial.print(", ");
    // Serial.println(desired_yaw);
    // Serial.printf("x:%f y:%f z:%f\n", err_x, err_y, err_z);
  }
}
