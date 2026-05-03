#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <ESP32Servo.h>

// ------ DEFINE I2C PINS -------
#define SDA_PIN 21
#define SCL_PIN 22
#define BNO_ADDR 0x4B

// --------- BNO080 GYRO ----------
BNO080 IMU;

// Servo
Servo servoPitch;
Servo servoYaw;
int servoPitchPin = 19;
int servoYawPin = 18;

constexpr uint16_t IMU_REPORT_INTERVAL_MS = 20; // 50 Hz
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float DT_FALLBACK = 0.02f;
constexpr float DT_MIN = 0.001f;
constexpr float DT_MAX = 0.1f;
constexpr float PITCH_DEADBAND = 0.20f;
constexpr float YAW_DEADBAND_RAW = 0.02f;
constexpr float PITCH_OUTPUT_SLEW_DPS = 90.0f;
constexpr float YAW_OUTPUT_SLEW_DPS = 220.0f;
constexpr float PITCH_OUTPUT_LIMIT = 46.0f;
constexpr float YAW_OUTPUT_LIMIT = 8.0f;
constexpr float PITCH_ERROR_FILTER_ALPHA = 0.12f;
constexpr float PITCH_HOLD_BAND_DEG = 1.0f; // for ~deg stabalization
constexpr float PITCH_HOLD_RATE_DPS = 8.0f; // dont do anything if pitch rate is under 8 deg/s
constexpr float PITCH_HOLD_OUTPUT_BAND_DEG = 0.49f; // if requested servo move is under 0.49
// Set PITCH_USE_Y_AXIS to true if the serial x/y/z print shows pitch motion on y.
constexpr bool PITCH_USE_Y_AXIS = true;
constexpr float PITCH_ERROR_SIGN = 1.0f;
constexpr float PITCH_RATE_SIGN = -1.0f;
constexpr float PITCH_OUTPUT_SIGN = 1.0f;
constexpr float YAW_ERROR_SIGN = 1.0f;
constexpr float YAW_RATE_SIGN = 1.0f;

float pitch_center = 100.0;
float yaw_center = 94.0;

float pitch_range_down = 90;
float pitch_range_up = 45;

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
Quaternion q_target;

PID pid_pitch = {.kp = 1.60, .ki = 0.0, .kd = 0.0, .integ = 0, .integLimit = 8};
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

  servoPitch.write(110);

  if (!IMU.begin(BNO_ADDR, Wire))
  {
    Serial.println("BNO080 not found 😭 BAD BAD BAD");
    while (1)
      ;
  }

  Serial.println("Connected, initializing...");

  delay(500);

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
  q_target = q_current;
  last_sample_us = micros();
  pitch_output_state = pitch_center;
  yaw_output_state = yaw_center;
  servoPitch.write(pitch_output_state);
  servoYaw.write(yaw_output_state);
}

void loop()
{
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
    if (q_err.w < 0)
    {
      q_err.w *= -1;
      q_err.x *= -1;
      q_err.y *= -1;
      q_err.z *= -1;
    }

    float err_x_raw = 2.0f * q_err.x;
    float err_y_raw = 2.0f * q_err.y;
    float err_z_raw = 2.0f * q_err.z;

    float err_x = RAD_TO_DEG_F * err_x_raw;
    float err_y = RAD_TO_DEG_F * err_y_raw;
    float err_z = RAD_TO_DEG_F * err_z_raw;

    float raw_pitch_error = PITCH_USE_Y_AXIS ? err_y : err_x;
    float raw_pitch_rate = RAD_TO_DEG_F * (PITCH_USE_Y_AXIS ? gyro_y : gyro_x);
    float pitch_error = PITCH_ERROR_SIGN * raw_pitch_error;
    float pitch_rate = PITCH_RATE_SIGN * raw_pitch_rate;
    float yaw_error = YAW_ERROR_SIGN * err_z_raw;
    float yaw_rate = YAW_RATE_SIGN * gyro_z;
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

    servoPitch.write(pitch_output_state);
    servoYaw.write(yaw_output_state);

    // Serial.print(dt, 4);
    Serial.print(pid_pitch);
    Serial.print(",");
    Serial.pring(pid_yaw);
    Serial.print(",");
    Serial.print(pitch_output_state);
    Serial.print(",");
    Serial.println(yaw_output_state);
    // Serial.printf("x:%f y:%f z:%f\n", err_x, err_y, err_z);
  }
}
