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
  uint8_t stab;
  uint8_t mode;
};
ControlPacket pkt;

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
constexpr float PITCH_ERROR_FILTER_ALPHA = 0.12f;
// Set PITCH_USE_Y_AXIS to true if the serial x/y/z print shows pitch motion on y.
constexpr float PITCH_ERROR_SIGN = 1.0f;
constexpr float PITCH_RATE_SIGN = -1.0f;
constexpr float PITCH_OUTPUT_SIGN = 1.0f;
constexpr float YAW_ERROR_SIGN = 1.0f;
constexpr float YAW_RATE_SIGN = 1.0f;
constexpr float DEG_TO_RAD_F = 0.01745329252f;

// manual calibration
float pitch_center = 103.0f;
float yaw_center = 96.0f;

// motor states
float pitchMotorAngle = pitch_center;
float yawMotorAngle = yaw_center;

// desired angles
float desired_roll = 0.0f;
float desired_pitch = 0.0f;
float desired_yaw = 0.0f;

// error processing
uint32_t last_sample_us = 0;
float filtered_pitch_error = 0.0f;

// stabilization toggle
bool stab_mode = true;
uint8_t stab_prev = 0;

// Data structures
typedef struct
{
  float w, x, y, z;
} Quaternion;

struct Vector3 {
  float x, y, z;
};

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

PID pid_pitch = {.kp = 0.10, .ki = 0.0, .kd = 0.0, .integ = 0, .integLimit = 8};
PID pid_yaw = {.kp = 0.90, .ki = 0.0, .kd = 0.0, .integ = 0, .integLimit = 10}; // not true PID, motor turns left if input < 94 and right if > 98

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


// callback on receiving packet
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  memcpy(&pkt, incomingData, sizeof(pkt)); // update packet information
}

// ------------- setup -------------
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

  // IMU setup
  IMU.enableRotationVector(IMU_REPORT_INTERVAL_MS);
  delay(200);
  IMU.enableGyro(IMU_REPORT_INTERVAL_MS);
  Serial.println("IMU ready");

  for (int i = 0; i < 10; i++)
  {
    while (!IMU.dataAvailable()) // wait for a few good samples
      ;

    q_current.w = IMU.getQuatReal();
    q_current.x = IMU.getQuatI();
    q_current.y = IMU.getQuatJ();
    q_current.z = IMU.getQuatK();
  }
  normalizeQuat(&q_current);

  // get current attitude from quaternions
  DCM setup_att = quatToDCM(q_current);
  EulerDeg setup_att_Euler = dcmToEulerDeg(setup_att);

  desired_yaw = setup_att_Euler.yaw; // zero on the current yaw reading

  servoPitch.write(pitch_center);
  last_sample_us = micros();
}

// ----------- main loop -----------
void loop()
{
  // edge detection for stabilization mode
  if (pkt.stab == 1 && stab_prev == 0)
  {
    stab_mode = !stab_mode;
  }
  stab_prev = pkt.stab;

  // IMU health status
  if (IMU.hasReset())
  {
    Serial.println("IMU reset!");
    delay(100);
  }

  // measure data
  if (IMU.dataAvailable())
  {
    q_current.w = IMU.getQuatReal();
    q_current.x = IMU.getQuatI();
    q_current.y = IMU.getQuatJ();
    q_current.z = IMU.getQuatK();

    normalizeQuat(&q_current);

    // get current attitude from quaternions
    DCM cur_att = quatToDCM(q_current);
    EulerDeg cur_att_Euler = dcmToEulerDeg(cur_att);

    gyro_x = IMU.getGyroX();
    gyro_y = IMU.getGyroY();
    gyro_z = IMU.getGyroZ();

    //stabilization loop
    if (stab_mode) 
    {
      // handle yaw wraparound at 180 degrees
      float err_att_yaw = desired_yaw - cur_att_Euler.yaw;
      if (err_att_yaw > 180.0f)  err_att_yaw -= 360.0f;
      if (err_att_yaw < -180.0f) err_att_yaw += 360.0f;

      // error postprocessing
      Vector3 err_att = {
        desired_roll - cur_att_Euler.roll,
        desired_pitch - cur_att_Euler.pitch,
        err_att_yaw
      };
      
      float pitch_error = PITCH_ERROR_SIGN * err_att.y;
      float pitch_rate = PITCH_RATE_SIGN * gyro_y;
      float yaw_error = YAW_ERROR_SIGN * err_att.z;
      float yaw_rate = YAW_RATE_SIGN * gyro_z;

      filtered_pitch_error = lowPass(filtered_pitch_error, pitch_error, PITCH_ERROR_FILTER_ALPHA);

      // base station controls
      if (pkt.mode == 0)
      {
        // open loop control; command value ranges from -1 to 1
        desired_yaw += float(-pkt.x) * 0.60f;
        desired_pitch += float(-pkt.y) * 0.40f;
      }
      else
      {
        // openCV tracking mode; command value ranges from -1000 to 1000
        desired_yaw += float(-pkt.x) * (0.75f / 1000.0f);
        desired_pitch += float(pkt.y) * (0.50f / 1000.0f);
      }

      // PID feedback
      float dt = computeSampleDt();
      float pitch_cmd = runPID(&pid_pitch, filtered_pitch_error, pitch_rate, dt);
      float yaw_cmd = runPID(&pid_yaw, yaw_error, yaw_rate, dt);
      
      float yaw_output_state = yaw_center - yaw_cmd;
      pitchMotorAngle = constrain(pitchMotorAngle + pitch_cmd, pitch_center - 60.0f, pitch_center + 40.0f); // increment from pitch center by pitch_cmd every loop, 60 deg up & 40deg down limit

      // writePitchPwm(pitch_output_state);
      servoPitch.write(pitchMotorAngle);
      servoYaw.write(yaw_output_state);
    }
    // disable stabilization
    else
    {
      // unstabilized loop, but set desired pitch/yaw angles to "remember" when restabilizing
      desired_pitch = cur_att_Euler.pitch;
      desired_yaw = cur_att_Euler.yaw;
    }
  }
}
