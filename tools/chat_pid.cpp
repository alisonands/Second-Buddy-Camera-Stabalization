float Kp = 2.5;
float Ki = 0.0;
float Kd = 0.8;

float errorPitch, errorYaw;
float prevErrorPitch = 0, prevErrorYaw = 0;

float integralPitch = 0, integralYaw = 0;

errorPitch = 0 - pitch;   // target is level

integralPitch += errorPitch;
float derivativePitch = errorPitch - prevErrorPitch;

float outputPitch = Kp*errorPitch + Ki*integralPitch + Kd*derivativePitch;

prevErrorPitch = errorPitch;

servoPitch.write(constrain(90 + outputPitch, 0, 180));



errorYaw = 0 - yaw;

integralYaw += errorYaw;
float derivativeYaw = errorYaw - prevErrorYaw;

float outputYaw = Kp*errorYaw + Ki*integralYaw + Kd*derivativeYaw;

prevErrorYaw = errorYaw;

servoYaw.write(constrain(90 + outputYaw, 0, 180));

if (myIMU.dataAvailable())
{
  roll = myIMU.getRoll() * 180.0f / PI;
  pitch = myIMU.getPitch() * 180.0f / PI;
  yaw = myIMU.getYaw() * 180.0f / PI;

  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);

  // PID CONTROL HERE
}