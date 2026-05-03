#pragma once

struct GimbalTargetDegrees
{
  float pitch;
  float yaw;
};

void setDesiredAnglesDeg(float pitch_deg, float yaw_deg);
void offsetDesiredAnglesDeg(float pitch_delta_deg, float yaw_delta_deg);
GimbalTargetDegrees getDesiredAnglesDeg();
void zeroDesiredAnglesAtCurrentPose();
