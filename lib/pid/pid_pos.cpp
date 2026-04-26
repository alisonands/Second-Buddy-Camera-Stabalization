# include "pid_pos.h"

float PIDReturn[] = {0, 0, 0};


void pid_pos_eqn(float Error, float P , float I, float D, float PrevError, float Previnteg, float dt) {
  float prop=P*Error;
  float integ=Previnteg+I*(Error+PrevError)*dt/2;
  // constraints. doesnt output more than 90. 
  if (integ > 90) integ=90;
  else if (integ <-90) integ=-90;

  float deriv=D*(Error-PrevError)/dt;
  float PIDOutput= prop+integ+deriv;
  if (PIDOutput>90) PIDOutput=90;
  else if (PIDOutput < -90) PIDOutput=-90;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=integ;
}
void reset_pid(void) {
  prev_err_roll=0; prev_err_pitch=0; prev_err_yaw=0;
  prev_int_roll=0; prev_int_pitch=0; prev_int_yaw=0;
}