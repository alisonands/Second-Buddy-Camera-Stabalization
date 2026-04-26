#ifndef PID_POS_H
#define PID_POS_H

extern float prev_err_roll, prev_err_pitch, prev_err_yaw;
extern float prev_int_roll, prev_int_pitch, prev_int_yaw;
extern float PIDReturn[];

void pid_pos_eqn(float Error, float P, float I, float D, float PrevError, float Previnteg, float dt);
void reset_pid(void);

#endif