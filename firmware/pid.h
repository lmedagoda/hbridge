#ifndef __PID_H
#define __PID_H

#include "stm32f10x_type.h"

struct pid_data {
  s32 kp;
  s32 ki;
  s32 kd;
  s32 target_val;
  s32 last_error;
  s32 max_command_val;
  s32 min_command_val;
  s32 error_sum;
};


/**
 * This function sets the minimum and maximum return 
 * value of the PID function. The return value of PID()
 * it clamped automatically clamped
 */
void setMinMaxCommandVal(struct pid_data *data, s32 min, s32 max);

/**
 * This function set the kp value. Note, 
 * that kp is a fixed point with 2 values
 * behind the , 
 */
void setKp(struct pid_data *data, s32 kp);

/**
 * This function set the ki value. Note, 
 * that ki is a fixed point with 2 values
 * behind the , 
 */
void setKi(struct pid_data *data, s32 ki);

/**
 * This function set the kd value. Note, 
 * that kd is a fixed point with 2 values
 * behind the , 
 */
void setKd(struct pid_data *data, s32 kd);

void setTargetValue(struct pid_data *data, s32 target_val);

s32 pid(struct pid_data *data, s32 cur_val);




#endif
