#ifndef __PID_H
#define __PID_H

#include "stm32f10x_type.h"

struct pid_data {
  s32 error;
  s32 integrated_error;
  s32 kp;
  s32 ki;
  s32 kd;
  s32 target_val;
  s32 last_target_val;
};

/**
 * This function set the kp value. Note, 
 * that kp is a fixed point with 3 values
 * behind the , and thus must be >= 1000
 */
void setKp(struct pid_data *data, s32 kp);

/**
 * This function set the ki value. Note, 
 * that ki is a fixed point with 3 values
 * behind the , and thus must be >= 1000
 */
void setKi(struct pid_data *data, s32 ki);

/**
 * This function set the kd value. Note, 
 * that kd is a fixed point with 3 values
 * behind the , and thus must be >= 1000
 */
void setKd(struct pid_data *data, s32 kd);

void setTargetValue(struct pid_data *data, s32 target_val);

s32 pid(struct pid_data *data, s32 cur_val);




#endif
