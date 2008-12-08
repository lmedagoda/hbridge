
#include "pid.h"
#include "stm32f10x_conf.h"

/**
 * This function set the kp value. Note, 
 * that kp is a fixed point with 2 values
 * behind the seperator
 */
void setKp(struct pid_data *data, s32 kp) {
  data->kp = kp;
}

/**
 * This function set the ki value. Note, 
 * that ki is a fixed point with 2 values
 * behind the seperator
 */
void setKi(struct pid_data *data, s32 ki) {
  data->ki = ki;
}

/**
 * This function set the kd value. Note, 
 * that kd is a fixed point with 2 values
 * behind the seperator.
 */
void setKd(struct pid_data *data, s32 kd) {
  data->kd = kd;
}

void setTargetValue(struct pid_data *data, s32 target_val) {
  data->target_val = target_val;
}


s32 pid(struct pid_data *data, s32 cur_val) {

  s32 result = data->last_command_val;
  s32 error = data->target_val - cur_val;

  if ( data->kp ) {
    result += ((error -data->last_error) * data->kp) / 100;
  }
  
  if ( data->ki ) {
    //(e+ e_old) * ki
    result += ((error + data->last_error) * data->ki) / 50;
  }

  if ( data->kd ) {
    result += ((error - (2 * data->last_error) + data->last2_error) * data->kd) / 100;
  }
  
  data->last2_error = data->last_error;
  data->last_error = error;
  
  if(result < data->min_command_val)
    result = data->min_command_val;
  
  if(result > data->max_command_val)
    result = data->max_command_val;
  
  data->last_command_val = result;
  
  return result;
}

void setMinMaxCommandVal(struct pid_data *data, s32 min, s32 max) {
  data->max_command_val = max;
  data->min_command_val = min;

  data->last_command_val = 0;
  data->target_val = 0;
  data->last_error = 0;
  data->last2_error = 0;
}

