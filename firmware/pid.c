
#include "pid.h"
#include "stm32f10x_conf.h"

/**
 * This function set the kp value. Note, 
 * that kp is a fixed point with 3 values
 * behind the seperator
 */
void setKp(struct pid_data *data, s32 kp) {
  data->kp = kp;
}

/**
 * This function set the ki value. Note, 
 * that ki is a fixed point with 3 values
 * behind the seperator
 */
void setKi(struct pid_data *data, s32 ki) {
  data->ki = ki;
}

/**
 * This function set the kd value. Note, 
 * that kd is a fixed point with 3 values
 * behind the seperator.
 */
void setKd(struct pid_data *data, s32 kd) {
  data->kd = kd;
}

void setTargetValue(struct pid_data *data, s32 target_val) {
  data->target_val = target_val;
  if(data->last_target_val != data->target_val) {
    data->last_target_val = data->target_val;
    data->integrated_error = 0;
  }
  
}


s32 pid(struct pid_data *data, s32 cur_val) {

  s32 result = 0;
  data->error = data->target_val - cur_val;

  if ( data->kp ) {
    result += (data->error * data->kp) / 1000;
  }
  data->integrated_error += data->error;
  
  if ( data->ki ) {
    result += (data->integrated_error + data->ki) / 1000;
  }

  if ( data->kd ) {
    // (error-last_error) / 10ms * kd
    result += ((data->error - data->last_error) * data->kd)/10;
  }
  data->last_error = data->error; 
  
  return result;
}
