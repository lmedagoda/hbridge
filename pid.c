
#include "pid.h"
#include "stm32f10x_conf.h"

/**
 * This function set the kp value. Note, 
 * that kp is a fixed point with 3 values
 * behind the , and thus must be >= 1000
 */
void setKp(struct pid_data *data, s32 kp) {
  assert_param(kp == 0 || kp >= 1000 || kp <= -1000);
  data->kp = kp;
}

/**
 * This function set the ki value. Note, 
 * that ki is a fixed point with 3 values
 * behind the , and thus must be >= 1000
 */
void setKi(struct pid_data *data, s32 ki) {
  assert_param(ki == 0 || ki >= 1000 || ki <= -1000);
  data->ki = ki;
}

/**
 * This function set the kd value. Note, 
 * that kd is a fixed point with 3 values
 * behind the , and thus must be >= 1000
 */
void setKd(struct pid_data *data, s32 kd) {
  assert_param(kd == 0 || kd >= 1000 || kd <= -1000);
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
  result += (data->error * data->kp) / 1000;

  data->integrated_error += data->error;
  
  result += (data->integrated_error + data->ki) / 1000;

  //TODO add diferential part
  
  return result;
}
