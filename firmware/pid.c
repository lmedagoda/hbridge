
#include "pid.h"
#include "stm32f10x_conf.h"
#include "protocol.h"
#include "can.h"

static s16 pPart = 0;
static s16 iPart = 0;
static s16 dPart = 0;

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

void getInternalPIDValues(s16 *pPart1, s16 *iPart1, s16 *dPart1) {
  *pPart1 = pPart;
  *iPart1 = iPart;
  *dPart1 = dPart;
}

s32 pid(struct pid_data *data, s32 cur_val) {

  pPart = 0;
  iPart = 0;
  dPart = 0;

  s32 result = 0;
  s32 error = data->target_val - cur_val;
  
  if ( data->kp ) {
    result += ((error) * data->kp) / 100;
    pPart = ((error) * data->kp) / 100;
  }
  
  if ( data->ki ) {

    data->error_sum += (error + data->last_error) / 2;
    
    if((data->error_sum * data->ki) / 100 > data->max_command_val) {
      data->error_sum = (data->max_command_val * 100) / data->ki;
    }
    
    if((data->error_sum * data->ki) / 100 < data->min_command_val) {
      data->error_sum = (data->min_command_val * 100) / data->ki;
    }

    //(e_sum)  * ki
    result += ((data->error_sum) * data->ki) / 100;

    iPart = ((data->error_sum) * data->ki) / 100;
  }

  if ( data->kd ) {
    result += ((error - data->last_error) * data->kd) / 100;
    dPart = ((error - data->last_error) * data->kd) / 100;
  }
  
  data->last_error = error;
  
  if(result < data->min_command_val)
    result = data->min_command_val;
  
  if(result > data->max_command_val)
    result = data->max_command_val;  
  
  return result;
}

void setMinMaxCommandVal(struct pid_data *data, s32 min, s32 max) {
  data->max_command_val = max;
  data->min_command_val = min;

  data->target_val = 0;
  data->last_error = 0;
  data->error_sum = 0;
}

