#include "pid.h"
#include "packets.h"

static int16_t pPart = 0;
static int16_t iPart = 0;
static int16_t dPart = 0;

/**
 * This function set the kp value. Note, 
 * that kp is a fixed point with 2 values
 * behind the seperator
 */
void setKp(struct pid_data *data, int32_t kp) {
  data->kp = kp;
}

/**
 * This function set the ki value. Note, 
 * that ki is a fixed point with 2 values
 * behind the seperator
 */
void setKi(struct pid_data *data, int32_t ki) {
  data->ki = ki;
}

/**
 * This function set the kd value. Note, 
 * that kd is a fixed point with 2 values
 * behind the seperator.
 */
void setKd(struct pid_data *data, int32_t kd) {
  data->kd = kd;
}

void setTargetValue(struct pid_data *data, int32_t target_val) {
  data->target_val = target_val;
}

void getInternalPIDValues(int16_t *pPart1, int16_t *iPart1, int16_t *dPart1) {
  *pPart1 = pPart;
  *iPart1 = iPart;
  *dPart1 = dPart;
}

int32_t pid(struct pid_data *data, int32_t cur_val) {

  pPart = 0;
  iPart = 0;
  dPart = 0;

  int32_t result = 0;
  int32_t error = data->target_val - cur_val;
  
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

void resetPIDStruct(struct pid_data *data) {
  data->target_val = 0;
  data->last_error = 0;
  data->error_sum = 0;
}


void initPIDStruct(struct pid_data *data) {
  data->target_val = 0;
  data->last_error = 0;
  data->error_sum = 0;
  data->max_command_val = 0;
  data->min_command_val = 0;
  data->kp = 0;
  data->ki = 0;
  data->kd = 0;
}


void setMinMaxCommandVal(struct pid_data *data, int32_t min, int32_t max) {
  data->max_command_val = max;
  data->min_command_val = min;
}

void setPidConfiguration(struct pid_data *data, const struct setPidData *config)
{
    setKp(data, config->kp);
    setKi(data, config->ki);
    setKd(data, config->kd);
    setMinMaxCommandVal(data , -config->minMaxPidOutput, config->minMaxPidOutput);    
}
