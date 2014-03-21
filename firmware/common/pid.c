#include "pid.h"
#include "packets.h"
#include "printf.h"

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

void getPidDebugData(const struct pid_data *data, struct pidDebugData *debug) 
{
    debug->pPart = data->pPart;
    debug->iPart = data->iPart;
    debug->dPart = data->dPart;

    if(data->pPart > INT16_MAX)
	debug->pPart = INT16_MAX;
    if(data->pPart < INT16_MIN)
	debug->pPart = INT16_MIN;

    if(data->iPart > INT16_MAX)
	debug->iPart = INT16_MAX;
    if(data->pPart < INT16_MIN)
	debug->iPart = INT16_MIN;

    if(data->iPart > INT16_MAX)
	debug->iPart = INT16_MAX;
    if(data->iPart < INT16_MIN)
	debug->iPart = INT16_MIN;

    debug->minMaxPidOutput = data->max_command_val;
}


int32_t pid(struct pid_data *data, int32_t cur_val) {

    int32_t result = 0;
    int32_t error = data->target_val - cur_val;
  
    if ( data->kp ) {
	data->pPart = ((error) * data->kp) / 10;
	result += data->pPart;
    }
  
    if ( data->ki ) {
	data->error_sum += (error + data->last_error) / 2;
	const int32_t maxErrorSum = (data->max_command_val * 10) / data->ki;

	//anti wind up, limit error sum
	if(data->error_sum > maxErrorSum) {
	    data->error_sum = maxErrorSum;
	}
	
	if(data->error_sum < -maxErrorSum) {
	    data->error_sum = -maxErrorSum;
	}
	
	//(e_sum)  * ki
	data->iPart = (data->error_sum * data->ki) / 10;
	result += data->iPart;
    }
    
    if ( data->kd ) {
	data->dPart = ((error - data->last_error) * data->kd) / 10;
	result += data->dPart;
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
  data->pPart = 0;
  data->iPart = 0;
  data->dPart = 0;
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
  data->pPart = 0;
  data->iPart = 0;
  data->dPart = 0;
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
    printf("Got new PID Values P:%i, I%i, D%i, MaxPWM%i MinPWM%i ", data->kp, data->ki, data->kd, data->max_command_val, data->min_command_val);

}
