#ifndef __PID_H
#define __PID_H

#include <stdint.h>

struct pid_data {
  int32_t kp;
  int32_t ki;
  int32_t kd;
  int32_t target_val;
  int32_t last_error;
  int32_t max_command_val;
  int32_t min_command_val;
  int32_t error_sum;
};


/**
 * This function sets the minimum and maximum return 
 * value of the PID function. The return value of PID()
 * it clamped automatically clamped
 */
void setMinMaxCommandVal(struct pid_data *data, int32_t min, int32_t max);

/**
 * This function set the kp value. Note, 
 * that kp is a fixed point with 2 values
 * behind the , 
 */
void setKp(struct pid_data *data, int32_t kp);

/**
 * This function set the ki value. Note, 
 * that ki is a fixed point with 2 values
 * behind the , 
 */
void setKi(struct pid_data *data, int32_t ki);

/**
 * This function set the kd value. Note, 
 * that kd is a fixed point with 2 values
 * behind the , 
 */
void setKd(struct pid_data *data, int32_t kd);

/**
 * This function clears internal values for
 * I and D part but not kp, ki, kd, min and 
 * maxval.
 */
void resetPIDStruct(struct pid_data *data);

void setTargetValue(struct pid_data *data, int32_t target_val);

int32_t pid(struct pid_data *data, int32_t cur_val);

void getInternalPIDValues(int16_t *pPart, int16_t *iPart, int16_t *dPart);

void initPIDStruct(struct pid_data *data);

#endif
