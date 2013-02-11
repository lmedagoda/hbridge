#ifndef TIMEOUT_H
#define TIMEOUT_H

#include <stdint.h>

/**
 * Initializes the timeout
 * @param timeoutInMs The time in ms after which the 
 * 			function timeout_hasTimeout will return a timeout 
 * */
void timeout_init(int timeoutInMs);

/**
 * Sets the timeout counter to zero.
 * */
void timeout_reset();

/**
 * Checks if a timeout occured.
 * @return !0 on a timeout.
 * */
uint8_t timeout_hasTimeout();

#endif
