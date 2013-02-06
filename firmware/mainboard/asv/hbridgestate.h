/* 
 * File:   hbridgestate.h
 * Author: clausen
 *
 * Created on 9. Januar 2013, 19:49
 */

#ifndef HBRIDGESTATE_H
#define	HBRIDGESTATE_H

/**
 * Initiates the HBridgeStateMaschine of the Mainboard. Call this first!
 */
void initHbridgeState();

/**
 * handles the states and tries to set the hbridges to the wanted states
 * call this each time you want to change something or in a loop 
 */
void processHbridgestate();


#endif	/* HBRIDGESTATE_H */

