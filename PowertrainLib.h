/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * File for the Digital (and Differential) Control of the BLDC Nova 15 Motor 
 * 
 * The Open Loop control is used for tests with the motor

 ***/

#ifndef _POWERTRAIN_LIB_H_
#define _POWERTRAIN_LIB_H_

#include "mbed.h"
#include <cstdint>

#include "Powertrain_Lib/adc_sensors.h"
#include "Powertrain_Lib/can_communication.h"
#include "Powertrain_Lib/control_system.h"
#include "Powertrain_Lib/telemetry_system.h"

inline void print_all2(){

    printf("hmmm");
}


#endif