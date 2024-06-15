/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * This file contains all Libraries used in the system's Powertrain
 * 

 ***/
#ifndef _POWERTRAIN_LIB_H_
#define _POWERTRAIN_LIB_H_

#include "mbed.h"
#include <cstdint>

#include "Powertrain_Lib/adc_sensors.h"                 // Read APPS, BSE and Steering Wheel sensors
#include "Powertrain_Lib/can_communication.h"           // CAN 2.0 communication with Inverter
#include "Powertrain_Lib/control_system.h"              // Control with Electronic Differential
#include "Powertrain_Lib/telemetry_system.h"            // Sends Data to Monitors and external Devices

inline void print_all2(){

    printf("hmmm");
}


#endif