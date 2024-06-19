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

#include "Powertrain/adc_sensors.h"                 // Read APPS, BSE and Steering Wheel sensors
#include "Powertrain/can_communication.h"           // CAN 2.0 communication with Inverter
#include "Powertrain/control_system.h"              // Control with Electronic Differential
#include "Powertrain/telemetry_system.h"            // Sends Data to Monitors and external Devices


#endif