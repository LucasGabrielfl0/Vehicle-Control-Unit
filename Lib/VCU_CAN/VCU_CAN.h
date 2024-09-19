// /***
//  * Capibarib E-racing
//  * Federal University of Pernambuco (UFPE)
//  * Group Area: Powertrain

//  * This file contains methods used for the car's embedded system's CAN communication with:
//  * 1. Baterry Management System (BMS)
//  * 2. ESP32 used to control the data-logger and display
//  ***/
#ifndef _VCU_CAN_H_
#define _VCU_CAN_H_

#include "mbed.h"
#include "CAN.h"

// CAN ADRESS
#define ESP32_CAN_ADDRESS       1
#define BMS_CAN_ADDRESS         2
#define VCU_CAN_ADDRESS         3

// Aux Functions
void receive_from_BMS();        // Get data from BMS via CAN
void send_to_ESP();             // Send data to ESP32 via CAN
void VCU_CAN_Received();

// Structs








#endif