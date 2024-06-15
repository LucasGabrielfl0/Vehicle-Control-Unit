/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file is part of the car's Telemetry system, tasked to send data to 
 * 
 * the interface monitors data from the Motor/Motor controller connected via USB
 ***/
#ifndef _TELEMETRY_SYSTEM_H_
#define _TELEMETRY_SYSTEM_H_

#include "mbed.h"
#include "CAN.h"
#include <cstdint>
#include "motor_can.h"

inline void Telemetry_Test(float val1, int val2, int val3);
inline void Telemetry_Print(Rx_struct Motor);



/*==================================  TEST ==================================*/
//test Telemetry graphs with apps sensors [so you cant test without the motor]
inline void Telemetry_Test(float val1, int val2, int val3){
    Rx_struct Rx_apps;
    
    Rx_apps.Supply_Voltage= 3*val1;
    Rx_apps.Temp_Controller= val2;
    Rx_apps.Temp_motor= val3;
    Rx_apps.RPM=val2*1000;
    Rx_apps.rx_PWM=val2;
    Rx_apps.Current=val2;

    Telemetry_Print(Rx_apps);
}


/*==================================  MOTOR MONITOR INTERFACE ==================================*/
inline void Telemetry_Print(Rx_struct Motor){
// message= '[CAN]: | TIME: 20:15:00 | PWM= 100%,  RPM= 300,  Vs=300 V,  Ic= 200 A ,  Tm= 20 °C ,  Tc= 21 °C'

    printf("\n==================================================================\n");
    printf("[CAN]: | TIME: 20:15:00 | PWM= %f%%,  RPM= %d",Motor.rx_PWM, Motor.RPM);
    printf(",\n  Vs=%f V,  Ic= %d A ", Motor.Supply_Voltage, Motor.Current);
    printf(",\n  Tm= %d °C ,  Tc= %d °C", Motor.Temp_motor, Motor.Temp_Controller);
    printf("\n==================================================================\n");
}







/*================================== TELEMETRY ==================================*/
// Future
#endif