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
#include "can_communication.h"

inline void Telemetry_Test(uint16_t val1, float val2, float val3);
inline void Telemetry_Print(Rx_struct Motor);



/*==================================  TEST ==================================*/
//test Telemetry graphs with apps sensors [so you cant test without the motor]
inline void Telemetry_Test(uint16_t val1, float val2, float val3){
    Rx_struct Rx_apps;
    
    Rx_apps.Supply_Voltage= 3*val3;
    Rx_apps.Temp_Controller= val2;
    Rx_apps.Temp_motor= val3;
    Rx_apps.RPM=val1;
    Rx_apps.rx_PWM=val2;
    Rx_apps.Current=val2;

    Telemetry_Print(Rx_apps);
}


/*==================================  MOTOR MONITOR INTERFACE ==================================*/
inline void Telemetry_Print(Rx_struct Motor){
// message= '[CAN]: PWM= 100%,  RPM= 300,  Vs=300 V,  Ic= 200 A ,  Tm= 20 °C ,  Tc= 21 °C'

    printf("\n==================================================================\n");
    printf("[CAN]: PWM= %f%%,  RPM= %d",Motor.rx_PWM, Motor.RPM);
    printf(", Vs=%f V, Ic= %d A ", Motor.Supply_Voltage, Motor.Current);
    printf(", Tm= %d °C , Tc= %d °C", Motor.Temp_motor, Motor.Temp_Controller);
    printf("\n==================================================================\n");
}


inline void Print_Velocity(uint16_t Vel){
    double Percentage=double(Vel)*100/65535;

    printf("\n Power [%%]: %.2f %%\n",Percentage);


}

inline void Print_Sensors(uint16_t Apps1,uint16_t Apps2, uint16_t BSE, float Steering){
    printf("\n APPS1: %d , APPS2: %d",Apps1, Apps2);
    printf(" Break: %d Steering: %.2f°\n",BSE, Steering);


}




/*================================== TELEMETRY ==================================*/
// Future
#endif