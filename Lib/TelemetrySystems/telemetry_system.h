// /***
//  * Capibarib E-racing
//  * Federal University of Pernambuco (UFPE)
//  * Group Area: Powertrain
 
//  * This file is part of the car's Telemetry system, tasked to send data to 
//  * 
//  * the interface monitors data from the Motor/Motor controller connected via USB
//  ***/
#ifndef _TELEMETRY_SYSTEM_H_
#define _TELEMETRY_SYSTEM_H_

// #include "mbed.h"
// #include "CAN.h"
// #include <cstdint>
// // #include "control_system.h"
// // #include "can_communication.h"

// class TelemetrySystem{
//     uint8_t Ts_ms;


//     void send();
// };




// // void Telemetry_Test(uint16_t val1, float val2, float val3);
// // void Telemetry_Print(RxStruct Motor);
// // void Print_Sensors(uint16_t Apps1, uint16_t Apps2, uint16_t BSE, float Steering);
// // void Print_Duty_c(uint16_t Vel);


// // /*==================================  TEST ==================================*/
// // //test Telemetry graphs with apps sensors [so you cant test without the motor]
// // inline void Telemetry_Test(uint16_t val1, uint16_t val2, float val3){
// //     RxStruct Rx_apps;
// //     double p1=(double(val1)/65535)*100;
// //     double p2=(double(val2)/65535)*9000;
    
// //     val3=100+val3;
// //     Rx_apps.Supply_Voltage= 4*p1;
// //     Rx_apps.Temp_Controller= val3;
// //     Rx_apps.Temp_motor= 2*val3;
// //     Rx_apps.RPM=p2;
// //     Rx_apps.PWM_read=p1;
// //     Rx_apps.Current=p1*2;

// //     Telemetry_Print(Rx_apps);
// // }


// /*==================================  MOTOR MONITOR INTERFACE ==================================*/
// // void Telemetry_Print(RxStruct Motor){
// // // message= '[CAN]: Time= 3000.20 ms || PWM= 100% || RPM= 300 || Vs=300 V || Ic= 200 A  || Tm= 20 °C || Tc= 21 °C'
// //     float time_sec= float(current_ms())/1000;
// //     // printf("\n==================================================================\n");
// //     printf("\n[CAN]: Time= %.2f || PWM= %f%%,  RPM= %d", time_sec,Motor.PWM_read, Motor.RPM);
// //     printf(", Vs=%f V, Ic= %d A ", Motor.Supply_Voltage, Motor.Current);
// //     printf(", Tm= %d °C , Tc= %d °C", Motor.Temp_motor, Motor.Temp_Controller);
// //     // printf("\n==================================================================\n");
// // }


// // void Print_Duty_c(uint16_t Vel){
// //     double Percentage=(double(Vel)/65535)*100;
// //     printf("\n Power [%%]: %.2f %%\n",Percentage);

// // }

// inline void Print_Sensors(float Apps1,float Apps2, float BSE, float Steering){
//     printf("\n APPS1: %.2f , APPS2: %.2f",Apps1, Apps2);
//     printf(" , Break: %.2f, Steering: %.2f°\n",BSE, Steering);

// }



// /*================================== TELEMETRY ==================================*/
// // Future
#endif