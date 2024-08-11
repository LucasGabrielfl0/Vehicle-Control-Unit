/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * File for the Digital (and Differential) Control of the BLDC Nova 15 Motor 
 * using the MST 400-200 Motor controller

 ***/

#ifndef _CONTROL_SYSTEM_H_
#define _CONTROL_SYSTEM_H_

#include "mbed.h"
#include <cstdint>
#include <time.h>
// #include "adc_sensors.h"
// #include "can_communication.h"

/*==================================== SAFETY PARAMETERS ====================================*/
#define MAX_TC      40                          // Motor Controller [inverter] Max temperature [°C]
#define MAX_TM      40                          // Motor Max temperature [°C]


/*==================================== CONTROL PARAMETERS ====================================*/
#define MAX_RPM     8000                         // PI constant 
#define PI  3.14159265358979323846                  // PI constant 


/*==================================== MECHANIC PARAMETERS ====================================*/
const float TRACK_WIDTH =  1.0;                     // [m] "Bitola" distance between the wheels on the same axle
const float WHEELBASE   =  1.0;                     // [m] Distance between the front and rear axles           
const float KC =  TRACK_WIDTH / ( 2*WHEELBASE );    // [adm] constant for Differential



/*======================================== Aux Functions ========================================*/
void ElectronicDifferential(float Steering_dg , float apps,float Wc_Motor[]);       // Calc Electronic Differential
void OpenLoopDifferential(float Steering_dg, uint16_t Apps, uint16_t Dc_Motor[]);   // Diff in Open Loop
void CalcDifferential(float Steering_dg, uint16_t Acc_pedal, uint16_t RPM_dif[]);

/*======================================== CLASS ========================================*/
class ControlSystem {

    // bool Error_APPS{0};                     // Apps eror, True = discrepancy between Apps 1 and 2
    // bool Error_BPPC{0};                     // BSE and error, 
    // bool Error_Circuit{0};                  // 
    // bool Error_Motor{0};                    // 
    public:
    bool ERROR_State{0};                    // 
    bool shutdown{0};

    private:
    float uk[3]{0};                         // Control Signal Iterations uk[0] = current, uk[1] = Previous ...
    float ek[3]{0};                         // Error Signal Iterations ek[0] = current, ek[1] = Previous ...

    float Kp{1};                            // Proporcional Gain
    float Ki{0};                            // Integral Gain
    float Kd{0};                            // Derivative Gain
    float Tf{0};                            // Filter Constant
    int Ts_ms{1};                           // Sampling Period


    // Methods
    public:
    void setGains(float Kp, float Ki, float Kd, float Ts);                  //
    void setup();                                                           //
    // void Error_Check();
    uint16_t control(float apps, float setpoint);
    uint16_t control_RPM(uint16_t RPM_c, uint16_t RPM_setpoint);
    void DifferentialControl(float Steering, uint16_t apps,uint16_t RPM_M[], uint16_t Dc_16b[]);    // Elec. Diff
    void OpenLoop();
    void OpenLoop(float DutyC);

    // Constructors
    public:
    // ControlSystem(PedalSensor APPS_1, PedalSensor APPS_2, PedalSensor BSE_sensor, SteeringSensor Steering_sensor);
    ControlSystem(float _kp, float _ki, float _kd, uint16_t _ts_ms);
    ControlSystem();
};

bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2);                // Acc. pedal Plausibility
bool BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val);            // Break pedal Plausibility
// void Temp_Error_Check(RxStruct Inverter_1, RxStruct Inverter_2);       //


#endif