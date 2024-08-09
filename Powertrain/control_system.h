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
#include "adc_sensors.h"
#include "can_communication.h"

/*==================================== SAFETY PARAMETERS ====================================*/
#define MAX_TC      40                          // Motor Controller [inverter] Max temperature [°C]
#define MAX_TM      40                          // Motor Max temperature [°C]


/*==================================== MECHANIC PARAMETERS ====================================*/
#define PI          3.14159265358979323846      // PI constant 
#define KC          1                           // Differential Constant ?TO BE DELETED 


/*======================================== Structs ========================================*/
// OLD: ?TO BE DELETED
struct VelocityStruct{
    uint16_t RPM_W1;    // [LEFT WHEEL ]
    uint16_t RPM_W2;    // [RIGHT WHEEL]
};

struct DcStruct{
    uint16_t Dc_1;    // [LEFT WHEEL ]
    uint16_t Dc_2;    // [RIGHT WHEEL]
};

/*======================================== Aux Functions ========================================*/
void ElectronicDifferential(float Steering_dg, uint16_t Wv_rpm, float apps,float Wc_Motor[]);    // Calc Electronic Differential

// Trash
VelocityStruct get_Differential(float Steering_dg, uint16_t Wv_rpm);
bool Error_Temperature(int16_t Temp_motor, int16_t Temp_controlller);
void print_differential(uint16_t Apps_1, float Steering_dg, VelocityStruct Wheel_Velocity);
void print_Control(uint16_t Apps_1,uint16_t Apps_2,uint16_t BSE, float Steering_dg, VelocityStruct Wheel_Velocity);
void print_errors(bool ERROR_State,bool Error_APPS,bool Error_BPPC, bool Error_Motor); 


/*======================================== CLASS ========================================*/
class ControlSystem {
    private:
    // Atributes
    PedalSensor APPS_1;                     // Accel. Pedal Sensor 1
    PedalSensor APPS_2;                     // Accel Pedal Sensor 2
    PedalSensor BSE_sensor;                 // Brake Pedal Sensor
    SteeringSensor Steering_sensor;         // Steering Wheel angle sensor
    
    bool Error_APPS{0};                     // Apps eror, True = discrepancy between Apps 1 and 2
    bool Error_BPPC{0};                     // BSE and error, 
    bool Error_Circuit{0};                  // 
    bool Error_Motor{0};                    // 

    bool AppsError_flag{0};                 // 
    bool ERROR_State{0};                    // 
    float Error_Start_Time{0};              // 

    float uk[3]{0};                         // Control Signal Iterations uk[0] = current, uk[1] = Previous ...
    float ek[3]{0};                         // Error Signal Iterations ek[0] = current, ek[1] = Previous ...

    float Kp{1};                            // Proporcional Gain
    float Ki{0};                            // Integral Gain
    float Kd{0};                            // Derivative Gain
    float Tf{0};                            // Filter Constant
    float Ts{1e-3};                         // Sampling Period



    public:
    // Methods
    void APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2);                // Acc. pedal Plausibility
    void BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val);            // Break pedal Plausibility
    void Motor_Error_Check(RxStruct Inverter_1, RxStruct Inverter_2);       //
    void setGains(float Kp, float Ki, float Kd, float Ts);
    
    // void Error_Check();
    DcStruct control();
    uint16_t control(float apps, float setpoint);
    void OpenLoop();
    

    // Constructors
    public:
    ControlSystem(PedalSensor APPS_1, PedalSensor APPS_2, PedalSensor BSE_sensor, SteeringSensor Steering_sensor);
    ControlSystem();
};


#endif