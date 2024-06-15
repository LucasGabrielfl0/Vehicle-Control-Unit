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
#define MAX_TM      40         // Motor Controller Max temperature [°C]
#define MAX_TC      40         // Motor Max temperature [°C]

/*==================================== MECHANIC PARAMETERS ====================================*/
#define PI          3.14159265358979323846      // PI constant 
#define KC          0.39                        // Differential Constant


/*======================================== Structs ========================================*/
struct Velocity_struct{
    uint16_t RPM_W1;    // [LEFT WHEEL ]
    uint16_t RPM_W2;    // [RIGHT WHEEL]
};

/*======================================== Aux Functions ========================================*/
Velocity_struct get_Differential(float Steering_dg, uint16_t Wv_rpm);
bool Error_Temperature(int16_t Temp_motor, int16_t Temp_controlller);


/*======================================== CLASS ========================================*/
class ControlSystem {
    private:
    // Atributes
    bool Error_APPS{0};
    bool Error_BPPC{0};
    bool Error_Motor{0};

    bool AppsError_flag{0};
    bool ERROR_State{0};
    float Error_Start_Time{0};

    public:
    // Methods
    bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2);
    bool BSE_Error_check(uint16_t Apps_val, uint16_t Break_val);
    void Motor_Error_Check(Rx_struct Inverter_1, Rx_struct Inverter_2);
    Velocity_struct control_test(uint16_t Apps_1, uint16_t Apps_2, uint16_t BSE_sensor, uint16_t Steering_dg);

    // Constructors
    public:
    ControlSystem();
};

/*======================================== Constructors ========================================*/
inline ControlSystem::ControlSystem(){

};


/*======================================== VECELOCITY CONTROL ==================================*/
// Apps_16b= Accel. Pedal signal scaled [0 = Min travel | 65535 = Max travel]
inline Velocity_struct ControlSystem::control_test(uint16_t Apps_1, uint16_t Apps_2, uint16_t BSE_sensor, uint16_t Steering_dg){
    Velocity_struct Velocity_Wheels;
    uint16_t Apps_val;
    bool Error_Apps, Error_BSE, Error_Temp;
    
    // Error Check
    Error_Apps = APPS_Error_check(Apps_1, Apps_2);              // Apps Error Check
    Error_BSE  = BSE_Error_check(Apps_1, BSE_sensor);           // Break Implausibility Check
    Error_Temp = Error_Temperature(1,1);

    Apps_val=Apps_1;

    if(Error_APPS or Error_BSE or Error_Temp){
        Velocity_Wheels.RPM_W1 = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }
    else{
        // get Differential
        Velocity_Wheels = get_Differential(Steering_dg, Apps_val);
    }

    Velocity_Wheels.RPM_W1=Apps_1;
    Velocity_Wheels.RPM_W2=Apps_1;

    // Return Velocity in each wheel
    return Velocity_Wheels;
}


/*================================== ACCELERATION PEDAL PLAUSIBILITY CHECK ==================================*/
// Checks if there's a discrepancy bigger than 10%, for longer than 100 ms
inline bool ControlSystem::APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2){
    if ( abs(Apps_1 - Apps_2) > (0.1 * max(Apps_1, Apps_2)) ){
        if(AppsError_flag == 0) { //Starts Counting
            Error_Start_Time = millis(); 
            AppsError_flag = 1;
        }

        if(millis() - Error_Start_Time > 100) { //if the error continues after 100ms
            Error_APPS= 1;
        } else{
            Error_APPS= 0;
        }

    } else { //if error ceased
        AppsError_flag = 0;
        Error_APPS= 0;
        }

    return Error_APPS;
}

/*====================================== BREAK PEDAL PLAUSIBILITY CHECK ======================================*/
// Checks if the Accel. and Break Were both pressed at the same time
inline bool ControlSystem::BSE_Error_check(uint16_t Apps_val, uint16_t Break_val){    
    //If APPS >= 25% of pedal travel and Break is pressed, stops the car 
    if ( (Apps_val >= 0.25*PEDAL_MAX) and (Break_val >= 0.02*PEDAL_MAX) ){
        Error_BPPC = 1;
    }

    // Error only Stops if APPS goes below 5% of pedal travel
    if(Apps_val <= 0.05*PEDAL_MAX){
        Error_BPPC = 0;
    }

    return Error_BPPC;
}
/*====================================== MOTOR/MOTOR CONTROLLER ERROR CHECK ======================================*/
// Checks if the motor sent 
inline void ControlSystem::Motor_Error_Check(Rx_struct Inverter_1, Rx_struct Inverter_2){
    int16_t Tm_1 = Inverter_1.Temp_motor; 
    int16_t Tm_2 = Inverter_2.Temp_motor; 
    int16_t Tc_1 = Inverter_1.Temp_Controller;
    int16_t Tc_2 = Inverter_2.Temp_Controller;
    
    // if Temperature is above limit, shuts the car down
    if( (Tm_1> MAX_TM) || (Tm_2> MAX_TM) || (Tc_1> MAX_TC) || (Tc_2> MAX_TC) ){
        Error_Motor = 1;
    } 
    else {
        Error_Motor = 0;
    }    
}

/*=========================================== Functions ===========================================*/

/*=========================================== DIFFERENTIAL ===========================================*/
// Calculates the Velocity [in RPM] of each wheel during a turn, W1 = [LEFT WHEEL] || W2 = [RIGHT WHEEL]
inline Velocity_struct get_Differential(float Steering_dg, uint16_t Wv_rpm){
    float Ack_rad;
    float del_W;
    Velocity_struct Differential_vel;

    // For very low angles in the Steering wheel, there's no change
    if( abs(Steering_dg)< 4){
        Steering_dg=0;
    }

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.1415 * Steering_dg)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);
    
    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    Differential_vel.RPM_W1= Wv_rpm * ( 1 + del_W);
    Differential_vel.RPM_W2= Wv_rpm * ( 1 - del_W);

    return Differential_vel;
}


#endif