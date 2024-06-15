/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * File for the Digital (and Differential) Control of the BLDC Nova 15 Motor 
 * 
 * The Open Loop control is used for tests with the motor

 ***/

#ifndef _CONTROL_SYSTEM_H_
#define _CONTROL_SYSTEM_H_

#include "mbed.h"
#include <cstdint>
#include <time.h>
#include "angle_sensor.h"

/*==================================== CONTROL PARAMETERS ====================================*/
#define MAX_MST_TEMPRATURE          100         // Motor Controller Max temperature [°C]
#define MAX_NOVA15_TEMPERATURE      100         // Motor Max temperature [°C]
#define MAX_RPM_ERROR               2           // Max acceptable error in Velocity [RPM]

#define MAX_CURRENT_LIMIT           10          // Max Phase Current in the motor [A]
#define MAX_RPM_LIMIT               4500        // Max Velocity of the motor [RPM] 


/*==================================== MECHANIC CONSTANTS ====================================*/
#define D_TW        1                           // Car's track Width [Bitola] in meters
#define A_WB        1                           // Car's Wheelbase in meters 
#define PI          3.14159265358979323846      // PI constant 
#define KC          0.39                        // Differential Constant


#define MAX_TRAVEL      65535                        // Differential Constant

/*======================================== Structs ========================================*/
struct Velocity_struct{
    uint16_t RPM_W1;    // [LEFT WHEEL ]
    uint16_t RPM_W2;    // [RIGHT WHEEL]
};

/*======================================== Functions ========================================*/
inline bool Apps_Error_check(uint16_t Apps_1, uint16_t Apps_2);
inline bool Break_Implausibility(uint16_t Apps_val, uint16_t Break_val);
inline Velocity_struct get_Differential(float Steering_dg, uint16_t Wv_rpm);


/*======================================== Functions ========================================*/
inline bool Apps_Error_check(uint16_t Apps_1, uint16_t Apps_2){

    return 1;
}

inline bool Break_Implausibility(uint16_t Apps_val, uint16_t Break_val){


    return 1;
}

/*======================================== CLASS ========================================*/
class control_systems {
    // Atributes
    bool Error_APPS;
    bool Error_BPPC;
    bool AppsError_flag;
    float Error_Start_Time;
    bool ERROR_State;

    // Methods
    bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2);
    bool BSE_Error_check(uint16_t Apps_val, uint16_t Break_val);

    // Constructors
    public:
    control_systems();
};

/*======================================== Constructors ========================================*/
inline control_systems::control_systems(){

};


// Temperature Shutdown
/*
Interrupt for either motor controller or motor above temperature limit
*/

// Current Shutdown

/*
Interrupt for either motor controller or motor above temperature limit
*/



/*================================== VECELOCITY SETPOINT [RPM] ==================================*/
// Checks if there's a discrepancy bigger than 10%, for longer than 100 ms
inline bool control_systems::APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2){
    // Apps_1 = Accelerator 1 Pedal travel (0 to 100% in the scale of 0 to 16b)
    // Apps_2 = Accelerator 2 Pedal travel (0 to 100% in the scale of 0 to 16b)

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

// Checks if the Accelerator Pedal and the Break are both pressed at the same time than 10%, for longer than 100 ms
inline bool control_systems::BSE_Error_check(uint16_t Apps_val, uint16_t Break_val){
    // Apps_1 = Accelerator 1 Pedal travel (0 to 100% in the scale of 0 to 16b)
    // Apps_2 = Accelerator 2 Pedal travel (0 to 100% in the scale of 0 to 16b)
    
    //If APPS >= 25% of pedal travel and Break is pressed, stops the car 
    if ( (Apps_val >= 0.25*MAX_TRAVEL) and (Break_val >= 0.02*MAX_TRAVEL) ){
        Error_BPPC=1;
    }

    // Error only Stops if APPS goes below 5% of pedal travel
    if(Apps_val <= 0.05*MAX_TRAVEL){
        Error_BPPC=0;
    }

    return Error_BPPC;
}

/*================================== DIFFERENTIAL ==================================*/
// Calculates the Velocity [in RPM] of each wheel during a turn, W1 = [LEFT WHEEL] || W2 = [RIGHT WHEEL]
inline Velocity_struct get_Differential(float Steering_dg, uint16_t Wv_rpm){
    float Ack_rad;
    float del_W;
    Velocity_struct Differential_vel;

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.1415 * Steering_dg - 0.092)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);
    
    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    Differential_vel.RPM_W1= Wv_rpm * ( 1 + del_W);
    Differential_vel.RPM_W2= Wv_rpm * ( 1 - del_W);

    return Differential_vel;
}




/*================================== VECELOCITY CONTROL ==================================*/
// Proportional control + Electronic Differential
// Apps_16b= Accel. Pedal signal scaled [0 = Min angle | 65535 = Max Angle]
inline Velocity_struct control_test(uint16_t Apps_1, uint16_t Apps_2, uint16_t BSE_sensor, uint16_t Steering_dg){
    Velocity_struct Velocity_Wheels;
    uint16_t Apps_val;
    bool Error_APPS, Error_BSE;
    
    // Error Check    
    Error_APPS = Apps_Error_check(Apps_1, Apps_2);              // Apps Error Check
    Error_BSE  = Break_Implausibility(Apps_1, BSE_sensor);      // Break Implausibility Check


    Apps_val=Apps_1;
    if(Error_APPS or Error_BSE){
        Velocity_Wheels.RPM_W1 = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }
    else{
        // get Differential
        Velocity_Wheels = get_Differential(Steering_dg, Apps_val);
    }


    // Return Velocity in each wheel
    return Velocity_Wheels;
}


#endif