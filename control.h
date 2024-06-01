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
#define MAX_MST_TEMPRATURE      100         // Motor Controller Max temperature [°C]
#define MAX_NOVA15_TEMPERATURE  100         // Motor Max temperature [°C]
#define MAX_RPM_ERROR           2           // Max acceptable error in Velocity [RPM]

#define MAX_CURRENT_LIMIT       10          // Max Phase Current in the motor [A]
#define MAX_RPM_LIMIT           4500        // Max Velocity of the motor [RPM] 

/*==================================== MECHANIC CONSTANTS ====================================*/
#define D_TW     1                          //Car's track Width [Bitola] in meters
#define A_WB     1                          //Car's Wheelbase in meters 
#define PI      3.14159265358979323846      // PI 
#define KC      0.39                        //Diferencial

/*======================================== Structs ========================================*/
struct Velocity_struct{
    uint16_t RPM_W1;  // [LEFT WHEEL ]
    uint16_t RPM_W2; // [RIGHT WHEEL]
};



/*======================================== CLASS ========================================*/
class control_systems {
    // Atributes

    // Methods


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
// Maps Apps signal in RPM
//  



/*================================== DIFFERENTIAL ==================================*/
// Calculates the Velocity [in RPM] of each wheel during a turn
// W1 = [LEFT WHEEL] || W2 = [RIGHT WHEEL]
inline Velocity_struct get_Differential(float Steering_dg, uint16_t Wv_rpm){
    float Ack_rad;
    float del_W;
    Velocity_struct Differential_vel;

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.1415 * Steering_dg - 0.092)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);
    
    // Calculate differential [in Rad/s] and turns into RPM
    // Note: there's a Implicit Type Conversion there
    Differential_vel.RPM_W1= Wv_rpm * ( 1 + del_W);
    Differential_vel.RPM_W2= Wv_rpm * ( 1 - del_W);

    return Differential_vel;
}

/*================================== VECELOCITY CONTROL ==================================*/
// Proportional control + Electronic Differential
// Apps_16b= Accel. Pedal signal scaled [0 = Min angle | 65535 = Max Angle]
// Wheel_dg= Steering Wheel
inline Velocity_struct get_PWM( uint16_t Apps_16b,float Wheel_dg, bool Error_flag ){
    uint16_t rpm_SETPOINT, rpm_MOTOR; // int or float? => check digital control
    int16_t rpm_error, Wx_rpm;
    Velocity_struct Velocity_Wheels;

    //Maps APPS signal into Velocity [Min angle, Max Angle] -> [Min Speed, Max Speed]
    Wx_rpm=map(Apps_16b, APPS_MIN_READ, APPS_MAX_READ, 0, 65535); //

    // Electronic Diffetential
    Velocity_Wheels= get_Differential(Wheel_dg, Apps_16b);

    //Error = Desired rpm - Current RPM in the motor
    rpm_error = rpm_SETPOINT - rpm_MOTOR;

    // // Only applies control if error is above limit
    // if (abs(rpm_error) >= MAX_RPM_ERROR){
    //     Velocity_Wheels.RPM_W1  = rpm_SETPOINT;
    //     Velocity_Wheels.RPM_W2 =  rpm_SETPOINT;
    // }

    // If theres an Error, PWM = 0%
    if(Error_flag==true){
        Velocity_Wheels.RPM_W1  = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }

    return Velocity_Wheels;
}

#endif