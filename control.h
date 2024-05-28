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
inline Velocity_struct get_Differential(float Steering_dg, float Vx_rpm){
    float Ack_dg,Ack_rad;
    float Wv, del_W;
    Velocity_struct Differential_vel;

    // Get Wv [Linear Angular velocity] in rad/s
    Wv= Vx_rpm * (2*PI)/60;
    
    // Get Ackerman Angle [in rad] using the steering Wheel rotation 
    Ack_rad=(0.1415 * Steering_dg - 0.092)* PI/180;

    // Variation 
    del_W= Wv * D_TW * tan(Ack_rad)/(2*A_WB);
    
    // Calculate differential [in Rad/s] and turns into RPM
    Differential_vel.RPM_W1= (Wv + del_W) * 60/(2*PI);
    Differential_vel.RPM_W2= (Wv - del_W) * 60/(2*PI);

    return Differential_vel;
}



/*================================== VECELOCITY CONTROL ==================================*/
// just a draft :)
inline void set_Velocity(){
    uint16_t rpm_SETPOINT, rpm_MOTOR; // int or float? => check digital control
    int16_t rpm_error;

    // rpm_SETPOINT= get_Differential(1);
    
    //Error = Desired rpm - Real RPM
    rpm_error = rpm_SETPOINT - rpm_MOTOR;

    // Only applies control if error is above limit
    if (abs(rpm_error) <= MAX_RPM_ERROR){
        rpm_error =0;
    }

    //control (z)
    //output: DutyCycle
}

/*================================== DUMMY CONTROL FOR TESTS ==================================*/
// Proportional control + Electronic Differential
inline Velocity_struct set_Velocity_1( uint16_t Apps_16b,int Wheel_dg ){
    uint16_t rpm_SETPOINT, rpm_MOTOR; // int or float? => check digital control
    int16_t rpm_error, Wx_rpm;
    Velocity_struct Wheels;

    //APPS TO Wx
    Wx_rpm= Apps_16b;
    // Electronic Diffetential
    //rpm_SETPOINT= get_Differential(1);

    rpm_SETPOINT=1;
    
    //Maps apps to speed

    //Maps apps to speed
    rpm_SETPOINT= Apps_16b;

    //Error = Desired rpm - Current RPM in the motor
    rpm_error = rpm_SETPOINT - rpm_MOTOR;

    // Only applies control if error is above limit
    if (abs(rpm_error) >= MAX_RPM_ERROR){
        Wheels.RPM_W1  = rpm_SETPOINT;
        Wheels.RPM_W2 = rpm_SETPOINT;
    }

    return Wheels;
}

// Proportional control without Electronic Differential
inline Velocity_struct set_Velocity_1(uint16_t Apps_16b){
    uint16_t rpm_SETPOINT, rpm_MOTOR; // int or float? => check digital control
    int16_t rpm_error;
    Velocity_struct Wheels;

    //Maps apps to speed
    rpm_SETPOINT= Apps_16b;

    //Error = Desired rpm - Current RPM in the motor
    rpm_error = rpm_SETPOINT - rpm_MOTOR;

    // Only applies control if error is above limit
    if (abs(rpm_error) >= MAX_RPM_ERROR){
        Wheels.RPM_W1  = rpm_SETPOINT;
        Wheels.RPM_W2 = rpm_SETPOINT;
    }

    return Wheels;
}




#endif