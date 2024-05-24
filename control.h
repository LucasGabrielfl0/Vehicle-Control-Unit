/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * File for the Digital (and Differential) Control of the BLDC Nova 15 Motor 
 * work in progress

 ***/

#ifndef _CONTROL_SYSTEM_H_
#define _CONTROL_SYSTEM_H_

#include "mbed.h"
#include <cstdint>
#include <time.h>

/*================================== CONTROL PARAMETERS ==================================*/
#define MAX_MST_TEMPRATURE      100         // Motor Controller Max temperature [°C]
#define MAX_NOVA15_TEMPERATURE  100         // Motor Max temperature [°C]
#define MAX_RPM_ERROR           2           // Max acceptable error in Velocity [RPM]

#define MAX_CURRENT_LIMIT       10          // Max Phase Current in the motor [A]
#define MAX_RPM_LIMIT           4500        // Max Velocity of the motor [RPM] 


/*======================================== Structs ========================================*/
struct Velocity_struct{
    uint16_t RPM_W1;  // [LEFT WHEEL ]
    uint16_t RPWM_W2; // [RIGHT WHEEL]
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
inline float get_Differencial(float Wheel_dg){
    float Ack_dg;

    return Ack_dg;
}

/*================================== VECELOCITY CONTROL ==================================*/
// just a draft :)
inline void set_Velocity(){
    uint16_t rpm_SETPOINT, rpm_MOTOR; // int or float? => check digital control
    int16_t rpm_error;

    rpm_SETPOINT= get_Differencial(1);
    
    //Error = Desired rpm - Real RPM
    rpm_error = rpm_SETPOINT - rpm_MOTOR;

    // Only applies control if error is above limit
    if (abs(rpm_error) <= MAX_RPM_ERROR){
        rpm_error =0;
    }

    //control (z)
    //output: DutyCycle
}




#endif