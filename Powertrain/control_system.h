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
#define KC          1                           // Differential Constant


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
    bool Error_Circuit{0};
    bool Error_Motor{0};

    bool AppsError_flag{0};
    bool ERROR_State{0};
    float Error_Start_Time{0};

    public:
    // Methods
    void APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2);
    void BSE_Error_check(uint16_t Apps_val, uint16_t Break_val);
    void Motor_Error_Check(Rx_struct Inverter_1, Rx_struct Inverter_2);
    Velocity_struct control(PedalSensor Apps_1, PedalSensor Apps_2, PedalSensor BSE_sensor, SteeringSensor Steering_sensor);
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
    
    // Error Check
    APPS_Error_check(Apps_1, Apps_2);              // Apps Error Check
    BSE_Error_check(Apps_1, BSE_sensor);           // Break Implausibility Check
    
    //debug
    Error_APPS  = false;
    Error_BPPC  = false;
    Error_Motor = false;

    if(Error_APPS or Error_BPPC or Error_Motor){
        Velocity_Wheels.RPM_W1 = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }
    else{
        // get Differential
        Velocity_Wheels = get_Differential(Steering_dg, Apps_1);
    }

    //debug
    // Velocity_Wheels.RPM_W1=Apps_1;
    // Velocity_Wheels.RPM_W2=Apps_1;

    // Return Velocity in each wheel
    return Velocity_Wheels;
}



/*======================================== VECELOCITY CONTROL ==================================*/
// Apps_16b= Accel. Pedal signal scaled [0 = Min travel | 65535 = Max travel]
inline Velocity_struct ControlSystem::control(PedalSensor APPS_1, PedalSensor APPS_2, PedalSensor BSE_sensor, SteeringSensor Steering_sensor){
    Velocity_struct Velocity_Wheels;
    
    uint16_t Apps_1 = APPS_1.read_pedal();
    uint16_t Apps_2 = APPS_1.read_pedal();
    uint16_t BSE = BSE_sensor.read_pedal();
    float Steering_dg= Steering_sensor.read_angle();

    // Error Check
    APPS_Error_check(Apps_1, Apps_2);                                   // Apps Error Check
    BSE_Error_check(Apps_1, BSE);                                       // Break Implausibility Check
    // Error_Circuit = APPS_1.
    // Circuit_Error_check(APPS_1, APPS_2, BSE_sensor, Steering_sensor);   // Short/Open Circuit check


    ERROR_State= Error_APPS || Error_BPPC || Error_Motor;
    
    //debug
    ERROR_State = false;

    if(ERROR_State){
        Velocity_Wheels.RPM_W1 = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }
    else{
        // get Differential
        Velocity_Wheels = get_Differential(Steering_dg, Apps_1);
    }

    //debug
    // Velocity_Wheels.RPM_W1=Apps_1;
    // Velocity_Wheels.RPM_W2=Apps_1;

    // Return Velocity in each wheel
    return Velocity_Wheels;
}











/*================================== ACCELERATION PEDAL PLAUSIBILITY CHECK ==================================*/
// Checks if there's a discrepancy bigger than 10%, for longer than 100 ms
inline void ControlSystem::APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2){
    // Checks 10% discrepancy
    if ( abs(Apps_1 - Apps_2) > (0.1 * max(Apps_1, Apps_2)) ){
        // if there's discrepancy,sets flag and starts Counting
        if(AppsError_flag == 0) {
            Error_Start_Time = millis(); 
            AppsError_flag = 1;
        }
        
        // Checks if the error continues after 100ms and sets error
        if(millis() - Error_Start_Time > 100) { 
            Error_APPS= 1;
        } else{
            Error_APPS= 0;
        }

    }
    //if error ceased resets flags
    else { 
        AppsError_flag = 0;
        Error_APPS= 0;
        }

}

/*====================================== BREAK PEDAL PLAUSIBILITY CHECK ======================================*/
// Checks if the Accel. and Break Were both pressed at the same time
inline void ControlSystem::BSE_Error_check(uint16_t Apps_val, uint16_t Break_val){    
    //If APPS >= 25% of pedal travel and Break is pressed, stops the car 
    if ( (Apps_val >= 0.25*PEDAL_MAX) && (Break_val >= 0.02*PEDAL_MAX) ){
        Error_BPPC = 1;
    }

    // Error only Stops if APPS goes below 5% of pedal travel
    if(Apps_val <= 0.05*PEDAL_MAX){
        Error_BPPC = 0;
    }

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

    // printf("%f",Steering_dg);
    // For very low angles in the Steering wheel, there's no change
    if( abs(Steering_dg)< 5){
        Steering_dg=0;
    }

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.2 * Steering_dg)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);

    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    Differential_vel.RPM_W1= Wv_rpm * ( 1 + del_W);
    Differential_vel.RPM_W2= Wv_rpm * ( 1 - del_W);

    return Differential_vel;
}


#endif