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
#include "telemetry_system.h"
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
void print_differential(uint16_t Apps_1, float Steering_dg, Velocity_struct Wheel_Velocity);
void print_Control(uint16_t Apps_1,uint16_t Apps_2,uint16_t BSE, float Steering_dg, Velocity_struct Wheel_Velocity);
void print_errors(bool ERROR_State,bool Error_APPS,bool Error_BPPC, bool Error_Motor); 
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
    
    // Test only
    Velocity_struct control(PedalSensor Apps_1);
    Velocity_struct control(PedalSensor Apps_1, SteeringSensor Steering_sensor);
    Velocity_struct control(PedalSensor Apps_1, PedalSensor Apps_2, PedalSensor BSE_sensor);
    Velocity_struct control_test(uint16_t Apps_1, uint16_t Apps_2, uint16_t BSE_sensor, uint16_t Steering_dg);

    // Constructors
    public:
    ControlSystem();
};

/*======================================== Constructors ========================================*/
inline ControlSystem::ControlSystem(){

};





/*======================================== VELOCITY CONTROL ==================================*/
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

    ERROR_State= Error_APPS || Error_BPPC || Error_Motor;

    if(ERROR_State){
        Velocity_Wheels.RPM_W1 = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }
    else{
        // get Differential
        Velocity_Wheels = get_Differential(Steering_dg, Apps_1);
    }
    
    print_Control(Apps_1, Apps_2, BSE, Steering_dg, Velocity_Wheels);
    print_errors(ERROR_State,Error_APPS,Error_BPPC,Error_Motor);

    // Return Velocity in each wheel
    return Velocity_Wheels;
}


// test plausibility/safety part of the control
inline Velocity_struct ControlSystem::control(PedalSensor APPS_1, PedalSensor APPS_2, PedalSensor BSE_sensor){
    Velocity_struct Velocity_Wheels;
    
    uint16_t Apps_1 = APPS_1.read_pedal();
    // uint16_t Apps_2 = APPS_2.read_pedal();
    uint16_t Apps_2 = Apps_1;
    uint16_t BSE = BSE_sensor.read_pedal();
    // uint16_t BSE = 0;

    // Error Check
    APPS_Error_check(Apps_1, Apps_2);                                   // Apps Error Check
    BSE_Error_check(Apps_1, BSE);                                       // Break Implausibility Check


    ERROR_State= Error_APPS || Error_BPPC || Error_Motor;
    
    Velocity_Wheels.RPM_W1=Apps_1;
    Velocity_Wheels.RPM_W2=Apps_1;
    print_errors(ERROR_State,Error_APPS,Error_BPPC,Error_Motor);

    if(ERROR_State){
        Velocity_Wheels.RPM_W1 = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }

    print_Control(Apps_1, Apps_2, BSE, 0, Velocity_Wheels);

    // Return Velocity in each wheel
    return Velocity_Wheels;
}

// Test Differential
inline Velocity_struct ControlSystem::control(PedalSensor APPS_1, SteeringSensor Steering_sensor){
    Velocity_struct Velocity_Wheels;
    
    uint16_t Apps_1 = APPS_1.read_pedal();
    float Steering_dg= Steering_sensor.read_angle();

    // get Differential
    Velocity_Wheels = get_Differential(Steering_dg, Apps_1);

    print_differential(Apps_1, Steering_dg,Velocity_Wheels);
    // Return Velocity in each wheel
    return Velocity_Wheels;
}


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
            printf("APPS: ERROR DETECTED");
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
        printf("BSE: ERROR DETECTED");
    }

    // Error only Stops if APPS goes below 5% of pedal travel
    if(Apps_val < 0.05*PEDAL_MAX){
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
    int64_t RPM_W1,RPM_W2;
    float Ack_rad;
    float del_W;
    Velocity_struct Differential_vel;

    // printf("%f",Steering_dg);
    // For very low angles in the Steering wheel, there's no change
    if( abs(Steering_dg)< 5){
        Steering_dg=0;
    }

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.14 * Steering_dg)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);

    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    RPM_W1= Wv_rpm * ( 1 + del_W);
    RPM_W2= Wv_rpm * ( 1 - del_W);


    // Prevent overflow/underflow [values above 16b or below 0 going to the uint16 output]
    //if overflow, sets max
    if(RPM_W1>PEDAL_MAX){
        RPM_W1=PEDAL_MAX;
    }
    if(RPM_W2>PEDAL_MAX){
        RPM_W2=PEDAL_MAX;
    }

    //if overflow, sets min
    if(RPM_W1<0){
        RPM_W1=0;
    }

    if(RPM_W2<0){
        RPM_W2=0;
    }

    // Return Velocity in each Wheel
    Differential_vel.RPM_W1 = uint16_t(RPM_W1);
    Differential_vel.RPM_W2 = uint16_t(RPM_W2);


    return Differential_vel;
}


inline void print_differential(uint16_t Apps_1, float Steering_dg, Velocity_struct Wheel_Velocity){
        double Pedal=(double(Apps_1)/65535)*100;
        double Pm1=(double(Wheel_Velocity.RPM_W1)/65535)*100;
        double Pm2=(double(Wheel_Velocity.RPM_W2)/65535)*100;

        printf("\n[Electronic Differential]  ==================================================#");
        printf("\n Pedal_Travel: %.2f%%,  Steering Angle : %.2f°\n", Pedal,Steering_dg);
        printf("\n Power_1: %.2f %%,  Power_2: %.2f %%\n", Pm1,Pm2);
        printf("#============================================================================#\n");

    }

inline void print_Control(uint16_t Apps_1,uint16_t Apps_2,uint16_t BSE, float Steering_dg, Velocity_struct Wheel_Velocity){
        double Pedal_1=(double(Apps_1)/65535)*100;
        double Pedal_2=(double(Apps_2)/65535)*100;
        double Break=(double(BSE)/65535)*100;
        double Pm1=(double(Wheel_Velocity.RPM_W1)/65535)*100;
        double Pm2=(double(Wheel_Velocity.RPM_W2)/65535)*100;

        printf("\n[Control]  ==================================================#");
        printf("\n Pedal_1: %.2f%%,  Pedal_2: %.2f%%, Break: %.2f%%\n", Pedal_1, Pedal_2, Break);
        printf("\n Pedal_Travel: %.2f%%,  Steering Angle : %.2f°\n", Pedal_1,Steering_dg);
        printf("\n Power_1: %.2f %%,  Power_2: %.2f %%\n", Pm1,Pm2);
        printf("#============================================================================#\n");
    }

inline void print_errors(bool ERROR_State,bool Error_APPS,bool Error_BPPC, bool Error_Motor){
    printf("\nState:  %d || ",ERROR_State);
    printf("Error_APPS:  %d || Error_BPPC:  %d || Error_Motor:  %d\n",Error_APPS, Error_BPPC, Error_Motor);
    }


#endif