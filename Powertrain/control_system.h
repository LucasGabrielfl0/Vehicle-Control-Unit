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
struct VelocityStruct{
    uint16_t RPM_W1;    // [LEFT WHEEL ]
    uint16_t RPM_W2;    // [RIGHT WHEEL]
};

struct DcStruct{
    uint16_t Dc_1;    // [LEFT WHEEL ]
    uint16_t Dc_2;    // [RIGHT WHEEL]
};

/*======================================== Aux Functions ========================================*/
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

    public:
    // Methods
    void APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2);
    void BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val);
    void Motor_Error_Check(RxStruct Inverter_1, RxStruct Inverter_2);
    
    // void Error_Check();
    DcStruct control();
    

    // Constructors
    public:
    ControlSystem(PedalSensor APPS_1, PedalSensor APPS_2, PedalSensor BSE_sensor, SteeringSensor Steering_sensor);

};

/*======================================== Constructors ========================================*/
inline ControlSystem::ControlSystem(PedalSensor _apps_1, PedalSensor _apps_2, PedalSensor _bse_sensor, SteeringSensor _steering_sensor)
:APPS_1{_apps_1}, APPS_2{_apps_2},BSE_sensor{_bse_sensor}, Steering_sensor{_steering_sensor} {};


/*======================================== VELOCITY CONTROL ==================================*/
// Apps_16b= Accel. Pedal signal scaled [0 = Min travel | 65535 = Max travel]
inline DcStruct ControlSystem::control(){
    VelocityStruct Velocity_Wheels;
    DcStruct Dc_Motor;
    
    uint16_t Apps_1 = APPS_1.read_pedal();
    uint16_t Apps_2 = APPS_1.read_pedal();
    uint16_t BSE = BSE_sensor.read_pedal();
    float Steering_dg= Steering_sensor.read_angle();

    // Error Check
    APPS_Error_check(Apps_1, Apps_2);                                   // Apps Error Check
    BSE_Error_check(Apps_1, BSE);                                       // Brake Implausibility Check

    ERROR_State= Error_APPS || Error_BPPC || Error_Motor;

    if(ERROR_State){
        Velocity_Wheels.RPM_W1 = 0;
        Velocity_Wheels.RPM_W2 = 0;
    }
    else{
        // get Differential
        Velocity_Wheels = get_Differential(Steering_dg, Apps_1);
    }
    // print_Control(Apps_1, Apps_2, BSE, Steering_dg, Velocity_Wheels);
    // print_errors(ERROR_State,Error_APPS,Error_BPPC,Error_Motor);

    //Open Loop Control
    Dc_Motor.Dc_1 = Velocity_Wheels.RPM_W1;
    Dc_Motor.Dc_2 = Velocity_Wheels.RPM_W2;
    
    // Return Velocity in each wheel
    return Dc_Motor;
}

/*================================== ACCELERATION PEDAL PLAUSIBILITY CHECK ==================================*/
// Every 100ms, checks if there's a discrepancy bigger than 10%
inline void ControlSystem::APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2){
    // Checks 10% discrepancy
    if ( abs(Apps_1 - Apps_2) > (0.1 * max(Apps_1, Apps_2)) ){
        // if there's discrepancy, sets flag
        if(AppsError_flag == 0) {
            AppsError_flag = 1;
        }
        else{
            Error_APPS= 1;
            printf("APPS: ERROR DETECTED");
        }
        
        // Checks if the error continues after 100ms and sets error
        if(current_ms() - Error_Start_Time > 100) { 
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

/*====================================== BRAKE PEDAL PLAUSIBILITY CHECK ======================================*/
// Checks if the Accel. and Brake Were both pressed at the same time
inline void ControlSystem::BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val){    
    //If APPS >= 25% of pedal travel and Brake is pressed, stops the car 
    if ( (Apps_val >= 0.25*PEDAL_MAX) && (Brake_val >= 0.03*PEDAL_MAX) ){
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
inline void ControlSystem::Motor_Error_Check(RxStruct Inverter_1, RxStruct Inverter_2){
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
inline VelocityStruct get_Differential(float Steering_dg, uint16_t Wv_rpm){
    int64_t RPM_W1,RPM_W2;
    float Ack_rad;
    float del_W;
    VelocityStruct Differential_vel;

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

    //if Underflow, sets min
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


inline void print_differential(uint16_t Apps_1, float Steering_dg, VelocityStruct Wheel_Velocity){
        double Pedal=(double(Apps_1)/65535)*100;
        double Pm1=(double(Wheel_Velocity.RPM_W1)/65535)*100;
        double Pm2=(double(Wheel_Velocity.RPM_W2)/65535)*100;

        printf("\n[Electronic Differential]  ==================================================#");
        printf("\n Pedal_Travel: %.2f%%,  Steering Angle : %.2f°\n", Pedal,Steering_dg);
        printf("\n Power_1: %.2f %%,  Power_2: %.2f %%\n", Pm1,Pm2);
        printf("#============================================================================#\n");

    }

inline void print_Control(uint16_t Apps_1,uint16_t Apps_2,uint16_t BSE, float Steering_dg, VelocityStruct Wheel_Velocity){
        double Pedal_1=(double(Apps_1)/65535)*100;
        double Pedal_2=(double(Apps_2)/65535)*100;
        double Brake=(double(BSE)/65535)*100;
        double Pm1=(double(Wheel_Velocity.RPM_W1)/65535)*100;
        double Pm2=(double(Wheel_Velocity.RPM_W2)/65535)*100;

        printf("\n[Control]  ==================================================#");
        printf("\n Pedal_1: %.2f%%,  Pedal_2: %.2f%%, Brake: %.2f%%\n", Pedal_1, Pedal_2, Brake);
        printf("\n Pedal_Travel: %.2f%%,  Steering Angle : %.2f°\n", Pedal_1,Steering_dg);
        printf("\n Power_1: %.2f %%,  Power_2: %.2f %%\n", Pm1,Pm2);
        printf("#============================================================================#\n");
    }

inline void print_errors(bool ERROR_State,bool Error_APPS,bool Error_BPPC, bool Error_Motor){
    printf("\nState:  %d || ",ERROR_State);
    printf("Error_APPS:  %d || Error_BPPC:  %d || Error_Motor:  %d\n",Error_APPS, Error_BPPC, Error_Motor);
    }


#endif