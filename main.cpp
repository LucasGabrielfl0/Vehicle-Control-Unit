/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file contains the Integrated VCU (VEHICLE CONTROL UNIT) that unites the
 * CAN communication with Inverters, the analog communication with all the sensors and 
 * the Electronic Differential Closed Loop control algorithm
 ***/

#include "mbed.h"
#include <cstdint>
#include "Powertrain.h"
#include "rtos.h"
// main() runs in its own thread in the OS

/*===================================== ADC PORTS (STM32 F746ZG) =====================================*/
#define Steering_WHEEL_PIN      PC_2
#define BSE_PIN                 PB_1
#define APPS1_PIN               PF_4
#define APPS2_PIN               PF_4
#define APPS_PIN_OUT            PA_5

/*===================================== ADC INPUT VOLTAGES  =====================================*/
//Steering Wheel Parameters
#define STEERING_VMIN           0.325       // Steering Wheel Sensors Minimum input voltage
#define STEERING_VMAX           3.0         // Steering Wheel Sensors Sensors Maximum input voltage

//BSE (Break System Enconder) Parameters
#define BSE_VMIN                0.3         // BSE Sensors Minimum input voltage
#define BSE_VMAX                3.0         // BSE Sensors Minimum input voltage

//APPS (Accelerator Pedal Position Sensor) Parameters
#define APPS1_VMIN              0.28        // APPS1 Minimum input voltage
#define APPS1_VMAX              3.0         // APPS1 Maximum input voltage

#define APPS2_VMIN              0.28        // APPS2 Minimum input voltage
#define APPS2_VMAX              3.0         // APPS2 Maximum input voltage


/*===================================== COMMUNICATION PORTS (STM32 F746ZG) =====================================*/
// CAN
#define CAN1_RX                 PD_0        // CAN 1: General communication in the VCU
#define CAN1_TX                 PD_1        //

#define CAN2_RX                 PB_5        // CAN 2: Communication with the motor controller
#define CAN2_TX                 PB_6        // 

#define CAN1_FREQUENCY           1e6        // CAN 1 frequency in Hz
#define CAN2_FREQUENCY           1e6        // CAN 2 frequency in Hz (for CAN 2.0, its 1MHz [1Mbit/s])


/*===================================== Objetcs =====================================*/
// Communication
CAN CAN_VCU(CAN1_RX, CAN1_TX, CAN2_FREQUENCY);                                          // VCU general CAN
MotorCAN CAN_Motor(CAN2_RX, CAN2_TX, CAN2_FREQUENCY);                                   // Motor/Inverter CAN (Hv)

// Pedal/Steering wheel Sensors
PedalSensor BSE(BSE_PIN, BSE_VMIN, BSE_VMAX);                                           // Break Pedal sensor
PedalSensor APPS_1(APPS1_PIN ,APPS1_VMIN, APPS1_VMAX);                                  // Accel. Pedal sensor 1
PedalSensor APPS_2(APPS2_PIN, APPS2_VMIN, APPS2_VMAX);                                  // Accel. Pedal sensor 2
SteeringSensor Steering_sensor (Steering_WHEEL_PIN, STEERING_VMIN, STEERING_VMAX);      // Steering Wheel sensor

// Velocity Control System
ControlSystem Motor_Control(APPS_1, APPS_2, BSE, Steering_sensor);                      // Motor Control
ControlSystem Motor_1(APPS_1, APPS_2, BSE, Steering_sensor);            // Motor Control
ControlSystem Motor_2(APPS_1, APPS_2, BSE, Steering_sensor);            // Motor Control

//*===================================== Aux Functions =====================================*//
void Run_Control();     // Routine to Read/Send control signal to the motor controllers
void getData();         //       


/*===================================== Global  Variables =====================================*/
// Structs
RxStruct Inv1_data, Inv2_data;     // Structs for data received from each controller

// Timers
Ticker timer_1ms;                   // Timed Interruprt for every 1ms

int main()
{
    /*================================== INITIALIZATION ==================================*/
    // Comm. system Initializaion
    CAN_Motor.set_CAN();                    // CAN communication with both Motor Controllers



    // Timers and Interruprts
    timer_1ms.attach(&Run_Control, 1ms);    // Runs Control system every 1 ms

    /*================================== LOOP ==================================*/
    while (true) {
        /* does something?*/
        getData();       
    }
}




/*========================================== POWERTRAIN ==========================================*/
/* Gets data from the Inverters*/
void getData(){
    Inv1_data = CAN_Motor.receive_from_inverter_1();      // Get data from Motor controller 1
    Inv2_data = CAN_Motor.receive_from_inverter_2();      // Get data from Motor controller 1
    // Telemetry_Print(Inv1_data);                         // Monitoring system
    
}

/* Open Loop Control*/
void OpenLoop(void * params){
    // Control System 
    // Dc_Motor = Motor_Control.control();                 // Get Control Signal [Duty Cycle]

    // Electronic Differential

    // Send data to Inverters 
    // CAN_Motor.send_to_inverter_1(Dc_Motor.Dc_1, 0);     // Send control Signal to Controller 1
    // CAN_Motor.send_to_inverter_2(Dc_Motor.Dc_2, 0);     // Send control Signal to Controller 2
}

/* Closed Loop Control*/
void Run_Control3(void * params){
    uint16_t Dc_Motor[2]{0};
    float Wm_ref[2]{0};
    float Wm_M1, Wm_M2;

    // Electronic Differential [Setpoint based on the Apps and Steering Wheel]
    ElectronicDifferential(1,1,1,Wm_ref);

    // Control System
    Dc_Motor[0]= Motor_1.control(Wm_M1, Wm_ref[0]);
    Dc_Motor[1]= Motor_2.control(Wm_M2, Wm_ref[1]);
    
    // Send data to Inverters 
    CAN_Motor.send_to_inverter_1(Dc_Motor[0], 0);     // Send control Signal to Controller 1
    CAN_Motor.send_to_inverter_2(Dc_Motor[1], 0);     // Send control Signal to Controller 2
}


/* Safety Check*/
void PlausibilityCheck(void * params){
    
}

