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
#include "PowertrainLib.h"
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


/*===================================== CONTROL SYSTEM =====================================*/
const float     Kp      = 1.000;          // Proporcional Gain         
const float     Ki      = 1.000;          // Integral Gain
const float     Kd      = 1.000;          // Derivative Gain
const uint16_t  Ts_ms   = 1;              // Sample time in ms        


//*===================================== Aux Functions =====================================*//
void RunControl();     // Send control signal to the motor controllers
void getData();        // Get motor Data via CAN



/*=================================================== Objetcs =====================================================*/
// Communication
CAN CAN_VCU(CAN1_RX, CAN1_TX, CAN2_FREQUENCY);                                          // VCU general CAN
MotorCAN CAN_Motor(CAN2_RX, CAN2_TX, CAN2_FREQUENCY);                                   // Motor/Inverter CAN (Hv)

// Pedal and Steering wheel Sensors
PedalSensor BSE(BSE_PIN, BSE_VMIN, BSE_VMAX);                                           // Break Pedal sensor
PedalSensor APPS_1(APPS1_PIN ,APPS1_VMIN, APPS1_VMAX);                                  // Accel. Pedal sensor 1
PedalSensor APPS_2(APPS2_PIN, APPS2_VMIN, APPS2_VMAX);                                  // Accel. Pedal sensor 2
SteeringSensor Steering_sensor (Steering_WHEEL_PIN, STEERING_VMIN, STEERING_VMAX);      // Steering Wheel sensor

// Velocity Control System
ControlSystem Motor(Kp,Ki,Kd,Ts_ms);            // Motor Control

// test only:
ControlSystem Motor_1(Kp,Ki,Kd,Ts_ms);            // Motor Control
ControlSystem Motor_2(Kp,Ki,Kd,Ts_ms);            // Motor Control



/*===================================== Global  Variables =====================================*/
// Structs
RxStruct Inv1_data, Inv2_data;     // Structs for data received from each controller

// Timers // trash
Ticker timer_1ms;                   // Timed Interruprt for every 1ms

int main()
{
    /*====================================== INITIALIZATION ======================================*/
    // Comm. system Initializaion
    CAN_Motor.set_CAN();                    // CAN communication with both Motor Controllers


    // Timers and Interruprts
    // timer_1ms.attach(&Run_Control, 1ms);    // Runs Control system every 1 ms
    // RTOS:

    /*=========================================== LOOP ===========================================*/
    while (true) {
        /* does something?*/
        getData();
        wait_us(500);
    }
}




/*========================================== POWERTRAIN ==========================================*/
/* Gets data from the Inverters: Every 1 ms*/
void ReadCAN(void const * args){
    Inv1_data = CAN_Motor.receive_from_inverter_1();      // Get data from Motor controller 1
    Inv2_data = CAN_Motor.receive_from_inverter_2();      // Get data from Motor controller 1
    // Telemetry_Print(Inv1_data);                         // Monitoring system
    
}

/* Open Loop Control*/
void OpenLoop(void const * args){
    uint16_t Dc_Motor[2]{0};                                // Control Signal
    uint16_t apps = APPS_1.read_pedal();                    // Acc. Pedal
    float Steering_dg = Steering_sensor.read_angle();       // Steering Wheel

    // Open Loop without Differential
    Dc_Motor[0]= apps;
    Dc_Motor[1]= apps;

    // Open Loop with Electronic Differential [Control]
    // OpenLoopDifferential(Steering_dg, apps, Dc_Motor);

    // Send data to Inverters 
    CAN_Motor.send_to_inverter_1(Dc_Motor[0], 0);     // Send control Signal to Controller 1
    CAN_Motor.send_to_inverter_2(Dc_Motor[1], 0);     // Send control Signal to Controller 2
}


/* Closed Loop Control: Every 1 ms*/
void Run_Control(void const * args){
    uint16_t Dc_Motor[2]{0};            // DutyCycle [0 to 16b]
    uint16_t RPM_c[2]{0};               // Current motor Velocity in RPM
    uint16_t RPM_setpoint[2]{0};        // Current motor Velocity in RPM
    
    uint16_t Apps         =  APPS_1.read_pedal();                   // Read Acc. Pedal
    uint16_t Brake_val    =  BSE.read_angle();                      // Read Brae Pedal
    float    Steering_dg  =  Steering_sensor.read_angle();          // Read Steering Wheel sensor

    // Current Velocity value
    RPM_c[0] = Inv1_data.RPM;
    RPM_c[1] = Inv2_data.RPM;

    // Setpoint
    RPM_setpoint[0] = 1000;
    RPM_setpoint[1] = 1000;


    // Control System
    Dc_Motor[0]= Motor_1.control(RPM_c[0], RPM_setpoint[0]);
    Dc_Motor[1]= Motor_2.control(RPM_c[1], RPM_setpoint[1]);
    
    // If Break
    if( BSE.get_circuit_error() || BSE_Error_check(Apps, Brake_val) ){
        printf("CIRCUIT ERROR DETECTED");
        Dc_Motor[0]= 0;
        Dc_Motor[1]= 0;
    }
    
    // Send data to Inverters 
    CAN_Motor.send_to_inverter_1(Dc_Motor[0], 0);     // Send control Signal to Controller 1
    CAN_Motor.send_to_inverter_2(Dc_Motor[1], 0);     // Send control Signal to Controller 2
}



/* Closed Loop Control: Every 1 ms*/
void Run_Control_dif(void const * args){
    uint16_t Dc_Motor[2]{0};           // DutyCycle [0 to 16b]
    uint16_t RPM_c[2]{0};              // Current motor Velocity in RPM
    
    uint16_t Apps         =  APPS_1.read_pedal();                   // Read Acc. Pedal
    uint16_t Brake_val    =  BSE.read_angle();                      // Read Brae Pedal
    float    Steering_dg  =  Steering_sensor.read_angle();          // Read Steering Wheel sensor

    // Current Velocity value [RPM]
    RPM_c[0] = Inv1_data.RPM;
    RPM_c[1] = Inv2_data.RPM;

    // Control System
    Motor.DifferentialControl(Steering_dg, Apps, RPM_c, Dc_Motor);

    // If Break
    if( BSE.get_circuit_error() || BSE_Error_check(Apps, Brake_val) ){
        printf("CIRCUIT ERROR DETECTED");
        Dc_Motor[0]= 0;
        Dc_Motor[1]= 0;
    }
    
    // Send data to Inverters 
    CAN_Motor.send_to_inverter_1(Dc_Motor[0], 0);     // Send control Signal to Controller 1
    CAN_Motor.send_to_inverter_2(Dc_Motor[1], 0);     // Send control Signal to Controller 2
}

/* Safety Check: Every 20 ms*/
void PlausibilityCheck(void * params){
    bool Error_State{0};

    /* Read all sensors*/
    uint16_t Apps_1 = APPS_1.read_pedal();
    uint16_t Apps_2 = APPS_2.read_pedal();
    uint16_t Brake_val = BSE.read_pedal();
    float Steering_dg = Steering_sensor.read_angle();

    // Check for errors
    if( APPS_Error_check(Apps_1, Apps_2) ){
        printf("APPS ERROR DETECTED");
        Error_State =1;
    }
    if( BSE_Error_check(Apps_1, Brake_val) ){
        printf("BSE ERROR DETECTED");
        Error_State =1;
    }

    /* If Error is Detected*/
    if(Error_State){
        Motor.shutdown = true;                  // Shutsdown Both motors
        CAN_Motor.send_to_inverter_1(0, 0);     // Send control Signal to Controller 1
        CAN_Motor.send_to_inverter_2(0, 0);     // Send control Signal to Controller 2
    
    } else{
        Motor.shutdown = false;

    }
}

