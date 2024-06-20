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

// main() runs in its own thread in the OS

/*===================================== ADC PORTS (STM32 F746ZG) =====================================*/
#define Steering_WHEEL_PIN      PC_2
#define BSE_PIN                 PB_1
#define APPS1_PIN               PF_4
#define APPS2_PIN               PF_4
#define APPS_PIN_OUT            PA_5

/*===================================== ADC INPUT VOLTAGES  =====================================*/
//Steering Wheel Parameters
#define STEERING_VMIN           0.325   //Steering Wheel Sensors Minimum input voltage
#define STEERING_VMAX           3.0   //Steering Wheel Sensors Sensors Maximum input voltage

//BSE (Break System Enconder) Parameters
#define BSE_VMIN                0.3        //BSE Sensors Minimum input voltage
#define BSE_VMAX                3.0        //BSE Sensors Minimum input voltage

//APPS (Accelerator Pedal Position Sensor) Parameters
#define APPS1_VMIN              0.28      //APPS1 Minimum input voltage
#define APPS1_VMAX              3.0      //APPS1 Maximum input voltage

#define APPS2_VMIN              0.28        //APPS2 Minimum input voltage
#define APPS2_VMAX              3.0        //APPS2 Maximum input voltage


/*===================================== COMMUNICATION PORTS (STM32 F746ZG) =====================================*/
#define CAN1_RX                 PD_0
#define CAN1_TX                 PD_1

#define CAN2_RX                 PB_5    //
#define CAN2_TX                 PB_6   //

#define CAN1_FREQUENCY           1e6     //CAN frequency in Hz
#define CAN2_FREQUENCY           1e6     //CAN frequency in Hz (for CAN 2.0, its 1Mbit/s [1MHz])

//#define I2C_SCL                 PF_1
//#define I2C_SDA                 PF_0





/*===================================== Objetcs =====================================*/
// Communication
CAN CAN_VCU(CAN1_RX, CAN1_TX, CAN2_FREQUENCY);                                          // VCU general CAN
MotorCAN CAN_Motor(CAN2_RX, CAN2_TX, CAN2_FREQUENCY);                                   // Motor/Inverter CAN (Hv)

// Pedal/Steering wheel Sensors
PedalSensor BSE(BSE_PIN, BSE_VMIN, BSE_VMAX);                                           // Break Pedal sensor
PedalSensor APPS_1(APPS1_PIN ,APPS1_VMIN, APPS1_VMAX);                                  // Accel. Pedal sensor 1
PedalSensor APPS_2(APPS2_PIN, APPS2_VMIN, APPS2_VMAX);                                  // Accel. Pedal sensor 2
SteeringSensor Steering_sensor (Steering_WHEEL_PIN, STEERING_VMIN, STEERING_VMAX);      // Steering Wheel sensor


// Velocity Control system
ControlSystem Motor_Control;    //

//MPU9250 Sensor (Gyroscope, Accelerometer, and Temperature)
//mpu


int main()
{
    /*================================== INITIALIZATION ==================================*/
    // Comm. system Initializaion
    CAN_Motor.set_CAN();


    // useless, debug
    uint16_t Apps1{0}, Apps2{0};    // Pedal Travel [0 to 16b]
    uint16_t Break_sensor{0};
    float Steering_dg{0};

    //Structs
    Rx_struct Inv1_data, Inv2_data;     // Data from Motor 1 and 2
    Velocity_struct Wheel_Velocity;     // RPM Velocity in each wheel

    /*================================== LOOP ==================================*/
    while (true) {

    //------------------------------ Receive Data from Inverters ---------------------------//
        // Inv1_data=can1.receive_from_inverter();
        // Inv2_data=can1.receive_from_inverter_2();
        // Motor_Control.Motor_Error_Check(Inv1_data, Inv2_data);

    //-------------------------------- Control System --------------------------------------------//
        Wheel_Velocity = Motor_Control.control(APPS_1, APPS_2, BSE, Steering_sensor);
        

        // debug:
        // Wheel_Velocity = Motor_Control.control(APPS_1, APPS_2, BSE);
        // Wheel_Velocity = Motor_Control.control(APPS_1, Steering_sensor);

    //----------------------------- Send data to Inverters ------------------------------//
        // can1.send_to_inverter_1(Wheel_Velocity.RPM_W1);
        // can1.send_to_inverter_2(Wheel_Velocity.RPM_W2);




    /*================================== Print/ Send Data ==================================*/

        //Datalogger

        //Telemetry
        // Telemetry_Test(Apps_1, Steering_dg, 10.0);
        // Telemetry_Print(Rx_apps);

        wait_us(10e5);

    }
}

