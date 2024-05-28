/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file contains the Integrated VCU (VEHICLE CONTROL UNIT) that unites the
 * CAN communication with Inverters, the analog communication with all the sensors and 
 * the Electronic Differential Closed Loop control algorithm
 ***/

#include "mbed.h"
#include "motor_can.h"
#include "control.h"
#include "angle_sensor.h"
#include <cstdint>
// main() runs in its own thread in the OS

/*===================================== PORTS (STM32 F746ZG)=====================================*/
#define Steering_WHEEL_PIN      PC_2
#define BSE_PIN                 PA_0
#define APPS1_PIN               PF_4
#define APPS2_PIN               PF_4
#define APPS_PIN_OUT            PA_5

/*===================================== COMMUNICATION (STM32 F746ZG)=====================================*/

#define CAN1_TX                 PC_2
#define CAN1_RX                 PA_0

#define CAN2_TX                 PC_2    //
#define CAN2_RX                 PA_0    //

#define CAN_FREQUENCY           1e6     //CAN frequency in Hz (for CAN 2.0, its 1Mbit/s [1MHz])

//#define I2C_SCL                 PF_1
//#define I2C_SDA                 PF_0





/*===================================== Objetcs =====================================*/
//Communication
motor_can can1(PA_11, PA_12, 1e6);

//Angle Sensors
BSE_Sensor BSE (BSE_PIN);
APP_Sensors APPS (APPS1_PIN ,APPS2_PIN, APPS_PIN_OUT);
Steering_Wheel_Sensor Steering_sensor (Steering_WHEEL_PIN);

//MPU9250 Sensor (Gyroscope, Accelerometer, and Temperature)
//mpu


int main()
{
    /*================================== INITIALIZATION ==================================*/
    can1.set_CAN();

    //Constant Variables
    float BSE_dg{0} ,Steering_dg{0};
    
    //Structs
    Rx_struct Inv1_data, Inv2_data;
    APPS_struct APPS_dg;
    Velocity_struct RPM_set;

    /*================================== LOOP ==================================*/
    while (true) {
        //Read all Angle Sensors connected to the VCU
        APPS_dg= APPS.read_APPS();
        BSE_dg = BSE.read();
        Steering_dg = Steering_sensor.read();
        
        //Read MPU9250
        //MPU9250: Accelerometer


        //MPU9250: Angular Velocity         


        //MPU9250: Temperature       


        //Velocity Sensor 
    
        //Receive Data from Inverters
        // Inv1_data=can1.receive_from_inverter();
        // Inv2_data=can1.receive_from_inverter_2();


        //Control
        wait_us(5e5);

        //Send data to Inverters
        // can1.send_to_inverter_1(1,RPM_set.RPM_W1,1);
        // can1.send_to_inverter_2(RPM_set.RPM_W2,1,1);
        

        //Datalogger

        APPS.APPS1.Voltage_print();

        //Telemetry

        // Print Sensors:
        // printf("\n ========");
        // printf("Apps1: %.2f, BSE: %.2f", APPS_dg.s1 ,BSE_dg);
        // printf("\n ========");
        //print Received Data
        //can1.Print_Datafields();

    }
}

