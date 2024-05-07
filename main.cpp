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
#include "angle_sensor.h"
#include <cstdint>
// main() runs in its own thread in the OS

/*===================================== PORTS (STM32 F746ZG)=====================================*/
#define Steering_WHEEL_PIN      PC_2
#define BSE_PIN                 PA_0
#define APPS1_PIN               PF_4
#define APPS2_PIN               PF_4
#define APPS_PIN_OUT            PA_5


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
    const uint16_t Current_inv1{39}, Current_inv2{30};
    const uint16_t RPM_inv1{9000}, RPM_inv2{9000};
    float BSE_dg{0} ,Steering_dg{0};
    
    //Structs
    Rx_struct Inv1_data, Inv2_data;
    APPS_struct APPS_dg;
    Wref_struct W_ref;

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


        //Send data to Inverters
        can1.send_to_inverter(RPM_inv1, W_ref.W1, Current_inv1);
        can1.send_to_inverter_2(RPM_inv2, W_ref.W2, Current_inv2);

    
        //Receive Data from Inverters
        Inv1_data=can1.receive_from_inverter();
        Inv2_data=can1.receive_from_inverter_2();


        //Calculates differential

        //Send data to Inverters
        can1.send_to_inverter(RPM_inv1, W_ref.W1, Current_inv1);
        can1.send_to_inverter_2(RPM_inv2, W_ref.W2, Current_inv2);
        

        //Datalogger


        //Telemetry


        //print Received Data
        can1.Print_Datafields();

    }
}

