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

// main() runs in its own thread in the OS

/*===================================== ADC PORTS (STM32 F746ZG) =====================================*/
#define Steering_WHEEL_PIN      PC_2
#define BSE_PIN                 PA_0
#define APPS1_PIN               PF_4
#define APPS2_PIN               PF_4
#define APPS_PIN_OUT            PA_5

/*===================================== ADC INPUT VOLTAGES  =====================================*/
//Steering Wheel Parameters
#define STEERING_VMIN           0.425   //Steering Wheel Sensors Minimum input voltage
#define STEERING_VMAX           2.825   //Steering Wheel Sensors Sensors Maximum input voltage

//BSE (Break System Enconder) Parameters
#define BSE_VMIN                0.3        //BSE Sensors Minimum input voltage
#define BSE_VMAX                3.0        //BSE Sensors Minimum input voltage

//APPS (Accelerator Pedal Position Sensor) Parameters
#define APPS1_VMIN              0.425      //APPS1 Minimum input voltage
#define APPS1_VMAX              3.021      //APPS1 Maximum input voltage

#define APPS2_VMIN              0.4        //APPS2 Minimum input voltage
#define APPS2_VMAX              2.9        //APPS2 Maximum input voltage

/*===================================== COMMUNICATION PORTS (STM32 F746ZG) =====================================*/

#define CAN1_TX                 PC_2
#define CAN1_RX                 PA_0

#define CAN2_TX                 PC_2    //
#define CAN2_RX                 PA_0    //

#define CAN_FREQUENCY           1e6     //CAN frequency in Hz (for CAN 2.0, its 1Mbit/s [1MHz])

//#define I2C_SCL                 PF_1
//#define I2C_SDA                 PF_0





/*===================================== Objetcs =====================================*/
// Communication
motor_can can1(PA_11, PA_12, CAN_FREQUENCY);

// Control Sensors
PedalSensor BSE(BSE_PIN, BSE_VMIN, BSE_VMAX);                                           // Break Pedal
PedalSensor APPS_1(APPS1_PIN ,APPS1_VMIN, APPS1_VMAX);                                  // Accel. Pedal 1
PedalSensor APPS_2(APPS2_PIN, APPS2_VMIN, APPS2_VMAX);                                  // Accel. Pedal 2
SteeringSensor Steering_sensor (Steering_WHEEL_PIN, STEERING_VMIN, STEERING_VMAX);      // Steering Wheel sensor

// Velocity Control
ControlSystem Motor_Control;    //

//MPU9250 Sensor (Gyroscope, Accelerometer, and Temperature)
//mpu


int main()
{
    /*================================== INITIALIZATION ==================================*/
    can1.set_CAN();

    uint16_t Apps1{0}, Apps2{0};    // Pedal Travel [0 to 16b]
    uint16_t Break_sensor{0};
    
    float Steering_dg{0};
    bool Error_flag;

    //Structs
    Rx_struct Inv1_data, Inv2_data;
    Velocity_struct Wheel_Velocity;

    /*================================== LOOP ==================================*/
    while (true) {

        //Read all Angle Sensors connected to the VCU
        Apps1 = APPS_1.read_pedal();
        Apps2 = APPS_1.read_pedal();
        Break_sensor = BSE.read_pedal();
        Steering_dg = Steering_sensor.read_angle();

        //Read MPU9250
        //MPU9250: Accelerometer

        //MPU9250: Angular Velocity         

        //MPU9250: Temperature       

    
    //------------------------------ Receive Data from Inverters ---------------------------//
        // Inv1_data=can1.receive_from_inverter();
        // Inv2_data=can1.receive_from_inverter_2();


    //-------------------------------- Control --------------------------------------------//
        // Wheel_Velocity = Motor_Control.control_test(Apps1, Apps2, Break_sensor, Steering_dg);
        Wheel_Velocity = Motor_Control.control_test(Apps1, Apps1, 0, 0);


    //----------------------------- Send data to Inverters ------------------------------//
        // can1.send_to_inverter_1(Wheel_Velocity.RPM_W1);
        // can1.send_to_inverter_2(Wheel_Velocity.RPM_W2);




    /*================================== Print/ Send Data ==================================*/

        //Datalogger

        //Telemetry
        // Telemetry_Test(Apps_1, Steering_dg, 10.0);
        // Telemetry_Print(Rx_apps);

        //Print voltage Read
        APPS_1.Voltage_print();
        Print_Velocity(Wheel_Velocity.RPM_W1);
        Print_Sensors(Apps1,  Apps2, Break_sensor, Steering_dg);
        print_all2();
        wait_us(10e5);

    }
}

