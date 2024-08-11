/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file is part of the car's VCU (VEHICLE CONTROL UNIT)
 * it contains the APPS plausibilty check, BSE plausibility check
 * as well the methods to read and map all the angle related sensors connected to the adc
 ***/

#ifndef _ADC_SENSORS_H_
#define _ADC_SENSORS_H_

#include "mbed.h"
#include <cstdint>
#include <time.h>

/*================================== SENSORS PARAMETERS ==================================*/
//General ADC Parameters 
#define VREF_ADC                3.3         // ADC Reference (in volts), it scales the 16bit in that range
#define INPUT_MIN               0.3         // Minimum 16bit Input the sensor should read
#define INPUT_MAX               3.2         // Maximum 16bit Input the sensor should read

#define MAX_NOISE               0.09        // Expected Noise [Voltage variation] in ADC read
#define SATURATION_VOLTAGE      0.1         // Saturation 

//Steering Wheel Parameters
#define Vol_ang_min             -80         //Minimum value for the Steering Wheel angle (Degrees)
#define Vol_ang_max             80          //Maximum value for the Steering Wheel angle (Degrees)

// Pedal Parameters
#define PEDAL_MIN               0           //Minimum value for the Accelerator Pedal angle (Degrees)
#define PEDAL_MAX               65535       //Maximum value for the Accelerator Pedal angle (Degrees)


//Ultility Functions
unsigned long current_ms();
float map(float Variable, float in_min, float in_max, float out_min, float out_max);
uint16_t map_u16 (float Variable, float in_min, float in_max, uint16_t out_min, uint16_t out_max);
void Calibrate_ADC();

/*================================== Angle Sensors ==================================*/
//class for Acelleration Pedal, break Pedal, and Steering wheel
class angle_sensor{
    //Atributes
    protected:
    float Angle{0};             // Angle's value in Degree
    float Current_ADC{0};       // Last ADC voltage read [V]
    float Volt_min;             // Sensor's Minimum voltage measurement
    float Volt_max;             // Sensor's Maximum voltage measurement
    float Angle_min;            // Sensor's Minimum angle
    float Angle_max;            // Sensor's Maximum angle

    bool Circuit_ERROR{0};      // Flag for short or open circuit

    AnalogIn ADC_Pin;           // Input Pin in the MicroController

    //Methods
    public:
    float read_angle();                             // returns scaled angle
    bool Circuit_Error_Check(float voltage_in);     // tests if ADC voltage is within bounds of sensor
    bool get_circuit_error();                       // Returns Circuit Error
    void Voltage_print();                           // prints pin's voltage


    //Constructors:
    angle_sensor(PinName adc_Pin, float _volt_min,float _volt_max, float _angle_min,float _angle_max);
};

/*====================================== Pedal Sensors ======================================*/
// Pedal Sensor Class (for the Accel. and break Pedals)
class PedalSensor: public angle_sensor{
    private:
    uint16_t Pedal_pos;     // Pedal Travel [0% = 0 | 100% = 16b]
    
    public:
    // Methods:
    uint16_t read_pedal();  // Reads current Pedal Position
    void Voltage_print();   // Print ADC voltage read and Pedal Travel
    
    // Constructors:
    PedalSensor(PinName adc_Pin, float _volt_min, float _volt_max);
};

/*================================== Steering Wheel Sensor ==================================*/
// Steering Wheel Sensor Class
class SteeringSensor: public angle_sensor{

    public:
    SteeringSensor(PinName adc_Pin, float _volt_min, float _volt_max);
};



#endif