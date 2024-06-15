/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file is part of the car's VCU (VEHICLE CONTROL UNIT)
 * it contains the APPS plausibilty check, BSE plausibility check
 * as well the methods to read and map all the angle related sensors
 ***/

#ifndef _ADC_SENSORS_H_
#define _ADC_SENSORS_H_

#include "mbed.h"
#include <cstdint>
#include <time.h>

/*================================== SENSORS PARAMETERS ==================================*/
//General Parameters 
#define VREF_ADC        3.3     // ADC Reference (in volts), it scales the 16bit in that range
#define INPUT_MIN       4500    // Minimum 16bit Input the sensor should read
#define INPUT_MAX       48000   // Maximum 16bit Input the sensor should read

#define MAX_NOISE       0.1     // Expected Noise [Voltage variation] in ADC read

//Steering Wheel Parameters
#define Vol_ang_min     -80     //Minimum value for the Steering Wheel angle (Degrees)
#define Vol_ang_max     80      //Maximum value for the Steering Wheel angle (Degrees)

#define PEDAL_MIN       0          //Minimum value for the Accelerator Pedal angle (Degrees)
#define PEDAL_MAX       65535     //Maximum value for the Accelerator Pedal angle (Degrees)

//Ultility Functions
double millis();
long map(long Variable, float in_min, float in_max, float out_min, float out_max);
void Calibrate_ADC();

/*================================== CLASSES ==================================*/
//class for Acelleration Pedal, break Pedal, and Steering wheel
class angle_sensor{
    //Atributes
    protected:
    float Angle{0};            // Angle's value in Degree
    float Current_ADC{0};      // Last ADC voltage read [V]
    float Volt_min;         // Sensor's Minimum voltage measurement
    float Volt_max;         // Sensor's Maximum voltage measurement
    float Angle_min;        // Sensor's Minimum angle
    float Angle_max;        // Sensor's Maximum angle
    AnalogIn ADC_Pin;       // Input Pin in the MicroController

    //Methods
    public:
    float read_angle();             //
    uint16_t read_scaled_u16();     //
    bool Input_Error_Check();       // tests if ADC value (16bit) is within bounds of sensor
    void Voltage_print();           // prints raw voltage

    //Constructors:
    angle_sensor(PinName adc_Pin, float _volt_min,float _volt_max, float _angle_min,float _angle_max);
};

class PedalSensor: public angle_sensor{
    private:
    uint16_t Pedal_pos;     //Pedal Position [0% = 0 | 100% = 16b]
    
    public:
    // Methods:
    uint16_t read_pedal();  //Reads current Pedal Position
    
    // Constructors:
    PedalSensor(PinName adc_Pin, float _volt_min, float _volt_max);
};

class SteeringSensor: public angle_sensor{

    public:
    SteeringSensor(PinName adc_Pin, float _volt_min, float _volt_max);
};

/*======================================== Constructors ========================================*/

//Angle Sensor
inline angle_sensor::angle_sensor(PinName adc_Pin, float _volt_min,float _volt_max, float _angle_min,float _angle_max)
:ADC_Pin{adc_Pin,VREF_ADC}, Volt_min{_volt_min},Volt_max{_volt_max}, Angle_min{_angle_min}, Angle_max{_angle_max}{
    ADC_Pin.set_reference_voltage(3.3);
};

// Pedal Sensor
inline PedalSensor::PedalSensor(PinName adc_Pin, float _volt_min, float _volt_max)
    :angle_sensor{adc_Pin, _volt_min, _volt_max, 0, 100}{}


//Steering Wheel Sensor
inline SteeringSensor::SteeringSensor(PinName adc_Pin, float _volt_min, float _volt_max)
    :angle_sensor{adc_Pin, _volt_min, _volt_max, Vol_ang_min, Vol_ang_max}{}



/*======================================== Methods ========================================*/

// Reads the ADC pin and returns the angle value in degrees 
inline float angle_sensor:: read_angle(){
    float New_ADC = ADC_Pin.read_voltage();
    
    // If variation is bigger than the expected noise, updates measurement 
    if(abs(New_ADC - Current_ADC)> MAX_NOISE){
        Current_ADC= New_ADC;
        Angle= map(Current_ADC, Volt_min, Volt_max, Angle_min, Angle_max);
    }
    return Angle;
}

// Reads the Pedal travel [0% = 0 | 100% = 16b]
inline uint16_t PedalSensor:: read_pedal(){
    float New_ADC = ADC_Pin.read_voltage();
    
    // If variation is bigger than the expected noise, updates measurement 
    if(abs(New_ADC - Current_ADC)> MAX_NOISE){
        Current_ADC= New_ADC;
        Pedal_pos= map(Current_ADC, Volt_min, Volt_max, PEDAL_MIN, PEDAL_MAX);
    }
    return Pedal_pos;
}

// Probably useless
inline uint16_t angle_sensor:: read_scaled_u16(){
    uint16_t u1= map( ADC_Pin.read_voltage(), Volt_min, Volt_max, 0, 65535);
    return u1;
}

// 
inline void angle_sensor:: Voltage_print(){
    uint16_t Voltage_16bit=ADC_Pin.read_u16();
    printf("\n[VCU] ADC: Voltage_Read[16bit]: %d , Voltage[V]: %.2f V  \n",Voltage_16bit, ADC_Pin.read_voltage() );    
}


/*======================================== Auxiliar functions ========================================*/
//time passed (in ms) since the program first started
inline double millis(){
    using namespace std::chrono;
    auto now_us = time_point_cast<microseconds>(Kernel::Clock::now());  //time of referece= program's begin
    long micros = now_us.time_since_epoch().count();                    //time (in us) since the reference 
    return micros / 1000.0;                                             //turns time passed from us to ms
}

//Maps the ADC 16bit voltage read into the angle's range 
inline long map (long Variable, float in_min, float in_max, float out_min, float out_max) {
    long Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return Mapped_Variable;
}


inline void Calibrate_ADC(){

}



#endif