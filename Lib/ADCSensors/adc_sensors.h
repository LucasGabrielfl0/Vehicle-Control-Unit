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


//Ultility Functions
unsigned long current_ms();
float map(float Variable, float in_min, float in_max, float out_min, float out_max);
uint16_t map_u16 (float Variable, float in_min, float in_max, uint16_t out_min, uint16_t out_max);
void Calibrate_ADC();

// Safety Checks
bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2, uint8_t* Error_Count);  // Acc. pedal Plausibility
bool BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val, uint8_t* Error_BPPC);   // Brake pedal Plausibility

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


/*                                  */
// #include "adc_sensors.h"

// /*================================== ACCELERATION PEDAL PLAUSIBILITY CHECK ==================================*/
// // Every 10ms, checks if there's a discrepancy bigger than 10% lastting more then 100 ms
// bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2, uint8_t* Error_Count){
//     uint8_t Error_Counter = *Error_Count;

//     // Checks 10% discrepancy
//     if ( abs(Apps_1 - Apps_2) > (0.1 * max(Apps_1, Apps_2)) ){
//         // if there's discrepancy, adds in counter
//         Error_Counter++;
//     }
//     else { 
//         Error_Counter= 0;      //if error ceased resets counter
//     }
    
//     // If Implausibility lasts 4 iterations (80ms - 100 ms response time), Sets Error
//     if(Error_Counter >=4){
//         printf("APPS ERROR DETECTED");
//         Error_Counter =4;
//         return 1;
//     }
//     else{
//         return 0;
//     }

// }

// /*====================================== BRAKE PEDAL PLAUSIBILITY CHECK ======================================*/
// // Checks if the Accel. and Brake Were both pressed at the same time
// bool BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val, bool Error_BPPC){    
//     //If APPS >= 25% of pedal travel and Brake is pressed, stops the car 
//     if ( (Apps_val >= 0.25*PEDAL_MAX) && (Brake_val >= 0.02*PEDAL_MAX) ){
//         Error_BPPC = 1;
//     }

//     // Error only Stops if APPS goes below 5% of pedal travel
//     if(Apps_val < 0.05*PEDAL_MAX){
//         Error_BPPC = 0;
//     }

//     if(Error_BPPC){
//         printf("BSE: ERROR IN PROGRESS");
//     }
//     return Error_BPPC;
// }





// /*================================== Angle Sensors ==================================*/
// // Constructors
// inline angle_sensor::angle_sensor(PinName adc_Pin, float _volt_min,float _volt_max, float _angle_min,float _angle_max)
//     :ADC_Pin{adc_Pin,VREF_ADC},
//      Volt_min{_volt_min},
//      Volt_max{_volt_max},
//      Angle_min{_angle_min},
//      Angle_max{_angle_max}
//     { ADC_Pin.set_reference_voltage(3.3); };

// // Methods
// /* Reads the ADC pin and returns the angle value in degrees */ 
// inline float angle_sensor:: read_angle(){
//     float New_ADC = ADC_Pin.read_voltage();

//     /* Tests if ADC voltage read is within the sensor's bounds [short or open circuit] */
//     if(New_ADC<Volt_min || New_ADC>Volt_max){
//         printf("\nCIRCUIT: ERROR DETECTED\n");
//         Circuit_ERROR=1;
//     }
//     else{
//         Circuit_ERROR=0;
//     }

//     // If variation is bigger than the expected noise, updates measurement 
//     if(abs(New_ADC - Current_ADC) > MAX_NOISE){
//         Current_ADC= New_ADC;
//         Angle= map(Current_ADC, Volt_min, Volt_max, Angle_min, Angle_max);
//     }

//     return Angle;
// }

// /* */
// bool angle_sensor::get_circuit_error(){      
//     return Circuit_ERROR;
// }


// /* Prints voltage read and 16b */
// void angle_sensor:: Voltage_print(){
//     uint16_t Voltage_16bit=ADC_Pin.read_u16();
//     printf("\n[VCU] ADC: Voltage_Read[16bit]: %d , Voltage[V]: %.2f V ",Voltage_16bit, ADC_Pin.read_voltage() );    
//     printf("Angle: %.2f\n", Angle);
// }


// /*====================================== Pedal Sensors ======================================*/
// // Constructors
// inline PedalSensor::PedalSensor(PinName adc_Pin, float _volt_min, float _volt_max)
//     :angle_sensor{adc_Pin, _volt_min, _volt_max, 0, 100}{}

// // Methods
// /* Reads the Pedal travel [0% = 0 | 100% = 16b] */
// inline uint16_t PedalSensor:: read_pedal(){
//     float New_ADC = ADC_Pin.read_voltage();

//     if(New_ADC<Volt_min || New_ADC>Volt_max){
        
//     }
//     // printf("\n NEW: %.2f , Current: %.2f, dif: %.2f\n",New_ADC, Current_ADC,abs(New_ADC - Current_ADC));
//     // If variation is bigger than the expected noise, updates measurement 
//     if(abs(New_ADC - Current_ADC) > MAX_NOISE){
//         Current_ADC= New_ADC;
//         Pedal_pos= map_u16(Current_ADC, Volt_min, Volt_max, PEDAL_MIN, PEDAL_MAX);
//     }

//     return Pedal_pos;
// }

// /* Print ADC voltage and Pedal Travel */
// inline void PedalSensor:: Voltage_print(){
//     uint16_t Voltage_16bit=ADC_Pin.read_u16();
//     double Percentage=(double(Voltage_16bit)/65535)*100;

//     printf("\n[VCU] ADC:\nVoltage_Read[16bit]: %d , Voltage[V]: %.2f V ",Voltage_16bit, ADC_Pin.read_voltage() );    
//     printf("\nPower [%%]: %.2f %%\n",Percentage);
// }


// /*================================== Steering Wheel Sensor ==================================*/
// // Constructos
// inline SteeringSensor::SteeringSensor(PinName adc_Pin, float _volt_min, float _volt_max)
//     :angle_sensor{adc_Pin, _volt_min, _volt_max, Vol_ang_min, Vol_ang_max}{}








// /*======================================== Auxiliar functions ========================================*/
// //time passed (in ms) since the program first started
// inline unsigned long current_ms(){
//     using namespace std::chrono;
//     auto now_us = time_point_cast<microseconds>(Kernel::Clock::now());      //time of referece= program's begin
//     unsigned long micros = now_us.time_since_epoch().count();               //time (in us) since the reference 
//     return micros / 1000.0;                                                 //turns time passed from us to ms
// }

// //Maps the ADC float voltage read into the angle's range 
// inline float map (float Variable, float in_min, float in_max, float out_min, float out_max) {
//     float Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
//     // Saturation + Out of bounds variable
//     if (Variable >= in_max ) {
//         Mapped_Variable= out_max;    
//     }
//     if (Variable <= (in_min + SATURATION_VOLTAGE) ) {
//         Mapped_Variable= out_min;    
//     }
//     return Mapped_Variable;
// }

// //Maps the ADC float voltage read into 16bit 
// inline uint16_t map_u16 (float Variable, float in_min, float in_max, uint16_t out_min, uint16_t out_max) {
//     uint16_t Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
//     // Saturation + Out of bounds variable
//     if (Variable >= (in_max - SATURATION_VOLTAGE) ) {
//         Mapped_Variable= out_max;    
//     }
    
//     // If it's close to min voltage, its the min voltage
//     if (Variable <= (in_min + SATURATION_VOLTAGE) ) {
//         Mapped_Variable= out_min;    
//     }
    
//     return Mapped_Variable;
// }







#endif