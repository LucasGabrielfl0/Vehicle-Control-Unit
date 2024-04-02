/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file is part of the car's VCU (VEHICLE CONTROL UNIT)
 * it contains the APPS plausibilty check, BSE plausibility check

 ***/

#ifndef _ANGLE_SENSOR_H_
#define _ANGLE_SENSOR_H_

#include "mbed.h"
#include <time.h>

/*================================== SENSORS PARAMETERS ==================================*/
//Steering Wheel Parameters
#define Vol_min         0.425   //Steering Wheel Sensors Minimum input voltage
#define Vol_max         2.825   //Steering Wheel Sensors Sensors Maximum input voltage
#define Vol_ang_min     -180    //Minimum value for the Steering Wheel angle (Degrees)
#define Vol_ang_max     180     //Maximum value for the Steering Wheel angle (Degrees)

//BSE (Break System Enconder) Parameters
#define BSE_min         0.3     //BSE Sensors Minimum input voltage
#define BSE_max         3.0     //BSE Sensors Minimum input voltage
#define BSE_ang_min     0.0       //Minimum value for the break Pedal angle (Degrees)
#define BSE_ang_max     120.0     //Maximum value for the break Pedal angle (Degrees)

//APPS (Accelerator Pedal Position Sensor) Parameters
#define APPS1_min       0.425   //APPS1 Minimum input voltage
#define APPS1_max       3.021   //APPS1 Maximum input voltage
#define APPS2_min       0.4     //APPS2 Minimum input voltage
#define APPS2_max       2.9     //APPS2 Maximum input voltage
#define APPS_ang_min    0       //Minimum value for the Accelerator Pedal angle (Degrees)
#define APPS_ang_max    120     //Maximum value for the Accelerator Pedal angle (Degrees)

//Car's Physical parameters (For speed control)
#define Radius_wheel    0.27    //Wheel's radius in Meters
#define N_Disc_Wholes   3       //Number of wholes in the break's disc


//Error check
#define Input_min 4500 //limite inferior do valor entrada para que não seja erro
#define Input_max 48000 //limite superior do valor entrada para que não seja erro


#define d_m     1 //Car's track Width [Bitola] in meters
#define A_m     1 //Car's Wheelbase in meters 
#define PI      3.14159265358979323846


//Ultility Functions
double millis();
double Differential();
void Calibrate_ADC();

/*================================== CLASSES ==================================*/
//class for Acelleration Pedal, break Pedal, and Steering wheel
class angle_sensor{

    //Atributes
    private:
    float Angle;        //Angle's value in Degree
    float Volt_min;     //Sensor's Minimum voltage measurement
    float Volt_max;     //Sensor's Maximum voltage measurement
    float Angle_min;    //Sensor's Minimum angle
    float Angle_max;    //Sensor's Maximum angle
    AnalogIn ADC_Pin;   //Input Pin in the MicroController

    //Methods
    public:
    float read();
    float Map(long Variable, float in_min, float in_max, float out_min, float out_max); //Maps the Input voltage into the Angle
    bool Input_Error_Check();
 
    //Constructors:
    angle_sensor(PinName adc_Pin, float _volt_min,float _volt_max, float _angle_min,float _angle_max);

};

//Sensors
class BSE_Sensor: public angle_sensor{
    //bool BSE_Error_check();
    
    //Constructors:
    public:
    BSE_Sensor(PinName adc_Pin);
};

class APP_Sensors{
    //Tests if theres more than 10% discrepancy between APPS1 and APPS2
    //If there is a discrapancy, and it maintains for more then 10ms, Error=1 and shuts down the motor
    private:
    angle_sensor APPS1;
    angle_sensor APPS2;
    AnalogIn APPS_out; 
    
    public:
    bool APPS_Error_check();
    float read_S1();
    float read_S2();

    public:
    APP_Sensors(PinName _apps1_pin, PinName _apps2_pin, PinName _apps_out_pin);

};

class Steering_Wheel_Sensor: public angle_sensor{

    public:
    Steering_Wheel_Sensor(PinName adc_Pin);
};


/*======================================== Constructors ========================================*/
//Angle Sensor
inline angle_sensor::angle_sensor(PinName adc_Pin, float _volt_min,float _volt_max, float _angle_min,float _angle_max)
:ADC_Pin{adc_Pin}, Volt_min{_volt_min},Volt_max{_volt_max}, Angle_min{_angle_min}, Angle_max{_angle_max}{};

//APP Sensors
inline APP_Sensors::APP_Sensors(PinName _apps1_pin, PinName _apps2_pin, PinName _apps_out_pin)
    :APPS1(_apps1_pin,APPS1_min,APPS1_max,APPS_ang_min,APPS_ang_max), //APPS 1
    APPS2(_apps2_pin,APPS2_min,APPS2_max,APPS_ang_min,APPS_ang_max), // APPS 2
    APPS_out{_apps_out_pin}{} //APPS out


//BSE Sensor
inline BSE_Sensor::BSE_Sensor(PinName adc_Pin)
    :angle_sensor{adc_Pin, BSE_min, BSE_max, BSE_ang_min, BSE_ang_max}{}

//Steering Wheel Sensor
inline Steering_Wheel_Sensor::Steering_Wheel_Sensor(PinName adc_Pin)
:angle_sensor{adc_Pin, Vol_min, Vol_max, Vol_ang_min, Vol_ang_max}{}




/*======================================== Methods ========================================*/
//Maps the input voltage's range into the angle's range 
inline float angle_sensor::Map (long Variable, float in_min, float in_max, float out_min, float out_max) {
    float Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return Mapped_Variable;
}

//Reads the ADC pin and returns the angle value in degrees 
inline float angle_sensor:: read(){
    Angle=Map( ADC_Pin.read_u16(), Volt_min, Volt_max, Angle_min, Angle_max);
    return Angle;    
}

/*======================================== APPS ========================================*/
inline float APP_Sensors:: read_S1(){
    float Angle=APPS1.read();
    return Angle;    
}

inline float APP_Sensors:: read_S2(){
    float Angle=APPS2.read();
    return Angle;    
}

//Checks if there's a discepancy bigger than 10%, for longer than 100 ms
inline bool APP_Sensors::APPS_Error_check(){
    float Apps1_value=APPS1.read();
    float Apps2_value=APPS2.read();
    int cont;
    int tempo;
    bool isError{0};

    if (abs(Apps1_value - Apps2_value) > (0.1 * max(Apps1_value, Apps2_value))){
        
        cont += 1;
        if(cont == 1) {//Starts Counting
            tempo = millis(); 
        }

        if(millis() - tempo > 100) {
            isError= 1;
        }
        else{
            isError= 0;
        }
    }

    if(isError==1){

    }
    else{

    }


    return isError;
}

inline bool angle_sensor::Input_Error_Check(){
    float Angle_Value=read();
    float prevTime{0};
    bool isError{0};

    if(Angle_Value < Input_min or Angle_Value > Input_max){
        if(millis() - prevTime >= 100){
            prevTime =  millis();
            isError=1;
        } 
    }
    else{
        prevTime = millis();
        isError=0;
    }

    return isError;
}

/*======================================== Auxiliar functions ========================================*/
//time passed (in ms) since the program first started
inline double millis(){
    using namespace std::chrono;
    auto now_us = time_point_cast<microseconds>(Kernel::Clock::now()); //time of referece= program's begin
    long micros = now_us.time_since_epoch().count(); //time (in us) since the reference 
    return micros / 1000.0; //turns time passed from us to ms
}

//Calculates angular speed of the inner and outter Wheels (W_in and W_out)
inline double Differential(float Ack_deg, float Wv){
    double Ack_rad=Ack_deg*PI/180; //Ackerman Angle, range [0 0.785] rad (0 to 45 Degrees)
    float W_out,W_int;
    float d_W, Tg_Ack;

    Tg_Ack=tan(Ack_rad);
    d_W= abs(Wv*d_m*Tg_Ack/(2*A_m) );

    W_out= Wv+d_W;
    W_int= Wv-d_W;

    return 1;
}

inline void Calibrate_ADC(){

}


#endif
