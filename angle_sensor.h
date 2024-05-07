/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file is part of the car's VCU (VEHICLE CONTROL UNIT)
 * it contains the APPS plausibilty check, BSE plausibility check
 * as well the algorithm for the Electronic Differential
 ***/

#ifndef _ANGLE_SENSOR_H_
#define _ANGLE_SENSOR_H_

#include "mbed.h"
#include <cstdint>
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
#define BSE_ang_min     0.0     //Minimum value for the break Pedal angle (Degrees)
#define BSE_ang_max     120.0   //Maximum value for the break Pedal angle (Degrees)

//APPS (Accelerator Pedal Position Sensor) Parameters
#define APPS1_min       0.425   //APPS1 Minimum input voltage
#define APPS1_max       3.021   //APPS1 Maximum input voltage
#define APPS2_min       0.4     //APPS2 Minimum input voltage
#define APPS2_max       2.9     //APPS2 Maximum input voltage
#define APPS_ang_min    0       //Minimum value for the Accelerator Pedal angle (Degrees)
#define APPS_ang_max    120     //Maximum value for the Accelerator Pedal angle (Degrees)

#define APPS1_start     0       //Start Angle for APPS1
#define APPS1_end       0       //end Angle for APPS1
#define APPS2_start     0       //Start Angle for APPS2
#define APPS2_end       0       //end Angle for APPS2



//Car's Physical parameters (For speed control)
#define RADIUS_WHEEL    0.27    //Wheel's radius in Meters
#define N_DISC_WHOLES   3       //Number of wholes in the break's disc


//Error check ?
#define INPUT_MIN 4500      //max pi
#define INPUT_MAX 48000     //limite superior do valor entrada para que não seja erro


#define D_TW     1 //Car's track Width [Bitola] in meters
#define A_WB     1 //Car's Wheelbase in meters 
#define PI      3.14159265358979323846


//Ultility Functions
double millis();
double Differential();
float RPM_to_W(uint16_t RPM_inv);
void Calibrate_ADC();

struct APPS_struct{
    float s1; //sensor 1 angle value
    float s2; //sensor 2 angle value
};

struct Wref_struct{
    uint16_t W1; //Win
    uint16_t W2; //Wout
};


/*================================== CLASSES ==================================*/
//class for Acelleration Pedal, break Pedal, and Steering wheel
class angle_sensor{

    //Atributes
    protected:
    float Angle;        //Angle's value in Degree
    float Volt_min;     //Sensor's Minimum voltage measurement
    float Volt_max;     //Sensor's Maximum voltage measurement
    float Angle_min;    //Sensor's Minimum angle
    float Angle_max;    //Sensor's Maximum angle
    AnalogIn ADC_Pin;   //Input Pin in the MicroController

    protected:
    bool ERROR_State{0}; //if there is actual error, 1= ERROR, 0= all good :) 

    //Methods
    public:
    float read();
    float map(long Variable, float in_min, float in_max, float out_min, float out_max);     //Maps varible (general func)
    float map_ADC(long Variable, float in_min, float in_max, float out_min, float out_max); //Maps Voltage (16bit) into Angle
    void Voltage_print();        //prints raw voltage
    bool Input_Error_Check();    //tests if ADC value (16bit) is within reasonable range
    bool get_Error_State();      //1= ERROR, 0= all good :) 


    //Constructors:
    public:
    angle_sensor(PinName adc_Pin, float _volt_min,float _volt_max, float _angle_min,float _angle_max);

};

//Sensors
class BSE_Sensor: public angle_sensor{

    //methods:
    public:
    bool Plausibility_check(float APPS_dg); //1= ERROR, 0= all good :) 

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
    AnalogOut APPS_out; //? => change name, change use
    //Angle Read
    float APPS1_Angle{0};           //angle value [degrees]
    float APPS2_Angle{0};           //angle value [degrees]
    //Safety
    double Error_Start_Time{0};     //Time stamp when error started
    bool AppsError_flag{0};         //just a flag, within the 100ms rule
    bool ERROR_State{0};            //if there is actual error, 1= ERROR, 0= all good :) 

    //methods
    public:
    APPS_struct read_APPS();
    float read_S1();
    float read_S2();
    bool Implausibility_check(); //1= ERROR, 0= all good :) 

    //constructor
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
:ADC_Pin{adc_Pin}, Volt_min{_volt_min},Volt_max{_volt_max}, Angle_min{_angle_min}, Angle_max{_angle_max}{

    ADC_Pin.set_reference_voltage(3.31);
};

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
//Maps the ADC 16bit voltage read into the angle's range 
inline float angle_sensor::map_ADC (long Variable, float in_min, float in_max, float out_min, float out_max) {
    //Maps the sensors input voltage from [0 - 3.3] to [0 - 16bit]
    in_min= (in_min/3.3)*65535;
    in_max= (in_max/3.3)*65535;

    float Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return Mapped_Variable;
}

inline float angle_sensor::map (long Variable, float in_min, float in_max, float out_min, float out_max) {
    float Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return Mapped_Variable;
}

//Reads the ADC pin and returns the angle value in degrees 
inline float angle_sensor:: read(){
    Angle=map_ADC( ADC_Pin.read_u16(), Volt_min, Volt_max, Angle_min, Angle_max);
    return Angle;    
}

inline void angle_sensor:: Voltage_print(){

    uint16_t Voltage_16bit=ADC_Pin.read_u16();
    printf("\n[VCU] ADC (16bit) Voltage: %d , Real Voltage: %.2f V  \n",Voltage_16bit, Voltage_16bit*3.3/65535);    
}



/*======================================== APPS ========================================*/
inline float APP_Sensors:: read_S1(){
    return APPS1.read();    
}

inline float APP_Sensors:: read_S2(){
    return APPS2.read();    
}

inline APPS_struct APP_Sensors:: read_APPS(){
    APPS_struct Apps_values;
    
    float Angle_S1=APPS1.read();
    float Angle_S2=APPS2.read();
    uint16_t max_Apps = uint16_t( max(Angle_S1,Angle_S2) );

    //? REALLY not sure about that one:
    if (Implausibility_check() == true){
        APPS_out.write_u16(0);
    } 
    else {
        APPS_out.write_u16(max_Apps);
    }

    Apps_values.s1=Angle_S1;
    Apps_values.s2=Angle_S2;
    
    return Apps_values;
}


//Checks if there's a discepancy bigger than 10%, for longer than 100 ms
inline bool APP_Sensors::Implausibility_check(){
    APPS1_Angle=APPS1.read();
    APPS2_Angle=APPS2.read();

    if ( abs(APPS1_Angle - APPS2_Angle) > (0.1 * max(APPS1_Angle, APPS2_Angle)) ){
        if(AppsError_flag == 0) { //Starts Counting
            Error_Start_Time = millis(); 
            AppsError_flag = 1;
        }
        
        //if the error continues after 100ms, shuts down motor
        if(millis() - Error_Start_Time > 100) {
            ERROR_State= 1;
            printf("[APPS]: IMPLAUSIBILITY CHECK ERROR | ERROR RUNNING TIME: %.2f ms",millis() - Error_Start_Time);

        } else{
            ERROR_State= 0;
        }

    } else { //if error ceased
        AppsError_flag = 0;
        ERROR_State= 0;
        }

    return ERROR_State;
}

inline bool angle_sensor::Input_Error_Check(){
    float Angle_Value=read();
    float prevTime{0};
    bool isError{0};

    if(Angle_Value < INPUT_MIN or Angle_Value > INPUT_MAX){
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

/*======================================== BSE ========================================*/
//Follows EV.5.7 (Fsae Rule) Break Pedal Plausibility Check (BPPC)
inline bool BSE_Sensor::Plausibility_check(float APPS_dg){
    float Total_Course{APPS1_end - APPS1_start};    //angle change between min and max torque                             //APPS current angle value
    float BSE_dg{read()};                           //BSE Current angle Value

    //if Acelerator Pedal is above 25% total course while break pedal is pressed => ERROR
    if(APPS_dg>0.25*Total_Course && BSE_dg>0.5){
        ERROR_State=1;
    }
    
    //Error only stops if Acelerator Pedal is below 5% of total course (regardless of break)  
    if(APPS_dg<0.05*Total_Course){
        ERROR_State=0;
    }

    if(ERROR_State==1){
        printf("[BSE]: IMPLAUSIBILITY CHECK ERROR | BSE: %.2f° , APPS: %.2f°",BSE_dg, APPS_dg);
    }
    
    return ERROR_State;
}





/*======================================== Auxiliar functions ========================================*/
//time passed (in ms) since the program first started
inline double millis(){
    using namespace std::chrono;
    auto now_us = time_point_cast<microseconds>(Kernel::Clock::now()); //time of referece= program's begin
    long micros = now_us.time_since_epoch().count(); //time (in us) since the reference 
    return micros / 1000.0; //turns time passed from us to ms
}

//Calculates angular velocity of the inner and outter Wheels (W_in and W_out)
inline Wref_struct Differential(float Steering_dg, uint16_t RPM_inv){    
    float Ack_dg{0}, Ack_rad{0}; //Ackerman Angle, rad[0 0.785], Dg[0 45]
    float W_out, W_in, d_W, Wv; //Wv= actual angular speed in the motor ???
    Wref_struct W_dif;

    Wv=RPM_to_W(RPM_inv);

    //Steering Wheel to Ackerman Angle
    Ack_dg=0.1415*Steering_dg-0.092;
    Ack_rad=Ack_dg*PI/180; //rad2deg
    
    //Ackerman Angle to angular speed (wheels)
    d_W= Wv*D_TW*tan(Ack_rad)/(2*A_WB);


    if(Wv<0){ //?
        W_out= 0;
        W_in=  0;
    }
    else{
        W_out= Wv+d_W;
        W_in= Wv-d_W;
    }

    W_dif.W1=W_in;
    W_dif.W2=W_out;
    
    return W_dif;
}

inline void Calibrate_ADC(){

}

//Controls 


//Turns the RPM values into Angular velocity [Rad/s]
inline float RPM_to_W(uint16_t RPM_inv){
    return (RPM_inv*2*PI/60);
}

#endif
