#include "control_system.h"
#include <cstdint>
#include <stdint.h>

int PEDAL_MAX=10;

/*======================================== Constructors ========================================*/
ControlSystem::ControlSystem(float _kp, float _ki, float _kd, uint16_t _ts_ms)
:Kp{_kp},
 Ki{_ki},
 Kd{_kd}, 
 Ts_ms{_ts_ms} {};

ControlSystem::ControlSystem(){};


// Setup Generic Controller Design
void ControlSystem::setup(){
    float Nc[3];
    float Dc[3];
}

/*======================================== VELOCITY CONTROL ==================================*/
/* Open Loop Control ?? */
void ControlSystem::OpenLoop(float DutyC){
    uint16_t Dc_16b;
    // Return the Dutycycle as 16b uint
    Dc_16b= UINT16_MAX*DutyC;
}

/* Closed Loop Control */
uint16_t ControlSystem::control_RPM(uint16_t RPM_c, uint16_t RPM_setpoint){
    uint16_t Dc_16b;

    // Current Error:
    ek[0] = RPM_setpoint - RPM_c;

    // Controller
    uk[0] = ek[0]*Kp;

    // Saturation
    if(uk[0]>1){
        uk[0]=1;
    }

    if(uk[0]<0){
        uk[0]=0;
    }

    // Update Variables
    uk[2] = uk[1];
    uk[1] = uk[0];

    ek[2] = ek[1];
    ek[1] = ek[0];

    // Return the Dutycycle as 16b uint
    Dc_16b = UINT16_MAX*uk[0];
    
    // If shutdown
    if(shutdown){
        Dc_16b = 0;
        uk[0]  = 0;
        uk[1]  = 0;
        uk[2]  = 0;
    }

    return Dc_16b;
}


/* Electronic Differential + Control */
void ControlSystem::DifferentialControl(float Steering, uint16_t apps, uint16_t RPM_c[], uint16_t Dc_16b[]){
    uint16_t RPM_dif[2];

    // Electronic Differential [Setpoint for each motor]
    CalcDifferential(Steering, apps, RPM_dif);

    // Get control signal for each motor
    Dc_16b[0] = control_RPM(RPM_c[0], RPM_dif[0]);            // Control signal for motor 1
    Dc_16b[1] = control_RPM(RPM_c[1], RPM_dif[1]);            // Control signal for motor 2
    
}


/*========================================= ELECTRONIC DIFFERENTIAL =========================================*/
// Calculates the Velocity [in RPM] of each wheel during a turn, W1 = [LEFT WHEEL] || W2 = [RIGHT WHEEL]
void CalcDifferential(float Steering_dg, uint16_t Acc_pedal, uint16_t RPM_dif[]){
    int64_t RPM_W1,RPM_W2, Wv_rpm;
    float Ack_rad;
    float del_W;

    // APPS to RPM
    Wv_rpm= Acc_pedal;

    // For very low angles in the Steering wheel, there's no change
    if( abs(Steering_dg)< 5){
        Steering_dg=0;
    }

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.14 * Steering_dg)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);

    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    RPM_W1= Wv_rpm * ( 1 + del_W);
    RPM_W2= Wv_rpm * ( 1 - del_W);

    // Prevent overflow/underflow [values above 16b or below 0 going to the uint16 output]
    //if overflow, sets max
    if(RPM_W1>PEDAL_MAX){
        RPM_W1=PEDAL_MAX;
    }
    if(RPM_W2>PEDAL_MAX){
        RPM_W2=PEDAL_MAX;
    }

    //if Underflow, sets min
    if(RPM_W1<0){
        RPM_W1=0;
    }

    if(RPM_W2<0){
        RPM_W2=0;
    }

    // Returns Velocity in each Wheel
    RPM_dif[0] = RPM_W1;
    RPM_dif[1] = RPM_W2;
}






void ElectronicDifferential(float Steering_dg, float apps, float Wc_Motor[]){
    int64_t RPM_W1,RPM_W2, Wv_rpm;
    float Ack_rad;
    float del_W;
    // int64_t MAX_RPM; 
    int64_t MIN_RPM; 

    // APPS to RPM
    Wv_rpm= apps;

    // For very low angles in the Steering wheel, there's no change
    if( abs(Steering_dg)< 5){
        Steering_dg=0;
    }

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.14 * Steering_dg)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);

    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    RPM_W1= Wv_rpm * ( 1 + del_W);
    RPM_W2= Wv_rpm * ( 1 - del_W);

    // Prevent overflow/underflow [values above 16b or below 0 going to the uint16 output]
    //if overflow, sets max
    if(RPM_W1>PEDAL_MAX){
        RPM_W1=PEDAL_MAX;
    }
    if(RPM_W2>PEDAL_MAX){
        RPM_W2=PEDAL_MAX;
    }

    //if Underflow, sets min
    if(RPM_W1<0){
        RPM_W1=0;
    }

    if(RPM_W2<0){
        RPM_W2=0;
    }

    // Return Velocity in each Wheel
    Wc_Motor[0] = 1;
    Wc_Motor[1] = 1;
    
}





/* Closed Loop Control */
uint16_t ControlSystem::control(float vel, float setpoint){
    uint16_t Dc_16b;

    //Current Error:
    ek[0]= setpoint-vel;

    // Controller
    uk[0] = ek[0]*Kp;

    // Saturation
    if(uk[0]>1){
        uk[0]=1;
    }

    if(uk[0]<0){
        uk[0]=0;
    }

    // Update Variables
    uk[2]=uk[1];
    uk[1]=uk[0];

    ek[2]=ek[1];
    ek[1]=ek[0];

    // Return the Dutycycle as 16b uint
    Dc_16b= UINT16_MAX*uk[0];
    
    if(shutdown){
        Dc_16b= 0;
    }


    return Dc_16b;
}





/**/
void OpenLoopDifferential(float Steering_dg, uint16_t Apps, uint16_t Dc_Motor[]){
    int64_t RPM_W1,RPM_W2, Wv_rpm;  // Aux Variables
    float Ack_rad;
    float del_W;

    // Purely Proporcional
    Wv_rpm= Apps;
    
    // For very low angles in the Steering wheel, there's no change
    if( abs(Steering_dg)< 5){
        Steering_dg=0;
    }

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    Ack_rad=(0.14 * Steering_dg)* PI/180;

    // Velocity Variation during turn
    del_W= KC * tan(Ack_rad);

    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    RPM_W1= Wv_rpm * ( 1 + del_W);
    RPM_W2= Wv_rpm * ( 1 - del_W);

    // Prevent overflow/underflow [values above 16b or below 0 going to the uint16 output]
    //if overflow, sets max
    if(RPM_W1>PEDAL_MAX){
        RPM_W1=PEDAL_MAX;
    }
    if(RPM_W2>PEDAL_MAX){
        RPM_W2=PEDAL_MAX;
    }

    //if Underflow, sets min
    if(RPM_W1<0){
        RPM_W1=0;
    }

    if(RPM_W2<0){
        RPM_W2=0;
    }

    // Returns Control signal in each Wheel
    Dc_Motor[0] = RPM_W1;
    Dc_Motor[1] = RPM_W2;
}






/*================================== ACCELERATION PEDAL PLAUSIBILITY CHECK ==================================*/
// Every 10ms, checks if there's a discrepancy bigger than 10% lastting more then 100 ms
bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2){
    bool AppsError_flag,Error_APPS;
    unsigned int Error_Counter{0};

    // Checks 10% discrepancy
    if ( abs(Apps_1 - Apps_2) > (0.1 * max(Apps_1, Apps_2)) ){
        // if there's discrepancy, adds in counter
        Error_Counter++;
    }
    else { 
        Error_Counter= 0;      //if error ceased resets counter
    }
    
    // If Implausibility lasts 10 iterations (100 ms), theres error
    if(Error_Counter >=10){
        return 1;
    }
    else{
        return 0;
    }

}

/*====================================== BRAKE PEDAL PLAUSIBILITY CHECK ======================================*/
// Checks if the Accel. and Brake Were both pressed at the same time
bool BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val){    
    bool Error_BPPC{0};

    //If APPS >= 25% of pedal travel and Brake is pressed, stops the car 
    if ( (Apps_val >= 0.25*PEDAL_MAX) && (Brake_val >= 0.02*PEDAL_MAX) ){
        Error_BPPC = 1;
        printf("BSE: ERROR DETECTED");
    }

    // Error only Stops if APPS goes below 5% of pedal travel
    if(Apps_val < 0.05*PEDAL_MAX){
        Error_BPPC = 0;
    }

    return Error_BPPC;
}


/*====================================== MOTOR/MOTOR CONTROLLER ERROR CHECK ======================================*/
// Checks if the motor sent 
// void Motor_Error_Check(RxStruct Inverter_1, RxStruct Inverter_2){
//     bool Error_Motor{0};

//     int16_t Tm_1 = Inverter_1.Temp_motor; 
//     int16_t Tm_2 = Inverter_2.Temp_motor; 
//     int16_t Tc_1 = Inverter_1.Temp_Controller;
//     int16_t Tc_2 = Inverter_2.Temp_Controller;
    
//     // if Temperature is above limit, shuts the car down
//     Error_Motor = ( (Tm_1> MAX_TM) || (Tm_2> MAX_TM) || (Tc_1> MAX_TC) || (Tc_2> MAX_TC) );
// }






// inline void print_differential(uint16_t Apps_1, float Steering_dg, VelocityStruct Wheel_Velocity){
//         double Pedal=(double(Apps_1)/65535)*100;
//         double Pm1=(double(Wheel_Velocity.RPM_W1)/65535)*100;
//         double Pm2=(double(Wheel_Velocity.RPM_W2)/65535)*100;

//         printf("\n[Electronic Differential]  ==================================================#");
//         printf("\n Pedal_Travel: %.2f%%,  Steering Angle : %.2f°\n", Pedal,Steering_dg);
//         printf("\n Power_1: %.2f %%,  Power_2: %.2f %%\n", Pm1,Pm2);
//         printf("#============================================================================#\n");

//     }

// inline void print_Control(uint16_t Apps_1,uint16_t Apps_2,uint16_t BSE, float Steering_dg, VelocityStruct Wheel_Velocity){
//         double Pedal_1=(double(Apps_1)/65535)*100;
//         double Pedal_2=(double(Apps_2)/65535)*100;
//         double Brake=(double(BSE)/65535)*100;
//         double Pm1=(double(Wheel_Velocity.RPM_W1)/65535)*100;
//         double Pm2=(double(Wheel_Velocity.RPM_W2)/65535)*100;

//         printf("\n[Control]  ==================================================#");
//         printf("\n Pedal_1: %.2f%%,  Pedal_2: %.2f%%, Brake: %.2f%%\n", Pedal_1, Pedal_2, Brake);
//         printf("\n Pedal_Travel: %.2f%%,  Steering Angle : %.2f°\n", Pedal_1,Steering_dg);
//         printf("\n Power_1: %.2f %%,  Power_2: %.2f %%\n", Pm1,Pm2);
//         printf("#============================================================================#\n");
//     }

inline void print_errors(bool ERROR_State,bool Error_APPS,bool Error_BPPC, bool Error_Motor){
    printf("\nState:  %d || ",ERROR_State);
    printf("Error_APPS:  %d || Error_BPPC:  %d || Error_Motor:  %d\n",Error_APPS, Error_BPPC, Error_Motor);
    }
