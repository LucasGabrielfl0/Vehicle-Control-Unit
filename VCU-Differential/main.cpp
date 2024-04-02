/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This code should ONLY BE USED FOR TESTING the angler_sensor header file, as it contains many legacy functions no
 * longer used by the team's embedded system's sector(such as 'enviar')
 ***/

#include "mbed.h"
#include "MPU9250.h"
#include "angle_sensor.h"
#include "time.h"
#include "CAN.h"
#include <cstdint>

/*===================================== PORTS (STM32 F746ZG)=====================================*/
#define Volante_in      PC_2
#define BSE_in          PA_0
#define APPS1_in        PF_4
#define APPS2_in        PF_4
#define APPS_out        PA_5

//legacy functions
void enviar(float Volante, float BSE_v, float APPS1_v, float APPS2_v,
            float Veloc_1,float Veloc_2,float ax_v,float ay_v, float az_v,
            float temperature_v, float gx_v, float gy_v, float gz_v);

//InterruptIn Velocidade_in(PA_4, PullNone); //(PA_4, PullNone)
CAN can1(PD_0,PD_1,500e3);     // Velocidade 1

/*===================================== VCU Sensors Initialization =====================================*/
BSE_Sensor BSE_1 (BSE_in);
APP_Sensors APPS (APPS1_in ,APPS2_in, APPS_out);
Steering_Wheel_Sensor Steering_sensor (Volante_in);


int main(){
/*================================== Parameters Initialization ==================================*/
    float BSE_ang= BSE_1.read();
    float APPS1_ang = APPS.read_S1();
    float APPS2_ang = APPS.read_S2();
    float Steering_ang= Steering_sensor.read();
    
    float Veloc_1{0.1};
    float Veloc_2{0.1};

    float Volante{0.1};
    float APPS1{0.1};
    float APPS2{0.1};
    float BSE{0.1};
    
    //float temperature_c{1.0}; //Temperature in °C
    //float Linear_Velocity_1{1.0},Linear_Velocity_2{1.0}; //Linear Velocity values in m/s
    //float Accel_x{0},Accel_y{0}, Accel_z{0}; //Accelaration values in m/s^2
    //float w_gx{0.0}, w_gy{0}, w_gz{0};        //Angular Velocity values in rad/s


/*================================== LOOP ==================================*/
    while(true){
    //Read Sensor data
        APPS1_ang = APPS.read_S1();
        APPS2_ang = APPS.read_S2();
        BSE_ang = BSE_1.read();
        Steering_ang = Steering_sensor.read();

        printf("APPS1: %.2f°",APPS1_ang);
        printf("APPS2: %.2f°",APPS2_ang);
        printf("BREAK: %.2f°",BSE_ang);  
        printf("Steering wheel: %.2f°",Steering_ang);     


/*================================== TO DO ==================================*/
        //    LerSensoresMPU(mpu9250);
        //LerSensoresVelocidade();
        //LerSensorTemperatura();
        //printf("Velocidade 1 %.2f m/s,Velocidade 2 %.2f m/s \n",Veloc_1, Veloc_2); // Converte a velocidade para RPM
        //printf("Volante: %.2f°, APPS1: %.2f°, APPS2: %.2f°, BSE: %.2f°\n", Volante,APPS1,APPS2,BSE);
        //printf("ax = %f, ay = %f, az = %f  m/s²\n", ax * 9.81 - 0.15, ay * 9.81 - 0.1, az * 9.81 + 0.12); //Printa os dados de aceleração convertendo para m/s² e fazendo a conversão
        //printf("gx = %f, gy = %f, gz = %f  rad/s\n", gx, gy, gz); // Printa os dados de velocidade angular
        //printf(" temperatura = %f  C\n\r", temperature_c);
        wait_us(500000);//0.5s
    }

    return 0;
}



//aux (legacy) Functions
void enviar(float Volante, float BSE_v, float APPS1_v, float APPS2_v,
            float Veloc_1,float Veloc_2,float ax_v,float ay_v, float az_v,
            float temperature_v, float gx_v, float gy_v, float gz_v){
    CANMessage SENSORES1, SENSORES2, SENSORES3;
    SENSORES1.id = 1;
    SENSORES1.len =8;
    SENSORES1.data[0]=  ((wchar_t)(int) (Volante*10))%256;
    SENSORES1.data[1]=  ((wchar_t)(int) (Volante*10))/256;
    SENSORES1.data[2]=  ((wchar_t)(int) (BSE_v*10))%256;
    SENSORES1.data[3]=  ((wchar_t)(int) (BSE_v*10))/256;
    SENSORES1.data[4]=  ((wchar_t)(int) (APPS1_v*10))%256;
    SENSORES1.data[5]=  ((wchar_t)(int) (APPS1_v*10))/256;
    SENSORES1.data[6]=  ((wchar_t)(int) (APPS2_v*10))%256;
    SENSORES1.data[7]=  ((wchar_t)(int) (APPS2_v*10))/256;

    SENSORES2.id = 2;
    SENSORES2.len =8;
    SENSORES2.data[0]=  (wchar_t)(int) Veloc_1;
    SENSORES2.data[1]=  (wchar_t)(int) Veloc_2;
    SENSORES2.data[2]=  ((wchar_t)(int) (ax_v*100))%256;
    SENSORES2.data[3]=  ((wchar_t)(int) (ax_v*100))/256;
    SENSORES2.data[4]=  ((wchar_t)(int) (ay_v*100))%256;
    SENSORES2.data[5]=  ((wchar_t)(int) (ay_v*100))/256;
    SENSORES2.data[6]=  ((wchar_t)(int) (az_v*100))%256;
    SENSORES2.data[7]=  ((wchar_t)(int) (az_v*100))/256;

    SENSORES3.id = 3;
    SENSORES3.len =8;
    SENSORES3.data[0]=  ((wchar_t)(int) (temperature_v*100))%256;
    SENSORES3.data[1]=  ((wchar_t)(int) (temperature_v*100))/256;
    SENSORES3.data[2]=  ((wchar_t)(int) (gx_v*100))%256;
    SENSORES3.data[3]=  ((wchar_t)(int) (gx_v*100))/256;
    SENSORES3.data[4]=  ((wchar_t)(int) (gy_v*100))%256;
    SENSORES3.data[5]=  ((wchar_t)(int) (gy_v*100))/256;
    SENSORES3.data[6]=  ((wchar_t)(int) (gz_v*100))%256;
    SENSORES3.data[7]=  ((wchar_t)(int) (gz_v*100))/256;   
    

    if (can1.write(SENSORES1)){
     //   printf("Mensagem SENSORES1 foi Enviada");
     
    }
    else{
        can1.reset();
    }

    if (can1.write(SENSORES2)){
       // printf("Mensagem SENSORES2 foi Enviada");
    }
    else{
        can1.reset();
    }

    if (can1.write(SENSORES3)){
        //printf("Mensagem SENSORES3 foi Enviada");
    }
    else{
        can1.reset();
    }
}
