/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 ***/

#include "mbed.h"
#include "MPU9250.h"
#include "Angle_Sensor.h"
#include "can_inverter.h"
#include "time.h"
#include "CAN.h"
#include <cstdint>

/*===================================== PORTS (STM32 F746ZG)=====================================*/
#define Volante_in      PC_2
#define BSE_in          PA_0
#define APPS1_in        PF_4
#define APPS2_in        PF_4
#define APPS_out        PA_5


//InterruptIn Velocidade_in(PA_4, PullNone); //(PA_4, PullNone)
//CAN can1(PD_0,PD_1,500e3);     // Velocidade 1
//class CAN_2 can1(PD_0,PD_1,500e3);
can_inverter can1(PD_0,PD_1,500e3);

/*===================================== VCU Sensors Initialization =====================================*/
BSE_Sensor BSE_1 (BSE_in);
APP_Sensors APPS (APPS1_in ,APPS2_in, APPS_out);
Steering_Wheel_Sensor Steering_sensor (Volante_in);
//Angle_Sensor(PD_0, 1.0, 2.0, 3.0, 4.0);

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
