/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This code should ONLY BE USED FOR TESTING, as it contains many legacy functions no longer used
 * by the team's embedded system's sector(such as 'enviar')
 ***/

#include "mbed.h"
#include "MPU9250.h"
#include "angle_sensor.h"
#include "time.h"
#include "CAN.h"
#include <cstdint>

/*===================================== PORTS (STM32 F746ZG)=====================================*/
#define Volante_in      PB_1
#define BSE_in          PA_0
#define APPS1_in        PC_2
#define APPS2_in        PF_4
#define APPS_out        PA_5

//legacy functions
void enviar(float Volante, float BSE_v, float APPS1_v, float APPS2_v,
            float Veloc_1,float Veloc_2,float ax_v,float ay_v, float az_v,
            float temperature_v, float gx_v, float gy_v, float gz_v);

void LerSensoresMPU(MPU9250& mpu9250_instance);


//InterruptIn Velocidade_in(PA_4, PullNone); //(PA_4, PullNone)
CAN can1(PD_0,PD_1,500e3);     // Velocidade 1

/*===================================== VCU Sensors Initialization =====================================*/
BSE_Sensor BSE (BSE_in);
APP_Sensors APPS (APPS1_in ,APPS2_in, APPS_out);
Steering_Wheel_Sensor Steering_sensor (Volante_in);


//MPU9250 mpu9250;


int main(){
/*================================== Parameters Initialization ==================================*/

/* MPU
    i2c.frequency(400000); //?

    mpu9250.resetMPU9250();
    mpu9250.MPU9250SelfTest(SelfTest);
    mpu9250.initMPU9250();
    mpu9250.getAres();
    mpu9250.getGres();
*/

    can1.mode(CAN::Normal);
    can1.filter(0,0,CANStandard);


   //t.start();
    APPS_struct Apps_ang= APPS.read_APPS();
    float BSE_ang= BSE.read();
    float Steering_ang= Steering_sensor.read();
    bool Error_check;
    float Veloc_1{0.1};
    float Veloc_2{0.1};
    

    //float temperature_c{1.0}; //Temperature in °C
    //float Linear_Velocity_1{1.0},Linear_Velocity_2{1.0}; //Linear Velocity values in m/s
    //float Accel_x{0},Accel_y{0}, Accel_z{0}; //Accelaration values in m/s^2
    //float w_gx{0.0}, w_gy{0}, w_gz{0};        //Angular Velocity values in rad/s

/*================================== LOOP ==================================*/
    while(true){
        //Read Sensor data
        Apps_ang= APPS.read_APPS();
        BSE_ang = BSE.read();
        Steering_ang = Steering_sensor.read();


        //Read_MPU


/*================================== PRINT ==================================*/
        //LerSensoresMPU(mpu9250);
        //LerSensoresVelocidade();
        //LerSensorTemperatura();
        //printf("Velocidade 1 %.2f m/s,Velocidade 2 %.2f m/s \n",Veloc_1, Veloc_2); // Converte a velocidade para RPM
        printf("\n===========================================================================\n");        
        printf("[VCU] Volante: %.2f , APPS1: %.2f , APPS2: %.2f , BSE: %.2f \n", Steering_ang, Apps_ang.s1, Apps_ang.s2, BSE_ang);
        Steering_sensor.Voltage_print();
        Error_check=BSE.Plausibility_check( max(Apps_ang.s1,Apps_ang.s2) );
        printf("\n===========================================================================\n");
        
        //printf("ax = %f, ay = %f, az = %f  m/s²\n", ax * 9.81 - 0.15, ay * 9.81 - 0.1, az * 9.81 + 0.12);
        //printf("gx = %f, gy = %f, gz = %f  rad/s\n", gx, gy, gz); // Printa os dados de velocidade angular
        //printf(" temperatura = %f  C\n\r", temperature_c);
        wait_us(1000000);//0.5s
    }

    return 0;
}


/*================================== aux (legacy) Functions ==================================*/
// Função para leitura dos sensores MPU9250
/*
void LerSensoresMPU(MPU9250& mpu9250_instance) {

      // Se o pino for para HIGh, todo os registradores recebem novos dados
        if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // Na interrupção, checa se a data está pronta para interromper
            
            mpu9250.readAccelData(accelCount);  // Lê os valores x/y/z do adc   
            // Calcula a aceleração em g's
           ax = (float)accelCount[0] * aRes - accelBias[0];
            ay = (float)accelCount[1] * aRes - accelBias[1];   
            az = (float)accelCount[2] * aRes - accelBias[2];  
    
            mpu9250.readGyroData(gyroCount);  // Lê os valores x/y/z do adc  
            // Calcula a velocidade angular em graus por segundo
            gx = (float)gyroCount[0] * gRes - gyroBias[0];
            gy = (float)gyroCount[1] * gRes - gyroBias[1];  
            gz = (float)gyroCount[2] * gRes - gyroBias[2];   
        }

        // Normalização e conversão dos valores obtidos
        mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);

}

*/

//void Send_msg(int id, int data_len, float )
//we should probably just send the 16bit raw data instead 

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
