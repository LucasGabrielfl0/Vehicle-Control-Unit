/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 * This file is part of the car's embedded system's communication
 * it contains the necessary methods communicate with Plettenberg motor controllers (MST)
 * Using CAN 2.0 protocol as described in it's datasheet

 ***/
#ifndef _MOTOR_CAN_H_
#define _MOTOR_CAN_H_

#include "mbed.h"
#include "CAN.h"
#include <cstdint>

/*================================== COMMUNICATION PARAMETERS ==================================*/
#define INVERSOR_TX_ID 0x100
#define INVERSOR_RX_ID 0x101

#define INVERSOR_TX_ID_2 0x200
#define INVERSOR_RX_ID_2 0x201

#define MAXRPM 9000
#define MAXPWM 65535
#define MAX_CAN_DATA_SIZE 8

#define N_Poles_Motor 15

#define N_LEITURAS 10
#define SENSOR_MIN_OFFSET_5V 6650  // Para 5 volts utilizando uma fonte debancada
#define SENSOR_MAX_OFFSET_5V 65535

/*================================== Receive Struct ==================================*/
struct Receveid_data{
    //Data receive from the motor controller (In order)
    uint8_t     Msg_Counter;        //[0]
    uint16_t    Input_Voltage;      //[1]
    uint8_t     Temp_Controller;
    uint8_t     Temp_motor;
    uint16_t    rx_PWM;
    uint8_t     Current;
};

/*================================== Send Struct ==================================*/
struct Send_Data{
    //Data receive from the motor controller (In order)
    uint16_t    RPM_Limit;
    //motor pole pair is a constant
    uint16_t    Tx_PWM;
    uint16_t    Current_Limit;
    bool        isBreak;    //1= break, 0= Throttle
    bool        isReverse; //1= Reverse, 0= Forward
};



/*================================== CLASS ==================================*/
class motor_can:public CAN{

   // private:

    //Methods
    public:
    void set_CAN();
    void reset_can();
    bool baud_test();
    bool is_can_active();
    //Send data to both motor Controllers
    void send_to_inverter(uint16_t rpm_1, uint16_t Angle_1, uint16_t current_1 );
    void send_to_inverter_2(uint16_t rpm_2, uint16_t Angle_2, uint16_t current_2 );
    // Receive data from both motor controllers
    void receive_from_inverter();
    void receive_from_inverter_2();

    //Constructors
    public:
    motor_can(PinName _can_pin_rx, PinName _can_pin_tx, uint32_t _can_frequency);
};


/*================================== Constructors ==================================*/
inline motor_can::motor_can(PinName _can_pin_rx, PinName _can_pin_tx, uint32_t _can_frequency)
:CAN(_can_pin_rx,_can_pin_tx, _can_frequency) {}

/*================================== METHODS ==================================*/
inline void motor_can:: set_CAN(){
    mode(CAN::Normal);
    filter(0, 0, CANStandard);
}


//Resets CAN chanel
inline void motor_can:: reset_can() {  
    reset();
    mode(CAN::Normal);
    filter(0, 0, CANStandard); // reconfigurar o filtro para receber todas as mensagens
}


//Tests if CAN can send messages
inline bool motor_can:: baud_test(){
    CANMessage msg;
    return write(msg);    
}


//Tests if CAN can read messages
inline bool motor_can::is_can_active(){
    CANMessage msg;
    return read(msg); // tente ler uma mensagem
}


inline void motor_can:: send_to_inverter(uint16_t rpm_1, uint16_t Angle_1, uint16_t current_1 ){
    CANMessage inverter_tx_msg;
    inverter_tx_msg.id = INVERSOR_TX_ID;
    inverter_tx_msg.len = 8; // Define o tamanho da msg, max = 8 Bytes
 
    inverter_tx_msg.data[0] = rpm_1 & 0xFF; //  Byte menos significativos do RPM
    inverter_tx_msg.data[1] = rpm_1 >> 8; // Byte mais significativos do RPM
    inverter_tx_msg.data[2] = N_Poles_Motor; // Numero de pares de polos do motor
    inverter_tx_msg.data[3] = Angle_1 & 0xFF; // Byte menos significativo do PWM
    inverter_tx_msg.data[4] = Angle_1 >> 8; // Byte mais significativo do PWM
    inverter_tx_msg.data[5] = current_1 & 0xFF; // Byte menos significativo corrente;
    inverter_tx_msg.data[6] = current_1 >> 8; // Byte mais significativo da corrente;
    inverter_tx_msg.data[7] = 0b00000000; 

}

inline void motor_can:: send_to_inverter_2(uint16_t rpm_2, uint16_t Angle_2, uint16_t current_2 ){
    CANMessage inverter_tx_msg_2;
    inverter_tx_msg_2.id = INVERSOR_TX_ID_2;
    inverter_tx_msg_2.len = 8; // Define o tamanho da msg, max = 8 Bytes
 
    inverter_tx_msg_2.data[0] = rpm_2 & 0xFF; //  Byte menos significativos do RPM
    inverter_tx_msg_2.data[1] = rpm_2 >> 8; // Byte mais significativos do RPM
    inverter_tx_msg_2.data[2] = N_Poles_Motor; // Numero de pares de polos do motor
    inverter_tx_msg_2.data[3] = Angle_2 & 0xFF; // Byte menos significativo do PWM
    inverter_tx_msg_2.data[4] = Angle_2 >> 8; // Byte mais significativo do PWM
    inverter_tx_msg_2.data[5] = current_2 & 0xFF; // Byte menos significativo corrente;
    inverter_tx_msg_2.data[6] = current_2 >> 8; // Byte mais significativo da corrente;
    inverter_tx_msg_2.data[7] = 0b00000000; 

}




#endif
