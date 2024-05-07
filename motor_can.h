/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * This file contains methods used for the car's embedded system's communication
 * The datafield follows the standard for CAN 2.0 communication as described in the
 * Plettenberg documentation (for MST motor controllers)
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
struct Rx_struct{
    //Receiver Datafield in CAN 2.0 standard (In order)
    uint8_t     Msg_Counter{0};        //[0]
    float       Input_Voltage{0};      //[1]
    uint8_t     Temp_Controller{0};
    uint8_t     Temp_motor{0};
    uint16_t    RPM{0};
    uint16_t    rx_PWM{0};
    uint8_t     Current{0};
};

/*================================== Send Struct ==================================*/
struct Tx_struct{
    //Transceiver Datafield in CAN 2.0 standard (In order)
    uint16_t    RPM_Limit{0};
    //motor pole pair is a constant
    uint16_t    Tx_PWM{0};
    uint16_t    Current_Limit{0};
    bool        isBreak{0};    //1= break, 0= Throttle
    bool        isReverse{0}; //1= Reverse, 0= Forward
};



/*================================== CLASS ==================================*/
class motor_can:public CAN{

    private:
    Rx_struct Datafield_inv1; //saves the datafield received from Inverter 1
    Rx_struct Datafield_inv2; //saves the datafield received from Inverter 2

    //Methods
    public:
    void set_CAN();
    void reset_can();
    bool baud_test();
    bool is_can_active();
    
    //Send data to both motor Controllers
    void send_to_inverter(uint16_t rpm_1, uint16_t pwm_1, uint16_t current_1 );
    void send_to_inverter_2(uint16_t rpm_2, uint16_t pwm_2, uint16_t current_2 );

    // Receive data from both motor controllers
    Rx_struct receive_from_inverter();
    Rx_struct receive_from_inverter_2();
    
    //print received data
    void Print_Datafields();
    void Print_Datafield(int Num, Rx_struct Inv);
    
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


inline void motor_can:: send_to_inverter(uint16_t rpm_1, uint16_t pwm_1, uint16_t current_1 ){
    CANMessage inverter_tx_msg;
    inverter_tx_msg.id = INVERSOR_TX_ID;
    inverter_tx_msg.len = 8; // Define o tamanho da msg, max = 8 Bytes
 
    inverter_tx_msg.data[0] = rpm_1 & 0xFF; //  Byte menos significativos do RPM
    inverter_tx_msg.data[1] = rpm_1 >> 8; // Byte mais significativos do RPM
    inverter_tx_msg.data[2] = N_Poles_Motor; // Numero de pares de polos do motor
    inverter_tx_msg.data[3] = pwm_1 & 0xFF; // Byte menos significativo do PWM
    inverter_tx_msg.data[4] = pwm_1 >> 8; // Byte mais significativo do PWM
    inverter_tx_msg.data[5] = current_1 & 0xFF; // Byte menos significativo corrente;
    inverter_tx_msg.data[6] = current_1 >> 8; // Byte mais significativo da corrente;
    inverter_tx_msg.data[7] = 0b00000000; 

}

inline void motor_can:: send_to_inverter_2(uint16_t rpm_2, uint16_t pwm_2, uint16_t current_2 ){
    CANMessage inverter_tx_msg_2;
    inverter_tx_msg_2.id = INVERSOR_TX_ID_2;
    inverter_tx_msg_2.len = 8; // Define o tamanho da msg, max = 8 Bytes
 
    inverter_tx_msg_2.data[0] = rpm_2 & 0xFF; //  Byte menos significativos do RPM
    inverter_tx_msg_2.data[1] = rpm_2 >> 8; // Byte mais significativos do RPM
    inverter_tx_msg_2.data[2] = N_Poles_Motor; // Numero de pares de polos do motor
    inverter_tx_msg_2.data[3] = pwm_2 & 0xFF; // Byte menos significativo do PWM
    inverter_tx_msg_2.data[4] = pwm_2 >> 8; // Byte mais significativo do PWM
    inverter_tx_msg_2.data[5] = current_2 & 0xFF; // Byte menos significativo corrente;
    inverter_tx_msg_2.data[6] = current_2 >> 8; // Byte mais significativo da corrente;
    inverter_tx_msg_2.data[7] = 0b00000000; 

}


inline Rx_struct motor_can:: receive_from_inverter() {
    //Definir mensagem CAN a ser recebida
    Rx_struct Datafield;

    //Aux variables
    int Voltage_Hb; //voltage High byte
    int Voltage_int;

    CANMessage inverter_rx_msg;
    inverter_rx_msg.len = 8;
    if(is_can_active()) {
        if(read(inverter_rx_msg)){
        // Aguardar a recepção da mensagem do inversor
            if(inverter_rx_msg.id == INVERSOR_RX_ID) {
                Datafield.Msg_Counter = inverter_rx_msg.data[0] & 0xF;
                
                Voltage_Hb = inverter_rx_msg.data[0] >> 4;
                Voltage_int = (Voltage_Hb<<8) | inverter_rx_msg.data[1];
                Datafield.Input_Voltage = Voltage_int/10.0f;

                Datafield.Temp_Controller = inverter_rx_msg.data[2]-100; //Range[0-255],Temp Range [-100°C to 155°C]
                Datafield.Temp_motor = inverter_rx_msg.data[3]-100; //Range[0-255],Temp Range [-100°C to 155°C]
                
                Datafield.RPM= (inverter_rx_msg.data[5]<< 8) | inverter_rx_msg.data[4] ;
                
                Datafield.rx_PWM=(inverter_rx_msg.data[6]/255.0f)*100;
                Datafield.Current = inverter_rx_msg.data[7];            
            
            }
        }
    }
    else{
        printf("Fail to stablish CAN connection. Reseting...\n");
        reset_can(); 
    }
    
    Datafield_inv1 =Datafield;
    return Datafield;

}

inline Rx_struct motor_can:: receive_from_inverter_2() {
    //Definir mensagem CAN a ser recebida
    Rx_struct Datafield;

    //Aux variables
    int Voltage_Hb; //voltage High byte
    int Voltage_int;

    CANMessage inverter_rx_msg2;
    inverter_rx_msg2.len = 8;
    if(is_can_active()) {
        if(read(inverter_rx_msg2)){
        // Aguardar a recepção da mensagem do inversor
            if(inverter_rx_msg2.id == INVERSOR_RX_ID_2) {
                Datafield.Msg_Counter = inverter_rx_msg2.data[0] & 0xF;
                
                Voltage_Hb = inverter_rx_msg2.data[0] >> 4;
                Voltage_int = (Voltage_Hb<<8) | inverter_rx_msg2.data[1];
                Datafield.Input_Voltage = Voltage_int/10.0f;

                Datafield.Temp_Controller = inverter_rx_msg2.data[2]-100; //Range[0-255],Temp Range [-100°C to 155°C]
                Datafield.Temp_motor = inverter_rx_msg2.data[3]-100; //Range[0-255],Temp Range [-100°C to 155°C]
                
                Datafield.RPM= (inverter_rx_msg2.data[5]<< 8) | inverter_rx_msg2.data[4] ;
                
                Datafield.rx_PWM=(inverter_rx_msg2.data[6]/255.0f)*100;
                Datafield.Current = inverter_rx_msg2.data[7];            
            
            }
        }
    }
    else{
        printf("Fail to stablish CAN connection. Reseting...\n");
        reset_can(); 
    }
    
    Datafield_inv2 =Datafield;
    return Datafield;

}


inline void motor_can::Print_Datafields(){
    Print_Datafield(1, Datafield_inv1);
    Print_Datafield(2, Datafield_inv2);
    
}

inline void Print_Datafield(int Num, Rx_struct Inv){
    printf("\r\n\t[CAN] Inverter %d: Volt=%.1f V, T_Ctrl= %d°C ,T_Motor = %d°C , RPM = %d, PWM = %d (%.2f %%), Ic= %d A",
    Num, Inv.Input_Voltage ,Inv.Temp_Controller ,
         Inv.Temp_motor    ,Inv.RPM ,
         Inv.rx_PWM, (Inv.rx_PWM/255.0f)*100,Inv.Current );
}

#endif