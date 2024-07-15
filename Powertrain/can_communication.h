/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * This file contains methods used for the car's embedded system's communication
 * The datafield follows the standard for CAN 2.0 communication as described in the
 * Plettenberg documentation (for MST motor controllers)
 ***/
#ifndef _CAN_COMMUNICATION_H_
#define _CAN_COMMUNICATION_H_

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

#define MOTOR_POLE_PAIRS 15            //Number of Pole Pairs in the Motor

#define N_LEITURAS 10
#define SENSOR_MIN_OFFSET_5V 6650  // Para 5 volts utilizando uma fonte debancada
#define SENSOR_MAX_OFFSET_5V 65535

/*==================================== SAFETY PARAMETERS ====================================*/
#define MAX_CURRENT_LIMIT       10          // Max Phase Current in the motor [A]
#define MAX_RPM_LIMIT           4500        // Max Velocity of the motor [RPM] 



/*================================== Receive Struct ==================================*/
struct RxStruct{
    //Receiver Datafield in CAN 2.0 standard (In order)
    uint8_t     Msg_Counter{0};        //[0]
    float       Supply_Voltage{0};      //[1]
    int16_t     Temp_Controller{0};
    int16_t     Temp_motor{0};
    uint16_t    RPM{0};
    float       rx_PWM{0};
    uint8_t     Current{0};
};

/*================================== Send Struct ==================================*/
struct TxStruct{
    //Transceiver Datafield in CAN 2.0 standard (In order)
    uint16_t    RPM_Limit{0};
    //motor pole pair is a constant
    uint16_t    Tx_PWM{0};
    uint16_t    Current_Limit{0};
    bool        isBreak{0};         //1= break, 0= Throttle
    bool        isReverse{0};       //1= Reverse, 0= Forward
};



/*================================== CLASS ==================================*/
class MotorCAN:public CAN{

    private:
    RxStruct Datafield_inv1; //saves the datafield received from Inverter 1
    RxStruct Datafield_inv2; //saves the datafield received from Inverter 2

    //Methods
    public:
    void set_CAN();
    void reset_can();
    bool baud_test();
    bool is_can_active();
    
    //Send data to both motor Controllers
    void send_to_inverter(unsigned int Motor_Id, uint16_t DC_pwm_1, bool IsBreak );
    void send_to_inverter_1(uint16_t DC_pwm_1, bool IsBreak);
    void send_to_inverter_2(uint16_t DC_pwm_2, bool IsBreak);

    // Receive data from both motor controllers
    RxStruct receive_from_inverter(unsigned int Inverter_Id);
    RxStruct receive_from_inverter_1();
    RxStruct receive_from_inverter_2();
    
    //print received data
    void Print_Datafields();
    void Print_Datafield(int Num, RxStruct Inv);
    
    //Constructors
    public:
    MotorCAN(PinName _can_pin_rx, PinName _can_pin_tx, uint32_t _can_frequency);
};


/*================================== Constructors ==================================*/
inline MotorCAN::MotorCAN(PinName _can_pin_rx, PinName _can_pin_tx, uint32_t _can_frequency)
:CAN(_can_pin_rx,_can_pin_tx, _can_frequency) {}

/*================================== METHODS ==================================*/
inline void MotorCAN:: set_CAN(){
    mode(CAN::Normal);
    filter(0, 0, CANStandard);
}


//Resets CAN chanel
inline void MotorCAN:: reset_can() {  
    reset();
    mode(CAN::Normal);
    filter(0, 0, CANStandard); // reconfigurar o filtro para receber todas as mensagens
}

//Tests if CAN can send messages (Creates empy CAN message and tests if it was sent)
inline bool MotorCAN:: baud_test(){
    CANMessage msg;
    return write(msg);    
}

//Tests if CAN can read messages
inline bool MotorCAN::is_can_active(){
    CANMessage msg;
    return read(msg); // tente ler uma mensagem
}

/*==========================================  SEND DATA ==========================================*/
// Sends Data to Motor Controller 1
inline void MotorCAN:: send_to_inverter_1(uint16_t DC_pwm_1, bool IsBreak){
    send_to_inverter(INVERSOR_TX_ID, DC_pwm_1, IsBreak);
}

// Sends Data to Motor Controller 2
inline void MotorCAN:: send_to_inverter_2(uint16_t DC_pwm_2, bool IsBreak){
    send_to_inverter(INVERSOR_TX_ID_2, DC_pwm_2, IsBreak);
}

// Sends Data to Motor Controller
inline void MotorCAN:: send_to_inverter(unsigned int Motor_Id, uint16_t DC_pwm, bool IsBreak ){    
    IsBreak=0; // DON'T use Break while using power source (only with batteries).
    
    CANMessage inverter_tx_msg;     // Creates Can message
    inverter_tx_msg.id= Motor_Id;   // Id
    inverter_tx_msg.len = 8;        // Datafield size [Bytes], the max size is 8
 
    inverter_tx_msg.data[0] = MAX_RPM_LIMIT & 0xFF;                 //  LSB RPM
    inverter_tx_msg.data[1] = MAX_RPM_LIMIT >> 8;                   // MSB RPM
    inverter_tx_msg.data[2] = MOTOR_POLE_PAIRS;                     // Constant (15 pairs)
    inverter_tx_msg.data[3] = DC_pwm & 0xFF;                        // LSB PWM (DutyCycle)
    inverter_tx_msg.data[4] = DC_pwm >> 8;                          // MSB PWM (DutyCycle)
    inverter_tx_msg.data[5] = MAX_CURRENT_LIMIT & 0xFF;             // Current's  LSB;
    inverter_tx_msg.data[6] = MAX_CURRENT_LIMIT >> 8;               // Current's  MSB;
    inverter_tx_msg.data[7] = 0b00000000;
    //inverter_tx_msg.data[7] = (IsBreak<<7);                       // b7: 0 = Throtle || 1 = Brake 0
    
    //Sends CAN message, resets can if there is an error
    if(baud_test()) {
        //if CAN channel is good, sends msg
        if(write(inverter_tx_msg)) {
        }
        else {
            printf("Mesagem não enviada...\n");
            reset_can(); 
            }
    }

}

/*==========================================  RECEIVE DATA ==========================================*/
// Receives Data from Motor Controller 1
inline RxStruct MotorCAN:: receive_from_inverter_1(){
    RxStruct Datafield_1 =receive_from_inverter(INVERSOR_RX_ID);
    return Datafield_1;
}

// Receives Data from Motor Controller 2
inline RxStruct MotorCAN:: receive_from_inverter_2(){
    RxStruct Datafield_2 =receive_from_inverter(INVERSOR_RX_ID_2);
    return Datafield_2;
}

// Receives Data FROM Motor Controller
inline RxStruct MotorCAN:: receive_from_inverter(unsigned int Inverter_Id){
    RxStruct Datafield;

    //Aux variables
    int Voltage_Hb; //voltage High byte
    int Voltage_int;

    CANMessage inverter_rx_msg;
    inverter_rx_msg.len = 8;
    if(is_can_active() ) {
        if(read(inverter_rx_msg)){
        // Aguardar a recepção da mensagem do inversor
            if(inverter_rx_msg.id == Inverter_Id) {
                Datafield.Msg_Counter = inverter_rx_msg.data[0] & 0xF;
                
                Voltage_Hb = inverter_rx_msg.data[0] >> 4;
                Voltage_int = (Voltage_Hb<<8) | inverter_rx_msg.data[1];
                Datafield.Supply_Voltage = Voltage_int/10.0f;

                Datafield.Temp_Controller = inverter_rx_msg.data[2]-100; //Range[0-255],Temp Range [-100°C to 155°C]
                Datafield.Temp_motor = inverter_rx_msg.data[3]-100;     //Range[0-255],Temp Range [-100°C to 155°C]
                
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


/*==========================================  PRINT DATA ==========================================*/
inline void MotorCAN::Print_Datafields(){
    Print_Datafield(1, Datafield_inv1);
    Print_Datafield(2, Datafield_inv2);
    
}

inline void Print_Datafield(int Num, RxStruct Inv){
    printf("\r\n\t[CAN] Inverter %d: Volt=%.1f V, T_Ctrl= %d°C ,T_Motor = %d°C , RPM = %d, PWM = %.2f (%.2f %%), Ic= %d A",
    Num, Inv.Supply_Voltage ,Inv.Temp_Controller ,
         Inv.Temp_motor    ,Inv.RPM ,
         Inv.rx_PWM, (Inv.rx_PWM/255.0f)*100,Inv.Current );
}

inline void Print_Datafield_3(RxStruct Motor_Data){
    printf("\r\n\t[CAN]Volt=%.1f V , RPM = %d Ic= %d A", 
    Motor_Data.Supply_Voltage , Motor_Data.RPM , Motor_Data.Current );
    
    // Dc Pwm
    printf(" PWM = %.2f , (%.2f %%)", Motor_Data.rx_PWM, (Motor_Data.rx_PWM/65535.0)*100.0);

    // Temperature
    printf(" Tc= %d°C , Tm = %d°C", Motor_Data.Temp_Controller , Motor_Data.Temp_motor);

}

#endif