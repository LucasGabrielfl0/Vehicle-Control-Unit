#include "VCU_CAN.h"

void receive_from_BMS(CANMessage bms_msg){
   int a = bms_msg.data[0];

}

void send_to_ESP(){
    
}

// Function Called when a CAN msg is received
void VCU_CAN_Received(){
    CANMessage msg;

    switch (msg.id) {
        case BMS_CAN_ADDRESS:
            receive_from_BMS(msg);
            break;
        case ESP32_CAN_ADDRESS:
            break;
    }


}
