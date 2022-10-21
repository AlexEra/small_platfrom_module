#include "Get_data.h"

Get_data::Get_data(): tmp(nullptr){}

Get_data:: ~Get_data(){
    if (tmp != nullptr) {delete[] tmp;}
}

void Get_data::print(uint8_t *data, uint32_t ID) {
        uint8_t size = 0;
    if (ID == ID_R){ //rapidity
        size = R_MAX*3 + SERIAL_R;
        tmp = new char[size];
        format_byte(data, R_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'R';
    }
    else if (ID == ID_IR){
        size = IR_MAX*3 + SERIAL_IR;
        tmp = new char[size];
        format_byte(data, IR_MAX, size - 1) ;
        tmp[0] = 'S';
        tmp[1] = 'I';
    }
    else if (ID == ID_RFID){
        size = RFID_MAX*3 + SERIAL_RFID;
        tmp = new char[size];
        format_byte(data, RFID_MAX, size - 1) ;
        tmp[0] = 'S';
        tmp[1] = 'F';
    }
    else if (ID > ID_IR) {
        //size = US_MAX*3 + SERIAL_US;
        size = 18;
        tmp = new char[size];
        for (int i = 0; i < size; ++i) {
            tmp[i] = '0';
        }
        //format_byte(data, US_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'U';
        // switch(ID){
        //     //US1
        //     case ID_US_F:
        //     tmp[2] = 'F'; //forward
        //     break;
        //     //US1
        //     case ID_US_FR:
        //     tmp[2] = 'J'; //back
        //     break;
        //     //US2
        //     case ID_US_L:
        //     tmp[2] = 'L'; //left
        //     break;
        //     //US4
        //     case ID_US_R:
        //     tmp[2] = 'R'; //right
        //     break;
        //     //US5
        //     case ID_US_FL:
        //     tmp[2] = 'K'; 
        //     break;
        //     default: { 
        //         Serial.print(ERR_ID);
        //         Serial.print(":");
        //         Serial.print(ID);
        //     }
        // }
    }
    tmp[size - 1] = 'E';
    if ((ID == ID_IR) || (ID == ID_RFID)) {
        for (int i = 0; i < size; i++){
        Serial.print(tmp[i]);
        }
        Serial.println();
    }

    // uint8_t size = 0;
    // if (ID == ID_R){ //rapidity
    //     size = R_MAX*3 + SERIAL_R;
    //     tmp = new char[size];
    //     format_byte(data, R_MAX, size - 1);
    //     tmp[0] = 'S';
    //     tmp[1] = 'R';
    // }
    // else if (ID == ID_IR){
    //     size = IR_MAX*3 + SERIAL_IR;
    //     tmp = new char[size];
    //     format_byte(data, IR_MAX, size - 1) ;
    //     tmp[0] = 'S';
    //     tmp[1] = 'I';
    // }
    // else if (ID == ID_RFID){
    //     size = RFID_MAX*3 + SERIAL_RFID;
    //     tmp = new char[size];
    //     format_byte(data, RFID_MAX, size - 1) ;
    //     tmp[0] = 'S';
    //     tmp[1] = 'F';
    // }
    // else if (ID > ID_IR) {
    //     size = US_MAX*3 + SERIAL_US;
    //     tmp = new char[size];
    //     format_byte(data, US_MAX, size - 1);
    //     tmp[0] = 'S';
    //     tmp[1] = 'U';
    //     switch(ID){
    //         //US1
    //         case ID_US_F:
    //         tmp[2] = 'F'; //forward
    //         break;
    //         //US1
    //         case ID_US_FR:
    //         tmp[2] = 'J'; //back
    //         break;
    //         //US2
    //         case ID_US_L:
    //         tmp[2] = 'L'; //left
    //         break;
    //         //US4
    //         case ID_US_R:
    //         tmp[2] = 'R'; //right
    //         break;
    //         //US5
    //         case ID_US_FL:
    //         tmp[2] = 'K'; 
    //         break;
    //         default: { 
    //             Serial.print(ERR_ID);
    //             Serial.print(":");
    //             Serial.print(ID);
    //         }
    //     }
    // }
    // tmp[size - 1] = 'E';
    // for (int i = 0; i < size; i++){
    //     Serial.print(tmp[i]);
    // }
    // Serial.println();
}

void Get_data::print(int32_t data, uint32_t ID){
    uint8_t size = 0;
    if (ID == ID_R){ //rapidity
        size = R_MAX*3 + SERIAL_R;
        tmp = new char[size];
        format_byte(data, R_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'R';
        if (data < 0){ tmp[2] = '-'; }
        else tmp[2] = '+';
       
    }
    else if (ID == ID_IR){
        size = IR_MAX*3 + SERIAL_IR;
        tmp = new char[size];
        format_byte(data, IR_MAX, size - 1) ;
        tmp[0] = 'S';
        tmp[1] = 'I';
    }
    else if (ID == ID_RFID){
        size = RFID_MAX*3 + SERIAL_RFID;
        tmp = new char[size];
        format_byte(data, RFID_MAX, size - 1) ;
        tmp[0] = 'S';
        tmp[1] = 'F';
    }
    else if (ID > ID_IR) {
        size = US_MAX*3 + SERIAL_US;
        tmp = new char[size];
        format_byte(data, US_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'U';
        switch(ID){
            //US1
            case ID_US_F:
            tmp[2] = 'F'; //forward
            break;
            //US2
            case ID_US_FR:
            tmp[2] = 'J'; //back
            break;
            //US3
            case ID_US_FL:
            tmp[2] = 'K'; 
            break;
            //US4
            //US3
            case ID_US_L:
            tmp[2] = 'L'; //left
            break;
            //US4
            case ID_US_R:
            tmp[2] = 'R'; //right
            break;
            default: {
                Serial.print(ERR_ID);
                Serial.print(":");
                Serial.print(ID);
            }
        }
    }
    tmp[size - 1] = 'E';
    for (int i = 0; i < size; i++){
        Serial.print(tmp[i]);
    }
   Serial.println();
}

void Get_data::format_byte(uint8_t *data, uint8_t len, uint8_t size){
    int32_t *get_data = new int32_t [len]; 
    for (int i = 0; i < len; i++){
        get_data[i] = data[i];
    }
    for (int i = len - 1; i > -1; i--){ 
        for (int j = 0 ; j < 3;  j++){
            tmp[--size] = get_data[i] % 10 + '0';
            get_data[i] /= 10;
            
        }
    }
    delete[] get_data;
}

void Get_data::format_byte(int32_t data, uint8_t len, uint8_t size){
    int32_t get_data = data;
    for (int j = 0 ; j < 3;  j++) {
        tmp[--size] = get_data % 10 + '0';
        get_data /= 10;
    }
}
