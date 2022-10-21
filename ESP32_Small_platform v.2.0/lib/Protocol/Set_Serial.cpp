#include "Set_Serial.h"

Set_Serial::Set_Serial(): val(0), size(0), tmp (nullptr){}

Set_Serial::~Set_Serial(){
    val = 0;
    if (tmp != nullptr) {delete[] tmp;}
}
int16_t Set_Serial::check_error_t(uint8_t *buf, bool num){
  val = 0;
  read_val(buf);
    if (val > 495 || val < -495){
       if (!num){  return INTERVAL_T_L;}
       else return INTERVAL_T_R;
    }
    if (buf[0] == '-')
    val*=-1;
    else if (buf[0] != '+') {
    if (!num){  return SIGN_T_L;}
    else return SIGN_T_R;
    }
    return val;
}

int16_t Set_Serial::check_error_c(uint8_t *buf, bool num, uint8_t LR){
    val = 0;
    read_val(buf);
    if (buf[0] == '-')
    val*=-1;
    else if (buf[0] != '+') {
    if (!num){  return SIGN_C_P;}
       else return SIGN_C_D;
    }
    if (LR != 'L' && LR != 'l' && LR != 'R' && LR != 'r') return NOT_L_or_R;
    return val;
}

int16_t Set_Serial::check_error_f(uint8_t *buf){
    val = 0;
    read_val(buf);
    if (val >= 10000 ){
        return INTERVAL_F;
    }
   return val;
}

void Set_Serial::read_val(uint8_t *buf) {
    for(int i = 1; i < 6; ++i) {
        if(('0' <= buf[i]) && (buf[i] <= '9')) {
            val *= 10;
            val += buf[i] - '0';
        }
        else {
            val = NOT_NUM;
            return; 
        }
    }
}

int16_t Set_Serial::read_command(uint8_t *buf, uint8_t size) {
    if((buf[S] != 's') && (buf[S] != 'S')) {
        if((buf[S] != '\r') && (buf[S] != '\n')) {return START_SYMBOL;}
        return 0;
    }
   if(size < (SET_MAX)) {return LENGTH_BYTES;}
   if((buf[SET_MAX-1] != 'e') && (buf[SET_MAX-1] != 'E')) {return END_SYMBOL;}
   if (buf[CMD] == 't' || buf[CMD] == 'c' || buf[CMD] == 'f' || buf[CMD] == 'u')
   {
       buf[CMD] -= 32;
   }
   switch(buf[CMD]){
        case 'T':
            return 1;
        case 'C':
            return 2;
        case 'F':
            return 3;
        case 'U':
            return 4;
        case 'S':
            return 5;
        default:
            return NOT_CMD;
    } 
}
