#define ERROR
#ifndef ERROR
#include "PID_controller.h"

void PID_controller::set_coefficient(int16_t kp, int16_t ki, int16_t kd) {
    coef_p = kp;
    coef_i = ki;
    coef_d = kd;
}
void PID_controller::set_coefficient(float kp, float ki, float kd) {
    set_coefficient((int16_t)(kp * 1000), (int16_t)(ki * 1000), (int16_t)(kd * 1000));
}
void PID_controller::set_iteration_time(uint32_t dt_ms) {
    iteration_time = dt_ms;
}
void PID_controller::set_limits(int16_t set_min_signal, int16_t set_max_signal) {
    min_signal = set_min_signal;
    max_signal = set_max_signal;
}


void PID_controller::set_target_value(int32_t set_target_value) {
    target_value = set_target_value;
}
int32_t PID_controller::get_target_value() {
    return target_value;
}

int16_t PID_controller::calculate_signal(int32_t present_value) {
    int32_t error = target_value - present_value;
    int32_t delta_input = previous_value - present_value;
    previous_value = present_value;

    int32_t output_signal;
    output_signal = error * coef_p / 1000; // пропорциональая составляющая

    output_signal += delta_input * coef_d / (int32_t)iteration_time; // дифференциальная составляющая
    integral += error * coef_i * (int32_t)iteration_time / 1000000; // обычное суммирование инт. суммы
    integral = constrain(integral, min_signal, max_signal); // ограничиваем инт. сумму
    output_signal += integral; // интегральная составляющая
    output_signal = constrain(output_signal, min_signal, max_signal); // ограничиваем выход

    return output_signal;
}


#endif
#if defined (ERROR)
#include "PID_controller.h"

#define PRINT(p_data) \
    {if(p_data < 0) Serial.print("-"); else Serial.print("+"); \
     if(abs(p_data)/1000 > 0) Serial.print(abs(p_data)); else { Serial.print("0");\
     if(abs(p_data)%1000/100 > 0) Serial.print(abs(p_data)); else { Serial.print("0");\
     if(abs(p_data)%100/10 > 0) Serial.print(abs(p_data)); else { Serial.print("0");\
     if(abs(p_data)%10/1 > 0) Serial.print(abs(p_data)); else { Serial.print("0");}}}}}

void PID_controller::set_coefficient(int16_t kp, int16_t ki, int16_t kd) {
    coef_p = kp;
    coef_i = ki;
    coef_d = kd;
}
void PID_controller::set_coefficient(float kp, float ki, float kd) {
    set_coefficient((int16_t)(kp * 1000), (int16_t)(ki * 1000), (int16_t)(kd * 1000));
}
void PID_controller::set_iteration_time(uint32_t dt_ms) {
    iteration_time = dt_ms;
}
void PID_controller::set_limits(int16_t set_min_signal, int16_t set_max_signal) {
    min_signal = set_min_signal;
    max_signal = set_max_signal;
}


void PID_controller::set_target_value(int32_t set_target_value) {
    target_value = set_target_value;
}
int32_t PID_controller::get_target_value() {
    return target_value;
}

int16_t PID_controller::calculate_signal(int32_t present_value) {
    int32_t error = target_value - present_value;
    int32_t delta_input = previous_value - present_value;
    previous_value = present_value;

    // Serial.print("(");
    // Serial.print("er=");
    // PRINT(error);
    // // Serial.print(error);
    // Serial.print(", del=");
    // PRINT(delta_input);

    int32_t output_signal;
    output_signal = error * coef_p / 1000; // пропорциональая составляющая

    output_signal += delta_input * coef_d / (int32_t)iteration_time; // дифференциальная составляющая
    integral += error * coef_i * (int32_t)iteration_time / 1000000; // обычное суммирование инт. суммы
    integral = constrain(integral, min_signal, max_signal); // ограничиваем инт. сумму
    output_signal += integral; // интегральная составляющая
    output_signal = constrain(output_signal, min_signal, max_signal); // ограничиваем выход

    // Serial.print(",   P=");
    // PRINT((error * coef_p / 1000));
    // Serial.print(", I=");
    // PRINT(integral);
    // Serial.print(", D=");
    // PRINT((delta_input * coef_d / (int32_t)iteration_time));
    // Serial.print(" )     ");

    return output_signal;
}
#endif