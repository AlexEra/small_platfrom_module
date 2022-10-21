#include "Engine.h"

bool Engine::init(uint8_t forward, uint8_t backward, uint8_t PWM, uint8_t encoder, uint8_t channel,
                  std::function<void(void)> interrupt_fun, uint16_t max_power_PWM) {
    encoder_counter = 0;
    pin_forward = forward;
    pin_backward = backward;
    pin_PWM = PWM;
    pin_encoder = encoder;
    num_PWM_channel = channel;
    encoder_fun = interrupt_fun;
    if(max_power_PWM > 4095) {
        max_power = 4095;
        init_state = false;
    }
    else {
        max_power = max_power_PWM;
        init_state = true;
    }
    pid.set_limits(-((int16_t)max_power_PWM), max_power_PWM);
    return !init_state;
}

bool Engine::begin() {
    if(!init_state)
        return true;
    encoder_counter = 0;
    // Настройка выходов
    pinMode(pin_forward, OUTPUT);
    pinMode(pin_backward, OUTPUT);
    pinMode(pin_PWM, OUTPUT);
    // Настройка аппаратного ШИМ 0-255 0-4095
    ledcSetup(num_PWM_channel, 1000, 12);
    ledcAttachPin(pin_PWM, num_PWM_channel);
    // Настройка прерывания (если есть)
    if(pin_encoder != 0 && encoder_fun != nullptr) {
        pinMode(pin_encoder, INPUT);
        attachInterrupt(pin_encoder, encoder_fun, RISING);
    }
    return false;
}

bool Engine::begin(uint8_t forward, uint8_t backward, uint8_t PWM, uint8_t encoder, uint8_t channel,
                   std::function<void(void)> interrupt_fun, uint16_t max_power_PWM) {
    init(forward, backward, PWM, encoder, channel, interrupt_fun, max_power_PWM);
    return begin();
}

// Прерывания датчика Холла

void Engine::interrupt() {
    //Serial.println(encoder_counter);
    ++encoder_counter;
}

uint32_t Engine::get_counter(bool clear) {
    if(!clear)
        return encoder_counter;
    uint32_t counter = encoder_counter;
    encoder_counter = 0;
    return counter;
}

int32_t Engine::get_speed(bool clear) {
    int32_t speed = get_counter(clear);
    speed = speed * 1000 / 234;
    if(get_revers())
        speed *= -1;
    return speed;
}

// ПИД-регулятор

void Engine::PID_calculate() {
    int32_t g_s = get_speed(false);
    int16_t pid_cal = pid.calculate_signal(g_s);
    if(get_revers()) {
        pid_cal = pid_cal - (int16_t)get_power();
    }
    else {
        pid_cal = pid_cal + (int16_t)get_power();
    }
    pid_cal = constrain(pid_cal, -(int32_t)max_power, (int32_t)max_power);
    set_power(pid_cal);
    get_speed(true);
    // Serial.println(". ");
    // Serial.print("speed = ");
    // Serial.print(g_s);
    // Serial.print("\t (TS = ");
    // Serial.print(get_target_speed());
    // Serial.println(")");
    // Serial.print("PWM = ");
    // Serial.print(pid_cal);
    // pid_cal = constrain(pid_cal, -(int32_t)max_power, (int32_t)max_power);
    // Serial.print("\t(");
    // Serial.print(pid_cal);
    // Serial.println(")");
}

void Engine::set_coefficient(int16_t kp, int16_t ki, int16_t kd) {
    pid.set_coefficient(kp, ki, kd);
}

void Engine::set_coefficient(float kp, float ki, float kd) {
    pid.set_coefficient(kp, ki, kd);
}

void Engine::set_iteration_time(uint32_t dt_ms) {
    pid.set_iteration_time(dt_ms);
}

// Управление двигателем

uint16_t Engine::get_power() const {
    return power;
}
bool Engine::get_revers() const {
    return reverse_movement;
}

bool Engine::set_power(uint16_t power_PWM, bool revers) {
    if(power_PWM > max_power)
        return true;
    power = power_PWM;
    reverse_movement = revers;

    ledcWrite(num_PWM_channel, power);
    digitalWrite(pin_forward, !revers);
    digitalWrite(pin_backward, revers);
    // Serial.println(power);
    
    // if(get_speed(false) <= 10)
    //     if(pid.get_target_value() == 0) {
    //         ledcWrite(num_PWM_channel, 0);
    //         digitalWrite(pin_forward, false);
    //         digitalWrite(pin_backward, false);
    //     }
    return false;
}

bool Engine::set_power(int16_t power_PWM) {
    if(power_PWM < 0)
        return set_power(-power_PWM, true);
    return set_power(power_PWM, false);
}

void Engine::set_target_speed(int32_t speed) {
    pid.set_target_value(speed);
}

int32_t Engine::get_target_speed() {
    return pid.get_target_value();
}