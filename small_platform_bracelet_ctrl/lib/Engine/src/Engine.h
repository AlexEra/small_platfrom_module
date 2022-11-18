#ifndef __ENGINE_H__
#define __ENGINE_H__

#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <PID_controller.h>

class Engine {
private:
    std::function<void(void)> encoder_fun;
    PID_controller pid{};
    uint32_t encoder_counter{};
    uint16_t power{};
    uint16_t max_power{};
    uint8_t pin_encoder;
    uint8_t pin_forward;
    uint8_t pin_backward;
    uint8_t pin_PWM;
    uint8_t num_PWM_channel;
    bool reverse_movement{};
    bool init_state{false};
public:
    // Engine(uint8_t pin_forward, uint8_t pin_backward, uint8_t pin_PWM, uint8_t pin_encoder,
    //        uint8_t num_PWM_channel, std::function<void(void)> interrupt_fun, uint16_t max_power=4095);
    Engine()=default;
    ~Engine()=default;

    bool init(uint8_t pin_forward, uint8_t pin_backward, uint8_t pin_PWM, uint8_t pin_encoder,
               uint8_t num_PWM_channel, std::function<void(void)> interrupt_fun=nullptr, uint16_t max_power=4095);
    bool begin();
    bool begin(uint8_t pin_forward, uint8_t pin_backward, uint8_t pin_PWM, uint8_t pin_encoder,
               uint8_t num_PWM_channel, std::function<void(void)> interrupt_fun=nullptr, uint16_t max_power=4095);
    // Прерывания датчика Холла
    void interrupt();
    uint32_t get_counter(bool clear=true);
    int32_t get_speed(bool clear=false);
    // ПИД-регулятор
    void PID_calculate();
    void set_coefficient(int16_t kp, int16_t ki, int16_t kd);
    void set_coefficient(float kp, float ki, float kd);
    void set_iteration_time(uint32_t dt_ms);

    // Управление двигателем
    uint16_t get_power() const;
    bool get_revers() const;
    bool set_power(uint16_t power, bool revers);
    bool set_power(int16_t power);
    void set_target_speed(int32_t speed);
    int32_t get_target_speed();
};

#endif // __ENGINE_H__
