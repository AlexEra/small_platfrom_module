#ifndef __ROBOTIC_PLATFORM_H__
#define __ROBOTIC_PLATFORM_H__

#include <Arduino.h>
#include <Engine.h>
#include <Ultrasonic_sensor.h>
#include <Infrared_sensor.h>


enum class Movement {
    forward,                 // движение прямо:    вперед
    backward,                // движение прямо:    назад
    forward_rotation_left,   // движение по дуге:  вперёд налево
    forward_rotation_right,  // движение по дуге:  вперёд направо
    backward_rotation_left,  // движение по дуге:  назад налево
    backward_rotation_right, // движение по дуге:  назад направо
    left_reversal,           // разворот на месте: налево
    right_reversal           // разворот на месте: направо
};

class Robotic_platform {
private:
    // Двигатели
    Engine engine_right;
    Engine engine_left;
    // УЗ датчики
    Ultrasonic_sensor US_left;
    Ultrasonic_sensor US_right;
    Ultrasonic_sensor US_forward;
    Ultrasonic_sensor US_backward;
    // ИК датчики
    Infrared_sensor IK_left;
    Infrared_sensor IK_right;

    // Данные
    int32_t speed{};
    Movement movement{};
public:
    Robotic_platform();
    ~Robotic_platform();

    bool set_speed();
    bool set_movement();
    bool get_speed();
    bool get_movement();
};

#endif // __ROBOTIC_PLATFORM_H__
