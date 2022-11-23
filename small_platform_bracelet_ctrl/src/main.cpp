#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Engine.h>
#include "platform_cfg.hpp"


imu_data_5B_s mpu_bracelet_data;
uint8_t cmd = 0;
unsigned long lightning_start_time = 0;
unsigned long commands_time = 0;
bool is_lightning_started = false;
bool is_bracelet_button_pressed = false;
bool cmd_light_off = false;

// Объекты двигателей
Engine engine_right;
Engine engine_left;

// Счётчики энкодеров двигателей (прерывания по сигналу)
void hall_left() {
    engine_left.interrupt();
}

void hall_right() {
    engine_right.interrupt();
}


void setup() {
    Serial.begin(115200);
    delay(10);

    ledcSetup(CHANNEL_UV, 1000, 10);
    ledcAttachPin(UV_LIGHT1, CHANNEL_UV);

    ledcSetup(CHANNEL_LIGHT, 2, 10);
    ledcAttachPin(UV_LIGHT2, CHANNEL_LIGHT);

    // ledcSetup(CHANNEL_SERVO, SERVO_FREQUENCY, SERVO_RESOLUTION);
    // ledcAttachPin(SERVO_PIN, CHANNEL_SERVO);

    pinMode(ACCUM_PIN, INPUT);

    // Инициализация двигателей
    engine_left.init(LEFT_WHEEL_FORWARD, LEFT_WHEEL_BACKWARD, LEFT_WHEEL_PWM, LEFT_ENCODER, CHANNEL_LEFT_WHELL, hall_left);
    engine_right.init(RIGHT_WHEEL_FORWARD, RIGHT_WHEEL_BACKWARD, RIGHT_WHEEL_PWM, RIGHT_ENCODER, CHANNEL_RIGHT_WHELL, hall_right);
    engine_left.begin();
    engine_right.begin();

    init_esp_now();
    esp_now_register_recv_cb(esp_now_callback);
    delay(1000);
}

void loop() {
    commands_accomplishing();
    check_time_btw_cmds();
}


void init_esp_now(void) {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error in initializing ESP-NOW");
        while (1);
    }
}

void commands_accomplishing(void) {
    int16_t pwm_left = 0;
    int16_t pwm_right = 0;
    if (cmd) commands_time = millis();
    switch (cmd) {
        case 0x30: // NEW_WHEELS_VELOCITY
            if (abs(mpu_bracelet_data.a_x) < 350) mpu_bracelet_data.a_x = 0;
            if (abs(mpu_bracelet_data.a_y) < 350) mpu_bracelet_data.a_y = 0;

            pwm_left = mpu_bracelet_data.a_x + mpu_bracelet_data.a_y;
            pwm_right = mpu_bracelet_data.a_x - mpu_bracelet_data.a_y;

            engine_left.set_power(pwm_left * 4.5);
            engine_right.set_power(pwm_right * 4.5);
            cmd = 0;
            break;
        case 0x04: // LIGHT_CONTROL
            /*if (!is_lightning_started) {
                is_lightning_started = true;
                lightning_start_time = millis();
                ledcWrite(CHANNEL_LIGHT, 512);
            }*/
            ledcWrite(CHANNEL_LIGHT, 1024);
            cmd = 0;
            break;
        case 0x01:
            ledcWrite(CHANNEL_LIGHT, 1024);
            engine_left.set_power(0);
            engine_right.set_power(0);
            delay(5000);
            cmd = 0;
            break;
        default:
            break;
    }

    if (cmd_light_off) {
        ledcWrite(CHANNEL_LIGHT, 0);
        cmd_light_off = false;
    }

    /*if (is_lightning_started) {
        unsigned long check_time = millis();
        if (check_time < lightning_start_time) check_time += (0xFFFFFFFF - lightning_start_time);
        if (check_time - lightning_start_time > 2000) {
            ledcWrite(CHANNEL_LIGHT, 0);
            is_lightning_started = false;
        }
    }*/
}

void check_time_btw_cmds(void) {
    unsigned long check_time = millis();

    if (check_time < commands_time) check_time += (0xFFFFFFFF - commands_time);
    if (check_time - commands_time > 1000) {
        engine_left.set_power(0);
        engine_right.set_power(0);
        delay(5000);
    }
}

/*
 *@brief Callback function that will be executed when data is received
*/
void esp_now_callback(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    switch ((*incomingData) & 0xFE) {
        case 0x90: // info about buttons and gestures 
            cmd = *(++incomingData);
            cmd = (cmd & 0x01) | (cmd & 0x04);
            if (cmd == 0) cmd_light_off = true;
            break;
        case 0x30: // for only two accelerations
            memcpy(&mpu_bracelet_data, incomingData, sizeof(mpu_bracelet_data));
            cmd = 0x30;
            break;
    }
}
