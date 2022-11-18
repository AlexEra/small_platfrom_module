#include <Arduino.h>
#include <Engine.h>
/*#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Get_data.h>*/
#include <Set_Serial.h>
// #include <ESP32_Servo.h> // this library should be used instead of ledc!!
// #include <MPU9250.h>
#include <esp_now.h>
#include <WiFi.h>
#include "platform_cfg.hpp"

typedef struct {
    uint8_t idx;
    int16_t a_x;
    int16_t a_y;
} imu_data_5B_s;

typedef struct {
    uint8_t idx;
    int16_t a_x;
    int16_t a_y;
    int16_t a_z;
    int16_t w_x;
    int16_t w_y;
    int16_t w_z;
} imu_data_13B_s;

#define ERROR 30000
#define MAX 16


void init_esp_now(void);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void servo_set_position(uint8_t angle);
void set_speed_serial();
void send_message();
void mpu_process_data(uint8_t *p_acc, uint8_t *p_gyr);


// --- Моторы ---
// Пины для драйверов (M1, M2)
constexpr uint8_t M1_WHEEL_BACKWARD = GPIO_NUM_25; // = GPIO_NUM_27;
constexpr uint8_t M1_WHEEL_FORWARD = GPIO_NUM_33;  // = GPIO_NUM_26;
constexpr uint8_t M1_WHEEL_PWM = GPIO_NUM_32;      // = GPIO_NUM_25;
constexpr uint8_t M1_ENCODER1 = GPIO_NUM_36;
constexpr uint8_t M1_ENCODER2 = GPIO_NUM_39;

constexpr uint8_t M2_WHEEL_BACKWARD = GPIO_NUM_27; // = GPIO_NUM_14;
constexpr uint8_t M2_WHEEL_FORWARD = GPIO_NUM_26;  // = GPIO_NUM_12;
// constexpr uint8_t M2_WHEEL_PWM      = GPIO_NUM_19;
constexpr uint8_t M2_WHEEL_PWM = GPIO_NUM_13; // = GPIO_NUM_13;
constexpr uint8_t M2_ENCODER1 = GPIO_NUM_35;
constexpr uint8_t M2_ENCODER2 = GPIO_NUM_34;

// Подключение моторов к драйверам (L и R к M1 и M2)
// #define M1R_M2L
#define M2R_M1L

#if defined(M1R_M2L)
// R = M1
constexpr uint8_t RIGHT_WHEEL_BACKWARD = M1_WHEEL_BACKWARD;
constexpr uint8_t RIGHT_WHEEL_FORWARD = M1_WHEEL_FORWARD;
constexpr uint8_t RIGHT_WHEEL_PWM = M1_WHEEL_PWM;
constexpr uint8_t RIGHT_ENCODER = M1_ENCODER1;
// L = M2
constexpr uint8_t LEFT_WHEEL_BACKWARD = M2_WHEEL_BACKWARD;
constexpr uint8_t LEFT_WHEEL_FORWARD = M2_WHEEL_FORWARD;
constexpr uint8_t LEFT_WHEEL_PWM = M2_WHEEL_PWM;
constexpr uint8_t LEFT_ENCODER = M2_ENCODER1;
#elif defined(M2R_M1L)
// R = M2
constexpr uint8_t RIGHT_WHEEL_BACKWARD = M2_WHEEL_BACKWARD;
constexpr uint8_t RIGHT_WHEEL_FORWARD = M2_WHEEL_FORWARD;
constexpr uint8_t RIGHT_WHEEL_PWM = M2_WHEEL_PWM;
constexpr uint8_t RIGHT_ENCODER = M2_ENCODER1;
// L = M1
constexpr uint8_t LEFT_WHEEL_BACKWARD = M1_WHEEL_BACKWARD;
constexpr uint8_t LEFT_WHEEL_FORWARD = M1_WHEEL_FORWARD;
constexpr uint8_t LEFT_WHEEL_PWM = M1_WHEEL_PWM;
constexpr uint8_t LEFT_ENCODER = M1_ENCODER1;
#endif

// Номера используемых ШИМ-каналов
constexpr uint8_t CHANNEL_RIGHT_WHELL = 0;
constexpr uint8_t CHANNEL_LEFT_WHELL = 1;
constexpr uint8_t CHANNEL_SERVO = 7;
constexpr uint8_t CHANNEL_UV = 3;
constexpr uint8_t CHANNEL_LIGHT = 4;

// --- УФ диод ---
constexpr uint8_t UV_LIGHT1 = GPIO_NUM_17;
constexpr uint8_t UV_LIGHT2 = GPIO_NUM_18;

// --- Сервопривод ---
constexpr uint8_t SERVO_PIN = GPIO_NUM_4;
constexpr uint8_t SERVO_FREQUENCY = 50;
constexpr uint8_t SERVO_RESOLUTION = 10;

uint8_t us_f = 0;
uint8_t us_l = 0;
uint8_t us_r = 0;
uint8_t us_fl = 0;
uint8_t us_fr = 0;

// --- Аккумулятор ---
constexpr uint8_t ACCUM_PIN = GPIO_NUM_14;
uint32_t charge = 0;
uint8_t charge_in_percent = 0;
uint32_t charge_send_time = 10000;
uint32_t us_send_time = 100;
uint32_t sp_send_time = 500;

bool receive_flag = false;
uint32_t receive_time = 0;
int16_t val1 = 0, val2 = 0;
uint8_t buffer[MAX];


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
    // Запуск UART'ов
    Serial.begin(115200);
    delay(10);

    ledcSetup(CHANNEL_UV, 1000, 10);
    ledcAttachPin(UV_LIGHT1, CHANNEL_UV);

    ledcSetup(CHANNEL_LIGHT, 1000, 10);
    ledcAttachPin(UV_LIGHT2, CHANNEL_LIGHT);

    ledcSetup(CHANNEL_SERVO, SERVO_FREQUENCY, SERVO_RESOLUTION);
    ledcAttachPin(SERVO_PIN, CHANNEL_SERVO);

    pinMode(ACCUM_PIN, INPUT);

    // Инициализация двигателей
    engine_left.init(LEFT_WHEEL_FORWARD, LEFT_WHEEL_BACKWARD, LEFT_WHEEL_PWM, LEFT_ENCODER, CHANNEL_LEFT_WHELL, hall_left);
    engine_right.init(RIGHT_WHEEL_FORWARD, RIGHT_WHEEL_BACKWARD, RIGHT_WHEEL_PWM, RIGHT_ENCODER, CHANNEL_RIGHT_WHELL, hall_right);
    engine_left.begin();
    engine_right.begin();

    init_esp_now();
    esp_now_register_recv_cb(OnDataRecv);
    delay(1000);
}

void loop() {
    set_speed_serial(); // it should be rewritten!!
}

// it should be rewritten!!
void set_speed_serial() {
    Set_Serial set_param;

    buffer[0] = Serial.read();
    if (buffer[0] == 'S') {
        receive_flag = false;
        for (int i = 1; i < MAX; ++i) {
            buffer[i] = 0;
        }
        uint8_t size = Serial.readBytes(&buffer[1], MAX - 1) + 1;
        int16_t cmd = set_param.read_command(buffer, size);
        
        if (cmd < ERROR) {
            switch (cmd) {
                case 0:
                    break;
                case 1:
                    val1 = set_param.check_error_t(&buffer[3], 0);
                    val2 = set_param.check_error_t(&buffer[9], 1);
                    if (val1 < ERROR) {
                        if (val1 > 100) {
                            val1 = 100;
                        } else if (val1 < -100) {
                            val1 = -100;
                        }
                        engine_left.set_power(val1 * 30);
                    }

                    if (val2 < ERROR) {
                        if (val2 > 100) {
                            val2 = 100;
                        } else if (val2 < -100) {
                            val2 = -100;
                        }
                        engine_right.set_power(val2 * 30);
                    }
                    break;
                case 2:
                    val1 = set_param.check_error_c(&buffer[3], 0, buffer[2]);
                    val2 = set_param.check_error_c(&buffer[9], 1, buffer[2]); //проверка на L/R (в самой функции)
                    if (val1 < ERROR) {
                        if (buffer[2] == 'L' || buffer[2] == 'l') {
                            if (val1 < ERROR && val2 < ERROR) {
                                engine_left.set_coefficient(val1, 0, val2);
                            } else if (val1 < ERROR && val2 >= ERROR) {
                                engine_left.set_coefficient(val1, 0, 10000);
                            } else if (val2 < ERROR && val1 >= ERROR) {
                                engine_left.set_coefficient(10000, 0, val2);
                            }
                        } else {
                            if (val1 < ERROR && val2 < ERROR) {
                                engine_right.set_coefficient(val1, 0, val2);
                            } else if (val1 < ERROR && val2 >= ERROR) {
                                engine_right.set_coefficient(val1, 0, 10000);
                            } else if (val2 < ERROR && val1 >= ERROR) {
                                engine_right.set_coefficient(10000, 0, val2);
                            }
                        }
                    }
                    break;
                case 3:
                    val1 = set_param.check_error_f(&buffer[3]);
                    break;
                case 4:
                    if (buffer[2] == '+') {
                        uint8_t uv_power = (buffer[4] - '0') * 100 + (buffer[5] - '0') * 10 + buffer[6] - '0';
                        uint16_t uv_pwm = map(uv_power, 0, 100, 0, 1023);
                        ledcWrite(CHANNEL_UV, uv_pwm);
                    } else if (buffer[2] == '-') ledcWrite(CHANNEL_UV, 0);

                    if (buffer[3] == '+') {
                        uint8_t light_power = (buffer[7] - '0') * 100 + (buffer[8] - '0') * 10 + buffer[9] - '0';
                        uint16_t light_pwm = map(light_power, 0, 100, 0, 1023);
                        ledcWrite(CHANNEL_LIGHT, light_pwm);
                    } else if (buffer[3] == '-') ledcWrite(CHANNEL_LIGHT, 0);
                    break;
                case 5:
                    uint8_t angle = (buffer[2] - '0') * 100 + (buffer[3] - '0') * 10 + buffer[4] - '0';
                    servo_set_position(angle);
                    break;
            }
        }
        val1 = 0;
        val2 = 0;
    }
}

/**@brief Set angle to the servo in degrees
*/
void servo_set_position(uint8_t angle) {
    angle += 20;
    if (angle > 180) angle = 180;
    uint16_t val = map(angle, 180, 0, 2400, 544) * 1024 / 20000;
    ledcWrite(CHANNEL_SERVO, val);
}

void init_esp_now(void) {
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error in initializing ESP-NOW");
        while (1) {
            digitalWrite(LED_3, 1);
            digitalWrite(LED_4, 1);
            delay(500);
            digitalWrite(LED_3, 0);
            digitalWrite(LED_4, 0);
            delay(500);
        }
    } else {
        for (uint8_t i = 0; i < 7; i++) {
            digitalWrite(LED_3, 1);
            digitalWrite(LED_4, 0);
            delay(100);
            digitalWrite(LED_3, 0);
            digitalWrite(LED_4, 1);
            delay(100);
        }
        digitalWrite(LED_3, 0);
        digitalWrite(LED_4, 0);
    }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    // for (uint8_t i = 0; i < len; i++) {
    //     Serial.print(*incomingData); Serial.print(',');
    //     incomingData++;
    // }
    // Serial.println();

    switch ((*incomingData) & 0xFE) {
        case 0x50: // charge info
        case 0x90: // info about buttons and gestures 
            Serial.print(*incomingData); // device id + message id
            Serial.print(',');
            Serial.print(*(++incomingData)); // value
            Serial.println(',');
            break;
        case 0x30: {// for only two accelerations
                imu_data_5B_s tmp_struct;
                memcpy(&tmp_struct, incomingData, sizeof(tmp_struct));
                Serial.print(tmp_struct.idx); Serial.print(',');
                Serial.print(tmp_struct.a_x); Serial.print(',');
                Serial.print(tmp_struct.a_y); Serial.println(',');
            }
            break;
        case 0x34: {// for all mpu data
                imu_data_13B_s tmp_struct;
                memcpy(&tmp_struct, incomingData, sizeof(tmp_struct));
                Serial.print(tmp_struct.idx); Serial.print(',');
                Serial.print(tmp_struct.a_x); Serial.print(',');
                Serial.print(tmp_struct.a_y); Serial.print(',');
                Serial.print(tmp_struct.a_z); Serial.print(',');
                Serial.print(tmp_struct.w_x); Serial.print(',');
                Serial.print(tmp_struct.w_y); Serial.print(',');
                Serial.print(tmp_struct.w_z); Serial.println(',');
            }
            break;
    }
}