#include <Arduino.h>

#include <Engine.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Set_Serial.h>
#include <Get_data.h>
#include <ESP32_Servo.h>

unsigned long timing_adc;
unsigned long timing_us;
void get_accum_charge();
void servo_set_position(uint8_t angle);
void set_speed_serial();
void work_with_CAN();
void send_message();
void read_light();

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

// --- CAN шина ---
CAN_device_t CAN_cfg;             // Конфигурация CAN
unsigned long previousMillis = 0; // Время последней отправки CAN сообщения
const int interval = 500;         // Интервал отправки CAN сообщений (мс)
const int rx_queue_size = 10;     // Размер буфера приёма
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_19;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_23;

// --- UART2 шина ---
constexpr uint8_t UART_TX_PIN = GPIO_NUM_16;
constexpr uint8_t UART_RX_PIN = GPIO_NUM_17;
constexpr uint16_t UART_BAUD = 38400;

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

bool receive_flag = false;
uint32_t receive_time = 0;

// Объекты двигателей
Engine engine_right;
Engine engine_left;

char us_data[19] = "000000000000000000";

// Счётчики энкодеров двигателей (прерывания по сигналу)
void hall_left()
{
    engine_left.interrupt();
}
void hall_right()
{
    engine_right.interrupt();
}

// Остановка двигателей
void stop()
{
    engine_left.set_target_speed(0);
    engine_right.set_target_speed(0);
}

Servo myservo;
int pos = 0;

void setup()
{
    // Запуск UART'ов
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(100);

    ledcSetup(CHANNEL_UV, 1000, 10);
    ledcAttachPin(UV_LIGHT1, CHANNEL_UV);

    ledcSetup(CHANNEL_LIGHT, 1000, 10);
    ledcAttachPin(UV_LIGHT2, CHANNEL_LIGHT);

    ledcSetup(CHANNEL_SERVO, SERVO_FREQUENCY, SERVO_RESOLUTION);
    ledcAttachPin(SERVO_PIN, CHANNEL_SERVO);

    // servo_set_position(110);
    //  Инициализация УФ-диода
    //  pinMode(UV_LIGHT1, OUTPUT);
    //  pinMode(UV_LIGHT2, OUTPUT);

    pinMode(ACCUM_PIN, INPUT);

    // Инициализация двигателей
    engine_left.init(LEFT_WHEEL_FORWARD, LEFT_WHEEL_BACKWARD, LEFT_WHEEL_PWM, LEFT_ENCODER, CHANNEL_LEFT_WHELL, hall_left);
    engine_right.init(RIGHT_WHEEL_FORWARD, RIGHT_WHEEL_BACKWARD, RIGHT_WHEEL_PWM, RIGHT_ENCODER, CHANNEL_RIGHT_WHELL, hall_right);
    // engine_left.init(LEFT_WHEEL_FORWARD, LEFT_WHEEL_BACKWARD, LEFT_WHEEL_PWM, LEFT_ENCODER, CHANNEL_LEFT_WHELL);
    // engine_right.init(RIGHT_WHEEL_FORWARD, RIGHT_WHEEL_BACKWARD, RIGHT_WHEEL_PWM, RIGHT_ENCODER, CHANNEL_RIGHT_WHELL);
    engine_left.begin();
    engine_right.begin();
    stop();
    // Инициализация CAN шины
    CAN_cfg.tx_pin_id = CAN_TX_PIN;
    CAN_cfg.rx_pin_id = CAN_RX_PIN;
#if defined(WIFI_LoRa_32) || defined(WIFI_LoRa_32_V2)
    // ESP типа heltec_wifi_lora_32 или heltec_wifi_lora_32_v2
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
#elif ~(defined(WIFI_LoRa_32) || defined(WIFI_LoRa_32_V2))
    // Другие ESP
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
#endif
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    ESP32Can.CANInit();
    delay(1000);
    ledcWrite(CHANNEL_SERVO, 63);
    delay(1000);
}

void loop()
{
    work_with_CAN();
    // delay(200);
    set_speed_serial();
    //delay(5);
    //Serial.println("--------------");
    if (millis() - timing_adc > charge_send_time)
    { // Вместо 10000 подставьте нужное вам значение паузы
        timing_adc = millis();
        get_accum_charge();
        //Serial.println("ACC");
    }
    if (millis() - timing_us > us_send_time)
    { // Вместо 10000 подставьте нужное вам значение паузы
        timing_us = millis();
        us_data[0] = 'S';
        us_data[1] = 'U';
        us_data[2] = ((us_l / 100) % 10) + '0';
        us_data[3] = ((us_l / 10) % 10) + '0';
        us_data[4] = (us_l % 10) + '0';
        us_data[5] = ((us_fl / 100) % 10) + '0';
        us_data[6] = ((us_fl / 10) % 10) + '0';
        us_data[7] = (us_fl % 10) + '0';
        us_data[8] = ((us_f / 100) % 10) + '0';
        us_data[9] = ((us_f / 10) % 10) + '0';
        us_data[10] = (us_f % 10) + '0';
        us_data[11] = ((us_fr / 100) % 10) + '0';
        us_data[12] = ((us_fr / 10) % 10) + '0';
        us_data[13] = (us_fr % 10) + '0';
        us_data[14] = ((us_r / 100) % 10) + '0';
        us_data[15] = ((us_r / 10) % 10) + '0';
        us_data[16] = (us_r % 10) + '0';
        us_data[17] = 'E';
        Serial.println(us_data);
        // Serial.println(us_f);
        // Serial.println(us_l);
        // Serial.println(us_r);
        // Serial.println(us_fr);
        // Serial.println(us_fl);
    }
    //     set_speed_serial();
    // if (millis() - timing > charge_send_time){ // Вместо 10000 подставьте нужное вам значение паузы
    //     timing = millis();
    //     get_accum_charge();
    // }
    // servo_set_position(120);
    // delay(1000);
    // servo_set_position(90);
    // delay(100);
}

#define ERROR 30000
#define MAX 16

uint8_t buffer[MAX];
uint8_t buffer2[MAX];
char buffer3[6];
int16_t val1 = 0, val2 = 0;
uint8_t angle = 90;

void set_speed_serial()
{
    Set_Serial set_param;
    // work_with_CAN();

    // if (Serial.available() != 0)
    // {
    //     if (!receive_flag)
    //     {
    //         receive_flag = true;
    //         receive_time = millis();
    //     }
    // }
    // if (receive_flag && (millis() - receive_time > 1U))
    // {
    //if (Serial.available() != 0) {
    if (Serial.read() == 'Z') {
        receive_flag = false;
        for (int i = 0; i < MAX; ++i)
        {
            buffer[i] = 0;
        }
        uint8_t size = Serial.readBytes(&buffer[0], MAX);
        int16_t cmd = set_param.read_command(buffer, size);
        if (cmd < ERROR)
        {
            switch (cmd)
            {
            case 0:
                break;
            case 1:
                // Serial2.write(buffer, MAX);
                val1 = set_param.check_error_t(&buffer[3], 0);
                val2 = set_param.check_error_t(&buffer[9], 1);
                if (val1 < ERROR)
                {
                    // engine_left.set_target_speed(val1);
                    if (val1 > 100) {
                        val1 = 100;
                    }
                    if (val1 < -100) {
                        val1 = -100;
                    }
                    engine_left.set_power(val1 * 30);
                }
                else
                {
                    Serial.print("!1!");
                    Serial.println("val1");
                }
                if (val2 < ERROR)
                {
                    // engine_right.set_target_speed(val2);
                    if (val2 > 100) {
                        val2 = 100;
                    }
                    if (val2 < -100) {
                        val2 = -100;
                    }
                    engine_right.set_power(val2 * 30);
                }
                else
                {
                    Serial.print("!2!");
                    Serial.println("val2");
                }
                Serial.println("wheel");
                break;
            case 2:
                val1 = set_param.check_error_c(&buffer[3], 0, buffer[2]);
                val2 = set_param.check_error_c(&buffer[9], 1, buffer[2]); //проверка на L/R (в самой функции)
                if (val1 < ERROR)
                {
                    if (buffer[2] == 'L' || buffer[2] == 'l')
                    {
                        if (val1 < ERROR && val2 < ERROR)
                        {
                            engine_left.set_coefficient(val1, 0, val2);
                        }
                        else if (val1 < ERROR && val2 >= ERROR)
                        {
                            engine_left.set_coefficient(val1, 0, 10000);
                        }
                        else if (val2 < ERROR && val1 >= ERROR)
                        {
                            engine_left.set_coefficient(10000, 0, val2);
                        }
                        else
                        {
                            Serial.print("!3!");
                            Serial.println(val1);
                            Serial.println(val2);
                        }
                    }
                    else
                    {
                        if (val1 < ERROR && val2 < ERROR)
                        {
                            engine_right.set_coefficient(val1, 0, val2);
                        }
                        else if (val1 < ERROR && val2 >= ERROR)
                        {
                            engine_right.set_coefficient(val1, 0, 10000);
                        }
                        else if (val2 < ERROR && val1 >= ERROR)
                        {
                            engine_right.set_coefficient(10000, 0, val2);
                        }
                        else
                        {
                            Serial.print("!4!");
                            Serial.println(val1);
                            Serial.println(val2);
                        }
                    }
                }
                else
                {
                    Serial.print("!5!");
                    Serial.println(val1);
                }
                break;
            case 3:
                val1 = set_param.check_error_f(&buffer[3]);
                if (val1 < ERROR)
                {
                    // freq_send = val1; (-) -----
                }
                else
                {
                    Serial.print("!6!");
                    Serial.println(val1);
                }
                break;
            case 4:
                // val1 = set_param.check_error_f(&buffer[3]);

                if (buffer[2] == '+')
                {
                    uint8_t uv_power = (buffer[4] - '0') * 100 + (buffer[5] - '0') * 10 + buffer[6] - '0';
                    uint16_t uv_pwm = map(uv_power, 0, 100, 0, 1023);
                    ledcWrite(CHANNEL_UV, uv_pwm);
                    // digitalWrite(UV_LIGHT2, HIGH);
                }
                else if (buffer[2] == '-')
                {
                    ledcWrite(CHANNEL_UV, 0);
                    // digitalWrite(UV_LIGHT2, LOW);
                }
                if (buffer[3] == '+')
                {
                    uint8_t light_power = (buffer[7] - '0') * 100 + (buffer[8] - '0') * 10 + buffer[9] - '0';
                    uint16_t light_pwm = map(light_power, 0, 100, 0, 1023);
                    ledcWrite(CHANNEL_LIGHT, light_pwm);
                }
                else if (buffer[3] == '-')
                {
                    ledcWrite(CHANNEL_LIGHT, 0);
                    // digitalWrite(UV_LIGHT2, LOW);
                }
                break;
            case 5:
                angle = (buffer[2] - '0') * 100 + (buffer[3] - '0') * 10 + buffer[4] - '0';
                // Serial.println(angle);
                servo_set_position(angle);
                // Serial.println("Servo");
                break;

            default:
                Serial.print("!7!");
                Serial.println("Unknown error");
                break;
            }
        }
        else
        {
            Serial.print("!8!");
            Serial.println(cmd);
        }
        val1 = 0;
        val2 = 0;
    }
}

void read_light()
{
    uint8_t num = 0;
    if (Serial2.available() != 0)
    {
        for (int i = 0; i < MAX; ++i)
        {
            buffer2[i] = 0;
        }
        delay(5);
        Serial2.readBytes(&buffer2[num], 1);
        if (buffer2[num++] != 'S')
            return;
        Serial2.readBytes(&buffer2[num], 6);
        if (buffer2[num++] != 'L')
            return;
        for (int i = 0; i < 4; ++i)
        {
            if (buffer2[num] < '0' || buffer2[num] > '9')
                return;
            ++num;
        }
        if (buffer2[num++] != 'E')
            return;
        Serial.write(buffer2, num);
        Serial.println();
    }
}

void work_with_CAN()
{
    CAN_frame_t rx_frame;
    Get_data data;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
        //Serial.println("CAN");
        // if (rx_frame.MsgID == ID_RFID) {
        if (rx_frame.MsgID == ID_US_F) {
            // Serial.println("CAN Work forward");
            // Serial.println(rx_frame.data.u8[0]);
            us_f = rx_frame.data.u8[0];
        }
        else if (rx_frame.MsgID == ID_US_L) {
            us_l = rx_frame.data.u8[0];
        }

        else if (rx_frame.MsgID == ID_US_R) {
            // Serial.println("CAN Work forward");
            // Serial.println(rx_frame.data.u8[0]);
            us_r = rx_frame.data.u8[0];
        }
        else if (rx_frame.MsgID == ID_US_FR) {
            us_fr = rx_frame.data.u8[0];
        }
        else if (rx_frame.MsgID == ID_US_FL) {
            us_fl = rx_frame.data.u8[0];
        }
        data.print(rx_frame.data.u8, rx_frame.MsgID);
        //}
        // Serial.println("CAN Work");
        // Serial.println(rx_frame.MsgID);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          ё(rx_frame.data.u8, rx_frame.MsgID);
    }
}

void servo_set_position(uint8_t angle)
{
    // if (angle < 30)
    //     angle = 30;
    // else if (angle > 150)
    //     angle = 150;
    // uint16_t val = map(angle, 0, 100, 26, 127);
    uint16_t val = map(angle, 0, 100, 36, 90);
    //Serial.println(val);
    ledcWrite(CHANNEL_SERVO, val);
}

void get_accum_charge()
{
    long accum_adc = analogRead(ACCUM_PIN);
    Serial.println(accum_adc);
    const long out_max = 100;
    const long out_min = 0;
    // const long in_max = 3400;
    // const long in_min = 2400;
    // const long dividend = out_max - out_min;
    // const long divisor = in_max - in_min;
    // if (accum_adc > 3400) {accum_adc = 3400;}
    // if (accum_adc < 2400) {accum_adc = 2400;}
    // const long in_max = 2070;
    // const long in_min = 1820;
    // const long in_max = 1700;
    // const long in_min = 1475;
    const long in_max = 3200;
    const long in_min = 2750;
    const long dividend = out_max - out_min;
    const long divisor = in_max - in_min;
    if (accum_adc > in_max)
    {
        accum_adc = in_max;
    }
    if (accum_adc < in_min)
    {
        accum_adc = in_min;
    }

    const long delta = accum_adc - in_min;
    uint8_t charge_in_percent = 0;
    charge_in_percent = static_cast<uint8_t>((delta * dividend + (divisor / 2)) / divisor + out_min);
    char cstr[3];
    for (int j = 0; j < 3; j++)
    {
        cstr[2 - j] = charge_in_percent % 10 + '0';
        charge_in_percent /= 10;
    }
    for (int i = 0; i < 6; ++i)
    {
        buffer3[i] = 0;
    }
    buffer3[0] = 'S';
    buffer3[1] = 'A';
    buffer3[2] = cstr[0];
    buffer3[3] = cstr[1];
    buffer3[4] = cstr[2];
    buffer3[5] = 'E';
    for (int i = 0; i < 6; i++)
    {
        Serial.print(buffer3[i]);
    }
    Serial.println();
}
