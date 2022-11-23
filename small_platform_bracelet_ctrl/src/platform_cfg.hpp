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

void init_esp_now(void);
void esp_now_callback(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void servo_set_position(uint8_t angle);
void commands_accomplishing(void);
void check_time_btw_cmds(void);

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

// --- Аккумулятор ---
constexpr uint8_t ACCUM_PIN = GPIO_NUM_14;