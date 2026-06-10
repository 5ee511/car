#include "headfile.h"

// 全局数组：存储8路灰度传感器的数字值（0或1），供巡线算法使用
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

// 巡线状态(供OLED显示)  Track state for OLED display
char g_track_state[8] = "INIT";  // 巡线状态字符串  Track state string

// 每个通道对应的标签，用于调试打印（可选用，可删除）
const char* sensor_labels[GRAYSCALE_SENSOR_CHANNELS] = {
    "X1", "X2", "X3", "X4", "X5", "X6", "X7", "X8"
};

/**
 * @brief 测试函数，读取所有传感器数据并打印（调试用）
 * @note  如果需要测试，取消注释调用函数
 */
/**

void sensor_test(void) {
    uint8_t i;
    printf("sensor_data:\r\n");
    Grayscale_Sensor_Read_All(g_sensor_data);
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        printf("%d ", g_sensor_data[i]);
    }
    printf("\r\n");
}
**/
void sensor_test(void) {
    uint8_t i;
    printf("Test start\r\n");
    Grayscale_Sensor_Read_All(g_sensor_data);
    
    // 打印原始数据值
    printf("Raw: ");
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        printf("%d ", g_sensor_data[i]);
    }
    printf("\r\n");
    
    // 打印 D1~D8 的值（通过 digtal 读取）
    printf("Dx:  ");
    for (i = 1; i <= GRAYSCALE_SENSOR_CHANNELS; i++) {
        printf("%d ", digtal(i));
    }
    printf("\r\n");
}

//=====================================================================================
//  重要: 黑线=1, 白=0  Black=1, White=0
//=====================================================================================
#define LINE_RAW_VALUE       1   // 踩线时传感器输出值  Sensor output when on line
//=====================================================================================
//  巡线速度配置 (可调)  Line-following speed config (adjustable)
//=====================================================================================
//                     ★ 所有可调参数都在这里 ★
//=====================================================================================
#define TRACK_SPEED_FAST    25   // 直道/基准速度
#define TRACK_SPEED_SLOW    14   // 丢线搜索速度 (FIND时用)
#define TRACK_SPEED_MIN      2   // 内轮最低速 (调小=差速大, 调大=差速小)
float g_track_kp = 18.0f;       // 转向P: 越大转弯越猛
float g_track_kd = 6.0f;        // 转向D: 越大越稳
#define STEER_MAX        18     // 转向输出上限 (最大差速 ≈ 这个值×2)
//=====================================================================================

int16_t g_speed_fast  = TRACK_SPEED_FAST;
int16_t g_speed_slow  = TRACK_SPEED_SLOW;
int16_t g_speed_min   = TRACK_SPEED_MIN;

/**
 * @brief 巡线控制函数 — 分支查表 + 转向PD + 防抖
 *
 *        三层架构:
 *          第1层: 8路传感器 → 状态分支 → error值(-6~+6)
 *          第2层: 防抖滤波 (连续2次相同分支才更新error)
 *          第3层: 转向PD (位置式PD, error→steer_out)
 *                 L = base - steer_out,  R = base + steer_out
 *
 *        调参命令: KP<值> / KD<值> (UART3)
 */
void track(void)
{
    uint8_t d1 = D1, d2 = D2, d3 = D3, d4 = D4;
    uint8_t d5 = D5, d6 = D6, d7 = D7, d8 = D8;
    #define ON  LINE_RAW_VALUE
    #define OFF !LINE_RAW_VALUE

    int16_t L, R;

    // ============================================================
    // 第1层: 分支查表 → 原始error值 (-6 ~ +6)
    // ============================================================
    float raw_error;
    uint8_t branch;

    if ((d4 == ON && d5 == ON) || (d4 == ON && d5 != ON) || (d4 != ON && d5 == ON))
    {
        branch = 0;  raw_error =  0.0f;   // GO  正中直行
    }
    else if (d3 == ON && d4 != ON)
    {
        branch = 1;  raw_error =  2.0f;   // R1  线偏左, 需右转
    }
    else if (d2 == ON)
    {
        branch = 2;  raw_error =  4.3f;   // R2  线偏左较多
    }
    else if (d1 == ON)
    {
        branch = 3;  raw_error =  6.0f;   // R3  线在最左, 急转
    }
    else if (d6 == ON && d5 != ON)
    {
        branch = 4;  raw_error = -2.0f;   // L1  线偏右, 需左转
    }
    else if (d7 == ON)
    {
        branch = 5;  raw_error = -4.3f;   // L2  线偏右较多
    }
    else if (d8 == ON)
    {
        branch = 6;  raw_error = -6.0f;   // L3  线在最右, 急转
    }
    else if (d1 == OFF && d2 == OFF && d3 == OFF && d4 == OFF
          && d5 == OFF && d6 == OFF && d7 == OFF && d8 == OFF)
    {
        branch = 7;  raw_error = 0.0f;    // FIND 全白丢线 (error保持)
    }
    else if (d1 == ON && d2 == ON && d3 == ON && d4 == ON
          && d5 == ON && d6 == ON && d7 == ON && d8 == ON)
    {
        branch = 8;  raw_error = 0.0f;    // CROSS 全黑十字
    }
    else
    {
        branch = 9;  raw_error = 0.0f;    // 兜底
    }

    // ============================================================
    // 第2层: 防抖 — 已禁用 (直接使用raw_error)
    // ============================================================
    float error = raw_error;
    static uint8_t ever_seen_line = 0;
    if (branch != 7) ever_seen_line = 1;

    // FIND丢线特殊处理: 保持上一帧error (顺着惯性走)
    static float last_valid_error = 0.0f;
    if (branch == 7)
    {
        if (!ever_seen_line)
        {
            L = -g_speed_slow;
            R =  g_speed_slow;
            strcpy(g_track_state, "FIND");
            motor_target_set(L, R);
            return;
        }
        error = last_valid_error;
    }
    else
    {
        last_valid_error = error;
    }

    // ============================================================
    // 第3层: 转向PD — error → steer_out
    //        公式: steer_out = Kp*error + Kd*(error - prev_error)
    // ============================================================
    static float prev_error = 0.0f;

    float diff  = error - prev_error;

    float steer_out = g_track_kp * error + g_track_kd * diff;
    prev_error = error;

    // 限幅
    if (steer_out >  STEER_MAX) steer_out =  STEER_MAX;
    if (steer_out < -STEER_MAX) steer_out = -STEER_MAX;

    // ============================================================
    // error → 左右轮速度 (差速转向)
    // ============================================================
    // 弯道减速: |error|越大, base越小
    float abs_err = (error > 0.0f) ? error : -error;
    float curve_factor = 1.0f - abs_err / 8.0f * 0.4f;
    int16_t base = (int16_t)((float)g_speed_fast * curve_factor);
    if (base < g_speed_min) base = g_speed_min;

    L = (int16_t)((float)base - steer_out);
    R = (int16_t)((float)base + steer_out);

    // 限幅
    int16_t wheel_max = g_speed_fast + (int16_t)STEER_MAX;
    if (L < g_speed_min) L = g_speed_min;
    if (L > wheel_max)   L = wheel_max;
    if (R < g_speed_min) R = g_speed_min;
    if (R > wheel_max)   R = wheel_max;

    // ============================================================
    // 状态显示 (OLED第2行)
    // ============================================================
    switch (branch)
    {
        case 0: strcpy(g_track_state, "GO  "); break;
        case 1: strcpy(g_track_state, "R1  "); break;
        case 2: strcpy(g_track_state, "R2  "); break;
        case 3: strcpy(g_track_state, "R3!!"); break;
        case 4: strcpy(g_track_state, "L1  "); break;
        case 5: strcpy(g_track_state, "L2  "); break;
        case 6: strcpy(g_track_state, "L3!!"); break;
        case 7: strcpy(g_track_state, "FIND"); break;
        case 8: strcpy(g_track_state, "CROSS");break;
        default:strcpy(g_track_state, "----"); break;
    }

    motor_target_set(L, R);
}

/**
 * @brief 获取指定通道的数字值（0或1）
 * @param channel 通道号 1~8
 * @return unsigned char 该通道的检测状态（0=黑线，1=白）
 */
unsigned char digtal(unsigned char channel)
{
    if (channel == 0 || channel > GRAYSCALE_SENSOR_CHANNELS) {
        return 0; // 无效通道，返回0（安全默认值）
    }
    return (unsigned char)g_sensor_data[channel - 1];
}
