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
#define TRACK_SPEED_FAST    45   // 直道/基准速度
#define TRACK_SPEED_SLOW    25   // 丢线搜索速度 (FIND时用)
#define TRACK_SPEED_MIN      4   // 内轮最低速 (调小=差速大, 调大=差速小)
// 加权差比和 ── 传感器权重 (外层权重大, 线偏得越多贡献越大)
#define W1 6  // D1 最左
#define W2 4  // D2
#define W3 2  // D3
#define W4 0  // D4 中心左
#define W5 0  // D5 中心右
#define W6 2  // D6
#define W7 4  // D7
#define W8 6  // D8 最右
// 转向PD ★ Kp已调低适配连续误差, UART3: KP/KD实时调
float g_track_kp = 6.0f;        // 转向P (误差-6~+6, 建议2~5)
float g_track_kd = 3.5f;        // 转向D (建议2~6)
#define STEER_MAX        35     // 转向输出上限 (最大差速 ≈ 这个值×2)
//=====================================================================================

int16_t g_speed_fast  = TRACK_SPEED_FAST;
int16_t g_speed_slow  = TRACK_SPEED_SLOW;
int16_t g_speed_min   = TRACK_SPEED_MIN;

/**
 * @brief 巡线控制函数 — 加权差比和 + 转向PD
 *
 *        三层架构:
 *          第1层: 8路传感器 → 加权差比和 → 连续error(-6~+6)
 *                 error = (sum_L - sum_R) / cnt
 *          第2层: FIND丢线处理 (保持上帧error走惯性)
 *          第3层: 转向PD (位置式PD, error→steer_out)
 *                 L = base - steer_out,  R = base + steer_out
 *
 *        调参命令: KP<值> / KD<值> (UART3)
 */
void track(void)
{
    uint8_t d1 = D1, d2 = D2, d3 = D3, d4 = D4;
    uint8_t d5 = D5, d6 = D6, d7 = D7, d8 = D8;

    int16_t L, R;

    // ============================================================
    // 第1层: 加权差比和 → 连续error (-6 ~ +6)
    //        sum_L = d1*6 + d2*4 + d3*2 + d4*1
    //        sum_R = d5*1 + d6*2 + d7*4 + d8*6
    //        error = (sum_L - sum_R) / cnt  (正=左偏需右转)
    // ============================================================
    int16_t sum_L = (int16_t)d1*W1 + d2*W2 + d3*W3 + d4*W4;
    int16_t sum_R = (int16_t)d5*W5 + d6*W6 + d7*W7 + d8*W8;
    uint8_t cnt   = d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;

    float raw_error;
    uint8_t branch;

    if (cnt == 0)            // 全白 → 丢线
    {
        branch = 7;
        raw_error = 0.0f;
    }
    else if (cnt == 8)       // 全黑 → 十字路口
    {
        branch = 8;
        raw_error = 0.0f;
    }
    else
    {
        // 连续误差: 左侧加权和 - 右侧加权和, 除以踩线数归一化
        raw_error = (float)(sum_L - sum_R) / (float)cnt;

        // OLED 状态: 按 |error| 阈值映射分支
        float abs_e = (raw_error > 0.0f) ? raw_error : -raw_error;
        if      (abs_e < 0.5f)  branch = 0;                               // GO
        else if (abs_e < 2.0f)  branch = (raw_error > 0.0f) ? 1 : 4;      // R1 / L1
        else if (abs_e < 4.0f)  branch = (raw_error > 0.0f) ? 2 : 5;      // R2 / L2
        else                    branch = (raw_error > 0.0f) ? 3 : 6;      // R3 / L3
    }

    // ============================================================
    // 第2层: FIND丢线处理 — 保持上帧error走惯性
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
