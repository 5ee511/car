#include "headfile.h"

// 全局数组：存储8路灰度传感器的数字值（0或1）
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

// 巡线状态(供OLED显示)
char g_track_state[8] = "INIT";

// 直角弯调试变量（OLED 转弯时显示）
uint8_t g_turn_active   = 0;
char    g_turn_dir_char = ' ';
uint8_t g_turn_frame_cnt = 0;
uint8_t g_turn_lcnt     = 0;
uint8_t g_turn_rcnt     = 0;
int16_t g_turn_speed_L  = 0;
int16_t g_turn_speed_R  = 0;

// ============================================================
//  重要: 黑线=1, 白=0
// ============================================================
#define LINE_RAW_VALUE       1

// ============================================================
//  巡线速度配置 (可调, UART3: S<值>)
// ============================================================
#define TRACK_SPEED_FAST    45   // 直道速度
#define TRACK_SPEED_SLOW    20   // 直角弯转弯速度 (内轮停转, 外轮用此速度)
#define TRACK_SPEED_MIN      2

// 加权权重 (直道微调用, 外层权重大)
#define W1 6
#define W2 4.2
#define W3 1.8
#define W4 0
#define W5 0
#define W6 1.8
#define W7 4.2
#define W8 6

// 转向PD (UART3: KP/KD实时调)
float g_track_kp = 5.8f;
float g_track_kd = 3.0f;
#define STEER_MAX        60

// 直角弯参数
#define TURN_TIMEOUT      200    // 转弯超时帧数 (2s@10ms), 防止死循环
// ============================================================

int16_t g_speed_fast  = TRACK_SPEED_FAST;
int16_t g_speed_slow  = TRACK_SPEED_SLOW;
int16_t g_speed_min   = TRACK_SPEED_MIN;

// 调试标签
const char* sensor_labels[GRAYSCALE_SENSOR_CHANNELS] = {
    "X1", "X2", "X3", "X4", "X5", "X6", "X7", "X8"
};

void sensor_test(void) {
    uint8_t i;
    printf("Test start\r\n");
    Grayscale_Sensor_Read_All(g_sensor_data);
    printf("Raw: ");
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        printf("%d ", g_sensor_data[i]);
    }
    printf("\r\n");
    printf("Dx:  ");
    for (i = 1; i <= GRAYSCALE_SENSOR_CHANNELS; i++) {
        printf("%d ", digtal(i));
    }
    printf("\r\n");
}

/**
 * @brief 巡线控制 v2.0 — 直角弯: 4灯触发 + 最外侧单灯退出
 *
 *  两状态:
 *    NORMAL  → 正常PD巡线, 检测直角弯触发条件
 *    TURNING → 开环差速转弯, 检测最外侧单灯退出条件
 *
 *  直角弯触发: 一侧 ≥3 灯同时踩线, 且另一侧全白
 *    左4灯(D1~D4)齐亮 → 左转
 *    右4灯(D5~D8)齐亮 → 右转
 *
 *  直角弯退出: 转弯侧最外侧单灯独亮
 *    左转中: 仅 D1==1 → 退出
 *    右转中: 仅 D8==1 → 退出
 */
void track(void)
{
    uint8_t d1 = D1, d2 = D2, d3 = D3, d4 = D4;
    uint8_t d5 = D5, d6 = D6, d7 = D7, d8 = D8;
    int16_t L, R;

    // ---- 传感器分组 ----
    uint8_t left_cnt  = d1 + d2 + d3 + d4;   // 左侧亮灯数
    uint8_t right_cnt = d5 + d6 + d7 + d8;   // 右侧亮灯数
    uint8_t cnt       = left_cnt + right_cnt;

    // ---- 直角弯触发条件: 一侧≥3灯亮, 另一侧≤1(容忍竖线残留) ----
    //  物理现实: 直角弯路口横线覆盖一侧全部传感器, 但旧竖线可能
    //  仍压住1~2个中心传感器, 所以另一侧不能要求严格==0
    uint8_t left_turn_trigger  = (left_cnt  >= 3 && right_cnt <= 1);
    uint8_t right_turn_trigger = (right_cnt >= 3 && left_cnt  <= 1);

    // ---- 转弯状态机 (static 保持跨帧) ----
    static uint8_t turning    = 0;   // 0=正常, 1=左转中, 2=右转中
    static uint8_t turn_frame = 0;   // 转弯持续帧计数

    // ---- 状态转移 ----
    if (!turning) {
        // 正常巡线中 → 检测直角弯
        if (left_turn_trigger)  { turning = 1; turn_frame = 0; }
        if (right_turn_trigger) { turning = 2; turn_frame = 0; }
    }

    if (turning) {
        turn_frame++;

        // 超时保护
        if (turn_frame > TURN_TIMEOUT) {
            turning = 0;
            turn_frame = 0;
        }
        // ★ 最小转弯帧数: 前3帧禁止退出, 保证转过足够角度
        else if (turn_frame <= 3) {
            // 刚触发, 继续转, 不检查退出条件
        }
        // 左转退出: 仅 D1==1 (最左侧单灯独亮, 且右侧全灭)
        else if (turning == 1 && d1 && !d2 && !d3 && !d4 && !d5 && !d6 && !d7 && !d8) {
            turning = 0;
            turn_frame = 0;
        }
        // 右转退出: 仅 D8==1
        else if (turning == 2 && !d1 && !d2 && !d3 && !d4 && !d5 && !d6 && !d7 && d8) {
            turning = 0;
            turn_frame = 0;
        }
    }

    // ---- 填直角弯调试变量 (main.c OLED 用) ----
    if (turning) {
        g_turn_active    = 1;
        g_turn_dir_char  = (turning == 1) ? 'L' : 'R';
        g_turn_frame_cnt = turn_frame;
        g_turn_lcnt      = left_cnt;
        g_turn_rcnt      = right_cnt;
    } else {
        g_turn_active    = 0;
    }

    // ---- 蜂鸣器: 转弯进出提示 (PB1, 低电平触发, 非阻塞帧计数) ----
    //   v2.1: 去掉 delay_ms(), 改用帧计数避免阻塞 track() 影响下次采样
    static uint8_t prev_turning = 0;
    static uint8_t beep_seq     = 0;   // 0=空闲, 1=进单响, 2=退第一响, 3=退间隔, 4=退第二响
    static uint8_t beep_tick    = 0;

    if (turning && !prev_turning) {
        beep_seq = 1; beep_tick = 0;
        gpio_set(GPIO_B, Pin_1, 0);              // 低电平=响
    }
    if (!turning && prev_turning) {
        beep_seq = 2; beep_tick = 0;
        gpio_set(GPIO_B, Pin_1, 0);              // 第一声
    }
    prev_turning = turning;

    if (beep_seq) {
        beep_tick++;
        if (beep_seq == 1 && beep_tick >= 3) {   // 进弯响3帧→关
            gpio_set(GPIO_B, Pin_1, 1);
            beep_seq = 0;
        } else if (beep_seq == 2 && beep_tick >= 2) { // 第一声2帧→停
            gpio_set(GPIO_B, Pin_1, 1);
            beep_seq = 3; beep_tick = 0;
        } else if (beep_seq == 3 && beep_tick >= 2) { // 间隔2帧→第二声
            gpio_set(GPIO_B, Pin_1, 0);
            beep_seq = 4; beep_tick = 0;
        } else if (beep_seq == 4 && beep_tick >= 2) { // 第二声2帧→关
            gpio_set(GPIO_B, Pin_1, 1);
            beep_seq = 0;
        }
    }

    // ============================================================
    // 转弯执行: 开环差速, 内轮停转+外轮低速
    // ============================================================
    if (turning == 1) {
        // 左转: 左轮=0(停), 右轮=低速前进
        L = 0;
        R = g_speed_slow;
        g_turn_speed_L = L;
        g_turn_speed_R = R;
        strcpy(g_track_state, "TURN");
        motor_target_set(L, R);
        return;
    }
    if (turning == 2) {
        // 右转: 右轮=0(停), 左轮=低速前进
        L = g_speed_slow;
        R = 0;
        g_turn_speed_L = L;
        g_turn_speed_R = R;
        strcpy(g_track_state, "TURN");
        motor_target_set(L, R);
        return;
    }

    // ============================================================
    // 正常巡线 (NORMAL) — 加权差比和 + PD
    // ============================================================
    int16_t sum_L = (int16_t)d1*W1 + d2*W2 + d3*W3 + d4*W4;
    int16_t sum_R = (int16_t)d5*W5 + d6*W6 + d7*W7 + d8*W8;

    float raw_error;
    uint8_t branch;

    if (cnt == 0) {
        branch = 7;  raw_error = 0.0f;
    } else if (cnt == 8) {
        branch = 8;  raw_error = 0.0f;
    } else {
        raw_error = (float)(sum_L - sum_R);
        float abs_e = (raw_error > 0.0f) ? raw_error : -raw_error;
        if      (abs_e < 1.5f)  branch = 0;
        else if (abs_e < 4.0f)  branch = (raw_error > 0.0f) ? 1 : 4;
        else if (abs_e < 7.5f)  branch = (raw_error > 0.0f) ? 2 : 5;
        else                    branch = (raw_error > 0.0f) ? 3 : 6;
    }

    // ---- FIND 丢线处理 ----
    float error = raw_error;
    static float   last_valid_error = 0.0f;
    static uint8_t ever_seen_line   = 0;

    if (cnt > 0) {
        ever_seen_line = 1;
        last_valid_error = raw_error;
    }

    if (cnt == 0) {
        if (!ever_seen_line) {
            L = -g_speed_slow;
            R =  g_speed_slow;
            strcpy(g_track_state, "FIND");
            motor_target_set(L, R);
            return;
        }
        error = last_valid_error;
    }

    // ---- 转向 PD ----
    static float prev_error = 0.0f;
    float diff = error - prev_error;
    prev_error = error;

    float steer_out = g_track_kp * error + g_track_kd * diff;
    if (steer_out >  STEER_MAX) steer_out =  STEER_MAX;
    if (steer_out < -STEER_MAX) steer_out = -STEER_MAX;

    // ---- 弯道减速 → 左右轮速度 ----
    float abs_err = (error > 0.0f) ? error : -error;
    float curve_factor = 1.0f - abs_err / 12.0f * 0.7f;
    int16_t base = (int16_t)((float)g_speed_fast * curve_factor);
    if (base < g_speed_min) base = g_speed_min;

    L = (int16_t)((float)base - steer_out);
    R = (int16_t)((float)base + steer_out);

    if (L < 0) L = 0;
    if (L > g_speed_fast) L = g_speed_fast;
    if (R < 0) R = 0;
    if (R > g_speed_fast) R = g_speed_fast;

    // ---- OLED 状态 ----
    switch (branch) {
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
 * @brief 获取指定通道的数字值
 */
unsigned char digtal(unsigned char channel)
{
    if (channel == 0 || channel > GRAYSCALE_SENSOR_CHANNELS) {
        return 0;
    }
    return (unsigned char)g_sensor_data[channel - 1];
}
