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
#define TRACK_SPEED_FAST    25   // 直道高速  Straight fast
#define TRACK_SPEED_MED     18   // 中速转弯  Medium turn
#define TRACK_SPEED_SLOW    12   // 慢速转弯  Slow turn
#define TRACK_SPEED_SHARP    8   // 急转弯    Sharp turn
#define TRACK_SPEED_MIN      4   // 原地微调  In-place adjust
//=====================================================================================

/**
 * @brief 巡线控制函数 — 分支判断法 (LINE_RAW_VALUE=踩线值)
 *        Line following — branch-based
 */
void track(void)
{
    uint8_t d1 = D1, d2 = D2, d3 = D3, d4 = D4;
    uint8_t d5 = D5, d6 = D6, d7 = D7, d8 = D8;
    #define ON  LINE_RAW_VALUE     // 踩线  on line
    #define OFF !LINE_RAW_VALUE    // 离线  off line

    int16_t L, R;

    // ============================================================
    // 情况1: 正中间 (X4或X5踩线) — 直行
    // ============================================================
    if ((d4 == ON && d5 == ON) || (d4 == ON && d5 != ON) || (d4 != ON && d5 == ON))
    {
        strcpy(g_track_state, "GO  ");
        L =  TRACK_SPEED_FAST;
        R =  TRACK_SPEED_FAST;
    }
    else if (d3 == ON && d4 != ON)
    {
        strcpy(g_track_state, "R1  ");
        L =  TRACK_SPEED_SLOW;
        R =  TRACK_SPEED_FAST;
    }
    else if (d2 == ON)
    {
        strcpy(g_track_state, "R2  ");
        L =  TRACK_SPEED_SHARP;
        R =  TRACK_SPEED_MED;
    }
    else if (d1 == ON)
    {
        strcpy(g_track_state, "R3!!");
        L =  TRACK_SPEED_MIN;
        R =  TRACK_SPEED_FAST;
    }
    else if (d6 == ON && d5 != ON)
    {
        strcpy(g_track_state, "L1  ");
        L =  TRACK_SPEED_FAST;
        R =  TRACK_SPEED_SLOW;
    }
    else if (d7 == ON)
    {
        strcpy(g_track_state, "L2  ");
        L =  TRACK_SPEED_MED;
        R =  TRACK_SPEED_SHARP;
    }
    else if (d8 == ON)
    {
        strcpy(g_track_state, "L3!!");
        L =  TRACK_SPEED_FAST;
        R =  TRACK_SPEED_MIN;
    }
    else if (d1 == OFF && d2 == OFF && d3 == OFF && d4 == OFF
          && d5 == OFF && d6 == OFF && d7 == OFF && d8 == OFF)
    {
        strcpy(g_track_state, "FIND");    // 全白=丢线   All white = lost
        L = -TRACK_SPEED_SLOW;
        R =  TRACK_SPEED_SLOW;
    }
    else if (d1 == ON && d2 == ON && d3 == ON && d4 == ON
          && d5 == ON && d6 == ON && d7 == ON && d8 == ON)
    {
        strcpy(g_track_state, "CROSS");   // 全黑=十字   All black = cross
        L =  TRACK_SPEED_FAST;
        R =  TRACK_SPEED_FAST;
    }
    else
    {
        strcpy(g_track_state, "----");
        L =  TRACK_SPEED_MED;
        R =  TRACK_SPEED_MED;
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