#ifndef __gray_track_h_
#define __gray_track_h_
#include "grayscale_sensor.h"   // 确保宏定义可见
#include "headfile.h"

#define D1 digtal(1)
#define D2 digtal(2)
#define D3 digtal(3)
#define D4 digtal(4)
#define D5 digtal(5)
#define D6 digtal(6)
#define D7 digtal(7)
#define D8 digtal(8)

void track(void);
void track_reset(void);              // 复位巡线内部状态 (跨步骤切换用)
extern char g_track_state[8];      // 巡线状态  Track state string
void sensor_test(void);
unsigned char digtal(unsigned char channel);

extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

// 运行时速度变量（可被 UART3 调参修改）
extern int16_t g_speed_fast;
extern int16_t g_speed_slow;
extern int16_t g_speed_min;

// 转向 PD 参数（可被 UART3 调参修改）
extern float g_track_kp;     // 比例系数
extern float g_track_kd;     // 微分系数

#endif
