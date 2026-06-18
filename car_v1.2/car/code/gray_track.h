#ifndef __gray_track_h_
#define __gray_track_h_
#include "grayscale_sensor.h"
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
extern char g_track_state[8];
void sensor_test(void);
unsigned char digtal(unsigned char channel);

extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

// 运行时速度变量（UART3 可调）
extern int16_t g_speed_fast;
extern int16_t g_speed_slow;
extern int16_t g_speed_min;

// 转向 PD 参数（UART3 可调）
extern float g_track_kp;
extern float g_track_kd;

// 直角弯调试变量（OLED 转弯时显示）
extern uint8_t g_turn_active;       // 1=转弯中, 0=正常
extern char    g_turn_dir_char;     // 'L' 或 'R'
extern uint8_t g_turn_frame_cnt;    // 当前转弯帧数
extern uint8_t g_turn_lcnt;         // 转弯中左侧亮灯数
extern uint8_t g_turn_rcnt;         // 转弯中右侧亮灯数
extern int16_t g_turn_speed_L;      // 转弯中左轮目标速度
extern int16_t g_turn_speed_R;      // 转弯中右轮目标速度

#endif
