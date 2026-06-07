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
extern char g_track_state[8];      // 巡线状态  Track state string
void sensor_test(void);
unsigned char digtal(unsigned char channel);

extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

#endif
