#ifndef _motor_h
#define _motor_h
#include "headfile.h"

// TB6612 STBY 引脚 PB12 (如果已硬接3.3V可忽略此宏)
#define MOTOR_STBY_PORT  GPIO_B
#define MOTOR_STBY_PIN   Pin_12

void motor_init(void);
void motorA_duty(int duty);
void motorB_duty(int duty);
void encoder_init(void);

extern int Encoder_count1, Encoder_count2;
extern int speed_now;
extern uint8_t motorA_dir, motorB_dir;

#endif
