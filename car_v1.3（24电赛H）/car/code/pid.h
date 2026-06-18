#ifndef __PID_h_
#define __PID_h_
#include "headfile.h"

enum
{
  POSITION_PID = 0,  // 位置式
  DELTA_PID,         // 增量式
};

typedef struct
{
	float target;	
	float now;
	float error[3];		
	float p,i,d;
	float pout, dout, iout;
	float out;   
	
	uint32_t pid_mode;

}pid_t;

void pid_cal(pid_t *pid);
void pid_control(void);
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void motor_target_set(int spe1, int spe2);
void pidout_limit(pid_t *pid);

// 角度PD控制 (无积分, 处理360°环绕)
//   target: 目标角度 (0~360°)
//   yaw:    当前偏航角 (任意范围)
//   返回:   转向修正量 (正值=右转, 负值=左转)
float pid_angle(float target, float yaw);

extern pid_t motorA;
extern pid_t motorB;
extern pid_t angle;
#endif
