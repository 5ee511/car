#include "headfile.h"

extern volatile uint32_t g_tim3_tick;

#define TEST_SPEED  10   // 测试速度(低), 便于调试  Test speed (low) for tuning

int main(void)
{
	OLED_Init();
	motor_init();                                  // 含STBY使能
	encoder_init();
	uart_init(UART_1, 115200, 0);
	pid_init(&motorA, DELTA_PID, 60, 40, 0);
	pid_init(&motorB, DELTA_PID, 60, 40, 0);
//	pid_init(&angle, POSITION_PID, 10, 0, 0.5);
	I2C_Init();
//	MPU6050_Init();
	exti_init(EXTI_PB0, RISING, 0);
	tim_interrupt_ms_init(TIM_3, 100, 0);
	Grayscale_Sensor_Init();

	while (1)
	{
		Grayscale_Sensor_Read_All(g_sensor_data);
		track();                                   // 巡线! 分支判断法  Line tracking (branch-based)

		// ==== OLED 4行布局 ====
		// Line1: 8路传感器值 (0=踩线 1=白)  Sensor values
		OLED_ShowNum(1, 1,  D1, 1); OLED_ShowNum(1, 3,  D2, 1);
		OLED_ShowNum(1, 5,  D3, 1); OLED_ShowNum(1, 7,  D4, 1);
		OLED_ShowNum(1, 9,  D5, 1); OLED_ShowNum(1, 11, D6, 1);
		OLED_ShowNum(1, 13, D7, 1); OLED_ShowNum(1, 15, D8, 1);

		// Line2: 巡线状态 + 目标速度  Track state + target speed
		OLED_ShowString(2, 1,  g_track_state);        // "GO  ""R1  "等
		OLED_ShowString(2, 6,  "A:");
		OLED_ShowNum   (2, 8,  (int)motorA.target, 3);
		OLED_ShowString(2, 12, "B:");
		OLED_ShowNum   (2, 14, (int)motorB.target, 3);

		// Line3: 编码器实际速度(有符号)  Encoder actual speed (signed)
		OLED_ShowString(2, 11, " ");  // 清掉残留
		OLED_ShowString   (3, 1,  "E1");
		OLED_ShowSignedNum(3, 3,  Encoder_count1, 4);
		OLED_ShowString   (3, 9,  "E2");
		OLED_ShowSignedNum(3, 11, Encoder_count2, 4);

		// Line4: 清空  Clear
		OLED_ShowString(4, 1,  "                ");

		delay_ms(80);
	}
}
