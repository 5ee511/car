#include "headfile.h"

extern volatile uint32_t g_tim3_tick;

#define TEST_SPEED  10   // 测试速度(低), 便于调试  Test speed (low) for tuning

int main(void)
{
	OLED_Init();
	motor_init();                                  // 含STBY使能
	encoder_init();
	uart_init(UART_1, 115200, 0);
	printf("Car v1.2 Ready\r\n");                  // 启动确认

	// ==== UART3 裸机测试 ====
	uart_init(UART_3, 115200, 1);
	uart_sendstr(UART_3, "=== UART3 TEST ===\r\n");

	pid_init(&motorA, DELTA_PID, 200, 88, 15);   // ★ 电机A: P=200 I=88 D=15
	pid_init(&motorB, DELTA_PID, 200, 88, 15);   // ★ 电机B: P=200 I=88 D=15
//	pid_init(&angle, POSITION_PID, 10, 0, 0.5);
	I2C_Init();
//	MPU6050_Init();
	exti_init(EXTI_PB0, RISING, 0);
	tim_interrupt_ms_init(TIM_3, 100, 0);
	Grayscale_Sensor_Init();

	while (1)
	{
		Grayscale_Sensor_Read_All(g_sensor_data);
		track();                                   // 巡线! 分支判断法

		// ==== OLED 4行布局 ====
		// Line1: 8路传感器 (0=白 1=踩线)
		OLED_ShowNum(1, 1,  D1, 1); OLED_ShowNum(1, 3,  D2, 1);
		OLED_ShowNum(1, 5,  D3, 1); OLED_ShowNum(1, 7,  D4, 1);
		OLED_ShowNum(1, 9,  D5, 1); OLED_ShowNum(1, 11, D6, 1);
		OLED_ShowNum(1, 13, D7, 1); OLED_ShowNum(1, 15, D8, 1);

		// Line2: 巡线状态 + 目标速度
		OLED_ShowString(2, 1,  g_track_state);
		OLED_ShowString(2, 6,  "A:");
		OLED_ShowNum   (2, 8,  (int)motorA.target, 3);
		OLED_ShowString(2, 12, "B:");
		OLED_ShowNum   (2, 14, (int)motorB.target, 3);

		// Line3: PID预估实际速度 (与串口CH2/CH4一致)
		OLED_ShowString   (3, 1,  "E1");
		OLED_ShowSignedNum(3, 3,  (int)motorA.now, 4);
		OLED_ShowString   (3, 9,  "E2");
		OLED_ShowSignedNum(3, 11, (int)motorB.now, 4);

		// Line4: PID实时值 格式: "A060/120B060/040"
		OLED_ShowString(4, 1,  "A");
		OLED_ShowNum   (4, 2,  (int)motorA.p, 3);
		OLED_ShowString(4, 5,  "/");
		OLED_ShowNum   (4, 6,  (int)motorA.i, 3);
		OLED_ShowString(4, 9,  "B");
		OLED_ShowNum   (4, 10, (int)motorB.p, 3);
		OLED_ShowString(4, 13, "/");
		OLED_ShowNum   (4, 14, (int)motorB.i, 3);

#ifdef USE_UART_TUNE
		uart_tune_process();                       // 处理 UART3 PID 调参命令
#endif

		// 看波形时注释掉printf, 让datavision_send独占串口
		//printf("A:%d/%d B:%d/%d\r\n",
		//       (int)motorA.target, (int)motorA.now,
		//       (int)motorB.target, (int)motorB.now);

		delay_ms(80);
	}
}
