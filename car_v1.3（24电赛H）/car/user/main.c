#include "headfile.h"

extern volatile uint32_t g_tim3_tick;

// ==============================
// 运行模式: 注释掉=正常巡线, Q1/Q2/Q3/Q4=电赛H题第N问
// ==============================
#define RUN_Q3

// ==== 蜂鸣器 (PB1, 低电平响) ====
#define BUZZER_GPIO   GPIO_B
#define BUZZER_PIN    Pin_1
#define BUZZER_ON()   gpio_set(BUZZER_GPIO, BUZZER_PIN, 0)
#define BUZZER_OFF()  gpio_set(BUZZER_GPIO, BUZZER_PIN, 1)

// ==== 第一问参数 (RUN_Q1 兼容) ====
#define Q1_BASE_SPEED    30
#define Q1_TARGET_ANGLE  0

// ============================================================
// Q2/Q3/Q4 统一参数表 — 数据驱动状态机
//   编码规则: angle=0 → 循迹模式, angle≠0 → 角度模式
//   每4步一个循环: 角度→循迹→角度→循迹
//   Q2(4步)/Q3(4步)/Q4(16步=4圈)
// ============================================================
#if defined(RUN_Q2) || defined(RUN_Q3) || defined(RUN_Q4)

#if defined(RUN_Q2)
	#define QX_LABEL  "Q2"
	#define QX_STEPS  4
	static const float   qx_angle[] = {0.0f, 0.0f, 168.0f, 0.0f};
	static const int16_t qx_speed[] = {  30,   25,    30,   25};
#elif defined(RUN_Q3)
	#define QX_LABEL  "Q3"
	#define QX_STEPS  4
	static const float   qx_angle[] = {50.0f, 0.0f, 130.0f, 0.0f};
	static const int16_t qx_speed[] = {  30,   25,    30,   22};
#elif defined(RUN_Q4)
	#define QX_LABEL  "Q4"
	#define QX_STEPS  16
	static const float   qx_angle[] = {
		50, 0, 129, 0,   59, 0, 137, 0,
		65, 0, 145, 0,   71, 0, 150, 0
	};
	static const int16_t qx_speed[] = {
		30, 25, 30, 22,  30, 25, 30, 22,
		30, 25, 30, 22,  30, 25, 30, 22
	};
#endif

#endif // RUN_Q2 || RUN_Q3 || RUN_Q4

int main(void)
{
	OLED_Init();
	motor_init();                                  // 含STBY使能
	encoder_init();
	uart_init(UART_1, 115200, 0);
	printf("Car v1.2 Ready\r\n");

	// ==== UART3 PID 实时调参 ====
	uart_tune_init(115200);

	pid_init(&motorA, DELTA_PID, 200, 88, 15);
	pid_init(&motorB, DELTA_PID, 200, 88, 15);

	// ==== MPU6050: 初始化 + WHO_AM_I验证 + 零偏校准 ====
	I2C_Init();
	if (MPU6050_Init() != 0)
	{
		printf("MPU6050 FAIL! Check SDA/SCL wiring\r\n");
		// OLED显示错误提示, 然后死循环
		OLED_ShowString(1, 1, "MPU6050 ERR!");
		OLED_ShowString(2, 1, "Check I2C wire");
		while(1);
	}
	delay_ms(50);                    // 等芯片稳定
	printf("MPU6050 Init OK\r\n");

	printf("Calibrating gyro bias... (keep car STILL!)\r\n");
	MPU6050_CalGyroBias();           // 采集零偏, 期间必须静止!
	printf("Gyro bias Z = %.1f\r\n", gyro_bias_z);

	// ==== 蜂鸣器初始化 (PB1, 推挽输出, 默认关) ====
	gpio_init(BUZZER_GPIO, BUZZER_PIN, OUT_PP);
	BUZZER_OFF();

	exti_init(EXTI_PB0, RISING, 0);  // 使能 MPU6050 INT 中断
	tim_interrupt_ms_init(TIM_3, 100, 0);
	Grayscale_Sensor_Init();

#if defined(RUN_Q2) || defined(RUN_Q3) || defined(RUN_Q4)
	{
		int _i;
		printf(QX_LABEL " Start! %d steps, angles: ", QX_STEPS);
		for (_i = 0; _i < QX_STEPS; _i++)
			if (qx_angle[_i] != 0.0f) printf("%.0f ", qx_angle[_i]);
		printf("\r\n");
	}
#elif defined(RUN_Q1)
	printf("Q1 Start! Target = %d deg, Base Speed = %d\r\n",
	       Q1_TARGET_ANGLE, Q1_BASE_SPEED);
#endif

	while (1)
	{
		// 读灰度传感器
		Grayscale_Sensor_Read_All(g_sensor_data);
		uint8_t d1=D1, d2=D2, d3=D3, d4=D4;
		uint8_t d5=D5, d6=D6, d7=D7, d8=D8;
		uint8_t cnt = d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;

#if defined(RUN_Q2) || defined(RUN_Q3) || defined(RUN_Q4)
		// ============================================
		// 电赛H题 Q2/Q3/Q4: 角度↔循迹交替, 数据驱动
		//   角度步(angle≠0): pid_angle() 直走→等踩线(cnt>1)
		//   循跡步(angle=0): track() 跟线→等脱线(cnt=0)
		//   每步过渡: 停车+蜂鸣+清PID+track_reset
		// ============================================
		static uint8_t  qx_step = 1;         // 当前步骤
		static uint8_t  qx_trans = 0;        // 过渡标志
		static uint16_t qx_trans_cnt = 0;    // 过渡帧计数
		static uint16_t qx_done_beep = 0;    // 完成蜂鸣

		float steer;
		int16_t L, R;

		if (qx_trans)
		{
			// ==== 步骤过渡: 停车+蜂鸣+稳车 ====
			motor_target_set(0, 0);
			motorA_duty(0); motorB_duty(0);
			BUZZER_ON();

			qx_trans_cnt++;
			if (qx_trans_cnt >= 2)  // 2帧≈160ms
			{
				qx_trans = 0;
				qx_trans_cnt = 0;
				BUZZER_OFF();

				// 清零PID & 巡线状态
				motorA.out = 0; motorA.error[0] = motorA.error[1] = motorA.error[2] = 0;
				motorB.out = 0; motorB.error[0] = motorB.error[1] = motorB.error[2] = 0;
				track_reset();

				qx_step++;
			}
		}
		else if (qx_step <= QX_STEPS)
		{
			BUZZER_OFF();
			uint8_t idx = qx_step - 1;        // 参数表索引
			float   ang = qx_angle[idx];       // 0=循迹, 非0=角度模式
			int16_t spd = qx_speed[idx];

			if (ang == 0.0f)
			{
				// ★ 循迹步: 设巡线速度, 跟线走, 等脱线
				g_speed_fast = spd;
				if (cnt == 0) { qx_trans = 1; }
				else          { track(); }
			}
			else
			{
				// ★ 角度步: 角度PD直走, 等踩线
				if (cnt <= 1)
				{
					steer = pid_angle(ang, g_yaw);
					L = spd - (int16_t)steer;
					R = spd + (int16_t)steer;
					if (L < 0) L = 0; if (R < 0) R = 0;
					motor_target_set(L, R);
				}
				else { qx_trans = 1; }
			}
		}
		else
		{
			// ★ 完成! 停车+蜂鸣器间歇响
			motor_target_set(0, 0);
			motorA_duty(0); motorB_duty(0);
			qx_done_beep++;
			if      (qx_done_beep < 5)  BUZZER_ON();
			else if (qx_done_beep < 10) BUZZER_OFF();
			else                        qx_done_beep = 0;
		}
#elif defined(RUN_Q1)
		// ============================================
		// 电赛H题 第一问: 角度PD直走, 遇线停车
		// ============================================
		float steer;
		int16_t L, R;

		if (cnt <= 1)  // 容忍1路噪声, ≤1 视为全白
		{
			steer = pid_angle(0.0f, g_yaw);
			L = (int16_t)((float)Q1_BASE_SPEED - steer);
			R = (int16_t)((float)Q1_BASE_SPEED + steer);

			if (L < 0) L = 0;
			if (R < 0) R = 0;
		}
		else
		{
			// 踩到线 → 彻底停车
			steer = 0.0f;
			L = 0;
			R = 0;

			// 清零PID状态, 防止余量输出导致电机嗡嗡响
			motorA.out = 0;  motorA.error[0] = motorA.error[1] = motorA.error[2] = 0;
			motorB.out = 0;  motorB.error[0] = motorB.error[1] = motorB.error[2] = 0;
			motorA_duty(0);
			motorB_duty(0);
		}
		motor_target_set(L, R);
#else
		// 正常巡线模式
		track();
#endif

		// ==== OLED 刷新 (先关EXTI0 + 设I2C忙标志, 防MPU6050抢总线) ====
		EXTI->IMR &= ~(1<<0);          // 关 MPU6050 INT
		g_i2c_busy = 1;                // 置忙标志 (ISR双重保险)

		OLED_ShowNum(1, 1,  D1, 1); OLED_ShowNum(1, 3,  D2, 1);
		OLED_ShowNum(1, 5,  D3, 1); OLED_ShowNum(1, 7,  D4, 1);
		OLED_ShowNum(1, 9,  D5, 1); OLED_ShowNum(1, 11, D6, 1);
		OLED_ShowNum(1, 13, D7, 1); OLED_ShowNum(1, 15, D8, 1);

#if defined(RUN_Q2) || defined(RUN_Q3) || defined(RUN_Q4)
		// Line2: Qx标志 + Step编号 + cnt + Yaw
		OLED_ShowString(2, 1,  QX_LABEL);
		OLED_ShowString(2, 3,  "S");
		OLED_ShowNum   (2, 4,  qx_step, 1);
		OLED_ShowString(2, 6,  " C");
		OLED_ShowNum   (2, 8,  cnt, 1);
		OLED_ShowString(2, 10, " Y:");
		OLED_ShowSignedNum(2, 13, (int)g_yaw, 3);

		// Line3: 当前步模式 + L/R目标
		if (qx_step <= QX_STEPS && qx_angle[qx_step-1] == 0.0f)
			OLED_ShowString(3, 1, "TRK");       // 循迹
		else if (qx_step <= QX_STEPS)
			OLED_ShowString(3, 1, "ANG");       // 角度
		else
			OLED_ShowString(3, 1, "DON");       // 完成
		OLED_ShowString(3, 5,  "L:");
		OLED_ShowSignedNum(3, 7,  motorA.target, 3);
		OLED_ShowString(3, 11, "R:");
		OLED_ShowSignedNum(3, 13, motorB.target, 3);

		// Line4: 编码器速度
		OLED_ShowString   (4, 1,  "E1");
		OLED_ShowSignedNum(4, 3,  (int)motorA.now, 4);
		OLED_ShowString   (4, 9,  "E2");
		OLED_ShowSignedNum(4, 11, (int)motorB.now, 4);
#elif defined(RUN_Q1)
		// Line2: cnt值 + Yaw + 状态
		OLED_ShowString(2, 1,  "C");
		OLED_ShowNum   (2, 2,  cnt, 1);
		OLED_ShowString(2, 4,  " Y:");
		OLED_ShowSignedNum(2, 7,  (int)g_yaw, 4);
		OLED_ShowString(2, 12, "S:");
		OLED_ShowSignedNum(2, 14, (int)steer, 2);

		// Line3: L/R目标速度 + 陀螺Z原始值
		OLED_ShowString(3, 1,  "L:");
		OLED_ShowSignedNum(3, 3,  L, 3);
		OLED_ShowString(3, 7,  "R:");
		OLED_ShowSignedNum(3, 9,  R, 3);
		OLED_ShowString(3, 13, "Z:");
		OLED_ShowSignedNum(3, 15, (int)((float)gz - gyro_bias_z), 2);

		// Line4: 编码器实际速度
		OLED_ShowString   (4, 1,  "E1");
		OLED_ShowSignedNum(4, 3,  (int)motorA.now, 4);
		OLED_ShowString   (4, 9,  "E2");
		OLED_ShowSignedNum(4, 11, (int)motorB.now, 4);
#else
		OLED_ShowString(2, 1,  g_track_state);
		OLED_ShowString(2, 6,  "A:");
		OLED_ShowNum   (2, 8,  (int)motorA.target, 3);
		OLED_ShowString(2, 12, "B:");
		OLED_ShowNum   (2, 14, (int)motorB.target, 3);

		OLED_ShowString   (3, 1,  "E1");
		OLED_ShowSignedNum(3, 3,  (int)motorA.now, 4);
		OLED_ShowString   (3, 9,  "E2");
		OLED_ShowSignedNum(3, 11, (int)motorB.now, 4);

		OLED_ShowString(4, 1,  "A");
		OLED_ShowNum   (4, 2,  (int)motorA.p, 3);
		OLED_ShowString(4, 5,  "/");
		OLED_ShowNum   (4, 6,  (int)motorA.i, 3);
		OLED_ShowString(4, 9,  "B");
		OLED_ShowNum   (4, 10, (int)motorB.p, 3);
		OLED_ShowString(4, 13, "/");
		OLED_ShowNum   (4, 14, (int)motorB.i, 3);
#endif

		EXTI->IMR |= (1<<0);           // 开 MPU6050 INT
		g_i2c_busy = 0;                // 释放I2C总线

#ifdef USE_UART_TUNE
		uart_tune_process();
#endif

		delay_ms(80);
	}
}
