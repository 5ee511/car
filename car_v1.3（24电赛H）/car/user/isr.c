#include "stm32f10x.h"                  // Device header
#include "headfile.h"

volatile uint32_t g_tim3_tick = 0;      // TIM3中断计数, 用于调试  TIM3 ISR counter for debug

// 定时器中断服务函数
void TIM2_IRQHandler(void)
{
if(TIM2->SR&1)
{

// 此处填写中断处理
TIM2->SR &= ~1; 
}
}

void TIM3_IRQHandler(void)
{
if(TIM3->SR&1)
{
pid_control();
g_tim3_tick++;                   // 中断计数器 ISR tick counter
TIM3->SR &= ~1; 
}
}

void TIM4_IRQHandler(void)
{
if(TIM4->SR&1)
{
// 此处填写中断处理

TIM4->SR &= ~1; 
}
}


// 串口中断服务函数
void USART1_IRQHandler(void)
{
if (USART1->SR&0x20)
{
// 此处填写中断处理

USART1->SR &= ~0x20;   // 清除标志位
}
}


void USART2_IRQHandler(void)
{
if (USART2->SR&0x20)
{
// 此处填写中断处理

USART2->SR &= ~0x20;   // 清除标志位
}
}

void USART3_IRQHandler(void)
{
if (USART3->SR&0x20)
{
#ifdef USE_UART_TUNE
	uart_tune_rx_isr((uint8_t)(USART3->DR & 0xFF));
#endif
USART3->SR &= ~0x20;
}
}


// 外部中断服务函数
void EXTI0_IRQHandler(void) // PB0: MPU6050 INT (100Hz data ready)
{
if(EXTI->PR&(1<<0))
{
	// 总线忙 → 丢弃本帧, 避免和OLED抢I2C (共享PB6/PB7)
	if (g_i2c_busy)
	{
		EXTI->PR = 1<<0;
		return;
	}
	
	g_i2c_busy = 1;
	// ★ BURST READ: 一次I2C事务读全部14字节, 比旧版快7倍 (~400μs vs ~3000μs)
	if (MPU6050_GetData() != 0)
	{
		// I2C通信失败 → 丢弃本帧, 不更新任何姿态数据
		g_i2c_busy = 0;
		EXTI->PR = 1<<0;
		return;
	}
	g_i2c_busy = 0;

	// ==== Yaw: 纯陀螺Z积分 (dt=10ms, ±2000dps → 16.4 LSB/(°/s)) ====
	float rate = ((float)gz - gyro_bias_z) / 16.4f;  // °/s
	if (rate > -0.15f && rate < 0.15f) rate = 0.0f;  // 死区 ±0.15°/s
	g_yaw += rate * 0.01f;

	// ==== Roll/Pitch: 卡尔曼融合 (有加速度计参考) ====
	roll_gyro  += (float)gx / 16.4f * 0.01f;
	pitch_gyro += (float)gy / 16.4f * 0.01f;

	roll_acc  =  atan2f((float)ay, (float)az) * 57.296f;
	pitch_acc = -atan2f((float)ax, (float)az) * 57.296f;

	roll_Kalman  = Kalman_Filter(&KF_Roll,  roll_acc,  (float)gx / 16.4f);
	pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4f);

	EXTI->PR = 1<<0; // 清除标志位
}
}

void EXTI1_IRQHandler(void) // PA1/PB1/PC1
{
if(EXTI->PR&(1<<1))
{
// 此处填写中断处理

EXTI->PR = 1<<1; // 清除标志位
}
}

void EXTI2_IRQHandler(void) // PA2/PB2/PC2
{
if(EXTI->PR&(1<<2))
{

// 此处填写中断处理

EXTI->PR = 1<<2; // 清除标志位
}
}

void EXTI3_IRQHandler(void) // PA3/PB3/PC3
{
if(EXTI->PR&(1<<3))
{
// 此处填写中断处理

EXTI->PR = 1<<3; // 清除标志位
}
}

void EXTI4_IRQHandler(void) // PA4/PB4/PC4 — 编码器2 (PB4 A相, PB5 B相判方向)
{
if(EXTI->PR&(1<<4))
{
if(gpio_get(GPIO_B, Pin_5))
Encoder_count2 --;
else
Encoder_count2 ++;
// 此处填写中断处理

EXTI->PR = 1<<4; // 清除标志位
}
}

void EXTI9_5_IRQHandler(void)
{
if(EXTI->PR&(1<<5))   //EXTI5  PA5/PB5/PC5
{

// 此处填写中断处理

EXTI->PR = 1<<5; // 清除标志位
}

if(EXTI->PR&(1<<6))   //EXTI6  PA6 — 编码器1 A相
{
if(gpio_get(GPIO_A, Pin_2))  // B相判方向
Encoder_count1 ++;  // 与实际电机接线匹配
else
Encoder_count1 --;
// 此处填写中断处理

EXTI->PR = 1<<6; // 清除标志位
}

if(EXTI->PR&(1<<7))   //EXTI7  PA7/PB7/PC7 (unused, MPU6050 moved to EXTI0)
{
// 此处填写中断处理

EXTI->PR = 1<<7; // 清除标志位
}

if(EXTI->PR&(1<<8))   //EXTI8
{
// 此处填写中断处理

EXTI->PR = 1<<8; // 清除标志位
}

if(EXTI->PR&(1<<9))   //EXTI9
{
// 此处填写中断处理

EXTI->PR = 1<<9; // 清除标志位
}
}
