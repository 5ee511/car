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
void EXTI0_IRQHandler(void) // PB0: MPU6050 INT
{
if(EXTI->PR&(1<<0))
{
// 读取原始传感器数据
MPU6050_GetData();
//HMC5883L_GetData();   // HMC5883L未初始化，暂不读取  HMC5883L not initialized, skip for now

// 陀螺仪角度
roll_gyro += (float)gx / 16.4 * 0.005;
pitch_gyro += (float)gy / 16.4 * 0.005;
yaw_gyro += (float)gz / 16.4 * 0.005;

// 加速度计角度
roll_acc = atan((float)ay/az) * 57.296;
pitch_acc = - atan((float)ax/az) * 57.296;
yaw_acc = atan((float)ay/ax) * 57.296;

// 磁力计角度
yaw_hmc = atan2((float)hmc_x, (float)hmc_y) * 57.296;

// 卡尔曼滤波融合角度
roll_Kalman = Kalman_Filter(&KF_Roll, roll_acc, (float)gx / 16.4);
pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4);
yaw_Kalman = Kalman_Filter(&KF_Yaw, yaw_hmc, (float)gz / 16.4);

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
