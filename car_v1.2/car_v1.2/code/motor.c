#include "motor.h"

/********** 电机方向标志 **********/
uint8_t motorA_dir = 1 ; // 1为正转，0为反转（对应 TB6612 的 AIN1/AIN2 逻辑电平组合）
uint8_t motorB_dir = 1; // 1为正转，0为反转（对应 TB6612 的 BIN1/BIN2 逻辑电平组合）

/********** 编码器脉冲计数（由外部中断累加） **********/
int Encoder_count1 = 0;  // 编码器1 脉冲计数值（EXTI6/PA6 A相触发，PA2 B相判方向）
int Encoder_count2 = 0;  // 编码器2 脉冲计数值（EXTI4/PB4 A相触发，PB5 B相判方向）

int speed_now;           // 当前转速计算值（暂未使用）

/*------------------------------------------------------------------
 * 电机驱动引脚定义（TB6612）
 *   AIN1 -> PA5     电机A 方向控制引脚1
 *   AIN2 -> PA7     电机A 方向控制引脚2
 *   PWMA -> TIM2_CH1 (通常映射到 PA0 / 具体依 pwm_init 实现而定)
 *   
 *   BIN1 -> PA4     电机B 方向控制引脚1
 *   BIN2 -> PA3     电机B 方向控制引脚2
 *   PWMB -> TIM2_CH2 (通常映射到 PA1 / 具体依 pwm_init 实现而定)
 *------------------------------------------------------------------*/

// 电机驱动初始化函数
void motor_init()
{
    // STBY 使能 (PB12) — 如果已硬接3.3V可删掉  STBY enable
    gpio_init(MOTOR_STBY_PORT, MOTOR_STBY_PIN, OUT_PP);
    gpio_set(MOTOR_STBY_PORT, MOTOR_STBY_PIN, 1);     // 拉高 STBY 拉高使能TB6612

    // 初始化 PWM 信号（定时器2，通道1和通道2，频率 1kHz）
    pwm_init(TIM_2, TIM2_CH1, 1000);   // PWMA 输出
    pwm_init(TIM_2, TIM2_CH2, 1000);   // PWMB 输出

    // 方向控制引脚初始化为推挽输出
    gpio_init(GPIO_A, Pin_5, OUT_PP);  // AIN1
    gpio_init(GPIO_A, Pin_7, OUT_PP);  // AIN2
    gpio_init(GPIO_A, Pin_4, OUT_PP);  // BIN1
    gpio_init(GPIO_A, Pin_3, OUT_PP);  // BIN2
}

// 设置电机 A 的转速（PWM 占空比）和方向
// duty: 0~1000 对应 0%~100% 占空比（实际范围取决于 pwm_update 的实现）
void motorA_duty(int duty)
{
    pwm_update(TIM_2, TIM2_CH1, duty);          // 更新 PWMA 占空比
    gpio_set(GPIO_A, Pin_5, motorA_dir);        // AIN1 = motorA_dir
    gpio_set(GPIO_A, Pin_7, !motorA_dir);       // AIN2 = 相反电平
    // TB6612 真值表：AIN1=1, AIN2=0 => 正转； AIN1=0, AIN2=1 => 反转
}

// 设置电机 B 的转速（PWM 占空比）和方向
void motorB_duty(int duty)
{
    pwm_update(TIM_2, TIM2_CH2, duty);          // 更新 PWMB 占空比
    gpio_set(GPIO_A, Pin_4, motorB_dir);        // BIN1 = motorB_dir
    gpio_set(GPIO_A, Pin_3, !motorB_dir);       // BIN2 = 相反电平
}

/*------------------------------------------------------------------
 * 编码器接口（霍尔编码器，单相计数+方向判断模式）
 *   编码器1 A相 -> PA6 (EXTI6, 下降沿触发)    B相 -> PA2 (输入上拉, 判方向)
 *   编码器2 A相 -> PB4 (EXTI4, 下降沿触发)    B相 -> PB5 (输入上拉, 判方向)
 * 注：ISR 中根据 B 相电平增减 Encoder_count1/2
 *------------------------------------------------------------------*/
void encoder_init()
{
    // 编码器1 A相：PA6 外部中断下降沿触发（优先级 0）
    exti_init(EXTI_PA6, FALLING, 0);
    // 编码器1 B相：PA2 输入上拉，用于确定旋转方向
    gpio_init(GPIO_A, Pin_2, IU);   // IU = 输入上拉

    // 编码器2 A相：PB4 外部中断下降沿触发（优先级 0）
    exti_init(EXTI_PB4, FALLING, 0);
    // 编码器2 B相：PB5 输入上拉，用于确定旋转方向
    gpio_init(GPIO_B, Pin_5, IU);
}
