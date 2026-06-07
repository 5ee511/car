#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时长度，范围：0~233015
  * @retval 无
  */
void delay_us(uint32_t xus)
{
SysTick->LOAD = 72 * xus;
SysTick->VAL = 0x00;
SysTick->CTRL = 0x00000005;
while(!(SysTick->CTRL & 0x00010000));
SysTick->CTRL = 0x00000004;
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时长度，范围：0~4294967295
  * @retval 无
  */
void delay_ms(uint32_t xms)
{
while(xms--)
{
delay_us(1000);
}
}

/**
  * @brief  秒级延时
  * @param  xs 延时长度，范围：0~4294967295
  * @retval 无
  */
void delay_s(uint32_t xs)
{
while(xs--)
{
delay_ms(1000);
}
}