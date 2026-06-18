#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "headfile.h"
//#include "delay.h"
//#include "usart.h"

//=====================================================================================
//  引脚配置接口 (Pin Configuration)
//=====================================================================================
// --- 通道选择引脚定义 (AD0, AD1, AD2)  Channel selection pin definition ---
#define SENSOR_AD0_PORT         GPIOB
#define SENSOR_AD0_PIN          GPIO_Pin_13

#define SENSOR_AD1_PORT         GPIOB
#define SENSOR_AD1_PIN          GPIO_Pin_14

#define SENSOR_AD2_PORT         GPIOB
#define SENSOR_AD2_PIN          GPIO_Pin_15

#define GrayS_OUT_PORT          GPIOA
#define GrayS_OUT_PIN           GPIO_Pin_15

//=====================================================================================
//  GPIO操作抽象接口 (GPIO Operation Macros)
//  使用BSRR寄存器(原子操作), 避免ODR读-改-写冲突  Use BSRR (atomic), avoids ODR RMW conflict
//=====================================================================================
#define SENSOR_AD0_WRITE(state)  do { if(state) GPIOB->BSRR = (1<<13); else GPIOB->BRR = (1<<13); } while(0)
#define SENSOR_AD1_WRITE(state)  do { if(state) GPIOB->BSRR = (1<<14); else GPIOB->BRR = (1<<14); } while(0)
#define SENSOR_AD2_WRITE(state)  do { if(state) GPIOB->BSRR = (1<<15); else GPIOB->BRR = (1<<15); } while(0)

#define SENSOR_OUT_READ()        gpio_get(GPIO_A, Pin_15)

//=====================================================================================
//  驱动函数接口 (Driver API)
//=====================================================================================

#define GRAYSCALE_SENSOR_CHANNELS   8   // 传感器通道总数

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif // __GRAYSCALE_SENSOR_H
