#ifndef _i2c_h
#define _i2c_h
#include "headfile.h"

/* 
   软件模拟I2C协议
	 修改下面三个参数 
	 自定义SCL和SDA引脚
	 需将对应引脚配置成开漏输出	 
*/
#define I2C_GPIO 	         GPIO_B
#define I2C_SCL_GPIO_Pin   Pin_6	
#define I2C_SDA_GPIO_Pin   Pin_7 	

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_SendByte(uint8_t byte);       // 返回ACK: 0=成功, 1=失败
uint8_t I2C_ReceiveByte(uint8_t ack);     // ack=0继续读, ack=1最后一字节
void I2C_SendAck(void);                    // 兼容旧API (功能已合并到ReceiveByte)
void I2C_NotSendAck(void);                 // 兼容旧API
uint8_t I2C_WaitAck(void);                 // 0=ACK成功, 1=NACK失败

// 总线忙标志 (MPU6050和OLED共享PB6/PB7, ISR中检查防冲突)
extern volatile uint8_t g_i2c_busy;

#endif
