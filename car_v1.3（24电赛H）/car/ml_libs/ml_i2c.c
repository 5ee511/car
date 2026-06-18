#include "ml_i2c.h"

// ==================== I2C 时序宏 ====================
// 软件I2C必须加延时! 72MHz下GPIO翻转仅~200ns, 远超I2C规范
// 不加延时 → 从机ACK失败 → 读回垃圾值 → MPU6050数据跳变
// 用简单NOP循环 (~1.5μs @72MHz), 避免SysTick开销, ISR安全
#define I2C_DELAY()  do { volatile uint8_t _d=36; while(_d--) __NOP(); } while(0)

#define SDA_Output(x)  gpio_set(I2C_GPIO, I2C_SDA_GPIO_Pin, x)
#define SCL_Output(x)  gpio_set(I2C_GPIO, I2C_SCL_GPIO_Pin, x)
#define SDA_Input()    gpio_get(I2C_GPIO, I2C_SDA_GPIO_Pin)

// I2C总线忙标志 (MPU6050和OLED共享PB6/PB7, 防止并发冲突)
volatile uint8_t g_i2c_busy = 0;

void I2C_Init()
{
	gpio_init(I2C_GPIO, I2C_SCL_GPIO_Pin, OUT_OD);
	gpio_init(I2C_GPIO, I2C_SDA_GPIO_Pin, OUT_OD);
	
	SDA_Output(1);
	SCL_Output(1);
}

// 起始信号: SCL高时SDA拉低
void I2C_Start()
{
	SDA_Output(1);
	I2C_DELAY();
	SCL_Output(1);
	I2C_DELAY();
	SDA_Output(0);
	I2C_DELAY();
	SCL_Output(0);
	I2C_DELAY();
}

// 终止信号: SCL高时SDA拉高
void I2C_Stop()
{
	SDA_Output(0);
	I2C_DELAY();
	SCL_Output(1);
	I2C_DELAY();
	SDA_Output(1);
	I2C_DELAY();
}

// 主机发送一个字节 (MSB先), 返回从机ACK: 0=ACK, 1=NACK
uint8_t I2C_SendByte(uint8_t byte)
{
	for(int i = 0; i < 8; i++)
	{
		if(byte & (0x80>>i))
			SDA_Output(1);
		else
			SDA_Output(0);
		I2C_DELAY();           // 数据建立时间
		SCL_Output(1);
		I2C_DELAY();           // SCL高电平保持
		SCL_Output(0);
		I2C_DELAY();           // SCL低电平保持
	}
	// 第9个时钟: 读ACK
	SDA_Output(1);             // 释放SDA
	I2C_DELAY();
	SCL_Output(1);
	I2C_DELAY();
	uint8_t ack = SDA_Input(); // 0=ACK, 1=NACK
	SCL_Output(0);
	I2C_DELAY();
	return ack;
}

// 主机接收一个字节, ack=0发ACK(继续读), ack=1发NACK(最后一字节)
uint8_t I2C_ReceiveByte(uint8_t ack)
{
	uint8_t byte = 0;
	SDA_Output(1);             // 释放SDA
	for(int i = 0; i < 8; i++)
	{
		I2C_DELAY();
		SCL_Output(1);
		I2C_DELAY();
		if(SDA_Input())
			byte |= (0x80>>i);
		SCL_Output(0);
	}
	// 第9个时钟: 发ACK/NACK
	if(ack)
		SDA_Output(1);         // NACK: 不再读
	else
		SDA_Output(0);         // ACK: 继续读下一个字节
	I2C_DELAY();
	SCL_Output(1);
	I2C_DELAY();
	SCL_Output(0);
	I2C_DELAY();
	SDA_Output(1);             // 释放SDA
	
	return byte;
}

// 等待从机应答 (兼容旧API, 返回0=ACK成功, 1=NACK失败)
uint8_t I2C_WaitAck()
{
	uint8_t ack;
	SDA_Output(1);
	I2C_DELAY();
	SCL_Output(1);
	I2C_DELAY();
	ack = SDA_Input();
	SCL_Output(0);
	I2C_DELAY();
	
	return ack;  // 0=ACK(成功), 1=NACK(失败)
}

// ==== 兼容旧API桩 (ACK/NACK已合并到I2C_ReceiveByte内部) ====
void I2C_SendAck(void)    { /* 已合并到 I2C_ReceiveByte(0) */ }
void I2C_NotSendAck(void) { /* 已合并到 I2C_ReceiveByte(1) */ }

