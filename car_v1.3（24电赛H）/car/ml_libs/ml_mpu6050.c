#include "ml_mpu6050.h"

int16_t ax, ay, az, gx, gy, gz;
float roll_gyro, pitch_gyro, yaw_gyro;
float roll_acc, pitch_acc, yaw_acc;
float roll_Kalman, pitch_Kalman, yaw_Kalman;

// 角度PD控制用
float g_yaw       = 0.0f;    // 累计偏航角 (度)
float gyro_bias_z = 0.0f;    // 陀螺Z零偏

// I2C通信状态 (调试用)
volatile uint8_t g_mpu_i2c_err = 0;  // 最近一次I2C错误码: 0=OK, 1=ACK失败, 2=ID不匹配

/* ========================== 底层I2C读写 (带ACK检查) ========================== */

/**
 * @brief 写MPU6050单个寄存器 (带ACK校验)
 * @return 0=成功, 1=I2C ACK失败
 */
static uint8_t MPU6050_Write(uint8_t addr, uint8_t dat)
{
	uint8_t err = 0;
	I2C_Start();
	err |= I2C_SendByte(MPU6050_ADDR);      // 设备地址(W), bit0=ACK
	err |= I2C_SendByte(addr);              // 寄存器地址
	err |= I2C_SendByte(dat);               // 数据
	I2C_Stop();
	return err;  // 0=全部ACK成功
}

/**
 * @brief 读MPU6050单个寄存器
 */
static uint8_t MPU6050_Read(uint8_t addr)
{
	I2C_Start();
	I2C_SendByte(MPU6050_ADDR);
	I2C_SendByte(addr);
	I2C_Stop();
	
	I2C_Start();
	I2C_SendByte(MPU6050_ADDR | 0x01);
	uint8_t dat = I2C_ReceiveByte(1);       // 1=最后一字节, 发NACK
	I2C_Stop();
	
	return dat;
}

/**
 * @brief 连续读取MPU6050多个寄存器 (BURST READ)
 * @param startAddr 起始寄存器地址 (如 ACCEL_XOUT_H = 0x3B)
 * @param buf       输出缓冲区
 * @param len       读取字节数 (最大14: 6加计+2温度+6陀螺)
 * @return 0=成功, 非0=I2C错误
 *
 * MPU6050支持地址自动递增: 从0x3B开始连续读14字节可一次获取全部6轴数据
 * 相比逐寄存器读14次 → 减少93%的I2C事务开销, ISR内耗时从~3ms降到~0.4ms
 */
static uint8_t MPU6050_BurstRead(uint8_t startAddr, uint8_t *buf, uint8_t len)
{
	uint8_t i;
	
	I2C_Start();
	if (I2C_SendByte(MPU6050_ADDR))     { I2C_Stop(); return 1; }  // 设备ACK失败
	if (I2C_SendByte(startAddr))        { I2C_Stop(); return 2; }  // 寄存器ACK失败
	I2C_Stop();
	
	I2C_Start();
	if (I2C_SendByte(MPU6050_ADDR | 0x01)) { I2C_Stop(); return 3; }  // 读ACK失败
	
	for (i = 0; i < len; i++)
	{
		uint8_t ack = (i == len - 1) ? 1 : 0;  // 最后一字节发NACK
		buf[i] = I2C_ReceiveByte(ack);
	}
	I2C_Stop();
	
	return 0;  // 成功
}

/* ========================== MPU6050 初始化 ========================== */

/**
 * @brief MPU6050 基础初始化 (100Hz采样, ±2000dps, ±2g, INT使能)
 * @return 0=成功, 1=WHO_AM_I不匹配(接线问题)
 */
uint8_t MPU6050_Init(void)
{
	uint8_t id;
	
	// ① 复位芯片 + 等100ms
	MPU6050_Write(PWR_MGMT_1, 0x80);
	delay_ms(100);
	
	// ② 验证WHO_AM_I (0x75寄存器应读到0x68)
	id = MPU6050_Read(WHO_AM_I);
	if (id != 0x68)
	{
		g_mpu_i2c_err = 2;
		return 1;  // ID不匹配: 检查SDA/SCL接线和供电
	}
	
	// ③ 唤醒, 时钟源选PLL X轴陀螺
	MPU6050_Write(PWR_MGMT_1, 0x01);
	delay_ms(10);
	
	// ④ 采样率: SMPLRT_DIV=9 → 1000/(1+9)=100Hz (dt=10ms)
	MPU6050_Write(SMPLRT_DIV, 9);
	
	// ⑤ DLPF: CFG=2 → 陀螺92Hz/加计94Hz带宽, 匹配100Hz防混叠
	MPU6050_Write(CONFIG, 0x02);
	
	// ⑥ 陀螺±2000dps (0x18=FS_SEL=3), 加速度±2g (0x00=FS_SEL=0)
	MPU6050_Write(GYRO_CONFIG, 0x18);
	MPU6050_Write(ACCEL_CONFIG, 0x00);
	
	// ⑦ 使能数据就绪中断 (INT引脚每10ms输出50μs正脉冲)
	MPU6050_Write(INT_ENABLE, 0x01);
	
	g_mpu_i2c_err = 0;
	return 0;
}

/* ========================== 陀螺零偏校准 ========================== */

/**
 * @brief 上电静止时采集陀螺Z零偏
 * @note  调用期间小车必须保持绝对静止! 耗时约2秒
 *
 * 优化: 使用burst read一次读6字节陀螺数据(0x43~0x48), 而非逐寄存器读
 * 丢前20帧(传感器唤醒不稳定) + 采200帧取算术平均
 */
void MPU6050_CalGyroBias(void)
{
	int i;
	int32_t sum_gz = 0;
	uint8_t buf[6];  // GYRO_XOUT_H~GYRO_ZOUT_L 共6字节
	
	// 丢弃前20帧 (刚唤醒数据不稳定)
	for (i = 0; i < 20; i++)
	{
		MPU6050_BurstRead(GYRO_XOUT_H, buf, 6);
		delay_ms(10);
	}
	
	// 采集200帧取平均
	for (i = 0; i < 200; i++)
	{
		MPU6050_BurstRead(GYRO_XOUT_H, buf, 6);
		// GZ = buf[4]<<8 | buf[5]
		sum_gz += (int16_t)(buf[5] | (buf[4] << 8));
		delay_ms(10);
	}
	
	gyro_bias_z = (float)sum_gz / 200.0f;
	g_yaw = 0.0f;   // 校准后归零偏航角
}

/* ========================== 6轴数据读取 ========================== */

/**
 * @brief 从MPU6050一次性burst读取全部6轴原始数据
 * @return 0=成功, 非0=I2C通信失败(本次数据无效, 应丢弃)
 *
 * 从ACCEL_XOUT_H(0x3B)开始连续读14字节:
 *   buf[0:1]=AX, buf[2:3]=AY, buf[4:5]=AZ,
 *   buf[6:7]=TEMP,
 *   buf[8:9]=GX, buf[10:11]=GY, buf[12:13]=GZ
 *
 * ⚠️ 此函数在EXTI0 ISR中调用, 必须快速完成!
 *    新burst read方式耗时约400μs, 旧逐寄存器方式约3000μs
 */
uint8_t MPU6050_GetData(void)
{
	uint8_t buf[14];
	uint8_t err;
	
	err = MPU6050_BurstRead(ACCEL_XOUT_H, buf, 14);
	if (err)
	{
		g_mpu_i2c_err = 1;
		return err;  // I2C失败, 本次数据无效
	}
	
	// 解析6轴数据 (大端序: 高字节在前)
	ax = (int16_t)(buf[1]  | (buf[0]  << 8));
	ay = (int16_t)(buf[3]  | (buf[2]  << 8));
	az = (int16_t)(buf[5]  | (buf[4]  << 8));
	// buf[6:7] = 温度, 暂不使用
	gx = (int16_t)(buf[9]  | (buf[8]  << 8));
	gy = (int16_t)(buf[11] | (buf[10] << 8));
	gz = (int16_t)(buf[13] | (buf[12] << 8));
	
	g_mpu_i2c_err = 0;
	return 0;
}



