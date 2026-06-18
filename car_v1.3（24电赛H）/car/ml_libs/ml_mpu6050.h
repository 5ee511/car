#ifndef _mpu6050_h
#define _mpu6050_h
#include "stdint.h"
#include "ml_i2c.h"
#include "math.h"

#define MPU6050_ADDR	 0xd0 
#define SMPLRT_DIV       0x19
#define CONFIG           0x1a
#define GYRO_CONFIG      0x1b
#define ACCEL_CONFIG     0x1c
#define ACCEL_XOUT_H     0x3b
#define ACCEL_XOUT_L     0x3c
#define ACCEL_YOUT_H     0x3d
#define ACCEL_YOUT_L     0x3e
#define ACCEL_ZOUT_H     0x3f
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x65
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define PWR_MGMT_1       0x6b
#define PWR_MGMT_2       0x6c
#define INT_ENABLE       0x38
#define WHO_AM_I         0x75

extern int16_t ax, ay, az, gx, gy, gz;
extern float roll_gyro, pitch_gyro, yaw_gyro;
extern float roll_acc, pitch_acc, yaw_acc;
extern float roll_Kalman, pitch_Kalman, yaw_Kalman;

// 角度PD控制用 — 纯陀螺积分Yaw (无磁力计)
extern float g_yaw;             // 累计偏航角, 上电归零
extern float gyro_bias_z;       // 陀螺Z零偏, 启动时校准

// I2C通信诊断
extern volatile uint8_t g_mpu_i2c_err;  // 0=OK, 1=ACK失败, 2=ID不匹配

uint8_t MPU6050_Init(void);              // 返回0=成功, 1=WHO_AM_I失败
uint8_t MPU6050_GetData(void);           // 返回0=成功, 非0=I2C错误
void    MPU6050_CalGyroBias(void);       // 上电采集零偏 (需静止!)

#endif

