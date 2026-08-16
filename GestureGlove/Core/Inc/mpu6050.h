#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"

// MPU6050 I2C地址（AD0接GND时为0x68）
#define MPU6050_ADDR        (0x68 << 1)

// 寄存器地址
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_WHO_AM_I     0x75

// 量程配置
#define ACCEL_RANGE_2G   0x00   // ±2g，灵敏度16384 LSB/g
#define GYRO_RANGE_250   0x00   // ±250°/s，灵敏度131 LSB/°/s

// 数据结构
typedef struct {
    float ax, ay, az;   // 加速度 (g)
    float gx, gy, gz;   // 角速度 (°/s)
    float roll, pitch;  // 姿态角 (°)
} MPU6050_Data;

// 函数声明
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
void MPU6050_CalcAttitude(MPU6050_Data *data, float dt);

#endif
