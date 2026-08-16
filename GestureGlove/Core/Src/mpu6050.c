#include "mpu6050.h"
#include <math.h>

// 互补滤波系数（0.98表示98%信任陀螺仪，2%信任加速度计）
#define ALPHA 0.98f

// 写寄存器
static void MPU6050_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buf, 2, 100);
}

// 读多个寄存器
static void MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, &reg, 1, 100);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buf, len, 100);
}

// 初始化MPU6050
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0;

    // 检查设备ID
    MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x68) return 0;  // 初始化失败

    // 唤醒设备（清除睡眠位）
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(100);

    // 采样率 = 1kHz / (1 + 9) = 100Hz
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x09);

    // 低通滤波器，截止频率约44Hz
    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, 0x03);

    // 陀螺仪量程 ±250°/s
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, GYRO_RANGE_250);

    // 加速度计量程 ±2g
    MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, ACCEL_RANGE_2G);

    return 1;  // 初始化成功
}

// 读取加速度和陀螺仪原始数据并转换
void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data *data)
{
    uint8_t buf[14];
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;

    // 一次读取14字节（加速度6字节 + 温度2字节 + 陀螺仪6字节）
    MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14);

    raw_ax = (int16_t)(buf[0]  << 8 | buf[1]);
    raw_ay = (int16_t)(buf[2]  << 8 | buf[3]);
    raw_az = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6]和buf[7]是温度，跳过
    raw_gx = (int16_t)(buf[8]  << 8 | buf[9]);
    raw_gy = (int16_t)(buf[10] << 8 | buf[11]);
    raw_gz = (int16_t)(buf[12] << 8 | buf[13]);

    // 转换为物理量
    // ±2g量程：灵敏度16384 LSB/g
    data->ax = raw_ax / 16384.0f;
    data->ay = raw_ay / 16384.0f;
    data->az = raw_az / 16384.0f;

    // ±250°/s量程：灵敏度131 LSB/°/s
    data->gx = raw_gx / 131.0f;
    data->gy = raw_gy / 131.0f;
    data->gz = raw_gz / 131.0f;
}

// 互补滤波计算姿态角（roll和pitch）
void MPU6050_CalcAttitude(MPU6050_Data *data, float dt)
{
    // 用加速度计计算角度
    float accel_roll  = atan2f(data->ay, data->az) * 180.0f / M_PI;
    float accel_pitch = atan2f(-data->ax,
                         sqrtf(data->ay * data->ay + data->az * data->az)) * 180.0f / M_PI;

    // 互补滤波：融合陀螺仪积分和加速度计
    data->roll  = ALPHA * (data->roll  + data->gx * dt) + (1.0f - ALPHA) * accel_roll;
    data->pitch = ALPHA * (data->pitch + data->gy * dt) + (1.0f - ALPHA) * accel_pitch;
}
