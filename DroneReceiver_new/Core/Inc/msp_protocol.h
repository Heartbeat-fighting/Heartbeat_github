#ifndef __MSP_PROTOCOL_H
#define __MSP_PROTOCOL_H

#include "stm32f1xx_hal.h"

// MSP协议定义
#define MSP_HEADER1  '$'
#define MSP_HEADER2  'M'
#define MSP_DIRECTION '<'  // 发送到飞控

// MSP命令
#define MSP_SET_RAW_RC  200  // 设置RC通道值

// RC通道数量
#define RC_CHANNEL_COUNT  8

// RC通道值范围
#define RC_MIN  1000
#define RC_MID  1500
#define RC_MAX  2000

// RC通道映射
typedef enum {
    RC_ROLL = 0,      // 通道1：横滚
    RC_PITCH,         // 通道2：俯仰
    RC_THROTTLE,      // 通道3：油门
    RC_YAW,           // 通道4：偏航
    RC_AUX1,          // 通道5：ARM开关
    RC_AUX2,          // 通道6：飞行模式
    RC_AUX3,          // 通道7：保留
    RC_AUX4           // 通道8：保留
} RC_Channel;

// 函数声明
void MSP_Init(UART_HandleTypeDef *huart);
void MSP_SendRawRC(uint16_t *channels);
void MSP_SetChannel(RC_Channel ch, uint16_t value);
uint16_t MSP_GetChannel(RC_Channel ch);

#endif