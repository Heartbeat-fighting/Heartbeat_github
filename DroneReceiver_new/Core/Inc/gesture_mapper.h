#ifndef __GESTURE_MAPPER_H
#define __GESTURE_MAPPER_H

#include "stm32f1xx_hal.h"
#include "nrf24l01_receiver.h"
#include "msp_protocol.h"

// 手势类型（与手套端一致）
typedef enum {
    GESTURE_NONE = 0,
    GESTURE_HOVER,
    GESTURE_FORWARD,
    GESTURE_BACKWARD,
    GESTURE_LEFT,
    GESTURE_RIGHT,
    GESTURE_UP,
    GESTURE_DOWN,
    GESTURE_EMERGENCY_STOP
} GestureType;

// 映射参数
#define ANGLE_TO_RC_SCALE  20.0f   // 角度到RC值的缩放系数
#define MAX_ANGLE          45.0f   // 最大倾角限制

// 函数声明
void GestureMapper_Init(void);
void GestureMapper_Update(NRF_DataPacket *packet);
void GestureMapper_GetRCChannels(uint16_t *channels);
uint8_t GestureMapper_IsArmed(void);

#endif