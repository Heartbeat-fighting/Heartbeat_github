#include "msp_protocol.h"
#include <string.h>

static UART_HandleTypeDef *msp_huart;
static uint16_t rc_channels[RC_CHANNEL_COUNT];

// 初始化MSP协议
void MSP_Init(UART_HandleTypeDef *huart)
{
    msp_huart = huart;
    
    // 初始化RC通道为中位值
    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        rc_channels[i] = RC_MID;
    }
    
    // 油门初始化为最低
    rc_channels[RC_THROTTLE] = RC_MIN;
}

// 计算校验和
static uint8_t MSP_Checksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// 发送RC通道值到飞控
void MSP_SendRawRC(uint16_t *channels)
{
    uint8_t buf[32];
    uint8_t idx = 0;
    
    // MSP帧头
    buf[idx++] = MSP_HEADER1;      // '$'
    buf[idx++] = MSP_HEADER2;      // 'M'
    buf[idx++] = MSP_DIRECTION;    // '<'
    
    // 数据长度（8个通道 × 2字节）
    uint8_t data_len = RC_CHANNEL_COUNT * 2;
    buf[idx++] = data_len;
    
    // 命令ID
    buf[idx++] = MSP_SET_RAW_RC;
    
    // RC通道数据（小端序）
    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        buf[idx++] = channels[i] & 0xFF;        // 低字节
        buf[idx++] = (channels[i] >> 8) & 0xFF; // 高字节
    }
    
    // 计算校验和（从data_len开始到数据结束）
    uint8_t checksum = MSP_Checksum(&buf[3], data_len + 2);
    buf[idx++] = checksum;
    
    // 发送到飞控
    HAL_UART_Transmit(msp_huart, buf, idx, 100);
}

// 设置单个通道值
void MSP_SetChannel(RC_Channel ch, uint16_t value)
{
    if (ch < RC_CHANNEL_COUNT) {
        // 限制范围
        if (value < RC_MIN) value = RC_MIN;
        if (value > RC_MAX) value = RC_MAX;
        
        rc_channels[ch] = value;
    }
}

// 获取单个通道值
uint16_t MSP_GetChannel(RC_Channel ch)
{
    if (ch < RC_CHANNEL_COUNT) {
        return rc_channels[ch];
    }
    return RC_MID;
}