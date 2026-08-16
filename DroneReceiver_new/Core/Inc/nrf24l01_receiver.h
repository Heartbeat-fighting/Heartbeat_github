#ifndef __NRF24L01_RECEIVER_H
#define __NRF24L01_RECEIVER_H

#include "stm32f1xx_hal.h"

// nRF24L01+寄存器地址
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_FIFO_STATUS 0x17

// nRF24L01+命令
#define NRF_CMD_R_RX_PAYLOAD  0x61
#define NRF_CMD_W_TX_PAYLOAD  0xA0
#define NRF_CMD_FLUSH_TX      0xE1
#define NRF_CMD_FLUSH_RX      0xE2
#define NRF_CMD_NOP           0xFF

// GPIO引脚定义（根据实际连接修改）
#define NRF_CE_PIN       GPIO_PIN_4
#define NRF_CE_PORT      GPIOA
#define NRF_CSN_PIN      GPIO_PIN_0   // PB0
#define NRF_CSN_PORT     GPIOB

// CE和CSN控制宏
#define NRF_CE_LOW()     HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET)
#define NRF_CE_HIGH()    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET)
#define NRF_CSN_LOW()    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)
#define NRF_CSN_HIGH()   HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET)

// 接收数据包结构（与手套端 nrf24l01.h 中的定义必须完全一致！）
// 当前布局（ARM 32位、自然对齐）：gesture(1) + 填充(3) + 4×float/uint32 = 20 字节
typedef struct {
    uint8_t  gesture;      // 手势类型（GestureType 枚举值，0~8）
    float    roll;         // 横滚角
    float    pitch;        // 俯仰角
    float    az;           // Z轴加速度
    uint32_t timestamp;    // 时间戳
} NRF_DataPacket;

// 函数声明
void NRF24L01_Receiver_Init(SPI_HandleTypeDef *hspi);
uint8_t NRF24L01_Receiver_Check(void);
uint8_t NRF24L01_Receiver_Available(void);
uint8_t NRF24L01_Receiver_Read(NRF_DataPacket *packet);
void NRF24L01_Receiver_DumpRegs(UART_HandleTypeDef *huart);

#endif