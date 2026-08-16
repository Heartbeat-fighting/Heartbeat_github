#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f1xx_hal.h"
#include "gesture.h"

// nRF24L01+ 引脚定义
#define NRF_CE_PIN       GPIO_PIN_8
#define NRF_CE_PORT      GPIOA
#define NRF_CSN_PIN      GPIO_PIN_4
#define NRF_CSN_PORT     GPIOA

// nRF24L01+ 寄存器地址
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

// nRF24L01+ 命令
#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_R_RX_PAYLOAD  0x61
#define NRF_CMD_W_TX_PAYLOAD  0xA0
#define NRF_CMD_FLUSH_TX      0xE1
#define NRF_CMD_FLUSH_RX      0xE2
#define NRF_CMD_NOP           0xFF

// 数据包结构
// 注意：该结构必须与无人机端 nrf24l01_receiver.h 中的定义保持完全一致！
// 当前布局（ARM 32位、自然对齐）：gesture(1) + 填充(3) + 4×float/uint32 = 20 字节
typedef struct {
    uint8_t  gesture;    // 手势类型（GestureType 枚举值，0~8）
    float    roll;       // 横滚角（度）
    float    pitch;      // 俯仰角（度）
    float    az;         // Z轴加速度（g）
    uint32_t timestamp;  // 时间戳
} NRF_DataPacket;

// 函数声明
void NRF24L01_Init(SPI_HandleTypeDef *hspi);
uint8_t NRF24L01_Check(void);
void NRF24L01_TxMode(void);
uint8_t NRF24L01_TxPacket(NRF_DataPacket *packet);

// 调试用函数声明
uint8_t NRF_ReadReg(uint8_t reg);
void NRF_WriteReg(uint8_t reg, uint8_t value);
void NRF24L01_DumpRegs(UART_HandleTypeDef *huart);

#endif
