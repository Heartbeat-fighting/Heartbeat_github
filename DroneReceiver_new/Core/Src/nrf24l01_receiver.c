#include "nrf24l01_receiver.h"
#include <stdio.h>

// 编译期检查：两端数据包必须为20字节，若布局变化将直接编译报错
typedef char nrf_packet_size_check[(sizeof(NRF_DataPacket) == 20) ? 1 : -1];

static SPI_HandleTypeDef *nrf_hspi;

// 通信地址（必须与手套端一致！）
static const uint8_t RX_ADDR[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

// 单字节SPI收发（nRF24L01的读寄存器 = 发命令字节 + 收1字节）
static uint8_t NRF_SPI_RW(uint8_t data)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(nrf_hspi, &data, &rx_data, 1, 100);
    return rx_data;
}

// 读寄存器（R_REGISTER=0x00，命令字节即寄存器地址）
static uint8_t NRF_ReadReg(uint8_t reg)
{
    uint8_t value;
    NRF_CSN_LOW();
    value = NRF_SPI_RW(reg);
    NRF_CSN_HIGH();
    return value;
}

// 写寄存器
static void NRF_WriteReg(uint8_t reg, uint8_t value)
{
    NRF_CSN_LOW();
    NRF_SPI_RW(reg | 0x20);   // W_REGISTER命令
    NRF_SPI_RW(value);
    NRF_CSN_HIGH();
}

// 写多字节寄存器
static void NRF_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    NRF_CSN_LOW();
    NRF_SPI_RW(reg | 0x20);   // W_REGISTER命令
    for (uint8_t i = 0; i < len; i++) {
        NRF_SPI_RW(buf[i]);
    }
    NRF_CSN_HIGH();
}

// 初始化为接收模式
void NRF24L01_Receiver_Init(SPI_HandleTypeDef *hspi)
{
    nrf_hspi = hspi;
    
    NRF_CE_LOW();
    NRF_CSN_HIGH();
    HAL_Delay(100);
    
    // 配置为接收模式
    NRF_WriteReg(NRF_REG_CONFIG, 0x0F);      // PWR_UP=1, PRIM_RX=1, CRC 2字节
    NRF_WriteReg(NRF_REG_EN_AA, 0x01);       // 使能通道0自动应答（必须，否则不发ACK）
    NRF_WriteReg(NRF_REG_EN_RXADDR, 0x01);   // 使能接收通道0
    NRF_WriteReg(NRF_REG_SETUP_AW, 0x03);    // 地址宽度5字节
    NRF_WriteReg(NRF_REG_SETUP_RETR, 0x1A);  // 自动重传：500us延迟，10次重传
    NRF_WriteReg(NRF_REG_RF_CH, 40);         // 频道40（2.440GHz），与手套端一致
    NRF_WriteReg(NRF_REG_RF_SETUP, 0x0F);    // 2Mbps, 0dBm，与手套端一致
    NRF_WriteReg(NRF_REG_RX_PW_P0, sizeof(NRF_DataPacket));  // 接收数据长度
    
    // 设置接收地址
    NRF_WriteBuf(NRF_REG_RX_ADDR_P0, RX_ADDR, 5);
    
    // 清除中断标志
    NRF_WriteReg(NRF_REG_STATUS, 0x70);
    
    // 清空RX FIFO
    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_CMD_FLUSH_RX);
    NRF_CSN_HIGH();
    
    // 启动接收（CE拉高进入PRX模式）
    NRF_CE_HIGH();
    HAL_Delay(2);
}

// 检查nRF24L01+是否正常
// 方法：写测试值到TX_ADDR再读回比较，验证SPI通路。
// 检测后恢复接收地址，确保寄存器状态与初始化一致。
uint8_t NRF24L01_Receiver_Check(void)
{
    uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t read_buf[5];
    uint8_t ok = 1;
    
    // 写入测试地址
    NRF_WriteBuf(NRF_REG_TX_ADDR, buf, 5);
    
    // 读回验证
    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_REG_TX_ADDR);   // R_REGISTER | TX_ADDR
    for (uint8_t i = 0; i < 5; i++) {
        read_buf[i] = NRF_SPI_RW(0xFF);
    }
    NRF_CSN_HIGH();
    
    // 比较
    for (uint8_t i = 0; i < 5; i++) {
        if (read_buf[i] != 0xA5) {
            ok = 0;
            break;
        }
    }
    
    // 恢复接收地址（防止检测过程残留状态影响接收）
    NRF_WriteBuf(NRF_REG_RX_ADDR_P0, RX_ADDR, 5);
    
    return ok;
}

// 检查是否有数据可读
uint8_t NRF24L01_Receiver_Available(void)
{
    uint8_t status = NRF_ReadReg(NRF_REG_STATUS);
    
    // 检查RX_DR标志（bit 6）
    if (status & 0x40) {
        return 1;
    }
    return 0;
}

// 读取接收到的数据
uint8_t NRF24L01_Receiver_Read(NRF_DataPacket *packet)
{
    uint8_t status = NRF_ReadReg(NRF_REG_STATUS);
    uint8_t fifo   = NRF_ReadReg(NRF_REG_FIFO_STATUS);
    
    // RX_DR置位且RX FIFO非空才读取，避免读到空FIFO的垃圾数据
    if ((status & 0x40) && !(fifo & 0x01)) {
        NRF_CSN_LOW();
        NRF_SPI_RW(NRF_CMD_R_RX_PAYLOAD);
        for (uint8_t i = 0; i < sizeof(NRF_DataPacket); i++) {
            ((uint8_t*)packet)[i] = NRF_SPI_RW(0xFF);
        }
        NRF_CSN_HIGH();
        
        // 只清除RX_DR标志
        NRF_WriteReg(NRF_REG_STATUS, 0x40);
        
        return 1;
    }
    return 0;
}

// 调试用：打印nRF24L01+关键寄存器，用于排查通信问题
void NRF24L01_Receiver_DumpRegs(UART_HandleTypeDef *huart)
{
    char buf[120];
    int len = sprintf(buf,
        "NRF: CONFIG=0x%02X STATUS=0x%02X RF_CH=%u RF_SETUP=0x%02X FIFO=0x%02X\r\n",
        NRF_ReadReg(NRF_REG_CONFIG),
        NRF_ReadReg(NRF_REG_STATUS),
        NRF_ReadReg(NRF_REG_RF_CH),
        NRF_ReadReg(NRF_REG_RF_SETUP),
        NRF_ReadReg(NRF_REG_FIFO_STATUS));
    HAL_UART_Transmit(huart, (uint8_t*)buf, len, 200);
}
