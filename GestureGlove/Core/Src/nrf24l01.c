#include "nrf24l01.h"
#include <stdio.h>

// 编译期检查：两端数据包必须为20字节，若布局变化将直接编译报错
typedef char nrf_packet_size_check[(sizeof(NRF_DataPacket) == 20) ? 1 : -1];

static SPI_HandleTypeDef *nrf_hspi;

// 通信地址（手套和无人机需要一致）
const uint8_t TX_ADDRESS[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t RX_ADDRESS[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

// CE引脚控制
#define NRF_CE_LOW()   HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET)
#define NRF_CE_HIGH()  HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET)

// CSN引脚控制
#define NRF_CSN_LOW()  HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)
#define NRF_CSN_HIGH() HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET)

// SPI读写
static uint8_t NRF_SPI_ReadWrite(uint8_t data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(nrf_hspi, &data, &rx_data, 1, 100);
    return rx_data;
}

// 写寄存器
void NRF_WriteReg(uint8_t reg, uint8_t value)
{
    NRF_CSN_LOW();
    NRF_SPI_ReadWrite(NRF_CMD_W_REGISTER | reg);
    NRF_SPI_ReadWrite(value);
    NRF_CSN_HIGH();
}

// 读寄存器
uint8_t NRF_ReadReg(uint8_t reg)
{
    uint8_t value;
    NRF_CSN_LOW();
    NRF_SPI_ReadWrite(NRF_CMD_R_REGISTER | reg);
    value = NRF_SPI_ReadWrite(NRF_CMD_NOP);
    NRF_CSN_HIGH();
    return value;
}

// 写多字节寄存器
static void NRF_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    NRF_CSN_LOW();
    NRF_SPI_ReadWrite(NRF_CMD_W_REGISTER | reg);
    for (uint8_t i = 0; i < len; i++) {
        NRF_SPI_ReadWrite(buf[i]);
    }
    NRF_CSN_HIGH();
}

// 初始化nRF24L01+
void NRF24L01_Init(SPI_HandleTypeDef *hspi)
{
    nrf_hspi = hspi;
    
    NRF_CE_LOW();
    NRF_CSN_HIGH();
    HAL_Delay(150);  // 增加延时到150ms，确保nRF完全启动
    
    // 配置为发送模式
    NRF_WriteReg(NRF_REG_CONFIG, 0x0E);      // PWR_UP=1, PRIM_RX=0
    NRF_WriteReg(NRF_REG_EN_AA, 0x01);       // 使能自动应答
    NRF_WriteReg(NRF_REG_EN_RXADDR, 0x01);   // 使能通道0
    NRF_WriteReg(NRF_REG_SETUP_AW, 0x03);    // 地址宽度5字节
    NRF_WriteReg(NRF_REG_SETUP_RETR, 0x1A);  // 重传延迟500us，重传10次
    NRF_WriteReg(NRF_REG_RF_CH, 40);         // 频道40（2440MHz）
    NRF_WriteReg(NRF_REG_RF_SETUP, 0x0F);    // 2Mbps，0dBm
    
    // 设置地址
    NRF_WriteBuf(NRF_REG_TX_ADDR, TX_ADDRESS, 5);
    NRF_WriteBuf(NRF_REG_RX_ADDR_P0, RX_ADDRESS, 5);
    
    // 设置接收数据长度
    NRF_WriteReg(NRF_REG_RX_PW_P0, sizeof(NRF_DataPacket));
    
    // 清除中断标志
    NRF_WriteReg(NRF_REG_STATUS, 0x70);
    
    // 清空FIFO
    NRF_CSN_LOW();
    NRF_SPI_ReadWrite(NRF_CMD_FLUSH_TX);
    NRF_CSN_HIGH();
    
    NRF_CSN_LOW();
    NRF_SPI_ReadWrite(NRF_CMD_FLUSH_RX);
    NRF_CSN_HIGH();
    
    HAL_Delay(10);  // 配置完成后再等待10ms
}

// 检测nRF24L01+是否存在
// 方法：把测试值写入TX_ADDR再读回比较，验证SPI通路。
// 注意：检测完成后必须恢复真实通信地址！
// 之前的bug：检测完TX_ADDR停留在0xA5A5A5A5A5，而接收端监听0xE7E7E7E7E7，
// 导致发送永远得不到ACK（TX:FAIL），手套端信息无法到达无人机端。
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t read_buf[5];
    uint8_t ok = 1;
    
    NRF_WriteBuf(NRF_REG_TX_ADDR, buf, 5);
    
    NRF_CSN_LOW();
    NRF_SPI_ReadWrite(NRF_CMD_R_REGISTER | NRF_REG_TX_ADDR);
    for (uint8_t i = 0; i < 5; i++) {
        read_buf[i] = NRF_SPI_ReadWrite(NRF_CMD_NOP);
    }
    NRF_CSN_HIGH();
    
    for (uint8_t i = 0; i < 5; i++) {
        if (read_buf[i] != 0xA5) {
            ok = 0;
            break;
        }
    }
    
    // 关键修复：恢复真实通信地址（TX_ADDR与RX_ADDR_P0必须与无人机端一致）
    NRF_WriteBuf(NRF_REG_TX_ADDR, TX_ADDRESS, 5);
    NRF_WriteBuf(NRF_REG_RX_ADDR_P0, RX_ADDRESS, 5);
    
    return ok;
}

// 切换到发送模式
void NRF24L01_TxMode(void)
{
    NRF_CE_LOW();
    NRF_WriteReg(NRF_REG_CONFIG, 0x0E);  // PWR_UP=1, PRIM_RX=0
    NRF_CE_HIGH();
    HAL_Delay(1);
}

// 发送数据包
uint8_t NRF24L01_TxPacket(NRF_DataPacket *packet)
{
    uint8_t status;
    
    NRF_CE_LOW();
    
    // 若上次发送失败残留导致TX FIFO已满，先清空FIFO
    if (NRF_ReadReg(NRF_REG_FIFO_STATUS) & 0x20) {
        NRF_CSN_LOW();
        NRF_SPI_ReadWrite(NRF_CMD_FLUSH_TX);
        NRF_CSN_HIGH();
    }
    
    // 写入数据
    NRF_CSN_LOW();
    NRF_SPI_ReadWrite(NRF_CMD_W_TX_PAYLOAD);
    uint8_t *data = (uint8_t*)packet;
    for (uint8_t i = 0; i < sizeof(NRF_DataPacket); i++) {
        NRF_SPI_ReadWrite(data[i]);
    }
    NRF_CSN_HIGH();
    
    // CE拉高至少10us启动发送
    NRF_CE_HIGH();
    HAL_Delay(1);
    NRF_CE_LOW();
    
    // 等待发送完成（自动重传10次×500us≈5ms，10ms超时足够）
    uint8_t timeout = 10;
    while (timeout--) {
        status = NRF_ReadReg(NRF_REG_STATUS);
        if (status & 0x20) {  // TX_DS：发送成功（收到ACK）
            NRF_WriteReg(NRF_REG_STATUS, 0x20);  // 清除标志
            return 1;
        }
        if (status & 0x10) {  // MAX_RT：达到最大重传次数（无ACK）
            NRF_WriteReg(NRF_REG_STATUS, 0x10);  // 清除标志
            NRF_CSN_LOW();
            NRF_SPI_ReadWrite(NRF_CMD_FLUSH_TX);  // 清空TX FIFO
            NRF_CSN_HIGH();
            return 0;
        }
        HAL_Delay(1);
    }
    
    return 0;  // 超时
}

// 调试用：打印nRF24L01+关键寄存器，用于排查通信问题
void NRF24L01_DumpRegs(UART_HandleTypeDef *huart)
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
