/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 手势手套主程序
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "mpu6050.h"
#include "gesture.h"
#include "nrf24l01.h"
#include <stdio.h>

/* 外设句柄由CubeMX生成的文件定义，这里只引用 */
extern I2C_HandleTypeDef  hi2c1;
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();

    HAL_UART_Transmit(&huart1, (uint8_t*)"=== SYSTEM START ===\r\n", 22, 100);

    /* 初始化MPU6050 */
    MPU6050_Data imu_data = {0};
    uint8_t init_ok = MPU6050_Init(&hi2c1);

    /* 读取WHO_AM_I */
    uint8_t who = 0, reg = 0x75;
    HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0xD0, &who, 1, 100);

    /* 初始化nRF24L01+ */
    NRF24L01_Init(&hspi1);
    uint8_t nrf_ok = NRF24L01_Check();
    NRF24L01_TxMode();
    NRF24L01_DumpRegs(&huart1);   // 打印关键寄存器，便于排查通信问题

    /* 启动信息重复打印3秒，确保串口打开后能看到 */
    for (int i = 0; i < 6; i++) {
        char info[100];
        int len = sprintf(info, "=== BOOT === MPU:%d WHO:0x%02X NRF:%s\r\n",
            init_ok, who, nrf_ok ? "OK" : "FAIL");
        HAL_UART_Transmit(&huart1, (uint8_t*)info, len, 200);
        HAL_Delay(500);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)"=== MAIN LOOP START ===\r\n", 25, 100);

    uint32_t timestamp = 0;
    uint32_t last_print_time = 0;
    uint32_t loop_count = 0;
    uint32_t tx_ok_count = 0, tx_fail_count = 0;

    while (1)
    {
        MPU6050_ReadAll(&hi2c1, &imu_data);
        MPU6050_CalcAttitude(&imu_data, 0.01f);
        GestureType gesture = Gesture_Recognize(&imu_data);

        NRF_DataPacket packet = {
            .gesture   = (uint8_t)gesture,
            .roll      = imu_data.roll,
            .pitch     = imu_data.pitch,
            .az        = imu_data.az,
            .timestamp = timestamp++
        };
        uint8_t tx_ok = NRF24L01_TxPacket(&packet);
        if (tx_ok) tx_ok_count++; else tx_fail_count++;
        loop_count++;

        if (HAL_GetTick() - last_print_time >= 500) {
            char buf[150];
            int len = sprintf(buf,
                "[%lu] Roll:%.1f Pitch:%.1f Az:%.2f | %s | TX:%s | cnt:%lu ok:%lu fail:%lu\r\n",
                HAL_GetTick(),
                imu_data.roll, imu_data.pitch, imu_data.az,
                Gesture_GetName(gesture),
                tx_ok ? "OK" : "FAIL",
                loop_count, tx_ok_count, tx_fail_count);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 200);
            last_print_time = HAL_GetTick();
            loop_count = 0;
        }

        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
