// 临时测试代码：在DroneReceiver的main.c中添加
// 用于测试MSP通信，不依赖nRF24L01+

// 在main函数的while循环中添加这段测试代码：
/*
// 测试MSP通信 - 模拟手势数据
static uint32_t test_time = 0;
if (HAL_GetTick() - test_time > 1000) {  // 每秒发送一次测试数据
    // 模拟不同手势的RC通道值
    uint16_t test_channels[8] = {
        1600,  // Roll - 向右倾斜
        1400,  // Pitch - 向前倾斜  
        1200,  // Throttle - 低油门
        1500,  // Yaw - 中位
        1000,  // AUX1 - 未解锁
        1500,  // AUX2 - 中位
        1500,  // AUX3 - 中位
        1500   // AUX4 - 中位
    };
    
    MSP_SendRawRC(test_channels);
    
    // 调试输出
    HAL_UART_Transmit(&huart1, (uint8_t*)"Test MSP sent\r\n", 15, 100);
    
    test_time = HAL_GetTick();
}
*/