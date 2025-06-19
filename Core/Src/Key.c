// Key.c

#include "Key.h"
#include "stm32f4xx_hal.h" // 包含 HAL 库定义

/**
  * @brief  按键初始化函数
  * @param  None
  * @retval None
  * @note   实际的 GPIO 初始化由 CubeMX 在 MX_GPIO_Init() 中完成，
  *         此函数目前为空，但保留是为了保持代码结构的完整性。
  */
void Key_Init(void)
{
    // GPIO 的初始化已由 CubeMX 在 main() 函数调用的 MX_GPIO_Init() 中自动完成。
    // 所以这个函数可以是空的，或者未来用于更复杂的按键初始化。
}


/**
  * @brief  获取按键键值 (非阻塞，带消抖)
  * @param  None
  * @retval 返回按下的键值。0: 无按键按下; 1: KEY0 按下
  * @note   此函数应在主循环中被反复调用。
  */
uint8_t Key_GetNum(void)
{
    uint8_t key_num = 0; // 默认没有按键按下

    // --- 检测 KEY0 (PE4) ---
    // 读取 KEY0 引脚的电平。由于是上拉输入，按下时为低电平 (GPIO_PIN_RESET)
    if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(20); // 延时 20ms 进行软件消抖
        // 再次确认按键仍然是按下的状态
        if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
        {
            // 等待按键释放，这是一个阻塞式等待，确保一次按下只触发一次
            while(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET);
            
            key_num = KEY0_PRES; // 确认是 KEY0 按下
        }
    }
    
    // --- 在这里可以添加对其他按键的检测 ---
    // if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) { ... }

    return key_num;
}