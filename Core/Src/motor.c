// Motor.c

#include "Motor.h"

// 【修正】外部引用句柄，与你的新配置完全匹配
extern TIM_HandleTypeDef htim3; // 左右轮 PWM
extern TIM_HandleTypeDef htim4; // 右轮编码器
extern TIM_HandleTypeDef htim2; // 左轮编码器

void Motor_Init(void)
{
    // 启动 PWM 通道
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // 启动 PB0 (右轮 PWM)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 启动 PB1 (左轮 PWM)
    
    // 启动编码器接口
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 【修正】启动 TIM4 (右轮编码器)
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // 【修正】启动 TIM2 (左轮编码器)
}

// 右轮控制函数 - PWM 使用 TIM3_CH1
void Motor_SetSpeed_Right(int16_t Speed)
{
    if (Speed > 100)  Speed = 100;
    if (Speed < -100) Speed = -100;

    if (Speed >= 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Speed);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -Speed);
    }
}

// 左轮控制函数 - PWM 使用 TIM3_CH2
void Motor_SetSpeed_Left(int16_t Speed)
{
    if (Speed > 100)  Speed = 100;
    if (Speed < -100) Speed = -100;

    if (Speed >= 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Speed);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -Speed);
    }
}