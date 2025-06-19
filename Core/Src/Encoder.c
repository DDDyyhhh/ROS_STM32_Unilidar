// Encoder.c
#include "Encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim4; // 【修改】右轮编码器现在是 TIM4
extern TIM_HandleTypeDef htim2; // 【修改】左轮编码器

// 【修改】返回值和 Temp 变量类型改为 int16_t
int16_t Encoder_Get_Right(void) 
{
    int16_t Temp = __HAL_TIM_GET_COUNTER(&htim4); // 【修改】使用 htim4
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return Temp;
}

int32_t Encoder_Get_Left(void)
{
    // 左轮的保持不变 (使用 TIM2)
    int32_t Temp = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return Temp;
}