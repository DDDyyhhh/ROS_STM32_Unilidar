// Encoder.h
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
int16_t Encoder_Get_Right(void); // 【修正】两个函数都返回 32 位值
int32_t Encoder_Get_Left(void);  // 【修正】因为 TIM2 和 TIM5 都是 32 位定时器

#endif /* __ENCODER_H */