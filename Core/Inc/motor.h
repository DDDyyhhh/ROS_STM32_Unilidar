// Motor.h

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h" // 包含 CubeMX 生成的主头文件
#include <stdint.h> // 包含标准整数类型定义

// 函数声明
void Motor_Init(void);
void Motor_SetSpeed_Right(int16_t Speed);
void Motor_SetSpeed_Left(int16_t Speed);

#endif /* __MOTOR_H */