// Key.h

#ifndef __KEY_H
#define __KEY_H

#include "main.h" // 包含主头文件以获取 GPIO 宏定义

// 定义键值，方便扩展
#define KEY0_PRES 	1  // KEY0 按下
#define KEY1_PRES	2  // KEY1 按下 (备用)
#define KEY2_PRES	3  // KEY2 按下 (备用)
#define WKUP_PRES   4  // WK_UP 按下 (备用)


// 函数声明
void Key_Init(void);
uint8_t Key_GetNum(void);

#endif