// Control.h

#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
// 函数声明
void Control_Init(void);
void Control_Loop(void);
void Control_Set_Target_Position_Left(float ticks);
void Control_Increase_Target_Speed_Ticks(float increment);
void Control_Set_Target_Position_Right(float ticks);

#endif /* __CONTROL_H */

