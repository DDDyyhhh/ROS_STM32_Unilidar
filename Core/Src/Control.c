// Control.c (使用新单位和封装的 PID 库)

#include "Control.h"
#include "main.h"
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"

// ==================== 【调试模式切换开关】 ====================
// 模式 1: 调试内环 (速度环)
// 模式 2: 调试外环 (位置环)，此时内环参数应已调好
#define CONTROL_MODE 3  // <<< 在这里切换模式：1 或 2 或3
// ==========================================================

// 全局 PID 实例
PID_t pid_speed_right;
PID_t pid_speed_left;
PID_t pid_position_left; // <<<【新增】左轮的位置环 PID
PID_t pid_position_right; // <<<【新增】右轮的位置环 PID

// 全局目标速度 (单位: ticks/10ms)
static float g_target_ticks_left = 0.0f;
static float g_target_position_left = 0.0f;  // 目标位置 (单位: ticks)
static float g_current_position_left = 0.0f; // 当前位置 (单位: ticks
static float g_target_position_right = 0.0f;  // 目标位置 (单位: ticks)
static float g_current_position_right = 0.0f; // 当前位置 (单位: ticks

/**
  * @brief  【新增】增加目标速度 (单位: ticks/10ms)
  */
void Control_Increase_Target_Speed_Ticks(float increment)
{
    g_target_ticks_left += increment;
}

/**
  * @brief  【新增】设置目标位置 (单位: ticks/10ms)
  */

void Control_Set_Target_Position_Left(float ticks)
{
    g_target_position_left = ticks;
}

void Control_Set_Target_Position_Right(float ticks)
{
    g_target_position_right = ticks;
}

/**
  * @brief  初始化所有控制相关的变量
  */
void Control_Init(void)
{
    // 为 PID 设置一个全新的初始值，因为单位变了
    // Kp, Ki, Kd 需要重新调试
		//********************************单环速度环**************************************
//    PID_Init(&pid_speed_left, 0.32f, 0.1f, 0.0f, 100.0f, -100.0f); // Kp 从 1.0 开始尝试
//    PID_Init(&pid_speed_right, 1.0f, 0.0f, 0.0f, 100.0f, -100.0f);
		
		//******************双环pid***********************
// === 1. 初始化【内环-速度环】PID 参数 ===
    // 使用你已经调试好的完美参数！
    // 假设你最终调好的值是 Kp=2.5, Ki=0.8 (这只是示例)
//    PID_Init(&pid_speed_left, 0.32f, 0.1f, 0.0f, 100.0f, -100.0f);
//    PID_Init(&pid_speed_right, 2.5f, 0.8f, 0.0f, 100.0f, -100.0f); // 右轮也用相同参数
		
		
		// === 2. 初始化【外环-位置环】PID 参数 ===
    // 位置环通常只需要 P 控制，或者一个很小的 D 控制。Ki 通常为 0。
    // P 控制器：Kp 决定了小车以多快的速度去接近目标位置。
    // OutMax/OutMin 限制了位置环输出的最大速度。
//    PID_Init(&pid_position_left, 0.1f, 0.0f, 0.0f, 80.0f, -80.0f); // Kp从0.1开始，最大速度限制在80 ticks/10ms
//    PID_Init(&pid_position_right, 0.1f, 0.0f, 0.0f, 80.0f, -80.0f);
		
		
		//*****************************************************
		
		
		//**********************调试模式************************
		#if (CONTROL_MODE == 1) // 如果是模式1：调试速度环
    // === 内环-速度环 PID 参数 ===
    // 从这里开始调试你的 Kp, Ki, Kd
    PID_Init(&pid_speed_left, 0.2f, 0.1f, 0.0f, 100.0f, -100.0f);
    
    // 外环不起作用，参数设为 0
    PID_Init(&pid_position_left, 0.0f, 0.0f, 0.0f, 80.0f, -80.0f);

		#elif (CONTROL_MODE == 2) // 如果是模式2：调试位置环
				// === 内环-速度环 PID 参数 ===
				// 【关键】在这里填入你最终调试好的速度环完美参数！
				// 假设你最终调好的值是 Kp=2.5, Ki=0.8, Kd=0.1
				PID_Init(&pid_speed_left, 0.2f, 0.1f, 0.0f, 100.0f, -100.0f);

				// === 外环-位置环 PID 参数 ===
				// 现在开始调试位置环的 Kp, Ki, Kd
				PID_Init(&pid_position_left, 2.0f, 0.0f, 1.0f, 80.0f, -80.0f);
		
		#elif (CONTROL_MODE == 3) // 如果是模式2：调试位置环
		
				//******************双环pid***********************
// === 1. 初始化【内环-速度环】PID 参数 ===
    // 使用你已经调试好的完美参数！
    // 假设你最终调好的值是 Kp=2.5, Ki=0.8 (这只是示例)
    PID_Init(&pid_speed_left, 0.2f, 0.1f, 0.0f, 100.0f, -100.0f);
    PID_Init(&pid_speed_right, 0.2f, 0.1f, 0.0f, 100.0f, -100.0f); // 右轮也用相同参数
		
		
		// === 2. 初始化【外环-位置环】PID 参数 ===
    // 位置环通常只需要 P 控制，或者一个很小的 D 控制。Ki 通常为 0。
    // P 控制器：Kp 决定了小车以多快的速度去接近目标位置。
    // OutMax/OutMin 限制了位置环输出的最大速度。
    PID_Init(&pid_position_left, 1.0f, 0.0f, 0.9f, 80.0f, -80.0f); // Kp从0.1开始，最大速度限制在80 ticks/10ms
    PID_Init(&pid_position_right, 1.0f, 0.0f, 0.9f, 80.0f, -80.0f);
		
		
		
		
		#endif

    
    
		
		//*******************************************************************

}

/**
  * @brief  主控制循环 - 【在 10ms 定时器中断中被调用】
  */
void Control_Loop(void)
{
    // ============ 左轮 速度环PID 调试 (单位: ticks/10ms) ============

    // 1. 读取由按键控制的全局目标速度
//    float target_ticks_left = g_target_ticks_left;

//    // 2. 读取左轮编码器增量，这就是我们的实际速度
//    float current_ticks_left = (float)Encoder_Get_Left();

//    // 3. 更新 PID 计算，得到 PWM 输出值
//    int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_ticks_left, current_ticks_left);

//    // 4. 应用到左轮电机
//    Motor_SetSpeed_Left(pwm_out_left);
//    
//    // 5. 停止右轮电机
//    Motor_SetSpeed_Right(0);
//    
//    // 6. 打印调试数据
//    Serial_Printf("%f,%f,%d\r\n", target_ticks_left, current_ticks_left, pwm_out_left);





   // 在循环开始时，只获取一次编码器读数
    int32_t ticks_now_left = Encoder_Get_Left();
    int32_t ticks_now_right = Encoder_Get_Right(); // 即使不用，也先读出来保持对称

#if (CONTROL_MODE == 1)
    // ==================== 模式1：调试内环 (速度环) ====================
    
    // 1. 设定一个固定的目标速度 (单位: ticks/10ms)
    float target_speed_left = 10.0f;

    // 2. 获取当前实际速度
    float current_speed_left = (float)ticks_now_left;

    // 3. 更新速度环 PID
    int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_speed_left, current_speed_left);

    // 4. 应用到电机
    Motor_SetSpeed_Left(pwm_out_left);

    // 5. 打印速度环的调试数据
    // 格式: 目标速度, 实际速度, PWM输出
    Serial_Printf("%f,%f,%d\r\n", target_speed_left, current_speed_left, pwm_out_left);


#elif (CONTROL_MODE == 2)
    // ==================== 模式2：调试外环 (位置环) ====================
    // ** 前提：内环的速度 PID 参数已经调试到完美 **

    // 1. 设定一个固定的目标位置 (单位: ticks)
    //    你可以修改这个值来测试不同的定位距离
    float target_position_left = 200.0f;

    // --- 外环 (位置环) 计算 ---
    // 2. 获取编码器增量，并累加得到当前位置
//    g_current_position_left += (float)Encoder_Get_Left();
		
    g_current_position_left += ticks_now_left;
		
    
    // 3. 更新位置环 PID，其输出是【内环的目标速度】
    float target_speed_left = PID_Update(&pid_position_left, target_position_left, g_current_position_left);
    
    // --- 内环 (速度环) 执行 ---
    // 4. 获取当前实际速度
    float current_speed_left = (float)ticks_now_left; // 注意：这里需要重新获取一次，因为上面的累加已经消耗了上次的值
                                                          // 一个更优的做法是把 Encoder_Get_Left() 的值先存起来
                                                          // int32_t ticks_now = Encoder_Get_Left();
                                                          // g_current_position_left += ticks_now;
                                                          // ...
                                                          // float current_speed_left = (float)ticks_now;

    // 5. 更新速度环 PID，其目标是【外环的输出】
    int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_speed_left, current_speed_left);

    // 6. 应用到电机
    Motor_SetSpeed_Left(pwm_out_left);

    // 7. 打印位置环的调试数据
    // 格式: 目标位置, 当前位置, 外环输出(目标速度)
    Serial_Printf("%f,%f,%f\r\n", target_position_left, g_current_position_left, target_speed_left);
		

#elif (CONTROL_MODE == 3)		

 // ==================== 左轮串级 PID 控制 ====================
    
    // --- 外环 (位置环) ---
    // 1. 获取编码器增量，并累加得到当前位置
 
    g_current_position_left += ticks_now_left;

    // 2. 更新位置环 PID，其输出是【内环的目标速度】
    float target_speed_left = PID_Update(&pid_position_left, g_target_position_left, g_current_position_left);
    
    // --- 内环 (速度环) ---
    // 3. 获取当前实际速度 (单位: ticks/10ms)
    float current_speed_left = (float)ticks_now_left;

    // 4. 更新速度环 PID，其目标是【外环的输出】
    int16_t pwm_out_left = (int16_t)PID_Update(&pid_speed_left, target_speed_left, current_speed_left);

    // 5. 应用到电机
    Motor_SetSpeed_Left(pwm_out_left);


    // ==================== 右轮串级 PID 控制 (逻辑相同) ====================
    // (等你的新驱动板到了之后，取消这里的注释即可)
    
     g_current_position_right += ticks_now_right;
     float target_speed_right = PID_Update(&pid_position_right, g_target_position_right, g_current_position_right);
     float current_speed_right = (float)ticks_now_right;
     int16_t pwm_out_right = (int16_t)PID_Update(&pid_speed_right, target_speed_right, current_speed_right);
     Motor_SetSpeed_Right(pwm_out_right);
    
//   Motor_SetSpeed_Right(0); // 调试期间先停止右轮


    // ==================== 串口打印调试信息 ====================
    // 打印位置信息来观察定位效果
    // 格式：目标位置, 当前位置, 外环输出(目标速度), 内环输出(PWM)
    Serial_Printf("%f,%f,%f,%d,%f,%f,%f,%d\r\n", 
                  g_target_position_left, 
                  g_current_position_left, 
                  target_speed_left, 
                  pwm_out_left,
									g_target_position_right, 
                  g_current_position_right, 
                  target_speed_right, 
                  pwm_out_right);

//************************************************************************************

#endif


}