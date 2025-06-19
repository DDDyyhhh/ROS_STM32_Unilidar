# ROS 与 STM32 履带底盘控制项目

这是一个基于 STM32F407ZGT6 的履带式机器人底层控制项目，通过 `rosserial` 协议与上位机（如树莓派、香橙派等运行 ROS 的设备）进行通信，实现了对底盘电机的速度和位置闭环 PID 控制。

本项目代码经历了详尽的调试过程，解决了从硬件配置、编译器兼容性到 `rosserial` 通信、PID 参数整定等一系列常见问题，旨在为构建类似的机器人底层系统提供一个稳定、可靠、易于扩展的参考框架。

## 项目亮点

*   **模块化代码结构**: 将电机驱动 (`Motor`)、编码器读取 (`Encoder`)、PID 算法 (`PID`)、调试串口 (`Serial`)、主控制逻辑 (`Control`) 等功能解耦，代码结构清晰，易于维护和移植。
*   **串级 PID 控制**: 实现了**位置环(外环) + 速度环(内环)**的串级 PID 控制，保证了定位的精确性和响应的平稳性。
*   **`rosserial` 通信**: 通过订阅 `/cmd_vel` (`geometry_msgs/Twist`) 话题接收上位机指令，实现了与 ROS 生态的无缝对接。
*   **强大的调试框架**: 内置了调试模式切换功能，可通过修改宏定义，方便地对内环（速度）和外环（位置）进行独立的 PID 参数整定，并通过串口打印实时数据，配合 `SerialPlot` 等工具进行可视化调试。
*   **硬件抽象**: 所有硬件相关的初始化和底层操作均由 STM32CubeMX 配置生成，并由HAL库函数实现，具有良好的可移植性。

## 技术栈

*   **主控 MCU**: STM32F407ZGT6
*   **电机驱动**: TB6612FNG
*   **编码器**: 13PPR AB相增量式编码器 (带 1:30 减速箱)
*   **开发环境**: STM32CubeIDE / Keil MDK 5 (已配置为 ARM Compiler 6)
*   **通信协议**: `rosserial` over UART
*   **上位机 ROS**: ROS Noetic

---

## 快速上手与配置

要将此项目应用到你自己的机器人上，你需要根据你的具体硬件情况，修改以下几个关键参数。

### **【重要】需要你修改的参数**

所有需要用户修改的核心参数都集中在 **`Core/Src/Control.c`** 文件中。

```c
// Core/Src/Control.c

// ====== 1. 机器人物理参数 ======
// 这些参数必须根据你的机器人实际测量值进行修改！

// 【修改】两个驱动轮中心之间的距离 (单位: 厘米)
const float WHEEL_SEPARATION_CM = 22.0f;

// 【修改】编码器输出轴（即履带驱动轮轴）每转一圈，编码器产生的总脉冲数
// 计算公式: 电机PPR * 4 (四倍频) * 减速比
// 示例: 13 * 4 * 30 = 1560
const float ENCODER_PULSES_PER_REV = 1560.0f;

// 【标定】每前进 1 厘米，编码器产生的脉冲数。
// 这是最关键的参数，强烈建议通过“开环前进”实验精确标定！
// 理论计算: TICKS_PER_CM = ENCODER_PULSES_PER_REV / 履带周长(cm)
const float TICKS_PER_CM = 25.16f;


// ====== 2. PID 参数 ======
// 这些参数需要你通过串口绘图工具，针对你的电机和负载进行反复调试。

void Control_Init(void)
{
    // 【调试】内环-速度环 PID 参数
    // 先调 Kp，再调 Ki, Kd
    PID_Init(&pid_speed_left, 2.5f, 0.8f, 0.1f, 100.0f, -100.0f);
    PID_Init(&pid_speed_right, 2.5f, 0.8f, 0.1f, 100.0f, -100.0f);

    // 【调试】外环-位置环 PID 参数
    // 通常 Ki 为 0
    PID_Init(&pid_position_left, 0.1f, 0.0f, 0.0f, 80.0f, -80.0f);
    PID_Init(&pid_position_right, 0.1f, 0.0f, 0.0f, 80.0f, -80.0f);
}
Use code with caution.
 

【重要】硬件引脚配置 (.ioc 文件)
本项目的引脚分配经过精心设计，以避开常见冲突。请在 STM32CubeMX 中打开 .ioc 文件，确保你的硬件接线与以下配置一致：
功能	定时器/GPIO	STM32 引脚
右轮 PWM	TIM3_CH4	PB1
左轮 PWM	TIM3_CH3	PB0
右轮编码器	TIM4_CH1/2	PB6 / PB7
左轮编码器	TIM2_CH1/2	PA15 / PB3
右轮方向 1	GPIO AIN1	PG14
右轮方向 2	GPIO AIN2	PB15
左轮方向 1	GPIO BIN1	PC4
左轮方向 2	GPIO BIN2	PC5
ROS 通信	USART3	PB10 / PB11 (或你选择的其他)
调试串口	USART1	PA9 / PA10
控制周期	TIM6	(无引脚)
用户按键	GPIO KEY0	PE4

**调试流程
本项目内置了强大的调试模式，位于 Core/Src/Control.c 的顶部。
Generated c
#define CONTROL_MODE 1 // 1: 调速度环, 2: 调位置环, 3: 正常运行
Use code with caution.
C
调试速度环 (内环):
设置 CONTROL_MODE 为 1。
在 Control_Init() 中修改 pid_speed_... 的参数。
编译烧录，通过调试串口连接 SerialPlot 等工具，观察波形。
目标：找到一组能让实际速度快速、平稳地跟随目标速度的 Kp, Ki, Kd 值。
调试位置环 (外环):
设置 CONTROL_MODE 为 2。
在 Control_Init() 中，将 pid_speed_... 的参数固定为你刚刚调好的值。
开始修改 pid_position_... 的参数（通常从 Kp 开始）。
编译烧录，观察波形。
目标：找到一组能让当前位置快速、平稳地到达目标位置的参数。
正常运行:
设置 CONTROL_MODE 为 3。
取消 main.c 中对 setup() 和 loop() 的注释，恢复 ROS 通信。
现在，你可以通过在上位机发布 /cmd_vel 话题来控制你的机器人了。
