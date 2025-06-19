#include "mainpp.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "main.h"
#include "pid.h"
#include "Motor.h" // 假设你已经有了电机驱动库
#include "std_msgs/Float32.h" // <<<【新增】包含 Float32 消息头文件
double vel[3];

extern "C" {
  float linear_velocity_x = 0.0f;
  float angular_velocity_z = 0.0f;
}

// 声明由 C 文件实现的接口函数
extern "C" {
  void Control_Set_Target_Position_Left(float ticks);
  void Control_Set_Target_Position_Right(float ticks); // 为右轮也准备好
}

void cmd_vel_cb(const geometry_msgs::Twist& msg) {
  linear_velocity_x = msg.linear.x;
  angular_velocity_z = msg.angular.z;
}
// === 回调函数 ===
// 当收到 /target_position_left 话题时，此函数被调用
void position_cb_left(const std_msgs::Float32& msg) {
  // 调用 C 函数，将 ROS 发来的目标位置设置给控制层
  Control_Set_Target_Position_Left(msg.data);
}


void vel_callback(const geometry_msgs::Twist &msg)
{
   vel[0] = msg.linear.x;  //double vel[3]������mainpp.h
   vel[1] = msg.linear.y;
   vel[2] = msg.angular.z;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("give_car_speed", vel_callback);

// 创建一个订阅者，订阅名为 "target_position_left" 的话题
ros::Subscriber<std_msgs::Float32> sub_pos_left("target_position_left", position_cb_left);

extern "C" {
  void rosserial_rx_cb(void) {
    nh.getHardware()->reset_rbuf();
  }
  
  void rosserial_tx_cb(void) {
    nh.getHardware()->flush();
  }
}

void setup(void)
{
    nh.initNode();
    //nh.subscribe(sub);
		nh.subscribe(sub_pos_left);
	
}
void loop(void)
{
    nh.spinOnce();
		HAL_Delay(10); // loop 里可以留一个小延时
}


