#include "mainpp.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "main.h"
#include "pid.h"
#include "Motor.h" // 假设你已经有了电机驱动库
#include "std_msgs/Float32.h" // <<<【新增】包含 Float32 消息头文件
#include <nav_msgs/Odometry.h>     // <<< 包含 Odometry 消息
#include <tf/transform_broadcaster.h> // <<< 包含 tf 广播头文件
#include <tf/tf.h>
#include "geometry_msgs/Vector3Stamped.h"
double vel[3];

// === 【新增】PID 调试信息发布者 ===
geometry_msgs::Vector3Stamped pid_debug_msg;
ros::Publisher pid_debug_pub("pid_debug", &pid_debug_msg);
char debug_frame[] = "pid_debug_frame"; // 随便起一个 frame_id

// 从 C 代码获取 PID 调试数据
extern "C" {
  extern float g_target_speed_left_cmps_debug;
  extern float g_current_speed_left_cmps_debug;
  extern int16_t g_pwm_out_left_debug;
}

// === C/C++ 共享变量 ===
extern "C" {
  float linear_velocity_x = 0.0f;  // m/s
  float angular_velocity_z = 0.0f; // rad/s
  // 从 Control.c 获取里程计数据
  extern float g_pos_x_cm, g_pos_y_cm, g_pos_th_rad;
  extern float g_vel_vx_cmps, g_vel_vth_radps;
}

// 声明由 C 文件实现的接口函数
extern "C" {
  void Control_Set_Target_Position_Left(float ticks);
  void Control_Set_Target_Position_Right(float ticks); // 为右轮也准备好
}

extern "C" {
  void rosserial_idle_cb(void) {
    // 这个函数可以什么都不做，因为 yoneken 的 read() 是轮询式的
    // 但保留这个接口，是为了将来可能的优化
  }
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


// === ROS 对象定义 ===
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", cmd_vel_cb);

// 里程计发布
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

// TF 广播
geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";

char odom[] = "/odom";



//extern "C" {
//  void rosserial_rx_cb(void) {
//    nh.getHardware()->reset_rbuf();
//  }
//  
//  void rosserial_tx_cb(void) {
//    nh.getHardware()->flush();
//  }
//}

void setup(void)
{
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.advertise(odom_pub);
  broadcaster.init(nh); // 初始化 TF 广播
	nh.advertise(pid_debug_pub);
}
void loop(void)
{
    ros::Time current_time = nh.now();
    
    // 1. 填充并发布 odom -> base_link 的 TF 变换
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom;
    odom_trans.child_frame_id = base_link; // <<<【修正】统一为 base_link
    odom_trans.transform.translation.x = g_pos_x_cm / 100.0;
    odom_trans.transform.translation.y = g_pos_y_cm / 100.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionFromYaw(g_pos_th_rad);
    broadcaster.sendTransform(odom_trans);

    // 2. 填充并发布 Odometry 消息
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.child_frame_id = base_link; // <<<【修正】统一为 base_link
    odom_msg.pose.pose.position.x = g_pos_x_cm / 100.0;
    odom_msg.pose.pose.position.y = g_pos_y_cm / 100.0;
    odom_msg.pose.pose.orientation = odom_trans.transform.rotation;
    odom_msg.twist.twist.linear.x = g_vel_vx_cmps / 100.0;
    odom_msg.twist.twist.angular.z = g_vel_vth_radps;
    // 【重要】填充协方差矩阵，告诉上层算法里程计的可信度
    // 这里我们先用一个简化的、比较理想的协方差
    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[35] = 0.003;
    odom_msg.twist.covariance[0] = 0.001;
    odom_msg.twist.covariance[35] = 0.003;
    odom_pub.publish(&odom_msg);
		
		// 可以在这里也加上频率控制，或者每次都发
    pid_debug_msg.header.stamp = nh.now();
    pid_debug_msg.header.frame_id = debug_frame;
    pid_debug_msg.vector.x = g_target_speed_left_cmps_debug; // 目标速度
    pid_debug_msg.vector.y = g_current_speed_left_cmps_debug; // 实际速度
    pid_debug_msg.vector.z = g_pwm_out_left_debug;           // PWM 输出
    pid_debug_pub.publish(&pid_debug_msg);
    
    nh.spinOnce();
}


