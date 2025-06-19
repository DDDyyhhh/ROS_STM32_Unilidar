#include "mainpp.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "main.h"
#include "pid.h"
#include "Motor.h" // 假设你已经有了电机驱动库


extern "C" {
  float linear_velocity_x = 0.0f;
  float angular_velocity_z = 0.0f;
}

void cmd_vel_cb(const geometry_msgs::Twist& msg) {
  linear_velocity_x = msg.linear.x;
  angular_velocity_z = msg.angular.z;
}

double vel[3];

void vel_callback(const geometry_msgs::Twist &msg)
{
   vel[0] = msg.linear.x;  //double vel[3]������mainpp.h
   vel[1] = msg.linear.y;
   vel[2] = msg.angular.z;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("give_car_speed", vel_callback);

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
    nh.subscribe(sub);
	
}
void loop(void)
{
    nh.spinOnce();
}


