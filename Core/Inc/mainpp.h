/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_
#include <math.h>
#ifdef __cplusplus
extern "C"
{
#endif
extern float linear_velocity_x;
extern float angular_velocity_z;    
void setup(void);
void loop(void);
extern double vel[3];
void rosserial_rx_cb(void); // 用于接收回调
void rosserial_tx_cb(void); // 用于发送回调
void rosserial_idle_cb(void); 
extern float g_target_speed_left_cmps_debug;
extern float g_current_speed_left_cmps_debug;
extern int16_t g_pwm_out_left_debug;   
#ifdef __cplusplus
}
#endif

#endif /* MAINPP_H_ */

