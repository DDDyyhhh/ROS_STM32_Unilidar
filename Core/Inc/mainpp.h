/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
extern "C"
{
#endif
    
void setup(void);
void loop(void);
extern double vel[3];
void rosserial_rx_cb(void); // 用于接收回调
void rosserial_tx_cb(void); // 用于发送回调
    
#ifdef __cplusplus
}
#endif

#endif /* MAINPP_H_ */

