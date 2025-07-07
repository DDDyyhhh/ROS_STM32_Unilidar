/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#define STM32F4xx  // Change for your device
#ifdef STM32F3xx
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#endif /* STM32F3xx */
#ifdef STM32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#endif /* STM32F4xx */
#ifdef STM32F7xx
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#endif /* STM32F7xx */

extern UART_HandleTypeDef huart3;

//class STM32Hardware {
//  protected:
//    UART_HandleTypeDef *huart;

//    const static uint16_t rbuflen = 512;
//    uint8_t rbuf[rbuflen];
//    uint32_t rind;
//    inline uint32_t getRdmaInd(void){ return (rbuflen - __HAL_DMA_GET_COUNTER(huart->hdmarx)) & (rbuflen - 1); }

//    const static uint16_t tbuflen = 512;
//    uint8_t tbuf[tbuflen];
//    uint32_t twind, tfind;

//  public:
//    STM32Hardware():
//      huart(&huart3), rind(0), twind(0), tfind(0){
//    }

//    STM32Hardware(UART_HandleTypeDef *huart_):
//      huart(huart_), rind(0), twind(0), tfind(0){
//    }
//  
//    void init(){
//      reset_rbuf();
//    }

//    void reset_rbuf(void){
//      HAL_UART_Receive_DMA(huart, rbuf, rbuflen);
//    }

//    int read(){
//      int c = -1;
//      if(rind != getRdmaInd()){
//        c = rbuf[rind++];
//        rind &= rbuflen - 1;
//      }
//      return c;
//    }

//    void flush(void){
//      static bool mutex = false;

//      if((huart->gState == HAL_UART_STATE_READY) && !mutex){
//        mutex = true;

//        if(twind != tfind){
//          uint16_t len = tfind < twind ? twind - tfind : tbuflen - tfind;
//          HAL_UART_Transmit_DMA(huart, &(tbuf[tfind]), len);
//          tfind = (tfind + len) & (tbuflen - 1);
//        }
//        mutex = false;
//      }
//    }

//    void write(uint8_t* data, int length){
//      int n = length;
//      n = n <= tbuflen ? n : tbuflen;

//      int n_tail = n <= tbuflen - twind ? n : tbuflen - twind;
//      memcpy(&(tbuf[twind]), data, n_tail);
//      twind = (twind + n) & (tbuflen - 1);

//      if(n != n_tail){
//        memcpy(tbuf, &(data[n_tail]), n - n_tail);
//      }

//      flush();
//    }

//    unsigned long time(){ return HAL_GetTick();; }

//  protected:
//};

//class STM32Hardware {
//  public:
//    STM32Hardware() {
//      // 在构造函数中可以不做任何事
//    }
//  
//    void init() {
//      // 初始化时也可以不做任何事，因为 CubeMX 已经完成了
//    }

//    // 读取一个字节 (阻塞式)
//    int read() {
//      uint8_t c;
//      // HAL_OK 意味着在超时时间内成功收到了1个字节
//      if (HAL_UART_Receive(&huart3, &c, 1, 1) == HAL_OK) {
//        return c;
//      } else {
//        return -1; // -1 表示没有数据或超时
//      }
//    }

//    // 写入字节数组 (阻塞式)
//    void write(uint8_t* data, int length) {
//      HAL_UART_Transmit(&huart3, data, length, 100);
//    }

//    // 返回系统时间 (毫秒)
//    unsigned long time() { 
//      return HAL_GetTick();
//    }
//};

class STM32Hardware {
  protected:
    UART_HandleTypeDef *huart;
    const static uint16_t rbuflen = 1024; // 接收环形缓冲区大小
    uint8_t rbuf[rbuflen];
    uint32_t rind;
    // 直接通过 DMA 寄存器计算当前缓冲区写入位置
    inline uint32_t getRdmaInd(void){ return (rbuflen - __HAL_DMA_GET_COUNTER(huart->hdmarx)) & (rbuflen - 1); }

  public:
    STM32Hardware(): huart(&huart3), rind(0) {}
  
    void init(){
      // 启动 DMA 循环接收
      HAL_UART_Receive_DMA(huart, rbuf, rbuflen);
      // 使能空闲中断
      __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }

    // 从环形缓冲区读取一个字节
    int read(){
      int c = -1;
      if(rind != getRdmaInd()){
        c = rbuf[rind++];
        rind &= rbuflen - 1;
      }
      return c;
    }

    // 写入数据 (阻塞式)
    void write(uint8_t* data, int length){
      HAL_UART_Transmit(huart, data, length, 100);
    }

    unsigned long time(){ return HAL_GetTick(); }
};



#endif
