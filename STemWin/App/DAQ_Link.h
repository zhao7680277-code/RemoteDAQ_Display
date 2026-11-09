/*
******************************************************************************
  File        : DAQ_Link.h
  Purpose     : 与采集板通信的 USART6 链路（数据帧解析）
                Protocol: 上行数据帧格式说明 v1.0  (1770 B, 921600 8N1)
******************************************************************************
*/

#ifndef __DAQ_LINK_H
#define __DAQ_LINK_H

#include "main.h"

#define DAQ_POINTS          400          /* 每通道采样点数：8 格 x 50       */
#define DAQ_DIG_BYTES       50           /* 400 位打包存放  */
#define DAQ_PAYLOAD_SIZE    1758
#define DAQ_FRAME_SIZE      1770

typedef struct
{
  uint8_t  valid;            /* 1 = 已收到至少一帧正常数据                 */
  uint8_t  mode;             /* 0 = 正常，1 = 内部方波                     */
  uint8_t  chMask;           /* bit0..4 对应 CH1、CH2、D0、D1、D2          */
  uint16_t timebaseUs;       /* 每格时间，单位 us                          */
  uint16_t pwmFreq;          /* PWM 频率，单位 Hz                          */
  uint8_t  pwmDuty;          /* 占空比，单位 %                             */
  uint32_t seq;              /* 帧序号                                     */
  uint16_t ch1[DAQ_POINTS];  /* ADC 原始值 0..4095                         */
  uint16_t ch2[DAQ_POINTS];
  uint8_t  dig[3][DAQ_DIG_BYTES];
  uint32_t frames;           /* 正常帧数                                   */
  uint32_t errors;           /* 帧头 / CRC / 长度错误 + 接收溢出           */
  uint32_t rxBytes;          /* 收到的原始字节数，仅作诊断                 */
  uint32_t uartErrors;       /* 接收噪声 / 帧错误 / 溢出错误               */
  uint32_t txBytes;          /* 发往采集板的字节数                         */
  uint32_t hostRxBytes;      /* 从 PC 收到的字节数（下载链路）             */
  char     lastCtrl[20];     /* 最近一次发送的控制帧，ASCII，供界面显示    */
  uint8_t  physLoop;         /* 1 = PC6 与 PC7 实际短接（D1-D0 直连）        */
  uint32_t lost;             /* 丢失的帧数，由 SEQ 推算                    */
  uint32_t lastTick;         /* 最近一帧正常数据的 HAL 节拍                */
  uint8_t  newFrame;         /* 由解析器置位，由应用清除                   */
} DAQ_FRAME;

extern DAQ_FRAME gDaq;
extern UART_HandleTypeDef huart6;

void    DAQ_Link_Init(void);
void    DAQ_Link_Poll(void);
void    DAQ_Link_SendControl(uint8_t mode, uint16_t freq, uint8_t duty);

/* 主机侧链路：ST-LINK 虚拟串口，USART1 TX = PA9 / RX = PB7，
   用于下载存放在板载存储器里的文件                                          */
extern UART_HandleTypeDef huart1;

void    Host_Init(void);
void    Host_Put(const uint8_t *buf, uint16_t len);
int     Host_GetChar(void);          /* 接收缓冲为空时返回 -1          */
uint8_t DAQ_DigGet(const uint8_t *dig, uint8_t ch, uint16_t i);

#endif /* __DAQ_LINK_H */
