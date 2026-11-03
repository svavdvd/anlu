#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"

/* USART1 初始化: PA9=TX(复用推挽), PA10=RX(浮空输入), 开接收中断 */
void Usart1_Init(uint32_t baud);

/* 发送 */
void Usart1_SendByte(uint8_t byte);
void Usart1_SendString(const char *str);

/* 取一行(以 \r 或 \n 结尾的完整行, 已去掉换行)。有完整行返回 1, 否则 0 */
uint8_t Usart1_ReadLine(char *buf, uint16_t max);

#endif
