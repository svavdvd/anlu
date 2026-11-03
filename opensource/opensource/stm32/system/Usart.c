#include "Usart.h"
#include <stdio.h>

#define RX_BUF_SIZE  64   /* 单行接收缓冲 */

static volatile uint8_t  s_rxBuf[RX_BUF_SIZE];
static volatile uint16_t s_rxLen = 0;      /* 当前行已收到字节数 */
static volatile uint8_t  s_lineReady = 0;  /* 是否已收到完整一行 */

/* ---------------------------------------------------------------------------
 * USART1 初始化: 115200/8N1, 开 RXNE 中断
 * -------------------------------------------------------------------------*/
void Usart1_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 = USART1_TX (复用推挽) */
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_9;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 = USART1_RX (浮空输入) */
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin   = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

/* ---------------------------------------------------------------------------
 * 发送单个字节(阻塞, 等待发送寄存器空)
 * -------------------------------------------------------------------------*/
void Usart1_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

void Usart1_SendString(const char *str)
{
    while (*str)
    {
        Usart1_SendByte((uint8_t)(*str++));
    }
}

/* ---------------------------------------------------------------------------
 * 取一行。调用方轮询此函数, 收到完整行时把内容拷到 buf 并返回 1
 * -------------------------------------------------------------------------*/
uint8_t Usart1_ReadLine(char *buf, uint16_t max)
{
    uint16_t i, n;

    if (!s_lineReady)
        return 0;

    if (max == 0)                    /* 防御: 调用方误传 0 时避免下溢/越界 */
        return 0;

    n = s_rxLen;
    if (n >= max)
        n = max - 1;

    for (i = 0; i < n; i++)
        buf[i] = (char)s_rxBuf[i];
    buf[n] = '\0';

    s_rxLen     = 0;
    s_lineReady = 0;
    return 1;
}

/* ---------------------------------------------------------------------------
 * USART1 接收中断: 逐字节攒成一行, 遇 \r/\n 置 s_lineReady
 * -------------------------------------------------------------------------*/
void USART1_IRQHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART1);

        if (s_lineReady)                 /* 上一行还没被取走, 丢弃新字节 */
            return;

        if (ch == '\r' || ch == '\n')    /* 行结束, 不存换行符 */
        {
            if (s_rxLen > 0)
                s_lineReady = 1;
            return;
        }

        if (s_rxLen < RX_BUF_SIZE - 1)
            s_rxBuf[s_rxLen++] = ch;
        else
            s_rxLen = 0;                 /* 溢出, 清空重来 */
    }
}

/* ---------------------------------------------------------------------------
 * printf 重定向到 USART1, 便于调试 (Keil/ARMCC 用 fputc, GCC 用 __io_putchar)
 * -------------------------------------------------------------------------*/
#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    Usart1_SendByte((uint8_t)ch);
    return ch;
}
