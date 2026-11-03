#include "stm32f10x.h"

void TIM2_Servo4CH_Init(void)
{
    //
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    // PA0 CH1
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // PA1 CH2
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // PA2 CH3
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // PA3 CH4
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStruct.TIM_Period = 20000 - 1;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 1500;

    TIM_OC1Init(TIM2, &TIM_OCInitStruct);
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);
    TIM_OC3Init(TIM2, &TIM_OCInitStruct);
    TIM_OC4Init(TIM2, &TIM_OCInitStruct);

    TIM_Cmd(TIM2, ENABLE);
}



// TIM3 CH1 PA6,
void TIM3_Servo1CH_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_TimeBaseStruct.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStruct.TIM_Period = 20000 - 1;
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV2;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 1500;	//
	TIM_OC1Init(TIM3, &TIM_OCInitStruct);

	TIM_Cmd(TIM3, ENABLE);
}


void TIM3_SetCH1Pulse(uint16_t pulse)
{
	if(pulse < 500) pulse = 500;
	if(pulse > 2500) pulse = 2500;
	TIM_SetCompare1(TIM3, pulse);
}


void TIM2_SetCH1Pulse(uint16_t pulse)
{
    if(pulse < 500) pulse = 500;
    if(pulse > 2500) pulse = 2500;
    TIM_SetCompare1(TIM2, pulse);
}
void TIM2_SetCH2Pulse(uint16_t pulse)
{
    if(pulse < 500) pulse = 500;
    if(pulse > 2500) pulse = 2500;
    TIM_SetCompare2(TIM2, pulse);
}
void TIM2_SetCH3Pulse(uint16_t pulse)
{
    if(pulse < 500) pulse = 500;
    if(pulse > 2500) pulse = 2500;
    TIM_SetCompare3(TIM2, pulse);
}
void TIM2_SetCH4Pulse(uint16_t pulse)
{
    if(pulse < 500) pulse = 500;
    if(pulse > 2500) pulse = 2500;
    TIM_SetCompare4(TIM2, pulse);
}
