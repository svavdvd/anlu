#ifndef __TIMPWM_H
#define __TIMPWM_H

	void TIM2_Servo4CH_Init(void);
	void TIM3_Servo1CH_Init(void);
	void TIM3_SetCH1Pulse(uint16_t pulse);
	void TIM2_SetCH1Pulse(uint16_t pulse);
	void TIM2_SetCH2Pulse(uint16_t pulse);
	void TIM2_SetCH3Pulse(uint16_t pulse);
	void TIM2_SetCH4Pulse(uint16_t pulse);

#endif
