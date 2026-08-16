#include "PWM.h"

void PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(PWM_TIMER_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(PWM_GPIO_CLK, ENABLE);
	
	/* 关闭 JTAG 保留 SWD，释放 PA15/PB3；TIM2 部分重映射到 PA15/PB3。 */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(PWM_TIM2_REMAP, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = PWM_RIGHT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_RIGHT_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM_LEFT_PIN;
	GPIO_Init(PWM_LEFT_PORT, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(PWM_TIMER);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = PWM_PRESCALER - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(PWM_TIMER, &TIM_TimeBaseInitStructure);
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(PWM_TIMER, &TIM_OCInitStructure);
	TIM_OC2Init(PWM_TIMER, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(PWM_TIMER, ENABLE);
	TIM_Cmd(PWM_TIMER, ENABLE);
}

void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(PWM_TIMER, Compare);
}

void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(PWM_TIMER, Compare);
}
