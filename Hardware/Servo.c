#include "Servo.h"

void Servo_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(SERVO_TIMER_CLK | SERVO_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = SERVO_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(SERVO_TIMER);
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = SERVO_PERIOD_US - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(SERVO_TIMER, &TIM_TimeBaseInitStructure);
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = SERVO_MID_PULSE_US;
	TIM_OC1Init(SERVO_TIMER, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(SERVO_TIMER, ENABLE);
	TIM_CtrlPWMOutputs(SERVO_TIMER, ENABLE);
	TIM_Cmd(SERVO_TIMER, ENABLE);
}

void Servo_SetAngle(uint8_t Angle)
{
	uint16_t Pulse;
	
	if (Angle > 180)
	{
		Angle = 180;
	}
	Pulse = SERVO_MIN_PULSE_US +
	        (uint16_t)(((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * Angle) / 180UL);
	TIM_SetCompare1(SERVO_TIMER, Pulse);
}
