#include "Encoder.h"

volatile int16_t Encoder1_RawCount = 0;
volatile int16_t Encoder2_RawCount = 0;
volatile uint8_t Encoder1_PinA = 0;
volatile uint8_t Encoder1_PinB = 0;
volatile uint8_t Encoder2_PinA = 0;
volatile uint8_t Encoder2_PinB = 0;

static void Encoder_TIMInit(TIM_TypeDef *TIMx)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	TIM_SetCounter(TIMx, 0);
	TIM_Cmd(TIMx, ENABLE);
}

void Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(ENCODER1_TIMER_CLK | ENCODER2_TIMER_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(ENCODER1_GPIO_CLK | ENCODER2_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = ENCODER1_A_PIN | ENCODER1_B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER1_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ENCODER2_A_PIN | ENCODER2_B_PIN;
	GPIO_Init(ENCODER2_GPIO_PORT, &GPIO_InitStructure);
	
	Encoder_TIMInit(ENCODER1_TIMER);
	Encoder_TIMInit(ENCODER2_TIMER);
}

int16_t Encoder1_Get(void)
{
	int16_t Temp;
	Encoder1_PinA = GPIO_ReadInputDataBit(ENCODER1_GPIO_PORT, ENCODER1_A_PIN);
	Encoder1_PinB = GPIO_ReadInputDataBit(ENCODER1_GPIO_PORT, ENCODER1_B_PIN);
	Temp = (int16_t)TIM_GetCounter(ENCODER1_TIMER);
	Encoder1_RawCount = Temp;
	TIM_SetCounter(ENCODER1_TIMER, 0);
	return (int16_t)(ENCODER1_DIR * Temp);
}

int16_t Encoder2_Get(void)
{
	int16_t Temp;
	Encoder2_PinA = GPIO_ReadInputDataBit(ENCODER2_GPIO_PORT, ENCODER2_A_PIN);
	Encoder2_PinB = GPIO_ReadInputDataBit(ENCODER2_GPIO_PORT, ENCODER2_B_PIN);
	Temp = (int16_t)TIM_GetCounter(ENCODER2_TIMER);
	Encoder2_RawCount = Temp;
	TIM_SetCounter(ENCODER2_TIMER, 0);
	return (int16_t)(ENCODER2_DIR * Temp);
}
