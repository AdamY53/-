#include "Ultrasonic.h"
#include "Delay.h"

typedef struct
{
	GPIO_TypeDef *TrigPort;
	uint16_t TrigPin;
	GPIO_TypeDef *EchoPort;
	uint16_t EchoPin;
} UltrasonicPin_t;

static const UltrasonicPin_t UltrasonicPins[3] =
{
	{US1_TRIG_PORT, US1_TRIG_PIN, US1_ECHO_PORT, US1_ECHO_PIN},
	{US2_TRIG_PORT, US2_TRIG_PIN, US2_ECHO_PORT, US2_ECHO_PIN},
	{US3_TRIG_PORT, US3_TRIG_PIN, US3_ECHO_PORT, US3_ECHO_PIN}
};

static void Ultrasonic_GPIOInit(GPIO_TypeDef *GPIOx, uint16_t Pin, GPIOMode_TypeDef Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = Mode;
	GPIO_InitStructure.GPIO_Pin = Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void Ultrasonic_Init(void)
{
	RCC_APB2PeriphClockCmd(US1_TRIG_GPIO_CLK | US1_ECHO_GPIO_CLK |
	                       US2_TRIG_GPIO_CLK | US2_ECHO_GPIO_CLK |
	                       US3_TRIG_GPIO_CLK | US3_ECHO_GPIO_CLK, ENABLE);
	
	Ultrasonic_GPIOInit(US1_TRIG_PORT, US1_TRIG_PIN, GPIO_Mode_Out_PP);
	Ultrasonic_GPIOInit(US2_TRIG_PORT, US2_TRIG_PIN, GPIO_Mode_Out_PP);
	Ultrasonic_GPIOInit(US3_TRIG_PORT, US3_TRIG_PIN, GPIO_Mode_Out_PP);
	
	Ultrasonic_GPIOInit(US1_ECHO_PORT, US1_ECHO_PIN, GPIO_Mode_IPD);
	Ultrasonic_GPIOInit(US2_ECHO_PORT, US2_ECHO_PIN, GPIO_Mode_IPD);
	Ultrasonic_GPIOInit(US3_ECHO_PORT, US3_ECHO_PIN, GPIO_Mode_IPD);
	
	GPIO_ResetBits(US1_TRIG_PORT, US1_TRIG_PIN);
	GPIO_ResetBits(US2_TRIG_PORT, US2_TRIG_PIN);
	GPIO_ResetBits(US3_TRIG_PORT, US3_TRIG_PIN);
}

uint16_t Ultrasonic_GetDistanceCm(UltrasonicChannel_t Channel)
{
	const UltrasonicPin_t *Pin;
	uint32_t WaitTime = 0;
	uint32_t HighTime = 0;
	
	if (Channel > US_CH_RIGHT)
	{
		return 0;
	}
	Pin = &UltrasonicPins[Channel];
	
	GPIO_ResetBits(Pin->TrigPort, Pin->TrigPin);
	Delay_us(2);
	GPIO_SetBits(Pin->TrigPort, Pin->TrigPin);
	Delay_us(12);
	GPIO_ResetBits(Pin->TrigPort, Pin->TrigPin);
	
	while (GPIO_ReadInputDataBit(Pin->EchoPort, Pin->EchoPin) == RESET)
	{
		Delay_us(1);
		if (++WaitTime >= US_TIMEOUT_US)
		{
			return 0;
		}
	}
	
	while (GPIO_ReadInputDataBit(Pin->EchoPort, Pin->EchoPin) == SET)
	{
		Delay_us(1);
		if (++HighTime >= US_TIMEOUT_US)
		{
			break;
		}
	}
	
	return (uint16_t)(HighTime / 58UL);
}
