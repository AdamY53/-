#include "Ultrasonic.h"
#include "Delay.h"
#include "system_stm32f10x.h"

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

static UltrasonicStatus_t UltrasonicStatus[3] =
{
	US_STATUS_WAIT_RISE_TIMEOUT,
	US_STATUS_WAIT_RISE_TIMEOUT,
	US_STATUS_WAIT_RISE_TIMEOUT
};

static uint32_t Ultrasonic_CyclesPerUs = 72;

static void Ultrasonic_GPIOInit(GPIO_TypeDef *GPIOx, uint16_t Pin, GPIOMode_TypeDef Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = Mode;
	GPIO_InitStructure.GPIO_Pin = Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

static void Ultrasonic_TimerStart(void)
{
	SysTick->CTRL = 0;
	SysTick->LOAD = 0x00FFFFFFUL;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x00000005;
}

static void Ultrasonic_TimerStop(void)
{
	SysTick->CTRL = 0x00000004;
}

static uint32_t Ultrasonic_ElapsedCycles(uint32_t StartValue)
{
	uint32_t CurrentValue = SysTick->VAL;

	if (StartValue >= CurrentValue)
	{
		return StartValue - CurrentValue;
	}
	return StartValue + (0x01000000UL - CurrentValue);
}

static uint8_t Ultrasonic_Timeout(uint32_t StartValue)
{
	return Ultrasonic_ElapsedCycles(StartValue)
	       >= (US_TIMEOUT_US * Ultrasonic_CyclesPerUs);
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

	SystemCoreClockUpdate();
	Ultrasonic_CyclesPerUs = SystemCoreClock / 1000000UL;
	if (Ultrasonic_CyclesPerUs == 0)
	{
		Ultrasonic_CyclesPerUs = 72;
	}
}

uint16_t Ultrasonic_GetDistanceCm(UltrasonicChannel_t Channel)
{
	const UltrasonicPin_t *Pin;
	uint32_t WaitStart;
	uint32_t PulseStart;
	uint32_t PulseCycles;
	uint32_t PulseUs;

	if (Channel > US_CH_RIGHT)
	{
		return US_INVALID_DISTANCE_CM;
	}
	Pin = &UltrasonicPins[Channel];

	Ultrasonic_TimerStart();
	WaitStart = SysTick->VAL;
	while (GPIO_ReadInputDataBit(Pin->EchoPort, Pin->EchoPin) == SET)
	{
		if (Ultrasonic_Timeout(WaitStart))
		{
			Ultrasonic_TimerStop();
			UltrasonicStatus[Channel] = US_STATUS_ECHO_STUCK_HIGH;
			return US_INVALID_DISTANCE_CM;
		}
	}
	Ultrasonic_TimerStop();

	GPIO_ResetBits(Pin->TrigPort, Pin->TrigPin);
	Delay_us(3);
	GPIO_SetBits(Pin->TrigPort, Pin->TrigPin);
	Delay_us(12);
	GPIO_ResetBits(Pin->TrigPort, Pin->TrigPin);

	Ultrasonic_TimerStart();
	WaitStart = SysTick->VAL;
	while (GPIO_ReadInputDataBit(Pin->EchoPort, Pin->EchoPin) == RESET)
	{
		if (Ultrasonic_Timeout(WaitStart))
		{
			Ultrasonic_TimerStop();
			UltrasonicStatus[Channel] = US_STATUS_WAIT_RISE_TIMEOUT;
			return US_INVALID_DISTANCE_CM;
		}
	}

	PulseStart = SysTick->VAL;
	while (GPIO_ReadInputDataBit(Pin->EchoPort, Pin->EchoPin) == SET)
	{
		if (Ultrasonic_Timeout(PulseStart))
		{
			Ultrasonic_TimerStop();
			UltrasonicStatus[Channel] = US_STATUS_HIGH_TIMEOUT;
			return US_INVALID_DISTANCE_CM;
		}
	}

	PulseCycles = Ultrasonic_ElapsedCycles(PulseStart);
	Ultrasonic_TimerStop();
	PulseUs = (PulseCycles + Ultrasonic_CyclesPerUs / 2UL)
	        / Ultrasonic_CyclesPerUs;

	if (PulseUs < 100UL)
	{
		UltrasonicStatus[Channel] = US_STATUS_WAIT_RISE_TIMEOUT;
		return US_INVALID_DISTANCE_CM;
	}

	UltrasonicStatus[Channel] = US_STATUS_OK;
	return (uint16_t)((PulseUs + 29UL) / 58UL);
}

UltrasonicStatus_t Ultrasonic_GetStatus(UltrasonicChannel_t Channel)
{
	if (Channel > US_CH_RIGHT)
	{
		return US_STATUS_INVALID_CHANNEL;
	}
	return UltrasonicStatus[Channel];
}
