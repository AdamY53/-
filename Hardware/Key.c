#include "Key.h"

uint8_t Key_Num;
uint8_t Key_LongNum;

void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(KEY_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN | KEY2_PIN | KEY3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);
}

uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if (Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		return Temp;
	}
	return 0;
}

uint8_t Key_GetLongNum(void)
{
	uint8_t Temp;
	if (Key_LongNum)
	{
		Temp = Key_LongNum;
		Key_LongNum = 0;
		return Temp;
	}
	return 0;
}

uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY1_PIN) == 0)
	{
		return KEY_NUM_K1;
	}
	if (GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY2_PIN) == 0)
	{
		return KEY_NUM_K2;
	}
	if (GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY3_PIN) == 0)
	{
		return KEY_NUM_K3;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t CurrState, PrevState;
	static uint8_t PressState;
	static uint8_t LongSent;
	static uint16_t PressTicks;
	
	PrevState = CurrState;
	CurrState = Key_GetState();
	
	if (CurrState != 0)
	{
		if (CurrState != PressState)
		{
			PressState = CurrState;
			PressTicks = 0;
			LongSent = 0;
		}
		else if (PressTicks < 1000)
		{
			PressTicks++;
		}
		
		if (PressTicks >= 50 && LongSent == 0)
		{
			Key_LongNum = KEY_LONG_FLAG | CurrState;
			LongSent = 1;
		}
	}
	else
	{
		if (PrevState != 0 && LongSent == 0)
		{
			Key_Num = PrevState;
		}
		PressState = 0;
		PressTicks = 0;
		LongSent = 0;
	}
}
