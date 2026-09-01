#include "Grayscale.h"

volatile uint16_t Gray_State = 0x0000;
volatile uint8_t Gray_Sensor[GRAY_SENSOR_COUNT] = {0};
volatile uint8_t Gray_ActiveCount = 0;

static uint8_t Grayscale_ReadActive(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t Level = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
	return (Level == GRAY_ACTIVE_LEVEL) ? 1 : 0;
}

void Grayscale_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(GRAY_GPIO_CLK, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GRAY_GPIOA_PINS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GRAY_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GRAY_GPIOB_PINS;
	GPIO_Init(GRAY_GPIOB_PORT, &GPIO_InitStructure);
}

uint16_t Grayscale_ReadState(void)
{
	uint16_t State = 0x0000;

	Gray_Sensor[GRAY_IDX_L3] = Grayscale_ReadActive(GRAY_GPIO_PORT, GRAY_L3_PIN);
	Gray_Sensor[GRAY_IDX_L2] = Grayscale_ReadActive(GRAY_GPIOB_PORT, GRAY_L2_PIN);
	Gray_Sensor[GRAY_IDX_L1] = Grayscale_ReadActive(GRAY_GPIO_PORT, GRAY_L1_PIN);
	Gray_Sensor[GRAY_IDX_M]  = Grayscale_ReadActive(GRAY_GPIO_PORT, GRAY_M_PIN);
	Gray_Sensor[GRAY_IDX_R1] = Grayscale_ReadActive(GRAY_GPIO_PORT, GRAY_R1_PIN);
	Gray_Sensor[GRAY_IDX_R2] = Grayscale_ReadActive(GRAY_GPIOB_PORT, GRAY_R2_PIN);
	Gray_Sensor[GRAY_IDX_R3] = Grayscale_ReadActive(GRAY_GPIO_PORT, GRAY_R3_PIN);

	State |= (Gray_Sensor[GRAY_IDX_L3] << 6);
	State |= (Gray_Sensor[GRAY_IDX_L2] << 5);
	State |= (Gray_Sensor[GRAY_IDX_L1] << 4);
	State |= (Gray_Sensor[GRAY_IDX_M]  << 3);
	State |= (Gray_Sensor[GRAY_IDX_R1] << 2);
	State |= (Gray_Sensor[GRAY_IDX_R2] << 1);
	State |= (Gray_Sensor[GRAY_IDX_R3] << 0);

	return State;
}

void Grayscale_Tick(void)
{
	Gray_State = Grayscale_ReadState();
	Gray_ActiveCount = Gray_Sensor[GRAY_IDX_L3]
	                 + Gray_Sensor[GRAY_IDX_L2]
	                 + Gray_Sensor[GRAY_IDX_L1]
	                 + Gray_Sensor[GRAY_IDX_M]
	                 + Gray_Sensor[GRAY_IDX_R1]
	                 + Gray_Sensor[GRAY_IDX_R2]
	                 + Gray_Sensor[GRAY_IDX_R3];
}
