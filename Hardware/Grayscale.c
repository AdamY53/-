#include "Grayscale.h"

volatile uint16_t Gray_State = 0x0000;
volatile uint8_t Gray_Sensor[GRAY_SENSOR_COUNT] = {0};
volatile uint8_t Gray_LineSensor[GRAY_SENSOR_COUNT] = {0};
volatile uint8_t Gray_ActiveCount = 0;

static uint8_t Gray_MiddleHistory[3][GRAY_MIDDLE_FILTER_WINDOW] = {{0}};
static uint8_t Gray_MiddleHistoryIndex = 0;
static uint8_t Gray_MiddleFilterReady = 0;

static uint8_t Grayscale_ReadActive(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t Level = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
	return (Level == GRAY_ACTIVE_LEVEL) ? 1 : 0;
}

static uint8_t Grayscale_MiddleMajority(uint8_t MiddleChannel)
{
	uint8_t i;
	uint8_t ActiveCount;

	ActiveCount = 0;
	for (i = 0; i < GRAY_MIDDLE_FILTER_WINDOW; i++)
	{
		ActiveCount += Gray_MiddleHistory[MiddleChannel][i];
	}

	return (ActiveCount >= (GRAY_MIDDLE_FILTER_WINDOW / 2u + 1u)) ? 1u : 0u;
}

void Grayscale_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t i;
	uint8_t j;

	RCC_APB2PeriphClockCmd(GRAY_GPIO_CLK, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GRAY_GPIOA_PINS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GRAY_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GRAY_GPIOB_PINS;
	GPIO_Init(GRAY_GPIOB_PORT, &GPIO_InitStructure);

	Gray_State = 0;
	Gray_ActiveCount = 0;
	Gray_MiddleHistoryIndex = 0;
	Gray_MiddleFilterReady = 0;
	for (i = 0; i < GRAY_SENSOR_COUNT; i++)
	{
		Gray_Sensor[i] = 0;
		Gray_LineSensor[i] = 0;
	}
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < GRAY_MIDDLE_FILTER_WINDOW; j++)
		{
			Gray_MiddleHistory[i][j] = 0;
		}
	}
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
	uint8_t i;

	Gray_State = Grayscale_ReadState();
	Gray_ActiveCount = Gray_Sensor[GRAY_IDX_L3]
	                 + Gray_Sensor[GRAY_IDX_L2]
	                 + Gray_Sensor[GRAY_IDX_L1]
	                 + Gray_Sensor[GRAY_IDX_M]
	                 + Gray_Sensor[GRAY_IDX_R1]
	                 + Gray_Sensor[GRAY_IDX_R2]
	                 + Gray_Sensor[GRAY_IDX_R3];

	/*
	 * 原始 Gray_Sensor 不做覆盖：
	 * - T 路口识别、OLED 调试仍读取原始信号；
	 * - Gray_LineSensor 只供普通循迹误差计算使用。
	 *
	 * 首次采样时填满窗口，避免上电后的前几次采样被初始 0 拉低。
	 */
	if (!Gray_MiddleFilterReady)
	{
		for (i = 0; i < GRAY_MIDDLE_FILTER_WINDOW; i++)
		{
			Gray_MiddleHistory[0][i] = Gray_Sensor[GRAY_IDX_L1];
			Gray_MiddleHistory[1][i] = Gray_Sensor[GRAY_IDX_M];
			Gray_MiddleHistory[2][i] = Gray_Sensor[GRAY_IDX_R1];
		}
		Gray_MiddleFilterReady = 1;
	}
	else
	{
		Gray_MiddleHistory[0][Gray_MiddleHistoryIndex] = Gray_Sensor[GRAY_IDX_L1];
		Gray_MiddleHistory[1][Gray_MiddleHistoryIndex] = Gray_Sensor[GRAY_IDX_M];
		Gray_MiddleHistory[2][Gray_MiddleHistoryIndex] = Gray_Sensor[GRAY_IDX_R1];
		Gray_MiddleHistoryIndex++;
		if (Gray_MiddleHistoryIndex >= GRAY_MIDDLE_FILTER_WINDOW)
		{
			Gray_MiddleHistoryIndex = 0;
		}
	}

	for (i = 0; i < GRAY_SENSOR_COUNT; i++)
	{
		Gray_LineSensor[i] = Gray_Sensor[i];
	}
	Gray_LineSensor[GRAY_IDX_L1] = Grayscale_MiddleMajority(0);
	Gray_LineSensor[GRAY_IDX_M] = Grayscale_MiddleMajority(1);
	Gray_LineSensor[GRAY_IDX_R1] = Grayscale_MiddleMajority(2);
}
