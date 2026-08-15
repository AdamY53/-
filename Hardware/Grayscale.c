#include "Grayscale.h"

uint8_t Stop_Way = 0, Middle_Way = 0, Right_Way = 0, Left_Way = 0, Start_Way = 0;
uint16_t Gray_State = 0x0000;
float Gray_Status[2] = {0};
float Gray_Status_Backup[2][20] = {0};
float Gray_Status_Worse = 0;
int8_t a;

volatile uint8_t Gray_Sensor[GRAY_SENSOR_COUNT] = {0};
volatile int16_t Gray_Error = 0;
volatile uint8_t Gray_ActiveCount = 0;
volatile uint8_t Gray_MidActiveCount = 0;
volatile uint8_t Gray_CrossFlag = 0;
volatile uint8_t Gray_LostFlag = 0;
volatile uint8_t Gray_TurnFlag = GRAY_TURN_NONE;
volatile uint8_t Gray_LastTurnFlag = GRAY_DEFAULT_TURN;
volatile float Gray_SteerGain = 5.0f;

static uint8_t Grayscale_ReadActive(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t Level = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
	return (Level == GRAY_ACTIVE_LEVEL) ? 1 : 0;
}

static float Grayscale_Limit(float Value, float Min, float Max)
{
	if (Value > Max) {return Max;}
	if (Value < Min) {return Min;}
	return Value;
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
	uint8_t i;
	uint8_t LeftCount;
	uint8_t RightCount;
	int16_t WeightSum = 0;
	float Correction;
	const int8_t Weight[GRAY_SENSOR_COUNT] = {-6, -4, -2, 0, 2, 4, 6};
	
	Gray_State = Grayscale_ReadState();
	Gray_ActiveCount = Gray_Sensor[GRAY_IDX_L3] + Gray_Sensor[GRAY_IDX_L2] +
	                   Gray_Sensor[GRAY_IDX_L1] + Gray_Sensor[GRAY_IDX_M]  +
	                   Gray_Sensor[GRAY_IDX_R1] + Gray_Sensor[GRAY_IDX_R2] +
	                   Gray_Sensor[GRAY_IDX_R3];
	Gray_MidActiveCount = Gray_Sensor[GRAY_IDX_L1] + Gray_Sensor[GRAY_IDX_M] + Gray_Sensor[GRAY_IDX_R1];
	Gray_CrossFlag = 0;
	Gray_LostFlag = 0;
	Gray_TurnFlag = GRAY_TURN_NONE;
	LeftCount = Gray_Sensor[GRAY_IDX_L3] + Gray_Sensor[GRAY_IDX_L2] + Gray_Sensor[GRAY_IDX_L1];
	RightCount = Gray_Sensor[GRAY_IDX_R1] + Gray_Sensor[GRAY_IDX_R2] + Gray_Sensor[GRAY_IDX_R3];
	
	for (i = 19; i > 0; i--)
	{
		Gray_Status_Backup[0][i] = Gray_Status_Backup[0][i - 1];
		Gray_Status_Backup[1][i] = Gray_Status_Backup[1][i - 1];
	}
	Gray_Status_Backup[0][0] = Gray_Status[0];
	Gray_Status_Backup[1][0] = Gray_Status[1];
	
	if (Gray_ActiveCount >= GRAY_CROSS_ACTIVE_THRESHOLD)
	{
		Gray_CrossFlag = 1;
		Middle_Way++;
		if (LeftCount > RightCount)
		{
			Gray_TurnFlag = GRAY_TURN_LEFT;
			Gray_LastTurnFlag = GRAY_TURN_LEFT;
			Left_Way++;
		}
		else if (RightCount > LeftCount)
		{
			Gray_TurnFlag = GRAY_TURN_RIGHT;
			Gray_LastTurnFlag = GRAY_TURN_RIGHT;
			Right_Way++;
		}
		else
		{
			Gray_TurnFlag = Gray_LastTurnFlag;
		}
		Gray_Error = 0;
		Gray_Status[0] = 0;
		Gray_Status[1] = 0;
		return;
	}
	
	if (Gray_ActiveCount == 0)
	{
		Gray_LostFlag = 1;
		Start_Way++;
		Gray_Status[0] = Gray_Status_Backup[0][0];
		Gray_Status[1] = Gray_Status_Backup[1][0];
		Gray_Status_Worse++;
		return;
	}
	
	for (i = 0; i < GRAY_SENSOR_COUNT; i++)
	{
		if (Gray_Sensor[i]) {WeightSum += Weight[i];}
	}
	
	Gray_Error = WeightSum / Gray_ActiveCount;
	Correction = Grayscale_Limit((float)Gray_Error * Gray_SteerGain, -GRAY_STEER_MAX, GRAY_STEER_MAX);
	Gray_Status[0] = Correction;
	Gray_Status[1] = Correction;
	Gray_Status_Worse /= 3.0f;
	Start_Way = 0;
	a = 1;
}
