#include "Motor.h"
#include "PWM.h"

static uint8_t Motor_LimitPWM(int8_t PWM)
{
	if (PWM >= 0)
	{
		return (PWM > 100) ? 100 : PWM;
	}
	return (PWM < -100) ? 100 : (uint8_t)(-PWM);
}

void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(MOTOR_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = MOTOR_STBY_PIN | MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
	                              MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_GPIO_PORT, &GPIO_InitStructure);
	
	// �ϵ��ȹض� TB6612 �ͷ���ţ���ֹ���������
	GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_STBY_PIN | MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
	                                MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN);
	PWM_Init();
	GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_STBY_PIN);
	Motor_Stop();
}

void Motor_SetPWM(uint8_t m, int8_t PWM)
{
	uint8_t Motor = m;
	uint8_t Duty = Motor_LimitPWM(PWM);
	
#if MOTOR_SWAP_LEFT_RIGHT
	if (Motor == MOTOR_RIGHT) {Motor = MOTOR_LEFT;}
	else if (Motor == MOTOR_LEFT) {Motor = MOTOR_RIGHT;}
#endif
	
	if (Motor == MOTOR_RIGHT)
	{
		// ���֣�BIN1/PB13��BIN2/PB14��PWMB/PA15/TIM2_CH1��
		// ʵ���� PWM ʱ���ֺ��ˣ�������������ת��ƽ�Ե���ʹ�� PWM = ǰ����
		if (PWM >= 0)
		{
			GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_RIGHT_IN1_PIN);
			GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_RIGHT_IN2_PIN);
		}
		else
		{
			GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_RIGHT_IN1_PIN);
			GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_RIGHT_IN2_PIN);
		}
		PWM_SetCompare1(Duty);
	}
	else if (Motor == MOTOR_LEFT)
	{
		// ���֣�AIN1/PB1��AIN2/PB15��PWMA/PB3/TIM2_CH2��
		// ������һ�£��� PWM ����ΪС��ǰ������
		if (PWM >= 0)
		{
			GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_LEFT_IN1_PIN);
			GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_LEFT_IN2_PIN);
		}
		else
		{
			GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_LEFT_IN1_PIN);
			GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_LEFT_IN2_PIN);
		}
		PWM_SetCompare2(Duty);
	}
}

void Motor_Stop(void)
{
	// IN1/IN2 ȫ�����ͣ�PWM ���㣬������뻬��ֹͣ��?
	GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
	                                MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN);
	PWM_SetCompare1(0);
	PWM_SetCompare2(0);
}

// �����ɽӿڣ���ǰ����ѭ������δʹ�á�
//void Motor_FL(void)
//{
//}

