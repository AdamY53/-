#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Grayscale.h"
#include "Ultrasonic.h"
#include "Servo.h"

/* Straight-line tracking tuning. Both motors are always commanded forward. */
#define CAR_BASE_PWM               36.0f
#define CAR_LINE_KP                0.090f
#define CAR_LINE_KD                0.180f
#define CAR_ENCODER_BALANCE_KP     0.350f
#define CAR_STEER_LIMIT            26.0f
#define CAR_BALANCE_LIMIT          8.0f
#define CAR_MIN_FORWARD_PWM        10.0f
#define CAR_PWM_LIMIT              70.0f

#define SERVO_FIXED_ANGLE          0
#define ULTRASONIC_SAMPLE_TICKS    3

#define DISPLAY_PAGE_LINE          0
#define DISPLAY_PAGE_ULTRASONIC    1

static uint8_t Car_Running = 0;
static uint8_t Display_Page = DISPLAY_PAGE_ULTRASONIC;

static uint16_t Front_Distance = US_INVALID_DISTANCE_CM;
static uint16_t Left_Distance = US_INVALID_DISTANCE_CM;
static uint16_t Right_Distance = US_INVALID_DISTANCE_CM;
static uint8_t Ultrasonic_Index = 0;

static int16_t Encoder_Right = 0;
static int16_t Encoder_Left = 0;

static int16_t Line_Error = 0;
static int16_t Line_Last_Error = 0;
static int16_t Line_Derivative = 0;
static int8_t Line_Steer_PWM = 0;
static int8_t Encoder_Balance_PWM = 0;
static uint8_t PWM_Right = 0;
static uint8_t PWM_Left = 0;

static float LimitFloat(float Value, float Min, float Max)
{
	if (Value > Max) {return Max;}
	if (Value < Min) {return Min;}
	return Value;
}

static uint8_t LimitForwardPWM(float Value)
{
	if (Value > CAR_PWM_LIMIT) {return (uint8_t)CAR_PWM_LIMIT;}
	if (Value < CAR_MIN_FORWARD_PWM) {return (uint8_t)CAR_MIN_FORWARD_PWM;}
	return (uint8_t)Value;
}

static void Car_SetForwardPWM(float LeftPWM, float RightPWM)
{
	PWM_Left = LimitForwardPWM(LeftPWM);
	PWM_Right = LimitForwardPWM(RightPWM);
	Motor_SetPWM(MOTOR_LEFT, (int8_t)PWM_Left);
	Motor_SetPWM(MOTOR_RIGHT, (int8_t)PWM_Right);
}

static void Car_Stop(void)
{
	PWM_Left = 0;
	PWM_Right = 0;
	Motor_Stop();
}

static void Car_ResetLineController(void)
{
	Line_Error = 0;
	Line_Last_Error = 0;
	Line_Derivative = 0;
	Line_Steer_PWM = 0;
	Encoder_Balance_PWM = 0;
}

static void Car_UpdateEncoders(void)
{
	Encoder_Right = Encoder1_Get();
	Encoder_Left = Encoder2_Get();
}

static void Car_UpdateUltrasonicOneStep(void)
{
	uint16_t Distance;

	if (Ultrasonic_Index == 0)
	{
		Distance = Ultrasonic_GetDistanceCm(US_CH_FRONT);
		Front_Distance = Distance;
	}
	else if (Ultrasonic_Index == 1)
	{
		Distance = Ultrasonic_GetDistanceCm(US_CH_LEFT);
		Left_Distance = Distance;
	}
	else
	{
		Distance = Ultrasonic_GetDistanceCm(US_CH_RIGHT);
		Right_Distance = Distance;
	}

	Ultrasonic_Index++;
	if (Ultrasonic_Index >= 3)
	{
		Ultrasonic_Index = 0;
	}
}

static int16_t Car_CalcLineError(void)
{
	int16_t PositionSum;

	if (Gray_ActiveCount == 0)
	{
		return 0;
	}

	PositionSum = Gray_Sensor[GRAY_IDX_L3] * (-300)
	            + Gray_Sensor[GRAY_IDX_L2] * (-200)
	            + Gray_Sensor[GRAY_IDX_L1] * (-100)
	            + Gray_Sensor[GRAY_IDX_M]  * 0
	            + Gray_Sensor[GRAY_IDX_R1] * 100
	            + Gray_Sensor[GRAY_IDX_R2] * 200
	            + Gray_Sensor[GRAY_IDX_R3] * 300;

	return PositionSum / Gray_ActiveCount;
}

static void Car_LineFollowStraight(void)
{
	float Steer;
	float Balance;
	float LeftPWM;
	float RightPWM;
	int16_t SpeedDiff;

	Grayscale_Tick();
	if (Gray_ActiveCount == 0)
	{
		Car_ResetLineController();
		Car_Stop();
		return;
	}

	Line_Error = Car_CalcLineError();
	Line_Derivative = Line_Error - Line_Last_Error;
	Line_Last_Error = Line_Error;

	Steer = (float)Line_Error * CAR_LINE_KP
	      + (float)Line_Derivative * CAR_LINE_KD;
	Steer = LimitFloat(Steer, -CAR_STEER_LIMIT, CAR_STEER_LIMIT);

	SpeedDiff = Encoder_Left - Encoder_Right;
	Balance = (float)SpeedDiff * CAR_ENCODER_BALANCE_KP;
	Balance = LimitFloat(Balance, -CAR_BALANCE_LIMIT, CAR_BALANCE_LIMIT);

	Line_Steer_PWM = (int8_t)Steer;
	Encoder_Balance_PWM = (int8_t)Balance;
	LeftPWM = CAR_BASE_PWM + Steer - Balance;
	RightPWM = CAR_BASE_PWM - Steer + Balance;
	Car_SetForwardPWM(LeftPWM, RightPWM);
}

static void OLED_ShowLineStateRToL(uint8_t X, uint8_t Y, uint8_t FontSize)
{
	const uint8_t Order[GRAY_SENSOR_COUNT] = {
		GRAY_IDX_R3, GRAY_IDX_R2, GRAY_IDX_R1, GRAY_IDX_M,
		GRAY_IDX_L1, GRAY_IDX_L2, GRAY_IDX_L3
	};
	uint8_t i;

	for (i = 0; i < GRAY_SENSOR_COUNT; i++)
	{
		OLED_ShowChar(X + i * FontSize, Y,
		              Gray_Sensor[Order[i]] ? '1' : '0', FontSize);
	}
}

static void OLED_ShowUltrasonicLine(uint8_t Y, char Label,
	                                UltrasonicChannel_t Channel,
	                                uint16_t Distance)
{
	if (Distance == US_INVALID_DISTANCE_CM)
	{
		OLED_Printf(0, Y, OLED_8X16, "%c:--- E:%d", Label,
		            Ultrasonic_GetStatus(Channel));
	}
	else
	{
		OLED_Printf(0, Y, OLED_8X16, "%c:%03d cm", Label, Distance);
	}
}

static void OLED_Task(void)
{
	OLED_Clear();
	if (Display_Page == DISPLAY_PAGE_LINE)
	{
		OLED_ShowString(0, 0, Car_Running ? "RUN  IR:" : "STOP IR:", OLED_6X8);
		OLED_ShowLineStateRToL(54, 0, OLED_6X8);
		OLED_Printf(0, 10, OLED_6X8, "E:%+4d D:%+4d", Line_Error, Line_Derivative);
		OLED_Printf(0, 20, OLED_6X8, "S:%+3d B:%+3d", Line_Steer_PWM, Encoder_Balance_PWM);
		OLED_Printf(0, 30, OLED_6X8, "PL:%3d PR:%3d", PWM_Left, PWM_Right);
		OLED_Printf(0, 40, OLED_6X8, "EL:%+4d ER:%+4d", Encoder_Left, Encoder_Right);
		OLED_ShowString(0, 52, "K1 RUN  K3 US", OLED_6X8);
	}
	else
	{
		OLED_Printf(0, 0, OLED_8X16, "%s SV:%03d",
		            Car_Running ? "RUN " : "STOP", SERVO_FIXED_ANGLE);
		OLED_ShowUltrasonicLine(16, 'F', US_CH_FRONT, Front_Distance);
		OLED_ShowUltrasonicLine(32, 'L', US_CH_LEFT, Left_Distance);
		OLED_ShowUltrasonicLine(48, 'R', US_CH_RIGHT, Right_Distance);
	}
	OLED_Update();
}

static void Key_Task(void)
{
	uint8_t KeyNum;

	Key_Tick();
	KeyNum = Key_GetNum();
	if (KeyNum == KEY_NUM_K1)
	{
		Car_Running = !Car_Running;
		Car_ResetLineController();
		if (!Car_Running)
		{
			Car_Stop();
		}
	}
	else if (KeyNum == KEY_NUM_K2)
	{
		Display_Page = DISPLAY_PAGE_LINE;
	}
	else if (KeyNum == KEY_NUM_K3)
	{
		Display_Page = DISPLAY_PAGE_ULTRASONIC;
	}
}

int main(void)
{
	uint16_t DisplayLoopCount = 0;
	uint8_t UltrasonicLoopCount = 0;

	OLED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Grayscale_Init();
	Ultrasonic_Init();
	Servo_Init();
	Servo_SetAngle(SERVO_FIXED_ANGLE);
	Car_Stop();

	OLED_Clear();
	OLED_ShowString(0, 0, "Straight Track", OLED_8X16);
	OLED_ShowString(0, 24, "K1 Start/Stop", OLED_8X16);
	OLED_ShowString(0, 48, "K2 Line K3 US", OLED_8X16);
	OLED_Update();
	Delay_ms(800);

	while (1)
	{
		Car_UpdateEncoders();
		Key_Task();

		if (++UltrasonicLoopCount >= ULTRASONIC_SAMPLE_TICKS)
		{
			UltrasonicLoopCount = 0;
			Car_UpdateUltrasonicOneStep();
		}

		if (Car_Running)
		{
			Car_LineFollowStraight();
		}
		else
		{
			Car_Stop();
			Grayscale_Tick();
			Line_Error = Car_CalcLineError();
		}

		if (++DisplayLoopCount >= 5)
		{
			DisplayLoopCount = 0;
			OLED_Task();
		}
		Delay_ms(20);
	}
}
