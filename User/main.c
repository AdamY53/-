#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Grayscale.h"
#include "Ultrasonic.h"
#include "Servo.h"

/* Line tracking tuning. Normal tracking keeps both motors forward. */
#define CAR_BASE_PWM               36.0f
#define CAR_LINE_KP                0.090f
#define CAR_LINE_KD                0.180f
#define CAR_ENCODER_BALANCE_KP     0.350f
#define CAR_STEER_LIMIT            26.0f
#define CAR_BALANCE_LIMIT          8.0f
#define CAR_MIN_FORWARD_PWM        10.0f
#define CAR_PWM_LIMIT              70.0f

#define CAR_SHARP_CONFIRM_TICKS    2
#define CAR_TURN_APPROACH_TICKS    4
#define CAR_TURN_MIN_TICKS         8
#define CAR_TURN_TIMEOUT_TICKS     45
#define CAR_REACQUIRE_TICKS        2
#define CAR_TURN_PWM               34
#define CAR_TURN_SEARCH_PWM        26

#define SERVO_FIXED_ANGLE          0
#define ULTRASONIC_SAMPLE_TICKS    3

#define DISPLAY_PAGE_LINE          0
#define DISPLAY_PAGE_ULTRASONIC    1

#define CAR_LINE_STATE_FOLLOW      0
#define CAR_LINE_STATE_APPROACH    1
#define CAR_LINE_STATE_TURN_MIN    2
#define CAR_LINE_STATE_SEARCH      3

#define CAR_TURN_NONE              0
#define CAR_TURN_LEFT              1
#define CAR_TURN_RIGHT             2

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
static int8_t PWM_Right = 0;
static int8_t PWM_Left = 0;

static uint8_t Line_State = CAR_LINE_STATE_FOLLOW;
static uint8_t Turn_Direction = CAR_TURN_NONE;
static uint8_t Turn_Tick = 0;
static uint8_t Turn_Reacquire_Count = 0;
static uint8_t Sharp_Left_Count = 0;
static uint8_t Sharp_Right_Count = 0;
static char Line_Mode = 'F';

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

static int8_t LimitSignedPWM(int16_t Value)
{
	if (Value > 100) {return 100;}
	if (Value < -100) {return -100;}
	return (int8_t)Value;
}

static void Car_SetForwardPWM(float LeftPWM, float RightPWM)
{
	PWM_Left = (int8_t)LimitForwardPWM(LeftPWM);
	PWM_Right = (int8_t)LimitForwardPWM(RightPWM);
	Motor_SetPWM(MOTOR_LEFT, (int8_t)PWM_Left);
	Motor_SetPWM(MOTOR_RIGHT, (int8_t)PWM_Right);
}

static void Car_SetSignedPWM(int16_t LeftPWM, int16_t RightPWM)
{
	PWM_Left = LimitSignedPWM(LeftPWM);
	PWM_Right = LimitSignedPWM(RightPWM);
	Motor_SetPWM(MOTOR_LEFT, PWM_Left);
	Motor_SetPWM(MOTOR_RIGHT, PWM_Right);
}

static void Car_Stop(void)
{
	PWM_Left = 0;
	PWM_Right = 0;
	Motor_Stop();
}

static void Car_ClearLinePD(void)
{
	Line_Error = 0;
	Line_Last_Error = 0;
	Line_Derivative = 0;
	Line_Steer_PWM = 0;
	Encoder_Balance_PWM = 0;
}

static void Car_ResetLineController(void)
{
	Car_ClearLinePD();
	Line_State = CAR_LINE_STATE_FOLLOW;
	Turn_Direction = CAR_TURN_NONE;
	Turn_Tick = 0;
	Turn_Reacquire_Count = 0;
	Sharp_Left_Count = 0;
	Sharp_Right_Count = 0;
	Line_Mode = 'F';
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

static uint8_t Car_CountLeftSensors(void)
{
	return Gray_Sensor[GRAY_IDX_L3]
	     + Gray_Sensor[GRAY_IDX_L2]
	     + Gray_Sensor[GRAY_IDX_L1];
}

static uint8_t Car_CountRightSensors(void)
{
	return Gray_Sensor[GRAY_IDX_R1]
	     + Gray_Sensor[GRAY_IDX_R2]
	     + Gray_Sensor[GRAY_IDX_R3];
}

static uint8_t Car_CenterLineDetected(void)
{
	return (Gray_Sensor[GRAY_IDX_L1]
	     || Gray_Sensor[GRAY_IDX_M]
	     || Gray_Sensor[GRAY_IDX_R1]) ? 1 : 0;
}

static void Car_SetTurnPWM(uint8_t Direction, uint8_t Speed)
{
	if (Direction == CAR_TURN_LEFT)
	{
		Car_SetSignedPWM(-(int16_t)Speed, Speed);
	}
	else if (Direction == CAR_TURN_RIGHT)
	{
		Car_SetSignedPWM(Speed, -(int16_t)Speed);
	}
}

static void Car_StartSharpTurn(uint8_t Direction)
{
	Car_ClearLinePD();
	Line_State = CAR_LINE_STATE_APPROACH;
	Turn_Direction = Direction;
	Turn_Tick = 0;
	Turn_Reacquire_Count = 0;
	Sharp_Left_Count = 0;
	Sharp_Right_Count = 0;
	Line_Mode = (Direction == CAR_TURN_LEFT) ? 'L' : 'R';
}

static uint8_t Car_UpdateSharpTurnDetect(void)
{
	uint8_t LeftCount = Car_CountLeftSensors();
	uint8_t RightCount = Car_CountRightSensors();

	if ((LeftCount >= 2) && (RightCount < 2))
	{
		Sharp_Left_Count++;
		Sharp_Right_Count = 0;
		if (Sharp_Left_Count >= CAR_SHARP_CONFIRM_TICKS)
		{
			Car_StartSharpTurn(CAR_TURN_LEFT);
			return 1;
		}
	}
	else if ((RightCount >= 2) && (LeftCount < 2))
	{
		Sharp_Right_Count++;
		Sharp_Left_Count = 0;
		if (Sharp_Right_Count >= CAR_SHARP_CONFIRM_TICKS)
		{
			Car_StartSharpTurn(CAR_TURN_RIGHT);
			return 1;
		}
	}
	else
	{
		Sharp_Left_Count = 0;
		Sharp_Right_Count = 0;
	}

	return 0;
}

static void Car_RunSharpTurn(void)
{
	Turn_Tick++;

	if (Line_State == CAR_LINE_STATE_APPROACH)
	{
		Line_Mode = 'A';
		Car_SetForwardPWM(CAR_BASE_PWM, CAR_BASE_PWM);
		if (Turn_Tick >= CAR_TURN_APPROACH_TICKS)
		{
			Turn_Tick = 0;
			Line_State = CAR_LINE_STATE_TURN_MIN;
		}
		return;
	}

	if (Line_State == CAR_LINE_STATE_TURN_MIN)
	{
		Line_Mode = (Turn_Direction == CAR_TURN_LEFT) ? 'L' : 'R';
		Car_SetTurnPWM(Turn_Direction, CAR_TURN_PWM);
		if (Turn_Tick >= CAR_TURN_MIN_TICKS)
		{
			Turn_Tick = 0;
			Turn_Reacquire_Count = 0;
			Line_State = CAR_LINE_STATE_SEARCH;
		}
		return;
	}

	Line_Mode = 'S';
	Car_SetTurnPWM(Turn_Direction, CAR_TURN_SEARCH_PWM);
	if (Car_CenterLineDetected())
	{
		Turn_Reacquire_Count++;
		if (Turn_Reacquire_Count >= CAR_REACQUIRE_TICKS)
		{
			Car_ResetLineController();
		}
	}
	else
	{
		Turn_Reacquire_Count = 0;
	}

	if (Turn_Tick >= CAR_TURN_TIMEOUT_TICKS)
	{
		Car_ResetLineController();
		Car_Stop();
	}
}

static void Car_LineFollowStraight(void)
{
	float Steer;
	float Balance;
	float LeftPWM;
	float RightPWM;
	int16_t SpeedDiff;

	Grayscale_Tick();

	if (Line_State != CAR_LINE_STATE_FOLLOW)
	{
		Car_RunSharpTurn();
		return;
	}

	if (Gray_ActiveCount == 0)
	{
		Car_ResetLineController();
		Car_Stop();
		return;
	}

	if (Car_UpdateSharpTurnDetect())
	{
		Car_RunSharpTurn();
		return;
	}

	Line_Error = Car_CalcLineError();
	Line_Derivative = Line_Error - Line_Last_Error;
	Line_Last_Error = Line_Error;
	Line_Mode = 'F';

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
		OLED_Printf(0, 30, OLED_6X8, "PL:%+3d PR:%+3d", PWM_Left, PWM_Right);
		OLED_Printf(0, 40, OLED_6X8, "EL:%+4d ER:%+4d", Encoder_Left, Encoder_Right);
		OLED_Printf(0, 52, OLED_6X8, "M:%c T:%02d K3 US", Line_Mode, Turn_Tick);
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
