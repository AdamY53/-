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

#define SERVO_RIGHT_ANGLE          30
#define SERVO_FRONT_ANGLE          90
#define SERVO_LEFT_ANGLE           150
#define SERVO_SETTLE_TICKS         12

#define OBSTACLE_DISTANCE_CM       20
#define AVOID_REVERSE_TICKS        8
#define AVOID_TURN_TICKS           16
#define AVOID_BYPASS_TICKS         30
#define AVOID_SEARCH_TIMEOUT_TICKS 70
#define AVOID_TURN_PWM             32
#define AVOID_FORWARD_PWM          34
#define AVOID_REVERSE_PWM          24
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

#define CAR_AVOID_STATE_NONE       0
#define CAR_AVOID_STATE_SCAN_LEFT  1
#define CAR_AVOID_STATE_SCAN_RIGHT 2
#define CAR_AVOID_STATE_BACK       3
#define CAR_AVOID_STATE_TURN       4
#define CAR_AVOID_STATE_BYPASS     5
#define CAR_AVOID_STATE_SEARCH     6

static uint8_t Car_Running = 0;
static uint8_t Display_Page = DISPLAY_PAGE_ULTRASONIC;

static uint16_t Front_Distance = US_INVALID_DISTANCE_CM;
static uint16_t Left_Distance = US_INVALID_DISTANCE_CM;
static uint16_t Right_Distance = US_INVALID_DISTANCE_CM;

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

static uint8_t Servo_Angle = SERVO_FRONT_ANGLE;
static uint8_t Avoid_State = CAR_AVOID_STATE_NONE;
static uint8_t Avoid_Tick = 0;
static uint8_t Avoid_Direction = CAR_TURN_RIGHT;
static uint8_t Avoid_Reacquire_Count = 0;

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

static void Car_SetServoAngle(uint8_t Angle)
{
	Servo_Angle = Angle;
	Servo_SetAngle(Angle);
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

static uint8_t Car_DistanceValid(uint16_t Distance)
{
	return (Distance != US_INVALID_DISTANCE_CM) ? 1 : 0;
}

static uint8_t Car_ObstacleDetected(uint16_t Distance)
{
	return (Car_DistanceValid(Distance) && (Distance <= OBSTACLE_DISTANCE_CM)) ? 1 : 0;
}

static void Car_UpdateEncoders(void)
{
	Encoder_Right = Encoder1_Get();
	Encoder_Left = Encoder2_Get();
}

static void Car_UpdateUltrasonicOneStep(void)
{
	if (Avoid_State == CAR_AVOID_STATE_NONE)
	{
		Front_Distance = Ultrasonic_GetDistanceCm(US_CH_FRONT);
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

static void Car_StartObstacleAvoidance(void)
{
	Car_ClearLinePD();
	Line_State = CAR_LINE_STATE_FOLLOW;
	Turn_Direction = CAR_TURN_NONE;
	Turn_Tick = 0;
	Turn_Reacquire_Count = 0;
	Sharp_Left_Count = 0;
	Sharp_Right_Count = 0;
	Avoid_State = CAR_AVOID_STATE_SCAN_LEFT;
	Avoid_Tick = 0;
	Avoid_Direction = CAR_TURN_RIGHT;
	Avoid_Reacquire_Count = 0;
	Line_Mode = 'O';
	Car_Stop();
	Car_SetServoAngle(SERVO_LEFT_ANGLE);
}

static void Car_ResetObstacleAvoidance(void)
{
	Avoid_State = CAR_AVOID_STATE_NONE;
	Avoid_Tick = 0;
	Avoid_Reacquire_Count = 0;
	Front_Distance = US_INVALID_DISTANCE_CM;
	Car_SetServoAngle(SERVO_FRONT_ANGLE);
	Car_ResetLineController();
}

static uint8_t Car_OppositeDirection(uint8_t Direction)
{
	return (Direction == CAR_TURN_LEFT) ? CAR_TURN_RIGHT : CAR_TURN_LEFT;
}

static uint8_t Car_SelectAvoidDirection(void)
{
	if (Car_DistanceValid(Left_Distance) && Car_DistanceValid(Right_Distance))
	{
		return (Left_Distance >= Right_Distance) ? CAR_TURN_LEFT : CAR_TURN_RIGHT;
	}
	if (Car_DistanceValid(Left_Distance))
	{
		return CAR_TURN_LEFT;
	}
	if (Car_DistanceValid(Right_Distance))
	{
		return CAR_TURN_RIGHT;
	}
	return CAR_TURN_RIGHT;
}

static void Car_RunObstacleAvoidance(void)
{
	Avoid_Tick++;

	if (Avoid_State == CAR_AVOID_STATE_SCAN_LEFT)
	{
		Line_Mode = 'O';
		Car_Stop();
		if (Avoid_Tick >= SERVO_SETTLE_TICKS)
		{
			Left_Distance = Ultrasonic_GetDistanceCm(US_CH_FRONT);
			Car_SetServoAngle(SERVO_RIGHT_ANGLE);
			Avoid_Tick = 0;
			Avoid_State = CAR_AVOID_STATE_SCAN_RIGHT;
		}
		return;
	}

	if (Avoid_State == CAR_AVOID_STATE_SCAN_RIGHT)
	{
		Line_Mode = 'O';
		Car_Stop();
		if (Avoid_Tick >= SERVO_SETTLE_TICKS)
		{
			Right_Distance = Ultrasonic_GetDistanceCm(US_CH_FRONT);
			Avoid_Direction = Car_SelectAvoidDirection();
			Car_SetServoAngle(SERVO_FRONT_ANGLE);
			Avoid_Tick = 0;
			Avoid_State = CAR_AVOID_STATE_BACK;
		}
		return;
	}

	if (Avoid_State == CAR_AVOID_STATE_BACK)
	{
		Line_Mode = 'B';
		Car_SetSignedPWM(-(int16_t)AVOID_REVERSE_PWM, -(int16_t)AVOID_REVERSE_PWM);
		if (Avoid_Tick >= AVOID_REVERSE_TICKS)
		{
			Avoid_Tick = 0;
			Avoid_State = CAR_AVOID_STATE_TURN;
		}
		return;
	}

	if (Avoid_State == CAR_AVOID_STATE_TURN)
	{
		Line_Mode = (Avoid_Direction == CAR_TURN_LEFT) ? 'L' : 'R';
		Car_SetTurnPWM(Avoid_Direction, AVOID_TURN_PWM);
		if (Avoid_Tick >= AVOID_TURN_TICKS)
		{
			Avoid_Tick = 0;
			Avoid_State = CAR_AVOID_STATE_BYPASS;
		}
		return;
	}

	if (Avoid_State == CAR_AVOID_STATE_BYPASS)
	{
		Line_Mode = 'P';
		Car_SetForwardPWM(AVOID_FORWARD_PWM, AVOID_FORWARD_PWM);
		if (Avoid_Tick >= AVOID_BYPASS_TICKS)
		{
			Avoid_Tick = 0;
			Avoid_Reacquire_Count = 0;
			Avoid_State = CAR_AVOID_STATE_SEARCH;
		}
		return;
	}

	Line_Mode = 'S';
	Grayscale_Tick();
	Car_SetTurnPWM(Car_OppositeDirection(Avoid_Direction), CAR_TURN_SEARCH_PWM);
	if (Car_CenterLineDetected())
	{
		Avoid_Reacquire_Count++;
		if (Avoid_Reacquire_Count >= CAR_REACQUIRE_TICKS)
		{
			Car_ResetObstacleAvoidance();
		}
	}
	else
	{
		Avoid_Reacquire_Count = 0;
	}

	if (Avoid_Tick >= AVOID_SEARCH_TIMEOUT_TICKS)
	{
		Car_ResetObstacleAvoidance();
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

	if (Avoid_State != CAR_AVOID_STATE_NONE)
	{
		Car_RunObstacleAvoidance();
		return;
	}

	if (Car_ObstacleDetected(Front_Distance))
	{
		Car_StartObstacleAvoidance();
		Car_RunObstacleAvoidance();
		return;
	}

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
		            Car_Running ? "RUN " : "STOP", Servo_Angle);
		OLED_ShowUltrasonicLine(16, 'F', US_CH_FRONT, Front_Distance);
		OLED_ShowUltrasonicLine(32, 'L', US_CH_FRONT, Left_Distance);
		OLED_ShowUltrasonicLine(48, 'R', US_CH_FRONT, Right_Distance);
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
		Car_ResetObstacleAvoidance();
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
	Car_SetServoAngle(SERVO_FRONT_ANGLE);
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
