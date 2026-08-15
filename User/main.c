#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Grayscale.h"
#include "Ultrasonic.h"
#include "Servo.h"
#include <stdio.h>

/* ==================== Obstacle line car tuning window ====================
 * K1: start/stop. K2: normal test / ultrasonic test. K3: obstacle mode.
 * Line display order on OLED: R3 R2 R1 M L1 L2 L3.
 * Straight line following uses line PD plus light encoder balance.
 */
#define CAR_BASE_PWM               36.0f
#define CAR_LINE_KP                0.090f
#define CAR_LINE_KD                0.180f
#define CAR_ENCODER_BALANCE_KP     0.350f
#define CAR_STEER_LIMIT            26.0f
#define CAR_BALANCE_LIMIT          8.0f
#define CAR_LOST_DELTA_PWM         13.0f
#define CAR_PWM_LIMIT              70.0f
#define ENCODER_COUNT_PER_REV      390L

#define OBSTACLE_DISTANCE_CM       18
#define OBSTACLE_TURN_PWM          30
#define OBSTACLE_FORWARD_PWM       30
#define OBSTACLE_TURN_TIME_MS      360
#define OBSTACLE_FORWARD_TIME_MS   520

#define SERVO_FRONT_ANGLE          90
#define SERVO_LEFT_ANGLE           150
#define SERVO_RIGHT_ANGLE          30

#define DISPLAY_PAGE_STATUS        0
#define DISPLAY_PAGE_LINE_TEST     1
#define DISPLAY_PAGE_US_TEST       2

static uint8_t Car_Running = 0;
static uint8_t Display_Page = DISPLAY_PAGE_STATUS;
static uint8_t Avoid_Mode = 0;
static int8_t Last_Line_Dir = 0;
static uint8_t Servo_Angle = SERVO_FRONT_ANGLE;

static uint16_t Front_Distance = 0;
static uint16_t Left_Distance = 0;
static uint16_t Right_Distance = 0;
static uint8_t Ultrasonic_Index = 0;

static int16_t Encoder_Right = 0;
static int16_t Encoder_Left = 0;
static int32_t Encoder_Right_Total = 0;
static int32_t Encoder_Left_Total = 0;

static int16_t Line_Weight = 0;
static int16_t Line_Last_Error = 0;
static int16_t Line_Derivative = 0;
static int8_t Line_Steer_PWM = 0;
static int8_t Encoder_Balance_PWM = 0;
static int8_t PWM_Right = 0;
static int8_t PWM_Left = 0;

static int8_t LimitPWMFloat(float Value)
{
	if (Value > CAR_PWM_LIMIT) {return (int8_t)CAR_PWM_LIMIT;}
	if (Value < -CAR_PWM_LIMIT) {return (int8_t)(-CAR_PWM_LIMIT);}
	return (int8_t)Value;
}

static int32_t Abs32(int32_t Value)
{
	return (Value < 0) ? -Value : Value;
}

static int32_t CountToRevTenths(int32_t Count)
{
	if (Count >= 0)
	{
		return (Count * 10L + ENCODER_COUNT_PER_REV / 2L) / ENCODER_COUNT_PER_REV;
	}
	return -((-Count * 10L + ENCODER_COUNT_PER_REV / 2L) / ENCODER_COUNT_PER_REV);
}

static int32_t Car_GetQTenths(void)
{
	int32_t AvgCount = (Abs32(Encoder_Right_Total) + Abs32(Encoder_Left_Total)) / 2L;
	return CountToRevTenths(AvgCount);
}

static float LimitFloat(float Value, float Min, float Max)
{
	if (Value > Max) {return Max;}
	if (Value < Min) {return Min;}
	return Value;
}

static void Car_SetServoAngle(uint8_t Angle)
{
	Servo_Angle = Angle;
	Servo_SetAngle(Angle);
}

static void Car_SetPWM(int16_t LeftPWM, int16_t RightPWM)
{
	PWM_Left = LimitPWMFloat((float)LeftPWM);
	PWM_Right = LimitPWMFloat((float)RightPWM);
	Motor_SetPWM(MOTOR_LEFT, PWM_Left);
	Motor_SetPWM(MOTOR_RIGHT, PWM_Right);
}

static void Car_Stop(void)
{
	PWM_Left = 0;
	PWM_Right = 0;
	Motor_Stop();
}

static void Car_UpdateEncoders(void)
{
	Encoder_Right = Encoder1_Get();
	Encoder_Left = Encoder2_Get();
	Encoder_Right_Total += Encoder_Right;
	Encoder_Left_Total += Encoder_Left;
}

static void Car_UpdateUltrasonicOneStep(void)
{
	if (Ultrasonic_Index == 0)
	{
		Front_Distance = Ultrasonic_GetDistanceCm(US_CH_FRONT);
	}
	else if (Ultrasonic_Index == 1)
	{
		Left_Distance = Ultrasonic_GetDistanceCm(US_CH_LEFT);
	}
	else
	{
		Right_Distance = Ultrasonic_GetDistanceCm(US_CH_RIGHT);
	}
	Ultrasonic_Index++;
	if (Ultrasonic_Index >= 3)
	{
		Ultrasonic_Index = 0;
	}
}

static int16_t Car_CalcLineError(void)
{
	int16_t PositionSum = 0;
	
	PositionSum = Gray_Sensor[GRAY_IDX_L3] * (-300)
	            + Gray_Sensor[GRAY_IDX_L2] * (-200)
	            + Gray_Sensor[GRAY_IDX_L1] * (-100)
	            + Gray_Sensor[GRAY_IDX_M]  * 0
	            + Gray_Sensor[GRAY_IDX_R1] * 100
	            + Gray_Sensor[GRAY_IDX_R2] * 200
	            + Gray_Sensor[GRAY_IDX_R3] * 300;
	
	if (Gray_ActiveCount == 0)
	{
		return Line_Last_Error;
	}
	return PositionSum / Gray_ActiveCount;
}

static void Car_LineFollowWeighted(void)
{
	float Steer;
	float Balance;
	float LeftPWM;
	float RightPWM;
	int16_t SpeedDiff;
	
	Grayscale_Tick();
	Line_Weight = Car_CalcLineError();
	
	if (Gray_ActiveCount == 0)
	{
		Line_Derivative = 0;
		Line_Steer_PWM = 0;
		Encoder_Balance_PWM = 0;
		if (Last_Line_Dir < 0)
		{
			LeftPWM = CAR_BASE_PWM - CAR_LOST_DELTA_PWM;
			RightPWM = CAR_BASE_PWM + CAR_LOST_DELTA_PWM;
		}
		else
		{
			LeftPWM = CAR_BASE_PWM + CAR_LOST_DELTA_PWM;
			RightPWM = CAR_BASE_PWM - CAR_LOST_DELTA_PWM;
		}
		Car_SetPWM((int16_t)LeftPWM, (int16_t)RightPWM);
		return;
	}
	
	if (Line_Weight < 0)
	{
		Last_Line_Dir = -1;
	}
	else if (Line_Weight > 0)
	{
		Last_Line_Dir = 1;
	}
	
	Line_Derivative = Line_Weight - Line_Last_Error;
	Line_Last_Error = Line_Weight;
	Steer = (float)Line_Weight * CAR_LINE_KP + (float)Line_Derivative * CAR_LINE_KD;
	Steer = LimitFloat(Steer, -CAR_STEER_LIMIT, CAR_STEER_LIMIT);
	
	SpeedDiff = Encoder_Left - Encoder_Right;
	Balance = (float)SpeedDiff * CAR_ENCODER_BALANCE_KP;
	Balance = LimitFloat(Balance, -CAR_BALANCE_LIMIT, CAR_BALANCE_LIMIT);
	
	Line_Steer_PWM = (int8_t)Steer;
	Encoder_Balance_PWM = (int8_t)Balance;
	LeftPWM = CAR_BASE_PWM + Steer - Balance;
	RightPWM = CAR_BASE_PWM - Steer + Balance;
	Car_SetPWM((int16_t)LeftPWM, (int16_t)RightPWM);
}

static void Car_OpenLoopSpeed(int16_t LeftPWM, int16_t RightPWM, uint16_t TimeMs)
{
	Car_Stop();
	Car_SetPWM(LeftPWM, RightPWM);
	Delay_ms(TimeMs);
	Car_Stop();
	Delay_ms(80);
}

static void Car_TurnLeft(uint16_t TimeMs)
{
	Car_OpenLoopSpeed(-OBSTACLE_TURN_PWM, OBSTACLE_TURN_PWM, TimeMs);
}

static void Car_TurnRight(uint16_t TimeMs)
{
	Car_OpenLoopSpeed(OBSTACLE_TURN_PWM, -OBSTACLE_TURN_PWM, TimeMs);
}

static uint16_t Car_ReadFrontWithServo(uint8_t Angle)
{
	Car_SetServoAngle(Angle);
	Delay_ms(220);
	return Ultrasonic_GetDistanceCm(US_CH_FRONT);
}

static void Car_AvoidObstacle(void)
{
	Car_Stop();
	Delay_ms(100);
	
	Left_Distance = Ultrasonic_GetDistanceCm(US_CH_LEFT);
	Delay_ms(40);
	Right_Distance = Ultrasonic_GetDistanceCm(US_CH_RIGHT);
	
	if (Avoid_Mode == 1)
	{
		Left_Distance = Car_ReadFrontWithServo(SERVO_LEFT_ANGLE);
		Right_Distance = Car_ReadFrontWithServo(SERVO_RIGHT_ANGLE);
		Car_SetServoAngle(SERVO_FRONT_ANGLE);
	}
	
	if (Left_Distance >= Right_Distance)
	{
		Car_TurnLeft(OBSTACLE_TURN_TIME_MS);
		Car_OpenLoopSpeed(OBSTACLE_FORWARD_PWM, OBSTACLE_FORWARD_PWM, OBSTACLE_FORWARD_TIME_MS);
		Car_TurnRight(OBSTACLE_TURN_TIME_MS);
	}
	else
	{
		Car_TurnRight(OBSTACLE_TURN_TIME_MS);
		Car_OpenLoopSpeed(OBSTACLE_FORWARD_PWM, OBSTACLE_FORWARD_PWM, OBSTACLE_FORWARD_TIME_MS);
		Car_TurnLeft(OBSTACLE_TURN_TIME_MS);
	}
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
		OLED_ShowChar(X + i * FontSize, Y, Gray_Sensor[Order[i]] ? '1' : '0', FontSize);
	}
}

static void OLED_ShowSignedTenths(uint8_t X, uint8_t Y, int32_t Value)
{
	char Sign = '+';
	if (Value < 0)
	{
		Sign = '-';
		Value = -Value;
	}
	OLED_Printf(X, Y, OLED_6X8, "%c%02ld.%ld", Sign, Value / 10L, Value % 10L);
}

static void OLED_Task(void)
{
	int32_t QR = CountToRevTenths(Encoder_Right_Total);
	int32_t QL = CountToRevTenths(Encoder_Left_Total);
	int32_t Q = Car_GetQTenths();
	
	OLED_Clear();
	if (Display_Page == DISPLAY_PAGE_STATUS)
	{
		OLED_ShowString(0, 0, "Obstacle Car", OLED_8X16);
		OLED_ShowString(0, 16, Car_Running ? "RUN " : "STOP", OLED_8X16);
		OLED_Printf(48, 16, OLED_8X16, "M:%d", Avoid_Mode);
		OLED_Printf(0, 32, OLED_8X16, "F:%03d SV:%03d", Front_Distance, Servo_Angle);
		OLED_ShowString(0, 48, "K2 Test", OLED_8X16);
	}
	else if (Display_Page == DISPLAY_PAGE_LINE_TEST)
	{
		OLED_ShowString(0, 0, "IR:", OLED_6X8);
		OLED_ShowLineStateRToL(18, 0, OLED_6X8);
		OLED_Printf(66, 0, OLED_6X8, "E:%+4d", Line_Weight);
		OLED_Printf(0, 10, OLED_6X8, "AR:%+4d AL:%+4d", Encoder_Right, Encoder_Left);
		OLED_Printf(0, 20, OLED_6X8, "S:%+3d B:%+3d", Line_Steer_PWM, Encoder_Balance_PWM);
		OLED_Printf(0, 30, OLED_6X8, "OR:%+3d OL:%+3d", PWM_Right, PWM_Left);
		OLED_ShowString(0, 40, "QR:", OLED_6X8);
		OLED_ShowSignedTenths(18, 40, QR);
		OLED_ShowString(62, 40, "QL:", OLED_6X8);
		OLED_ShowSignedTenths(80, 40, QL);
		OLED_ShowString(0, 52, "Q:", OLED_6X8);
		OLED_ShowSignedTenths(12, 52, Q);
		OLED_Printf(62, 52, OLED_6X8, "SV:%03d", Servo_Angle);
	}
	else
	{
		OLED_ShowString(0, 0, "Ultrasonic", OLED_8X16);
		OLED_Printf(0, 16, OLED_8X16, "F:%03d cm", Front_Distance);
		OLED_Printf(0, 32, OLED_8X16, "L:%03d R:%03d", Left_Distance, Right_Distance);
		OLED_Printf(0, 48, OLED_8X16, "SV:%03d M:%d", Servo_Angle, Avoid_Mode);
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
		if (!Car_Running)
		{
			Car_Stop();
		}
	}
	else if (KeyNum == KEY_NUM_K2)
	{
		if (Display_Page == DISPLAY_PAGE_LINE_TEST)
		{
			Display_Page = DISPLAY_PAGE_US_TEST;
		}
		else
		{
			Display_Page = DISPLAY_PAGE_LINE_TEST;
		}
	}
	else if (KeyNum == KEY_NUM_K3)
	{
		Avoid_Mode ^= 1;
	}
}

int main(void)
{
	uint16_t LoopCount = 0;
	
	OLED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Grayscale_Init();
	Ultrasonic_Init();
	Servo_Init();
	Car_SetServoAngle(SERVO_FRONT_ANGLE);
	
	OLED_Clear();
	OLED_ShowString(0, 0, "Obstacle Car", OLED_8X16);
	OLED_ShowString(0, 24, "K1 Start", OLED_8X16);
	OLED_ShowString(0, 48, "K2 Test K3 Mode", OLED_8X16);
	OLED_Update();
	Delay_ms(800);
	
	while (1)
	{
		Car_UpdateEncoders();
		Key_Task();
		Car_UpdateUltrasonicOneStep();
		
		if (Car_Running)
		{
			if (Front_Distance > 0 && Front_Distance < OBSTACLE_DISTANCE_CM)
			{
				Car_AvoidObstacle();
			}
			else
			{
				Car_LineFollowWeighted();
			}
		}
		else
		{
			Car_Stop();
			Grayscale_Tick();
			Line_Weight = Car_CalcLineError();
		}
		
		if (++LoopCount >= 5)
		{
			LoopCount = 0;
			OLED_Task();
		}
		Delay_ms(20);
	}
}
