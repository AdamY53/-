#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Grayscale.h"

/* Line tracking tuning. Normal tracking keeps both motors forward. */
#define CAR_BASE_PWM               33.0f
#define CAR_LINE_KP                0.090f
#define CAR_LINE_KD                0.180f
#define CAR_ENCODER_BALANCE_KP     0.350f
#define CAR_STEER_LIMIT            26.0f
#define CAR_BALANCE_LIMIT          8.0f
#define CAR_MIN_FORWARD_PWM        10.0f
#define CAR_PWM_LIMIT              70.0f

/* 可调窗口：90度拐点需要连续确认的周期数（整数），每个周期约 20ms，数值越大越不容易误触发。 */
#define CAR_SHARP_CONFIRM_TICKS    2
/* 可调窗口：T 路口判定时，M+左侧三路或 M+右侧三路中至少几个高电平才触发。 */
#define CAR_SHARP_GROUP_ACTIVE_MIN 3
/* 可调窗口：检测到 T 路口后先前进的编码器累计值，单位和 OLED 第五行 L/R 显示一致。 */
#define CAR_TURN_ENTRY_FORWARD_COUNT 120
/* 可调窗口：前进累计值到目标前的允许误差，数值越大越早进入转弯。 */
#define CAR_TURN_ENTRY_COUNT_WINDOW  8
/* 可调窗口：编码器异常时最大前探周期数，每个周期约 20ms，防止一直前进。 */
#define CAR_TURN_ENTRY_MAX_TICKS     20
/* 可调窗口：固定转弯时两个电机反方向差速 PWM，数值越大转弯越猛。 */
#define CAR_FIXED_TURN_PWM           30
/* 可调窗口：固定转弯持续周期数，每个周期约 20ms，数值越大转弯幅度越大。 */
#define CAR_FIXED_TURN_TICKS         14
/* 可调窗口：每次完成 90 度转弯后的屏蔽周期数，屏蔽期内不再次触发 90 度转弯。 */
#define CAR_TURN_COOLDOWN_TICKS      23

/* 主循环固定时间片：约 20ms。当前仅做循迹和 T 路口逻辑。 */
#define CAR_LOOP_PERIOD_MS           20

#define CAR_LINE_STATE_FOLLOW      0
#define CAR_LINE_STATE_APPROACH    1
#define CAR_LINE_STATE_FIXED_TURN  2

#define CAR_TURN_NONE              0
#define CAR_TURN_LEFT              1
#define CAR_TURN_RIGHT             2

static uint8_t Car_Running = 0;

static int16_t Encoder_Right = 0;
static int16_t Encoder_Left = 0;
static int32_t Encoder_Right_Total = 0;
static int32_t Encoder_Left_Total = 0;

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
static uint8_t Turn_Cooldown_Tick = 0;
static int32_t Turn_Entry_Left_Total = 0;
static int32_t Turn_Entry_Right_Total = 0;
static uint16_t Turn_Forward_Count = 0;
static uint16_t Turn_Last_Forward_Count = 0;
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
	Turn_Cooldown_Tick = 0;
	Turn_Entry_Left_Total = 0;
	Turn_Entry_Right_Total = 0;
	Turn_Forward_Count = 0;
	Sharp_Left_Count = 0;
	Sharp_Right_Count = 0;
	Line_Mode = 'F';
}

static void Car_UpdateEncoders(void)
{
	Encoder_Right = Encoder1_Get();
	Encoder_Left = Encoder2_Get();
	Encoder_Right_Total += Encoder_Right;
	Encoder_Left_Total += Encoder_Left;
}

static void Car_ResetEncoderTotals(void)
{
	Encoder_Right_Total = 0;
	Encoder_Left_Total = 0;
	Turn_Entry_Left_Total = 0;
	Turn_Entry_Right_Total = 0;
	Turn_Forward_Count = 0;
	Turn_Last_Forward_Count = 0;
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

static uint8_t Car_CountLeftTurnSensors(void)
{
	return Gray_Sensor[GRAY_IDX_M]
	     + Gray_Sensor[GRAY_IDX_L1]
	     + Gray_Sensor[GRAY_IDX_L2]
	     + Gray_Sensor[GRAY_IDX_L3];
}

static uint8_t Car_CountRightTurnSensors(void)
{
	return Gray_Sensor[GRAY_IDX_M]
	     + Gray_Sensor[GRAY_IDX_R1]
	     + Gray_Sensor[GRAY_IDX_R2]
	     + Gray_Sensor[GRAY_IDX_R3];
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
	Turn_Cooldown_Tick = 0;
	Sharp_Left_Count = 0;
	Sharp_Right_Count = 0;
	Turn_Entry_Left_Total = Encoder_Left_Total;
	Turn_Entry_Right_Total = Encoder_Right_Total;
	Turn_Forward_Count = 0;
	Line_Mode = 'A';
}

static void Car_FinishSharpTurn(void)
{
	Turn_Last_Forward_Count = Turn_Forward_Count;
	Car_ResetLineController();
	Turn_Cooldown_Tick = CAR_TURN_COOLDOWN_TICKS;
	Line_Mode = 'K';
}

static uint8_t Car_UpdateSharpTurnDetect(void)
{
	uint8_t LeftT;
	uint8_t RightT;

	/* 悬空保护：有效灰度数少于 2 时立刻清零左右确认计数，
	 * 防止小车被拿起/悬空时灰度误读导致 90 度转弯误触发。 */
	if (Gray_ActiveCount < 2)
	{
		Sharp_Left_Count = 0;
		Sharp_Right_Count = 0;
		return 0;
	}

	LeftT = Car_CountLeftTurnSensors();
	RightT = Car_CountRightTurnSensors();

	if ((LeftT >= CAR_SHARP_GROUP_ACTIVE_MIN) && (RightT < CAR_SHARP_GROUP_ACTIVE_MIN))
	{
		Sharp_Left_Count++;
		Sharp_Right_Count = 0;
		if (Sharp_Left_Count >= CAR_SHARP_CONFIRM_TICKS)
		{
			Car_StartSharpTurn(CAR_TURN_LEFT);
			return 1;
		}
	}
	else if ((RightT >= CAR_SHARP_GROUP_ACTIVE_MIN) && (LeftT < CAR_SHARP_GROUP_ACTIVE_MIN))
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
		int32_t ForwardDelta;

		Line_Mode = 'A';
		ForwardDelta = ((Encoder_Left_Total - Turn_Entry_Left_Total)
		              + (Encoder_Right_Total - Turn_Entry_Right_Total)) / 2;
		if (ForwardDelta < 0)
		{
			ForwardDelta = 0;
		}
		if (ForwardDelta > 65535)
		{
			ForwardDelta = 65535;
		}
		Turn_Forward_Count = (uint16_t)ForwardDelta;
		Car_SetForwardPWM(CAR_BASE_PWM, CAR_BASE_PWM);
		if (((Turn_Forward_Count + CAR_TURN_ENTRY_COUNT_WINDOW) >= CAR_TURN_ENTRY_FORWARD_COUNT)
		 || (Turn_Tick >= CAR_TURN_ENTRY_MAX_TICKS))
		{
			Turn_Tick = 0;
			Line_State = CAR_LINE_STATE_FIXED_TURN;
		}
		return;
	}

	Line_Mode = (Turn_Direction == CAR_TURN_LEFT) ? 'L' : 'R';
	Car_SetTurnPWM(Turn_Direction, CAR_FIXED_TURN_PWM);
	if (Turn_Tick >= CAR_FIXED_TURN_TICKS)
	{
		Car_FinishSharpTurn();
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

	if (Turn_Cooldown_Tick > 0)
	{
		Turn_Cooldown_Tick--;
		Sharp_Left_Count = 0;
		Sharp_Right_Count = 0;
	}
	else if (Car_UpdateSharpTurnDetect())
	{
		Car_RunSharpTurn();
		return;
	}

	Line_Error = Car_CalcLineError();
	Line_Derivative = Line_Error - Line_Last_Error;
	Line_Last_Error = Line_Error;
	Line_Mode = (Turn_Cooldown_Tick > 0) ? 'K' : 'F';

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

static void OLED_Task(void)
{
	uint16_t DisplayForwardCount;

	DisplayForwardCount = (Turn_Forward_Count != 0) ? Turn_Forward_Count : Turn_Last_Forward_Count;
	OLED_Clear();
	OLED_ShowString(0, 0, Car_Running ? "RUN  IR:" : "STOP IR:", OLED_6X8);
	OLED_ShowLineStateRToL(54, 0, OLED_6X8);
	OLED_Printf(0, 10, OLED_6X8, "E:%+4d D:%+4d", Line_Error, Line_Derivative);
	OLED_Printf(0, 20, OLED_6X8, "S:%+3d B:%+3d", Line_Steer_PWM, Encoder_Balance_PWM);
	OLED_Printf(0, 30, OLED_6X8, "PL:%+3d PR:%+3d", PWM_Left, PWM_Right);
	OLED_Printf(0, 40, OLED_6X8, "L:%+5ld R:%+5ld",
	            (long)Encoder_Left_Total, (long)Encoder_Right_Total);
	OLED_Printf(0, 52, OLED_6X8, "M:%c C:%04d W:%04d",
	            Line_Mode, DisplayForwardCount, CAR_TURN_ENTRY_FORWARD_COUNT);
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
		if (Car_Running)
		{
			Car_ResetEncoderTotals();
		}
		if (!Car_Running)
		{
			Car_Stop();
		}
	}
}

int main(void)
{
	uint16_t DisplayLoopCount = 0;

	OLED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Grayscale_Init();
	Car_Stop();

	OLED_Clear();
	OLED_ShowString(0, 0, "Straight Track", OLED_8X16);
	OLED_ShowString(0, 24, "K1 Start/Stop", OLED_8X16);
	OLED_Update();
	Delay_ms(800);

	while (1)
	{
		/* 固定 20ms 时间片；超声波已停用，循迹状态机每个循环都运行 */
		Delay_ms(CAR_LOOP_PERIOD_MS);

		Car_UpdateEncoders();
		Key_Task();

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
	}
}
