#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Grayscale.h"

/* Line tracking tuning. Normal tracking keeps both motors forward. */
#define CAR_BASE_PWM               36.0f
#define CAR_LINE_KP                0.090f
#define CAR_LINE_KD                0.180f
#define CAR_ENCODER_BALANCE_KP     0.350f
#define CAR_STEER_LIMIT            26.0f
#define CAR_BALANCE_LIMIT          8.0f
#define CAR_MIN_FORWARD_PWM        10.0f
#define CAR_PWM_LIMIT              70.0f

/* 可调窗口：90度拐点需要连续确认的周期数（整数），每个周期约 20ms，数值越大越不容易误触发。 */
#define CAR_SHARP_CONFIRM_TICKS    1

/* 左右 T 弯检测模式（参考送药小车_四段PID循迹版工程移植）。
 * 模式字符串按 R3,R2,R1,M,L1,L2,L3 位序解释：'1'=必须亮，'0'=必须灭，'-'=不关心。
 * 注意：送药小车代码里 LEFT_T 命名与物理位序相反，且其转弯方向由预置路线表决定；
 * 这里按避障小车物理方向归组：右侧三路亮→右 T，左侧三路亮→左 T。 */
#define CAR_RIGHT_T_PATTERN1_ENABLE 0
#define CAR_RIGHT_T_PATTERN1        "1111000" /* 位序：R3R2R1M 亮（未启用） */
#define CAR_RIGHT_T_PATTERN2_ENABLE 1
#define CAR_RIGHT_T_PATTERN2        "111-000" /* 右 T：右侧三路亮，左侧灭，M 不关心 */
#define CAR_RIGHT_T_PATTERN3_ENABLE 1
#define CAR_RIGHT_T_PATTERN3        "1100000" /* 右 T（宽松）：R3R2 亮 */
#define CAR_RIGHT_T_PATTERN4_ENABLE 0
#define CAR_RIGHT_T_PATTERN4        "-------" /* 备用自定义右 T 信号 */
#define CAR_LEFT_T_PATTERN1_ENABLE  0
#define CAR_LEFT_T_PATTERN1         "0001111" /* 位序：ML1L2L3 亮（未启用） */
#define CAR_LEFT_T_PATTERN2_ENABLE  1
#define CAR_LEFT_T_PATTERN2         "000-111" /* 左 T：左侧三路亮，右侧灭，M 不关心 */
#define CAR_LEFT_T_PATTERN3_ENABLE  1
#define CAR_LEFT_T_PATTERN3         "0000011" /* 左 T（宽松）：L2L3 亮 */
#define CAR_LEFT_T_PATTERN4_ENABLE  0
#define CAR_LEFT_T_PATTERN4         "-------" /* 备用自定义左 T 信号 */

/* 可调窗口：T 弯方向映射。若实车转弯方向相反，只改这两个宏即可。 */
#define CAR_T_LEFT_ACTION           CAR_TURN_LEFT
#define CAR_T_RIGHT_ACTION          CAR_TURN_RIGHT
/* 可调窗口：固定转弯时两个电机反方向差速 PWM，数值越大转弯越猛。 */
#define CAR_FIXED_TURN_PWM           30
/* 可调窗口：方案一固定转弯持续周期数，每个周期约 20ms，数值越大转弯幅度越大。 */
#define CAR_FIXED_TURN_TICKS         13
/* 可调窗口：每次完成90度转弯后的屏蔽周期数，屏蔽期内不再次触发90度转弯。 */
#define CAR_TURN_COOLDOWN_TICKS      18

/* 主循环固定时间片：约 20ms。超声波已停用（专注循迹调试），循迹状态机每轮都运行。 */
#define CAR_LOOP_PERIOD_MS           20

#define CAR_LINE_STATE_FOLLOW      0
#define CAR_LINE_STATE_FIXED_TURN  1

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

static uint8_t Car_GetLineSensorMask(void)
{
	uint8_t Mask = 0;
	uint8_t i;

	for (i = 0; i < GRAY_SENSOR_COUNT; i++)
	{
		if (Gray_Sensor[i])
		{
			Mask |= (uint8_t)(1u << i);
		}
	}
	return Mask;
}

/* 按 R3,R2,R1,M,L1,L2,L3 位序匹配模式字符串：'1'=必须亮，'0'=必须灭，'-'=不关心。
 * 移植自送药小车_四段PID循迹版工程的 Car_MatchDisplayPatternByMask。 */
static uint8_t Car_MatchDisplayPatternByMask(uint8_t Mask, const char *Pattern)
{
	uint8_t i;
	const uint8_t Order[GRAY_SENSOR_COUNT] = {
		GRAY_IDX_R3, GRAY_IDX_R2, GRAY_IDX_R1, GRAY_IDX_M,
		GRAY_IDX_L1, GRAY_IDX_L2, GRAY_IDX_L3
	};

	if (Pattern == 0) {return 0;}
	for (i = 0; i < GRAY_SENSOR_COUNT; i++)
	{
		if (Pattern[i] == '1')
		{
			if ((Mask & (uint8_t)(1u << Order[i])) == 0) {return 0;}
		}
		else if (Pattern[i] == '0')
		{
			if (Mask & (uint8_t)(1u << Order[i])) {return 0;}
		}
		else if (Pattern[i] == '\0')
		{
			return 0;
		}
	}
	return 1;
}

static uint8_t Car_MatchEnabledDisplayPattern(uint8_t Mask, const char *Pattern, uint8_t Enable)
{
	if (!Enable) {return 0;}
	return Car_MatchDisplayPatternByMask(Mask, Pattern);
}

/* 左 T：物理左侧亮（L1/L2/L3），右侧灭。触发左转。 */
static uint8_t Car_IsLeftTBranchSignal(void)
{
	uint8_t Mask;

	Mask = Car_GetLineSensorMask();
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_LEFT_T_PATTERN1, CAR_LEFT_T_PATTERN1_ENABLE)) {return 1;}
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_LEFT_T_PATTERN2, CAR_LEFT_T_PATTERN2_ENABLE)) {return 1;}
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_LEFT_T_PATTERN3, CAR_LEFT_T_PATTERN3_ENABLE)) {return 1;}
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_LEFT_T_PATTERN4, CAR_LEFT_T_PATTERN4_ENABLE)) {return 1;}
	return 0;
}

/* 右 T：物理右侧亮（R1/R2/R3），左侧灭。触发右转。 */
static uint8_t Car_IsRightTBranchSignal(void)
{
	uint8_t Mask;

	Mask = Car_GetLineSensorMask();
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_RIGHT_T_PATTERN1, CAR_RIGHT_T_PATTERN1_ENABLE)) {return 1;}
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_RIGHT_T_PATTERN2, CAR_RIGHT_T_PATTERN2_ENABLE)) {return 1;}
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_RIGHT_T_PATTERN3, CAR_RIGHT_T_PATTERN3_ENABLE)) {return 1;}
	if (Car_MatchEnabledDisplayPattern(Mask, CAR_RIGHT_T_PATTERN4, CAR_RIGHT_T_PATTERN4_ENABLE)) {return 1;}
	return 0;
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
	Line_State = CAR_LINE_STATE_FIXED_TURN;
	Turn_Direction = Direction;
	Turn_Tick = 0;
	Turn_Cooldown_Tick = 0;
	Sharp_Left_Count = 0;
	Sharp_Right_Count = 0;
	Line_Mode = (Direction == CAR_TURN_LEFT) ? 'L' : 'R';
}

static void Car_FinishSharpTurn(void)
{
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

	LeftT = Car_IsLeftTBranchSignal();
	RightT = Car_IsRightTBranchSignal();

	if (LeftT && !RightT)
	{
		Sharp_Left_Count++;
		Sharp_Right_Count = 0;
		if (Sharp_Left_Count >= CAR_SHARP_CONFIRM_TICKS)
		{
			Car_StartSharpTurn(CAR_T_LEFT_ACTION);
			return 1;
		}
	}
	else if (RightT && !LeftT)
	{
		Sharp_Right_Count++;
		Sharp_Left_Count = 0;
		if (Sharp_Right_Count >= CAR_SHARP_CONFIRM_TICKS)
		{
			Car_StartSharpTurn(CAR_T_RIGHT_ACTION);
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

	if (Gray_ActiveCount == 0)
	{
		Car_ResetLineController();
		Car_Stop();
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
	OLED_Clear();
	OLED_ShowString(0, 0, Car_Running ? "RUN  IR:" : "STOP IR:", OLED_6X8);
	OLED_ShowLineStateRToL(54, 0, OLED_6X8);
	OLED_Printf(0, 10, OLED_6X8, "E:%+4d D:%+4d", Line_Error, Line_Derivative);
	OLED_Printf(0, 20, OLED_6X8, "S:%+3d B:%+3d", Line_Steer_PWM, Encoder_Balance_PWM);
	OLED_Printf(0, 30, OLED_6X8, "PL:%+3d PR:%+3d", PWM_Left, PWM_Right);
	OLED_Printf(0, 40, OLED_6X8, "L:%+5ld R:%+5ld",
	            (long)Encoder_Left_Total, (long)Encoder_Right_Total);
	OLED_Printf(0, 52, OLED_6X8, "M:%c", Line_Mode);
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
