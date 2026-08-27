#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Grayscale.h"

/* Line tracking tuning. Normal tracking keeps both motors forward. */
#define CAR_BASE_PWM               25.0f
#define CAR_LINE_KP                0.092f
#define CAR_LINE_KD                0.180f
#define CAR_ENCODER_BALANCE_KP     0.350f
#define CAR_STEER_LIMIT            26.0f
#define CAR_BALANCE_LIMIT          8.0f
#define CAR_MIN_FORWARD_PWM        10.0f
#define CAR_PWM_LIMIT              70.0f

/* 可调窗口：90度拐点需要连续确认的周期数（整数），每个周期约 10ms，数值越大越不容易误触发。 */
#define CAR_SHARP_CONFIRM_TICKS    1
/* 可调窗口：T 路口判定时，M+左侧三路或 M+右侧三路中至少几个高电平才触发。 */
#define CAR_SHARP_GROUP_ACTIVE_MIN 3
/* 可调窗口：边缘双探头 T 路口兜底开关。1=最左两路或最右两路同时高电平也触发前进转弯。 */
#define CAR_EDGE_PAIR_T_ENABLE     1
/* 可调窗口：检测到 T 路口后先前进的编码器累计值，单位和 OLED 第五行 L/R 显示一致。 */
#define CAR_TURN_ENTRY_FORWARD_COUNT 160
/* 可调窗口：前进累计值到目标前的允许误差，数值越大越早进入转弯。 */
#define CAR_TURN_ENTRY_COUNT_WINDOW  8
/* 可调窗口：编码器异常时最大前探周期数，每个周期约 10ms，防止一直前进。 */
#define CAR_TURN_ENTRY_MAX_TICKS     20
/* 可调窗口：T 弯方向映射。若实车转弯方向相反，只改这两个宏即可。 */
#define CAR_T_LEFT_ACTION           CAR_TURN_LEFT
#define CAR_T_RIGHT_ACTION          CAR_TURN_RIGHT
/* 可调窗口：固定转弯时两个电机反方向差速 PWM，数值越大转弯越猛。 */
#define CAR_FIXED_TURN_PWM           30
/* 可调窗口：左转时左轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_LEFT_TURN_LEFT_TARGET    (-550)
/* 可调窗口：左转时右轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_LEFT_TURN_RIGHT_TARGET   820
/* 可调窗口：右转时左轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_RIGHT_TURN_LEFT_TARGET   550
/* 可调窗口：右转时右轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_RIGHT_TURN_RIGHT_TARGET  (-820)
/* 可调窗口：编码器未达到目标时的最大固定转弯周期数，每个周期约10ms。 */
#define CAR_FIXED_TURN_MAX_TICKS     300
/* 可调窗口：每次完成 90 度转弯后的屏蔽周期数，每个周期约 10ms，屏蔽期内不再次触发 90 度转弯。 */
#define CAR_TURN_COOLDOWN_TICKS      24

/* 主循环固定时间片：约 10ms。当前仅做循迹、路口和计时逻辑。 */
#define CAR_LOOP_PERIOD_MS           10

/* 可调窗口：A-D/D-A 路段包含 4 个中间 T 和第 5 个目标 T。 */
#define CAR_ROUTE_AD_T_COUNT         5
/* 可调窗口：其他普通路段到达目标点需要经过的 T 数量。 */
#define CAR_ROUTE_NORMAL_T_COUNT     1

#define CAR_LINE_STATE_FOLLOW      0
#define CAR_LINE_STATE_APPROACH    1
#define CAR_LINE_STATE_FIXED_TURN  2

#define CAR_TURN_NONE              0
#define CAR_TURN_LEFT              1
#define CAR_TURN_RIGHT             2

#define CAR_DISPLAY_PAGE_TRACK      0
#define CAR_DISPLAY_PAGE_ROUTE      1

#define CAR_ROUTE_MODE_COUNT        8
#define CAR_ROUTE_SEGMENT_COUNT     4

typedef struct
{
	char From;
	char To;
} CAR_ROUTE_STEP;

static const char Car_RouteModeText[CAR_ROUTE_MODE_COUNT][8] = {
	"A_TO_B", "B_TO_C", "C_TO_D", "D_TO_A",
	"B_TO_A", "A_TO_D", "D_TO_C", "C_TO_B"
};

static const CAR_ROUTE_STEP Car_RouteMap[CAR_ROUTE_MODE_COUNT][CAR_ROUTE_SEGMENT_COUNT] = {
	{{'A', 'B'}, {'B', 'C'}, {'C', 'D'}, {'D', 'A'}},
	{{'B', 'C'}, {'C', 'D'}, {'D', 'A'}, {'A', 'B'}},
	{{'C', 'D'}, {'D', 'A'}, {'A', 'B'}, {'B', 'C'}},
	{{'D', 'A'}, {'A', 'B'}, {'B', 'C'}, {'C', 'D'}},
	{{'B', 'A'}, {'A', 'D'}, {'D', 'C'}, {'C', 'B'}},
	{{'A', 'D'}, {'D', 'C'}, {'C', 'B'}, {'B', 'A'}},
	{{'D', 'C'}, {'C', 'B'}, {'B', 'A'}, {'A', 'D'}},
	{{'C', 'B'}, {'B', 'A'}, {'A', 'D'}, {'D', 'C'}}
};

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
static uint16_t Turn_Tick = 0;
static uint8_t Turn_Cooldown_Tick = 0;
static int32_t Turn_Entry_Left_Total = 0;
static int32_t Turn_Entry_Right_Total = 0;
static uint16_t Turn_Forward_Count = 0;
static uint16_t Turn_Last_Forward_Count = 0;
static int32_t Turn_Left_Encoder_Count = 0;
static int32_t Turn_Right_Encoder_Count = 0;
static int32_t Turn_Left_Encoder_Target = 0;
static int32_t Turn_Right_Encoder_Target = 0;
static uint8_t Turn_Left_Encoder_Reached = 0;
static uint8_t Turn_Right_Encoder_Reached = 0;
static uint8_t Sharp_Left_Count = 0;
static uint8_t Sharp_Right_Count = 0;
static char Line_Mode = 'F';
static uint8_t OLED_Page = CAR_DISPLAY_PAGE_TRACK;
static uint8_t Route_SelectedMode = 0;
static uint8_t Route_Active = 0;
static uint8_t Route_Done = 0;
static uint8_t Route_CurrentSegment = 0;
static uint8_t Route_SegmentWaiting = 0;
static uint8_t Route_JustStarted = 0;
static uint8_t Route_SegmentTurnCount = 0;
static uint8_t Route_TurnEndsSegment = 0;
static uint32_t Route_CurrentMs = 0;
static uint32_t Route_SegmentMs[CAR_ROUTE_SEGMENT_COUNT] = {0};
static uint8_t Route_SegmentClosed[CAR_ROUTE_SEGMENT_COUNT] = {0};

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
	Turn_Left_Encoder_Count = 0;
	Turn_Right_Encoder_Count = 0;
	Turn_Left_Encoder_Target = 0;
	Turn_Right_Encoder_Target = 0;
	Turn_Left_Encoder_Reached = 0;
	Turn_Right_Encoder_Reached = 0;
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

static void Car_ResetRouteTiming(void)
{
	uint8_t i;

	Route_Active = 0;
	Route_Done = 0;
	Route_CurrentSegment = 0;
	Route_SegmentWaiting = 0;
	Route_JustStarted = 0;
	Route_SegmentTurnCount = 0;
	Route_TurnEndsSegment = 0;
	Route_CurrentMs = 0;
	for (i = 0; i < CAR_ROUTE_SEGMENT_COUNT; i++)
	{
		Route_SegmentMs[i] = 0;
		Route_SegmentClosed[i] = 0;
	}
}

static uint8_t Car_GetRouteTurnTarget(void)
{
	const CAR_ROUTE_STEP *Step;

	if ((Route_SelectedMode >= CAR_ROUTE_MODE_COUNT)
	 || (Route_CurrentSegment >= CAR_ROUTE_SEGMENT_COUNT))
	{
		return CAR_ROUTE_NORMAL_T_COUNT;
	}

	Step = &Car_RouteMap[Route_SelectedMode][Route_CurrentSegment];
	if (((Step->From == 'A') && (Step->To == 'D'))
	 || ((Step->From == 'D') && (Step->To == 'A')))
	{
		return CAR_ROUTE_AD_T_COUNT;
	}

	return CAR_ROUTE_NORMAL_T_COUNT;
}

static void Car_StartRouteTiming(void)
{
	Car_ResetRouteTiming();
	Route_Active = 1;
	Route_JustStarted = 1;
}

static void Car_StopRouteTiming(void)
{
	Car_ResetRouteTiming();
}

static void Car_UpdateRouteTiming(void)
{
	if (Route_JustStarted)
	{
		Route_JustStarted = 0;
		return;
	}

	if (Route_Active && !Route_Done && (Route_CurrentSegment < CAR_ROUTE_SEGMENT_COUNT))
	{
		Route_CurrentMs += CAR_LOOP_PERIOD_MS;
	}
}

static void Car_CloseRouteSegment(void)
{
	if (Route_Done || (Route_CurrentSegment >= CAR_ROUTE_SEGMENT_COUNT) || !Route_Active)
	{
		return;
	}

	Route_SegmentMs[Route_CurrentSegment] = Route_CurrentMs;
	Route_SegmentClosed[Route_CurrentSegment] = 1;
	Route_CurrentMs = 0;
	Route_CurrentSegment++;
	Route_Active = 0;
	Route_SegmentWaiting = 1;
}

static void Car_AdvanceRouteSegment(void)
{
	if (!Route_SegmentWaiting || Route_Done)
	{
		return;
	}

	Route_SegmentWaiting = 0;
	Route_SegmentTurnCount = 0;
	if (Route_CurrentSegment < CAR_ROUTE_SEGMENT_COUNT)
	{
		Route_Active = 1;
		Route_JustStarted = 1;
	}
	else
	{
		Route_Done = 1;
		Route_Active = 0;
		Route_CurrentMs = 0;
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

static uint8_t Car_IsLeftEdgePairTSignal(void)
{
	/* 最左边两路同时高电平时，也认为遇到左侧 T 路口。
	 * 这条兜底判定不看 L1/M/R1，避免中间三路循迹调整影响 T 路口触发。 */
	return (Gray_Sensor[GRAY_IDX_L2] && Gray_Sensor[GRAY_IDX_L3]) ? 1 : 0;
}

static uint8_t Car_IsRightEdgePairTSignal(void)
{
	/* 最右边两路同时高电平时，也认为遇到右侧 T 路口。
	 * 这条兜底判定不看 L1/M/R1，避免中间三路循迹调整影响 T 路口触发。 */
	return (Gray_Sensor[GRAY_IDX_R2] && Gray_Sensor[GRAY_IDX_R3]) ? 1 : 0;
}

static void Car_NextRouteMode(void)
{
	Route_SelectedMode = (uint8_t)((Route_SelectedMode + 1u) % CAR_ROUTE_MODE_COUNT);
}

static uint8_t Car_IsTurnEncoderTargetReached(int32_t Current, int32_t Target)
{
	if (Target >= 0)
	{
		return (Current >= Target) ? 1 : 0;
	}
	return (Current <= Target) ? 1 : 0;
}

static void Car_SetTurnPWM(uint8_t Direction, uint8_t Speed)
{
	int16_t LeftPWM;
	int16_t RightPWM;

	if (Direction == CAR_TURN_LEFT)
	{
		LeftPWM = -(int16_t)Speed;
		RightPWM = Speed;
	}
	else if (Direction == CAR_TURN_RIGHT)
	{
		LeftPWM = Speed;
		RightPWM = -(int16_t)Speed;
	}
	else
	{
		LeftPWM = 0;
		RightPWM = 0;
	}

	/* 某个轮子先达到目标后，单独停止该轮，另一轮继续完成剩余角度。 */
	if (Turn_Left_Encoder_Reached)
	{
		LeftPWM = 0;
	}
	if (Turn_Right_Encoder_Reached)
	{
		RightPWM = 0;
	}

	Car_SetSignedPWM(LeftPWM, RightPWM);
}

static void Car_StartSharpTurn(uint8_t Direction)
{
	uint8_t TargetTurnCount;

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
	Turn_Left_Encoder_Count = 0;
	Turn_Right_Encoder_Count = 0;
	Turn_Left_Encoder_Reached = 0;
	Turn_Right_Encoder_Reached = 0;
	if (Direction == CAR_TURN_LEFT)
	{
		Turn_Left_Encoder_Target = CAR_LEFT_TURN_LEFT_TARGET;
		Turn_Right_Encoder_Target = CAR_LEFT_TURN_RIGHT_TARGET;
	}
	else
	{
		Turn_Left_Encoder_Target = CAR_RIGHT_TURN_LEFT_TARGET;
		Turn_Right_Encoder_Target = CAR_RIGHT_TURN_RIGHT_TARGET;
	}
	Line_Mode = 'A';

	/*
	 * 每个 T 路口都执行完整的前进和转弯动作。
	 * 只有当前路段累计达到目标 T 数量时，才在固定动作结束后关闭本段计时。
	 */
	Route_TurnEndsSegment = 0;
	if (Route_Active && !Route_Done
		&& (Route_CurrentSegment < CAR_ROUTE_SEGMENT_COUNT))
	{
		if (Route_SegmentTurnCount < 255)
		{
			Route_SegmentTurnCount++;
		}
		TargetTurnCount = Car_GetRouteTurnTarget();
		if (Route_SegmentTurnCount >= TargetTurnCount)
		{
			Route_TurnEndsSegment = 1;
		}
	}
}

static void Car_FinishSharpTurn(void)
{
	Turn_Last_Forward_Count = Turn_Forward_Count;
	Car_ResetLineController();
	Turn_Cooldown_Tick = CAR_TURN_COOLDOWN_TICKS;
	Line_Mode = 'K';

	/*
	 * 当前 T 的固定动作结束后，才结束目标路段计时。
	 * A-D/D-A 的前 4 个中间 T 不会切换路段，计时会继续累加。
	 */
	if (Route_TurnEndsSegment)
	{
		Route_TurnEndsSegment = 0;
		Car_CloseRouteSegment();
		Car_AdvanceRouteSegment();
	}
}

static uint8_t Car_UpdateSharpTurnDetect(void)
{
	uint8_t LeftTCount;
	uint8_t RightTCount;
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

	LeftTCount = Car_CountLeftTurnSensors();
	RightTCount = Car_CountRightTurnSensors();
	LeftT = (LeftTCount >= CAR_SHARP_GROUP_ACTIVE_MIN)
	     || (CAR_EDGE_PAIR_T_ENABLE && Car_IsLeftEdgePairTSignal());
	RightT = (RightTCount >= CAR_SHARP_GROUP_ACTIVE_MIN)
	      || (CAR_EDGE_PAIR_T_ENABLE && Car_IsRightEdgePairTSignal());

	if (LeftT && !RightT)
	{
		Sharp_Left_Count++;
		Sharp_Right_Count = 0;
		if (Sharp_Left_Count >= CAR_SHARP_CONFIRM_TICKS)
		{
			Car_StartSharpTurn(CAR_TURN_LEFT);
			return 1;
		}
	}
	else if (RightT && !LeftT)
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
		/* 这里直接用左右编码器累计值的平均数当“前进距离”。
		 * OLED 第五行显示的 L/R 数值，就是你现场调这个窗口的依据。 */
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
			/*
			 * 前进阶段结束后重新记录起点，
			 * 90度目标只统计固定转弯阶段的左右轮计数。
			 */
			Turn_Entry_Left_Total = Encoder_Left_Total;
			Turn_Entry_Right_Total = Encoder_Right_Total;
			Turn_Left_Encoder_Count = 0;
			Turn_Right_Encoder_Count = 0;
			Turn_Left_Encoder_Reached = Car_IsTurnEncoderTargetReached(
				0, Turn_Left_Encoder_Target);
			Turn_Right_Encoder_Reached = Car_IsTurnEncoderTargetReached(
				0, Turn_Right_Encoder_Target);
			Line_State = CAR_LINE_STATE_FIXED_TURN;
			/* 同一循环立即切换到反向差速，避免多跑一个10ms直行周期。 */
			Car_SetTurnPWM(Turn_Direction, CAR_FIXED_TURN_PWM);
		}
		return;
	}

	Line_Mode = (Turn_Direction == CAR_TURN_LEFT) ? 'L' : 'R';
	Turn_Left_Encoder_Count = Encoder_Left_Total - Turn_Entry_Left_Total;
	Turn_Right_Encoder_Count = Encoder_Right_Total - Turn_Entry_Right_Total;
	Turn_Left_Encoder_Reached = Car_IsTurnEncoderTargetReached(
		Turn_Left_Encoder_Count, Turn_Left_Encoder_Target);
	Turn_Right_Encoder_Reached = Car_IsTurnEncoderTargetReached(
		Turn_Right_Encoder_Count, Turn_Right_Encoder_Target);

	if (Turn_Left_Encoder_Reached && Turn_Right_Encoder_Reached)
	{
		/* 两轮都达到目标后立即清零PWM，避免下一循环继续保持反向差速。 */
		Car_Stop();
		Car_FinishSharpTurn();
		return;
	}

	if (Turn_Tick >= CAR_FIXED_TURN_MAX_TICKS)
	{
		/* 编码器异常或机械卡住时，用最大周期作为保护性结束条件。 */
		Car_Stop();
		Car_FinishSharpTurn();
		return;
	}

	Car_SetTurnPWM(Turn_Direction, CAR_FIXED_TURN_PWM);
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

static uint32_t Car_GetRouteDisplayMs(uint8_t Index)
{
	if (Index >= CAR_ROUTE_SEGMENT_COUNT)
	{
		return 0;
	}

	if (Route_SegmentClosed[Index])
	{
		return Route_SegmentMs[Index];
	}
	if (Index == Route_CurrentSegment)
	{
		return Route_CurrentMs;
	}
	return 0;
}

static void Car_MsToDisplayTime(uint32_t Ms, uint32_t *Sec, uint32_t *Centi)
{
	uint32_t TotalCenti;

	TotalCenti = (Ms + 5u) / 10u;
	*Sec = TotalCenti / 100u;
	*Centi = TotalCenti % 100u;
}

static void OLED_ShowTrackPage(void)
{
	const CAR_ROUTE_STEP *Step;
	int32_t AvgEncoderTotal;

	Step = &Car_RouteMap[Route_SelectedMode][(Route_CurrentSegment < CAR_ROUTE_SEGMENT_COUNT) ? Route_CurrentSegment : (CAR_ROUTE_SEGMENT_COUNT - 1u)];
	AvgEncoderTotal = (Encoder_Left_Total + Encoder_Right_Total) / 2;
	OLED_Clear();
	OLED_ShowString(0, 0, "IR:", OLED_8X16);
	OLED_ShowLineStateRToL(24, 0, OLED_8X16);
	OLED_Printf(0, 16, OLED_8X16, "AVG:%+5ld", (long)AvgEncoderTotal);
	OLED_Printf(0, 32, OLED_8X16, "MODE:%s", (char *)Car_RouteModeText[Route_SelectedMode]);
	if (Route_Done)
	{
		OLED_ShowString(0, 48, "SEG:DONE", OLED_8X16);
	}
	else
	{
		OLED_Printf(0, 48, OLED_8X16, "SEG:%c-%c", Step->From, Step->To);
	}
	OLED_Update();
}

static void OLED_ShowRoutePage(void)
{
	uint8_t i;
	uint8_t ShowCount;
	uint32_t Ms;
	uint32_t Sec;
	uint32_t Centi;
	const CAR_ROUTE_STEP *Step;

	OLED_Clear();
	if (Route_Done || (Route_CurrentSegment >= CAR_ROUTE_SEGMENT_COUNT))
	{
		ShowCount = CAR_ROUTE_SEGMENT_COUNT;
	}
	else
	{
		ShowCount = (uint8_t)(Route_CurrentSegment + 1u);
	}

	for (i = 0; i < ShowCount; i++)
	{
		Step = &Car_RouteMap[Route_SelectedMode][i];
		Ms = Car_GetRouteDisplayMs(i);
		Car_MsToDisplayTime(Ms, &Sec, &Centi);
		OLED_Printf(0, (int16_t)(i * 10u), OLED_6X8, "%c-%c:%02lu.%02luS",
		            Step->From, Step->To, (unsigned long)Sec, (unsigned long)Centi);
	}
	OLED_Update();
}

static void OLED_Task(void)
{
	if (OLED_Page == CAR_DISPLAY_PAGE_ROUTE)
	{
		OLED_ShowRoutePage();
	}
	else
	{
		OLED_ShowTrackPage();
	}
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
			Car_StartRouteTiming();
		}
		if (!Car_Running)
		{
			Car_Stop();
			Car_StopRouteTiming();
		}
	}
	else if (KeyNum == KEY_NUM_K2)
	{
		OLED_Page ^= 1u;
	}
	else if ((KeyNum == KEY_NUM_K3) && !Car_Running)
	{
		Car_NextRouteMode();
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
		/* 固定 10ms 名义时间片；超声波已停用，循迹状态机每个循环都运行 */
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

		Car_UpdateRouteTiming();

		if (++DisplayLoopCount >= 5)
		{
			DisplayLoopCount = 0;
			OLED_Task();
		}
	}
}
