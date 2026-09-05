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
#define CAR_BASE_PWM               31.0f
#define CAR_LINE_KP                0.084f
#define CAR_LINE_KD                0.064f
#define CAR_ENCODER_BALANCE_KP     0.350f
#define CAR_STEER_LIMIT            25.0f
#define CAR_BALANCE_LIMIT          5.0f
#define CAR_MIN_FORWARD_PWM        15.0f
#define CAR_PWM_LIMIT              60.0f

/* 可调窗口：90度拐点需要连续确认的周期数（整数），每个周期约 10ms，数值越大越不容易误触发。 */
#define CAR_SHARP_CONFIRM_TICKS    1
/* 可调窗口：T 路口判定时，M+左侧三路或 M+右侧三路中至少几个高电平才触发。 */
#define CAR_SHARP_GROUP_ACTIVE_MIN 3
/* 可调窗口：边缘双探头 T 路口兜底开关。1=最左两路或最右两路同时高电平也触发前进转弯。 */
#define CAR_EDGE_PAIR_T_ENABLE     0
/* 可调窗口：检测到 T 路口后先前进的编码器累计值，单位和 OLED 第五行 L/R 显示一致。 */
#define CAR_TURN_ENTRY_FORWARD_COUNT 300
/* 可调窗口：前进累计值到目标前的允许误差，数值越大越早进入转弯。 */
#define CAR_TURN_ENTRY_COUNT_WINDOW  8
/* 可调窗口：编码器异常时最大前探周期数，每个周期约 10ms，防止一直前进。 */
#define CAR_TURN_ENTRY_MAX_TICKS     20
/* 可调窗口：T 弯方向映射。若实车转弯方向相反，只改这两个宏即可。 */
#define CAR_T_LEFT_ACTION           CAR_TURN_LEFT
#define CAR_T_RIGHT_ACTION          CAR_TURN_RIGHT
/* 可调窗口：固定转弯时两个电机反方向差速 PWM，数值越大转弯越猛。 */
#define CAR_FIXED_TURN_PWM           27
/* 可调窗口：左转时左轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_LEFT_TURN_LEFT_TARGET    (-440)
/* 可调窗口：左转时右轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_LEFT_TURN_RIGHT_TARGET   730
/* 可调窗口：右转时左轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_RIGHT_TURN_LEFT_TARGET   440
/* 可调窗口：右转时右轮的编码器目标值，单位和OLED第五行L/R累计值相同。 */
#define CAR_RIGHT_TURN_RIGHT_TARGET  (-730)
/* 可调窗口：编码器未达到目标时的最大固定转弯周期数，每个周期约10ms。 */
#define CAR_FIXED_TURN_MAX_TICKS     300
/* 可调窗口：每次完成 90 度转弯后的屏蔽周期数，每个周期约 10ms，屏蔽期内不再次触发 90 度转弯。 */
#define CAR_TURN_COOLDOWN_TICKS      24

/* 主循环固定时间片：约 10ms。当前仅做循迹、路口和计时逻辑。 */
#define CAR_LOOP_PERIOD_MS           20

/* 可调窗口：A-D/D-A 路段包含 4 个中间 T 和第 5 个目标 T。 */
#define CAR_ROUTE_AD_T_COUNT         5
/* 可调窗口：其他普通路段到达目标点需要经过的 T 数量。 */
#define CAR_ROUTE_NORMAL_T_COUNT     1
/* 可调窗口：模式B中，第一个特殊 T 后直行到 Q 点附近的编码器平均累计值，单位和 OLED 的 AVG 一样。 */
#define CAR_MODE_B_Q_FORWARD_COUNT   2130
/* 可调窗口：模式B中到 Q/O 点后的停稳时间，每个周期约 10ms，50=约0.5秒。 */
#define CAR_MODE_B_STOP_TICKS        50
/* 可调窗口：模式B中，无黑线直行后看到多少路灰度为高电平才认为重新遇到黑线。 */
#define CAR_MODE_B_LINE_ACTIVE_MIN   3
/* 可调窗口：模式B重新回到黑线后，后面第几个 T 路口才算 A-D/D-A 路段结束。 */
#define CAR_MODE_B_REJOIN_T_COUNT    3

/* 可调窗口：超声波采样间隔，每个周期约10ms；前方和当前外侧模块轮流采样。 */
#define CAR_ULTRASONIC_SAMPLE_TICKS  10
/* 可调窗口：外围物块认定范围，单位厘米。 */
#define CAR_OBJECT_MIN_CM            23
#define CAR_OBJECT_MAX_CM            35
/* 可调窗口：物块离开范围的释放阈值，留出滞回避免边界抖动重复计数。 */
#define CAR_OBJECT_RELEASE_CM        45
/* 可调窗口：连续多少次进入20~40cm才计为发现一个物块。 */
#define CAR_OBJECT_CONFIRM_SAMPLES   3
/* 可调窗口：连续多少次离开释放范围才允许下一物块重新计数。 */
#define CAR_OBJECT_RELEASE_SAMPLES   3
/* 可调窗口：最多显示/统计题目要求的3个外围物块。 */
#define CAR_OBJECT_MAX_COUNT         3

/* ===== 循迹途中障碍(木块)绕行参数(重写版,按实车流程·反向D-C右绕为例) =====
 * 触发：车头云台超声(US1,90°=正前)<CAR_OBST_TRIGGER_CM 连续
 *       CAR_OBST_CONFIRM_TIMES 次前端采样后进入绕行。
 * 绕向：正向路线0-3(AB/BC/CD/DA)=左绕、贴壁用右侧超声(US3)、云台预转右；
 *       反向路线4-7(AD/DC/CB/BA)=右绕、贴壁用左侧超声(US2)、云台预转左。
 * 流程(反向·右绕)：停车0.5s(云台先预转左)→车身右转90°→直行横移
 *       (US1仍指向木块,直到无回波)→固定直走CAR_OBST_ENC_FIXED编码器→
 *       左转90°回正→停稳→贴壁直行:侧超声经历 无值→有值→无值,第二次
 *       无值时停稳→左转90°→停稳→直行找线,灰度≥LINE_MIN时停稳→右转
 *       90°回正→云台回正→交还正常循迹。所有“停稳”统一用
 *       CAR_OBST_STOP_TICKS 一个窗口调试。 */
#define CAR_OBST_TRIGGER_CM          20   /* 前端触发阈值(cm) */
#define CAR_OBST_CONFIRM_TIMES       2    /* 前端连续几次采样<阈值才触发 */
#define CAR_OBST_STOP_TICKS          50   /* 统一停稳窗口(约0.5s) */
#define CAR_OBST_DRIVE_PWM           24   /* 各直行段速度 */
#define CAR_OBST_ENC_FIXED           400  /* 固定直走编码器值(OLED AVG同单位) */
#define CAR_OBST_SIDE_DETECT_CM      60   /* 侧超声“有值”上限,大于此或无效=0 */
#define CAR_OBST_GONE_CONFIRM        2    /* 无回波/归零的确认帧数 */
#define CAR_OBST_LINE_CONFIRM        2    /* 灰度见线确认帧数 */
#define CAR_OBST_X1_MAX_TICKS        200  /* 横移段超时(约2s) */
#define CAR_OBST_WALL_MAX_TICKS      400  /* 贴壁直行超时(约4s) */
#define CAR_OBST_FIND_MAX_TICKS      300  /* 找线超时(约3s) */
#define CAR_OBST_LINE_MIN            3    /* 灰度几路亮=重新见线 */
#define CAR_OBST_SERVO_FRONT         90   /* 舵机角度=正前(用户确认90°朝前) */
#define CAR_OBST_SERVO_RIGHT         180  /* 舵机角度=朝右(实测标定) */
#define CAR_OBST_SERVO_LEFT          0    /* 舵机角度=朝左(实测标定) */

/* 绕障子状态 */
#define CAR_OBST_STATE_IDLE          0
#define CAR_OBST_STATE_STOP          1    /* 停车(云台已先预转) */
#define CAR_OBST_STATE_TURN          2    /* 正在执行一次90°闭环转弯 */
#define CAR_OBST_STATE_WAIT          3    /* 停稳窗口 */
#define CAR_OBST_STATE_X1            4    /* 横移直行:US1盯木块直到无回波 */
#define CAR_OBST_STATE_X2            5    /* 固定直走ENC编码器 */
#define CAR_OBST_STATE_WALL          6    /* 贴壁直行:侧超声 0→值→0 判零 */
#define CAR_OBST_STATE_FIND          7    /* 找线直行 */
#define CAR_OBST_STATE_ABORT         8    /* 超时保护停车 */

#define CAR_LINE_STATE_FOLLOW      0
#define CAR_LINE_STATE_APPROACH    1
#define CAR_LINE_STATE_FIXED_TURN  2

#define CAR_TURN_NONE              0
#define CAR_TURN_LEFT              1
#define CAR_TURN_RIGHT             2

#define CAR_DISPLAY_PAGE_TRACK      0
#define CAR_DISPLAY_PAGE_ROUTE      1

#define CAR_WORK_MODE_A             0
#define CAR_WORK_MODE_B             1

#define CAR_MODE_B_STATE_IDLE            0
#define CAR_MODE_B_STATE_FORWARD_TO_Q    1
#define CAR_MODE_B_STATE_STOP_Q          2
#define CAR_MODE_B_STATE_TURN_TO_O       3
#define CAR_MODE_B_STATE_FORWARD_TO_LINE 4
#define CAR_MODE_B_STATE_STOP_O          5
#define CAR_MODE_B_STATE_REJOIN_TURN     6
#define CAR_MODE_B_STATE_AFTER_REJOIN    7

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
static uint8_t Work_Mode = CAR_WORK_MODE_A;
static uint8_t ModeB_State = CAR_MODE_B_STATE_IDLE;
static uint16_t ModeB_Stop_Tick = 0;
static int32_t ModeB_Straight_Left_Total = 0;
static int32_t ModeB_Straight_Right_Total = 0;
static uint16_t ModeB_Straight_Count = 0;
static uint16_t Ultrasonic_Front_Distance = US_INVALID_DISTANCE_CM;
static uint16_t Ultrasonic_Side_Distance = US_INVALID_DISTANCE_CM;
static UltrasonicChannel_t Ultrasonic_ActiveSide = US_CH_RIGHT;
static uint8_t Ultrasonic_Sample_Tick = 0;
static uint8_t Ultrasonic_NextChannel = 0;
static uint8_t Object_Count = 0;
static uint8_t Object_Enter_Count = 0;
static uint8_t Object_Release_Count = 0;
static uint8_t Object_Latched = 0;

/* 循迹途中障碍(木块)绕行状态 */
static uint8_t Obst_State = CAR_OBST_STATE_IDLE;
static uint8_t Obst_SampleConfirm = 0;   /* 前端采样连续<阈值计数 */
static uint8_t Obst_Ready = 0;           /* 触发就绪(采样计数置位,Monitor消费) */
static uint8_t Obst_TurnSeq = 0;         /* 当前第几次90°(1..4) */
static uint8_t Obst_TurnDir = CAR_TURN_LEFT;   /* 第1/4次转向方向=绕向 */
static UltrasonicChannel_t Obst_SideChan = US_CH_RIGHT; /* 贴壁侧固定超声 */
static uint8_t Obst_Tick = 0;            /* 段内计时 */
static uint8_t Obst_AfterWait = 0;       /* WAIT结束动作 0=进WALL 1=进FIND 2=执行第3次转 3=执行第4次转 */
static uint8_t Obst_Confirm2 = 0;        /* 段内防抖计数(无回波/见线/判零) */
static uint8_t Obst_SideSeen = 0;        /* WALL段:侧超声是否已见过有值 */
static int32_t Obst_EntryLeftTotal = 0;  /* 编码器直行段起点 */
static int32_t Obst_EntryRightTotal = 0;

static uint32_t Car_Time_LastCycle = 0;
static uint8_t Car_Time_Ready = 0;

/* 本工程使用的老版本 CMSIS 未提供 DWT 结构体定义，这里只声明所需的两个寄存器。 */
typedef struct
{
	volatile uint32_t CTRL;
	volatile uint32_t CYCCNT;
} CAR_DWT_Type;

#define CAR_DWT ((CAR_DWT_Type *)0xE0001000UL)

static void Car_RunSharpTurn(void);

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

static void Car_TimeInit(void)
{
	/* 使用 Cortex-M3 的 DWT 周期计数器，不占用 TIM2/TIM3/TIM4 等外设定时器。 */
	CoreDebug->DEMCR |= 0x01000000UL;
	CAR_DWT->CYCCNT = 0;
	CAR_DWT->CTRL |= 0x00000001UL;
	Car_Time_LastCycle = CAR_DWT->CYCCNT;
	Car_Time_Ready = 1;
}

static void Car_TimeSync(void)
{
	if (Car_Time_Ready)
	{
		Car_Time_LastCycle = CAR_DWT->CYCCNT;
	}
}

static uint32_t Car_TimeElapsedMs(void)
{
	uint32_t CurrentCycle;
	uint32_t DeltaCycle;

	if (!Car_Time_Ready)
	{
		Car_TimeInit();
		return CAR_LOOP_PERIOD_MS;
	}

	CurrentCycle = CAR_DWT->CYCCNT;
	DeltaCycle = CurrentCycle - Car_Time_LastCycle;
	Car_Time_LastCycle = CurrentCycle;
	return (uint32_t)(((uint64_t)DeltaCycle * 1000ULL)
	                / (uint64_t)SystemCoreClock);
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

static UltrasonicChannel_t Car_GetActiveUltrasonicSide(void)
{
	/* 0~3为顺时针起步路线，车外侧在右边；4~7为逆时针路线，车外侧在左边。 */
	if (Route_SelectedMode < 4u)
	{
		return US_CH_RIGHT;
	}
	return US_CH_LEFT;
}

static void Car_ResetUltrasonicDetection(void)
{
	Ultrasonic_Front_Distance = US_INVALID_DISTANCE_CM;
	Ultrasonic_Side_Distance = US_INVALID_DISTANCE_CM;
	Ultrasonic_ActiveSide = Car_GetActiveUltrasonicSide();
	Ultrasonic_Sample_Tick = 0;
	Ultrasonic_NextChannel = 0;
	Object_Count = 0;
	Object_Enter_Count = 0;
	Object_Release_Count = 0;
	Object_Latched = 0;
}

static void Car_UpdateObjectCount(uint16_t Distance)
{
	uint8_t InObjectRange;

	InObjectRange = (Distance != US_INVALID_DISTANCE_CM)
	             && (Distance >= CAR_OBJECT_MIN_CM)
	             && (Distance <= CAR_OBJECT_MAX_CM);

	if (InObjectRange)
	{
		Object_Release_Count = 0;
		if (Object_Enter_Count < CAR_OBJECT_CONFIRM_SAMPLES)
		{
			Object_Enter_Count++;
		}
		if (!Object_Latched
			&& (Object_Enter_Count >= CAR_OBJECT_CONFIRM_SAMPLES))
		{
			if (Object_Count < CAR_OBJECT_MAX_COUNT)
			{
				Object_Count++;
			}
			Object_Latched = 1;
		}
	}
	else
	{
		Object_Enter_Count = 0;
		if ((Distance == US_INVALID_DISTANCE_CM)
		 || (Distance < CAR_OBJECT_MIN_CM)
		 || (Distance > CAR_OBJECT_RELEASE_CM))
		{
			if (Object_Release_Count < CAR_OBJECT_RELEASE_SAMPLES)
			{
				Object_Release_Count++;
			}
			if (Object_Release_Count >= CAR_OBJECT_RELEASE_SAMPLES)
			{
				Object_Latched = 0;
			}
		}
	}
}

static void Car_UltrasonicTask(void)
{
	uint16_t Distance;

	/* 基本要求3的外围物块统计只在模式A的地图循迹过程中启用。 */
	if ((Work_Mode != CAR_WORK_MODE_A)
	 || !Car_Running
	 || !Route_Active
	 || Route_Done)
	{
		return;
	}

	if (++Ultrasonic_Sample_Tick < CAR_ULTRASONIC_SAMPLE_TICKS)
	{
		return;
	}
	Ultrasonic_Sample_Tick = 0;
	/* 绕障期间贴壁侧固定用 Obst_SideChan；平时按当前路线选择外侧。 */
	Ultrasonic_ActiveSide = (Obst_State != CAR_OBST_STATE_IDLE)
	                      ? Obst_SideChan : Car_GetActiveUltrasonicSide();

	/* 前方和当前外侧模块轮流测量，避免三个 HC-SR04 同时发声串扰。 */
	if (Ultrasonic_NextChannel == 0u)
	{
		Distance = Ultrasonic_GetDistanceCm(US_CH_FRONT);
		Ultrasonic_Front_Distance = Distance;
		/* 障碍触发：绕障空闲时，连续几次前端采样<阈值 → 置就绪(由循迹Monitor消费) */
		if ((Obst_State == CAR_OBST_STATE_IDLE)
		 && (Distance != US_INVALID_DISTANCE_CM)
		 && (Distance < CAR_OBST_TRIGGER_CM))
		{
			if (Obst_SampleConfirm < 200u) {Obst_SampleConfirm++;}
			if (Obst_SampleConfirm >= CAR_OBST_CONFIRM_TIMES) {Obst_Ready = 1;}
		}
		else
		{
			Obst_SampleConfirm = 0;
		}
		Ultrasonic_NextChannel = 1u;
	}
	else
	{
		Distance = Ultrasonic_GetDistanceCm(Ultrasonic_ActiveSide);
		Ultrasonic_Side_Distance = Distance;
		if (Work_Mode == CAR_WORK_MODE_A)
		{
			Car_UpdateObjectCount(Distance);
		}
		Ultrasonic_NextChannel = 0u;
	}
}

static void Car_ResetModeBNav(void)
{
	ModeB_State = CAR_MODE_B_STATE_IDLE;
	ModeB_Stop_Tick = 0;
	ModeB_Straight_Left_Total = 0;
	ModeB_Straight_Right_Total = 0;
	ModeB_Straight_Count = 0;
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
	Car_ResetModeBNav();
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
		if ((Work_Mode == CAR_WORK_MODE_B)
		 && (ModeB_State == CAR_MODE_B_STATE_AFTER_REJOIN))
		{
			return CAR_MODE_B_REJOIN_T_COUNT;
		}
		return CAR_ROUTE_AD_T_COUNT;
	}

	return CAR_ROUTE_NORMAL_T_COUNT;
}

static uint8_t Car_IsADRouteStep(const CAR_ROUTE_STEP *Step)
{
	if (((Step->From == 'A') && (Step->To == 'D'))
	 || ((Step->From == 'D') && (Step->To == 'A')))
	{
		return 1;
	}
	return 0;
}

static uint8_t Car_IsCurrentModeBSpecialSegment(void)
{
	if ((Work_Mode != CAR_WORK_MODE_B)
	 || !Route_Active
	 || Route_Done
	 || (Route_SelectedMode >= CAR_ROUTE_MODE_COUNT)
	 || (Route_CurrentSegment >= CAR_ROUTE_SEGMENT_COUNT))
	{
		return 0;
	}

	return Car_IsADRouteStep(&Car_RouteMap[Route_SelectedMode][Route_CurrentSegment]);
}

static void Car_StartRouteTiming(void)
{
	Car_ResetRouteTiming();
	Car_ResetUltrasonicDetection();
	Car_TimeSync();
	Route_Active = 1;
	Route_JustStarted = 1;
}

static void Car_StopRouteTiming(void)
{
	Car_ResetRouteTiming();
}

static void Car_UpdateRouteTiming(void)
{
	uint32_t ElapsedMs;

	if (Route_JustStarted)
	{
		Route_JustStarted = 0;
		Car_TimeSync();
		return;
	}

	if (Route_Active && !Route_Done && (Route_CurrentSegment < CAR_ROUTE_SEGMENT_COUNT))
	{
		ElapsedMs = Car_TimeElapsedMs();
		if (ElapsedMs == 0u)
		{
			ElapsedMs = 1u;
		}
		Route_CurrentMs += ElapsedMs;
	}
	else
	{
		Car_TimeSync();
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
	Car_ResetModeBNav();
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
	Car_ResetUltrasonicDetection();
	Car_ResetModeBNav();
}

static void Car_ToggleWorkMode(void)
{
	/* K3 长按在 A(地图循迹)/B(点导航) 之间循环 */
	if (Work_Mode == CAR_WORK_MODE_A)
	{
		Work_Mode = CAR_WORK_MODE_B;
	}
	else
	{
		Work_Mode = CAR_WORK_MODE_A;
	}
	Car_ResetUltrasonicDetection();
	Car_ResetModeBNav();
}

static uint8_t Car_IsTurnEncoderTargetReached(int32_t Current, int32_t Target)
{
	if (Target >= 0)
	{
		return (Current >= Target) ? 1 : 0;
	}
	return (Current <= Target) ? 1 : 0;
}

static void Car_LoadTurnEncoderTargets(uint8_t Direction)
{
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

static void Car_StartSharpTurnEx(uint8_t Direction, uint8_t CountRouteTurn)
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
	Car_LoadTurnEncoderTargets(Direction);
	Line_Mode = 'A';

	/*
	 * 每个 T 路口都执行完整的前进和转弯动作。
	 * 只有当前路段累计达到目标 T 数量时，才在固定动作结束后关闭本段计时。
	 */
	Route_TurnEndsSegment = 0;
	if (CountRouteTurn
		&& Route_Active && !Route_Done
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

static void Car_StartSharpTurn(uint8_t Direction)
{
	Car_StartSharpTurnEx(Direction, 1);
}

static uint8_t Car_GetModeBQTurnDirection(void)
{
	const CAR_ROUTE_STEP *Step;

	Step = &Car_RouteMap[Route_SelectedMode][Route_CurrentSegment];
	if ((Step->From == 'A') && (Step->To == 'D'))
	{
		/* A-D 方向：到 Q 点附近后向右 90 度转向。 */
		return CAR_TURN_RIGHT;
	}

	/* D-A 方向：到 Q 点附近后向左 90 度转向。 */
	return CAR_TURN_LEFT;
}

static uint8_t Car_GetModeBRejoinTurnDirection(void)
{
	const CAR_ROUTE_STEP *Step;

	Step = &Car_RouteMap[Route_SelectedMode][Route_CurrentSegment];
	if ((Step->From == 'A') && (Step->To == 'D'))
	{
		/* A-D 方向：重新识别黑线后向左转，回到正常巡线。 */
		return CAR_TURN_LEFT;
	}

	/* D-A 方向：重新识别黑线后向右转，回到正常巡线。 */
	return CAR_TURN_RIGHT;
}

static void Car_ModeBStartStraight(void)
{
	Car_ClearLinePD();
	ModeB_Straight_Left_Total = Encoder_Left_Total;
	ModeB_Straight_Right_Total = Encoder_Right_Total;
	ModeB_Straight_Count = 0;
	Sharp_Left_Count = 0;
	Sharp_Right_Count = 0;
	Line_State = CAR_LINE_STATE_FOLLOW;
}

static uint8_t Car_ModeBRunEncoderStraight(uint8_t HasTarget, uint16_t TargetCount)
{
	int32_t ForwardDelta;
	int16_t SpeedDiff;
	float Balance;
	float LeftPWM;
	float RightPWM;

	/* 无黑线直行时，不看灰度偏差，只用左右编码器速度差做修正。 */
	ForwardDelta = ((Encoder_Left_Total - ModeB_Straight_Left_Total)
	              + (Encoder_Right_Total - ModeB_Straight_Right_Total)) / 2;
	if (ForwardDelta < 0)
	{
		ForwardDelta = 0;
	}
	if (ForwardDelta > 65535)
	{
		ForwardDelta = 65535;
	}
	ModeB_Straight_Count = (uint16_t)ForwardDelta;

	SpeedDiff = Encoder_Left - Encoder_Right;
	Balance = (float)SpeedDiff * CAR_ENCODER_BALANCE_KP;
	Balance = LimitFloat(Balance, -CAR_BALANCE_LIMIT, CAR_BALANCE_LIMIT);
	Encoder_Balance_PWM = (int8_t)Balance;
	Line_Steer_PWM = 0;
	LeftPWM = CAR_BASE_PWM - Balance;
	RightPWM = CAR_BASE_PWM + Balance;
	Car_SetForwardPWM(LeftPWM, RightPWM);

	if (HasTarget
	 && ((ModeB_Straight_Count + CAR_TURN_ENTRY_COUNT_WINDOW) >= TargetCount))
	{
		return 1;
	}
	return 0;
}

static void Car_ModeBStartEncoderTurn(uint8_t Direction)
{
	Car_ClearLinePD();
	Line_State = CAR_LINE_STATE_FIXED_TURN;
	Turn_Direction = Direction;
	Turn_Tick = 0;
	Turn_Cooldown_Tick = 0;
	Route_TurnEndsSegment = 0;
	Turn_Entry_Left_Total = Encoder_Left_Total;
	Turn_Entry_Right_Total = Encoder_Right_Total;
	Turn_Forward_Count = 0;
	Turn_Left_Encoder_Count = 0;
	Turn_Right_Encoder_Count = 0;
	Car_LoadTurnEncoderTargets(Direction);
	Turn_Left_Encoder_Reached = Car_IsTurnEncoderTargetReached(0, Turn_Left_Encoder_Target);
	Turn_Right_Encoder_Reached = Car_IsTurnEncoderTargetReached(0, Turn_Right_Encoder_Target);
	Line_Mode = (Direction == CAR_TURN_LEFT) ? 'L' : 'R';
	Car_SetTurnPWM(Direction, CAR_FIXED_TURN_PWM);
}

static void Car_ModeBStartForwardToQ(void)
{
	/* 第一个特殊 T 只作为“进入 Q/O 导航”的触发点，不执行普通前进转弯。 */
	ModeB_State = CAR_MODE_B_STATE_FORWARD_TO_Q;
	Line_Mode = 'Q';
	Car_ModeBStartStraight();
}

static void Car_ModeBStartRejoinTurn(void)
{
	ModeB_State = CAR_MODE_B_STATE_REJOIN_TURN;
	/* 重新识别黑线后的这次固定动作不计入 A-D/D-A 的后续 3 个 T。 */
	Car_StartSharpTurnEx(Car_GetModeBRejoinTurnDirection(), 0);
}

static uint8_t Car_IsModeBNavBusy(void)
{
	if ((Work_Mode == CAR_WORK_MODE_B)
	 && (ModeB_State != CAR_MODE_B_STATE_IDLE)
	 && (ModeB_State != CAR_MODE_B_STATE_AFTER_REJOIN))
	{
		return 1;
	}
	return 0;
}

static uint8_t Car_GetSharpTurnCandidate(uint8_t *Direction)
{
	uint8_t LeftTCount;
	uint8_t RightTCount;
	uint8_t LeftT;
	uint8_t RightT;

	/* 悬空保护：有效灰度数少于 2 时，不认为遇到路口。 */
	if (Gray_ActiveCount < 2)
	{
		*Direction = CAR_TURN_NONE;
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
		*Direction = CAR_TURN_LEFT;
		return 1;
	}
	if (RightT && !LeftT)
	{
		*Direction = CAR_TURN_RIGHT;
		return 1;
	}
	if (LeftT || RightT)
	{
		/* 左右都像 T 时，模式B可把它当“第一个特殊T”的触发点；
		 * 普通模式仍不直接转弯，避免方向不明确。 */
		*Direction = CAR_TURN_NONE;
		return 1;
	}

	*Direction = CAR_TURN_NONE;
	return 0;
}

static uint8_t Car_UpdateModeBFirstTDetect(void)
{
	uint8_t Direction;

	if (!Car_IsCurrentModeBSpecialSegment()
	 || (ModeB_State != CAR_MODE_B_STATE_IDLE))
	{
		return 0;
	}

	if (Car_GetSharpTurnCandidate(&Direction))
	{
		Sharp_Left_Count++;
		Sharp_Right_Count = 0;
		if (Sharp_Left_Count >= CAR_SHARP_CONFIRM_TICKS)
		{
			Car_ModeBStartForwardToQ();
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

static void Car_RunModeBNav(void)
{
	if (ModeB_State == CAR_MODE_B_STATE_FORWARD_TO_Q)
	{
		Line_Mode = 'Q';
		if (Car_ModeBRunEncoderStraight(1, CAR_MODE_B_Q_FORWARD_COUNT))
		{
			Car_Stop();
			ModeB_Stop_Tick = CAR_MODE_B_STOP_TICKS;
			ModeB_State = CAR_MODE_B_STATE_STOP_Q;
		}
		return;
	}

	if (ModeB_State == CAR_MODE_B_STATE_STOP_Q)
	{
		Car_Stop();
		Line_Mode = 'Q';
		if (ModeB_Stop_Tick > 0)
		{
			ModeB_Stop_Tick--;
			return;
		}
		ModeB_State = CAR_MODE_B_STATE_TURN_TO_O;
		Car_ModeBStartEncoderTurn(Car_GetModeBQTurnDirection());
		return;
	}

	if (ModeB_State == CAR_MODE_B_STATE_TURN_TO_O)
	{
		Car_RunSharpTurn();
		return;
	}

	if (ModeB_State == CAR_MODE_B_STATE_FORWARD_TO_LINE)
	{
		Line_Mode = 'O';
		if (Gray_ActiveCount >= CAR_MODE_B_LINE_ACTIVE_MIN)
		{
			Car_Stop();
			ModeB_Stop_Tick = CAR_MODE_B_STOP_TICKS;
			ModeB_State = CAR_MODE_B_STATE_STOP_O;
			return;
		}
		Car_ModeBRunEncoderStraight(0, 0);
		return;
	}

	if (ModeB_State == CAR_MODE_B_STATE_STOP_O)
	{
		Car_Stop();
		Line_Mode = 'O';
		if (ModeB_Stop_Tick > 0)
		{
			ModeB_Stop_Tick--;
			return;
		}
		Car_ModeBStartRejoinTurn();
		return;
	}

	if (ModeB_State == CAR_MODE_B_STATE_REJOIN_TURN)
	{
		Car_RunSharpTurn();
	}
}

static void Car_FinishSharpTurn(void)
{
	Turn_Last_Forward_Count = Turn_Forward_Count;
	Car_ResetLineController();
	Turn_Cooldown_Tick = CAR_TURN_COOLDOWN_TICKS;
	Line_Mode = 'K';

	if (ModeB_State == CAR_MODE_B_STATE_TURN_TO_O)
	{
		ModeB_State = CAR_MODE_B_STATE_FORWARD_TO_LINE;
		Line_Mode = 'O';
		Car_ModeBStartStraight();
		return;
	}
	if (ModeB_State == CAR_MODE_B_STATE_REJOIN_TURN)
	{
		ModeB_State = CAR_MODE_B_STATE_AFTER_REJOIN;
		Route_SegmentTurnCount = 0;
		return;
	}

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


/* ================= 循迹途中障碍(木块)绕行(重写版) =================
 * 按用户实车流程(反向D-C·右绕为例)：停车时云台先预转→车身右转90°
 * →直行横移(US1指向木块直到无回波)→固定直走ENC编码器→左转90°回正
 * →停稳→贴壁直行(侧超声0→值→0,第二次0停稳)→左转90°→停稳→找线
 * (灰度≥LINE_MIN停稳)→右转90°回正→云台回正→交还循迹。
 * 方向：正向路线(0-3)左绕用右超声(US3)；反向路线(4-7)右绕用左超声(US2)。 */
static uint8_t Obst_OppDir(uint8_t Dir)
{
	return (Dir == CAR_TURN_LEFT) ? CAR_TURN_RIGHT : CAR_TURN_LEFT;
}

/* 纯原地闭环90°：跳过 APPROACH，从当前编码器位置直接转 */
static void Obst_StartPivot(uint8_t Direction)
{
	Car_ClearLinePD();
	Line_State = CAR_LINE_STATE_FIXED_TURN;
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
	Car_LoadTurnEncoderTargets(Direction);
	Route_TurnEndsSegment = 0;
}

/* 启动第 Seq 次90°转弯(1/4=绕向,2/3=反向) */
static void Obst_DoPivot(uint8_t Seq)
{
	uint8_t Dir = ((Seq == 1u) || (Seq == 4u)) ? Obst_TurnDir : Obst_OppDir(Obst_TurnDir);

	Obst_TurnSeq = Seq;
	Obst_StartPivot(Dir);
	Obst_State = CAR_OBST_STATE_TURN;
}

static void Obst_ResetNav(void)
{
	Obst_State = CAR_OBST_STATE_IDLE;
	Obst_SampleConfirm = 0;
	Obst_Ready = 0;
	Obst_Tick = 0;
	Obst_Confirm2 = 0;
	Obst_SideSeen = 0;
}

/* 触发绕行：按当前路线方向定绕向/贴壁侧，停车并让云台预转 */
static void Obst_BeginBlock(void)
{
	Obst_SampleConfirm = 0;
	Obst_Ready = 0;
	Obst_Tick = 0;
	Obst_Confirm2 = 0;
	Obst_SideSeen = 0;
	Obst_EntryLeftTotal = Encoder_Left_Total;
	Obst_EntryRightTotal = Encoder_Right_Total;

	if (Route_SelectedMode >= CAR_ROUTE_MODE_COUNT / 2)
	{
		/* 反向路线(4-7)：右绕，贴壁用左超声，云台预转向左 */
		Obst_TurnDir = CAR_TURN_RIGHT;
		Obst_SideChan = US_CH_LEFT;
		Servo_SetAngle(CAR_OBST_SERVO_LEFT);
	}
	else
	{
		/* 正向路线(0-3)：左绕，贴壁用右超声，云台预转向右 */
		Obst_TurnDir = CAR_TURN_LEFT;
		Obst_SideChan = US_CH_RIGHT;
		Servo_SetAngle(CAR_OBST_SERVO_RIGHT);
	}
	Car_ResetLineController();
	Car_Stop();
	Obst_State = CAR_OBST_STATE_STOP;
}

/* 车头云台超声是否无回波(横移段:已脱离木块视场) */
static uint8_t Obst_FrontGone(void)
{
	return (Ultrasonic_Front_Distance == US_INVALID_DISTANCE_CM) ? 1 : 0;
}

/* 贴壁侧超声是否"有值"(木块侧面在测距范围) */
static uint8_t Obst_SideHas(void)
{
	return ((Ultrasonic_Side_Distance != US_INVALID_DISTANCE_CM)
	     && (Ultrasonic_Side_Distance < CAR_OBST_SIDE_DETECT_CM)) ? 1 : 0;
}

static void Car_RunObstNav(void)
{
	int32_t AvgEnc;

	if (Obst_State == CAR_OBST_STATE_IDLE)
	{
		return;
	}

	if (Obst_State == CAR_OBST_STATE_ABORT)
	{
		Car_Stop();
		return;
	}

	/* —— TURN：推进当前一次90°闭环转弯，转完按序列进下一段 —— */
	if (Obst_State == CAR_OBST_STATE_TURN)
	{
		if (Line_State != CAR_LINE_STATE_FOLLOW)
		{
			Car_RunSharpTurn();
			return;
		}
		if (Obst_TurnSeq == 1u)
		{
			/* T1完成 → 横移直行 */
			Obst_Tick = 0;
			Obst_State = CAR_OBST_STATE_X1;
		}
		else if (Obst_TurnSeq == 2u)
		{
			/* T2回正完成 → 停稳 → 贴壁 */
			Obst_Tick = 0;
			Obst_AfterWait = 0u;   /* 0=进WALL */
			Obst_State = CAR_OBST_STATE_WAIT;
		}
		else if (Obst_TurnSeq == 3u)
		{
			/* T3完成 → 停稳 → 找线 */
			Obst_Tick = 0;
			Obst_AfterWait = 1u;   /* 1=进FIND */
			Obst_State = CAR_OBST_STATE_WAIT;
		}
		else
		{
			/* T4回正完成 → 云台回正，交还循迹 */
			Servo_SetAngle(CAR_OBST_SERVO_FRONT);
			Obst_ResetNav();
		}
		return;
	}

	/* —— WAIT：统一停稳窗口 —— */
	if (Obst_State == CAR_OBST_STATE_WAIT)
	{
		Obst_Tick++;
		if (Obst_Tick >= CAR_OBST_STOP_TICKS)
		{
			Obst_Tick = 0;
			if (Obst_AfterWait == 2u)
			{
				Obst_DoPivot(3u);        /* 去执行第3次转(左转90) */
			}
			else if (Obst_AfterWait == 3u)
			{
				Obst_DoPivot(4u);        /* 去执行第4次转(右转90回正) */
			}
			else if (Obst_AfterWait == 1u)
			{
				Obst_State = CAR_OBST_STATE_FIND;
				Obst_Confirm2 = 0;
				Obst_EntryLeftTotal = Encoder_Left_Total;
				Obst_EntryRightTotal = Encoder_Right_Total;
			}
			else
			{
				Obst_State = CAR_OBST_STATE_WALL;
				Obst_Confirm2 = 0;
				Obst_SideSeen = 0;
				Obst_EntryLeftTotal = Encoder_Left_Total;
				Obst_EntryRightTotal = Encoder_Right_Total;
			}
		}
		else
		{
			Car_Stop();
		}
		return;
	}

	/* —— X1：横移直行，直到车头云台超声无回波(已越过木块视场) —— */
	if (Obst_State == CAR_OBST_STATE_X1)
	{
		Obst_Tick++;
		if (Obst_Tick >= CAR_OBST_X1_MAX_TICKS)
		{
			Obst_State = CAR_OBST_STATE_ABORT;
			Car_Stop();
			return;
		}
		if (Obst_FrontGone())
		{
			if (++Obst_Confirm2 >= CAR_OBST_GONE_CONFIRM)
			{
				Obst_Confirm2 = 0;
				Obst_Tick = 0;
				Obst_State = CAR_OBST_STATE_X2;   /* 进入固定直走 */
			}
		}
		else
		{
			Obst_Confirm2 = 0;
		}
		Car_SetSignedPWM(CAR_OBST_DRIVE_PWM, CAR_OBST_DRIVE_PWM);
		return;
	}

	/* —— X2：固定直走 CAR_OBST_ENC_FIXED 编码器 —— */
	if (Obst_State == CAR_OBST_STATE_X2)
	{
		Obst_Tick++;
		if (Obst_Tick >= CAR_OBST_WALL_MAX_TICKS)
		{
			Obst_State = CAR_OBST_STATE_ABORT;
			Car_Stop();
			return;
		}
		AvgEnc = ((Encoder_Left_Total - Obst_EntryLeftTotal)
		        + (Encoder_Right_Total - Obst_EntryRightTotal)) / 2;
		if (AvgEnc >= CAR_OBST_ENC_FIXED)
		{
			Obst_DoPivot(2u);            /* 回正(T2) */
			return;
		}
		Car_SetSignedPWM(CAR_OBST_DRIVE_PWM, CAR_OBST_DRIVE_PWM);
		return;
	}

	/* —— WALL：贴壁直行；侧超声经历 0→值→0，第二次0时停 —— */
	if (Obst_State == CAR_OBST_STATE_WALL)
	{
		Obst_Tick++;
		if (Obst_Tick >= CAR_OBST_WALL_MAX_TICKS)
		{
			Obst_State = CAR_OBST_STATE_ABORT;
			Car_Stop();
			return;
		}
		if (Obst_SideHas())
		{
			Obst_SideSeen = 1;
			Obst_Confirm2 = 0;
		}
		else if (Obst_SideSeen)
		{
			/* 已见过木块侧面，现在归零=完全越过 */
			if (++Obst_Confirm2 >= CAR_OBST_GONE_CONFIRM)
			{
				Obst_Confirm2 = 0;
				Obst_Tick = 0;
				Obst_AfterWait = 2u;     /* WAIT后执行第3次转 */
				Obst_State = CAR_OBST_STATE_WAIT;
				return;
			}
		}
		else
		{
			Obst_Confirm2 = 0;
		}
		Car_SetSignedPWM(CAR_OBST_DRIVE_PWM, CAR_OBST_DRIVE_PWM);
		return;
	}

	/* —— FIND：蛇行找线，灰度见线后停稳再第4次转回正 —— */
	if (Obst_State == CAR_OBST_STATE_FIND)
	{
		Obst_Tick++;
		if (Obst_Tick >= CAR_OBST_FIND_MAX_TICKS)
		{
			Obst_State = CAR_OBST_STATE_ABORT;
			Car_Stop();
			return;
		}
		if (Gray_ActiveCount >= CAR_OBST_LINE_MIN)
		{
			if (++Obst_Confirm2 >= CAR_OBST_LINE_CONFIRM)
			{
				Obst_Confirm2 = 0;
				Obst_Tick = 0;
				Obst_AfterWait = 3u;     /* WAIT后执行第4次转(回正) */
				Obst_State = CAR_OBST_STATE_WAIT;
				return;
			}
		}
		else
		{
			Obst_Confirm2 = 0;
		}
		/* 小幅蛇行扫线 */
		if ((Obst_Tick / 20u) & 1u)
		{
			Car_SetSignedPWM(CAR_OBST_DRIVE_PWM - 4, CAR_OBST_DRIVE_PWM);
		}
		else
		{
			Car_SetSignedPWM(CAR_OBST_DRIVE_PWM, CAR_OBST_DRIVE_PWM - 4);
		}
		return;
	}

	/* —— STOP：停车(云台已预转)，窗口后执行第1次转 —— */
	Obst_Tick++;
	if (Obst_Tick >= CAR_OBST_STOP_TICKS)
	{
		Obst_DoPivot(1u);
	}
	else
	{
		Car_Stop();
	}
}

/* 循迹每帧调用：空闲时检查触发就绪；绕障忙时接管循迹，返回1=本帧已占用 */
static uint8_t Car_ObstMonitor(void)
{
	if (Obst_State == CAR_OBST_STATE_IDLE)
	{
		/* 仅模式A地图循迹、正常压线(非转弯/冷却)时响应触发就绪 */
		if ((Work_Mode != CAR_WORK_MODE_A)
		 || !Car_Running
		 || !Route_Active
		 || Route_Done
		 || (Line_State != CAR_LINE_STATE_FOLLOW)
		 || (Turn_Cooldown_Tick > 0))
		{
			return 0;
		}
		if (Obst_Ready)
		{
			Obst_BeginBlock();
		}
		if (Obst_State == CAR_OBST_STATE_IDLE)
		{
			return 0;
		}
	}
	Car_RunObstNav();
	return 1;
}


static void Car_LineFollowStraight(void)
{
	float Steer;
	float Balance;
	float LeftPWM;
	float RightPWM;
	int16_t SpeedDiff;

	Grayscale_Tick();

	if (Car_IsModeBNavBusy())
	{
		Car_RunModeBNav();
		return;
	}

	/* 前方木块检测与绕障接管（绕障忙时本帧循迹让位） */
	if (Car_ObstMonitor())
	{
		return;
	}

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
	else if (Car_UpdateModeBFirstTDetect())
	{
		Car_RunModeBNav();
		return;
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
	if (Obst_State != CAR_OBST_STATE_IDLE)
	{
		/* 绕障调试：MODE 行显示 OBSx，x=状态号(2停/3转/4横移/5贴壁/6找线/7异常) */
		uint16_t ObstFrontDisp = (Ultrasonic_Front_Distance == US_INVALID_DISTANCE_CM)
		                       ? 0u : Ultrasonic_Front_Distance;
		uint16_t ObstSideDisp = (Ultrasonic_Side_Distance == US_INVALID_DISTANCE_CM)
		                      ? 0u : Ultrasonic_Side_Distance;
		OLED_Printf(0, 32, OLED_8X16, "OBS%d %c", Obst_State,
		            (Obst_SideChan == US_CH_RIGHT) ? 'R' : 'L');
		OLED_Printf(0, 48, OLED_8X16, "F:%u S:%u", (unsigned int)ObstFrontDisp,
		            (unsigned int)ObstSideDisp);
	}
	else
	{
	OLED_Printf(0, 32, OLED_8X16, "MODE:%s", (char *)Car_RouteModeText[Route_SelectedMode]);
	if (Route_Done)
	{
		OLED_Printf(0, 48, OLED_8X16, "SEG:DONE  %c", (Work_Mode == CAR_WORK_MODE_B) ? 'B' : 'A');
	}
	else
	{
		OLED_Printf(0, 48, OLED_8X16, "SEG:%c-%c    %c", Step->From, Step->To,
		            (Work_Mode == CAR_WORK_MODE_B) ? 'B' : 'A');
	}
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
	uint16_t FrontDistance;
	uint16_t SideDistance;
	char SideLabel;

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
	FrontDistance = (Ultrasonic_Front_Distance == US_INVALID_DISTANCE_CM)
	              ? 0u : Ultrasonic_Front_Distance;
	SideDistance = (Ultrasonic_Side_Distance == US_INVALID_DISTANCE_CM)
	             ? 0u : Ultrasonic_Side_Distance;
	SideLabel = (Ultrasonic_ActiveSide == US_CH_RIGHT) ? 'R' : 'L';
	OLED_Printf(0, 40, OLED_6X8, "F:%3ucm %c:%3ucm",
	            (unsigned int)FrontDistance, SideLabel, (unsigned int)SideDistance);
	OLED_Printf(0, 50, OLED_6X8, "OBJ:%u", (unsigned int)Object_Count);
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
	uint8_t KeyLongNum;

	Key_Tick();
	KeyLongNum = Key_GetLongNum();
	if ((KeyLongNum == KEY_LONG_K3) && !Car_Running)
	{
		Car_ToggleWorkMode();
		return;
	}

	KeyNum = Key_GetNum();
	if (KeyNum == KEY_NUM_K1)
	{
		Car_Running = !Car_Running;
		Obst_ResetNav();
		Servo_SetAngle(CAR_OBST_SERVO_FRONT);   /* 云台始终复位到正前 */
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
	Ultrasonic_Init();
	Servo_Init();
	Servo_SetAngle(CAR_OBST_SERVO_FRONT);   /* 车头云台超声默认朝正前，用于循迹中检测木块 */
	Car_TimeInit();
	Car_ResetUltrasonicDetection();
	Car_Stop();

	OLED_Clear();
	OLED_ShowString(0, 0, "Straight Track", OLED_8X16);
	OLED_ShowString(0, 24, "K1 Start/Stop", OLED_8X16);
	OLED_Update();
	Delay_ms(800);

	while (1)
	{
		/* 固定 10ms 名义时间片；主循环调度循迹/贴壁、按键、超声、计时 */
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

		Car_UltrasonicTask();
		Car_UpdateRouteTiming();

		if (++DisplayLoopCount >= 5)
		{
			DisplayLoopCount = 0;
			OLED_Task();
		}
	}
}
