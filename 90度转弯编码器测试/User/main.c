#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"

/*
 * 90度转弯编码器测试工程
 *
 * 测试流程：
 * K1启动 -> 原工程速度直行1秒 -> 编码器闭环90度转弯
 * -> 转弯完成后原工程速度直行1秒 -> 自动停止。
 *
 * 本工程不运行灰度、超声波、舵机和地图计时逻辑，只验证：
 * 1. 左右轮分别读取编码器累计计数；
 * 2. 左右轮分别达到各自的有符号目标值；
 * 3. 固定PWM转弯动作能否稳定完成。
 */

/* 可调窗口：主循环名义时间片，单位ms。 */
#define TEST_LOOP_PERIOD_MS          10
/* 可调窗口：转弯前直行时间，单位ms；1000表示直行1秒。 */
#define TEST_STRAIGHT_BEFORE_MS      1000
/* 可调窗口：转弯后直行时间，单位ms；1000表示直行1秒。 */
#define TEST_STRAIGHT_AFTER_MS       1000
/* 可调窗口：直行速度，沿用当前原工程CAR_BASE_PWM数值。 */
#define TEST_STRAIGHT_PWM            25
/* 可调窗口：转弯差速PWM，沿用当前原工程CAR_FIXED_TURN_PWM数值。 */
#define TEST_TURN_PWM                30
/* 可调窗口：左轮90度转弯目标，单位和原工程AVG编码器计数相同，但左右分开设置。 */
#define TEST_TURN_LEFT_TARGET        (-120)
/* 可调窗口：右轮90度转弯目标，单位和原工程AVG编码器计数相同，但左右分开设置。 */
#define TEST_TURN_RIGHT_TARGET       120
/* 可调窗口：转弯最大执行时间，防止编码器异常时小车一直转动。 */
#define TEST_TURN_MAX_MS             3000
/* 可调窗口：OLED刷新间隔，单位为主循环次数。 */
#define TEST_OLED_REFRESH_LOOPS      5

#define TEST_STATE_IDLE              0
#define TEST_STATE_STRAIGHT_BEFORE   1
#define TEST_STATE_TURN              2
#define TEST_STATE_STRAIGHT_AFTER    3
#define TEST_STATE_DONE              4
#define TEST_STATE_TIMEOUT           5

static uint8_t Test_State = TEST_STATE_IDLE;
static uint32_t Test_StateMs = 0;
static uint32_t Test_TurnMs = 0;
static int16_t Test_EncoderLeftDelta = 0;
static int16_t Test_EncoderRightDelta = 0;
static int32_t Test_LeftTotal = 0;
static int32_t Test_RightTotal = 0;
static int32_t Test_TurnLeftTotal = 0;
static int32_t Test_TurnRightTotal = 0;
static uint8_t Test_LeftReached = 0;
static uint8_t Test_RightReached = 0;

static uint8_t Test_TargetReached(int32_t Current, int32_t Target)
{
	if (Target >= 0)
	{
		return (Current >= Target) ? 1 : 0;
	}
	return (Current <= Target) ? 1 : 0;
}

static int8_t Test_TargetDirection(int32_t Target)
{
	return (Target >= 0) ? 1 : -1;
}

static void Test_StopMotors(void)
{
	Motor_Stop();
}

static void Test_SetStraightPWM(void)
{
	Motor_SetPWM(MOTOR_LEFT, TEST_STRAIGHT_PWM);
	Motor_SetPWM(MOTOR_RIGHT, TEST_STRAIGHT_PWM);
}

static void Test_SetTurnPWM(void)
{
	if (Test_LeftReached)
	{
		Motor_SetPWM(MOTOR_LEFT, 0);
	}
	else
	{
		Motor_SetPWM(MOTOR_LEFT,
		             (int8_t)(Test_TargetDirection(TEST_TURN_LEFT_TARGET) * TEST_TURN_PWM));
	}

	if (Test_RightReached)
	{
		Motor_SetPWM(MOTOR_RIGHT, 0);
	}
	else
	{
		Motor_SetPWM(MOTOR_RIGHT,
		             (int8_t)(Test_TargetDirection(TEST_TURN_RIGHT_TARGET) * TEST_TURN_PWM));
	}
}

static void Test_ClearEncoderHardware(void)
{
	/* 丢弃启动前残留的计数，确保目标从本次测试的0开始计算。 */
	Encoder1_Get();
	Encoder2_Get();
}

static void Test_ResetValues(void)
{
	Test_StateMs = 0;
	Test_TurnMs = 0;
	Test_EncoderLeftDelta = 0;
	Test_EncoderRightDelta = 0;
	Test_LeftTotal = 0;
	Test_RightTotal = 0;
	Test_TurnLeftTotal = 0;
	Test_TurnRightTotal = 0;
	Test_LeftReached = 0;
	Test_RightReached = 0;
}

static void Test_Start(void)
{
	Test_StopMotors();
	Test_ClearEncoderHardware();
	Test_ResetValues();
	Test_State = TEST_STATE_STRAIGHT_BEFORE;
}

static void Test_Abort(void)
{
	Test_StopMotors();
	Test_ResetValues();
	Test_State = TEST_STATE_IDLE;
}

static void Test_UpdateEncoders(void)
{
	Test_EncoderRightDelta = Encoder1_Get();
	Test_EncoderLeftDelta = Encoder2_Get();
	Test_RightTotal += Test_EncoderRightDelta;
	Test_LeftTotal += Test_EncoderLeftDelta;

	if (Test_State == TEST_STATE_TURN)
	{
		Test_TurnRightTotal += Test_EncoderRightDelta;
		Test_TurnLeftTotal += Test_EncoderLeftDelta;
	}
}

static void Test_UpdateState(void)
{
	if (Test_State == TEST_STATE_IDLE
	 || Test_State == TEST_STATE_DONE
	 || Test_State == TEST_STATE_TIMEOUT)
	{
		Test_StopMotors();
		return;
	}

	Test_StateMs += TEST_LOOP_PERIOD_MS;

	if (Test_State == TEST_STATE_STRAIGHT_BEFORE)
	{
		Test_SetStraightPWM();
		if (Test_StateMs >= TEST_STRAIGHT_BEFORE_MS)
		{
			/*
			 * 直行阶段结束时立即停下并清掉硬件计数，
			 * 避免下一次10ms采样把直行惯性误算到90度转弯计数中。
			 */
			Test_StopMotors();
			Test_ClearEncoderHardware();
			Test_State = TEST_STATE_TURN;
			Test_StateMs = 0;
			Test_TurnMs = 0;
			Test_TurnLeftTotal = 0;
			Test_TurnRightTotal = 0;
			Test_LeftReached = Test_TargetReached(0, TEST_TURN_LEFT_TARGET);
			Test_RightReached = Test_TargetReached(0, TEST_TURN_RIGHT_TARGET);
			Test_SetTurnPWM();
		}
		return;
	}

	if (Test_State == TEST_STATE_TURN)
	{
		Test_TurnMs += TEST_LOOP_PERIOD_MS;
		Test_LeftReached = Test_TargetReached(Test_TurnLeftTotal, TEST_TURN_LEFT_TARGET);
		Test_RightReached = Test_TargetReached(Test_TurnRightTotal, TEST_TURN_RIGHT_TARGET);

		if (Test_LeftReached && Test_RightReached)
		{
			Test_StopMotors();
			Test_State = TEST_STATE_STRAIGHT_AFTER;
			Test_StateMs = 0;
			return;
		}

		if (Test_TurnMs >= TEST_TURN_MAX_MS)
		{
			Test_StopMotors();
			Test_State = TEST_STATE_TIMEOUT;
			return;
		}

		Test_SetTurnPWM();
		return;
	}

	if (Test_State == TEST_STATE_STRAIGHT_AFTER)
	{
		Test_SetStraightPWM();
		if (Test_StateMs >= TEST_STRAIGHT_AFTER_MS)
		{
			Test_StopMotors();
			Test_State = TEST_STATE_DONE;
		}
	}
}

static char *Test_GetStateText(void)
{
	if (Test_State == TEST_STATE_STRAIGHT_BEFORE)
	{
		return "BEFORE";
	}
	if (Test_State == TEST_STATE_TURN)
	{
		return "TURN";
	}
	if (Test_State == TEST_STATE_STRAIGHT_AFTER)
	{
		return "AFTER";
	}
	if (Test_State == TEST_STATE_DONE)
	{
		return "DONE";
	}
	if (Test_State == TEST_STATE_TIMEOUT)
	{
		return "TIMEOUT";
	}
	return "IDLE";
}

static void Test_ShowOLED(void)
{
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "L90:%+06ld",
	            (long)Test_TurnLeftTotal);
	OLED_Printf(0, 16, OLED_8X16, "R90:%+06ld",
	            (long)Test_TurnRightTotal);
	OLED_Printf(0, 32, OLED_8X16, "T:%+06ld,%+06ld",
	            (long)TEST_TURN_LEFT_TARGET, (long)TEST_TURN_RIGHT_TARGET);
	OLED_Printf(0, 48, OLED_8X16, "S:%s", Test_GetStateText());
	OLED_Update();
}

static void Test_KeyTask(void)
{
	uint8_t KeyNum;

	Key_Tick();
	KeyNum = Key_GetNum();
	if (KeyNum == KEY_NUM_K1)
	{
		if (Test_State == TEST_STATE_IDLE
		 || Test_State == TEST_STATE_DONE
		 || Test_State == TEST_STATE_TIMEOUT)
		{
			Test_Start();
		}
		else
		{
			/* 测试过程中按K1作为紧急停止，并清空本次测试数据。 */
			Test_Abort();
		}
	}
}

int main(void)
{
	uint8_t OLEDLoopCount = 0;

	OLED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Test_StopMotors();
	Test_ShowOLED();

	while (1)
	{
		Delay_ms(TEST_LOOP_PERIOD_MS);
		Test_UpdateEncoders();
		Test_KeyTask();
		Test_UpdateState();

		OLEDLoopCount++;
		if (OLEDLoopCount >= TEST_OLED_REFRESH_LOOPS)
		{
			OLEDLoopCount = 0;
			Test_ShowOLED();
		}
	}
}
