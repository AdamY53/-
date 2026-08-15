#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "stm32f10x.h"                  // Device header

/* 7 路一体循迹模块，逻辑顺序：L3,L2,L1,M,R1,R2,R3�?
 * 接线沿用原工程可用引脚：
 * L3 -> PA4, L2 -> PB4, L1 -> PA3, M -> PA2,
 * R1 -> PA1, R2 -> PB5, R3 -> PA0.
 */
#define GRAY_GPIO_CLK        (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO)
#define GRAY_GPIO_PORT       GPIOA
#define GRAY_R3_PIN          GPIO_Pin_0
#define GRAY_R1_PIN          GPIO_Pin_1
#define GRAY_M_PIN           GPIO_Pin_2
#define GRAY_L1_PIN          GPIO_Pin_3
#define GRAY_L3_PIN          GPIO_Pin_4
#define GRAY_GPIOA_PINS      (GRAY_R3_PIN | GRAY_R1_PIN | GRAY_M_PIN | GRAY_L1_PIN | GRAY_L3_PIN)
#define GRAY_GPIOB_PORT      GPIOB
#define GRAY_R2_PIN          GPIO_Pin_5
#define GRAY_L2_PIN          GPIO_Pin_4
#define GRAY_GPIOB_PINS      (GRAY_R2_PIN | GRAY_L2_PIN)

#define GRAY_ACTIVE_LEVEL    1
#define GRAY_STEER_MAX       80.0f
#define GRAY_CROSS_ACTIVE_THRESHOLD 4
#define GRAY_DEFAULT_TURN    GRAY_TURN_RIGHT

#define GRAY_TURN_NONE       0
#define GRAY_TURN_RIGHT      1
#define GRAY_TURN_LEFT       2
#define GRAY_TURN_CROSS      3

#define GRAY_SENSOR_COUNT    7

#define GRAY_IDX_L3          0
#define GRAY_IDX_L2          1
#define GRAY_IDX_L1          2
#define GRAY_IDX_M           3
#define GRAY_IDX_R1          4
#define GRAY_IDX_R2          5
#define GRAY_IDX_R3          6

void Grayscale_Init(void);
uint16_t Grayscale_ReadState(void);
void Grayscale_Tick(void);

extern float Gray_Status[];
extern int8_t a;
extern uint16_t Gray_State;
extern uint8_t Stop_Way;
extern uint8_t Middle_Way;
extern uint8_t Right_Way;
extern uint8_t Left_Way;
extern uint8_t Start_Way;
extern float Gray_Status_Worse;

/* Gray_Sensor logical order: L3,L2,L1,M,R1,R2,R3 */
extern volatile uint8_t Gray_Sensor[GRAY_SENSOR_COUNT];
extern volatile int16_t Gray_Error;
extern volatile uint8_t Gray_ActiveCount;
extern volatile uint8_t Gray_MidActiveCount;
extern volatile uint8_t Gray_CrossFlag;
extern volatile uint8_t Gray_LostFlag;
extern volatile uint8_t Gray_TurnFlag;
extern volatile uint8_t Gray_LastTurnFlag;
extern volatile float Gray_SteerGain;

#endif

