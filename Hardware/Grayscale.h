#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "stm32f10x.h"

/* Seven-channel line sensor, logical order: L3,L2,L1,M,R1,R2,R3.
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
#define GRAY_SENSOR_COUNT    7
/* 可调窗口：中间三路循迹信号使用的多数滤波采样数，保持为奇数；3表示最近3次采样至少2次为高才输出高。 */
#define GRAY_MIDDLE_FILTER_WINDOW 3

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

extern volatile uint16_t Gray_State;
extern volatile uint8_t Gray_Sensor[GRAY_SENSOR_COUNT];
/* 循迹专用信号：外侧四路保持原始值，中间 L1/M/R1 使用多数滤波值。 */
extern volatile uint8_t Gray_LineSensor[GRAY_SENSOR_COUNT];
extern volatile uint8_t Gray_ActiveCount;

#endif
