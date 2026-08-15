#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

/* K1/K2/K3 -> PB10/PB11/PB12, active low. */
#define KEY_GPIO_CLK        RCC_APB2Periph_GPIOB
#define KEY_GPIO_PORT       GPIOB
#define KEY1_PIN            GPIO_Pin_10
#define KEY2_PIN            GPIO_Pin_11
#define KEY3_PIN            GPIO_Pin_12

#define KEY_NUM_K1          1
#define KEY_NUM_K2          2
#define KEY_NUM_K3          3
#define KEY_LONG_FLAG       0x80
#define KEY_LONG_K1         (KEY_LONG_FLAG | KEY_NUM_K1)
#define KEY_LONG_K2         (KEY_LONG_FLAG | KEY_NUM_K2)
#define KEY_LONG_K3         (KEY_LONG_FLAG | KEY_NUM_K3)

void Key_Init(void);
uint8_t Key_GetNum(void);
uint8_t Key_GetLongNum(void);
uint8_t Key_GetState(void);
void Key_Tick(void);

#endif
