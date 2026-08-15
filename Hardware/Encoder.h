#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"                  // Device header

/* E1A/E1B -> PA6/PA7, TIM3, ÓÒÂÖ±àÂëÆ÷ */
#define ENCODER1_TIMER          TIM3
#define ENCODER1_TIMER_CLK      RCC_APB1Periph_TIM3
#define ENCODER1_GPIO_CLK       RCC_APB2Periph_GPIOA
#define ENCODER1_GPIO_PORT      GPIOA
#define ENCODER1_A_PIN          GPIO_Pin_6
#define ENCODER1_B_PIN          GPIO_Pin_7

/* E2A/E2B -> PB6/PB7, TIM4, ×óÂÖ±àÂëÆ÷ */
#define ENCODER2_TIMER          TIM4
#define ENCODER2_TIMER_CLK      RCC_APB1Periph_TIM4
#define ENCODER2_GPIO_CLK       RCC_APB2Periph_GPIOB
#define ENCODER2_GPIO_PORT      GPIOB
#define ENCODER2_A_PIN          GPIO_Pin_6
#define ENCODER2_B_PIN          GPIO_Pin_7

/* Direction correction: forward wheel speed must be positive on OLED. */
#define ENCODER1_DIR            (-1)
#define ENCODER2_DIR            (1)

extern volatile int16_t Encoder1_RawCount;
extern volatile int16_t Encoder2_RawCount;
extern volatile uint8_t Encoder1_PinA;
extern volatile uint8_t Encoder1_PinB;
extern volatile uint8_t Encoder2_PinA;
extern volatile uint8_t Encoder2_PinB;
void Encoder_Init(void);
int16_t Encoder1_Get(void);
int16_t Encoder2_Get(void);

#endif
