 #ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"                  // Device header

/* TIM2 ²¿·ÖÖØÓ³Éä 1£ºCH1->PA15(ÓÒÂÖ PWMB)£¬CH2->PB3(×óÂÖ PWMA) */
#define PWM_TIMER           TIM2
#define PWM_TIMER_CLK       RCC_APB1Periph_TIM2
#define PWM_GPIO_CLK        (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO)
#define PWM_TIM2_REMAP      GPIO_PartialRemap1_TIM2
#define PWM_RIGHT_PORT      GPIOA
#define PWM_RIGHT_PIN       GPIO_Pin_15
#define PWM_LEFT_PORT       GPIOB
#define PWM_LEFT_PIN        GPIO_Pin_3
#define PWM_PERIOD          100
#define PWM_PRESCALER       72
void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);

#endif
