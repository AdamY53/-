#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/* SG90 signal -> PA8 / TIM1_CH1.
 * SG90 power should use an external 5 V supply. Connect GND with STM32 GND.
 */
#define SERVO_TIMER           TIM1
#define SERVO_TIMER_CLK       RCC_APB2Periph_TIM1
#define SERVO_GPIO_CLK        RCC_APB2Periph_GPIOA
#define SERVO_GPIO_PORT       GPIOA
#define SERVO_GPIO_PIN        GPIO_Pin_8

#define SERVO_PERIOD_US       20000
#define SERVO_MIN_PULSE_US    500
#define SERVO_MID_PULSE_US    1500
#define SERVO_MAX_PULSE_US    2500

void Servo_Init(void);
void Servo_SetAngle(uint8_t Angle);

#endif
