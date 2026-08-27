#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"                  // Device header

/* TB6612 ½Ó¿Ú£ºÓÒÂÖ BIN/PWMB£¬×óÂÖ AIN/PWMA */
#define MOTOR_GPIO_CLK      RCC_APB2Periph_GPIOB
#define MOTOR_GPIO_PORT     GPIOB

#define MOTOR_STBY_PIN      GPIO_Pin_0
#define MOTOR_LEFT_IN1_PIN  GPIO_Pin_1
#define MOTOR_LEFT_IN2_PIN  GPIO_Pin_15
#define MOTOR_RIGHT_IN1_PIN GPIO_Pin_13
#define MOTOR_RIGHT_IN2_PIN GPIO_Pin_14

#define MOTOR_RIGHT         0
#define MOTOR_LEFT          1
#define MOTOR_SWAP_LEFT_RIGHT 1

void Motor_Init(void);
void Motor_SetPWM(uint8_t m, int8_t PWM);
void Motor_Stop(void);
void Motor_FL(void);

#endif
