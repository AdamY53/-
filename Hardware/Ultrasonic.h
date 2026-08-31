#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

/* HC-SR04 wiring.
 * US1 front: TRIG PA9,  ECHO PA10
 * US2 left : TRIG PA11, ECHO PA12
 * US3 right: TRIG PC14, ECHO PC13
 *
 * ECHO is normally 5 V. Use a divider or level shifter before the STM32 pin.
 */
#define US1_TRIG_GPIO_CLK      RCC_APB2Periph_GPIOA
#define US1_ECHO_GPIO_CLK      RCC_APB2Periph_GPIOA
#define US1_TRIG_PORT          GPIOA
#define US1_ECHO_PORT          GPIOA
#define US1_TRIG_PIN           GPIO_Pin_9
#define US1_ECHO_PIN           GPIO_Pin_10

#define US2_TRIG_GPIO_CLK      RCC_APB2Periph_GPIOA
#define US2_ECHO_GPIO_CLK      RCC_APB2Periph_GPIOA
#define US2_TRIG_PORT          GPIOA
#define US2_ECHO_PORT          GPIOA
#define US2_TRIG_PIN           GPIO_Pin_11
#define US2_ECHO_PIN           GPIO_Pin_12

#define US3_TRIG_GPIO_CLK      RCC_APB2Periph_GPIOC
#define US3_ECHO_GPIO_CLK      RCC_APB2Periph_GPIOC
#define US3_TRIG_PORT          GPIOC
#define US3_ECHO_PORT          GPIOC
#define US3_TRIG_PIN           GPIO_Pin_14
#define US3_ECHO_PIN           GPIO_Pin_13

/* 40cm以内的回波约在2.3ms内返回，5ms超时可减少无回波时对主循环的阻塞。 */
#define US_TIMEOUT_US          5000UL
#define US_INVALID_DISTANCE_CM 0xFFFFU

typedef enum
{
	US_CH_FRONT = 0,
	US_CH_LEFT = 1,
	US_CH_RIGHT = 2
} UltrasonicChannel_t;

typedef enum
{
	US_STATUS_OK = 0,
	US_STATUS_WAIT_RISE_TIMEOUT = 1,
	US_STATUS_HIGH_TIMEOUT = 2,
	US_STATUS_INVALID_CHANNEL = 3,
	US_STATUS_ECHO_STUCK_HIGH = 4
} UltrasonicStatus_t;

void Ultrasonic_Init(void);
uint16_t Ultrasonic_GetDistanceCm(UltrasonicChannel_t Channel);
UltrasonicStatus_t Ultrasonic_GetStatus(UltrasonicChannel_t Channel);

#endif
