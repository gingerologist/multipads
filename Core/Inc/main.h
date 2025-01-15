/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// TODO many weird declaration
void print_line(const char* str);
void update_switch(int port_index, int pin_index, GPIO_PinState c_state, GPIO_PinState s_state);
void update_all_switches(uint32_t config[4]);
int detect_single_keydown(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW0_Pin GPIO_PIN_4
#define SW0_GPIO_Port GPIOA
#define SW9_Pin GPIO_PIN_5
#define SW9_GPIO_Port GPIOA
#define SW8_Pin GPIO_PIN_6
#define SW8_GPIO_Port GPIOA
#define SW7_Pin GPIO_PIN_7
#define SW7_GPIO_Port GPIOA
#define SW6_Pin GPIO_PIN_4
#define SW6_GPIO_Port GPIOC
#define SW5_Pin GPIO_PIN_5
#define SW5_GPIO_Port GPIOC
#define SW4_Pin GPIO_PIN_0
#define SW4_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_11
#define SW2_GPIO_Port GPIOF
#define SW1_Pin GPIO_PIN_12
#define SW1_GPIO_Port GPIOF
#define D9S_Pin GPIO_PIN_13
#define D9S_GPIO_Port GPIOF
#define D9C_Pin GPIO_PIN_14
#define D9C_GPIO_Port GPIOF
#define D8S_Pin GPIO_PIN_15
#define D8S_GPIO_Port GPIOF
#define D8C_Pin GPIO_PIN_0
#define D8C_GPIO_Port GPIOG
#define D7S_Pin GPIO_PIN_1
#define D7S_GPIO_Port GPIOG
#define D7C_Pin GPIO_PIN_7
#define D7C_GPIO_Port GPIOE
#define D6S_Pin GPIO_PIN_8
#define D6S_GPIO_Port GPIOE
#define D6C_Pin GPIO_PIN_9
#define D6C_GPIO_Port GPIOE
#define D5S_Pin GPIO_PIN_10
#define D5S_GPIO_Port GPIOE
#define D5C_Pin GPIO_PIN_11
#define D5C_GPIO_Port GPIOE
#define D4S_Pin GPIO_PIN_12
#define D4S_GPIO_Port GPIOE
#define D4C_Pin GPIO_PIN_13
#define D4C_GPIO_Port GPIOE
#define D3S_Pin GPIO_PIN_14
#define D3S_GPIO_Port GPIOE
#define D3C_Pin GPIO_PIN_15
#define D3C_GPIO_Port GPIOE
#define D2S_Pin GPIO_PIN_10
#define D2S_GPIO_Port GPIOB
#define D2C_Pin GPIO_PIN_11
#define D2C_GPIO_Port GPIOB
#define D1S_Pin GPIO_PIN_12
#define D1S_GPIO_Port GPIOB
#define D1C_Pin GPIO_PIN_13
#define D1C_GPIO_Port GPIOB
#define C9S_Pin GPIO_PIN_14
#define C9S_GPIO_Port GPIOB
#define C9C_Pin GPIO_PIN_15
#define C9C_GPIO_Port GPIOB
#define C8S_Pin GPIO_PIN_8
#define C8S_GPIO_Port GPIOD
#define C8C_Pin GPIO_PIN_9
#define C8C_GPIO_Port GPIOD
#define C7S_Pin GPIO_PIN_10
#define C7S_GPIO_Port GPIOD
#define C7C_Pin GPIO_PIN_11
#define C7C_GPIO_Port GPIOD
#define C6S_Pin GPIO_PIN_12
#define C6S_GPIO_Port GPIOD
#define C6C_Pin GPIO_PIN_13
#define C6C_GPIO_Port GPIOD
#define C5S_Pin GPIO_PIN_14
#define C5S_GPIO_Port GPIOD
#define C5C_Pin GPIO_PIN_15
#define C5C_GPIO_Port GPIOD
#define C4S_Pin GPIO_PIN_2
#define C4S_GPIO_Port GPIOG
#define C4C_Pin GPIO_PIN_3
#define C4C_GPIO_Port GPIOG
#define C3S_Pin GPIO_PIN_4
#define C3S_GPIO_Port GPIOG
#define C3C_Pin GPIO_PIN_5
#define C3C_GPIO_Port GPIOG
#define C2S_Pin GPIO_PIN_6
#define C2S_GPIO_Port GPIOG
#define C2C_Pin GPIO_PIN_7
#define C2C_GPIO_Port GPIOG
#define C1S_Pin GPIO_PIN_8
#define C1S_GPIO_Port GPIOG
#define C1C_Pin GPIO_PIN_6
#define C1C_GPIO_Port GPIOC
#define B9S_Pin GPIO_PIN_7
#define B9S_GPIO_Port GPIOC
#define B9C_Pin GPIO_PIN_8
#define B9C_GPIO_Port GPIOC
#define B8S_Pin GPIO_PIN_9
#define B8S_GPIO_Port GPIOC
#define B8C_Pin GPIO_PIN_8
#define B8C_GPIO_Port GPIOA
#define B7S_Pin GPIO_PIN_9
#define B7S_GPIO_Port GPIOA
#define B7C_Pin GPIO_PIN_10
#define B7C_GPIO_Port GPIOA
#define B6S_Pin GPIO_PIN_11
#define B6S_GPIO_Port GPIOA
#define B6C_Pin GPIO_PIN_12
#define B6C_GPIO_Port GPIOA
#define B5S_Pin GPIO_PIN_15
#define B5S_GPIO_Port GPIOA
#define B5C_Pin GPIO_PIN_10
#define B5C_GPIO_Port GPIOC
#define B4S_Pin GPIO_PIN_11
#define B4S_GPIO_Port GPIOC
#define B4C_Pin GPIO_PIN_12
#define B4C_GPIO_Port GPIOC
#define B3S_Pin GPIO_PIN_0
#define B3S_GPIO_Port GPIOD
#define B3C_Pin GPIO_PIN_1
#define B3C_GPIO_Port GPIOD
#define B2S_Pin GPIO_PIN_2
#define B2S_GPIO_Port GPIOD
#define B2C_Pin GPIO_PIN_3
#define B2C_GPIO_Port GPIOD
#define B1S_Pin GPIO_PIN_4
#define B1S_GPIO_Port GPIOD
#define B1C_Pin GPIO_PIN_5
#define B1C_GPIO_Port GPIOD
#define A9S_Pin GPIO_PIN_6
#define A9S_GPIO_Port GPIOD
#define A9C_Pin GPIO_PIN_7
#define A9C_GPIO_Port GPIOD
#define A8S_Pin GPIO_PIN_9
#define A8S_GPIO_Port GPIOG
#define A8C_Pin GPIO_PIN_10
#define A8C_GPIO_Port GPIOG
#define A7S_Pin GPIO_PIN_11
#define A7S_GPIO_Port GPIOG
#define A7C_Pin GPIO_PIN_12
#define A7C_GPIO_Port GPIOG
#define A6S_Pin GPIO_PIN_13
#define A6S_GPIO_Port GPIOG
#define A6C_Pin GPIO_PIN_14
#define A6C_GPIO_Port GPIOG
#define A5S_Pin GPIO_PIN_15
#define A5S_GPIO_Port GPIOG
#define A5C_Pin GPIO_PIN_3
#define A5C_GPIO_Port GPIOB
#define A4S_Pin GPIO_PIN_4
#define A4S_GPIO_Port GPIOB
#define A4C_Pin GPIO_PIN_5
#define A4C_GPIO_Port GPIOB
#define A3S_Pin GPIO_PIN_6
#define A3S_GPIO_Port GPIOB
#define A3C_Pin GPIO_PIN_7
#define A3C_GPIO_Port GPIOB
#define A2S_Pin GPIO_PIN_8
#define A2S_GPIO_Port GPIOB
#define A2C_Pin GPIO_PIN_9
#define A2C_GPIO_Port GPIOB
#define A1S_Pin GPIO_PIN_0
#define A1S_GPIO_Port GPIOE
#define A1C_Pin GPIO_PIN_1
#define A1C_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
