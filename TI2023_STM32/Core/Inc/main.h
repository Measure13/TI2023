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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ADC_DATA_NUM 1024
#define CLK_FREQ 84000000
#define LENGTH 300
#define WIDTH 300
#define M_MARGIN 60
#define HALF_SQUARE (LENGTH / 2 + M_MARGIN)
#define V_BOARD 3000
#define UARTHMI_LATTICE 12
#define LATTICE_6_UNIT 50
#define LATTICE_12_UNIT 25
#define LATTICE_6_SQUARE_UH 36
#define LATTICE_12_SQUARE_UH 18
#define DEFAULT_DDS_FREQ 16000
#define DEFAULT_SAMPLE_RATE 160000
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint16_t adc_values[ADC_DATA_NUM + 4];
extern bool adc_values_sqr[ADC_DATA_NUM];
extern uint32_t quadrant_time_stamp[4];
extern int flag;
extern volatile bool interrupt_dis[4];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Q1_Pin GPIO_PIN_1
#define Q1_GPIO_Port GPIOF
#define Q1_EXTI_IRQn EXTI1_IRQn
#define Q2_Pin GPIO_PIN_2
#define Q2_GPIO_Port GPIOF
#define Q2_EXTI_IRQn EXTI2_IRQn
#define Q3_Pin GPIO_PIN_3
#define Q3_GPIO_Port GPIOF
#define Q3_EXTI_IRQn EXTI3_IRQn
#define Q4_Pin GPIO_PIN_4
#define Q4_GPIO_Port GPIOF
#define Q4_EXTI_IRQn EXTI4_IRQn
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_FSYNC_Pin GPIO_PIN_6
#define SPI1_FSYNC_GPIO_Port GPIOA
#define SPI1_DAT_Pin GPIO_PIN_7
#define SPI1_DAT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define Q1 0
#define Q2 1
#define Q3 2
#define Q4 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
