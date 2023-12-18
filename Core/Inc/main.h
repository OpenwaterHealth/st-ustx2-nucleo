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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define BUS0_bit0_Pin GPIO_PIN_0
#define BUS0_bit0_GPIO_Port GPIOC
#define BUS0_bit1_Pin GPIO_PIN_1
#define BUS0_bit1_GPIO_Port GPIOC
#define BUS0_bit2_Pin GPIO_PIN_2
#define BUS0_bit2_GPIO_Port GPIOC
#define BUS0_bit3_Pin GPIO_PIN_3
#define BUS0_bit3_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BUS0_bit10_Pin GPIO_PIN_4
#define BUS0_bit10_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BUS0_spi1_miso_Pin GPIO_PIN_6
#define BUS0_spi1_miso_GPIO_Port GPIOA
#define BUS0_spi1_mosi_Pin GPIO_PIN_7
#define BUS0_spi1_mosi_GPIO_Port GPIOA
#define BUS0_bit4_Pin GPIO_PIN_4
#define BUS0_bit4_GPIO_Port GPIOC
#define BUS0_bit5_Pin GPIO_PIN_5
#define BUS0_bit5_GPIO_Port GPIOC
#define BUS0_i2c2_scl_Pin GPIO_PIN_10
#define BUS0_i2c2_scl_GPIO_Port GPIOB
#define BUS0_cs0_Pin GPIO_PIN_12
#define BUS0_cs0_GPIO_Port GPIOB
#define BUS0_cs1_Pin GPIO_PIN_13
#define BUS0_cs1_GPIO_Port GPIOB
#define BUS0_cs2_Pin GPIO_PIN_14
#define BUS0_cs2_GPIO_Port GPIOB
#define BUS0_cs3_Pin GPIO_PIN_15
#define BUS0_cs3_GPIO_Port GPIOB
#define BUS0_bit6_Pin GPIO_PIN_6
#define BUS0_bit6_GPIO_Port GPIOC
#define BUS0_bit7_Pin GPIO_PIN_7
#define BUS0_bit7_GPIO_Port GPIOC
#define BUS0_bit8_Pin GPIO_PIN_8
#define BUS0_bit8_GPIO_Port GPIOC
#define BUS0_bit9_Pin GPIO_PIN_9
#define BUS0_bit9_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BUS0_uart3_tx_Pin GPIO_PIN_10
#define BUS0_uart3_tx_GPIO_Port GPIOC
#define BUS0_uart3_rx_Pin GPIO_PIN_11
#define BUS0_uart3_rx_GPIO_Port GPIOC
#define BUS0_i2c2_sda_Pin GPIO_PIN_12
#define BUS0_i2c2_sda_GPIO_Port GPIOC
#define BUS0_spi1_ck_Pin GPIO_PIN_3
#define BUS0_spi1_ck_GPIO_Port GPIOB
#define SW_TRIGGER_Pin GPIO_PIN_4
#define SW_TRIGGER_GPIO_Port GPIOB
#define BUS0_timch1_Pin GPIO_PIN_6
#define BUS0_timch1_GPIO_Port GPIOB
#define BUS0_timch2_Pin GPIO_PIN_7
#define BUS0_timch2_GPIO_Port GPIOB
#define BUS0_timch3_Pin GPIO_PIN_8
#define BUS0_timch3_GPIO_Port GPIOB
#define BUS0_timch4_Pin GPIO_PIN_9
#define BUS0_timch4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
