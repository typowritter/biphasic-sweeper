/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32h7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AD9854_A0_Pin GPIO_PIN_5
#define AD9854_A0_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_9
#define SPI1_MISO_GPIO_Port GPIOG
#define ADS124S_RST_Pin GPIO_PIN_5
#define ADS124S_RST_GPIO_Port GPIOD
#define ADS124S_SYNC_Pin GPIO_PIN_4
#define ADS124S_SYNC_GPIO_Port GPIOD
#define AD9854_A1_Pin GPIO_PIN_6
#define AD9854_A1_GPIO_Port GPIOB
#define ADS124S_CS_Pin GPIO_PIN_6
#define ADS124S_CS_GPIO_Port GPIOD
#define ADS124S_DRDY_Pin GPIO_PIN_3
#define ADS124S_DRDY_GPIO_Port GPIOD
#define ADS124S_DRDY_EXTI_IRQn EXTI3_IRQn
#define AD9854_A2_Pin GPIO_PIN_7
#define AD9854_A2_GPIO_Port GPIOB
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define AD9854_RST_Pin GPIO_PIN_12
#define AD9854_RST_GPIO_Port GPIOG
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOD
#define AD9854_A4_Pin GPIO_PIN_9
#define AD9854_A4_GPIO_Port GPIOB
#define AD9854_A3_Pin GPIO_PIN_8
#define AD9854_A3_GPIO_Port GPIOB
#define AD9854_WR_Pin GPIO_PIN_13
#define AD9854_WR_GPIO_Port GPIOG
#define DCR_SWITCH_Pin GPIO_PIN_2
#define DCR_SWITCH_GPIO_Port GPIOD
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define AD9854_D7_Pin GPIO_PIN_8
#define AD9854_D7_GPIO_Port GPIOC
#define AD9854_D6_Pin GPIO_PIN_7
#define AD9854_D6_GPIO_Port GPIOC
#define AD9854_D5_Pin GPIO_PIN_6
#define AD9854_D5_GPIO_Port GPIOC
#define AD9854_D0_Pin GPIO_PIN_1
#define AD9854_D0_GPIO_Port GPIOC
#define AD9854_D1_Pin GPIO_PIN_2
#define AD9854_D1_GPIO_Port GPIOC
#define AD9854_D2_Pin GPIO_PIN_3
#define AD9854_D2_GPIO_Port GPIOC
#define AD9854_FSK_Pin GPIO_PIN_2
#define AD9854_FSK_GPIO_Port GPIOA
#define AD9854_A5_Pin GPIO_PIN_10
#define AD9854_A5_GPIO_Port GPIOB
#define AD9854_OSK_Pin GPIO_PIN_11
#define AD9854_OSK_GPIO_Port GPIOB
#define AD9854_UDCL_Pin GPIO_PIN_7
#define AD9854_UDCL_GPIO_Port GPIOA
#define AD9854_D3_Pin GPIO_PIN_4
#define AD9854_D3_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOB
#define TFT_RX_Pin GPIO_PIN_12
#define TFT_RX_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_3
#define LED_B_GPIO_Port GPIOA
#define AD9854_D4_Pin GPIO_PIN_5
#define AD9854_D4_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOB
#define TFT_TX_Pin GPIO_PIN_13
#define TFT_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
