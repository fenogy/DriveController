/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

//extern bool fetchNextMaster;
//extern bool fetchNextSlave;
//extern bool isMasterInit;
//extern bool isSlaveInit;
//extern bool isDelayUpdate;

#include <user_command_decoder.h>
#include <state_machine.h>
#include "stm32f1xx_hal.h"
#include "command_decoder.h"
#include "command_queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

//DAC_HandleTypeDef hdac;
//
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;
//
//UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart2;
//UART_HandleTypeDef huart3;
//
///* USER CODE BEGIN PV */
//uint8_t UART1_rxBuffer[2] = {0};
//uint8_t UART2_rxBuffer[2] = {0};
//uint8_t UART3_rxBuffer[2] = {0};
//extern bool fetchNextMaster;
//extern bool fetchNextSlave;
//extern bool isMasterInit;
//extern bool isSlaveInit;
//extern bool isDelayUpdate;
//extern bool isDelayUpdate;
//extern bool fetchNextMaster;
//extern bool fetchNextSlave;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void writeStringDrive1(uint8_t *pStr);
void writeStringDrive2(uint8_t *pStr);
void writeStringPC(uint8_t *pStr);
//void logPC(uint8_t *pStr,bool isMaster);
void clearNextCommandMaster();
void clearNextCommandSlave();
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
