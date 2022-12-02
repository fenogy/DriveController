/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern uint8_t UART1_rxBuffer[2];
extern uint8_t UART2_rxBuffer[2];
extern uint8_t UART3_rxBuffer[2];
extern nextCmdMaster[15];
extern nextCmdSlave[15];
extern DriveTypeDef drive1;
extern DriveTypeDef drive2;
extern UserCommandTypeDef userCmd;
int16_t size(char *ptr);
uint8_t ii;
//uint8_t rx1[400];
//uint8_t rxIndex = 0;
extern uint8_t lastChar;
extern uint8_t userChar;
extern bool isLogging;
extern bool isMasterInit;
extern uint8_t pollIndex;
extern uint8_t fetchNextMaster;
extern uint8_t PollingList[4][10];
extern uint8_t MasterQueue[5][20];
extern uint8_t SlaveQueue[5][20];
extern uint8_t masterInitCmdIndex;
extern uint8_t slaveInitCmdIndex;
extern uint8_t MasterInitList[12][15];
extern uint8_t SlaveInitList[12][15];
extern uint8_t state;
extern enum { INIT = 1, RUN = 2, SYNC_RUN = 3, RE_INIT = 4,START = 5 };
extern queue q1,q2;

extern uint32_t time;
extern bool isPollMaster;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
//	uint8_t One[10][8] = {"IQ\r","SR\r","UM=1\r","TC\r","PX\r","UM\r","SP\r","AC\r","DC\r","PR\r"};
//	uint8_t Two[] = "SR\r";
//	uint8_t Three[] = "3\n";
	//uint8_t cmdStr[20];
	ClearReceiveBuffer(&drive1);

	if(state != START){

		/*Alternate between master and slave polling*/
		if(isPollMaster == TRUE){
			writeStringDrive1(nextCmdMaster);
			clearNextCommandMaster();
			fetchNextMaster = TRUE;
			isPollMaster = FALSE;
		}else{
			writeStringPC(nextCmdSlave);
			writeStringDrive2(nextCmdSlave);
			clearNextCommandSlave();
			fetchNextSlave = TRUE;
			isPollMaster = TRUE;
		}
	}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}
/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
//	uint8_t One[] = "6";
//	uint8_t Two[] = "5";
//	uint8_t Three[] = "7";
//	writeStringDrive1(One);
//	writeStringDrive2(Two);
//	writeStringPC(Three);
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}
/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
//	uint8_t One[] = "4";
//	uint8_t Two[] = "8";
//	uint8_t Three[] = "9";
//	writeStringDrive1(One);
//	writeStringDrive2(Two);
//	writeStringPC(Three);
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

	  lastChar = UART1_GetChar();
	  drive1.rxBuffer[drive1.rxBufPtr++] = lastChar;
//	  uint8_t x = 0;
//	  if(x != 0){
//		  rx1[rxIndex++] = x;
//
//		if(rxIndex == 200){
//			__NOP();
//		}
//	  }
	  return;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);


  //HAL_UART_Transmit(&huart3,UART1_rxBuffer[0],1,1);

  //HAL_UART_Transmit(&huart1,UART1_rxBuffer[0],1,1);
  //HAL_UART_Transmit(&huart1,UART1_rxBuffer,sizeof(UART1_rxBuffer),10);

//  if(UART1_rxBuffer[*(huart1.pRxBuffPtr)] == ';'){
//	  *(huart1.pRxBuffPtr) = 0;
//	  HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, 30);
//  }
//  uint8_t i;
//  for(i = 0; i < 30; ++i)
//  {
//	  UART1_rxBuffer[i] = 0;
//  }
  //memset(UART1_rxBuffer, 0, sizeof(UART1_rxBuffer) * (sizeof UART1_rxBuffer[0]) );
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	  lastChar = UART2_GetChar();
	  drive2.rxBuffer[drive2.rxBufPtr++] = lastChar;
	  return;
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	userChar = UART3_GetChar();
	userCmd.rxBuffer[userCmd.rxBufPtr++] = userChar;
	/*Echo on, if in console mode*/
	//if(isLogging == FALSE){
	if(isPolling == FALSE){
		//writeStringPC(&userChar);
		putCharPC(userChar);
	}

	if(userChar == '\r'){
		userCmd.isCmdAvailable = TRUE;
		ProcessUserBuffer(&userCmd);
		userChar = 0;
	}

	return;
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);

  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
