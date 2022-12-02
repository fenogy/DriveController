/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "string.h"
#include <stdlib.h>

#define DEBUG
#define CMD_DELAY 200
//#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DriveTypeDef drive1;
DriveTypeDef drive2;
UserCommandTypeDef userCmd;

/* USER CODE BEGIN PV */
uint8_t UART1_rxBuffer[2];
uint8_t UART2_rxBuffer[2];
uint8_t UART3_rxBuffer[2];
uint8_t nextCmdMaster[15];
uint8_t nextCmdSlave[15];
uint8_t rx[200];
uint8_t test = 0;
uint8_t lastChar = 0;
uint8_t userChar;
uint16_t interCmdDelay = 0;
uint8_t pollIndex = 0;

bool isStart = FALSE;
bool isPolling = FALSE;/*Need to poll drive currents and status continuously*/
bool isLogging = TRUE;/*Need to log drive communications to PC over UART3*/
bool isGraphing = TRUE;/*Need to log drive communications to PC over UART3*/
bool isSyncing = FALSE;/*Need to log drive communications to PC over UART3*/
bool isPositionCheck = FALSE;/*Need to log drive communications to PC over UART3*/
bool isMasterInit = FALSE;
bool isSlaveInit = FALSE;
bool isDelayUpdate = TRUE;
bool fetchNextMaster = TRUE;
bool fetchNextSlave = TRUE;
bool isPollMaster = TRUE;
enum { INIT = 1, RUN = 2, SYNC_RUN = 3, RE_INIT = 4,START = 5 };
uint8_t tst1[15] = "SR\r\0";
uint8_t tst2[15] = "PX=0\r\0";
uint8_t tst3[15] = "IQ\r\0";
uint8_t state = START;
uint8_t prevState = START;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
uint16_t size(char *ptr);
queue q1,q2;
uint8_t* t;
uint32_t time = 0;

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*This function handles all the sequencing */
void StateMachine(){

	/*Main States are 1.Init, 2.ReInit, 3.Run, 4.SyncRun */
	/*Different behaviors depending on isPolling, isGraphing, isPollPosition */
	switch(state){

	case START: isDelayUpdate = TRUE; 	/*At the startup it will be reside on this state */
				setDelay(200);			/*No communication will be happen on this state */

				if(isStart == TRUE){
					writeStringPC("\r\nStarting..\r\n\0");
					HAL_Delay(100);
					setState(INIT);
					ClearCurrentBuffer(&drive1);
					fetchNextMaster = TRUE;
					fetchNextSlave = TRUE;
					isStart = FALSE;
				}
				break;

	case INIT:  if(isSyncing == TRUE){ /*If two or one motor is connected slowly send init commands */

					/*Send Initializing set of commands to both motors*/
					if(isMasterInit == FALSE || isSlaveInit == FALSE){

						/*Get the init commands for master drive */
						if(fetchNextMaster == TRUE){
							getNextMasterCommand(nextCmdMaster);
							fetchNextMaster = FALSE;
						}

						/*Get the init commands for slave drive */
						if(fetchNextSlave == TRUE){
							getNextSlaveCommand(nextCmdSlave);
							fetchNextSlave = FALSE;
						}

					}else{

						/*Inits are completed, set the next state and query the current at a faster rate*/
						isPolling = TRUE;
						setState(SYNC_RUN);
						isDelayUpdate = TRUE;
						setDelay(5);
					}
				}else{

					/*Send Initializing set of commands to Drive1*/
					if(isMasterInit == FALSE){

						if(fetchNextMaster == TRUE){
							getNextMasterCommand(nextCmdMaster);
							fetchNextMaster = FALSE;
						}

					}else{

						/*Init is completed, set the next state and query the current at a faster rate*/
						isPolling = TRUE;
						isDelayUpdate = TRUE;
						setState(RUN);
						setDelay(5);
					}

				}
				break;
	case RUN: /*Keep record the currents and position with drive status*/
			  /*If SYNC command issues in the middle go to re init and init the slave and
			   * then go to the SYNC_RUN state
			   */
				/*if user Buffer is empty, get the commands from pollBuffer and record process and log results*/
				if(fetchNextMaster == TRUE){
					getNextMasterCommand(nextCmdMaster);
					fetchNextMaster = FALSE;
				}

				__NOP();
				break;

	case SYNC_RUN: /*Query current, run the sync algorithm and sync the current*/
				/*if user Buffer is empty, get the commands from pollBuffer and record process and log results*/
				if(fetchNextMaster == TRUE){
					getNextMasterCommand(nextCmdMaster);
					fetchNextMaster = FALSE;
				}

				if(fetchNextSlave == TRUE){
					getNextSlaveCommand(nextCmdSlave);
					fetchNextSlave = FALSE;
				}

				__NOP();
				break;
	case RE_INIT: /*This will happen at the request of user INIT command*/

				break;
	default: break;



	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*Use default delay value*/
  interCmdDelay = CMD_DELAY;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  /* Will not use the HAL UART libraries due to overrun issue when reading one char at a time */
  /*Start receiving one character in interrupt mode*/
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);


  InitQueue(&q1);
  InitQueue(&q2);

  drive1.isMaster = TRUE;
  drive2.isMaster = FALSE;

  InitDriveData(&drive1);
  InitDriveData(&drive2);

  CreateInitList();
  CreatePollingList();

  htim2.Init.Period = 8333;
  htim2.Instance->ARR = 8333;
  //Setup the timers
  HAL_TIM_Base_Start_IT(&htim2);

  //while(1) time++;
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);


  writeStringPC("\r\nServo Drive Controller Command Interface..\r\n\>\0");
  writeStringDrive1("\r\0");
  writeStringDrive2("\r\0");

  //HAL_Delay(100);
  //getNextMasterCommand(rx);
 // while (1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	 if(drive1.isCmdAvailable == TRUE){
		 DecodeCommand(&drive1);
		 drive1.isCmdAvailable = FALSE;
		 //if(isLogging == TRUE) void logPC(uint8_t *pStr,bool isMaster)((&drive1)->rxBuffer);
		 if(isLogging == TRUE) logPC((&drive1)->rxBuffer,TRUE);
		 ClearReceiveBuffer(&drive1);
	 }
	 if(drive2.isCmdAvailable == TRUE){
		 DecodeCommand(&drive2);
		 drive2.isCmdAvailable = FALSE;
		 //if(isLogging  == TRUE) writeStringPC((&drive2)->rxBuffer);
		 if(isLogging  == TRUE) logPC((&drive2)->rxBuffer,FALSE);
		 ClearReceiveBuffer(&drive2);
	 }
	 if(userCmd.isCmdAvailable == TRUE){
		 //DecodeCommand(&drive2);
		 userCmd.isCmdAvailable = FALSE;
		 ClearUserBuffer(&userCmd);
	 }
	 CheckReceiveBuffer(&drive1);
	 CheckReceiveBuffer(&drive2);
	 StateMachine();
	 //setDelay(100);
	 //setDelay(5);
	 //drive1.syncCurrent = -10.34568;
	 //CreateCurrentCommand(&drive1);
//	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
//	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	 //ProcessUserBuffer(&userCmd);
	 //HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, 1);
	 //HAL_UART_Receive_IT (&huart2, UART2_rxBuffer, 1);
	 //HAL_UART_Receive_IT (&huart3, UART3_rxBuffer, 1);

	  //HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, 1);
    /* USER CODE END WHILE */

		  //HAL_Delay(100);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  //htim2.Init.Prescaler = 10;
  htim2.Init.Prescaler = CMD_DELAY;    //Number of ms
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8333;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

uint8_t UART1_GetChar (void)
{
		/*********** STEPS FOLLOWED *************

	1. Wait for the RXNE bit to set. It indicates that the data has been received and can be read.
	2. Read the data from USART_DR  Register. This also clears the RXNE bit

	****************************************/
	uint8_t Temp;
	uint32_t cnt;
	while ((!(USART1->SR & (1<<5))) && (cnt < 100000) )cnt++;
	//while (!(USART1->SR & (1<<5)) ){
		// Wait for RXNE to SET.. This indicates that the data has been Received

	Temp = USART1->DR;  // Read the data.
	return Temp;
}

uint8_t UART2_GetChar (void)
{
		/*********** STEPS FOLLOWED *************

	1. Wait for the RXNE bit to set. It indicates that the data has been received and can be read.
	2. Read the data from USART_DR  Register. This also clears the RXNE bit

	****************************************/
	uint8_t Temp;
	uint32_t cnt;
	while ((!(USART2->SR & (1<<5))) && (cnt < 100000) )cnt++;
	//while (!(USART2->SR & (1<<5)));  // Wait for RXNE to SET.. This indicates that the data has been Received
	Temp = USART2->DR;  // Read the data.
	return Temp;
}

uint8_t UART3_GetChar (void)
{
		/*********** STEPS FOLLOWED *************

	1. Wait for the RXNE bit to set. It indicates that the data has been received and can be read.
	2. Read the data from USART_DR  Register. This also clears the RXNE bit

	****************************************/
	uint8_t Temp;
	uint32_t cnt;
	while ((!(USART3->SR & (1<<5))) && (cnt < 100000) ) cnt++;
	//while (!(USART3->SR & (1<<5)));  // Wait for RXNE to SET.. This indicates that the data has been Received
	Temp = USART3->DR;  // Read the data.
	return Temp;
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void writeStringDrive1(uint8_t *pStr){

	//uint8_t s = size(pStr);
	if(*pStr == 0) return;
	HAL_UART_Transmit(&huart1,pStr,size(pStr),10000);


}
void writeStringDrive2(uint8_t *pStr){

	if(*pStr == 0) return;
	//HAL_UART_Transmit(&huart2,pStr,size(pStr),10000);
	HAL_UART_Transmit(&huart2,pStr,size(pStr),10000);

}
void writeStringPC(uint8_t *pStr){

	if(*pStr == 0) return;
	HAL_UART_Transmit(&huart3,pStr,size(pStr),10);
	//Non-Blocking
	//HAL_UART_Transmit_IT(&huart3,pStr,size(pStr));

}

void logPC(uint8_t *pStr,bool isMaster){

	/*Will first remove the CR*/
	//uint8_t size = size(pStr);
	uint8_t *modifiedStr;

	if(isMaster == TRUE)
		writeStringPC("M,\0");
	else
		writeStringPC("S,\0");

	writeStringPC(pStr);

//	for(int i; i<size; i++){
//		if(*(pStr + i) == '\n'
//		*()
//	}
}

void putCharPC(uint8_t chr){

	uint8_t str[2] = {0,0};
	str[0] = chr;
	HAL_UART_Transmit(&huart3,str,1,10);
	//Non-Blocking
	//HAL_UART_Transmit_IT(&huart3,pStr,size(pStr));

}
uint16_t size(char *ptr)
{
    //variable used to access the subsequent array elements.
    int offset = 0;
    //variable that counts the number of elements in your array
    int count = 0;

    //While loop that tests whether the end of the array has been reached
    while (*(ptr + offset) != '\0')
    {
        //increment the count variable
        ++count;
        //advance to the next element of the array
        ++offset;
    }
    //return the size of the array
    return count;
}

void setState(uint8_t st){

	prevState = state;
	state = st;
}

void clearNextCommandMaster(){
	memset(nextCmdMaster,0,15);
}

void clearNextCommandSlave(){
	memset(nextCmdSlave,0,15);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
