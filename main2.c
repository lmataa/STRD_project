/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "dwt_stm32_delay.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
SPI_HandleTypeDef hspi1;
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
uint32_t TxMailbox; 
uint8_t ByteSent = 0; //declare byte to be transmitted //declare a receive byte
uint8_t ByteReceived = 0; //declare a receive byte
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

osThreadId Tarea1Handle;
osMutexId mutex1Handle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
uint32_t DWT_Delay_Init(void);
uint32_t readDistance(void);

/* Variables para calcular la inclinacion en funcion de los valores leidos en los tres ejes del acelerometro */
int Ix, Iy, Iz;
uint8_t Ix1, Ix2;
uint8_t Iy1, Iy2;
uint8_t Iz1, Iz2;
double X, Y, Z;
double rotX, rotY;

/*funcion para las lecturas de los registros del acelerometro */
uint8_t spiTxBuf[2], spiRxBuf[2];
uint8_t SPI_Read (uint8_t address);

/* funcion para inicializar el acelerometro */
void Inicializa_Acelerometro ();

/* Tareas perodicas */
void inclinacionCabeza(void const * argument);
void deteccionVolantazos(void const * argument);
void relaxVolante(void const * argument);
void riesgos(void const * argument);
void deteccionPulsador(void const * argument);

/* Variables para depuracion */
int ContTarea1 = 0;
int ContTarea2 = 0;
int ContTarea3 = 0;

/*Variables Compartidas*/
int sintomas1 = 0; //inclinacion
int rotacionX = 0;
int rotacionY = 0;
int sintomas2 = 0; // Agarradfo
int sintomas3 = 0;// volantazos
int modo = 1;
int p = 0;
int l = 0;
int b = 0;
int r = 0;
float vel = 0.0;
int diff = 0;

/* Declaraciones para tranmisiones en CAN BUS ------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
uint32_t TxMailbox; 
//uint8_t ByteSent = 0; //declare byte to be transmitted //declare a receive byte
//uint8_t ByteReceived = 0; //declare a receive byte
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

/* mutex */
SemaphoreHandle_t sem_sintomas1 = NULL;
SemaphoreHandle_t sem_sintomas2 = NULL;
SemaphoreHandle_t sem_sintomas3 = NULL;
SemaphoreHandle_t sem_modo = NULL;
SemaphoreHandle_t sem_int_pulsador = NULL;

/*Prioridades de las Tareas Periodicas*/
#define PR_TAREA1 4 // incCabeza
#define PR_TAREA2 5 // detVol
#define PR_TAREA3 1 // relaxVol
#define PR_TAREA4 2 // risgos
#define PR_TAREA5 3 // detPulsador

/*Periodos de las tareas*/
#define T_TAREA1 600 // incCabeza
#define T_TAREA2 400 // detVol
#define T_TAREA3 500 // relaxVol
#define T_TAREA4 300 // risgos
#define T_TAREA5 350 // detPulsador
void InicializaTransmisionesCAN();
#define TRUE 1
#define FALSE 0

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/
	InicializaTransmisionesCAN();
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
	
	Inicializa_Acelerometro();
	
  /* Create the mutex(es) */
  /* definition and creation of mutex1 */
  sem_sintomas1 = xSemaphoreCreateMutex();
	sem_sintomas2 = xSemaphoreCreateMutex();
	sem_sintomas3 = xSemaphoreCreateMutex();
	sem_modo = xSemaphoreCreateMutex();
	
	sem_int_pulsador = xSemaphoreCreateBinary();

  /* Create the thread(s) */
  /* definition and creation of Tarea1 */
	xTaskCreate (inclinacionCabeza, (const char *)"LaOtra", configMINIMAL_STACK_SIZE, NULL, PR_TAREA1, NULL);
	xTaskCreate (deteccionVolantazos, (const char *)"LaOtra2", configMINIMAL_STACK_SIZE, NULL, PR_TAREA2, NULL);
	xTaskCreate (relaxVolante, (const char *)"LaOtra3", configMINIMAL_STACK_SIZE, NULL, PR_TAREA3, NULL);
	xTaskCreate (riesgos, (const char *)"LaOtra4", configMINIMAL_STACK_SIZE, NULL, PR_TAREA4, NULL);
	xTaskCreate (deteccionPulsador, (const char *)"LaOtra5", configMINIMAL_STACK_SIZE, NULL, PR_TAREA5, NULL);
	vTaskStartScheduler (); // arranca el planificador 


  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/*Configure GPIO pins : PB0 PB3: Interrupcion botones externos */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|
	                      GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}




void InicializaTransmisionesCAN() {

	pHeader.DLC=1; //give message size of 1 byte
	pHeader.IDE=CAN_ID_STD; //set identifier to standard
	pHeader.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
	pHeader.StdId=0x2FF; //define a standard identifier, used for message identification by filters (##switch this for the other microcontroller##)
	
	//filter one (stack light blink)
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x2F4<<5; //the ID that the filter looks for (##switch this for the other microcontroller##)
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); //configure CAN filter

	HAL_CAN_Start(&hcan1); //start CAN
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
};

/* USER CODE BEGIN Header_StartTarea1 */
/**
  * @brief  Function implementing the Tarea1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTarea1 */
void inclinacionCabeza(void const * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int volante;
	double antesX, antesY = 0;
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  /* Infinite loop */
  for(;;)
  {
		ContTarea1 ++;
		 /* Calculo de la trotacion en el eje X e Y, dentro de la tarea que controla la inclinaci?n de la cabeza */

		Ix1 = SPI_Read (0x28);
		Ix2 = SPI_Read (0x29);
		Ix = (Ix2 << 8) + Ix1;
		if (Ix >= 0x8000) Ix = -(65536 - Ix);
		X = Ix/16384.0;
			
		Iy1 = SPI_Read (0x2A);
		Iy2 = SPI_Read (0x2B);
		Iy = (Iy2 << 8) + Iy1;
		if (Iy >= 0x8000) Iy = -(65536 - Iy);
		Y = Iy/16384.0;
			
		Iz1 = SPI_Read (0x2C);
		Iz2 = SPI_Read (0x2D);
		Iz = (Iz2 << 8) + Iz1;
		if (Iz >= 0x8000) Iz = -(65536 - Iz);
		Z = Iz/16384.0;
			
		rotX = atan2(Y, sqrt(X*X+Z*Z)) * 180.0/3.1416;
		rotY = - atan2(X, sqrt(Y*Y+Z*Z)) * 180.0/3.1416;
		
		if( sem_sintomas1 != NULL )
			{
				if( xSemaphoreGive( sem_sintomas1 ) != pdTRUE )
				{ 
				}
				if( xSemaphoreTake(sem_sintomas1, ( TickType_t ) 0 ) )
				{
					rotacionX = rotX;
					rotacionY = rotY;
					if((rotY>0.3||rotY<-0.3)&&(antesY>0.3||antesY<-0.3)){
						sintomas1 = 1;
					} else {
						sintomas1 = 0;
					}
					if((rotX>0.3||rotX<-0.3)&&(antesX>0.3||antesX<-0.3)){
						sintomas1 = 1;
					} else {
						sintomas1 = 0;
					}
					if( xSemaphoreGive( sem_sintomas1 ) != pdTRUE )
					{
					}
				}
			}
		antesX = rotX;
		antesY = rotY;
		
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( T_TAREA1 )); 
  }
}

void riesgos(void const * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  /* Infinite loop */
  for(;;)
  {
		// Se lee la rotacion
		xSemaphoreGive( sem_sintomas1 );
		xSemaphoreTake(sem_sintomas1, ( TickType_t ) 0 );
		int rotacionXriesgos = rotacionX;
		int rotacionYriesgos = rotacionY;
		xSemaphoreGive( sem_sintomas1 );
		
		//Se lee el sintoma2
		xSemaphoreGive( sem_sintomas2 );
		xSemaphoreTake(sem_sintomas2, ( TickType_t ) 0 );
		int sintomas2riesgos = sintomas2;
		xSemaphoreGive( sem_sintomas2 );
		
		//Se lee el sintoma3
		xSemaphoreGive( sem_sintomas3 );
		xSemaphoreTake(sem_sintomas3, ( TickType_t ) 0 );
		int sintomas3riesgos = sintomas3;
		xSemaphoreGive( sem_sintomas3 );
		
		//Se recibe la velocidad
		float velocidadRecibida = (float)ByteReceived;
		vel = velocidadRecibida;
		//Se recibe la distancia
		int riesgo = 0;
		//Manipulando el M�VIL
		if(rotacionXriesgos>20 && rotacionYriesgos>20 && sintomas2riesgos==1){
			riesgo++;
		}
		//NO ATIENDE a la carretera
		if((rotacionXriesgos>20 || rotacionYriesgos>20) && sintomas2riesgos==0 && velocidadRecibida>70){
			riesgo++;
		}
		//SOMNOLENCIA
		if(rotacionYriesgos>30 && sintomas3riesgos==1 ){
			riesgo++;
		}
		r = riesgo;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( T_TAREA4 )); 
  }
}

void deteccionPulsador(void const * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  /* Infinite loop */
  for(;;)
  {
			xSemaphoreTake( sem_int_pulsador,  portMAX_DELAY );
			xSemaphoreGive( sem_modo );
			xSemaphoreTake( sem_modo, ( TickType_t ) 0 );
			modo++;
			if(modo > 3){
				modo = 1;
			}
			xSemaphoreGive( sem_modo );
			
		
		ContTarea2 ++;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( T_TAREA5 )); 
  }
}


void deteccionVolantazos(void const * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int anterior = 0;
	int veces = 0;
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	
  for(;;)
  {
			int actual = 0; //Valor actual
			//Lectura del canal ADC0
			ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_0; // seleccionamos el canal 0
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1); // comenzamos la convers�n AD
			if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
				actual = HAL_ADC_GetValue(&hadc1); // leemos el valor
				//PosicionVolante = actual; // actualizamos una variable global
			}
			
			xSemaphoreGive( sem_sintomas3 );
			xSemaphoreTake( sem_sintomas3, ( TickType_t ) 0 );
			int diferencia = anterior - actual;
			diff = anterior - actual;
			
			if(diferencia > 150 || diferencia < -150){
				sintomas3 = 1;
				veces = 0;
			} else {
				veces++;
			}
			if(veces >= 12){
				sintomas3 = 0;
			}
			xSemaphoreGive( sem_sintomas3 );
			anterior = actual;
		
		ContTarea2 ++;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( T_TAREA2 )); 
  }
}

void relaxVolante(void const * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
			xSemaphoreGive( sem_sintomas2 );
			xSemaphoreTake( sem_sintomas2, ( TickType_t ) 0 );
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == GPIO_PIN_SET){
				sintomas2 = 1;
			} else {
				sintomas2 = 0;
			}
			xSemaphoreGive( sem_sintomas2 );
			
		ContTarea2 ++;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( T_TAREA4 )); 
  }
}







/* Funcion para el tratamiento de interrupciones */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken;
	if( sem_int_pulsador != NULL )
	{
		xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR( sem_int_pulsador, &xHigherPriorityTaskWoken );
	}
  long yield = pdFALSE;
//  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  portYIELD_FROM_ISR(yield);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

uint32_t DWT_Delay_Init(void) {
	/* Disable TRC */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
	/* Enable TRC */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
	/* Disable clock cycle counter */
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	/* Enable clock cycle counter */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
	/* Reset the clock cycle counter value */
	DWT->CYCCNT = 0;
	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	/* Check if clock cycle counter has started */
	if(DWT->CYCCNT)
	{
	return 0; /*clock cycle counter started*/
	}
	else
	{
	return 1; /*clock cycle counter not started*/
	}
}

uint32_t readDistance(void)
{
	__IO uint8_t flag=0;
	__IO uint32_t disTime=0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
	DWT_Delay_us(10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
	while(flag == 0){
	while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11) == GPIO_PIN_SET){
	disTime++;
	flag = 1;
	}
	}
	return disTime;
}



uint8_t SPI_Read (uint8_t address)
{
	  // 1.Bring slave select low
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	    // 2.Transmit register + 0x80 (To set MSB high) Most Significant Bit(MSB) high = read mode
	    spiTxBuf[0] = address|0x80; //Register
	    HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
	    // 3.Receive data
	    HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
	    // 4.Bring slave select high
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

        return spiRxBuf[0];
}



void Inicializa_Acelerometro ()
{
 /*To transmit data in SPI follow the next steps: */
 // 1. Bring slave select to low
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
 // 2. Transmit register + data
 spiTxBuf[0] = 0x20; // control Register
 spiTxBuf[1] = 0x17; //Data  Enable X Y Z Rate 3.125 Hz --- Valor original = 0x11
 //								size, timeout
 HAL_SPI_Transmit(&hspi1, spiTxBuf, 2, 50);
 // 3. Bring slave select high
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

 /*To receive data in SPI follow the next steps: */
 // 1.Bring slave select low
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
 // 2.Transmit register + 0x80 (To set MSB high) Most Significant Bit(MSB) high = read mode
 spiTxBuf[0] = 0x20|0x80; //Register
 HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
 // 3.Receive data
 HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
 // 4.Bring slave select high
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
