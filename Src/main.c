/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "tm_stm32_hd44780.h"
#include <speex/speex.h>
#include "conspeex.h"
#include <stdlib.h>
#include <string.h>
#include "spx.h"
#include "Matrix_Control.h"
#include "ws2812b.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart2;

osThreadId AdminLaunchHandle;
osThreadId ButtonHandle;
osThreadId USARTHandle;
osThreadId LCDHandle;
osThreadId LEDmatrixHandle;
osThreadId AudioMessageHandle;
osThreadId RGBws2812bHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t buttonStatus;
uint8_t demoModeStatus;
uint8_t updateModeStatus;

//...............................SPEEX.......................................//
__IO int16_t outBuffer[2][FRAME_SIZE];
__IO int16_t *outBuff = outBuffer[0];
__IO uint8_t startDecoding;
__IO uint16_t nbFrames=0;

SpeexBits bits;

void *encState, *decState;
int quality = 4, complexity=1, vbr=0, enh=1;
int quantityAudio = 0;
char inputBytes[ENCODED_FRAME_SIZE];

//...............................FLASH.......................................//
// Здесь храниться адрес ячейки памяти с которой необходимо начинать запись/чтение
uint32_t addressDes = START_FLASH_PAGE;

// Размер(в байтах) всех полученных данных
uint16_t allDataSize;

// Число страниц флеш памяти
uint8_t pageNum = 0;

// Сообщает нам о том, была ли уже очищена первая страница
uint8_t firstErase = 0;

// Здесь храниться информация о занятом месте
uint32_t state;

//Здесь храниться размер Текста №1
uint32_t sizeText1;

// Здесь храниться Текст №1
uint8_t textMessage1[40];

// Здесь храниться размер Текста №2
uint32_t sizeText2;

// Здесь храниться Текст №2
uint8_t textMessage2[40];

// Здесь храниться размер аудиофайла в фреймах (блоки по 20 байт)
uint32_t sizeSpeex;

// Массив, хранящий кодировку для уириллицы
uint8_t rusText[66];

uint8_t creatCharMas[][8] = {
		{0x0a,0x0a,0x0a,0x04,0x04,0x0e,0x15,0x00},
		{0x0a,0x0a,0x0a,0x04,0x15,0x0e,0x04,0x00},
		{0x00,0x0a,0x0a,0x0a,0x04,0x04,0x0e,0x15},
		{0x00,0x00,0x11,0x0a,0x04,0x04,0x0e,0x15}
};

//// Значения переменной:
//// 0 - Запрещено обновление
//// 1 - Разрешено обновление
//// 2 - Ничего не делать/показывать/проигрывать пока значение не поменяется
//// на 0 или 1
//uint8_t updateSystem = 2;
//
//// Состояние режима "обновление"
//uint8_t updateStatus = 0;
//uint8_t buttonStatus = 0;
uint8_t LED_BYTE_Buffer[QUANTITY_OF_LED*24 + TRAILING_BYTES];


uint32_t formalSizeText1;
uint32_t formalSizeText2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
void StartAdminLaunchTask(void const * argument);
void StartButtonTask(void const * argument);
void StartUSARTTask(void const * argument);
void StartLCDTask(void const * argument);
void StartLEDmatrixTask(void const * argument);
void StartAudioMessageTask(void const * argument);
void StartRGBws2812bTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/// @brief Brief of PlayMessage
///
/// Функция воспроизведения аудиофайла
/// Функция принимает указатель на массив с данными
/// и переменную с количеством фреймов (отрезков по 20 байт)
void PlayMessage(unsigned char const *array, uint16_t frame_number);

/// @brief Brief of SpeexInit
///
/// Производит инитиализацию speex декодера
void SpeexInit(void);

/// @brief Brief of SuFlashSaveUserData
// Запись данных во flash память
AnswerStatus SU_FLASH_Save_User_Data(speex_data parsData, uint8_t numReceivedByts);

/// @brief Brief of ReadMemory
///
/// Функция чтения записанных данных из памяти
//void ReadMemory(void);

/// @brief Brief of AntiContactBounce
///
/// Функция для проверки контактов на дребезг
/// На вход подаётся имя порта и номер пина
/// На выходе получаем 1 или 0
GPIO_PinState AntiContactBounce(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

//void WS2812_send(uint8_t redLED,uint8_t greenLED, uint8_t blueLED, uint16_t len);
void WS2812_send_noPTR(uint8_t redLED, uint8_t greenLED, uint8_t blueLED, uint16_t len);

EXIT DrawVert(uint8_t* array, int symbol, uint8_t speed);
EXIT DrawGor(uint8_t* array, int symbol, uint8_t speed);
EXIT DrawTS(uint8_t* array, int symbol, uint8_t speed);
EXIT DrawAll(uint8_t state);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
	// Инициализация led матрицы.
	// На вход подается интенсивность
	MAX729_Init(0x05);

	TM_HD44780_Init(LENGTH_OF_LINE_LCD, NUMBER_OF_LINES_LCD);

	SpeexInit();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of AdminLaunch */
  osThreadDef(AdminLaunch, StartAdminLaunchTask, osPriorityIdle, 0, 256);
  AdminLaunchHandle = osThreadCreate(osThread(AdminLaunch), NULL);

  /* definition and creation of Button */
  osThreadDef(Button, StartButtonTask, osPriorityIdle, 0, 256);
  ButtonHandle = osThreadCreate(osThread(Button), NULL);

  /* definition and creation of USART */
  osThreadDef(USART, StartUSARTTask, osPriorityIdle, 0, 256);
  USARTHandle = osThreadCreate(osThread(USART), NULL);

  /* definition and creation of LCD */
  osThreadDef(LCD, StartLCDTask, osPriorityIdle, 0, 256);
  LCDHandle = osThreadCreate(osThread(LCD), NULL);

  /* definition and creation of LEDmatrix */
  osThreadDef(LEDmatrix, StartLEDmatrixTask, osPriorityIdle, 0, 256);
  LEDmatrixHandle = osThreadCreate(osThread(LEDmatrix), NULL);

  /* definition and creation of AudioMessage */
  osThreadDef(AudioMessage, StartAudioMessageTask, osPriorityIdle, 0, 256);
  AudioMessageHandle = osThreadCreate(osThread(AudioMessage), NULL);

  /* definition and creation of RGBws2812b */
  osThreadDef(RGBws2812b, StartRGBws2812bTask, osPriorityIdle, 0, 256);
  RGBws2812bHandle = osThreadCreate(osThread(RGBws2812b), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 79;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//.............................LED MATRIX.....................................//
void sendData(uint16_t data)
{
	// reset chipselect line, in oder to chose MAX729
	HAL_GPIO_WritePin(PORT_NCC, PIN_NCC, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)&data, 1, 500);
	// set chipselect line
	HAL_GPIO_WritePin(PORT_NCC, PIN_NCC, GPIO_PIN_SET);
}

void MAX729_SetIntensivity(uint8_t intensivity)
{
	if (intensivity > 0x0F) return;
	// test of indicators is off
	sendData(REG_DISPLAY_TEST | 0x00);
	// wake up
	sendData(REG_SHUTDOWN | 0x01);
	// quantity of register = number of row of our matrix = 7(0,1,2,3,4,5,6,7)
	sendData(REG_SCAN_LIMIT | 0x07);
	// intensivity of brightness (from 0...0xFF)
	sendData(REG_INTENSITY | intensivity);
	// decoders are turned off
	sendData(REG_DECODE_MODE | 0x00);
}

void MAX729_Clean(void)
{
	// Чистка LED матрицы
	sendData(REG_DIGIT_0 | 0x00);
	sendData(REG_DIGIT_1 | 0x00);
	sendData(REG_DIGIT_2 | 0x00);
	sendData(REG_DIGIT_3 | 0x00);
	sendData(REG_DIGIT_4 | 0x00);
	sendData(REG_DIGIT_5 | 0x00);
	sendData(REG_DIGIT_6 | 0x00);
	sendData(REG_DIGIT_7 | 0x00);
}

void MAX729_Init(uint8_t intensivity)
{
	MAX729_SetIntensivity(intensivity);
	MAX729_Clean();
}

// Двигаем изображение по горизонтали
EXIT DrawGor(uint8_t* array, int symbol, uint8_t speed)
{
	// Переменная для хранения...(скроллинг)
	uint8_t q = 0;

	// Здесь описан момент движения (справа налево) изоображения до
	// тех пор, пока оно не будет полностью видно
	for(uint8_t i = 8; i > 0; i--)
	{
		for(uint8_t j = 0; j < 8; j++)
		{

			// Делаем битовый сдвиг (двигаем изображение)
			q = array[8*symbol + j] << i;
			// Отправляем данные LedMatrix
			sendData(max7219DigitRegisters[j] | q);
		}
		if(buttonStatus) return TIME_TO_GO;
		osDelay(SPEED*speed);
	}
	// Здесь описан момент движения (справа налево) изоображения с положения
	// нормальной видимости до его полного исчезновения
	for(uint8_t i = 0; i < 8; i++)
	{
		for(uint8_t j = 0; j < 8; j++)
		{

			// Делаем битовый сдвиг (двигаем изображение)
			q = array[8*symbol + j] >> i;
			// Отправляем данные LedMatrix
			sendData(max7219DigitRegisters[j] | q);
		}
		if(buttonStatus) return TIME_TO_GO;
		osDelay(SPEED*speed);
	}
	return NOT_TIME_TO_GO;
}

EXIT DrawVert(uint8_t* array, int symbol, uint8_t speed) // Зажигает светодиоиды в LED matrix
{
	// Переменная для хранения...(скроллинг)
	uint8_t q = 0;

	// Здесь описан момент "поднятия" изоображения снизу до
	// тех пор, пока оно не будет полностью видно
	for(uint8_t i = 8; i > 0; i--)
	{
		for(uint8_t j = 0; j < 8; j++)
		{

			// Если сумма номера столбца и
			// количества сдвигов больше 7, то это значит,
			// что изображение ещё не дошло до данной строчки
			// и, следовательно, нужно показать пустоту.
			if((j+i) > 7)
			{
				q = 0;
			}
			else
			{
				q = array[8*symbol + j + i];
			}
			// Отправляем данные LedMatrix
			sendData(max7219DigitRegisters[j] | q);
		}
		if(buttonStatus) return TIME_TO_GO;
		osDelay(SPEED*speed);
	}

	// Здесь описан момент "поднятия" изоображения с положения
	// нормальной видимости до его полного исчезновения
	for(uint8_t i = 0; i <= 8; i++)
	{
		for(uint8_t j = 0; j < 8; j++)
		{

			// Если разница между номером столбца и
			// количеством сдвигов меньше нуля, то это значит,
			// что изображение уже прошло данную строку
			// и, следовательно, нужно показать пустоту.
			if((j-i) < 0)
			{
				q = 0;
			}
			else
			{
				q = array[8*symbol + j - i];
			}
			// Отправляем данные LedMatrix
			sendData(max7219DigitRegisters[j] | q);
		}
		if(buttonStatus) return TIME_TO_GO;
		osDelay(SPEED*speed);
	}
	return NOT_TIME_TO_GO;
}

EXIT DrawTS(uint8_t* array, int symbol, uint8_t speed)
{
	// Переменная для хранения...(скроллинг)
	uint8_t q = 0;

	// Здесь описан момент движения (справа налево) изоображения с положения
	// нормальной видимости до его полного исчезновения
	Draw(array, symbol);
	for(uint8_t j = 0; j < 8; j++)
	{
		// Делаем битовый сдвиг (двигаем изображение)
		q = array[8*symbol + j] >> 1;
		// Отправляем данные LedMatrix
		sendData(max7219DigitRegisters[j] | q);
	}
	if(buttonStatus) return TIME_TO_GO;
	osDelay(SPEED*speed);

	for(uint8_t j = 0; j < 8; j++)
	{
		// Делаем битовый сдвиг (двигаем изображение)
		q = array[8*symbol + j - 1] >> 1;
		// Отправляем данные LedMatrix
		sendData(max7219DigitRegisters[j] | q);
		sendData(max7219DigitRegisters[0] | 0);
	}
	if(buttonStatus) return TIME_TO_GO;
	osDelay(SPEED*speed);

	for(uint8_t j = 0; j < 8; j++)
	{
		// Делаем битовый сдвиг (двигаем изображение)
		q = array[8*symbol + j - 1];
		// Отправляем данные LedMatrix
		sendData(max7219DigitRegisters[j] | q);
		sendData(max7219DigitRegisters[0] | 0);
	}
	if(buttonStatus) return TIME_TO_GO;
	osDelay(SPEED*speed);

	for(uint8_t j = 0; j < 8; j++)
	{
		// Делаем битовый сдвиг (двигаем изображение)
		q = array[8*symbol + j - 1] << 1;
		// Отправляем данные LedMatrix
		sendData(max7219DigitRegisters[j] | q);
		sendData(max7219DigitRegisters[0] | 0);
	}
	if(buttonStatus) return TIME_TO_GO;
	osDelay(SPEED*speed);

	for(uint8_t j = 0; j < 8; j++)
	{
		// Делаем битовый сдвиг (двигаем изображение)
		q = array[8*symbol + j] << 1;
		// Отправляем данные LedMatrix
		sendData(max7219DigitRegisters[j] | q);
	}
	if(buttonStatus) return TIME_TO_GO;
	osDelay(SPEED*speed);

	for(uint8_t j = 0; j < 8; j++)
	{

		// Делаем битовый сдвиг (двигаем изображение)
		q = array[8*symbol + j];
		// Отправляем данные LedMatrix
		sendData(max7219DigitRegisters[j] | q);
	}
	if(buttonStatus) return TIME_TO_GO;
	osDelay(SPEED*speed);
	return NOT_TIME_TO_GO;
}

void Draw(uint8_t* array, int symbol) // Зажигает светодиоиды в LED matrix
{
	sendData(REG_DIGIT_0 | array[8*symbol + 0]);
	sendData(REG_DIGIT_1 | array[8*symbol + 1]);
	sendData(REG_DIGIT_2 | array[8*symbol + 2]);
	sendData(REG_DIGIT_3 | array[8*symbol + 3]);
	sendData(REG_DIGIT_4 | array[8*symbol + 4]);
	sendData(REG_DIGIT_5 | array[8*symbol + 5]);
	sendData(REG_DIGIT_6 | array[8*symbol + 6]);
	sendData(REG_DIGIT_7 | array[8*symbol + 7]);
}

EXIT DrawAll(uint8_t state)
{

	// Пускаем змейку
	for(uint8_t i = 0; i < lenSnake; i++)
	{
		Draw((uint8_t*)snake, i);
		osDelay(SPEED*50);
	}

	for(uint8_t i = 100; i > 1; i-=2)
	{
		if(DrawTS((uint8_t*)symbols, state, i)) return TIME_TO_GO;
	}
	//пускаем анимацию
	switch(state)
	{
	case 1:
		for(uint8_t i = 0; i < lenAnim1; i++)
		{
			Draw((uint8_t*)anim1, i);
			if(buttonStatus) return TIME_TO_GO;
			osDelay(SPEED*50);
		}
		break;

	case 2:
		for(uint8_t i = 0; i < lenAnim2; i++)
		{
			Draw((uint8_t*)anim2, i);
			if(buttonStatus) return TIME_TO_GO;
			osDelay(SPEED*50);
		}
		break;

	case 3:
		for(uint8_t i = 0; i < lenAnim3; i++)
		{
			Draw((uint8_t*)anim3, i);
			if(buttonStatus) return TIME_TO_GO;
			osDelay(SPEED*50);
		}
		break;
	default:
		// Рисуем грустный смайлик
		break;
	}
	// Показываем занятое место
	Draw((uint8_t*)symbols, state);
	// Делаем задержку примерно 10 секунд и параллельно смотрим в очередь
	for(uint8_t i = 0; i!= 100; i++)
	{
		if(buttonStatus) return TIME_TO_GO;
		osDelay(100);
	}
	for(uint8_t i = 100; i > 1; i-=2)
	{
		if(DrawGor((uint8_t*)symbols, state, i)) return TIME_TO_GO;
	}
	// Показываем занятое место
	Draw((uint8_t*)symbols, state);
	// Делаем задержку примерно 10 секунд и параллельно смотрим в очередь
	for(uint8_t i = 0; i!= 100; i++)
	{
		if(buttonStatus) return TIME_TO_GO;
		osDelay(100);
	}
	for(uint8_t i = 100; i > 1; i-=2)
	{
		if(DrawVert((uint8_t*)symbols, state, i)) return TIME_TO_GO;
	}
	// Показываем занятое место
	Draw((uint8_t*)symbols, state);
	// Делаем задержку примерно 10 секунд и параллельно смотрим в очередь
	for(uint8_t i = 0; i!= 100; i++)
	{
		if(buttonStatus) return TIME_TO_GO;
		osDelay(100);
	}
	return NOT_TIME_TO_GO;
}
//....................................SPEEX...................................//
void SpeexInit(void)
{
	speex_bits_init(&bits);
	decState = speex_decoder_init(&speex_nb_mode);
	speex_decoder_ctl(decState, SPEEX_SET_ENH, &enh);
}

void PlayMessage(unsigned char const *array, uint16_t frame_number)
{
	int i;
	uint16_t sample_index = 0;

	for(i=0;i<ENCODED_FRAME_SIZE; i++)
	{
		inputBytes[i] = array[sample_index];
		sample_index++;
	}
	speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
	speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[0]);

	for(i=0;i<ENCODED_FRAME_SIZE; i++)
	{
		inputBytes[i] = array[sample_index];
		sample_index++;
	}

	speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
	speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[1]);

	nbFrames++;

	while(nbFrames < frame_number)
	{
		if(startDecoding == 1)
		{
			for(i=0;i<ENCODED_FRAME_SIZE; i++)
			{
				inputBytes[i] = array[sample_index];
				sample_index++;
			}

			speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
			speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[0]);

			startDecoding = 0;
			nbFrames++;
		}
		if(startDecoding == 2)
		{
			for(i=0;i<ENCODED_FRAME_SIZE; i++)
			{
				inputBytes[i] = array[sample_index];
				sample_index++;
			}

			speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
			speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[1]);

			startDecoding = 0;
			nbFrames++;
		}

	}//end while

	sample_index = 0;
	nbFrames = 0;
	outBuff = outBuffer[0];
}

//.....................................FLASH..................................//
// Функция записи в флеш-память
AnswerStatus SU_FLASH_Save_User_Data(speex_data parsData, uint8_t numReceivedByts)
{
	// Размер принятого сообщения
	uint8_t messageSize = 0;

	uint32_t convert = 0;

	// Создаём указатель на записываемые данные
	uint32_t *addressSrc;

	uint8_t multipleMessageSize = 0;

	switch (parsData.command)
	{
	// Занятое место
	case 1:
		convert |= parsData.data[0];
		addressSrc = &convert;
		// Место будет занимать в памяти 4 байта
		messageSize = 4;
		allDataSize += messageSize;
		break;
		// Тексты 1 и 2
	case 2:
	case 3:
		addressSrc = (uint32_t*)&parsData.data;

		// Заменяем два байта кириллицы одним
		//ConvertRus(dataBufferPars.data, messageSize);

		// Вычитаем 15 из общего числа полученных байт, так как это не сами данные
		// ,а лишь информация о роде данных.
		messageSize = numReceivedByts - 15;
		multipleMessageSize = messageSize;
		// Если размер сообщения не кратен 4, то прибавляем до кратности
		// и обнуляем элемент массива под номером messageSize,
		// так как в нём лежат старые данные
		if((messageSize % 4) != 0)
		{
			while((messageSize % 4) != 0)
			{
				parsData.data[messageSize] = 0;
				messageSize++;
			}
		}
		allDataSize += messageSize;
		// Прибавляем 4, так как нужно учесть количество символов
		// ,которое храниться в памяти в 4 байтах
		allDataSize += 4;
		break;
		// Количество фреймов (блоки по 20 байт)
	case 4:
		convert |= parsData.blockNumber;
		addressSrc = &convert;
		// Количество фреймов будет занимать в памяти 4 байта
		messageSize = 4;
		allDataSize += messageSize;
		break;
		// Аудиоданные
	case 5:
		addressSrc = (uint32_t*)&parsData.data;
		// Аудиоданные будут занимать в памяти 20 байт
		messageSize = 20;
		allDataSize += messageSize;
		break;
	}
	osDelay(1);

	if(HAL_FLASH_Unlock() != HAL_OK)  // Открываем флеш для чистки
	{
		return ERROR9;
	}

	// Если это первая страница и мы её ещё не чистили, то чистим её
	if(!firstErase)
	{
		firstErase = 1;
		FLASH_PageErase(START_FLASH_PAGE);
		// Очищаем бит PER в регистре FLASH->CR,
		// который мы записали туда в ходе чистки страницы памяти
		CLEAR_BIT(FLASH->CR, (FLASH_CR_PER));
	}
	else
		// Если размер уже принятых данных больше размера страницы флеша,
		// то чистим следующую страницу
		if(allDataSize >= MEMORY_PAGE_SIZE)
		{
			// Вычисляем, сколько байт у нас не поместилось на предыдущей странице
			allDataSize -= MEMORY_PAGE_SIZE;
			// Увеличиваем счётчик страниц флеша на 1
			pageNum++;
			FLASH_PageErase(START_FLASH_PAGE + 0x00000800*pageNum);
			// Очищаем бит PER в регистре FLASH->CR,
			// который мы записали туда в ходе чистки страницы памяти
			CLEAR_BIT(FLASH->CR, (FLASH_CR_PER));
		}

	if(parsData.command == 2 || parsData.command == 3)
	{
		// Записываем в память количество байт в тексте
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD , addressDes, multipleMessageSize) != HAL_OK)
		{
			// Закрываем флеш в случае неудачной записи
			HAL_FLASH_Lock();
			return ERRORA;
		}
		addressDes += 4;
	}
	// Записываем по 4 байта данные из буфера
	for(uint8_t i = 0; i<messageSize; i+=4)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD , addressDes, *addressSrc) == HAL_OK)
		{
			addressDes+=4;
			addressSrc++;
		}
		// Закрываем флеш в случае неудачной записи
		else
		{
			HAL_FLASH_Lock();
			return ERRORB;
		}
	}
	// Очищаем бит PER в регистре FLASH->CR,
	// которые мы записали туда в ходе чистки страницы памяти
	CLEAR_BIT(FLASH->CR, (FLASH_CR_PER));
	if(HAL_FLASH_Lock() != HAL_OK)
	{
		return ERRORC;
	}
	return Flash_OK;
}

/* This function sends data bytes out to a string of WS2812s
 * The first argument is a pointer to the first RGB triplet to be sent
 * The seconds argument is the number of LEDs in the chain
 *
 * This will result in the RGB triplet passed by argument 1 being sent to
 * the LED that is the furthest away from the controller (the point where
 * data is injected into the chain)
 * Функция принимает на вход три значения для каждого из трёх цветов
 * и количество светодиодов
 */
//void WS2812_send(uint8_t redLED, uint8_t greenLED, uint8_t blueLED, uint16_t len)
//{
//	uint8_t j;
//	uint8_t *memaddr; 	 	// Указатель на элемент в массиве
//	uint16_t recordedBytes;
//	uint16_t buffersize; 	// Размер массива, в который будет вестись запись
//
//	recordedBytes = 0;
//	buffersize = (len*24) + TRAILING_BYTES;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
//	// fill transmit buffer with correct compare values to achieve
//	// correct pulse widths according to color values
//	memaddr = LED_BYTE_Buffer;
//
//	while (len != 0)
//	{
//		for (j = 0; j < 8; j++)				// GREEN data
//		{
//			if ( (greenLED << j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
//			{
//				*memaddr = PWM_FOR_RGB_HIGH; 		// compare value for logical 1
//			}
//			else
//			{
//				*memaddr = PWM_FOR_RGB_LOW;		// compare value for logical 0
//			}
//			memaddr++;
//			recordedBytes++;
//		}
//
//		for (j = 0; j < 8; j++)				// RED data
//		{
//			if ( (redLED << j) & 0x80 )		// data sent MSB first, j = 0 is MSB j = 7 is LSB
//			{
//				*memaddr = PWM_FOR_RGB_HIGH; 		// compare value for logical 1
//			}
//			else
//			{
//				*memaddr = PWM_FOR_RGB_LOW;		// compare value for logical 0
//			}
//			memaddr++;
//			recordedBytes++;
//		}
//
//		for (j = 0; j < 8; j++)				// BLUE data
//		{
//			if ( (blueLED << j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
//			{
//				*memaddr = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
//			}
//			else
//			{
//				*memaddr = PWM_FOR_RGB_LOW;		// compare value for logical 0
//			}
//			memaddr++;
//			recordedBytes++;
//		}
//		len--;
//	}
//	// add needed delay at end of byte cycle, pulsewidth = 0
//	while(recordedBytes < buffersize)
//	{
//		*memaddr = 0;
//		memaddr++;
//		recordedBytes++;
//	}
//	// Запускаем передачу и включаем шим
//	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_2, (uint32_t*)LED_BYTE_Buffer, buffersize);
//	while(HAL_DMA_GetState(&hdma_tim8_ch2) == HAL_DMA_STATE_BUSY) osDelay(10);
//}

/************************************************************************************/
void WS2812_send_noPTR(uint8_t redLED, uint8_t greenLED, uint8_t blueLED, uint16_t len)
{
	uint8_t j;
	uint8_t led;
	uint16_t memaddr;
	uint16_t buffersize;

	buffersize = (len*24)+TRAILING_BYTES;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index
	led = 0;					// reset led index

	// fill transmit buffer with correct compare values to achieve
	// correct pulse widths according to color values
	while (len)
	{
		for (j = 0; j < 8; j++)					// GREEN data
		{
			if ( (greenLED<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;	// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)					// RED data
		{
			if ( (redLED<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;	// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)					// BLUE data
		{
			if ( (blueLED<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;	// compare value for logical 0
			}
			memaddr++;
		}

		led++;
		len--;
	}

	// add needed delay at end of byte cycle, pulsewidth = 0
	while(memaddr < buffersize)
	{
		LED_BYTE_Buffer[memaddr] = 0;
		memaddr++;
	}
	// Запускаем передачу и включаем шим
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)LED_BYTE_Buffer, buffersize);
	while(HAL_DMA_GetState(&hdma_tim3_ch1_trig) == HAL_DMA_STATE_BUSY) osDelay(10);
}
/************************************************************************************/

GPIO_PinState AntiContactBounce(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	uint8_t numOfPolls;
	numOfPolls = 0;
	while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
	{
		numOfPolls++;
		if(numOfPolls >= NUMBER_OF_POLLS) return GPIO_PIN_SET;
	}
	return GPIO_PIN_RESET;
}
/* USER CODE END 4 */

/* StartAdminLaunchTask function */
void StartAdminLaunchTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	// Отправляем все остальные задачи спать, так как пока что-то делать не требуется
	vTaskSuspend(AudioMessageHandle);
	vTaskSuspend(LCDHandle);
	vTaskSuspend(LEDmatrixHandle);
	vTaskSuspend(USARTHandle);
	//vTaskSuspend(WS2812_RGBHandle);

	formalSizeText1 = 0;
	formalSizeText2 = 0;

	for(;;)
	{
		// Проверяем не нажата ли кнопка
		if(AntiContactBounce(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN))
		{
			// Включаем режим обновления
			updateModeStatus = 1;
		}
		else
		{
			// Читаем из flash памяти информацию о
			// занятом месте
			memcpy(&state, (void *)START_FLASH_PAGE, 4);
			// Если кубок не прошит, то запускаем демо режим
			if((state == 0xFFFFFFFF)||(state == 0))
			{
				demoModeStatus = 1;
			}
			else
			{
				//Читаем из flash памяти информацию о...
				//размере первого текстового сообщения
				memcpy(&sizeText1, (void *)SIZE_TEXT1_address, 4);

				//содержании первого текстового сообщения
				memcpy(textMessage1, (void *)TEXT1_address, sizeText1);
				formalSizeText1 = sizeText1;
				while(formalSizeText1 % 4 !=0)
				{
					formalSizeText1++;
				}
				//размере второго текстового сообщения
				memcpy(&sizeText2, (void *)SIZE_TEXT2_address, 4);
				//содержании второго текстового сообщения
				memcpy(textMessage2, (void *)TEXT2_address, sizeText2);
				formalSizeText2 = sizeText2;
				while(formalSizeText2 % 4 !=0)
				{
					formalSizeText2++;
				}
				//размере аудиофайла (в фреймах по 20 байт)
				memcpy(&sizeSpeex, (void *)BLOCK_address, 4);
				// Удаляем задачу из списка задач
				vTaskDelete(NULL);
			}
		}
		// Отправляем задачу спать
		vTaskSuspend(NULL);
	}
  /* USER CODE END 5 */ 
}

/* StartButtonTask function */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN StartButtonTask */
	/* Infinite loop */
	uint8_t pressTime; // Создаем счётчик для фиксации времени нажатия кнопки

	// Будим задачи
	vTaskResume(LCDHandle);
	vTaskResume(LEDmatrixHandle);
	// vTaskResume(WS2812_RGBHandle);

	for(;;)
	{
		if(AntiContactBounce(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN))
		{
			pressTime = 1;
			// Крутим счётчик до тех пор пока зажата кнопка или
			// значение счётчика меньше 255
			while((HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN))&&(pressTime != 255))
			{
				osDelay(100);
				pressTime++;
			}
			// Если было одно нажатие, то включай запись
			// и зависай в цикле
			if((pressTime <= 30)&&(pressTime >= 1))
			{
				vTaskResume(AudioMessageHandle);
				// Обнуляем счётчик нажатия кнопки
				pressTime = 0;
			}
			// Если кнопка была зажата с момента включения,
			// то включай режим обновления и зависай в цикле
			if((pressTime >= 30)&&(updateModeStatus))
			{
				// Отправляем все остальные задачи спать, так как пока что-то делать не требуется
				vTaskSuspend(AudioMessageHandle);
				vTaskSuspend(LEDmatrixHandle);
				vTaskSuspend(LCDHandle);
				vTaskSuspend(RGBws2812bHandle);

				// Запускаем задачу USART
				vTaskResume(USARTHandle);

				// Отправляем текущую задачу спать
				vTaskSuspend(NULL);
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartButtonTask */
}

/* StartUSARTTask function */
void StartUSARTTask(void const * argument)
{
  /* USER CODE BEGIN StartUSARTTask */
	/* Infinite loop */
	// Счетчик, показывающий сколько мы уже приняли байт в "data_buffer"
	uint8_t numBuff;

	// Ответ о состоянии принятого сообщения
	uint8_t answer;

	// Здесь будут храниться данные после парсинга
	speex_data dataBufferPars;

	// Массив, хранящий принимаемые данные до парсинга
	uint8_t dataBuffer[SIZE_BUFF];

	uint8_t *ptrBuff;

	uint8_t updateLine;

	for(;;)
	{
		// Проверяем, включен ли режим обновления
		if(updateModeStatus)
		{
			// Крутимся в цикле пока не закончится обновление
			// Обнуляем счётчик полученных данных
			allDataSize = 0;

			// Чистим экран
			TM_HD44780_Clear();
			updateLine = 0;

			// Сообщаем, что мы готовы к получению первого блока данных
			answer = 's';
			HAL_UART_Transmit(&huart2, (uint8_t*)&answer, 1, 0xFFFF);
			while(1)
			{
				// Крутимся в цикле пока не встретим символ окончания сообщения
				// или пока не будет переполнения буфера данных "data_buffer",
				// или пока не перестанут приходить данные

				// Присваеваем указателю адрес начала буфера
				ptrBuff = dataBuffer;
				// Обнуляем счётчик байтов в буфере "data_buffer"
				numBuff = 0;
				// По умолчанию ответное сообщение означает успешную передачу
				answer = 'g';
				// Принимаем данные по байту и забрасываем в наш буфер
				while(numBuff < SIZE_BUFF)
				{
					// Приём
					HAL_UART_Receive(&huart2, ptrBuff, 1, 0xFFFF);
					// Увеличиваем счётчик принятых байтов
					numBuff++;
					// Если встретили символ окончания сообщения, то выходим из цикла
					if(*ptrBuff == '>')
						break;
					// Двигаем указатель на следующий элемент массива
					ptrBuff++;
				}
				// Если переданный пакет байтов состоит лишь из <0>, то это означает конец
				// передачи. Выходим из цикла приёма пакетов
				if((*ptrBuff == '>')&&(*(ptrBuff-1) == '0')&&(*(ptrBuff-2) == '<'))
				{
					break;
				}
				// Парсим полученные данные
				dataBufferPars = parsing((char*)dataBuffer, numBuff);

				// Полоска закрузки на LCD
				// Если полоска достигла окончания строки дисплея, то...
				if(!((updateLine + 1) % 17))
				{
					// Чистим экран
					TM_HD44780_Clear();
					// Обнуляем счётчик полоски
					updateLine = 0;
					// Выводим сообщение об обновлении, так как оно стёрлось
					TM_HD44780_Puts(0, 0, UPDATE_TEXT, LEN_UPDATE_TEXT);
				}
				// Выводим элемент полоски загрузки на LCD
				TM_HD44780_PutCustom(updateLine, 1, 255);
				// Увеличиваем счётчик полоски загрузки
				updateLine++;

				// Если не возникло ошибок при парсинге, то сохраняем
				// полученные данные в памяти МК
				if(dataBufferPars.command != 6)
				{
					// Сохраняем данные в памяти
					answer = SU_FLASH_Save_User_Data(dataBufferPars, numBuff);
				}
				else
				{
					// Если возникли проблемы при записи, то переменной,
					// в которой хранится ответ приложению, присваеваем
					// символ ошибки
					answer = 'f';
				}
				// Отправляем результат работы
				HAL_UART_Transmit(&huart2, (uint8_t*)&answer, 1, 0xFFFF);
			}

			// Будим все остальные задачи
			vTaskResume(AdminLaunchHandle);

			vTaskResume(ButtonHandle);

			vTaskResume(AudioMessageHandle);

			vTaskResume(LEDmatrixHandle);

			vTaskResume(LCDHandle);

			// Удаляем эту задачу
			vTaskDelete(NULL);
		}
		osDelay(1);
	}
  /* USER CODE END StartUSARTTask */
}

/* StartLCDTask function */
void StartLCDTask(void const * argument)
{
  /* USER CODE BEGIN StartLCDTask */
	/* Infinite loop */
	for(;;)
	{
		// Чистим экран
		TM_HD44780_Clear();
		// Если включен демо-режим, то показываем демо-сообщение,
		// если нет, то выводим текст, записанный в память
		if(demoModeStatus)
		{
			TM_HD44780_Puts(0, 0, DEMO_TEXT_1, LEN_DEMO_TEXT_1);
			TM_HD44780_Puts(0, 1, DEMO_TEXT_2, LEN_DEMO_TEXT_2);
		}
		else
		{
			TM_HD44780_Puts(0, 0, textMessage1, (uint8_t*)sizeText1);
			TM_HD44780_Puts(0, 1, textMessage2, (uint8_t*)sizeText2);
		}
		// Отправляем задачу спать
		vTaskSuspend(NULL);
	}
  /* USER CODE END StartLCDTask */
}

/* StartLEDmatrixTask function */
void StartLEDmatrixTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDmatrixTask */
	/* Infinite loop */
	for(;;)
	{
		// Чистка LED матрицы от изображений
		MAX729_Clean();
		// Если включен демо режим, то крутим демо-анимацию
		// Если демо-режим выключен, то показываем место+анимация
		if(demoModeStatus)
		{
			while(1)
			{
				// Выводим анимацию на LED matrix
				for(uint8_t i = 1; i < 4; i++)
				{
					DrawAll(i);
				}
				osDelay(1);
			}
		}
		else
		{
			// Выводим место пока не будет нажата кнопка
			switch (state)
			{
			case 1:
				while(DrawAll(1)) osDelay(1);
				break;
			case 2:
				while(DrawAll(2)) osDelay(1);
				break;
			case 3:
				while(DrawAll(3)) osDelay(1);
				break;
			default:
				Draw((uint8_t*)symbols, 0);  // смайлик
				break;
				// Пока сотояние кнопки не изменится на "выкл"
				// крутим особую анимацию
				while(buttonStatus) Draw((uint8_t*)symbols, 0);
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartLEDmatrixTask */
}

/* StartAudioMessageTask function */
void StartAudioMessageTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioMessageTask */
	/* Infinite loop */
	for(;;)
	{
		// Включаем прерывания по таймеру
		HAL_TIM_Base_Start_IT(&htim6);
		// Включаем ЦАП
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

		// Если включен демо-режим, то включаем демо-запись
		if(demoModeStatus) PlayMessage(&spx_voice2[0], spx_frames2);
		// В противном случае проигрываем то, что мы записали в память
		else PlayMessage((void*)SPEEX_address, sizeSpeex);

		// Выключаем прерывания по таймеру
		HAL_TIM_Base_Stop_IT(&htim6);
		// Выключаем ЦАП
		HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);

		// Отправляем текущую задачу спать
		vTaskSuspend(NULL);
	}
  /* USER CODE END StartAudioMessageTask */
}

/* StartRGBws2812bTask function */
void StartRGBws2812bTask(void const * argument)
{
  /* USER CODE BEGIN StartRGBws2812bTask */
	/* Infinite loop */
	for(;;)
	{
		/* first cycle through the colors on 2 LEDs chained together
		 * last LED in the chain will receive first sent triplet
		 * --> last LED in the chain will 'lead'
		 */
		for (uint16_t i = 0; i < 766; i += 1)
		{
			WS2812_send_noPTR(eightbit[i][0], eightbit[i][1], eightbit[i][2], QUANTITY_OF_LED);//eightbit[i][1], eightbit[i][2], QUANTITY_OF_LED);
			osDelay(300);
		}
//		osDelay(5000);
//		for (uint16_t i = 0; i < 766; i += 1)
//		{
//			WS2812_send(eightbit[i][0], eightbit[i][1], eightbit[i][2], QUANTITY_OF_LED);//eightbit[i][1], eightbit[i][2], QUANTITY_OF_LED);
//			osDelay(300);
//		}
		//osDelay(2500);
	}
  /* USER CODE END StartRGBws2812bTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
