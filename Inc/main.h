/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
// Количество опро
#define NUMBER_OF_POLLS 10

#define MEM_STATUS (state == 0xFFFFFFFF)

/*.................................FLASH......................................*/
// Адрес начальной страницы в памяти, а также
// адрес места в памяти для хранения занятого на соревнованиях места
#define START_FLASH_PAGE       0x801A000

// Адрес места в памяти для хранения размера Text1
#define SIZE_TEXT1_address     (START_FLASH_PAGE + 0x00000004)

// Адрес места в памяти для хранения Text1
#define TEXT1_address          (START_FLASH_PAGE + 0x00000008)

// Адрес места в памяти для хранения размера Text2
#define SIZE_TEXT2_address     (START_FLASH_PAGE + 0x00000008 + formalSizeText1)

// Адрес места в памяти для хранения Text2
#define TEXT2_address          (START_FLASH_PAGE + 0x0000000C + formalSizeText1)

// Адрес места в памяти для хранения количества фреймов(для аудиофайла)
#define BLOCK_address          (START_FLASH_PAGE + 0x0000000C + formalSizeText1 + formalSizeText2)

// Адрес места в памяти для хранения аудиофайла
#define SPEEX_address          (START_FLASH_PAGE + 0x00000010 + formalSizeText1 + formalSizeText2)

// Если мы получил текст, место или кол-во фреймов(speex), то занимаем 16 байт
#define TEXTSIZE               16

// Если мы получили speex данные, то прибавляем размер speex пакета
#define AUDIOSIZE              20

// Максимальное число байт, которое может уместиться на странице флеша
#define MEMORY_PAGE_SIZE  	   2048

// Скорость прокрутки изображений на LED_MATRIX (больше значение == меньше скорость)
#define SPEED                  10

//............................LED MATRIX.....................................//
#define PORT_NCC    GPIOB        // Ножка CS led matrix
#define PIN_NCC     GPIO_PIN_12

//...............................SPEEX.......................................//
#define FRAME_SIZE              160
#define ENCODED_FRAME_SIZE      20
#define SIZE_BUFF               60

#define BUTTON_GPIO_PORT        GPIOA
#define BUTTON_GPIO_PIN         GPIO_PIN_11

#define LENGTH_OF_LINE_LCD    	16
#define NUMBER_OF_LINES_LCD   	2

#define UPDATE_TEXT				"Идёт обновление!"
#define DEMO_TEXT_1             "Демо текст 1"
#define DEMO_TEXT_2             "Demo text 2"
#define LEN_DEMO_TEXT_1         strlen(DEMO_TEXT_1)
#define LEN_DEMO_TEXT_2         strlen(DEMO_TEXT_2)
#define LEN_UPDATE_TEXT			strlen(UPDATE_TEXT)

/*Defaines для работы с WS2812b*/
#define QUANTITY_OF_LED  		4 	// Количество светодиодов в ленте
#define TRAILING_BYTES			48 	// Длина низкого уровня для разделения команд
#define COUNTER_PERIOD_PWM		80 	// Делитель, настраиваемый при инициализации прерываний для ШИМ
#define PWM_FOR_RGB_HIGH		(uint8_t)ceil(0.72*COUNTER_PERIOD_PWM)  // Длина высокого уровня, необходимая
// для создание логической единицы в WS2812b
#define PWM_FOR_RGB_LOW			(uint8_t)floor(0.28*COUNTER_PERIOD_PWM) // Длина высокого уровня, необходимая
// для создание логического нуля в WS2812b

typedef enum
{
	Flash_OK = 'g',
	ERROR0 = '0',
	ERROR1 = '1',
	ERROR2 = '2',
	ERROR3 = '3',
	ERROR4 = '4',
	ERROR5 = '5',
	ERROR6 = '6',
	ERROR7 = '7',
	ERROR8 = '8',
	ERROR9 = '9',
	ERRORA = 'A',
	ERRORB = 'B',
	ERRORC = 'C',
	ERRORD = 'D',
	ERRORE = 'E',
	ERRORF = 'F'
} AnswerStatus;

typedef enum
{
	NOT_TIME_TO_GO,
	TIME_TO_GO
} EXIT;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
