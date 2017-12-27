/*
 * defines.h
 *
 *  Created on: 22 дек. 2017 г.
 *      Author: root
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEFINES_H
#define __DEFINES_H

/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*---------------------------------MODE---------------------------------------*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
#define SYSTEM_MODE_NORMAL		0
#define SYSTEM_MODE_DEMO		1
#define SYSTEM_MODE_UPDATE		2
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/


/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*--------------------------------FLASH---------------------------------------*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/* Дефайны для записи во вфлеш по порядку
 * // Значение, которое должно быть в месте записи занятого места, чтобы включился
// демо-режим
#define MEM_STATUS 				(state == 0xFFFFFFFF)
// Адрес начальной страницы в памяти, а также
// адрес места в памяти для хранения занятого на соревнованиях места
#define START_FLASH_PAGE       	0x801A000
// Адрес места в памяти для хранения размера Text1
#define SIZE_TEXT1_address     	(START_FLASH_PAGE + 0x00000004)
// Адрес места в памяти для хранения Text1
#define TEXT1_address          	(START_FLASH_PAGE + 0x00000008)
// Адрес места в памяти для хранения размера Text2
#define SIZE_TEXT2_address     	(START_FLASH_PAGE + 0x00000008 + formalSizeText1)
// Адрес места в памяти для хранения Text2
#define TEXT2_address          	(START_FLASH_PAGE + 0x0000000C + formalSizeText1)
// Адрес места в памяти для хранения количества фреймов(для аудиофайла)
#define BLOCK_address          	(START_FLASH_PAGE + 0x0000000C + formalSizeText1 + formalSizeText2)
// Адрес места в памяти для хранения аудиофайла
#define SPEEX_address          	(START_FLASH_PAGE + 0x00000010 + formalSizeText1 + formalSizeText2)
*/

// Значение, которое должно быть в месте записи занятого места, чтобы включился
// демо-режим
#define MEM_STATUS 				(state == 0xFFFFFFFF)
// Адрес начальной страницы в памяти, а также
// адрес места в памяти для хранения занятого на соревнованиях места
#define START_FLASH_PAGE       	0x801A000
// Адрес места в памяти для хранения размера Text1
#define SIZE_TEXT1_address     	0x801A004
// Адрес места в памяти для хранения Text1
#define TEXT1_address          	0x801A008
// Адрес места в памяти для хранения размера Text2
#define SIZE_TEXT2_address     	0x801A058
// Адрес места в памяти для хранения Text2
#define TEXT2_address          	0x801A05C
// Адрес места в памяти для хранения количества фреймов(для аудиофайла)
#define BLOCK_address          	0x801A0AC
// Адрес места в памяти для хранения аудиофайла
#define SPEEX_address          	0x801A0B0

#define STATE_MEMORY_SIZE		4
#define SIZE_TEXT1_MEMORY_SIZE	4
#define TEXT1_MEMORY_SIZE		80
#define SIZE_TEXT2_MEMORY_SIZE	4
#define TEXT2_MEMORY_SIZE		80
#define BLOCK_MEMORY_SIZE		4

// Если мы получил текст, место или кол-во фреймов(speex), то занимаем 16 байт
#define TEXTSIZE               	16
// Если мы получили speex данные, то прибавляем размер speex пакета
#define AUDIOSIZE              	20
// Максимальное число байт, которое может уместиться на странице флеша
#define MEMORY_PAGE_SIZE  	   	2048
// Размер буфера, в который записываются принимаемые по USART данные
#define SIZE_BUFF               60
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/


/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*-----------------------------LED MATRIX-------------------------------------*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
// Если мы определяем макрос LED_MATRIX_ENABLE, то тем самым подрубаем в проекте работу
// с LED-матрицу 8x8
#define LED_MATRIX_ENABLE

#ifdef LED_MATRIX_ENABLE
// Скорость прокрутки изображений на LED_MATRIX (больше значение == меньше скорость)
#define SPEED                  	10
// Ножка CS для подключения LED-матрицы
#define PORT_NCC    			GPIOB
#define PIN_NCC     			GPIO_PIN_12

#endif
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/


/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*--------------------------------SPEEX---------------------------------------*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
// Если мы определяем макрос AUDIO_ENABLE, то тем самым подрубаем в проект возможность
// воспроизводить голосовое сообщение
#define AUDIO_ENABLE

#ifdef AUDIO_ENABLE
// Количество блоков по 20 байт для тестового аудиофайла("Соединение со спутником...")
#define FRAME_SIZE              160
// Количество байт в одном блоке (фрейме)
#define ENCODED_FRAME_SIZE      20
#endif
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/



/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*--------------------------------BUTTON--------------------------------------*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
// Количество опросов состояния кнопки для программного антидребезга
#define NUMBER_OF_POLLS 		10
// Вывод МК, на который будет цепляться кнопка
#define BUTTON_GPIO_PORT        GPIOA
#define BUTTON_GPIO_PIN         GPIO_PIN_11


/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*-----------------------------------LCD--------------------------------------*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
// Если мы определяем макрос LCD_ENABLE, то тем самым подрубаем в проект возможность
// выводить сообщения на LCD
#define LCD_ENABLE

#ifdef LCD_ENABLE
// Длина строки дисплея
#define LENGTH_OF_LINE_LCD    	16
// Количество строк дисплея
#define NUMBER_OF_LINES_LCD   	2
// Текст, который будет выводиться на дисплей во время обновления
#define UPDATE_TEXT				"Идёт обновление!"
// Демо-текст на первой строчке дисплея
#define DEMO_TEXT_1             "Демо текст 1"
// Демо-текст на второй строчке дисплея
#define DEMO_TEXT_2             "Demo text 2"
// Длина демо-текста №1
#define LEN_DEMO_TEXT_1         strlen(DEMO_TEXT_1)
// Длина демо-текста №2
#define LEN_DEMO_TEXT_2         strlen(DEMO_TEXT_2)
// Длина текста, выводимого во время обновления
#define LEN_UPDATE_TEXT			strlen(UPDATE_TEXT)

/* Pin defENABLEions */
#define HD44780_RS_LOW              HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_RESET)
#define HD44780_RS_HIGH             HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_SET)
#define HD44780_E_LOW               HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_RESET)
#define HD44780_E_HIGH              HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_SET)

#define HD44780_E_BLINK             HD44780_E_HIGH; HD44780_Delay(20); HD44780_E_LOW; HD44780_Delay(20)
#define HD44780_Delay(x)            osDelay(x)

/* Commands*/
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80

/* Flags for display entry mode */
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

/* Flags for display on/off control */
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01

/* Flags for display/cursor shift */
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00

/* Flags for function set */
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

/* 4-х канальный режим управления LCD */
/* Управляющие контакты могут быть перезаписаны */
/* RS - Register select pin */
#ifndef HD44780_RS_PIN
#define HD44780_RS_PORT				GPIOA
#define HD44780_RS_PIN				GPIO_PIN_10
#endif
/* E - Enable pin */
#ifndef HD44780_E_PIN
#define HD44780_E_PORT				GPIOA
#define HD44780_E_PIN				GPIO_PIN_9
#endif
/* Выводы данных */
/* D4 - Data 4 pin */
#ifndef HD44780_D4_PIN
#define HD44780_D4_PORT				GPIOA
#define HD44780_D4_PIN				GPIO_PIN_8
#endif
/* D5 - Data 5 pin */
#ifndef HD44780_D5_PIN
#define HD44780_D5_PORT				GPIOC
#define HD44780_D5_PIN				GPIO_PIN_9
#endif
/* D6 - Data 6 pin */
#ifndef HD44780_D6_PIN
#define HD44780_D6_PORT				GPIOC
#define HD44780_D6_PIN				GPIO_PIN_8
#endif
/* D7 - Data 7 pin */
#ifndef HD44780_D7_PIN
#define HD44780_D7_PORT				GPIOC
#define HD44780_D7_PIN				GPIO_PIN_7
#endif
#endif
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/


/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*---------------------------------WS2812B------------------------------------*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
// Если мы определяем макрос WS2812B_ENABLE, то тем самым подрубаем в проект возможность
// работы с управляемыми светодиодами WS2812B
#define WS2812B_ENABLE

#ifdef WS2812B_ENABLE
// Количество светодиодов в первой группе
#define NUM_LED_OF_GROUP_ONE	3
// Количество светодиодов во второй группе
#define NUM_LED_OF_GROUP_TWO	2
// Количество светодиодов в третьей группе
#define NUM_LED_OF_GROUP_THREE	2

#define NUM_LED_OF_GROUP_FOUR	2
// Количество групп светодиодов
#define QUANTITY_OF_GROUPS		4
// Количество светодиодов в ленте
#define QUANTITY_OF_LED  		9

#if((NUM_LED_OF_GROUP_ONE + NUM_LED_OF_GROUP_TWO + NUM_LED_OF_GROUP_THREE + NUM_LED_OF_GROUP_FOUR) != QUANTITY_OF_LED)
{
	#error QUANTITY_OF_LED and sum of NUM_LED_OF_GROUP should be equal!
}
#endif

// Длина низкого уровня для разделения команд
#define TRAILING_BYTES			42
// Делитель, настраиваемый при инициализации прерываний для ШИМ
#define COUNTER_PERIOD_PWM		80
// Длина высокого уровня, необходимая для создание логической единицы в WS2812b
#define PWM_FOR_RGB_HIGH		(uint8_t)ceil(0.72*COUNTER_PERIOD_PWM)
// Длина высокого уровня, необходимая для создание логического нуля в WS2812b
#define PWM_FOR_RGB_LOW			(uint8_t)floor(0.28*COUNTER_PERIOD_PWM)
#endif
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
#endif
