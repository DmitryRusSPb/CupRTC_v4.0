/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/07/hal-library-15-hd44780-for-stm32fxxx/
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   HD44780 LCD driver library for STM32Fxxx
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
#ifndef TM_HD44780_H
#define TM_HD44780_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup TM_HD44780
 * @brief    HD44780 LCD driver library for STM32Fxxx - http://stm32f4-discovery.net/2015/07/hal-library-15-hd44780-for-stm32fxxx/
 * @{
 *
 *	It also supports all HD44780 compatible LCD drivers.
 *
 * \par Default pinout
 *	
\verbatim
LCD   STM32Fxxx         DESCRIPTION

GND   GND               Ground
VCC   +5V               Power supply for LCD
V0    Potentiometer	    Contrast voltage. Connect to potentiometer
RS    PB2               Register select, can be overwritten in your project's defines.h file
RW    GND               Read/write
E     PB7               Enable pin, can be overwritten in your project's defines.h file
D0    -                 Data 0 - doesn't care
D1    -                 Data 1 - doesn't care
D2    -                 Data 2 - doesn't care
D3    -                 Data 3 - doesn't  care
D4    PC12              Data 4, can be overwritten in your project's defines.h file
D5    PC13              Data 5, can be overwritten in your project's defines.h file
D6    PB12              Data 6, can be overwritten in your project's defines.h file
D7    PB13              Data 7, can be overwritten in your project's defines.h file
A     +3V3              Back light positive power
K     GND               Ground for back light
\endverbatim	
 *	
 * If you want to change pinout, do this in your defines.h file with lines below and set your own settings:
 *	
\code
//RS - Register select pin
#define HD44780_RS_PORT     GPIOB
#define HD44780_RS_PIN      GPIO_PIN_2
//E - Enable pin
#define HD44780_E_PORT      GPIOB
#define HD44780_E_PIN       GPIO_PIN_7
//D4 - Data 4 pin
#define HD44780_D4_PORT     GPIOC
#define HD44780_D4_PIN      GPIO_PIN_12
//D5 - Data 5 pin
#define HD44780_D5_PORT     GPIOC
#define HD44780_D5_PIN      GPIO_PIN_13
//D6 - Data 6 pin
#define HD44780_D6_PORT     GPIOB
#define HD44780_D6_PIN      GPIO_PIN_12
//D7 - Data 7 pin
#define HD44780_D7_PORT     GPIOB
#define HD44780_D7_PIN      GPIO_PIN_13
\endcode
 *
 * \par Changelog
 *
\verbatim
 Version 1.0
  - First release
\endverbatim
 *
 * \par Dependencies
 *
\verbatim
 - STM32Fxxx HAL
 - defines.h
 - TM DELAY
 - TM GPIO
\endverbatim
 */
#include "stm32fxxx_hal.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_gpio.h"

/**
 * @defgroup TM_HD44780_Macros
 * @brief    Library defines
 * @{
 */

/* 4 bit mode */
/* Control pins, can be overwritten */
/* RS - Register select pin */
#ifndef HD44780_RS_PIN
#define HD44780_RS_PORT				GPIOB
#define HD44780_RS_PIN				GPIO_PIN_2
#endif
/* E - Enable pin */
#ifndef HD44780_E_PIN
#define HD44780_E_PORT				GPIOB
#define HD44780_E_PIN				GPIO_PIN_7
#endif
/* Data pins */
/* D4 - Data 4 pin */
#ifndef HD44780_D4_PIN
#define HD44780_D4_PORT				GPIOC
#define HD44780_D4_PIN				GPIO_PIN_12
#endif
/* D5 - Data 5 pin */
#ifndef HD44780_D5_PIN
#define HD44780_D5_PORT				GPIOC
#define HD44780_D5_PIN				GPIO_PIN_13
#endif
/* D6 - Data 6 pin */
#ifndef HD44780_D6_PIN
#define HD44780_D6_PORT				GPIOB
#define HD44780_D6_PIN				GPIO_PIN_12
#endif
/* D7 - Data 7 pin */
#ifndef HD44780_D7_PIN
#define HD44780_D7_PORT				GPIOB
#define HD44780_D7_PIN				GPIO_PIN_13
#endif

/**
 * @}
 */

/**
 * @defgroup TM_HD44780_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes HD44780 LCD
 * @brief  cols: Width of lcd
 * @param  rows: Height of lcd
 * @retval None
 */
void TM_HD44780_Init(uint8_t cols, uint8_t rows);

/**
 * @brief  Turn display on
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOn(void);

/**
 * @brief  Turn display off
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOff(void);

/**
 * @brief  Clears entire LCD
 * @param  None
 * @retval None
 */
void TM_HD44780_Clear(void);

/**
 * @brief  Puts string on lcd
 * @param  x: X location where string will start
 * @param  y; Y location where string will start
 * @param  *str: pointer to string to display
 * @retval None
 */
void TM_HD44780_Puts(uint8_t x, uint8_t y, uint8_t * str, uint8_t len);

/**
 * @brief  Enables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOn(void);

/**
 * @brief  Disables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOff(void);

/**
 * @brief  Shows cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOn(void);

/**
 * @brief  Hides cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOff(void);

/**
 * @brief  Scrolls display to the left
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollLeft(void);

/**
 * @brief  Scrolls display to the right
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollRight(void);

/**
 * @brief  Creates custom character
 * @param  location: Location where to save character on LCD. LCD supports up to 8 custom characters, so locations are 0 - 7
 * @param *data: Pointer to 8-bytes of data for one character
 * @retval None
 */
void TM_HD44780_CreateChar(uint8_t location, uint8_t* data);

/**
 * @brief  Puts custom created character on LCD
 * @param  x: X location where character will be shown
 * @param  y: Y location where character will be shown
 * @param  location: Location on LCD where character is stored, 0 - 7
 * @retval None
 */
void TM_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);

void TM_HD44780_Puts_test(uint8_t x, uint8_t y, uint8_t *str, uint8_t len);

typedef enum
{
	//А
	RUS00 = 0x41,
	//Б
	RUS01 = 0xA0,
	//В
	RUS02 = 0x42,
	//Г
	RUS03 = 0xA1,
	//Д
	RUS04 = 0xE0,
	//Е
	RUS05 = 0x45,
	//Ё
	RUS06 = 0xA2,
	//Ж
	RUS07 = 0xA3,
	//З
	RUS08 = 0xA4,
	//И
	RUS09 = 0xA5,
	//Й
	RUS0A = 0xA6,
	//К
	RUS0B = 0x4B,
	//Л
	RUS0C = 0xA7,
	//М
	RUS0D = 0x4D,
	//Н
	RUS0E = 0x48,
	//О
	RUS0F = 0x4F,
	//П
	RUS10 = 0xA8,
	//Р
	RUS11 = 0x50,
	//С
	RUS12 = 0x43,
	//Т
	RUS13 = 0x54,
	//У
	RUS14 = 0xA9,
	//Ф
	RUS15 = 0xAA,
	//Х
	RUS16 = 0x58,
	//Ц
	RUS17 = 0xE1,
	//Ч
	RUS18 = 0xAB,
	//Ш
	RUS19 = 0xAC,
	//Щ
	RUS1A = 0xE2,
	//Ъ
	RUS1B = 0xAD,
	//Ы
	RUS1C = 0xAE,
	//Ь
	RUS1D = 0x62,
	//Э
	RUS1E = 0xAF,
	//Ю
	RUS1F = 0xB0,
	//Я
	RUS20 = 0xB1,
	//а
	RUS21 = 0x61,
	//б
	RUS22 = 0xB2,
	//в
	RUS23 = 0xB3,
	//г
	RUS24 = 0xB4,
	//д
	RUS25 = 0xE3,
	//е
	RUS26 = 0x65,
	//ё
	RUS27 = 0xB5,
	//ж
	RUS28 = 0xB6,
	//з
	RUS29 = 0xB7,
	//и
	RUS2A = 0xB8,
	//й
	RUS2B = 0xB9,
	//к
	RUS2C = 0xBA,
	//л
	RUS2D = 0xBB,
	//м
	RUS2E = 0xBC,
	//н
	RUS2F = 0xBD,
	//о
	RUS30 = 0x6F,
	//п
	RUS31 = 0xBE,
	//р
	RUS32 = 0x70,
	//с
	RUS33 = 0x63,
	//т
	RUS34 = 0xBF,
	//у
	RUS35 = 0x79,
	//ф
	RUS36 = 0xE4,
	//х
	RUS37 = 0x78,
	//ц
	RUS38 = 0xE5,
	//ч
	RUS39 = 0xC0,
	//ш
	RUS3A = 0xC1,
	//щ
	RUS3B = 0xE6,
	//ъ
	RUS3C = 0xC2,
	//ы
	RUS3D = 0xC3,
	//ь
	RUS3E = 0xC4,
	//э
	RUS3F = 0xC5,
	//ю
	RUS40 = 0xC6,
	//я
	RUS41 = 0xC7,
	//ERORR
	RUSerror = 0x00
} CONVERTtoRUS;

//uint8_t CONVERTtoRUS[] =
//{
//		//А
//		0x41,
//		//Б
//		0xA0,
//		//В
//		0x42,
//		//Г
//		0xA1,
//		//Д
//		0xE0,
//		//Е
//		0x45,
//		//Ё
//		0xA2,
//		//Ж
//		0xA3,
//		//З
//		0xA4,
//		//И
//		0xA5,
//		//Й
//		0xA6,
//		//К
//		0x4B,
//		//Л
//		0xA7,
//		//М
//		0x4D,
//		//Н
//		0x48,
//		//О
//		0x4F,
//		//П
//		0xA8,
//		//Р
//		0x50,
//		//С
//		0x43,
//		//Т
//		0x54,
//		//У
//		0xA9,
//		//Ф
//		0xAA,
//		//Х
//		0x58,
//		//Ц
//		0xE1,
//		//Ч
//		0xAB,
//		//Ш
//		0xAC,
//		//Щ
//		0xE2,
//		//Ъ
//		0xAD,
//		//Ы
//		0xAE,
//		//Ь
//		0x62,
//		//Э
//		0xAF,
//		//Ю
//		0xB0,
//		//Я
//		0xB1,
//		//а
//		0x61,
//		//б
//		0xB2,
//		//в
//		0xB3,
//		//г
//		0xB4,
//		//д
//		0xE3,
//		//е
//		0x65,
//		//ё
//		0xB5,
//		//ж
//		0xB6,
//		//з
//		0xB7,
//		//и
//		0xB8,
//		//й
//		0xB9,
//		//к
//		0xBA,
//		//л
//		0xBB,
//		//м
//		0xBC,
//		//н
//		0xBD,
//		//о
//		0x6F,
//		//п
//		0xBE,
//		//р
//		0x70,
//		//с
//		0x63,
//		//т
//		0xBF,
//		//у
//		0x79,
//		//ф
//		0xE4,
//		//х
//		0x78,
//		//ц
//		0xE5,
//		//ч
//		0xC0,
//		//ш
//		0xC1,
//		//щ
//		0xE6,
//		//ъ
//		0xC2,
//		//ы
//		0xC3,
//		//ь
//		0xC4,
//		//э
//		0xC5,
//		//ю
//		0xC6,
//		//я
//		0xC7,
//		//ERORR
//		0x00
//};

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
