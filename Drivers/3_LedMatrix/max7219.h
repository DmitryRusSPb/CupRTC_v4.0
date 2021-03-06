/*
 * max7219.h
 *
 *  Created on: 21 дек. 2017 г.
 *      Author: root
 */

#ifndef MAX7219_H_
#define MAX7219_H_

/*  File: "Matrix_Control.h"
 *   Led Matrix based on MAX729 Serially Interfaced, 8-digit LED display driver
 *   Parameters:
 *   - 10 MHz Serial Interface;
 *   - Size of serial-data format: 16 bits;
 *   - The oldest bit is first;
 *   - ChipSelect from 0 to 1, on leading edge;
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

// Скорость прокрутки изображений на LED_MATRIX (больше значение == меньше скорость)
#define SPEED                  	10
// Ножка CS для подключения LED-матрицы
#define PORT_NCC    			GPIOB
#define PIN_NCC     			GPIO_PIN_12

// Перечисление для выхода из функций анимации LED-Matrix 8х8 в случае, если требуется
// выводить какое-то совершенно другое изображение
typedef enum
{
	NOT_TIME_TO_GO,
	TIME_TO_GO
} EXIT;


/* @brief Brief of DrawVert
 *
 * Функция позволяет двигать изображение LED-матрицы 8х8 по вертикали
 */
EXIT DrawVert(uint8_t* array, int symbol, uint8_t speed);

/* @brief Brief of DrawGor
 *
 * Функция позволяет двигать изображение LED-матрицы 8х8 по горизонтали
 */
EXIT DrawGor(uint8_t* array, int symbol, uint8_t speed);

/* @brief Brief of DrawTS
 *
 * Функция позволяет двигать изображение LED-матрицы 8х8 по окружности
 */
EXIT DrawTS(uint8_t* array, int symbol, uint8_t speed);

/* @brief Brief of DrawAll
 *
 * Функция отвечает за комплексную анимацию LED-матрицы 8х8, включая скроллинг,
 * различные движения и т.д.
 */

EXIT DrawAll(uint8_t state);

void sendData(uint16_t data);
void MAX729_SetIntensivity(uint8_t intensivity);
void MAX729_Clean(void);
void MAX729_Init(uint8_t intensivity);
void Draw(uint8_t* array, int symbol);
/* Packet of serial data includes:
 *  - ADRESS(what we wanna do, and where we wanna put our data of bits...pointer for decoder);
 *  - DATA;
 *  We should use operation shift to left "<<", because Serial-data packet's
 *  format contains of 2 byte (8 bit + 8 bit). This enum is used for making
 *  "adress" part of serial-data packet, so we should shift to left on 8 bits our
 *  number.
 *  {|15|14|13|12|11|10|9|} - we make this part of packet   |8|7|6|5|4|3|2|1|0| - this part for data
 *
 *  So enum MAX7219_REGISTERS contain the list of registers. The decoder of MAX729 see the type of register, and
 *  then put data from field "DATA" to place, which was defined by value of field "ADRESS".
 */
typedef enum
{
	REG_NO_OP         = 0x00 << 8,          // nothing to do
	REG_DIGIT_0       = 0x01 << 8,          // adress cells from 0....7
	REG_DIGIT_1       = 0x02 << 8,
	REG_DIGIT_2       = 0x03 << 8,
	REG_DIGIT_3       = 0x04 << 8,
	REG_DIGIT_4       = 0x05 << 8,
	REG_DIGIT_5       = 0x06 << 8,
	REG_DIGIT_6       = 0x07 << 8,
	REG_DIGIT_7       = 0x08 << 8,
	REG_DECODE_MODE   = 0x09 << 8,          // mode of decode for digital indicators(for led matrix "no decode")
	REG_INTENSITY     = 0x0A << 8,          // level of intensity
	REG_SCAN_LIMIT    = 0x0B << 8,          // number of digital registers, which are should be scanned
	REG_SHUTDOWN      = 0x0C << 8,          // set normal status of workability
	REG_DISPLAY_TEST  = 0x0F << 8,
} MAX7219_REGISTERS;

/* The sights of smile and cross */
typedef enum
{
	CROSS   = 0,
	SMILEY  = 1,
} MARKS;

typedef enum
{
	LEDEXIT0   = 0,
	LEDEXIT1  = 1,
} LEDEXIT;

/* Array of user symbols */
const static uint8_t symbols[4][8]=
{
		// {0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81},  // X
		{0x3c,0x42,0x95,0xa1,0xa1,0x95,0x42,0x3c},  // смайлик
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0x18,0x00},  // 1 место
		{0x7e,0x06,0x0c,0x30,0x60,0x66,0x3c,0x00},  // 2 место
		{0x3c,0x66,0x60,0x38,0x60,0x66,0x3c,0x00},  // 3 место
};

const static uint8_t symbols3Gor[15][8]=
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x01,0x01,0x00,0x01,0x01,0x00,0x00},
		{0x01,0x03,0x03,0x01,0x03,0x03,0x01,0x00},
		{0x03,0x06,0x06,0x03,0x06,0x06,0x03,0x00},
		{0x07,0x0c,0x0c,0x07,0x0c,0x0c,0x07,0x00},
		{0x0f,0x19,0x18,0x0e,0x18,0x19,0x0f,0x00},
		{0x1e,0x33,0x30,0x1c,0x30,0x33,0x1e,0x00},
		{0x3c,0x66,0x60,0x38,0x60,0x66,0x3c,0x00},
		{0x78,0xcc,0xc0,0x70,0xc0,0xcc,0x78,0x00},
		{0xf0,0x98,0x80,0xe0,0x80,0x98,0xf0,0x00},
		{0xe0,0x30,0x00,0xc0,0x00,0x30,0xe0,0x00},
		{0xc0,0x60,0x00,0x80,0x00,0x60,0xc0,0x00},
		{0x80,0xc0,0x00,0x00,0x00,0xc0,0x80,0x00},
		{0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

const static uint8_t lenAnim1 = 105;
const static uint8_t anim1[][8]=
{
		{0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x08,0x08,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x08,0x08,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x08,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x1c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x14,0x08,0x14,0x00,0x00},
		{0x00,0x00,0x2a,0x1c,0x36,0x1c,0x2a,0x00},
		{0x00,0x41,0x14,0x22,0x08,0x22,0x14,0x41},
		{0x00,0x14,0x00,0x41,0x00,0x41,0x00,0x14},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x80,0x40,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x40,0x20,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x20,0x10,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x10,0x08,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x0c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x1c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x14,0x08,0x14,0x00,0x00},
		{0x00,0x00,0x1c,0x36,0x2a,0x36,0x1c,0x00},
		{0x00,0x08,0x36,0x22,0x41,0x22,0x36,0x08},
		{0x00,0x49,0x22,0x00,0x41,0x00,0x22,0x49},
		{0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x41},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x04,0x08,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x08,0x10,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x10,0x38,0x10,0x00,0x00,0x00},
		{0x00,0x00,0x28,0x10,0x28,0x00,0x00,0x00},
		{0x00,0x10,0x10,0x7c,0x10,0x10,0x00,0x00},
		{0x00,0x54,0x28,0x54,0x28,0x54,0x00,0x00},
		{0x00,0x54,0x38,0x6c,0x38,0x54,0x00,0x00},
		{0xc6,0xd6,0x38,0x6c,0x38,0xd6,0xc6,0x00},
		{0x44,0xd6,0x28,0x44,0x28,0xd6,0x44,0x00},
		{0xc6,0x92,0x00,0x44,0x00,0x92,0xc6,0x00},
		{0xc6,0xc6,0x00,0x00,0x00,0xc6,0xc6,0x00},
		{0x00,0x42,0x00,0x00,0x00,0x04,0x40,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x55,0x00,0x00,0x80,0x01,0x00,0x00,0xaa},
		{0x55,0x55,0x00,0xc0,0x03,0x00,0xaa,0xaa},
		{0x55,0x55,0x05,0xe0,0x07,0xa0,0xaa,0xaa},
		{0x55,0x55,0x05,0xf0,0x0f,0xa0,0xaa,0xaa},
		{0x55,0x55,0x05,0xf8,0x1f,0xa0,0xaa,0xaa},
		{0xaa,0xaa,0xfa,0x07,0xe0,0x5f,0x55,0x55},
		{0xaa,0xaa,0xfa,0x1f,0xf8,0x5f,0x55,0x55},
		{0x57,0x57,0x07,0xe0,0x07,0xe0,0xea,0xea},
		{0xbc,0xba,0xf9,0x1f,0xf8,0x9f,0x5d,0x3d},
		{0x43,0x45,0x06,0xe8,0x17,0x60,0xa2,0xc2},
		{0x53,0x55,0x06,0xe8,0x17,0x60,0xaa,0xca},
		{0xac,0xaa,0xf9,0x17,0xe8,0x9f,0x55,0x35},
		{0xac,0xae,0xff,0x17,0xe8,0xff,0x75,0x35},
		{0xec,0xee,0xff,0x97,0xe9,0xff,0x77,0x37},
		{0xef,0xef,0xff,0x9f,0xf9,0xff,0xf7,0xf7},
		{0xff,0xef,0xff,0xdf,0xfb,0xff,0xf7,0xff},
		{0xdf,0xef,0xff,0xdf,0xfb,0xff,0xf7,0xfb},
		{0xdf,0xff,0xff,0xdf,0xfb,0xff,0xff,0xfb},
		{0xdf,0xff,0xff,0xff,0xff,0xff,0xff,0xfb},
		{0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x04},
		{0x20,0x20,0x00,0x00,0x00,0x00,0x04,0x04},
		{0x20,0x20,0x40,0x00,0x00,0x02,0x04,0x04},
		{0x20,0x20,0xc0,0x00,0x00,0x03,0x04,0x04},
		{0x21,0x20,0xc0,0x00,0x00,0x03,0x04,0x84},
		{0x21,0x21,0xc0,0x00,0x00,0x03,0x84,0x84},
		{0x21,0x21,0xc1,0x00,0x00,0x83,0x84,0x84},
		{0x21,0x21,0xc3,0x00,0x00,0xc3,0x84,0x84},
		{0x21,0x21,0xc7,0x00,0x00,0xe3,0x84,0x84},
		{0x21,0x21,0xcf,0x00,0x00,0xf3,0x84,0x84},
		{0x21,0x21,0xcf,0x08,0x10,0xf3,0x84,0x84},
		{0x21,0x21,0xcf,0x18,0x18,0xf3,0x84,0x84},
		{0x21,0x21,0xdf,0x18,0x18,0xfb,0x84,0x84},
		{0x21,0x31,0xdf,0x18,0x18,0xfb,0x8c,0x84},
		{0x21,0x39,0xdf,0x18,0x18,0xfb,0x9c,0x84},
		{0x21,0x7d,0xdf,0x18,0x18,0xfb,0xbe,0x84},
		{0x21,0x7f,0xff,0x18,0x18,0xff,0xfe,0x84},
		{0xa3,0x7f,0xff,0x18,0x18,0xff,0xfe,0xc5},
		{0xa7,0xff,0xff,0x18,0x18,0xff,0xff,0xe5},
		{0xef,0xff,0xff,0x18,0x18,0xff,0xff,0xf7},
		{0xff,0xff,0xff,0x19,0x98,0xff,0xff,0xff},
		{0xff,0xff,0xff,0x59,0x9a,0xff,0xff,0xff},
		{0xff,0xff,0xff,0x5d,0xba,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xdd,0xbb,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xfd,0xbf,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x7e,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x3c,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0xdb,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0x99,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0x7e,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0x3c,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0xff,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0xdf,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0x9d,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0xff,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0x7e,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0x3c,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0xff,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0xdb,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0x99,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0x18,0xff},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0x18,0x7e},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0x18,0x3c},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0x18,0x18},
		{0x7e,0x18,0x18,0x18,0x1c,0x18,0x18,0x00}
};

const static uint8_t lenAnim2 = 104;
const static uint8_t anim2[][8]=
{
		{0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x08,0x08,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x08,0x08,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x08,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x1c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x14,0x08,0x14,0x00,0x00},
		{0x00,0x00,0x2a,0x1c,0x36,0x1c,0x2a,0x00},
		{0x00,0x41,0x14,0x22,0x08,0x22,0x14,0x41},
		{0x00,0x14,0x00,0x41,0x00,0x41,0x00,0x14},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x80,0x40,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x40,0x20,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x20,0x10,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x10,0x08,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x0c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x1c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x14,0x08,0x14,0x00,0x00},
		{0x00,0x00,0x1c,0x36,0x2a,0x36,0x1c,0x00},
		{0x00,0x08,0x36,0x22,0x41,0x22,0x36,0x08},
		{0x00,0x49,0x22,0x00,0x41,0x00,0x22,0x49},
		{0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x41},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x04,0x08,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x08,0x10,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x10,0x38,0x10,0x00,0x00,0x00},
		{0x00,0x00,0x28,0x10,0x28,0x00,0x00,0x00},
		{0x00,0x10,0x10,0x7c,0x10,0x10,0x00,0x00},
		{0x00,0x54,0x28,0x54,0x28,0x54,0x00,0x00},
		{0x00,0x54,0x38,0x6c,0x38,0x54,0x00,0x00},
		{0xc6,0xd6,0x38,0x6c,0x38,0xd6,0xc6,0x00},
		{0x44,0xd6,0x28,0x44,0x28,0xd6,0x44,0x00},
		{0xc6,0x92,0x00,0x44,0x00,0x92,0xc6,0x00},
		{0xc6,0xc6,0x00,0x00,0x00,0xc6,0xc6,0x00},
		{0x00,0x42,0x00,0x00,0x00,0x04,0x40,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x55,0x00,0x00,0x80,0x01,0x00,0x00,0xaa},
		{0x55,0x55,0x00,0xc0,0x03,0x00,0xaa,0xaa},
		{0x55,0x55,0x05,0xe0,0x07,0xa0,0xaa,0xaa},
		{0x55,0x55,0x05,0xf0,0x0f,0xa0,0xaa,0xaa},
		{0x55,0x55,0x05,0xf8,0x1f,0xa0,0xaa,0xaa},
		{0xaa,0xaa,0xfa,0x07,0xe0,0x5f,0x55,0x55},
		{0xaa,0xaa,0xfa,0x1f,0xf8,0x5f,0x55,0x55},
		{0x57,0x57,0x07,0xe0,0x07,0xe0,0xea,0xea},
		{0xbc,0xba,0xf9,0x1f,0xf8,0x9f,0x5d,0x3d},
		{0x43,0x45,0x06,0xe8,0x17,0x60,0xa2,0xc2},
		{0x53,0x55,0x06,0xe8,0x17,0x60,0xaa,0xca},
		{0xac,0xaa,0xf9,0x17,0xe8,0x9f,0x55,0x35},
		{0xac,0xae,0xff,0x17,0xe8,0xff,0x75,0x35},
		{0xec,0xee,0xff,0x97,0xe9,0xff,0x77,0x37},
		{0xef,0xef,0xff,0x9f,0xf9,0xff,0xf7,0xf7},
		{0xff,0xef,0xff,0xdf,0xfb,0xff,0xf7,0xff},
		{0xdf,0xef,0xff,0xdf,0xfb,0xff,0xf7,0xfb},
		{0xdf,0xff,0xff,0xdf,0xfb,0xff,0xff,0xfb},
		{0xdf,0xff,0xff,0xff,0xff,0xff,0xff,0xfb},
		{0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x04},
		{0x20,0x20,0x00,0x00,0x00,0x00,0x04,0x04},
		{0x20,0x20,0x40,0x00,0x00,0x02,0x04,0x04},
		{0x20,0x20,0xc0,0x00,0x00,0x03,0x04,0x04},
		{0x21,0x20,0xc0,0x00,0x00,0x03,0x04,0x84},
		{0x21,0x21,0xc0,0x00,0x00,0x03,0x84,0x84},
		{0x21,0x21,0xc1,0x00,0x00,0x83,0x84,0x84},
		{0x21,0x21,0xc3,0x00,0x00,0xc3,0x84,0x84},
		{0x21,0x21,0xc7,0x00,0x00,0xe3,0x84,0x84},
		{0x21,0x21,0xcf,0x00,0x00,0xf3,0x84,0x84},
		{0x21,0x21,0xcf,0x08,0x10,0xf3,0x84,0x84},
		{0x21,0x21,0xcf,0x18,0x18,0xf3,0x84,0x84},
		{0x21,0x21,0xdf,0x18,0x18,0xfb,0x84,0x84},
		{0x21,0x31,0xdf,0x18,0x18,0xfb,0x8c,0x84},
		{0x21,0x39,0xdf,0x18,0x18,0xfb,0x9c,0x84},
		{0x21,0x7d,0xdf,0x18,0x18,0xfb,0xbe,0x84},
		{0x21,0x7f,0xff,0x18,0x18,0xff,0xfe,0x84},
		{0xa3,0x7f,0xff,0x18,0x18,0xff,0xfe,0xc5},
		{0xa7,0xff,0xff,0x18,0x18,0xff,0xff,0xe5},
		{0xef,0xff,0xff,0x18,0x18,0xff,0xff,0xf7},
		{0xff,0xff,0xff,0x19,0x98,0xff,0xff,0xff},
		{0xff,0xff,0xff,0x59,0x9a,0xff,0xff,0xff},
		{0xff,0xff,0xff,0x5d,0xba,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xdd,0xbb,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xfd,0xbf,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x7e,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x3e,0xfe,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x1e,0xfc,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0x0e,0xfc,0xfe,0xff,0xff,0xff,0xff},
		{0x7e,0x06,0xfc,0xfc,0xff,0xff,0xff,0xff},
		{0x7e,0x06,0xec,0xf8,0xff,0xff,0xff,0xff},
		{0x7e,0x06,0x8c,0xf0,0xff,0xff,0xff,0xff},
		{0x7e,0x06,0x0c,0xf0,0xef,0xff,0xff,0xff},
		{0x7e,0x06,0x0c,0xf0,0xef,0xff,0xff,0xff},
		{0x7e,0x06,0x0c,0x70,0xe7,0xff,0xff,0xff},
		{0x7e,0x06,0x0c,0x30,0xe3,0xff,0xff,0xff},
		{0x7e,0x06,0x0c,0x30,0x61,0xff,0xff,0xff},
		{0x7e,0x06,0x0c,0x30,0x60,0x7f,0xff,0xff},
		{0x7e,0x06,0x0c,0x30,0x60,0x7e,0x7f,0xff},
		{0x7e,0x06,0x0c,0x30,0x60,0x76,0x7f,0xff},
		{0x7e,0x06,0x0c,0x30,0x60,0x66,0x3f,0xff},
		{0x7e,0x06,0x0c,0x30,0x60,0x66,0x3e,0x7f},
		{0x7e,0x06,0x0c,0x30,0x60,0x66,0x3e,0x3e},
		{0x7e,0x06,0x0c,0x30,0x60,0x66,0x3e,0x1c},
		{0x7e,0x06,0x0c,0x30,0x60,0x66,0x3c,0x0c},
		{0x7e,0x06,0x0c,0x30,0x60,0x66,0x3c,0x00}
};

const static uint8_t lenAnim3 = 104;
const static uint8_t anim3[][8]=
{
		{0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x08,0x08,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x08,0x08,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x08,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x1c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x14,0x08,0x14,0x00,0x00},
		{0x00,0x00,0x2a,0x1c,0x36,0x1c,0x2a,0x00},
		{0x00,0x41,0x14,0x22,0x08,0x22,0x14,0x41},
		{0x00,0x14,0x00,0x41,0x00,0x41,0x00,0x14},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x80,0x40,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x40,0x20,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x20,0x10,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x10,0x08,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x0c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x08,0x1c,0x08,0x00,0x00},
		{0x00,0x00,0x00,0x14,0x08,0x14,0x00,0x00},
		{0x00,0x00,0x1c,0x36,0x2a,0x36,0x1c,0x00},
		{0x00,0x08,0x36,0x22,0x41,0x22,0x36,0x08},
		{0x00,0x49,0x22,0x00,0x41,0x00,0x22,0x49},
		{0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x41},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x04,0x08,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x08,0x10,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x10,0x38,0x10,0x00,0x00,0x00},
		{0x00,0x00,0x28,0x10,0x28,0x00,0x00,0x00},
		{0x00,0x10,0x10,0x7c,0x10,0x10,0x00,0x00},
		{0x00,0x54,0x28,0x54,0x28,0x54,0x00,0x00},
		{0x00,0x54,0x38,0x6c,0x38,0x54,0x00,0x00},
		{0xc6,0xd6,0x38,0x6c,0x38,0xd6,0xc6,0x00},
		{0x44,0xd6,0x28,0x44,0x28,0xd6,0x44,0x00},
		{0xc6,0x92,0x00,0x44,0x00,0x92,0xc6,0x00},
		{0xc6,0xc6,0x00,0x00,0x00,0xc6,0xc6,0x00},
		{0x00,0x42,0x00,0x00,0x00,0x04,0x40,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x55,0x00,0x00,0x80,0x01,0x00,0x00,0xaa},
		{0x55,0x55,0x00,0xc0,0x03,0x00,0xaa,0xaa},
		{0x55,0x55,0x05,0xe0,0x07,0xa0,0xaa,0xaa},
		{0x55,0x55,0x05,0xf0,0x0f,0xa0,0xaa,0xaa},
		{0x55,0x55,0x05,0xf8,0x1f,0xa0,0xaa,0xaa},
		{0xaa,0xaa,0xfa,0x07,0xe0,0x5f,0x55,0x55},
		{0xaa,0xaa,0xfa,0x1f,0xf8,0x5f,0x55,0x55},
		{0x57,0x57,0x07,0xe0,0x07,0xe0,0xea,0xea},
		{0xbc,0xba,0xf9,0x1f,0xf8,0x9f,0x5d,0x3d},
		{0x43,0x45,0x06,0xe8,0x17,0x60,0xa2,0xc2},
		{0x53,0x55,0x06,0xe8,0x17,0x60,0xaa,0xca},
		{0xac,0xaa,0xf9,0x17,0xe8,0x9f,0x55,0x35},
		{0xac,0xae,0xff,0x17,0xe8,0xff,0x75,0x35},
		{0xec,0xee,0xff,0x97,0xe9,0xff,0x77,0x37},
		{0xef,0xef,0xff,0x9f,0xf9,0xff,0xf7,0xf7},
		{0xff,0xef,0xff,0xdf,0xfb,0xff,0xf7,0xff},
		{0xdf,0xef,0xff,0xdf,0xfb,0xff,0xf7,0xfb},
		{0xdf,0xff,0xff,0xdf,0xfb,0xff,0xff,0xfb},
		{0xdf,0xff,0xff,0xff,0xff,0xff,0xff,0xfb},
		{0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x04},
		{0x20,0x20,0x00,0x00,0x00,0x00,0x04,0x04},
		{0x20,0x20,0x40,0x00,0x00,0x02,0x04,0x04},
		{0x20,0x20,0xc0,0x00,0x00,0x03,0x04,0x04},
		{0x21,0x20,0xc0,0x00,0x00,0x03,0x04,0x84},
		{0x21,0x21,0xc0,0x00,0x00,0x03,0x84,0x84},
		{0x21,0x21,0xc1,0x00,0x00,0x83,0x84,0x84},
		{0x21,0x21,0xc3,0x00,0x00,0xc3,0x84,0x84},
		{0x21,0x21,0xc7,0x00,0x00,0xe3,0x84,0x84},
		{0x21,0x21,0xcf,0x00,0x00,0xf3,0x84,0x84},
		{0x21,0x21,0xcf,0x08,0x10,0xf3,0x84,0x84},
		{0x21,0x21,0xcf,0x18,0x18,0xf3,0x84,0x84},
		{0x21,0x21,0xdf,0x18,0x18,0xfb,0x84,0x84},
		{0x21,0x31,0xdf,0x18,0x18,0xfb,0x8c,0x84},
		{0x21,0x39,0xdf,0x18,0x18,0xfb,0x9c,0x84},
		{0x21,0x7d,0xdf,0x18,0x18,0xfb,0xbe,0x84},
		{0x21,0x7f,0xff,0x18,0x18,0xff,0xfe,0x84},
		{0xa3,0x7f,0xff,0x18,0x18,0xff,0xfe,0xc5},
		{0xa7,0xff,0xff,0x18,0x18,0xff,0xff,0xe5},
		{0xef,0xff,0xff,0x18,0x18,0xff,0xff,0xf7},
		{0xff,0xff,0xff,0x19,0x98,0xff,0xff,0xff},
		{0xff,0xff,0xff,0x59,0x9a,0xff,0xff,0xff},
		{0xff,0xff,0xff,0x5d,0xba,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xdd,0xbb,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xfd,0xbf,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x7e,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x3c,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x3c,0x7e,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x3c,0x66,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x3c,0x66,0x7e,0xff,0xff,0xff,0xff,0xff},
		{0x3c,0x66,0x7c,0xff,0xff,0xff,0xff,0xff},
		{0x3c,0x66,0x78,0xff,0xff,0xff,0xff,0xff},
		{0x3c,0x66,0x70,0x7f,0xff,0xff,0xff,0xff},
		{0x3c,0x66,0x60,0x3f,0xff,0xff,0xff,0xff},
		{0x3c,0x66,0x60,0x3b,0x7f,0xff,0xff,0xff},
		{0x3c,0x66,0x60,0x39,0x7f,0x7f,0xff,0xff},
		{0x3c,0x66,0x60,0x38,0x7f,0x7f,0x7f,0xff},
		{0x3c,0x66,0x60,0x38,0x7e,0x7f,0x3f,0xff},
		{0x3c,0x66,0x60,0x38,0x7c,0x7f,0x3f,0x7f},
		{0x3c,0x66,0x60,0x38,0x78,0x7f,0x3f,0x3f},
		{0x3c,0x66,0x60,0x38,0x70,0x7f,0x3f,0x1f},
		{0x3c,0x66,0x60,0x38,0x60,0x7f,0x3f,0x0f},
		{0x3c,0x66,0x60,0x38,0x60,0x6f,0x3f,0x07},
		{0x3c,0x66,0x60,0x38,0x60,0x67,0x3f,0x03},
		{0x3c,0x66,0x60,0x38,0x60,0x66,0x3f,0x01},
		{0x3c,0x66,0x60,0x38,0x60,0x66,0x3d,0x00},
		{0x3c,0x66,0x60,0x38,0x60,0x66,0x3c,0x00}
};

const static uint8_t lenSnake = 64;
const static uint8_t snake[][8]=
{
		{0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x10,0x18,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x1c,0x18,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x1c,0x1c,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x1c,0x1c,0x04,0x00,0x00},
		{0x00,0x00,0x00,0x1c,0x1c,0x0c,0x00,0x00},
		{0x00,0x00,0x00,0x1c,0x1c,0x1c,0x00,0x00},
		{0x00,0x00,0x00,0x1c,0x1c,0x3c,0x00,0x00},
		{0x00,0x00,0x00,0x1c,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x00,0x3c,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x20,0x3c,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x30,0x3c,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x38,0x3c,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x3c,0x3c,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x3e,0x3c,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x3e,0x3e,0x3c,0x3c,0x00,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3c,0x00,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3e,0x00,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3e,0x02,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3e,0x06,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3e,0x0e,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3e,0x1e,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3e,0x3e,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x3e,0x7e,0x00},
		{0x00,0x00,0x3e,0x3e,0x3e,0x7e,0x7e,0x00},
		{0x00,0x00,0x3e,0x3e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x00,0x3e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x00,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x40,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x60,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x70,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x78,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x7c,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x7e,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x7f,0x7e,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x7f,0x7f,0x7e,0x7e,0x7e,0x7e,0x00},
		{0x00,0x7f,0x7f,0x7f,0x7e,0x7e,0x7e,0x00},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7e,0x7e,0x00},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7e,0x00},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x00},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x01},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x03},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x07},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x0f},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x1f},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x3f},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0xff},
		{0x00,0x7f,0x7f,0x7f,0x7f,0x7f,0xff,0xff},
		{0x00,0x7f,0x7f,0x7f,0x7f,0xff,0xff,0xff},
		{0x00,0x7f,0x7f,0x7f,0xff,0xff,0xff,0xff},
		{0x00,0x7f,0x7f,0xff,0xff,0xff,0xff,0xff},
		{0x00,0x7f,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0x80,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0xc0,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0xe0,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0xf0,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0xf8,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0xfc,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0xfe,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
		{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}
};

#endif /* MAX7219_H_ */
