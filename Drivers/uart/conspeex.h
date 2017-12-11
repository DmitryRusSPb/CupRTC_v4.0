/*
 * conspeex.h
 *
 *  Created on: 22.05.2017
 *      Author: artem
 */

#ifndef CONSPEEX_H_
#define CONSPEEX_H_
#include <stdint.h>


#define STATE 1
#define TEXT1 2
#define TEXT2 3
#define BLOCK 4
#define SPEEX 5
#define ERROR 6
#define BUFLEN 100

typedef enum enumFormat{
	BIGEN,															// порядок хранения данных. Порядок от старшего к младшему (англ. big-endian — большим концом)
	LITTLEN															// Порядок от младшего к старшему (англ. little-endian)
}Format;

typedef struct{
	char command;
	char blockNumber[5];  													// один дополнительный байт взят, для удобства распарсивания
	char data[BUFLEN];
	char hesh[4];
} str_RecData;


typedef struct{
	uint8_t command;														// комманда
	uint16_t blockNumber;													// *****
	uint8_t data[BUFLEN];													// поле данных
} RecData;

/*****
 * Поле данных blockNumber для комманды SPEEX хранит номер блока
 * Для комманд TEXT1 и TEXT2 хранит кол-во символов, находящихся в поле data
 * Для комманды BLOCK хранит общее количество блоков
*/

uint8_t convert_hexchar_to_char(char );                                  	// перевод шестнадцатиричного символа в число типа данных uint16_t
uint8_t convert_hexstring(uint8_t*, char *, uint8_t, Format);		// перевод текстовой шестнадцатиричной строки в байтовый массив
void convert_text(uint8_t* , char* , uint8_t);								// копирование строки из char в uint16_t
char command_definer(char*);												// определитель комманд
str_RecData parsing_str_data(char*);										// распарсивание строки в структуру str_RecData
RecData convert_speex_str_to_data(str_RecData );						// перевод данных из структуры str_RecData в str_RecData
void print(RecData );													// ф-ия печати RecData(для отладки)
RecData parsing(char*, uint8_t);													// парсинг данных(пользовательская ф-ия)
uint16_t calcrc(char*, uint8_t );											// вычисление хеш-суммы




#endif /* CONSPEEX_H_ */
