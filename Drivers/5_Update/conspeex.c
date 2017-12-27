/*
 * conspeex.c
 *
 *  Created on: 22.05.2017
 *      Author: artem
 */

#include "conspeex.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

char buffer[BUFLEN];
uint8_t UINTBUFFER[4];

uint8_t convert_hexchar_to_char(char c){   									// перевод строчного символа в число
	switch(c){
	case '0':
		return 0;
		break;

	case '1':
		return 1;
		break;

	case '2':
		return 2;
		break;

	case '3':
		return 3;
		break;

	case '4':
		return 4;
		break;

	case '5':
		return 5;
		break;

	case '6':
		return 6;
		break;

	case '7':
		return 7;
		break;

	case '8':
		return 8;
		break;

	case '9':
		return 9;
		break;

	case 'A':
		return 10;
		break;

	case 'B':
		return 11;
		break;

	case 'C':
		return 12;
		break;

	case 'D':
		return 13;
		break;

	case 'E':
		return 14;
		break;

	case 'F':
		return 15;
		break;

	default:
		return 0;
		break;
	}
	return 0;
}


uint8_t convert_hexstring(uint8_t* ret, char* dat, uint8_t n, Format format){
	switch(format){
	case LITTLEN:
		for(uint8_t i=0; i<(n>>1); i++){
			ret[(n>>1)-i-1] = convert_hexchar_to_char(dat[2*i])*16 + convert_hexchar_to_char(dat[2*i+1]);
		}
		return 1;
		break;

	case BIGEN:
		for(uint8_t i=0; i<(n>>1); i++){
			ret[i] = convert_hexchar_to_char(dat[2*i])*16 + convert_hexchar_to_char(dat[2*i+1]);
		}
		return 1;
		break;

	default:
		return 0;
		break;
	}
}


void convert_text(uint8_t* str1, char* str2, uint8_t n){                               	// ф-ия копирования текста, аналогичная strcpy, для unsigned char*
	for(uint8_t i=0;i<n;i++){
		str1[i]=(uint8_t)str2[i];
	}
	str1[n]=(uint8_t)'\0';
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



char command_definer(char *dat){                                                 	// определение комманды в строке данных

	if(strncmp(dat, "STATE", 5)==0) {												// сравнение первых 5 символов строки
		return STATE;
	}

	if(strncmp(dat, "TEXT1", 5)==0) {
		return TEXT1;
	}

	if(strncmp(dat, "TEXT2", 5)==0) {
		return TEXT2;
	}

	if(strncmp(dat, "BLOCK", 5)==0) {
		return BLOCK;
	}

	if(strncmp(dat, "SPEEX", 5)==0) {
		return SPEEX;
	}

	return -1;
}


str_RecData parsing_str_data(char* dat){                     // распарсивание данных сырых данных и заполнение структуры str_RecData
	strcpy(buffer, dat+1);
	char *tempstr = strtok(buffer," ");							// ф-ия strtok при каждом своем вызове возвращает элемент строки разделенный символом, в данном случае пробелом
	str_RecData spdata;

	switch(command_definer(tempstr)){							// в зависимости, что вернет command_definer, выполняется какое-то действие

	case STATE:													// у комманды STATE не заполнено поле blockNumber
		spdata.command=STATE;

		tempstr=strtok(NULL, " ");								// следующий вызов strtok

		strncpy(spdata.data, tempstr, 40);

		tempstr=strtok(NULL, " >");								// следующий вызов strtok c другим разделителем
		strncpy(spdata.hesh, tempstr, 4);


		break;

	case TEXT1:													// у комманды TEXT1 и TEXT2 не заполнено поле blockNumber
		spdata.command=TEXT1;

		tempstr=strtok(NULL, "&");

		if(strlen(tempstr)<BUFLEN){
			strcpy(spdata.data, tempstr);
			tempstr=strtok(NULL, " &>");
			strcpy(spdata.hesh, tempstr);
		}
		else
			spdata.command=ERROR;

		break;


	case TEXT2:
		spdata.command=TEXT2;

		tempstr=strtok(NULL, "&");

		if(strlen(tempstr)<BUFLEN){
			strcpy(spdata.data, tempstr);
			tempstr=strtok(NULL, " &>");
			strcpy(spdata.hesh, tempstr);
		}
		else
			spdata.command=ERROR;

		break;


	case BLOCK:
		spdata.command=BLOCK;

		tempstr=strtok(NULL, " ");
		strncpy(spdata.blockNumber, tempstr, 4);

		tempstr=strtok(NULL, " >");
		strncpy(spdata.hesh, tempstr, 4);

		break;


	case SPEEX:
		spdata.command=SPEEX;

		tempstr=strtok(NULL, " ");
		strcpy(spdata.blockNumber, tempstr);

		tempstr=strtok(NULL, " ");
		strcpy(spdata.data, tempstr);

		tempstr=strtok(NULL, " >");
		strcpy(spdata.hesh, tempstr);
		break;


	default:
		spdata.command=ERROR;
		break;

	}
	return spdata;
}



RecData convert_speex_str_to_data(str_RecData data_str){
	RecData ret;																	// возвращаемая структура

	switch(data_str.command){

	case SPEEX:
		ret.command=(uint16_t)data_str.command;
		convert_hexstring(UINTBUFFER, data_str.blockNumber, 4, LITTLEN);
		ret.blockNumber=*(uint16_t*)UINTBUFFER;
		convert_hexstring(ret.data , data_str.data, 40, BIGEN);
		break;

	case BLOCK:
		ret.command=(uint16_t)data_str.command;
		convert_hexstring(UINTBUFFER ,data_str.blockNumber, 4, LITTLEN);							// количества блоков в BLOCK хранится в поле blockNumber, а не data
		ret.blockNumber=*(uint16_t*)UINTBUFFER;;
		break;

	case TEXT1:
		ret.command=(uint16_t)data_str.command;                                     // количество символов текста зранится в боле blockNumber
		ret.blockNumber=(uint16_t)strlen(data_str.data);
		convert_text(ret.data, data_str.data, strlen(data_str.data));

		break;

	case TEXT2:
		ret.command=(uint16_t)data_str.command;
		ret.blockNumber=(uint16_t)strlen(data_str.data);
		convert_text(ret.data, data_str.data, strlen(data_str.data));

		break;

	case STATE:																		// место находится в первом элементе массива
		ret.command=(uint16_t)data_str.command;
		convert_hexstring(UINTBUFFER ,data_str.data, 2, LITTLEN);
		ret.data[0]=*UINTBUFFER;
		ret.blockNumber=(uint16_t)0;


		break;

	default:
		ret.command=ERROR;
		break;
	}
	return ret;
}


uint16_t calcrc(char *ptr, uint8_t count)   // вычисление хеш-суммы crc16Xmodem
{
	uint16_t crc;
	uint8_t i;
    crc = 0;
    while (count--!=0)
    {
        crc = crc ^ (uint8_t) *ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;

        } while(--i);
    }
    return (crc);
}

RecData parsing(char* data, uint8_t n){												// перевод данных из str_RecData в RecData  и проверка хеш-суммы
	RecData sp;
	if(n<=BUFLEN){
		str_RecData data_str = parsing_str_data(data);
		uint16_t temp=0;
		switch(data_str.command){

		case STATE:
			temp = calcrc(data_str.data, 2);									// вычисление хеш-суммы
			break;

		case BLOCK:
			temp = calcrc(data_str.blockNumber, 4);								// вычисление хеш-суммы
			break;

		case SPEEX:
			strncpy(buffer, data_str.blockNumber, 5);							// 5 байт, один для символа конца строки(взят из-за buffer, чтобы определить, где заканчивается blockNumber), потом этот символ конца строки стирается strncat
			strncat(buffer, data_str.data, 40);
			temp = calcrc(buffer, 44);											// вычисление хеш-суммы
			break;

		case TEXT1:
			temp = calcrc(data_str.data, strlen(data_str.data));				// вычисление хеш-суммы
			break;

		case TEXT2:
			temp = calcrc(data_str.data, strlen(data_str.data));				// вычисление хеш-суммы
			break;

		default:																// если пришла какая-то другая комманда, вернуть структуру с коммандой ERROR
			sp.command=ERROR;
			return sp;
		}

		convert_hexstring(UINTBUFFER, data_str.hesh, 4, LITTLEN);
		uint16_t hesh= *(uint16_t*)UINTBUFFER;
		if(temp == hesh){														// если хеш сумма не совпадает, возвращается структура с коммандой ERROR
			sp=convert_speex_str_to_data(data_str);
			return sp;
		}
		else{
			sp.command=ERROR;
			return sp;
		}
	}
	else{
		sp.command=ERROR;
		return sp;
	}
}


void print(RecData sp){													// ф-ия для отладки

	printf("%d\n", sp.command);

	if(sp.command==BLOCK)	printf("%X\n", sp.blockNumber);
	else printf("%X\n", sp.blockNumber);

	if(sp.command==SPEEX){
		for(uint8_t i=0;i<20;i++){
			printf("%X", sp.data[i]);
			printf(" ");

		}
	}

	if(sp.command==STATE){
		printf("%u", sp.data[0]);
	}
	if((sp.command==TEXT1)||(sp.command==TEXT2)){
		for(uint8_t i=0;i<sp.blockNumber;i++){
			printf("%c", sp.data[i]);
		}

	}

	if(sp.command==ERROR)  printf("%s\n","Error");


}


