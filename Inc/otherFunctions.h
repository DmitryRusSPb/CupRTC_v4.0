/*
 * otherFunctions.h
 *
 *  Created on: 21 дек. 2017 г.
 *      Author: root
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "defines.h"
#include "conspeex.h"

// Структурка для выявления места возникновения ошибки при записи прошивки в память
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

/* @brief Brief of AntiContactBounce
 *
 * Функция для проверки контактов на дребезг
 * На вход подаётся имя порта и номер пина
 * На выходе получаем 1 или 0
 */
GPIO_PinState AntiContactBounce(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* @brief Brief of SuFlashSaveUserData
 *
 * Запись данных во flash память
 */
AnswerStatus SU_FLASH_Save_User_Data(RecData parsData, uint8_t numReceivedBytes);

void WriteToFlash(uint32_t writeAddress, uint32_t sizeData, uint8_t *data);

/* OTHERFUNCTIONS_H_ */
