/*
 * otherFunctions.c
 *
 *  Created on: 21 дек. 2017 г.
 *      Author: root
 */
#include "otherFunctions.h"

// Размер(в байтах) всех полученных данных
uint16_t allDataSize;
// Число страниц флеш памяти
uint8_t pageNum = 0;
// Сообщает нам о том, была ли уже очищена первая страница
uint8_t firstErase = 0;
// Массив, хранящий принимаемые данные до парсинга
uint8_t dataBuffer[60];


GPIO_PinState AntiContactBounce(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	uint8_t numOfPolls;
	numOfPolls = 1;
	while(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
	{
		numOfPolls++;
		osDelay(1);
		if(numOfPolls >= NUMBER_OF_POLLS)
			return GPIO_PIN_SET;
	}
	return GPIO_PIN_RESET;
}

/*----------------------------------------------------------------------------*/
/*--------------------------------FLASH---------------------------------------*/
/*----------------------------------------------------------------------------*/
// Функция записи в флеш-память
AnswerStatus SU_FLASH_Save_User_Data(RecData parsData, uint8_t numReceivedBytes)
{
	// Размер принятого сообщения
	uint8_t messageSize = 0;

	uint32_t convert = 0;

	uint8_t multipleMessageSize = 0;

	switch (parsData.command)
	{
	// Занятое место
	case 1:
		// Место будет занимать в памяти 4 байта
		messageSize = 4;
		allDataSize += messageSize;
		break;
		// Тексты 1 и 2
	case 2:
	case 3:
		// Вычитаем 15 из общего числа полученных байт, так как это не сами данные
		// ,а лишь информация о роде данных.
		messageSize = numReceivedBytes - 15;
		multipleMessageSize = messageSize;
		allDataSize += 80;
		// Прибавляем 4, так как нужно учесть количество символов
		// ,которое храниться в памяти в 4 байтах
		allDataSize += 4;
		break;
		// Количество фреймов (блоки по 20 байт)
	case 4:
		// Количество фреймов будет занимать в памяти 4 байта
		messageSize = 4;
		allDataSize += messageSize;
		break;
		// Аудиоданные
	case 5:
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
	switch (parsData.command)
	{
	case STATE:
		convert |= parsData.data[0];
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, START_FLASH_PAGE, convert);
		break;
	case TEXT1:
		WriteToFlash(SIZE_TEXT1_address, SIZE_TEXT1_MEMORY_SIZE, (uint8_t*)&multipleMessageSize);
		WriteToFlash(TEXT1_address, (uint32_t)multipleMessageSize, parsData.data);
		break;
	case TEXT2:
		WriteToFlash(SIZE_TEXT2_address, SIZE_TEXT2_MEMORY_SIZE, (uint8_t*)&multipleMessageSize);
		WriteToFlash(TEXT2_address, (uint32_t)multipleMessageSize, parsData.data);
		break;
	case BLOCK:
		convert |= parsData.blockNumber;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BLOCK_address, convert);
		break;
	case SPEEX:
		WriteToFlash(SPEEX_address + parsData.blockNumber*20, 20, parsData.data);
		break;
	}
	HAL_FLASH_Lock();
	return Flash_OK;
}

void WriteToFlash(uint32_t writeAddress, uint32_t sizeData, uint8_t *data)
{
	// Записываем по 4 байта данные из буфера
	for(uint8_t i = 0; i < sizeData; i+=4)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writeAddress, *(uint32_t*)(data)) == HAL_OK)
		{
			writeAddress ++;
			data += 4;
		}
		// Закрываем флеш в случае неудачной записи
		else
		{
			HAL_FLASH_Lock();
		}
	}
}
