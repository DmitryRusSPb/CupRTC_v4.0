/*
 * ws2812b.c
 *
 *  Created on: 21 дек. 2017 г.
 *      Author: root
 */

#ifdef WS2812B_ENABLE

#include "ws2812b.h"

extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;

extern uint8_t LED_BYTE_Buffer[QUANTITY_OF_LED*24 + TRAILING_BYTES];

uint8_t LED_BYTE_Buffer[QUANTITY_OF_LED*24 + TRAILING_BYTES];

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

	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	buffersize = (len*24)+TRAILING_BYTES;
	// reset buffer memory index
	memaddr = 0;
	// reset led index
	led = 0;

	// fill transmit buffer with correct compare values to achieve
	// correct pulse widths according to color values
	while (len)
	{
		for (j = 0; j < 8; j++)									// GREEN data
		{
			if ( (greenLED<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// RED data
		{
			if ( (redLED<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// BLUE data
		{
			if ( (blueLED<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
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
//void WS2812_send_group_standart(void)
//{
//	uint16_t buffersize;
//	uint16_t memaddr;
//	uint8_t j;
//	uint8_t numLedOfGroup[3];
//
//	numLedOfGroup[0] = NUM_LED_OF_GROUP_ONE;
//	numLedOfGroup[1] = NUM_LED_OF_GROUP_TWO;
//	numLedOfGroup[2] = NUM_LED_OF_GROUP_THREE;
//
//	buffersize = (NUM_LED_OF_GROUP_ONE + NUM_LED_OF_GROUP_ONE + NUM_LED_OF_GROUP_THREE)*24 + TRAILING_BYTES;
//	memaddr = 0;
//
//	for(uint8_t i = 0; i < 255; i++)
//		{
//			while (numLedOfGroup[i])
//			{
//				for (j = 0; j < 8; j++)									// GREEN data
//				{
//					if ( (0<<j) & 0x80 )				// data sent MSB first, j = 0 is MSB j = 7 is LSB
//					{
//						LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
//					}
//					else
//					{
//						LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
//					}
//					memaddr++;
//				}
//
//				for (j = 0; j < 8; j++)									// RED data
//				{
//					if ( (i<<j) & 0x80 )				// data sent MSB first, j = 0 is MSB j = 7 is LSB
//					{
//						LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
//					}
//					else
//					{
//						LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
//					}
//					memaddr++;
//				}
//
//				for (j = 0; j < 8; j++)									// BLUE data
//				{
//					if ( (groupsColor[i][2]<<j) & 0x80 )				// data sent MSB first, j = 0 is MSB j = 7 is LSB
//					{
//						LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
//					}
//					else
//					{
//						LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
//					}
//					memaddr++;
//				}
//				numLedOfGroup[i]--;
//			}
//		}
//		// add needed delay at end of byte cycle, pulsewidth = 0
//		while(memaddr < buffersize)
//		{
//			LED_BYTE_Buffer[memaddr] = 0;
//			memaddr++;
//		}
//		// Запускаем передачу и включаем шим
//		HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)LED_BYTE_Buffer, buffersize);
//		while(HAL_DMA_GetState(&hdma_tim3_ch1_trig) == HAL_DMA_STATE_BUSY) osDelay(10);
//
//}

void WS2812_send_group_short(uint8_t (*groupsColor)[3], uint8_t quantityOfGroups)
{
	uint16_t buffersize;
	uint16_t memaddr;
	uint8_t j;
	uint8_t numLedOfGroup[4];

	numLedOfGroup[0] = NUM_LED_OF_GROUP_ONE;
	numLedOfGroup[1] = NUM_LED_OF_GROUP_TWO;
	numLedOfGroup[2] = NUM_LED_OF_GROUP_THREE;
	numLedOfGroup[3] = NUM_LED_OF_GROUP_FOUR;


	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	buffersize = (NUM_LED_OF_GROUP_ONE + NUM_LED_OF_GROUP_ONE + NUM_LED_OF_GROUP_THREE + NUM_LED_OF_GROUP_FOUR)*24 + TRAILING_BYTES;
	memaddr = 0;
	for(uint8_t i = 0; i < QUANTITY_OF_GROUPS; i++)
	{
		while (numLedOfGroup[i])
		{
			for (j = 0; j < 8; j++)									// GREEN data
			{
				if ( (groupsColor[0][1]<<j) & 0x80 )				// data sent MSB first, j = 0 is MSB j = 7 is LSB
				{
					LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
				}
				else
				{
					LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
				}
				memaddr++;
			}

			for (j = 0; j < 8; j++)									// RED data
			{
				if ( (groupsColor[0][0]<<j) & 0x80 )				// data sent MSB first, j = 0 is MSB j = 7 is LSB
				{
					LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
				}
				else
				{
					LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
				}
				memaddr++;
			}

			for (j = 0; j < 8; j++)									// BLUE data
			{
				if ( (groupsColor[0][2]<<j) & 0x80 )				// data sent MSB first, j = 0 is MSB j = 7 is LSB
				{
					LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
				}
				else
				{
					LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
				}
				memaddr++;
			}
			numLedOfGroup[i]--;
		}
	}
	// add needed delay at end of byte cycle, pulsewidth = 0
	while(memaddr < buffersize)
	{
		LED_BYTE_Buffer[memaddr] = 0;
		memaddr++;
	}
	// Запускаем передачу и включаем шим
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)LED_BYTE_Buffer, buffersize);
	while(HAL_DMA_GetState(&hdma_tim3_ch1_trig) == HAL_DMA_STATE_BUSY)
		osDelay(10);
}


void WS2812_send_group(uint8_t redLED1, uint8_t greenLED1, uint8_t blueLED1,
		uint8_t redLED2, uint8_t greenLED2, uint8_t blueLED2,
		uint8_t redLED3, uint8_t greenLED3, uint8_t blueLED3,
		uint8_t redLED4, uint8_t greenLED4, uint8_t blueLED4)
{
	uint16_t buffersize;
	uint16_t memaddr;
	uint8_t j;
	uint8_t numLedOfGroup[4];

	numLedOfGroup[0] = NUM_LED_OF_GROUP_ONE;
	numLedOfGroup[1] = NUM_LED_OF_GROUP_TWO;
	numLedOfGroup[2] = NUM_LED_OF_GROUP_THREE;
	numLedOfGroup[3] = NUM_LED_OF_GROUP_FOUR;

	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	buffersize = (NUM_LED_OF_GROUP_ONE + NUM_LED_OF_GROUP_ONE + NUM_LED_OF_GROUP_THREE + NUM_LED_OF_GROUP_FOUR)*24 + TRAILING_BYTES;
	memaddr = 0;

	while (numLedOfGroup[0])
	{
		for (j = 0; j < 8; j++)									// GREEN data
		{
			if ( (greenLED1<<j) & 0x80 )					// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// RED data
		{
			if ( (redLED1<<j) & 0x80 )					// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// BLUE data
		{
			if ( (blueLED1<<j) & 0x80 )					// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}
		numLedOfGroup[0]--;
	}

	while (numLedOfGroup[1])
	{
		for (j = 0; j < 8; j++)									// GREEN data
		{
			if ( (greenLED2<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// RED data
		{
			if ( (redLED2<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// BLUE data
		{
			if ( (blueLED2<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}
		numLedOfGroup[1]--;
	}

	while (numLedOfGroup[2])
	{
		for (j = 0; j < 8; j++)									// GREEN data
		{
			if ( (greenLED3<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// RED data
		{
			if ( (redLED3<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// BLUE data
		{
			if ( (blueLED3<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}
		numLedOfGroup[2]--;
	}

	while (numLedOfGroup[3])
	{
		for (j = 0; j < 8; j++)									// GREEN data
		{
			if ( (greenLED4<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// RED data
		{
			if ( (redLED4<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)									// BLUE data
		{
			if ( (blueLED4<<j) & 0x80 )							// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_HIGH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = PWM_FOR_RGB_LOW;		// compare value for logical 0
			}
			memaddr++;
		}
		numLedOfGroup[3]--;
	}
	// add needed delay at end of byte cycle, pulsewidth = 0
	while(memaddr < buffersize)
	{
		LED_BYTE_Buffer[memaddr] = 0;
		memaddr++;
	}

	// Запускаем передачу и включаем шим
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)LED_BYTE_Buffer, buffersize);
	while(HAL_DMA_GetState(&hdma_tim3_ch1_trig) == HAL_DMA_STATE_BUSY)
		osDelay(10);
}

#endif
