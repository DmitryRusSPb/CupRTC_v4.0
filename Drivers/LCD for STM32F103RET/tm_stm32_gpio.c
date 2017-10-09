/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_gpio.h"

/* Private function */
static uint16_t GPIO_UsedPins[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};

/* Private functions */
void TM_GPIO_INT_EnableClock(GPIO_TypeDef* GPIOx);
void TM_GPIO_INT_DisableClock(GPIO_TypeDef* GPIOx);
void TM_GPIO_INT_Init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, TM_GPIO_Mode_t GPIO_Mode, TM_GPIO_PuPd_t GPIO_PuPd, TM_GPIO_Speed_t GPIO_Speed);

void TM_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TM_GPIO_Mode_t GPIO_Mode, TM_GPIO_PuPd_t GPIO_PuPd, TM_GPIO_Speed_t GPIO_Speed) {
	/* Check input */
	if (GPIO_Pin == 0x00) {
		return;
	}

	/* Enable clock for GPIO */
	TM_GPIO_INT_EnableClock(GPIOx);

	/* Do initialization */
	TM_GPIO_INT_Init(GPIOx, GPIO_Pin, GPIO_Mode, GPIO_PuPd, GPIO_Speed);
}

void TM_GPIO_InitAlternate(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,  TM_GPIO_PuPd_t GPIO_PuPd, TM_GPIO_Speed_t GPIO_Speed, uint8_t Alternate) {
	/* Check input */
	if (GPIO_Pin == 0x00) {
		return;
	}

	/* Enable GPIOx clock */
	TM_GPIO_INT_EnableClock(GPIOx);

	/* Set alternate functions for all pins */

	/* Do initialization */
	TM_GPIO_INT_Init(GPIOx, GPIO_Pin, TM_GPIO_Mode_AF, GPIO_PuPd, GPIO_Speed);
}


void TM_GPIO_SetPinAsInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void TM_GPIO_SetPinAsOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void TM_GPIO_SetPinAsAnalog(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void TM_GPIO_SetPinAsAlternate(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//void TM_GPIO_SetPullResistor(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TM_GPIO_PuPd_t GPIO_PuPd) {
//uint8_t pinpos;

/* Go through all pins */
//for (pinpos = 0; pinpos < 0x10; pinpos++) {
/* Check if pin available */
//	if ((GPIO_Pin & (1 << pinpos)) == 0) {
//		continue;
//	}

/* Set GPIO PUPD register */
//	GPIOx->PUPDR = (GPIOx->PUPDR & ~(0x03 << (2 * pinpos))) | ((uint32_t)(GPIO_PuPd << (2 * pinpos)));
//}
//}

void TM_GPIO_Lock(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	uint32_t d;

	/* Set GPIO pin with 16th bit set to 1 */
	d = 0x00010000 | GPIO_Pin;

	/* Write to LCKR register */
	GPIOx->LCKR = d;
	GPIOx->LCKR = GPIO_Pin;
	GPIOx->LCKR = d;

	/* Read twice */
	(void)GPIOx->LCKR;
	(void)GPIOx->LCKR;
}

uint16_t TM_GPIO_GetPinSource(uint16_t GPIO_Pin) {
	uint16_t pinsource = 0;

	/* Get pinsource */
	while (GPIO_Pin > 1) {
		/* Increase pinsource */
		pinsource++;
		/* Shift right */
		GPIO_Pin >>= 1;
	}

	/* Return source */
	return pinsource;
}

uint16_t TM_GPIO_GetPortSource(GPIO_TypeDef* GPIOx) {
	/* Get port source number */
	/* Offset from GPIOA                       Difference between 2 GPIO addresses */
	return ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));
}

/* Private functions */
void TM_GPIO_INT_EnableClock(GPIO_TypeDef* GPIOx) {
	/* Set bit according to the 1 << portsourcenumber */
	if(GPIOx == GPIOA)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	if(GPIOx == GPIOB)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	if(GPIOx == GPIOC)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	if(GPIOx == GPIOD)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}
}

void TM_GPIO_INT_DisableClock(GPIO_TypeDef* GPIOx) {
	/* Clear bit according to the 1 << portsourcenumber */
	if(GPIOx == GPIOA)
	{
		__HAL_RCC_GPIOA_CLK_DISABLE();
	}
	if(GPIOx == GPIOB)
	{
		__HAL_RCC_GPIOB_CLK_DISABLE();
	}
	if(GPIOx == GPIOC)
	{
		__HAL_RCC_GPIOC_CLK_DISABLE();
	}
	if(GPIOx == GPIOD)
	{
		__HAL_RCC_GPIOD_CLK_DISABLE();
	}
}
void TM_GPIO_INT_Init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, TM_GPIO_Mode_t GPIO_Mode, TM_GPIO_PuPd_t GPIO_PuPd, TM_GPIO_Speed_t GPIO_Speed) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_Mode;
	GPIO_InitStruct.Pull = GPIO_PuPd;
	GPIO_InitStruct.Speed = GPIO_Speed;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint16_t TM_GPIO_GetUsedPins(GPIO_TypeDef* GPIOx) {
	/* Return used */
	return GPIO_UsedPins[TM_GPIO_GetPortSource(GPIOx)];
}

uint16_t TM_GPIO_GetFreePins(GPIO_TypeDef* GPIOx) {
	/* Return free pins */
	return ~GPIO_UsedPins[TM_GPIO_GetPortSource(GPIOx)];
}
