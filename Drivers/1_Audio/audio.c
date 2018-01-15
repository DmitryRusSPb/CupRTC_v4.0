/*
 * audio.c
 *
 *  Created on: 21 дек. 2017 г.
 *      Author: root
 */
#include "audio.h"

__IO int16_t outBuffer[2][FRAME_SIZE];
__IO int16_t *outBuff;
__IO uint8_t startDecoding;
__IO uint16_t nbFrames;
SpeexBits bits;
// Параметры для функций библиотеки SPEEX
void *decState;
int enh;
char inputBytes[ENCODED_FRAME_SIZE];


void SpeexInit(void)
{
	speex_bits_init(&bits);
	decState = speex_decoder_init(&speex_nb_mode);
	speex_decoder_ctl(decState, SPEEX_SET_ENH, &enh);
}

void PlayMessage(unsigned char const *array, uint16_t frame_number)
{
	int i;
	uint16_t sample_index = 0;

	for(i=0;i<ENCODED_FRAME_SIZE; i++)
	{
		inputBytes[i] = array[sample_index];
		sample_index++;
	}
	speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
	speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[0]);

	for(i=0;i<ENCODED_FRAME_SIZE; i++)
	{
		inputBytes[i] = array[sample_index];
		sample_index++;
	}

	speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
	speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[1]);

	nbFrames++;

	while(nbFrames < frame_number)
	{
		if(startDecoding == 1)
		{
			for(i=0;i<ENCODED_FRAME_SIZE; i++)
			{
				inputBytes[i] = array[sample_index];
				sample_index++;
			}

			speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
			speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[0]);

			startDecoding = 0;
			nbFrames++;
		}
		if(startDecoding == 2)
		{
			for(i=0;i<ENCODED_FRAME_SIZE; i++)
			{
				inputBytes[i] = array[sample_index];
				sample_index++;
			}

			speex_bits_read_from(&bits, inputBytes, ENCODED_FRAME_SIZE);
			speex_decode_int(decState, &bits, (spx_int16_t*)outBuffer[1]);

			startDecoding = 0;
			nbFrames++;
		}

	}//end while

	sample_index = 0;
	nbFrames = 0;
	outBuff = outBuffer[0];
}

