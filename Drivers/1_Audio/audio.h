/*
 * audio.h
 *
 *  Created on: 21 дек. 2017 г.
 *      Author: root
 */

#ifndef AUDIO_H_
#define AUDIO_H_

/* @brief Brief of PlayMessage
 *
 * Функция воспроизведения аудиофайла
 * Функция принимает указатель на массив с данными
 * и переменную с количеством фреймов (отрезков по 20 байт)
 */
void PlayMessage(unsigned char const *array, uint16_t frame_number);

/* @brief Brief of SpeexInit
 *
 * Производит инитиализацию speex декодера
 */
void SpeexInit(void);


#endif /* AUDIO_H_ */
