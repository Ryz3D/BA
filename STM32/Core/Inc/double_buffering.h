/*
 * double_buffering.h
 *
 * Manages flags and pointers for double buffering
 *
 *  Created on: Aug 18, 2024
 *      Author: mirco
 */

#ifndef INC_DOUBLE_BUFFERING_H_
#define INC_DOUBLE_BUFFERING_H_

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"

typedef struct
{
	uint32_t element_size;
	uint32_t buffer_len;
	volatile void *buffer1;
	volatile void *buffer2;
	volatile void *buffer_current;
	volatile uint32_t write_index;
	volatile uint32_t save_len;
	volatile uint8_t flag_save_buffer_1;
	volatile uint8_t flag_save_buffer_2;
	volatile uint8_t flag_overflow;
} Double_Buffer_t;

void Double_Buffer_Init(Double_Buffer_t *hbuffer);
volatile void *Double_Buffer_Current(Double_Buffer_t *hbuffer);
void Double_Buffer_Increment(Double_Buffer_t *hbuffer);
void Double_Buffer_Flush(Double_Buffer_t *hbuffer);

#endif /* INC_DOUBLE_BUFFERING_H_ */
