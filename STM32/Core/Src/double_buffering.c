/*
 * double_buffering.c
 *
 * Manages flags and pointers for double buffering
 *
 *  Created on: Aug 18, 2024
 *      Author: mirco
 */

#include "double_buffering.h"

void Double_Buffer_Init(Double_Buffer_t *hbuffer)
{
	if (hbuffer->buffer1 == NULL || hbuffer->buffer2 == NULL)
	{
		printf("(%lu) WARNING: Double_Buffer_Init: Null pointer in buffer1/buffer2\r\n", HAL_GetTick());
	}

	// Provide default values
	if (hbuffer->element_size == 0)
	{
		hbuffer->element_size = 1;
	}

	// Init struct
	hbuffer->buffer_current = hbuffer->buffer1;
	hbuffer->write_index = 0;
	hbuffer->flag_save_buffer_1 = 0;
	hbuffer->flag_save_buffer_2 = 0;
	hbuffer->flag_overflow = 0;
}

volatile void* Double_Buffer_Current(Double_Buffer_t *hbuffer)
{
	return hbuffer->buffer_current + hbuffer->element_size * hbuffer->write_index;
}

void Double_Buffer_Increment(Double_Buffer_t *hbuffer)
{
	// If next index would overflow
	if (++hbuffer->write_index >= hbuffer->buffer_len)
	{
		// Typically save entire buffer
		hbuffer->save_len = hbuffer->buffer_len;
		// Switch buffers
		hbuffer->write_index = 0;
		// If buffer_2 has been filled
		if (hbuffer->buffer_current == hbuffer->buffer2)
		{
			// If buffer_1 has been emptied
			if (!hbuffer->flag_save_buffer_1)
			{
				// buffer_2 full
				hbuffer->flag_save_buffer_2 = 1;
				// Reset buffer_1 to use
				hbuffer->buffer_current = hbuffer->buffer1;
			}
			else
			{
				// Both buffers are full, flag as overflow
				hbuffer->flag_overflow = 1;
			}
		}
		// If buffer_1 has been filled
		else
		{
			// If buffer_2 has been emptied
			if (!hbuffer->flag_save_buffer_2)
			{
				// buffer_1 full
				hbuffer->flag_save_buffer_1 = 1;
				// Reset buffer_2 to use
				hbuffer->buffer_current = hbuffer->buffer2;
			}
			else
			{
				// Both buffers are full, flag as overflow
				hbuffer->flag_overflow = 1;
			}
		}
	}
}

void Double_Buffer_Flush(Double_Buffer_t *hbuffer)
{
	uint32_t flush_len = hbuffer->write_index;
	if (hbuffer->write_index == 0)
	{
		return;
	}
	// Increment with write index at end, forcing a buffer save
	hbuffer->write_index = hbuffer->buffer_len;
	Double_Buffer_Increment(hbuffer);
	hbuffer->save_len = flush_len;
}
