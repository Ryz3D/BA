/*
 * log.c
 *
 *  Created on: Aug 25, 2024
 *      Author: mirco
 */

#include "log.h"

void Log_Save(Log_t *hlog, char *buffer);

void Log_Init(Log_t *hlog)
{
	// Provide default values
	if (hlog->flush_timeout == 0)
	{
		hlog->flush_timeout = 100;
	}

	// Init struct
	hlog->last_write = 0;
	hlog->hbuffer.buffer1 = hlog->buffer1;
	hlog->hbuffer.buffer2 = hlog->buffer2;
	hlog->hbuffer.buffer_len = LOG_BUFFER_LEN;
	hlog->hbuffer.element_size = sizeof(char);

	Double_Buffer_Init(&hlog->hbuffer);
}

void Log_Uninit(Log_t *hlog)
{
	Double_Buffer_Flush(&hlog->hbuffer);
	Log_Loop(hlog);
}

void Log_Loop(Log_t *hlog)
{
	if (hlog->hbuffer.write_index > 0 && HAL_GetTick() - hlog->last_write > hlog->flush_timeout)
	{
		hlog->last_write = HAL_GetTick();
		Double_Buffer_Flush(&hlog->hbuffer);
	}

	if (hlog->hbuffer.flag_save_buffer_1)
	{
		Log_Save(hlog, hlog->buffer1);
		hlog->hbuffer.flag_save_buffer_1 = 0;
	}
	if (hlog->hbuffer.flag_save_buffer_2)
	{
		Log_Save(hlog, hlog->buffer2);
		hlog->hbuffer.flag_save_buffer_2 = 0;
	}
	if (hlog->hbuffer.flag_overflow)
	{
		printf("(%lu) WARNING: Log_Loop: Log buffer overflow\r\n", HAL_GetTick());
	}
}

void Log_Char(Log_t *hlog, char c)
{
	if (c != '\0')
	{
		hlog->last_write = HAL_GetTick();
		*((char*)Double_Buffer_Current(&hlog->hbuffer)) = c;
		Double_Buffer_Increment(&hlog->hbuffer);
	}
}

void Log_Save(Log_t *hlog, char *buffer)
{
	if (hlog->hvsd != NULL)
	{
		SD_WriteBuffer(hlog->hvsd, hlog->hvsd->log_file_path, (void*)buffer, hlog->hbuffer.save_len);
	}

	if (hlog->huart != NULL)
	{
		HAL_UART_Transmit(hlog->huart, (uint8_t*)buffer, hlog->hbuffer.save_len, 100);
	}

	CDC_Transmit_FS((uint8_t*)buffer, hlog->hbuffer.save_len);
}
