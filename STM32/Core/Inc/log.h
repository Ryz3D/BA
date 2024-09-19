/*
 * logger.h
 *
 *  Created on: Aug 25, 2024
 *      Author: mirco
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "sd.h"
#include "double_buffering.h"
#include "stm32f7xx_hal.h"
#include "usbd_cdc_if.h"

#define LOG_BUFFER_LEN 2048

typedef struct {
	Vera_SD_t *hvsd;
	UART_HandleTypeDef *huart;
	uint32_t flush_timeout;
	uint32_t last_write;
	char buffer1[LOG_BUFFER_LEN];
	char buffer2[LOG_BUFFER_LEN];
	Double_Buffer_t hbuffer;
} Log_t;

void Log_Init(Log_t *hlog);
void Log_Uninit(Log_t *hlog);
void Log_Loop(Log_t *hlog);
void Log_Char(Log_t *hlog, char c);

#endif /* INC_LOG_H_ */
