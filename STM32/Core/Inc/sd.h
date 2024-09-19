/*
 * sd.h
 *
 * Helper file for SD card access
 *
 *  Created on: Jul 26, 2024
 *      Author: mirco
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"
#include "fatfs.h"
#include "data_points.h"

#define CONFIG_FILE_PATH "config.txt"
#define DIR_FORMAT "%04hu-%02hhu-%02hhu_%lu"
#define A_FILE_FORMAT DIR_FORMAT "/a_%li.bin"
#define P_FILE_FORMAT DIR_FORMAT "/p_%li.bin"
#define LOG_FILE_FORMAT DIR_FORMAT "/_log.txt"

#define PATH_LEN 50

typedef struct
{
	GPIO_TypeDef *Detect_GPIO_Port;
	uint16_t Detect_Pin;

	FATFS *fatfs;
	FIL *fatfs_file;
	const TCHAR *fatfs_path;

	uint32_t dir_num, page_num;

	uint16_t date_year;
	uint8_t date_month, date_day;

	TCHAR dir_path[PATH_LEN];
	TCHAR a_file_path[PATH_LEN];
	TCHAR p_file_path[PATH_LEN];
	TCHAR log_file_path[PATH_LEN];

	a_data_header_t a_header;
	p_data_header_t p_header;
} Vera_SD_t;

HAL_StatusTypeDef SD_Init(Vera_SD_t *hsd, uint8_t do_format);
HAL_StatusTypeDef SD_InitDir(Vera_SD_t *hsd);
HAL_StatusTypeDef SD_UpdateFilepaths(Vera_SD_t *hsd);
HAL_StatusTypeDef SD_NewPage(Vera_SD_t *hsd);
HAL_StatusTypeDef SD_Uninit(Vera_SD_t *hsd);
HAL_StatusTypeDef SD_TouchFile(Vera_SD_t *hsd, TCHAR *path);
uint8_t SD_FileExists(Vera_SD_t *hsd, TCHAR *path);
HAL_StatusTypeDef SD_ReadBuffer(Vera_SD_t *hsd, TCHAR *path, void *data, UINT size, UINT *size_read);
HAL_StatusTypeDef SD_WriteBuffer(Vera_SD_t *hsd, TCHAR *path, void *data, UINT size);

#endif /* INC_SD_H_ */
