/*
 * Copyright (c) 2024 Mirco Heitmann
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * sd.c
 *
 * Helper file for SD card access
 */

#include "sd.h"

HAL_StatusTypeDef SD_Init(Vera_SD_t *hsd, uint8_t do_format)
{
	// Init struct
	hsd->dir_num = 0;
	hsd->page_num = 0;
	hsd->date_year = 0;
	hsd->date_month = 0;
	hsd->date_day = 0;

	hsd->dir_path[0] = '\0';
	hsd->a_file_path[0] = '\0';
	hsd->p_file_path[0] = '\0';
	hsd->log_file_path[0] = '\0';

	// Check if SD card detected
	if (HAL_GPIO_ReadPin(hsd->Detect_GPIO_Port, hsd->Detect_Pin) == GPIO_PIN_SET)
	{
		printf("(%lu) WARNING: SD_Init: No SD card detected!\r\n", HAL_GetTick());
	}

	// Try mounting file system
	if (f_mount(hsd->fatfs, hsd->fatfs_path, 0) != FR_OK)
	{
		printf("(%lu) ERROR: SD_Init: Failed to mount\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	if (do_format)
	{
		printf("(%lu) Formatting SD card...\r\n", HAL_GetTick());
		// Formatting working buffer
		uint8_t rtext[_MAX_SS];
		if (f_mkfs(hsd->fatfs_path, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
		{
			printf("(%lu) ERROR: SD_Init: Failed to create FAT volume (can be caused by code generation)\r\n", HAL_GetTick());
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

// Create new measurement directory
HAL_StatusTypeDef SD_InitDir(Vera_SD_t *hsd)
{
	// Open root dir
	DIR dir_root;
	if (f_opendir(&dir_root, "") != FR_OK)
	{
		printf("(%lu) ERROR: SD_Init: Root dir open failed (can be caused by code generation)\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	// Iterate root dir
	FILINFO file_info;
	hsd->dir_num = 0;
	while (f_readdir(&dir_root, &file_info) == FR_OK)
	{
		if (file_info.fname[0] == '\0')
		{
			break;
		}
		if (file_info.fattrib & AM_DIR)
		{
			// Test directory names for format
			uint16_t test_y = 0;
			uint8_t test_m = 0, test_d = 0;
			uint32_t test_num = 0;
			sscanf(file_info.fname, DIR_FORMAT, &test_y, &test_m, &test_d, &test_num);
			if (test_y == hsd->date_year && test_m == hsd->date_month && test_d == hsd->date_day)
			{
				// Find maximum "num" for current date
				hsd->dir_num = test_num > hsd->dir_num ? test_num : hsd->dir_num;
			}
		}
	}
	f_closedir(&dir_root);
	// Increment "num" above highest found value
	hsd->dir_num++;

	// Set new dir path
	sprintf(hsd->dir_path, DIR_FORMAT, hsd->date_year, hsd->date_month, hsd->date_day, hsd->dir_num);
	// Create new dir
	FRESULT res;
	if ((res = f_mkdir(hsd->dir_path)) != FR_OK)
	{
		printf("(%lu) ERROR: SD_Init: Create dir (\"%s\") failed\r\n", HAL_GetTick(), hsd->dir_path);
		return HAL_ERROR;
	}

	// Set new log file path
	sprintf(hsd->log_file_path, LOG_FILE_FORMAT, hsd->date_year, hsd->date_month, hsd->date_day, hsd->dir_num);
	// Create log file
	if (SD_TouchFile(hsd, hsd->log_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: SD_Init: Log file (\"%s\") touch failed\r\n", HAL_GetTick(), hsd->log_file_path);
		return HAL_ERROR;
	}

	return HAL_OK;
}

// Update paths of data files, create new files if needed
HAL_StatusTypeDef SD_UpdateFilepaths(Vera_SD_t *hsd)
{
	// Set new file paths
	sprintf(hsd->a_file_path, A_FILE_FORMAT, hsd->date_year, hsd->date_month, hsd->date_day, hsd->dir_num, hsd->page_num);
	sprintf(hsd->p_file_path, P_FILE_FORMAT, hsd->date_year, hsd->date_month, hsd->date_day, hsd->dir_num, hsd->page_num);

	// Create new files
	if (SD_TouchFile(hsd, hsd->a_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: SD_UpdateFilepaths: Accel. file (\"%s\") touch failed\r\n", HAL_GetTick(), hsd->a_file_path);
		return HAL_ERROR;
	}
	if (SD_TouchFile(hsd, hsd->p_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: SD_UpdateFilepaths: Pos. file (\"%s\") touch failed\r\n", HAL_GetTick(), hsd->p_file_path);
		return HAL_ERROR;
	}

	return HAL_OK;
}

// Increment page number, creating new data files
HAL_StatusTypeDef SD_NewPage(Vera_SD_t *hsd)
{
	hsd->page_num++;
	// Update file paths and create files
	if (SD_UpdateFilepaths(hsd) != HAL_OK)
	{
		return HAL_ERROR;
	}
	// Write file headers
	if (SD_WriteBuffer(hsd, hsd->a_file_path, (void*)&hsd->a_header, sizeof(a_data_header_t)) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (SD_WriteBuffer(hsd, hsd->p_file_path, (void*)&hsd->p_header, sizeof(p_data_header_t)) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef SD_Uninit(Vera_SD_t *hsd)
{
	f_close(hsd->fatfs_file);
	f_mount(NULL, hsd->fatfs_path, 0);

	return HAL_OK;
}

// Create file if it doesn't exist
HAL_StatusTypeDef SD_TouchFile(Vera_SD_t *hsd, TCHAR *path)
{
	if (f_open(hsd->fatfs_file, path, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	{
		return HAL_ERROR;
	}
	f_close(hsd->fatfs_file);

	return HAL_OK;
}

// Check if file exists
uint8_t SD_FileExists(Vera_SD_t *hsd, TCHAR *path)
{
	if (f_open(hsd->fatfs_file, path, FA_READ) != FR_OK)
	{
		return 0;
	}
	f_close(hsd->fatfs_file);

	return 1;
}

// Read file into buffer
HAL_StatusTypeDef SD_ReadBuffer(Vera_SD_t *hsd, TCHAR *path, void *data, UINT size, UINT *size_read)
{
	if (size == 0)
	{
		return HAL_OK;
	}

	// Open file with read access
	if (f_open(hsd->fatfs_file, path, FA_READ) != FR_OK)
	{
		printf("(%lu) ERROR: SD_ReadBuffer: SD File \"%s\": file open failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	// Read data
	FRESULT res = f_read(hsd->fatfs_file, data, size, size_read);
	if (res != FR_OK)
	{
		printf("(%lu) ERROR: SD_ReadBuffer: SD File \"%s\": file read failed\r\n", HAL_GetTick(), path);
		f_close(hsd->fatfs_file);
		return HAL_ERROR;
	}

	// Close file
	if (f_close(hsd->fatfs_file) != FR_OK)
	{
		printf("(%lu) ERROR: SD_ReadBuffer: SD File \"%s\": file close failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	return HAL_OK;
}

// Write (append) buffer into file
HAL_StatusTypeDef SD_WriteBuffer(Vera_SD_t *hsd, TCHAR *path, void *data, UINT size)
{
	if (size == 0)
	{
		return HAL_OK;
	}

	// Open file with append write access
	// Reference: http://elm-chan.org/fsw/ff/doc/open.html
	if (f_open(hsd->fatfs_file, path, FA_OPEN_APPEND | FA_WRITE | FA_READ) != FR_OK)
	{
		printf("(%lu) ERROR: SD_WriteBuffer: SD File \"%s\": file open failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	// Write data
	UINT bytes_written;
	FRESULT res = f_write(hsd->fatfs_file, data, size, &bytes_written);
	if (res != FR_OK || bytes_written != size)
	{
		printf("(%lu) ERROR: SD_WriteBuffer: SD File \"%s\": file write failed (%hu / %u bytes)\r\n", HAL_GetTick(), path, bytes_written, size);
		f_close(hsd->fatfs_file);
		return HAL_ERROR;
	}

	// Close file
	if (f_close(hsd->fatfs_file) != FR_OK)
	{
		printf("(%lu) ERROR: SD_WriteBuffer: SD File \"%s\": file close failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	return HAL_OK;
}
