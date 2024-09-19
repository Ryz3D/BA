/*
 * adxl.h
 *
 * SPI driver for MEMS sensor ADXL357 at 4 kSa/s
 *
 *  Created on: Jul 30, 2024
 *      Author: mirco
 */

#pragma once

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "stm32f7xx_hal.h"

#define ADXL_REG_DEVID_AD 0x00
#define ADXL_REG_DEVID_MST 0x01
#define ADXL_REG_PARTID 0x02
#define ADXL_REG_REVID 0x03
#define ADXL_REG_STATUS 0x04
#define ADXL_REG_FIFO_ENTRIES 0x05
#define ADXL_REG_TEMP2 0x06
#define ADXL_REG_TEMP1 0x07
#define ADXL_REG_XDATA3 0x08
#define ADXL_REG_XDATA2 0x09
#define ADXL_REG_XDATA1 0x0A
#define ADXL_REG_YDATA3 0x0B
#define ADXL_REG_YDATA2 0x0C
#define ADXL_REG_YDATA1 0x0D
#define ADXL_REG_ZDATA3 0x0E
#define ADXL_REG_ZDATA2 0x0F
#define ADXL_REG_ZDATA1 0x10
#define ADXL_REG_FIFO_DATA 0x11
#define ADXL_REG_OFFSET_X_H 0x1E
#define ADXL_REG_OFFSET_X_L 0x1F
#define ADXL_REG_OFFSET_Y_H 0x20
#define ADXL_REG_OFFSET_Y_L 0x21
#define ADXL_REG_OFFSET_Z_H 0x22
#define ADXL_REG_OFFSET_Z_L 0x23
#define ADXL_REG_ACT_EN 0x24
#define ADXL_REG_ACT_THRESH_H 0x25
#define ADXL_REG_ACT_THRESH_L 0x26
#define ADXL_REG_ACT_COUNT 0x27
#define ADXL_REG_FILTER 0x28
#define ADXL_REG_FIFO_SAMPLES 0x29
#define ADXL_REG_INT_MAP 0x2A
#define ADXL_REG_SYNC 0x2B
#define ADXL_REG_RANGE 0x2C
#define ADXL_REG_POWER_CTL 0x2D
#define ADXL_REG_SELF_TEST 0x2E
#define ADXL_REG_RESET 0x2F

#define ADXL_CHIP_SELECT(hadxl) HAL_GPIO_WritePin(hadxl->CS_GPIO_Port, hadxl->CS_Pin, GPIO_PIN_RESET);
#define ADXL_CHIP_DESELECT(hadxl) HAL_GPIO_WritePin(hadxl->CS_GPIO_Port, hadxl->CS_Pin, GPIO_PIN_SET);

typedef enum
{
	ADXL_STATE_READY,
	ADXL_STATE_BUSY_TX,
	ADXL_STATE_BUSY_RX,
	ADXL_STATE_ERROR
} ADXL_State_t;

typedef struct
{
	ADXL_State_t state;
	SPI_HandleTypeDef *hspi;
	uint32_t timeout;
	uint16_t sampling_rate; // IMPORTANT: Provide sampling rate as truncated (rounded-down) given in datasheet S. 48
	uint16_t acceleration_range;
	GPIO_TypeDef *CS_GPIO_Port;
	uint16_t CS_Pin;
	volatile uint8_t identification[3];
	uint8_t request_buffer[12];
	volatile uint8_t data_buffer[12];
} ADXL_t;

typedef struct
{
	uint8_t data_valid;
	uint16_t temp;
	int32_t x, y, z;
} ADXL_Data_t;

HAL_StatusTypeDef ADXL_ReadRegisters(ADXL_t *hadxl, uint8_t count, uint8_t register_addr, uint8_t *buffer);
HAL_StatusTypeDef ADXL_WriteRegisters(ADXL_t *hadxl, uint8_t count, uint8_t register_addr, uint8_t *buffer);
HAL_StatusTypeDef ADXL_ReadRegisterSingle(ADXL_t *hadxl, uint8_t register_addr, uint8_t *buffer);
HAL_StatusTypeDef ADXL_WriteRegisterSingle(ADXL_t *hadxl, uint8_t register_addr, uint8_t data);
HAL_StatusTypeDef ADXL_Init(ADXL_t *hadxl);
HAL_StatusTypeDef ADXL_RequestData(ADXL_t *hadxl);
ADXL_Data_t ADXL_RxCallback(ADXL_t *hadxl);
