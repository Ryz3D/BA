/*
 * adxl.c
 *
 * SPI driver for MEMS sensor ADXL357 at 4 kSa/s
 *
 *  Created on: Aug 02, 2024
 *      Author: mirco
 */

#include "adxl.h"

HAL_StatusTypeDef ADXL_ReadRegisters(ADXL_t *hadxl, uint8_t count, uint8_t register_addr, uint8_t *buffer)
{
	ADXL_CHIP_SELECT(hadxl);

	hadxl->state = ADXL_STATE_BUSY_TX;
	uint8_t addr_tx = (register_addr << 1) | 1;
	if (HAL_SPI_Transmit(hadxl->hspi, &addr_tx, sizeof(uint8_t), hadxl->timeout) != HAL_OK)
	{
		hadxl->state = ADXL_STATE_ERROR;
		return HAL_ERROR;
	}

	hadxl->state = ADXL_STATE_BUSY_RX;
	if (HAL_SPI_Receive(hadxl->hspi, buffer, count * sizeof(uint8_t), hadxl->timeout) != HAL_OK)
	{
		hadxl->state = ADXL_STATE_ERROR;
		return HAL_ERROR;
	}

	hadxl->state = ADXL_STATE_READY;
	ADXL_CHIP_DESELECT(hadxl);

	return HAL_OK;
}

HAL_StatusTypeDef ADXL_WriteRegisters(ADXL_t *hadxl, uint8_t count, uint8_t register_addr, uint8_t *buffer)
{
	ADXL_CHIP_SELECT(hadxl);

	hadxl->state = ADXL_STATE_BUSY_TX;
	uint8_t addr_tx = (register_addr << 1) | 0;
	if (HAL_SPI_Transmit(hadxl->hspi, &addr_tx, sizeof(uint8_t), hadxl->timeout) != HAL_OK)
	{
		hadxl->state = ADXL_STATE_ERROR;
		return HAL_ERROR;
	}

	hadxl->state = ADXL_STATE_BUSY_RX;
	if (HAL_SPI_Transmit(hadxl->hspi, buffer, count, hadxl->timeout) != HAL_OK)
	{
		hadxl->state = ADXL_STATE_ERROR;
		return HAL_ERROR;
	}

	hadxl->state = ADXL_STATE_READY;
	ADXL_CHIP_DESELECT(hadxl);

	return HAL_OK;
}

HAL_StatusTypeDef ADXL_ReadRegisterSingle(ADXL_t *hadxl, uint8_t register_addr, uint8_t *buffer)
{
	return ADXL_ReadRegisters(hadxl, 1, register_addr, buffer);
}

HAL_StatusTypeDef ADXL_WriteRegisterSingle(ADXL_t *hadxl, uint8_t register_addr, uint8_t buffer)
{
	return ADXL_WriteRegisters(hadxl, 1, register_addr, &buffer);
}

HAL_StatusTypeDef ADXL_Init(ADXL_t *hadxl)
{
	// Provide default values
	if (hadxl->timeout == 0)
	{
		hadxl->timeout = 100;
	}

	// Flush SPI with SCK pulses
	uint8_t ignore;
	for (uint8_t i = 0; i < 10; i++)
	{
		ADXL_ReadRegisterSingle(hadxl, 0x00, &ignore);
	}

	// Read identification registers
	if (ADXL_ReadRegisters(hadxl, 3, ADXL_REG_DEVID_AD, (uint8_t*)hadxl->identification) != HAL_OK)
	{
		printf("(%lu) ERROR: ADXL_Init: Identification register read failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	if (hadxl->identification[0] == 0xFF && hadxl->identification[1] == 0xFF && hadxl->identification[2] == 0xFF)
	{
		printf("(%lu) WARNING: ADXL_Init: ADXL357 not connected\r\n", HAL_GetTick());
		return HAL_TIMEOUT;
	}

	if (hadxl->identification[0] != 0xAD)
	{
		printf("(%lu) WARNING: ADXL_Init: Incorrect DEVID_AD (Expected: 0xAD, Received: 0x%02X)\r\n", HAL_GetTick(), hadxl->identification[0]);
	}
	if (hadxl->identification[1] != 0x1D)
	{
		printf("(%lu) WARNING: ADXL_Init: Incorrect DEVID_MST (Expected: 0x1D, Received: 0x%02X)\r\n", HAL_GetTick(), hadxl->identification[1]);
	}
	if (hadxl->identification[2] != 0xED)
	{
		printf("(%lu) WARNING: ADXL_Init: Incorrect PARTID (Expected: 0xED, Received: 0x%02X)\r\n", HAL_GetTick(), hadxl->identification[2]);
	}

	// Filter
	uint8_t hpf = 0b000; // HPF off
	// Get ratio of set sampling rate to max sampling rate
	uint16_t sampling_rate_ratio = 4000 / hadxl->sampling_rate;
	// log2 to get register setting
	uint8_t lpf = 0;
	while (sampling_rate_ratio >>= 1)
	{
		lpf++;
	}
	if (lpf > 10)
	{
		lpf = 0;
		printf("(%lu) WARNING: ADXL_Init: Invalid sampling_rate %hu, using 4kHz\r\n", HAL_GetTick(), hadxl->sampling_rate);
	}
	if (ADXL_WriteRegisterSingle(hadxl, ADXL_REG_FILTER, (hpf << 4) | lpf) != HAL_OK)
	{
		printf("(%lu) ERROR: ADXL_Init: Register write failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	// Acceleration range
	uint8_t i2c_hs = 1; // I2C high speed
	uint8_t int_pol = 0; // INT1, INT2 active low
	// Get ratio of set range to min range
	uint16_t range_ratio = hadxl->acceleration_range / 10;
	// log2 to get register setting
	uint8_t range = 1;
	while (range_ratio >>= 1)
	{
		range++;
	}
	if (range == 0 || range > 3)
	{
		range = 0;
		printf("(%lu) WARNING: ADXL_Init: Invalid range %hu, using +-40g\r\n", HAL_GetTick(), hadxl->acceleration_range);
	}
	if (ADXL_WriteRegisterSingle(hadxl, ADXL_REG_RANGE, (i2c_hs << 7) | (int_pol << 6) | range) != HAL_OK)
	{
		printf("(%lu) ERROR: ADXL_Init: Register write failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	// POWER_CTL
	// Measurement mode
	if (ADXL_WriteRegisterSingle(hadxl, ADXL_REG_POWER_CTL, 0b00000000) != HAL_OK)
	{
		printf("(%lu) ERROR: ADXL_Init: Register write failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	// Prepare buffer for data request
	for (uint8_t i = 0; i < sizeof(hadxl->request_buffer); i++)
	{
		hadxl->request_buffer[i] = 0;
	}
	hadxl->request_buffer[0] = (ADXL_REG_TEMP2 << 1) | 1;

	printf("(%lu) ADXL357 initialized\r\n", HAL_GetTick());
	return HAL_OK;
}

HAL_StatusTypeDef ADXL_RequestData(ADXL_t *hadxl)
{
	ADXL_CHIP_SELECT(hadxl);

	// Send request_buffer
	hadxl->state = ADXL_STATE_BUSY_RX;
	if (HAL_SPI_TransmitReceive_DMA(hadxl->hspi, hadxl->request_buffer, (uint8_t*)hadxl->data_buffer, sizeof(hadxl->request_buffer)) == HAL_ERROR)
	{
		hadxl->state = ADXL_STATE_ERROR;
		return HAL_ERROR;
	}

	return HAL_OK;
}

ADXL_Data_t ADXL_RxCallback(ADXL_t *hadxl)
{
	hadxl->state = ADXL_STATE_READY;
	ADXL_CHIP_DESELECT(hadxl);

	ADXL_Data_t data;
	data.temp = ((uint16_t)(hadxl->data_buffer[1] & 0b00001111) << 8) | (hadxl->data_buffer[2]);
	data.x = ((uint32_t)hadxl->data_buffer[3] << 12) | ((uint32_t)hadxl->data_buffer[4] << 4) | (hadxl->data_buffer[5] >> 4);
	data.y = ((uint32_t)hadxl->data_buffer[6] << 12) | ((uint32_t)hadxl->data_buffer[7] << 4) | (hadxl->data_buffer[8] >> 4);
	data.z = ((uint32_t)hadxl->data_buffer[9] << 12) | ((uint32_t)hadxl->data_buffer[10] << 4) | (hadxl->data_buffer[11] >> 4);

	// 20-bit sign -> 32-bit sign
	if (data.x & 0x80000)
	{
		data.x -= 0x100000;
	}
	if (data.y & 0x80000)
	{
		data.y -= 0x100000;
	}
	if (data.z & 0x80000)
	{
		data.z -= 0x100000;
	}

	if (data.x == 0xFFFFFFFF && data.y == 0xFFFFFFFF && data.z == 0xFFFFFFFF)
	{
		data.data_valid = 0;
	}
	else
	{
		data.data_valid = 1;
	}

	return data;
}
