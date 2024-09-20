/*
 * Copyright (c) 2024 Mirco Heitmann
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * nmea.c
 *
 * UART (RS-232) NMEA driver for u-blox 8 (Navilock 62528)
 */

#include "nmea.h"

const char nmea_pformat_rmc[] = "fcfcfcffifcc";
const char nmea_pformat_gga[] = "ffcfcciffcfcfi";
const char nmea_pformat_gll[] = "fcfcfcc";
const char nmea_pformat_vtg[] = "fcfcfcfcc";

void NMEA_ParsePacket(NMEA_t *hnmea, void *packet_buffer, const char format[]);
uint8_t NMEA_ChecksumValid(NMEA_t *hnmea);
uint8_t NMEA_Hex2Dec(char c);
void NMEA_Dec2Hex(uint8_t dec, char *c1, char *c2);
void NMEA_ConvertTime(NMEA_Data_t *data, float time);
void NMEA_ConvertDate(NMEA_Data_t *data, int32_t date);
void NMEA_ConvertLatLon(NMEA_Data_t *data, float lat, char lat_dir, float lon, char lon_dir);

// Transmit PUBX protocol
HAL_StatusTypeDef NMEA_TxPUBX(NMEA_t *hnmea, char *msg_buffer)
{
	char tx_buffer[100];
	// Put message into packet format
	sprintf(tx_buffer, "$PUBX,%s*00\r\n", msg_buffer);
	// Calculate checksum
	uint32_t tx_len = strlen(tx_buffer);
	uint8_t checksum = 0;
	for (uint32_t i = 1; i < tx_len - 5; i++)
	{
		checksum ^= tx_buffer[i];
	}
	// Write checksum as hexadecimal into packet
	NMEA_Dec2Hex(checksum, &tx_buffer[tx_len - 4], &tx_buffer[tx_len - 3]);
	// Transmit packet
	if (HAL_UART_Transmit(hnmea->huart, (uint8_t*)tx_buffer, tx_len, hnmea->tx_timeout) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

// Transmit UBX protocol
HAL_StatusTypeDef NMEA_TxUBX(NMEA_t *hnmea, uint32_t header, void *packet, size_t packet_size)
{
	// Protocol header
	uint8_t tx_buffer[100] = { 0xB5, 0x62 };
	uint32_t tx_len = packet_size + 8;
	// Packet header
	*(uint32_t*)(tx_buffer + 2) = header;
	// Packet content
	memcpy(tx_buffer + 6, packet, packet_size);
	// Packet checksum
	tx_buffer[tx_len - 2] = 0;
	tx_buffer[tx_len - 1] = 0;
	for (uint8_t i = 2; i < tx_len - 2; i++)
	{
		tx_buffer[tx_len - 2] += tx_buffer[i];
		tx_buffer[tx_len - 1] += tx_buffer[tx_len - 2];
	}
	// Save transmitted header for received checksum
	hnmea->last_ubx_header = header & 0xFFFF;
	if (HAL_UART_Transmit(hnmea->huart, tx_buffer, tx_len, hnmea->tx_timeout) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

// Receive ACK in UBX protocol
HAL_StatusTypeDef NMEA_RxAckUBX(NMEA_t *hnmea)
{
	// Wait for protocol header
	uint8_t header = 0;
	for (uint32_t i = 0; i < 100; i++)
	{
		HAL_UART_Receive(hnmea->huart, &header, sizeof(uint8_t), 10);
		if (header == 0xB5)
		{
			HAL_UART_Receive(hnmea->huart, &header, sizeof(uint8_t), 10);
			if (header == 0x62)
			{
				break;
			}
		}
		header = 0;
	}
	if (header == 0)
	{
		return HAL_ERROR;
	}
	// Receive 8 bytes as 4 * uint16
	uint16_t rx_buffer[8 / 2];
	if (HAL_UART_Receive(hnmea->huart, (uint8_t*)rx_buffer, sizeof(rx_buffer), hnmea->rx_timeout) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	// Check if ACK/NAK
	if (rx_buffer[0] != 0x0105)
	{
		// NAK: rx_buffer[0] == 0x0005
		return HAL_ERROR;
	}
	// Check length
	if (rx_buffer[1] != 2)
	{
		return HAL_ERROR;
	}
	// Check if corresponding ACK for transmitted message
	if (rx_buffer[2] != hnmea->last_ubx_header)
	{
		return HAL_ERROR;
	}
	// Check checksum
	uint8_t ck_a = 0, ck_b = 0;
	for (uint8_t i = 0; i < sizeof(rx_buffer) - 2; i++)
	{
		ck_a += ((uint8_t*)rx_buffer)[i];
		ck_b += ck_a;
	}
	if ((rx_buffer[3] & 0xFF) != ck_a)
	{
		return HAL_ERROR;
	}
	if ((rx_buffer[3] >> 8) != ck_b)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef NMEA_Init(NMEA_t *hnmea)
{
	// Provide default values
	if (hnmea->tx_timeout == 0)
	{
		hnmea->tx_timeout = 1000;
	}
	if (hnmea->rx_timeout == 0)
	{
		hnmea->rx_timeout = 1000;
	}
	if (hnmea->baud == 0)
	{
		hnmea->baud = 9600;
	}
	if (hnmea->sampling_rate == 0)
	{
		hnmea->sampling_rate = 10;
	}

	// Init struct
	hnmea->rx_buffer_write_index = 0;
	hnmea->circular_read_index = 0;
	hnmea->circular_write_index = 0;
	hnmea->last_ubx_header = 0;

	// Delay to let GNSS-module boot
	HAL_Delay(1000);

	// Set baud rate
	char pubx_buffer[100];
	uint16_t inProto = 0b000011; // Module should accept NMEA and UBX via UART
	uint16_t outProto = 0b000010; // Module should transmit NMEA via UART
	sprintf(pubx_buffer, "41,1,%04hX,%04hX,%lu,0", inProto, outProto, hnmea->baud);
	if (NMEA_TxPUBX(hnmea, pubx_buffer) == HAL_ERROR)
	{
		printf("(%lu) ERROR: NMEA_Init: NMEA_SendPUBX failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	hnmea->huart->Init.BaudRate = hnmea->baud;
	if (HAL_UART_Init(hnmea->huart) != HAL_OK)
	{
		printf("(%lu) ERROR: NMEA_Init: HAL_UART_Init failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	HAL_Delay(250);

	// Set output data rate
	NMEA_UBX_CFG_RATE_t ubx_rate = {
		.measRate = 1000 / hnmea->sampling_rate,
		.navRate = 1,
		.timeRef = 0,
	};
	NMEA_TxUBX(hnmea, NMEA_UBX_CFG_RATE_HEADER, &ubx_rate, sizeof(ubx_rate));
	if (NMEA_RxAckUBX(hnmea) != HAL_OK)
	{
		printf("(%lu) WARNING: UBX-CFG-RATE not acknowledged\r\n", HAL_GetTick());
	}

	// Start receiving data
	if (HAL_UART_Receive_DMA(hnmea->huart, (uint8_t*)&hnmea->dma_buffer, NMEA_DMA_BUFFER_SIZE) == HAL_ERROR)
	{
		printf("(%lu) ERROR: NMEA_Init: HAL_UART_Receive_DMA failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	return HAL_OK;
}

// Wait for GNSS date, return packet with valid date if received before timeout
NMEA_Data_t NMEA_GetDate(NMEA_t *hnmea)
{
	printf("(%lu) Waiting for GNSS date (timeout after %i s)...\r\n", HAL_GetTick(), NMEA_DATE_WAIT_DURATION / 1000);

	NMEA_Data_t data = {
		.position_valid = 0,
		.speed_valid = 0,
		.altitude_valid = 0,
		.date_valid = 0,
		.time_valid = 0,
	};
	uint32_t time_end = HAL_GetTick() + NMEA_DATE_WAIT_DURATION;
	while (HAL_GetTick() <= time_end)
	{
		if (hnmea->circular_write_index != hnmea->circular_read_index)
		{
			if (NMEA_ProcessLine(hnmea, &data))
			{
				if (data.date_valid)
				{
					return data;
				}
			}
		}
	}

	return data;
}

// Handle full DMA buffer
HAL_StatusTypeDef NMEA_ProcessDMABuffer(NMEA_t *hnmea)
{
	for (uint32_t i = 0; i < NMEA_DMA_BUFFER_SIZE; i++)
	{
		// If end of line
		if (hnmea->dma_buffer[i] == '\n')
		{
			// If current line has content
			if (hnmea->rx_buffer_write_index > 1)
			{
				// Copy finished current line to circular buffer and remove trailing \r
				memcpy((void*)hnmea->circular_buffer[hnmea->circular_write_index].buffer, (void*)hnmea->rx_buffer, hnmea->rx_buffer_write_index - 1);
				// Add string termination
				hnmea->circular_buffer[hnmea->circular_write_index].buffer[hnmea->rx_buffer_write_index - 1] = '\0';
				// Increment circular buffer index
				hnmea->circular_write_index = (hnmea->circular_write_index + 1) % NMEA_CIRCULAR_BUFFER_SIZE;
				// If incremented up to read index, there was a circular buffer overflow
				if (hnmea->circular_write_index == hnmea->circular_read_index)
				{
					hnmea->overflow_circular_buffer = 1;
				}
			}
			// Reset to start of line buffer
			hnmea->rx_buffer_write_index = 0;
		}
		else
		{
			// If start of line
			if (hnmea->dma_buffer[i] == '$')
			{
				// Reset to start of line buffer
				hnmea->rx_buffer_write_index = 0;
				// Save timestamp of start of transmission in circular buffer
				hnmea->circular_buffer[hnmea->circular_write_index].timestamp = HAL_GetTick();
			}
			// Write character to line buffer
			hnmea->rx_buffer[hnmea->rx_buffer_write_index++] = hnmea->dma_buffer[i];
			// If line buffer at end, there was a line buffer overflow
			if (hnmea->rx_buffer_write_index >= NMEA_RX_BUFFER_SIZE)
			{
				// Reset to start of line buffer
				hnmea->rx_buffer_write_index = 0;
				hnmea->overflow_rx_buffer = 1;
			}
		}
	}

	// Keep receiving data
	if (HAL_UART_Receive_DMA(hnmea->huart, (uint8_t*)&hnmea->dma_buffer, NMEA_DMA_BUFFER_SIZE) != HAL_OK)
	{
		printf("(%lu) ERROR: NMEA_ProcessChar: HAL_UART_Receive_DMA failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	return HAL_OK;
}

// Process line from circular buffer, returns 1 if valid data was written
uint8_t NMEA_ProcessLine(NMEA_t *hnmea, NMEA_Data_t *data)
{
	if (hnmea->circular_read_index == hnmea->circular_write_index)
	{
		return 0;
	}

	uint8_t any_valid = 0;

	data->timestamp = hnmea->circular_buffer[hnmea->circular_read_index].timestamp;
	data->position_valid = 0;
	data->speed_valid = 0;
	data->altitude_valid = 0;
	data->date_valid = 0;
	data->time_valid = 0;

	volatile char *line_buffer = hnmea->circular_buffer[hnmea->circular_read_index].buffer;
	// If packet starting with GNSS talker ID
	if (strncmp("$G", (char*)line_buffer, 2) == 0)
	{
		// Get GNSS identifier from talker ID
		data->talker = line_buffer[2];

		// If RMC packet
		if (strncmp("RMC", (char*)line_buffer + 3, 3) == 0)
		{
			if (!NMEA_ChecksumValid(hnmea))
			{
				return 0;
			}

			NMEA_Packet_RMC_t packet;
			NMEA_ParsePacket(hnmea, &packet, nmea_pformat_rmc);

			// Check if date valid
			if (packet.date > 0)
			{
				any_valid = 1;
				data->date_valid = 1;
				NMEA_ConvertDate(data, packet.date);
			}
			// Check if time valid
			if (packet.time > 0)
			{
				any_valid = 1;
				data->time_valid = 1;
				NMEA_ConvertTime(data, packet.time);
			}
			// Check if position and speed valid
			if (packet.mode != NMEA_MODE_NOT_VALID)
			{
				any_valid = 1;
				data->position_valid = 1;
				NMEA_ConvertLatLon(data, packet.lat, packet.lat_dir, packet.lon, packet.lon_dir);

				data->speed_valid = 1;
				data->speed_kmh = packet.speed_kn * 1.852f;
			}
		}
		// If GGA packet
		else if (strncmp("GGA", (char*)line_buffer + 3, 3) == 0)
		{
			if (!NMEA_ChecksumValid(hnmea))
			{
				return 0;
			}

			NMEA_Packet_GGA_t packet;
			NMEA_ParsePacket(hnmea, &packet, nmea_pformat_gga);

			// If altitude valid
			if (packet.altitude > 0)
			{
				any_valid = 1;
				data->altitude_valid = 1;
				data->altitude = packet.altitude;
			}
		}
	}

	// Increment circular buffer read index
	hnmea->circular_read_index = (hnmea->circular_read_index + 1) % NMEA_CIRCULAR_BUFFER_SIZE;

	return any_valid;
}

// Check checksum
uint8_t NMEA_ChecksumValid(NMEA_t *hnmea)
{
	volatile char *line_buffer = hnmea->circular_buffer[hnmea->circular_read_index].buffer;
	uint8_t checksum_calc = 0;
	for (uint16_t i = 1; line_buffer[i] != '\0'; i++)
	{
		// If end of data
		if (line_buffer[i] == '*')
		{
			// Read and check checksum
			uint8_t checksum = NMEA_Hex2Dec(line_buffer[i + 1]) << 4 | NMEA_Hex2Dec(line_buffer[i + 2]);
			if (checksum == checksum_calc)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		// Add to checksum calculation
		checksum_calc ^= line_buffer[i];
	}
	return 0;
}

// Hexadecimal char to numeric value
uint8_t NMEA_Hex2Dec(char c)
{
	if (c >= '0' && c <= '9')
	{
		return c - '0';
	}
	if (c >= 'A' && c <= 'F')
	{
		return c + 10 - 'A';
	}
	if (c >= 'a' && c <= 'f')
	{
		return c + 10 - 'a';
	}
	return 0;
}

// Decimal value to hexadecimal chars
void NMEA_Dec2Hex(uint8_t dec, char *c1, char *c2)
{
	*c1 = '0' + dec / 16;
	if (*c1 > '9')
	{
		*c1 += 'A' - '0' - 10;
	}
	*c2 = '0' + dec % 16;
	if (*c2 > '9')
	{
		*c2 += 'A' - '0' - 10;
	}
}

// Parse packet data with given format string from circular buffer into struct
void NMEA_ParsePacket(NMEA_t *hnmea, void *packet_buffer, const char format[])
{
	// Read from circular buffer
	volatile char *line_buffer = hnmea->circular_buffer[hnmea->circular_read_index].buffer;
	volatile char *line_pointer = line_buffer;
	// Skip message header
	while (*line_pointer != ',' && *line_pointer != '\0')
	{
		line_pointer++;
	}
	line_pointer++;

	// Find end of string to compare against
	volatile char *line_end_pointer = line_buffer + strlen((char*)line_buffer);
	// Pointer to currently processed character
	void *buffer_pointer = packet_buffer;
	for (uint8_t i_format = 0; i_format < strlen(format) && line_pointer < line_end_pointer; i_format++)
	{
		// Pointer to end of strtol/strtof conversion
		char *end_pointer;
		switch (format[i_format])
		{
		case 'i':
			// If pointer is unaligned for data type
			if ((uintptr_t)buffer_pointer % sizeof(int32_t) != 0)
			{
				// Align pointer (to match address of member in struct)
				buffer_pointer += sizeof(int32_t) - (uintptr_t)buffer_pointer % sizeof(int32_t);
			}
			// Check if value is present (comma skips current value, moves on to next value immediately)
			if (*line_pointer != ',')
			{
				*(int32_t*)buffer_pointer = strtol((char*)line_pointer, &end_pointer, 10);
				line_pointer = end_pointer;
			}
			else
			{
				*(int32_t*)buffer_pointer = 0;
			}
			line_pointer++;
			buffer_pointer += sizeof(int32_t);
			break;
		case 'x':
			if ((uintptr_t)buffer_pointer % sizeof(int32_t) != 0)
			{
				buffer_pointer += sizeof(int32_t) - (uintptr_t)buffer_pointer % sizeof(int32_t);
			}
			if (*line_pointer != ',')
			{
				*(int32_t*)buffer_pointer = strtol((char*)line_pointer, &end_pointer, 16);
				line_pointer = end_pointer;
			}
			else
			{
				*(int32_t*)buffer_pointer = 0;
			}
			line_pointer++;
			buffer_pointer += sizeof(int32_t);
			break;
		case 'f':
			if ((uintptr_t)buffer_pointer % sizeof(float) != 0)
			{
				buffer_pointer += sizeof(float) - (uintptr_t)buffer_pointer % sizeof(float);
			}
			if (*line_pointer != ',')
			{
				*(float*)buffer_pointer = strtof((char*)line_pointer, &end_pointer);
				line_pointer = end_pointer;
			}
			else
			{
				*(float*)buffer_pointer = 0;
			}
			line_pointer++;
			buffer_pointer += sizeof(float);
			break;
		case 'c':
			if (*line_pointer != ',')
			{
				// Get single char from packet
				*(char*)buffer_pointer = *line_pointer;
				line_pointer++;
			}
			else
			{
				*(char*)buffer_pointer = '\0';
			}
			line_pointer++;
			buffer_pointer += sizeof(char);
			break;
		default:
			printf("(%lu) WARNING: NMEA_ParsePacket: Unknown format %c\r\n", HAL_GetTick(), format[i_format]);
			// Ignore all values until next value (',') or end of buffer ('\0')
			while (*line_pointer != ',' && *line_pointer != '\0')
			{
				line_pointer++;
			}
			line_pointer++;
			buffer_pointer++;
			break;
		}
	}
}

// Convert time from NMEA float format to NMEA_Data_t hour/minute/second
void NMEA_ConvertTime(NMEA_Data_t *data, float time)
{
	data->hour = (uint32_t)time / 10000;
	data->minute = ((uint32_t)time / 100) % 100;
	data->second = fmod(time, 100);
}

// Convert date from NMEA integer format to NMEA_Data_t day/month/year
void NMEA_ConvertDate(NMEA_Data_t *data, int32_t date)
{
	data->day = date / 10000;
	data->month = (date / 100) % 100;
	data->year = date % 100;
}

// Convert coordinates from NMEA float/char format to NMEA_Data_t signed true degrees
void NMEA_ConvertLatLon(NMEA_Data_t *data, float lat, char lat_dir, float lon, char lon_dir)
{
	lat /= 100.0f;
	lon /= 100.0f;

	int lat_deg = (int)lat;
	int lon_deg = (int)lon;
	float lat_minutes = lat - lat_deg;
	float lon_minutes = lon - lon_deg;
	// Convert degrees and minutes to true degrees
	data->lat = lat_deg + lat_minutes / 0.6f;
	data->lon = lon_deg + lon_minutes / 0.6f;

	// Convert direction to sign
	if (lat_dir == 'S')
	{
		data->lat *= -1;
	}
	if (lon_dir == 'W')
	{
		data->lon *= -1;
	}
}
