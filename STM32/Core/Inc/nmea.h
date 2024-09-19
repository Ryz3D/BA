/*
 * nmea.h
 *
 * UART (RS-232) NMEA driver for u-blox 8 (Navilock 62528)
 *
 *  Created on: Aug 16, 2024
 *      Author: mirco
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "config.h"
#include "stm32f7xx_hal.h"

#define NMEA_TALKER_GPS 'P'
#define NMEA_TALKER_GLONASS 'L'
#define NMEA_TALKER_GALILEO 'A'
#define NMEA_TALKER_BEIDOU 'B'
#define NMEA_TALKER_ANY 'N'

#define NMEA_MODE_AUTONOMOUS 'A'
#define NMEA_MODE_DIFFERENTIAL 'D'
#define NMEA_MODE_ESTIMATED 'E'
#define NMEA_MODE_MANUAL 'M'
#define NMEA_MODE_NOT_VALID 'N'

#define NMEA_PORT_ID_DDC 0
#define NMEA_PORT_ID_UART1 1
#define NMEA_PORT_ID_UART2 2
#define NMEA_PORT_ID_USB 3
#define NMEA_PORT_ID_SPI 4

#define UBX_GNSS_ID_GPS 0
#define UBX_GNSS_ID_SBAS 1
#define UBX_GNSS_ID_GALILEO 2
#define UBX_GNSS_ID_BEIDOU 3
#define UBX_GNSS_ID_IMES 4
#define UBX_GNSS_ID_QZSS 5
#define UBX_GNSS_ID_GLONASS 6

#define UBX_RESET_BBR_HOT_START 0x0000
#define UBX_RESET_BBR_WARM_START 0x0001
#define UBX_RESET_BBR_COLD_START 0xFFFF

#define UBX_RESET_MODE_HW 0x00
#define UBX_RESET_MODE_SW 0x01
#define UBX_RESET_MODE_SW_GNSS 0x02
#define UBX_RESET_MODE_HW_SHUTDOWN 0x04
#define UBX_RESET_MODE_GNSS_STOP 0x08
#define UBX_RESET_MODE_GNSS_START 0x09

#define NMEA_RX_BUFFER_SIZE 128
#define NMEA_DMA_BUFFER_SIZE 1
#define NMEA_CIRCULAR_BUFFER_SIZE 32

typedef struct
{
	uint32_t timestamp;
	char talker; // See #define NMEA_TALKER_XXX
	// Position
	uint8_t position_valid;
	float lat, lon;
	// Speed
	uint8_t speed_valid;
	float speed_kmh;
	// Altitude
	uint8_t altitude_valid;
	float altitude;
	// Date
	uint8_t date_valid;
	uint8_t year;
	uint8_t month;
	uint8_t day;
	// Time
	uint8_t time_valid;
	uint8_t hour;
	uint8_t minute;
	float second;
} NMEA_Data_t;

typedef struct
{
	uint32_t timestamp;
	volatile char buffer[NMEA_RX_BUFFER_SIZE];
} NMEA_Line_t;

typedef struct
{
	UART_HandleTypeDef *huart;
	uint32_t tx_timeout;
	uint32_t rx_timeout;
	uint32_t baud;
	uint8_t sampling_rate;
	volatile char dma_buffer[NMEA_DMA_BUFFER_SIZE];
	volatile char rx_buffer[NMEA_RX_BUFFER_SIZE];
	volatile uint16_t rx_buffer_write_index;
	volatile uint8_t overflow_rx_buffer;
	volatile char line_buffer[NMEA_RX_BUFFER_SIZE];
	volatile uint32_t circular_read_index;
	volatile uint32_t circular_write_index;
	volatile NMEA_Line_t circular_buffer[NMEA_CIRCULAR_BUFFER_SIZE];
	volatile uint8_t overflow_circular_buffer;
	uint16_t last_ubx_header;
} NMEA_t;

extern const char nmea_pformat_rmc[];

typedef struct
{
	float time;
	char status;
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	float speed_kn;
	float track;
	int32_t date;
	float magvar;
	char magvar_dir;
	char mode; // See #define NMEA_MODE_XXX
} NMEA_Packet_RMC_t;

extern const char nmea_pformat_gga[];

typedef struct
{
	float time;
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	char quality;
	int32_t num_SV;
	float HDOP;
	float altitude;
	char M1;
	float sep;
	char M2;
	float diff_age;
	int32_t diff_station;
} NMEA_Packet_GGA_t;

extern const char nmea_pformat_gll[];

typedef struct
{
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	float time;
	char status;
	char mode; // See #define NMEA_MODE_XXX
} NMEA_Packet_GLL_t;

extern const char nmea_pformat_vtg[];

typedef struct
{
	float track;
	char T;
	float track_mag;
	char M;
	float speed_kn;
	char N;
	float speed_kmh;
	char K;
	char mode; // See #define NMEA_MODE_XXX
} NMEA_Packet_VTG_t;

// header = Class, ID, Length
#define NMEA_UBX_CFG_RATE_HEADER (0x06 | (0x08 << 8) | (6 << 16))
typedef struct
{
	uint16_t measRate;
	uint16_t navRate;
	uint16_t timeRef;
} NMEA_UBX_CFG_RATE_t;

#define NMEA_UBX_CFG_PRT_HEADER (0x06 | (0x00 << 8) | (20 << 16))
typedef struct
{
	uint8_t portID; // See #define NMEA_PORT_ID_XXX
	uint16_t txReady;
	uint32_t mode;
	uint32_t baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint16_t reserved;
} NMEA_UBX_CFG_PRT_t;

#define NMEA_UBX_CFG_GNSS_NUMCONFIGBLOCKS 2
#define NMEA_UBX_CFG_GNSS_HEADER (0x06 | (0x3E << 8) | ((4 + 8 * NMEA_UBX_CFG_GNSS_NUMCONFIGBLOCKS) << 16))

typedef struct
{
	uint8_t gnssId; // See #define UBX_GNSS_ID_XXX
	uint8_t resTrkCh;
	uint8_t maxTrkCh;
	uint32_t flags;
} NMEA_UBX_CFG_GNSS_CFGBLOCK_t;

typedef struct
{
	uint8_t msgVer;
	uint8_t numTrkChHw;
	uint8_t numTrkChUse;
	uint8_t numConfigBlocks;
	NMEA_UBX_CFG_GNSS_CFGBLOCK_t configBlocks[NMEA_UBX_CFG_GNSS_NUMCONFIGBLOCKS];
} NMEA_UBX_CFG_GNSS_t;

#define NMEA_UBX_CFG_CFG_HEADER (0x06 | (0x09 << 8) | (12 << 16))

typedef struct
{
	uint32_t clearMask;
	uint32_t saveMask;
	uint32_t loadMask;
} NMEA_UBX_CFG_CFG_t;

#define NMEA_UBX_CFG_RST_HEADER (0x06 | (0x04 << 8) | (4 << 16))

typedef struct
{
	uint16_t navBbrMask;
	uint8_t resetMode;
	uint8_t reserved;
} NMEA_UBX_CFG_RST_t;

HAL_StatusTypeDef NMEA_TxPUBX(NMEA_t *hnmea, char *msg_buffer);
HAL_StatusTypeDef NMEA_TxUBX(NMEA_t *hnmea, uint32_t header, void *packet, size_t packet_size);
HAL_StatusTypeDef NMEA_RxAckUBX(NMEA_t *hnmea);
HAL_StatusTypeDef NMEA_Init(NMEA_t *hnmea);
NMEA_Data_t NMEA_GetDate(NMEA_t *hnmea);
HAL_StatusTypeDef NMEA_ProcessDMABuffer(NMEA_t *hnmea);
uint8_t NMEA_ProcessLine(NMEA_t *hnmea, NMEA_Data_t *data);

#endif /* INC_NMEA_H_ */
