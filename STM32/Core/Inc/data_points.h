/*
 * data_points.h
 *
 *  Created on: Jul 24, 2024
 *      Author: mirco
 */

#ifndef INC_DATA_POINTS_H_
#define INC_DATA_POINTS_H_

#include "config.h"

#define A_COMPLETE_TIMESTAMP 0
#define A_COMPLETE_MEMS 1
#define A_COMPLETE_PZ 2

#define P_COMPLETE_TIMESTAMP 0
#define P_COMPLETE_GNSS_TIME 1
#define P_COMPLETE_POSITION 2
#define P_COMPLETE_SPEED 3
#define P_COMPLETE_ALTITUDE 4

typedef struct
{
	uint8_t version;
	uint32_t boot_duration;
	uint32_t a_buffer_len;
	uint32_t a_sampling_rate;
	uint8_t piezo_count_max;
	uint8_t piezo_count;
	uint8_t oversampling_ratio;
	uint32_t fir_taps_len;
} a_data_header_t;

typedef struct
{
	uint8_t complete;
	uint32_t timestamp;
	uint16_t temp_mems1;
	int32_t xyz_mems1[3];
	int16_t a_piezo[PIEZO_COUNT_MAX];
} a_data_point_t;

typedef struct
{
	uint8_t version;
	uint32_t boot_duration;
	uint32_t p_buffer_len;
	uint32_t p_sampling_rate;
	uint16_t year;
	uint8_t month;
	uint8_t day;
} p_data_header_t;

typedef struct
{
	uint8_t complete;
	uint32_t timestamp;
	uint8_t gnss_hour;
	uint8_t gnss_minute;
	float gnss_second;
	float lat;
	float lon;
	float speed;
	float altitude;
} p_data_point_t;

#endif /* INC_DATA_POINTS_H_ */
