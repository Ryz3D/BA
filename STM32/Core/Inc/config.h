/*
 * config.h
 *
 *  Created on: Jul 26, 2024
 *      Author: mirco
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"

#define VERSION 1
// Load config from SD
// default: 1
#define LOAD_CONFIG 1

#define OVERSAMPLING_RATIO_MAX 20
#define PIEZO_COUNT_MAX 5
#define A_BUFFER_LEN_MAX 4096
#define P_BUFFER_LEN_MAX 128
#define NMEA_DATE_WAIT_DURATION 180000
#define NMEA_PACKET_MERGE_DURATION 25
#define NMEA_NO_PACKET_DURATION 5000

#define C_F_BOOT_WITHOUT_DATE "boot_without_date=%hhu"
#define C_F_PRINT_ACCELERATION_DATA "print_acceleration_data=%hhu"
#define C_F_PRINT_POSITION_DATA "print_position_data=%hhu"
#define C_F_PIEZO_COUNT "piezo_count=%hhu"
#define C_F_FIR_TYPE "fir_type=%hhu"
#define C_F_ADXL_RANGE "adxl_range=%hu"
#define C_F_PAGE_DURATION_MS "page_duration_ms=%lu"
#define C_F_A_SAMPLING_RATE "a_sampling_rate=%lu"
#define C_F_P_SAMPLING_RATE "p_sampling_rate=%lu"
#define C_F_OVERSAMPLING_RATIO "oversampling_ratio=%hhu"
#define C_F_A_BUFFER_LEN "a_buffer_len=%lu"
#define C_F_P_BUFFER_LEN "p_buffer_len=%lu"

typedef struct
{
	// Skip waiting for date from GNSS module, start capture immediately (in directory "0000-00-00")
	uint8_t boot_without_date;
	// Print live acceleration as amplitude and offset
	uint8_t print_acceleration_data;
	// Print live position, speed, altitude, GNSS time
	uint8_t print_position_data;
	// Number of ADC channels
	uint8_t piezo_count;
	// Select FIR taps (0: disable)
	uint8_t fir_type;
	// ADXL357 measurement range
	uint16_t adxl_range;
	// Duration before switching to next page (file) in milliseconds
	uint32_t page_duration_ms;
	// Rate of saved acceleration samples
	uint32_t a_sampling_rate;
	// Rate of saved position samples
	uint32_t p_sampling_rate;
	// ADC sampling rate = a_sampling_rate * oversampling_ratio
	uint8_t oversampling_ratio;
	// Length of acceleration data point buffer (write to SD-card every (4096 Sa) / (4 kSa/s) = 1.024 s)
	// This option can have a huge impact on RAM usage, it can be reduced if the main function has enough time to save one buffer before the other is filled
	uint32_t a_buffer_len;
	// Length of position data point buffer (write to SD-card every (128 Sa) / (40 Sa/s) = 3.2 s)
	uint32_t p_buffer_len;
} config_t;

extern config_t default_config, config;

#define DEBUG1 HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#define DEBUG2 HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);

// Following defines are called at start and end of a function -> Debug pin is high for entire duration
// While loop in main function
#define DEBUG_MAIN_LOOP ;
// Saving a_buffer_1 to a file
#define DEBUG_A_BUFFER_1_SD ;
// Saving a_buffer_2 to a file
#define DEBUG_A_BUFFER_2_SD ;
// Saving p_buffer_1 to a file
#define DEBUG_P_BUFFER_1_SD ;
// Saving p_buffer_2 to a file
#define DEBUG_P_BUFFER_2_SD ;
// Piezo timer interrupt (@ 4 kHz)
#define DEBUG_A_TIMER ;
// Piezo ADC result (@ 4 kHz)
#define DEBUG_ADC_PZ_CONV ;
// Processing MEMS data
#define DEBUG_ADXL_PROCESS ;
// Processing NMEA char/line
#define DEBUG_NMEA_PROCESS ;

#define DEBUG_TEST_NO_CONFIG_LOG 0
#define DEBUG_TEST_ALWAYS_FORMAT_SD 0
#define DEBUG_TEST_NEVER_FORMAT_SD 0
#define DEBUG_TEST_FIR_FREQUENCY_SWEEP 0
#define DEBUG_TEST_FIR_DAC 0
#define DEBUG_TEST_PRINT_NEW_PAGE 0
#define DEBUG_TEST_FAST_BOOT 0

void Config_Default(void);
void Config_Load(char *buffer, uint32_t size);
void Config_Save(char *buffer, uint32_t size);
HAL_StatusTypeDef Config_Init(ADC_HandleTypeDef *hadc1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3);

#endif /* INC_CONFIG_H_ */
