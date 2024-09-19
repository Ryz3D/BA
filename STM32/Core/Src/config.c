/*
 * config.c
 *
 *  Created on: Aug 9, 2024
 *      Author: mirco
 */

#include "config.h"

config_t default_config =
	{
		.boot_without_date = 0, // default: 0
		.print_acceleration_data = 0, // default: 0
		.print_position_data = 0, // default: 0
		.piezo_count = 3, // default: 3
		.fir_type = 1, // default: 1
		.adxl_range = 40, // default: 40 (g)
		.page_duration_ms = 30 * 60 * 1000, // default: 30 * 60 * 1000 (ms) -> 30 minutes
		.a_sampling_rate = 4000, // default: 4000 (Sa/s)
		.p_sampling_rate = 4, // default: 4 (Sa/s)
		.oversampling_ratio = 4, // default: 4 (16 kSa/s)
		.a_buffer_len = 4096, // default: 4096 (Sa)
		.p_buffer_len = 32, // default: 32 (Sa)
	};

config_t config;

#define C_READ_VAR(format, var) \
	if (sscanf(buffer + i, format "%n", &var, &n)) \
		i += n; \

#define C_CHECK_VAR(format, var, v_min, v_max) \
	if (var < v_min || var > v_max) \
	{ \
		typeof(var) v_def = *(typeof(var)*)((void*)&var - (void*)&config + (void*)&default_config); \
		printf("(%lu) WARNING: Config_Load: Invalid value " format ", resetting to " format "\r\n", HAL_GetTick(), var, v_def); \
		var = v_def; \
	}

#define C_WRITE_VAR(format, var) \
	i += sprintf(buffer + i, format "\r\n", var); \
	if (i >= size) \
		return;

HAL_StatusTypeDef Config_Init_ADC1();
HAL_StatusTypeDef Config_Init_TIM2();
HAL_StatusTypeDef Config_Init_TIM3();

void Config_Default(void)
{
	memcpy(&config, &default_config, sizeof(config_t));
}

void Config_Load(char *buffer, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++)
	{
		if (buffer[i] == '\0')
			break;
		if (buffer[i] == '\r' || buffer[i] == '\n')
			continue;

		int n = 0;
		// C_READ_VAR(C_F_, config.);
		C_READ_VAR(C_F_BOOT_WITHOUT_DATE, config.boot_without_date);
		C_READ_VAR(C_F_PRINT_ACCELERATION_DATA, config.print_acceleration_data);
		C_READ_VAR(C_F_PRINT_POSITION_DATA, config.print_position_data);
		C_READ_VAR(C_F_PIEZO_COUNT, config.piezo_count);
		C_READ_VAR(C_F_FIR_TYPE, config.fir_type);
		C_READ_VAR(C_F_ADXL_RANGE, config.adxl_range);
		C_READ_VAR(C_F_PAGE_DURATION_MS, config.page_duration_ms);
		C_READ_VAR(C_F_A_SAMPLING_RATE, config.a_sampling_rate);
		C_READ_VAR(C_F_P_SAMPLING_RATE, config.p_sampling_rate);
		C_READ_VAR(C_F_OVERSAMPLING_RATIO, config.oversampling_ratio);
		C_READ_VAR(C_F_A_BUFFER_LEN, config.a_buffer_len);
		C_READ_VAR(C_F_P_BUFFER_LEN, config.p_buffer_len);
	}

	// C_CHECK_VAR(C_F_, config., 0, 1);
	C_CHECK_VAR(C_F_BOOT_WITHOUT_DATE, config.boot_without_date, 0, 1);
	C_CHECK_VAR(C_F_PRINT_ACCELERATION_DATA, config.print_acceleration_data, 0, 1);
	C_CHECK_VAR(C_F_PRINT_POSITION_DATA, config.print_position_data, 0, 1);
	C_CHECK_VAR(C_F_PIEZO_COUNT, config.piezo_count, 1, PIEZO_COUNT_MAX);
	C_CHECK_VAR(C_F_FIR_TYPE, config.fir_type, 0, 7);
	C_CHECK_VAR(C_F_ADXL_RANGE, config.adxl_range, 10, 40);
	C_CHECK_VAR(C_F_PAGE_DURATION_MS, config.page_duration_ms, 1, 100000000);
	C_CHECK_VAR(C_F_A_SAMPLING_RATE, config.a_sampling_rate, 1, 100000);
	C_CHECK_VAR(C_F_P_SAMPLING_RATE, config.p_sampling_rate, 1, 30);
	C_CHECK_VAR(C_F_OVERSAMPLING_RATIO, config.oversampling_ratio, 1, OVERSAMPLING_RATIO_MAX);
	C_CHECK_VAR(C_F_A_BUFFER_LEN, config.a_buffer_len, 1, A_BUFFER_LEN_MAX);
	C_CHECK_VAR(C_F_P_BUFFER_LEN, config.p_buffer_len, 1, P_BUFFER_LEN_MAX);
}

void Config_Save(char *buffer, uint32_t size)
{
	uint32_t i = 0;
	// C_WRITE_VAR(C_F_, config.);
	C_WRITE_VAR(C_F_BOOT_WITHOUT_DATE, config.boot_without_date);
	C_WRITE_VAR(C_F_PRINT_ACCELERATION_DATA, config.print_acceleration_data);
	C_WRITE_VAR(C_F_PRINT_POSITION_DATA, config.print_position_data);
	C_WRITE_VAR(C_F_PIEZO_COUNT, config.piezo_count);
	C_WRITE_VAR(C_F_FIR_TYPE, config.fir_type);
	C_WRITE_VAR(C_F_ADXL_RANGE, config.adxl_range);
	C_WRITE_VAR(C_F_PAGE_DURATION_MS, config.page_duration_ms);
	C_WRITE_VAR(C_F_A_SAMPLING_RATE, config.a_sampling_rate);
	C_WRITE_VAR(C_F_P_SAMPLING_RATE, config.p_sampling_rate);
	C_WRITE_VAR(C_F_OVERSAMPLING_RATIO, config.oversampling_ratio);
	C_WRITE_VAR(C_F_A_BUFFER_LEN, config.a_buffer_len);
	C_WRITE_VAR(C_F_P_BUFFER_LEN, config.p_buffer_len);
}

HAL_StatusTypeDef Config_Init(ADC_HandleTypeDef *hadc1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3)
{
	if (Config_Init_ADC1(hadc1) != HAL_OK)
	{
		printf("(%lu) ERROR: Config_Init: ADC1 Init failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	if (Config_Init_TIM2(htim2) != HAL_OK)
	{
		printf("(%lu) ERROR: Config_Init: TIM2 Init failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	if (Config_Init_TIM3(htim3) != HAL_OK)
	{
		printf("(%lu) ERROR: Config_Init: TIM3 Init failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef Config_Init_ADC1(ADC_HandleTypeDef *hadc1)
{
	ADC_ChannelConfTypeDef sConfig = {
		0 };
	hadc1->Init.NbrOfConversion = config.piezo_count;
	if (HAL_ADC_Init(hadc1) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (config.piezo_count >= 1)
	{
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	if (config.piezo_count >= 2)
	{
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = ADC_REGULAR_RANK_2;
		if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	if (config.piezo_count >= 3)
	{
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = ADC_REGULAR_RANK_3;
		if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	if (config.piezo_count >= 4)
	{
		sConfig.Channel = ADC_CHANNEL_6;
		sConfig.Rank = ADC_REGULAR_RANK_4;
		if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	if (config.piezo_count >= 5)
	{
		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = ADC_REGULAR_RANK_5;
		if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef Config_Init_TIM2(TIM_HandleTypeDef *htim2)
{
	// Clock source 108 MHz
	// f = config.a_sampling_rate * config.oversampling_ratio
	htim2->Init.Prescaler = 0;
	htim2->Init.Period = 108000UL / (config.a_sampling_rate * config.oversampling_ratio / 1000) - 1;
	if (HAL_TIM_Base_Init(htim2) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef Config_Init_TIM3(TIM_HandleTypeDef *htim3)
{
	// Clock source TIM2
	// f = config.a_sampling_rate
	htim3->Init.Prescaler = 0;
	htim3->Init.Period = config.oversampling_ratio - 1;
	if (HAL_TIM_Base_Init(htim3) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}
