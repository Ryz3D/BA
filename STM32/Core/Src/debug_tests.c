/*
 * debug_tests.c
 *
 *  Created on: Aug 6, 2024
 *      Author: mirco
 */

#include "debug_tests.h"

void Debug_test_fast_boot(ADC_HandleTypeDef *hadc1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, uint16_t *pz_dma_buffer)
{
	Config_Default();
	Config_Init(hadc1, htim2, htim3);
	HAL_ADC_Start_DMA(hadc1, (uint32_t*)pz_dma_buffer, 9);
	HAL_TIM_Base_Start_IT(htim2);
	HAL_TIM_Base_Start_IT(htim3);
}

void Debug_test_print_config()
{
	printf("(%lu) Compiled Config:\r\n", HAL_GetTick());
	printf("    VERSION=%u\r\n", VERSION);
	printf("    LOAD_CONFIG=%u\r\n", LOAD_CONFIG);
	printf("    OVERSAMPLING_RATIO_MAX=%u\r\n", OVERSAMPLING_RATIO_MAX);
	printf("    PIEZO_COUNT_MAX=%u\r\n", PIEZO_COUNT_MAX);
	printf("    A_BUFFER_LEN_MAX=%u\r\n", A_BUFFER_LEN_MAX);
	printf("    P_BUFFER_LEN_MAX=%u\r\n", P_BUFFER_LEN_MAX);
	printf("    NMEA_DATE_WAIT_DURATION=%u\r\n", NMEA_DATE_WAIT_DURATION);
	printf("    NMEA_PACKET_MERGE_DURATION=%u\r\n", NMEA_PACKET_MERGE_DURATION);
	printf("    NMEA_NO_PACKET_DURATION=%u\r\n", NMEA_NO_PACKET_DURATION);
	char config_buffer[500];
	Config_Save(config_buffer, sizeof(config_buffer));
	printf("(%lu) Loaded Config:\r\n    ", HAL_GetTick());
	for (uint32_t i = 0; i < strlen(config_buffer); i++)
	{
		if (config_buffer[i] == '\n' && i < strlen(config_buffer) - 1)
			printf("\n    ");
		else
			printf("%c", config_buffer[i]);
	}
}

void Debug_test_FIR_frequency_sweep(FIR_t *hfir)
{
	float freqs[400];
	float amps[400];

	for (uint16_t f_i = 0; f_i < 400; f_i++)
	{
		freqs[f_i] = 10 + f_i * 10;
		amps[f_i] = 0;
	}

	for (uint16_t f_i = 0; f_i < 400; f_i++)
	{
		uint32_t sim_t = 0;
		int16_t a_max = 0;
		for (uint16_t iter = 0; iter < 400; iter++)
		{
			for (uint8_t i = 0; i < config.oversampling_ratio; i++)
			{
				hfir->In[i] = (int16_t)(8192.0f * arm_sin_f32(2.0f * PI * freqs[f_i] / 16000.0f * (float)sim_t));
				sim_t++;
			}
			FIR_Update(hfir);
			for (uint8_t i = 0; i < config.oversampling_ratio; i++)
			{
				a_max = hfir->Out[i] > a_max ? hfir->Out[i] : a_max;
			}
		}
		amps[f_i] = a_max;
	}

	printf("[");
	for (uint16_t i = 0; i < 400; i++)
	{
		printf("(%hu, %.4f)", (uint16_t)freqs[i], amps[i] / 8192.0f);
		if (i < 399)
			printf(",");
	}
	printf("]\r\n\r\n");
}

void Debug_test_print_a(volatile a_data_point_t *buffer)
{
	uint8_t ch = 1;
	int16_t pz_min = 5000, pz_max = -5000;
	int32_t mems_min = 1000000000, mems_max = -1000000000;
	for (uint16_t i = 0; i < config.a_buffer_len; i++)
	{
		pz_min = buffer[i].a_piezo[ch] < pz_min ? buffer[i].a_piezo[ch] : pz_min;
		pz_max = buffer[i].a_piezo[ch] > pz_max ? buffer[i].a_piezo[ch] : pz_max;
		mems_min = buffer[i].xyz_mems1[2] < mems_min ? buffer[i].xyz_mems1[2] : mems_min;
		mems_max = buffer[i].xyz_mems1[2] > mems_max ? buffer[i].xyz_mems1[2] : mems_max;
	}
	float pz_ampl = (pz_max - pz_min) / 4095.0f * 3.3f;
	float pz_offs = (pz_min + pz_max) / 4095.0f * 1.65f;
	float mems_ampl = mems_max - mems_min;
	float mems_offs = (mems_min + mems_max) / 2.0f;
	printf("(%lu) Peak-peak: Piezo: %.3f V\tMEMS: %.3f\tOffset: Piezo: %.3f V\tMEMS: %.3f\r\n", HAL_GetTick(), pz_ampl, mems_ampl, pz_offs, mems_offs);
}

void Debug_test_print_p(volatile p_data_point_t *dp)
{
	uint8_t any_valid = 0;
	if (dp->complete & (1 << P_COMPLETE_GNSS_TIME))
	{
		any_valid = 1;
		printf("UTC: %02u:%02u:%06.3f ", dp->gnss_hour, dp->gnss_minute, dp->gnss_second);
	}
	if (dp->complete & (1 << P_COMPLETE_POSITION))
	{
		any_valid = 1;
		printf("Lat/Lon: %.3f %.3f ", dp->lat, dp->lon);
	}
	if (dp->complete & (1 << P_COMPLETE_SPEED))
	{
		any_valid = 1;
		printf("Speed: %.1fkm/h ", dp->speed);
	}
	if (dp->complete & (1 << P_COMPLETE_ALTITUDE))
	{
		any_valid = 1;
		printf("Altitude: %.1fm ", dp->altitude);
	}
	if (any_valid)
	{
		printf("\r\n");
	}
}
