/*
 * Copyright (c) 2024 Mirco Heitmann
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * fir.c
 *
 * Wrapper for CMSIS-DSP FIR filter (16-bit int)
 */

#include <fir.h>

// arm_fir requires taps as reversed array, set to 1 if taps array has not been reversed in source
#define FIR_REV 1

HAL_StatusTypeDef FIR_Init(FIR_t *hfir)
{
	// Init struct
#if FIR_REV
	for (uint16_t i = 0; i < hfir->Nt; i++)
	{
		q15_t temp = hfir->Taps[i];
		hfir->Taps[i] = hfir->Taps[hfir->Nt - 1 - i];
		hfir->Taps[hfir->Nt - 1 - i] = temp;
	}
#endif
	for (uint16_t i = 0; i < hfir->Nt + config.oversampling_ratio - 1; i++)
	{
		hfir->State[i] = 0;
	}
	for (uint16_t i = 0; i < config.oversampling_ratio; i++)
	{
		hfir->In[i] = hfir->Out[i] = 0;
	}
	// Init CMSIS-DSP FIR
	arm_fir_init_q15(&hfir->Instance, hfir->Nt, (q15_t*)hfir->Taps, hfir->State, config.oversampling_ratio);
	return HAL_OK;
}

HAL_StatusTypeDef FIR_Update(FIR_t *hfir)
{
	// Calculate CMSIS-DSP FIR
	arm_fir_q15(&hfir->Instance, hfir->In, hfir->Out, config.oversampling_ratio);
	return HAL_OK;
}
