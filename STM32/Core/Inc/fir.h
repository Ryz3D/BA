/*
 * fir.h
 *
 * Wrapper for CMSIS-DSP FIR filter (16-bit int)
 *
 *  Created on: Jul 29, 2024
 *      Author: mirco
 */

#ifndef INC_FIR_H_
#define INC_FIR_H_

#include "stm32f7xx_hal.h"
#include "arm_math.h"
#include "config.h"

#define FIR_TAPS_LEN_MAX 128

typedef struct
{
	arm_fir_instance_q15 Instance;
	q15_t Taps[FIR_TAPS_LEN_MAX];
	uint16_t Nt;
	q15_t State[FIR_TAPS_LEN_MAX + OVERSAMPLING_RATIO_MAX - 1];
	q15_t In[OVERSAMPLING_RATIO_MAX];
	q15_t Out[OVERSAMPLING_RATIO_MAX];
} FIR_t;

HAL_StatusTypeDef FIR_Init(FIR_t *filter);
HAL_StatusTypeDef FIR_Update(FIR_t *filter);

#endif /* INC_FIR_H_ */
