/*
 * Copyright (c) 2024 Mirco Heitmann
 * All rights reserved.
 * 
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 * 
 * debug_tests.h
 */

#ifndef INC_DEBUG_TESTS_H_
#define INC_DEBUG_TESTS_H_

#include <stdio.h>

#include "config.h"
#include "fir.h"
#include "data_points.h"

void Debug_test_fast_boot(ADC_HandleTypeDef *hadc1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, uint16_t *pz_dma_buffer);
void Debug_test_print_config();
void Debug_test_FIR_frequency_sweep(FIR_t *hfir);
void Debug_test_print_a(volatile a_data_point_t *buffer);
void Debug_test_print_p(volatile p_data_point_t *dp);

#endif /* INC_DEBUG_TESTS_H_ */
