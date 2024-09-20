/*
 * Copyright (c) 2024 Mirco Heitmann
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * platform.h
 */

#ifndef INC_PLATFORM_H_
#define INC_PLATFORM_H_

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"

// Do not change
#define BOARD_PROTO 1
#define BOARD_PCB 2

// Select current board type here
#define BOARD_TYPE BOARD_PCB

#if BOARD_TYPE == BOARD_PROTO
#define NMEA_HUART huart7
#define ADXL_SPI hspi4
#define ADXL_CS_GPIO_Port SPI4_CS_GPIO_Port
#define ADXL_CS_Pin SPI4_CS_Pin
#define LED_GNSS_LOCK LD1_GPIO_Port, LD1_Pin
#define LED_ACTIVE LD2_GPIO_Port, LD2_Pin
#define LED_ERROR LD3_GPIO_Port, LD3_Pin
#elif BOARD_TYPE == BOARD_PCB
#define NMEA_HUART huart1
#define ADXL_SPI hspi4
#define ADXL_CS_GPIO_Port SPI4_CS_GPIO_Port
#define ADXL_CS_Pin SPI4_CS_Pin
#define LED_GNSS_LOCK LD1_GPIO_Port, LD1_Pin
#define LED_ACTIVE LD2_GPIO_Port, LD2_Pin
#define LED_ERROR LD3_GPIO_Port, LD3_Pin
#endif

#endif /* INC_CONFIG_H_ */
