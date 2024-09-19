/*
 * platform.h
 *
 *  Created on: Aug 21, 2024
 *      Author: mirco
 */

#ifndef INC_PLATFORM_H_
#define INC_PLATFORM_H_

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"

#define BOARD_PROTO 1
#define BOARD_PCB 2

#define BOARD_TYPE BOARD_PCB

#if BOARD_TYPE == BOARD_PROTO
#define NMEA_HUART huart7
#define ADXL_SPI hspi4
#define ADXL_CS_GPIO_Port SPI4_CS_GPIO_Port
#define ADXL_CS_Pin SPI4_CS_Pin
#elif BOARD_TYPE == BOARD_PCB
#define NMEA_HUART huart1
#define ADXL_SPI hspi4
#define ADXL_CS_GPIO_Port SPI4_CS_GPIO_Port
#define ADXL_CS_Pin SPI4_CS_Pin
#endif

#endif /* INC_CONFIG_H_ */
