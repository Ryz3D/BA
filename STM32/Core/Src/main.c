/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "platform.h"
#include "log.h"
#include "sd.h"
#include "config.h"
#include "data_points.h"
#include "debug_tests.h"
#include "adxl.h"
#include "nmea.h"
#include "fir.h"
#include "fir_taps.h"
#include "double_buffering.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
Log_t hlog; // Logger
Vera_SD_t hvsd1; // SD card
ADXL_t hadxl; // MEMS sensor ADXL-357
NMEA_t hnmea; // GNSS module Navilock 62528
FIR_t hfir_pz[PIEZO_COUNT_MAX]; // FIR filters for ADC channels
Double_Buffer_t hbuffer_a, hbuffer_p; // Manages double buffering flags for acceleration and position buffers

uint32_t last_page_change = 0; // Time of last call to SD_NewPage
volatile uint8_t capture_running = 0; // 0: Not running, 1: running
volatile uint32_t ticks_counter = 0; // Increments with each acceleration data point
uint32_t time_p_inc = 0; // When a valid NMEA packet is received, this is set to end time of current data point (NMEA_PACKET_MERGE_DURATION)
uint32_t time_p_last = 0; // Time of last NMEA packet
uint32_t time_p_last_lock = 0; // Time of last NMEA packet with valid position

volatile uint16_t pz_dma_buffer[PIEZO_COUNT_MAX * OVERSAMPLING_RATIO_MAX];

volatile a_data_point_t a_buffer_1[A_BUFFER_LEN_MAX];
volatile a_data_point_t a_buffer_2[A_BUFFER_LEN_MAX];
volatile a_data_point_t *a_current_data_point;
volatile p_data_point_t p_buffer_1[P_BUFFER_LEN_MAX];
volatile p_data_point_t p_buffer_2[P_BUFFER_LEN_MAX];
volatile p_data_point_t *p_current_data_point;

volatile uint8_t flag_complete_a_mems = 0;
volatile uint8_t flag_complete_a_pz = 0;
volatile uint8_t flag_complete_p_position = 0;
volatile uint8_t flag_complete_p_speed = 0;
volatile uint8_t flag_complete_p_altitude = 0;
volatile uint8_t flag_complete_p_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Main_Save_a_Buffer(volatile a_data_point_t *buffer);
void Main_Save_p_Buffer(volatile p_data_point_t *buffer);
void Main_Double_Buffer_Loop();
void Main_NMEA_Loop();
void Main_Increment_a_Buffer();
void Main_Increment_p_Buffer();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SDMMC1_SD_Init();
	MX_FATFS_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_DAC_Init();
	MX_SPI4_Init();
	MX_UART7_Init();
	MX_USB_DEVICE_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	// Check if card should be formatted -> either by config or by button push
	uint8_t do_format_sd = DEBUG_TEST_ALWAYS_FORMAT_SD;
#if DEBUG_TEST_NEVER_FORMAT_SD
	do_format_sd = 0;
#else
	if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
	{
		do_format_sd = 1;
	}
#endif

	// Boot blink sequence
	for (uint8_t i = 0; i < 3; i++)
	{
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, i == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, i == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, i == 2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_Delay(125);
	}
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Debug1_GPIO_Port, Debug1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Debug2_GPIO_Port, Debug2_Pin, GPIO_PIN_RESET);

	if (do_format_sd)
	{
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	}

	// Give some time to connect virtual COM port
	HAL_Delay(2500);

	hlog.huart = &huart3;
	hlog.hvsd = &hvsd1;
	hlog.flush_timeout = 100;
	Log_Init(&hlog);

	printf("\r\n\r\n(%lu) Booting...\r\n", HAL_GetTick());

#if DEBUG_TEST_FAST_BOOT
	Debug_test_fast_boot(&hadc1, &htim2, &htim3, (uint16_t*)pz_dma_buffer);
	capture_running = 1;
	while (1)
		;
#endif

	// Init SD card
	hvsd1.Detect_GPIO_Port = uSD_Detect_GPIO_Port;
	hvsd1.Detect_Pin = uSD_Detect_Pin;
	hvsd1.fatfs = &SDFatFS;
	hvsd1.fatfs_file = &SDFile;
	hvsd1.fatfs_path = SDPath;

	if (SD_Init(&hvsd1, do_format_sd) != HAL_OK)
	{
		Error_Handler();
	}
	printf("(%lu) SD card initialized\r\n", HAL_GetTick());

	// Load default configuration
	Config_Default();
#if LOAD_CONFIG
	// Load configuration from SD card
	char config_buffer[500];
	if (SD_FileExists(&hvsd1, CONFIG_FILE_PATH))
	{
		printf("(%lu) Config found, reading...\r\n", HAL_GetTick());
		UINT config_buffer_size = 0;
		if (SD_ReadBuffer(&hvsd1, CONFIG_FILE_PATH, config_buffer, sizeof(config_buffer), &config_buffer_size) != HAL_OK)
		{
			Error_Handler();
		}
		config_buffer[config_buffer_size] = '\0';
		Config_Load(config_buffer, strlen(config_buffer));
	}
	else
	{
		printf("(%lu) No config found, writing default...\r\n", HAL_GetTick());
		Config_Save(config_buffer, sizeof(config_buffer));
		if (SD_TouchFile(&hvsd1, CONFIG_FILE_PATH) != HAL_OK)
		{
			Error_Handler();
		}
		if (SD_WriteBuffer(&hvsd1, CONFIG_FILE_PATH, config_buffer, strlen(config_buffer)) != HAL_OK)
		{
			Error_Handler();
		}
	}
#endif
#if !DEBUG_TEST_NO_CONFIG_LOG
	Debug_test_print_config();
#endif

	// Initialize based on configuration
	if (Config_Init(&hadc1, &htim2, &htim3) != HAL_OK)
	{
		Error_Handler();
	}
	printf("(%lu) Configuration loaded\r\n", HAL_GetTick());

	// Init acceleration double buffering
	hbuffer_a.buffer_len = config.a_buffer_len;
	hbuffer_a.buffer1 = a_buffer_1;
	hbuffer_a.buffer2 = a_buffer_2;
	hbuffer_a.element_size = sizeof(a_data_point_t);
	Double_Buffer_Init(&hbuffer_a);
	a_current_data_point = Double_Buffer_Current(&hbuffer_a);

	// Init position double buffering
	hbuffer_p.buffer_len = config.p_buffer_len;
	hbuffer_p.buffer1 = p_buffer_1;
	hbuffer_p.buffer2 = p_buffer_2;
	hbuffer_p.element_size = sizeof(p_data_point_t);
	Double_Buffer_Init(&hbuffer_p);
	p_current_data_point = Double_Buffer_Current(&hbuffer_p);

	// Initialize ADXL357
	hadxl.hspi = &ADXL_SPI;
	hadxl.sampling_rate = config.a_sampling_rate;
	hadxl.acceleration_range = config.adxl_range;
	hadxl.timeout = 100;
	hadxl.CS_GPIO_Port = ADXL_CS_GPIO_Port;
	hadxl.CS_Pin = ADXL_CS_Pin;
	if (ADXL_Init(&hadxl) == HAL_ERROR)
	{
		Error_Handler();
	}

	// Initialize u-blox 8
	hnmea.huart = &NMEA_HUART;
	hnmea.baud = 115200;
	hnmea.tx_timeout = 1000;
	hnmea.rx_timeout = 1000;
	hnmea.sampling_rate = config.p_sampling_rate;
	if (NMEA_Init(&hnmea) == HAL_ERROR)
	{
		Error_Handler();
	}

	if (!config.boot_without_date)
	{
		NMEA_Data_t gnss_data = NMEA_GetDate(&hnmea);
		if (gnss_data.date_valid)
		{
			hvsd1.date_year = gnss_data.year + 2000;
			hvsd1.date_month = gnss_data.month;
			hvsd1.date_day = gnss_data.day;
			printf("(%lu) GNSS date: %04hu-%02u-%02u ", HAL_GetTick(), hvsd1.date_year, hvsd1.date_month, hvsd1.date_day);
			if (gnss_data.time_valid)
			{
				printf("%02u:%02u:%06.3f\r\n", gnss_data.hour, gnss_data.minute, gnss_data.second);
			}
			else
			{
				printf("\r\n");
			}
		}
		else
		{
			printf("(%lu) WARNING: No GNSS date received, saving as %04hu_%02u_%02u\r\n", HAL_GetTick(), hvsd1.date_year, hvsd1.date_month, hvsd1.date_day);
		}
	}

	// Create directory, initialize files
	if (SD_InitDir(&hvsd1) != HAL_OK)
	{
		// Try writing to different dir in case of error
		sprintf(hvsd1.dir_path, DIR_FORMAT, 1, 1, 1, 1L);
		printf("(%lu) WARNING: SD_Init_Dir failed, writing to error dir %s\r\n", HAL_GetTick(), hvsd1.dir_path);
		FRESULT res;
		if ((res = f_mkdir(hvsd1.dir_path)) != FR_OK)
		{
			printf("(%lu) ERROR: main: Create error dir (\"%s\") failed\r\n", HAL_GetTick(), hvsd1.dir_path);
			Error_Handler();
			while (1)
				;
		}
		if (SD_UpdateFilepaths(&hvsd1) != HAL_OK)
		{
			printf("(%lu) ERROR: main: SD_UpdateFilepaths failed\r\n", HAL_GetTick());
			Error_Handler();
			while (1)
				;
		}
	}
	printf("(%lu) Writing dir \"%s\"\r\n", HAL_GetTick(), hvsd1.dir_path);

	// Init digital FIR filter
	if (config.fir_type > 0)
	{
		for (uint8_t i_ch = 0; i_ch < config.piezo_count; i_ch++)
		{
			memcpy(hfir_pz[i_ch].Taps, fir_taps_types[config.fir_type], fir_taps_lens[config.fir_type] * sizeof(q15_t));
			hfir_pz[i_ch].Nt = fir_taps_lens[config.fir_type];
			FIR_Init(&hfir_pz[i_ch]);
		}
	}

	a_current_data_point->timestamp = 0;
	p_current_data_point->timestamp = 0;

#if DEBUG_TEST_FIR_FREQUENCY_SWEEP
	Debug_test_FIR_frequency_sweep(&hfir_pz[0]);
#endif

#if DEBUG_TEST_FIR_DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
#endif

	// If button is held down, wait for release. Otherwise capture is stopped immediately
	while (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
		;
	// Button debounce
	HAL_Delay(250);

	// Start piezo ADC
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pz_dma_buffer, config.piezo_count * config.oversampling_ratio) == HAL_ERROR)
	{
		printf("(%lu) ERROR: main: ADC1 HAL_ADC_Start_DMA failed\r\n", HAL_GetTick());
		Error_Handler();
	}
	// Start oversampling timer
	if (HAL_TIM_Base_Start_IT(&htim2) == HAL_ERROR)
	{
		printf("(%lu) ERROR: main: HAL_TIM_Base_Start_IT failed\r\n", HAL_GetTick());
		Error_Handler();
	}
	// Start regular piezo timer
	if (HAL_TIM_Base_Start_IT(&htim3) == HAL_ERROR)
	{
		printf("(%lu) ERROR: main: HAL_TIM_Base_Start_IT failed\r\n", HAL_GetTick());
		Error_Handler();
	}

	uint32_t boot_duration = HAL_GetTick();
	printf("(%lu) Capture started\r\n", boot_duration);
	HAL_Delay(hlog.flush_timeout);
	Log_Loop(&hlog);

	// Write file headers
	hvsd1.a_header.version = VERSION;
	hvsd1.a_header.a_buffer_len = config.a_buffer_len;
	hvsd1.a_header.a_sampling_rate = config.a_sampling_rate;
	hvsd1.a_header.boot_duration = boot_duration;
	hvsd1.a_header.fir_taps_len = fir_taps_lens[config.fir_type];
	hvsd1.a_header.oversampling_ratio = config.oversampling_ratio;
	hvsd1.a_header.piezo_count_max = PIEZO_COUNT_MAX;
	hvsd1.a_header.piezo_count = config.piezo_count;
	hvsd1.p_header.version = VERSION;
	hvsd1.p_header.p_buffer_len = config.p_buffer_len;
	hvsd1.p_header.p_sampling_rate = config.p_sampling_rate;
	hvsd1.p_header.boot_duration = boot_duration;
	hvsd1.p_header.year = hvsd1.date_year;
	hvsd1.p_header.month = hvsd1.date_month;
	hvsd1.p_header.day = hvsd1.date_day;
	SD_NewPage(&hvsd1);
	// Offset so page change doesn't happen simultaneously with buffer save -> more even file size
	last_page_change = HAL_GetTick() - 181;

	// Wait for NMEA_NO_PACKET_DURATION from this point
	time_p_last = HAL_GetTick();
	time_p_last_lock = HAL_GetTick();

#if DEBUG_TEST_PRINT_NEW_PAGE
	printf("(%lu) Page %li (\"%s\", \"%s\")\r\n", HAL_GetTick(), hvsd1.page_num, hvsd1.a_file_path, hvsd1.p_file_path);
#endif
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	capture_running = 1;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (capture_running)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		DEBUG_MAIN_LOOP

		// Blink dir_num with LD1 (green)
		if (HAL_GetTick() - hvsd1.a_header.boot_duration < 2000)
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		}
		else if (HAL_GetTick() - hvsd1.a_header.boot_duration < 2200)
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		}
		else if (HAL_GetTick() - hvsd1.a_header.boot_duration < 2200 + hvsd1.dir_num * 400)
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, (HAL_GetTick() - hvsd1.a_header.boot_duration - 2200) % 400 < 200);
		}

		Main_Double_Buffer_Loop();
		Main_NMEA_Loop();

		if (HAL_GetTick() - last_page_change > config.page_duration_ms)
		{
			SD_NewPage(&hvsd1);
#if DEBUG_TEST_PRINT_NEW_PAGE
			printf("(%lu) Page %li (\"%s\", \"%s\")\r\n", HAL_GetTick(), hvsd1.page_num, hvsd1.a_file_path, hvsd1.p_file_path);
#endif
			last_page_change = HAL_GetTick();
		}

		Log_Loop(&hlog);

		DEBUG_MAIN_LOOP

		if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
		{
			capture_running = 0;
			break;
		}
	}

	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_ADC_Stop_IT(&hadc1);

	// Save remaining data
	Double_Buffer_Flush(&hbuffer_a);
	Double_Buffer_Flush(&hbuffer_p);
	Main_Double_Buffer_Loop();

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	printf("(%lu) Capture stopped (\"%s\")\r\n", HAL_GetTick(), hvsd1.dir_path);

	Log_Uninit(&hlog);
	SD_Uninit(&hvsd1);

	while (1)
		;
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 5;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void)
{

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */

	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

}

/**
 * @brief SDMMC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDMMC1_SD_Init(void)
{

	/* USER CODE BEGIN SDMMC1_Init 0 */

	/* USER CODE END SDMMC1_Init 0 */

	/* USER CODE BEGIN SDMMC1_Init 1 */

	/* USER CODE END SDMMC1_Init 1 */
	hsd1.Instance = SDMMC1;
	hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
	hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd1.Init.ClockDiv = 0;
	/* USER CODE BEGIN SDMMC1_Init 2 */

	/* USER CODE END SDMMC1_Init 2 */

}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void)
{

	/* USER CODE BEGIN SPI4_Init 0 */

	/* USER CODE END SPI4_Init 0 */

	/* USER CODE BEGIN SPI4_Init 1 */

	/* USER CODE END SPI4_Init 1 */
	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 7;
	hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI4_Init 2 */

	/* USER CODE END SPI4_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 6750 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 4 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void)
{

	/* USER CODE BEGIN UART7_Init 0 */

	/* USER CODE END UART7_Init 0 */

	/* USER CODE BEGIN UART7_Init 1 */

	/* USER CODE END UART7_Init 1 */
	huart7.Instance = UART7;
	huart7.Init.BaudRate = 9600;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_TX_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart7.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	if (HAL_UART_Init(&huart7) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART7_Init 2 */

	/* USER CODE END UART7_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* DMA2_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, SPI4_CS_Pin | Debug1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Debug2_GPIO_Port, Debug2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : SPI4_CS_Pin Debug1_Pin */
	GPIO_InitStruct.Pin = SPI4_CS_Pin | Debug1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDIO_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin = RMII_MDIO_Pin | RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Debug2_Pin */
	GPIO_InitStruct.Pin = Debug2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(Debug2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : uSD_Detect_Pin USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = uSD_Detect_Pin | USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_DRDY_Pin MEMS_INT2_Pin MEMS_INT1_Pin */
	GPIO_InitStruct.Pin = MEMS_DRDY_Pin | MEMS_INT2_Pin | MEMS_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_UART5;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 *   None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	Log_Char(&hlog, ch);
	return ch;
}

void Main_Save_a_Buffer(volatile a_data_point_t *buffer)
{
	if (SD_WriteBuffer(&hvsd1, hvsd1.a_file_path, (void*)buffer, hbuffer_a.save_len * sizeof(a_data_point_t)) != HAL_OK)
	{
		Error_Handler();
	}
	if (config.print_acceleration_data)
	{
		Debug_test_print_a(buffer);
	}
}

void Main_Save_p_Buffer(volatile p_data_point_t *buffer)
{
	if (SD_WriteBuffer(&hvsd1, hvsd1.p_file_path, (void*)buffer, hbuffer_p.save_len * sizeof(p_data_point_t)) != HAL_OK)
	{
		Error_Handler();
	}
}

void Main_Double_Buffer_Loop()
{
	// If a_buffer_1 is ready to save
	if (hbuffer_a.flag_save_buffer_1)
	{
		DEBUG_A_BUFFER_1_SD
		Main_Save_a_Buffer(a_buffer_1);
		// Flag a_buffer_1 as saved
		hbuffer_a.flag_save_buffer_1 = 0;
		DEBUG_A_BUFFER_1_SD
	}
	// If a_buffer_2 is ready to save
	if (hbuffer_a.flag_save_buffer_2)
	{
		DEBUG_A_BUFFER_2_SD
		Main_Save_a_Buffer(a_buffer_2);
		// Flag a_buffer_2 as saved
		hbuffer_a.flag_save_buffer_2 = 0;
		DEBUG_A_BUFFER_2_SD
	}
	if (hbuffer_a.flag_overflow)
	{
		printf("(%lu) WARNING: main: a_buffer overflow\r\n", HAL_GetTick());
		hbuffer_a.flag_overflow = 0;
	}

	// If p_buffer_1 is ready to save
	if (hbuffer_p.flag_save_buffer_1)
	{
		DEBUG_P_BUFFER_1_SD
		Main_Save_p_Buffer(p_buffer_1);
		// Flag p_buffer_1 as saved
		hbuffer_p.flag_save_buffer_1 = 0;
		DEBUG_P_BUFFER_1_SD
	}
	// If p_buffer_2 is ready to save
	if (hbuffer_p.flag_save_buffer_2)
	{
		DEBUG_P_BUFFER_2_SD
		Main_Save_p_Buffer(p_buffer_2);
		// Flag p_buffer_2 as saved
		hbuffer_p.flag_save_buffer_2 = 0;
		DEBUG_P_BUFFER_2_SD
	}
	if (hbuffer_p.flag_overflow)
	{
		printf("(%lu) WARNING: main: p_buffer overflow\r\n", HAL_GetTick());
		hbuffer_p.flag_overflow = 0;
	}
}

void Main_NMEA_Loop()
{
	// Parse buffered NMEA data
	NMEA_Data_t data;
	while (NMEA_ProcessLine(&hnmea, &data))
	{
		uint8_t any_valid = 0;
		if (data.position_valid)
		{
			any_valid = 1;
			time_p_last_lock = HAL_GetTick();
			flag_complete_p_position = 1;
			p_current_data_point->lat = data.lat;
			p_current_data_point->lon = data.lon;

			// Activate GNSS lock LED
			if (HAL_GetTick() - hvsd1.a_header.boot_duration > 4000 + hvsd1.dir_num * 400)
			{
				HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			}
		}
		if (data.speed_valid)
		{
			any_valid = 1;
			flag_complete_p_speed = 1;
			p_current_data_point->speed = data.speed_kmh;
		}
		if (data.altitude_valid)
		{
			any_valid = 1;
			flag_complete_p_altitude = 1;
			p_current_data_point->altitude = data.altitude;
		}
		if (data.time_valid)
		{
			any_valid = 1;
			flag_complete_p_time = 1;
			p_current_data_point->gnss_hour = data.hour;
			p_current_data_point->gnss_minute = data.minute;
			p_current_data_point->gnss_second = data.second;
		}
		if (any_valid)
		{
			time_p_last = HAL_GetTick();
			// If timer for incrementing the data point index is not running yet
			if (time_p_inc == 0)
			{
				// Start the timer
				time_p_inc = HAL_GetTick() + NMEA_PACKET_MERGE_DURATION;
			}
			// If the timer was running, the data was merged into the same p_data_point_t and the timer keeps running
		}
	}
	// Next position data point if timer ended
	if (HAL_GetTick() > time_p_inc && time_p_inc != 0)
	{
		time_p_inc = 0;
		Main_Increment_p_Buffer();
	}
	// Warning if no NMEA data
	if (HAL_GetTick() - time_p_last > NMEA_NO_PACKET_DURATION && time_p_last != 0)
	{
		printf("(%lu) WARNING: No NMEA data\r\n", HAL_GetTick());
		time_p_last = 0;
	}
	// Warning if no position lock
	if (HAL_GetTick() - time_p_last_lock > NMEA_NO_PACKET_DURATION && time_p_last_lock != 0)
	{
		printf("(%lu) WARNING: No position lock\r\n", HAL_GetTick());
		// Deactivate GNSS lock LED
		if (HAL_GetTick() - hvsd1.a_header.boot_duration > 4000 + hvsd1.dir_num * 400)
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		}
		time_p_last_lock = 0;
	}
}

void Main_Increment_a_Buffer()
{
	// Set complete bits of last data point
	a_current_data_point->complete |= (1 << A_COMPLETE_TIMESTAMP)
		| (flag_complete_a_mems << A_COMPLETE_MEMS)
		| (flag_complete_a_pz << A_COMPLETE_PZ);

	if (ticks_counter > 1)
	{
		Double_Buffer_Increment(&hbuffer_a);
		a_current_data_point = Double_Buffer_Current(&hbuffer_a);
	}

	a_current_data_point->timestamp = ticks_counter;
}

void Main_Increment_p_Buffer()
{
	// Set complete bits of last data point
	p_current_data_point->complete |= (1 << P_COMPLETE_TIMESTAMP)
		| (flag_complete_p_time << P_COMPLETE_GNSS_TIME)
		| (flag_complete_p_position << P_COMPLETE_POSITION)
		| (flag_complete_p_speed << P_COMPLETE_SPEED)
		| (flag_complete_p_altitude << P_COMPLETE_ALTITUDE);

	if (config.print_position_data)
	{
		Debug_test_print_p(p_current_data_point);
	}

	Double_Buffer_Increment(&hbuffer_p);
	p_current_data_point = Double_Buffer_Current(&hbuffer_p);

	p_current_data_point->timestamp = ticks_counter;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		DEBUG_A_TIMER

		if (capture_running)
		{
			// Increment data point index
			Main_Increment_a_Buffer();
			ticks_counter++;

			// Request MEMS data packet
			if (ADXL_RequestData(&hadxl) == HAL_ERROR)
			{
				printf("(%lu) WARNING: ADXL_RequestData: TxRx failed\r\n", HAL_GetTick());
			}
		}

		DEBUG_A_TIMER
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (capture_running)
	{
		// ADC for piezos
		if (hadc->Instance == ADC1)
		{
			DEBUG_ADC_PZ_CONV

			for (uint8_t i_sample = 0; i_sample < config.oversampling_ratio; i_sample++)
			{
				for (uint8_t i_ch = 0; i_ch < config.piezo_count; i_ch++)
				{
					hfir_pz[i_ch].In[i_sample] = (pz_dma_buffer[i_sample * config.piezo_count + i_ch] << 1) - 4094;
				}
			}
			for (uint8_t i_ch = 0; i_ch < config.piezo_count; i_ch++)
			{
				if (config.fir_type > 0)
				{
					FIR_Update(&hfir_pz[i_ch]);
					a_current_data_point->a_piezo[i_ch] = hfir_pz[i_ch].Out[0];
				}
				else
				{
					a_current_data_point->a_piezo[i_ch] = hfir_pz[i_ch].In[0];
				}
			}
			flag_complete_a_pz = 1;

#if DEBUG_TEST_FIR_DAC
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ((uint16_t)hfir_pz[1].Out[0] >> 1) + 2047);
#endif

			DEBUG_ADC_PZ_CONV
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == hadxl.hspi->Instance)
	{
		DEBUG_ADXL_PROCESS

		ADXL_Data_t adxl_data = ADXL_RxCallback(&hadxl);
		a_current_data_point->xyz_mems1[0] = adxl_data.x;
		a_current_data_point->xyz_mems1[1] = adxl_data.y;
		a_current_data_point->xyz_mems1[2] = adxl_data.z;
		a_current_data_point->temp_mems1 = adxl_data.temp;
		flag_complete_a_mems = adxl_data.data_valid;

		DEBUG_ADXL_PROCESS
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == hnmea.huart->Instance)
	{
		DEBUG_NMEA_PROCESS

		if (NMEA_ProcessDMABuffer(&hnmea) == HAL_ERROR)
		{
			Error_Handler();
		}

		DEBUG_NMEA_PROCESS
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	printf("(%lu) Fatal Error, but attempting to continue\r\n", HAL_GetTick());
	/*
	 SD_FlushLog();
	 SD_Uninit();
	 __disable_irq();
	 while (1)
	 ;
	 */
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
