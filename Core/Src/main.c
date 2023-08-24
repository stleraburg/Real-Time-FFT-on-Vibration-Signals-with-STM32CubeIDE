/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
#include "math.h"
#include <arm_math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "fatfs_sd_card.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//Must the power of 2!
#define FFT_BUFFER_SIZE 1024
#define VIBRO_BUFFER_SIZE 256
#define SAMPLE_RATE 1000.0f

//Two buffers to collect raw data from two ADXL335 accelerometers sampled by ADC using DMA transfer
uint32_t accelRight[VIBRO_BUFFER_SIZE], accelLeft[VIBRO_BUFFER_SIZE];

//Two input and output buffers for FFT analysis
float fftBufInRight[FFT_BUFFER_SIZE];
float fftBufInLeft[FFT_BUFFER_SIZE];
float fftBufOutRight[FFT_BUFFER_SIZE];
float fftBufOutLeft[FFT_BUFFER_SIZE];

//Two FFT instances from CMSIS DSP arm_math.h library
arm_rfft_fast_instance_f32 R;
arm_rfft_fast_instance_f32 L;

//Flags to know when the DMA callback functions are triggered
uint8_t processHalfFlag = 0;
uint8_t fftHalfFlag = 0;
uint8_t fftFullFlag = 0;
uint8_t fftFlag = 0;
uint16_t freqRight = 0;
uint16_t freqLeft = 0;

uint32_t tempRight = 0;
uint32_t tempLeft = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
void process_SD_card(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//sine wave generation
#define NS  128

uint32_t Wave_LUT[NS] = { 512, 537, 562, 587, 612, 636, 661, 684, 708, 731, 753,
		775, 796, 817, 837, 856, 874, 891, 908, 923, 938, 951, 964, 975, 985,
		994, 1002, 1009, 1014, 1018, 1022, 1023, 1024, 1023, 1022, 1018, 1014,
		1009, 1002, 994, 985, 975, 964, 951, 938, 923, 908, 891, 874, 856, 837,
		817, 796, 775, 753, 731, 708, 684, 661, 636, 612, 587, 562, 537, 512,
		487, 462, 437, 412, 388, 363, 340, 316, 293, 271, 249, 228, 207, 187,
		168, 150, 133, 116, 101, 86, 73, 60, 49, 39, 30, 22, 15, 10, 6, 2, 1, 0,
		1, 2, 6, 10, 15, 22, 30, 39, 49, 60, 73, 86, 101, 116, 133, 150, 168,
		187, 207, 228, 249, 271, 293, 316, 340, 363, 388, 412, 437, 462, 487 };

void Process_HalfBuffer() {

	static int16_t fftIndex = 0;

	uint16_t lowerLimit = 0;
	uint16_t upperLimit = 0;

	if (fftHalfFlag) {
		lowerLimit = 0;
		upperLimit = VIBRO_BUFFER_SIZE / 2;
	}

	if (fftFullFlag) {
		lowerLimit = VIBRO_BUFFER_SIZE / 2;
		upperLimit = VIBRO_BUFFER_SIZE;
	}

	for (uint16_t n = lowerLimit; n < upperLimit; n++) {

		fftBufInRight[fftIndex] = (float) accelRight[n];
		fftBufInLeft[fftIndex] = (float) accelLeft[n];

		fftIndex++;

		if (fftIndex == FFT_BUFFER_SIZE) {

			/*Only the first half of the FFT output buffer provides the correct result
				due to a symmetric nature of real-valued signals. */

			arm_rfft_fast_f32(&L, fftBufInLeft, fftBufOutLeft, 0);
			arm_cmplx_mag_f32(fftBufOutLeft, fftBufOutLeft,
					FFT_BUFFER_SIZE / 2);
			fftBufOutLeft[0] = 0;

			arm_rfft_fast_f32(&R, fftBufInRight, fftBufOutRight, 0);
			arm_cmplx_mag_f32(fftBufOutRight, fftBufOutRight,
					FFT_BUFFER_SIZE / 2);
			fftBufOutRight[0] = 0;

			//Use arm_max_f32() to print the dominant frequency
			/*
			*************************************************************
			float peakValRight = 0;
			uint32_t freqRight = 0;
			float peakValLeft = 0;
			uint32_t freqLeft = 0;

			arm_max_f32(fftBufOutRight, FFT_BUFFER_SIZE, &peakValRight,
					&freqRight);
			arm_max_f32(fftBufOutLeft, FFT_BUFFER_SIZE, &peakValLeft,
					&freqLeft);
			printf("Magnitude Right: %f, Frequency: %lu \n", peakValRight,
					freqRight);
			printf("Magnitude Left: %f, Frequency: %lu \n", peakValLeft,
					freqLeft);

			printf("\n");
			*************************************************************
			*/


			//This part of the code might be useful for printing the frequencies of several harmonics of a signal
			/*
			*************************************************************
			float curValRight = 0;
			float peakValRight = fftBufOutLeft[1];
			float curValLeft = 0;
			float peakValLeft = fftBufOutRight[1];

			for (uint16_t index = 1; index < FFT_BUFFER_SIZE/2; index ++) {
			 curValRight = fftBufOutRight[index];
			 curValLeft = fftBufOutLeft[index];

			 uint16_t frequency = index * SAMPLE_RATE / FFT_BUFFER_SIZE;

			 if (curValRight > peakValRight) {
			 peakValRight = curValRight;
			 freqRight = frequency;
			 }

			 if (curValLeft > peakValLeft) {
			 peakValLeft = curValLeft;
			 freqLeft = frequency;
			 }

			 printf("Magnitude Right: %f, Frequency: %u \n", peakValRight, freqRight);
			 printf("Magnitude Left: %f, Frequency: %u \n", peakValLeft, freqLeft);

			 printf("\n");
			 }
			 *************************************************************
			 */

			fftFlag = 1;
			fftIndex = 0;
		}
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	fftHalfFlag = 1;
	fftFullFlag = 0;
	processHalfFlag = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	processHalfFlag = 1;
	fftHalfFlag = 0;
	fftFullFlag = 1;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DAC_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_FATFS_Init();
	MX_SPI2_Init();
	MX_USART2_UART_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) Wave_LUT, 128,
	DAC_ALIGN_12B_R);

	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc2, accelRight, VIBRO_BUFFER_SIZE);
	HAL_ADC_Start_DMA(&hadc3, accelLeft, VIBRO_BUFFER_SIZE);

	arm_rfft_fast_init_f32(&R, FFT_BUFFER_SIZE);
	arm_rfft_fast_init_f32(&L, FFT_BUFFER_SIZE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	//process_SD_card();
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/*
		 * When the DMA callback function is triggered, we start processing the first half of the
		 * ADC buffer, while other half continues being filled with the data.
		*/
		if (processHalfFlag) {
			Process_HalfBuffer();
			processHalfFlag = 0;
		}

	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
	RCC_OscInitStruct.PLL.PLLN = 80;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ENABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ENABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = ENABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void) {

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */
	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 80 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 39;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 80 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SD_CS_Pin */
	GPIO_InitStruct.Pin = SD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

//This part of the code is taken from
//https://controllerstech.com/sd-card-using-spi-in-stm32
//and adapted to the purposes of this project
void process_SD_card(void) {
 FATFS FatFs;                //Fatfs handle
 FIL fil;                  //File handle
 FRESULT fres;                 //Result after operations
 char buf[100];
 UINT bytesWritten;

 do {
 //Mount the SD Card
 fres = f_mount(&FatFs, "", 1);    //1=mount now
 if (fres != FR_OK) {
 printf("No SD Card found : (%i)\r\n", fres);
 break;
 }
 printf("SD Card Mounted Successfully!!!\r\n");

 //Read the SD Card Total size and Free Size
 FATFS *pfs;
 DWORD fre_clust;
 uint32_t totalSpace, freeSpace;

 f_getfree("", &fre_clust, &pfs);
 totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
 freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

 printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace,
 freeSpace);

 //Open the file
 fres = f_open(&fil, "adc.csv", FA_WRITE | FA_CREATE_ALWAYS);
 if (fres != FR_OK) {
 printf("File creation/open Error : (%i)\r\n", fres);
 break;
 } else {
 float timeStep = 1.0f / SAMPLE_RATE;

 sprintf(buf, "Time (s),ADC Value\r\n");
 fres = f_write(&fil, buf, strlen(buf), &bytesWritten);

 if (fres != FR_OK) {
 printf("Error writing header to the file : (%i)\r\n", fres);
 break;
 }

 for (int i = 0; i < VIBRO_BUFFER_SIZE / 4; i++) {
 float time = i * timeStep;

 float voltage = ((accelRight[i] * 5.0f) / 4095.0f - 1.8)/0.330; //0g bias level at Zout - see datasheet
 sprintf(buf, "%.6f,%.6f\r\n", time, voltage);
 fres = f_write(&fil, buf, strlen(buf), &bytesWritten);
 if (fres != FR_OK) {
 printf("Error writing data to the file : (%i)\r\n", fres);
 break;
 }
 }
 }
 //write the data

 //close your file
 f_close(&fil);
 printf("Writing data to file completed!!!\r\n");

 #if 0
 //Delete the file.
 fres = f_unlink(EmbeTronicX.txt);
 if (fres != FR_OK)
 {
 printf("Cannot able to delete the file\n");
 }
 #endif
 } while ( false);

 //We're done, so de-mount the drive
 f_mount(NULL, "", 1);
 printf("SD Card Unmounted Successfully!!!\r\n");
 }
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
