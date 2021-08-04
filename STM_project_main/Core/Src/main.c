/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f446xx.h"
#include "NRF24L01.h"
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern volatile uint32_t NRF24_IRQ;

uint8_t uart_receve[64] = {0};
uint8_t uart_transmit[64] = {0};
uint8_t uart_flag = 0;

uint8_t timer_flag = 0;
uint32_t timer_counter = 0;
uint32_t seconds = 0;
uint32_t average = 0;
float humi_sum = 0;
float temp_sum = 0;
float adc_sum = 0;
float humi = 0;
float temp = 0;
float adc = 0;
float temperature = 0;
float humidity = 0;

uint8_t com_flag = 0;

uint8_t step_flag = 0;
uint8_t nrf_receive_flag = 0;

uint8_t buf_nrf_transmit[27] = {0};
uint8_t buf_nrf_receive[27] = {0};

RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void read_temp_humi();
static float read_adc();
static void rtc_set(
        uint8_t year, uint8_t month, uint8_t day, uint8_t week_day,
        uint8_t hour, uint8_t min, uint8_t sec
        );

void init_timer(uint32_t sec, uint32_t aver);
static void step_timer();

static void prepare_transmission_data();
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  NVIC_EnableIRQ (USART2_IRQn);
  NRF24_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  init_timer(3, 10);
  rtc_set(20, 12, 1, 1, 12, 0, 0);
  NRF24_Receive_base();

  while (1)
  {

	  if (uart_flag)
	  {
		  char array[10] = {0};
		  uint8_t temp_buf[64];
		  uint32_t array_index = 0;
		  uint32_t num_array_index = 0;
		  uart_flag = 0;

		  memcpy(temp_buf, uart_receve, 64);

		  if (temp_buf[0] == 'u')
		  {
			  com_flag = 1;
		  }
		  else if (temp_buf[0] == 'n')
		  {
			  com_flag = 0;
		  }
		  else if (temp_buf[0] == 'a')
		  {
			  buf_nrf_transmit[0] = temp_buf[0];

			  for (uint32_t i = 1; temp_buf[i] != '\0'; i++)
			  {
				  if (temp_buf[i] != ':')
				  {
					  array[array_index] = temp_buf[i];
					  array_index++;
				  }
				  else
				  {
					  array[array_index] = '\0';
					  *((uint32_t*)(buf_nrf_transmit + 1)) = atoi(array);
					  array_index = 0;
				  }
			  }

			  array[array_index] = '\0';
			  *((uint32_t*)(buf_nrf_transmit + 5)) = atoi(array);

			  init_timer(*((uint32_t*)(buf_nrf_transmit + 1)), *((uint32_t*)(buf_nrf_transmit + 5)));
		  }
		  else
		  {
			  for (uint32_t i = 0; temp_buf[i] != '\0'; i++)
			  {
				  if (temp_buf[i] != ':')
				  {
					  array[array_index] = temp_buf[i];
					  array_index++;
				  }
				  else
				  {
					  array[array_index] = '\0';
					  buf_nrf_transmit[num_array_index] = atoi(array);
					  num_array_index++;
					  array_index = 0;
				  }
			  }

			  array[array_index] = '\0';
			  buf_nrf_transmit[num_array_index] = atoi(array);

			  rtc_set(buf_nrf_transmit[0], buf_nrf_transmit[1], buf_nrf_transmit[2], buf_nrf_transmit[3],
					  buf_nrf_transmit[4], buf_nrf_transmit[5], buf_nrf_transmit[6]);
		  }

	  }


	  if (timer_flag)
	  {
		  timer_flag = 0;
		  step_timer();
	  }

	  if (step_flag)
	  {
		  step_flag = 0;

		  prepare_transmission_data();

		  if (com_flag)
		  {
			  sprintf((char*)uart_transmit, ";%f:%f:%f:%d:%d:%d:%d:%d:%d:%d:%lu:%lu;", *((float*)buf_nrf_transmit), *((float*)(buf_nrf_transmit + 4)),
			  				  *((float*)(buf_nrf_transmit + 8)), buf_nrf_transmit[12], buf_nrf_transmit[13], buf_nrf_transmit[14],
							  buf_nrf_transmit[15], buf_nrf_transmit[16], buf_nrf_transmit[17], buf_nrf_transmit[18],
							  *((uint32_t*)(buf_nrf_transmit + 19)), *((uint32_t*)(buf_nrf_transmit + 23)));
			  		  HAL_UART_Transmit(&huart2, uart_transmit, strlen((char*)uart_transmit), 100);
		  }
		  else
		  {
			  NRF24_Transmit_base(buf_nrf_transmit, 27);
		  }
	  }

	  if (NRF24_IRQ)
	  {
		  uint8_t r_rx_payload = (uint8_t)R_RX_PAYLOAD;
		  uint8_t temp_status = NRF24_ReadReg((uint8_t)STATUS);

		  NRF24_IRQ = 0;

		  if (temp_status & (1UL << RX_DR))
		  {
			  NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS) | 0x70); // clear interupt
			  NRF24_Read_Buf(r_rx_payload, buf_nrf_receive, 27);
			  NRF24_Receive_base();

			  nrf_receive_flag = 1;
		  }
		  else if (temp_status & (1UL << TX_DS))
		  {
			  NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS) | 0x70); // clear interupt
			  NRF24_Receive_base();
		  }
		  else if (temp_status & (1UL << MAX_RT))
		  {
			  NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS) | 0x70); // clear interupt
			  NRF24_Receive_base();
		  }
	  }


	  if (nrf_receive_flag)
	  {
		  nrf_receive_flag = 0;

		  if (buf_nrf_receive[0] == 'a')
		  {
			  init_timer(*((uint32_t*)(buf_nrf_receive + 1)), *((uint32_t*)(buf_nrf_receive + 5)));
		  }
		  else if (buf_nrf_receive[0] == 'u')
		  {
			  com_flag = 1;
		  }
		  else if (buf_nrf_receive[0] == 'n')
		  {
			  com_flag = 0;
		  }
		  else
		  {
			  rtc_set(buf_nrf_receive[0], buf_nrf_receive[1], buf_nrf_receive[2], buf_nrf_receive[3],
					  buf_nrf_receive[4], buf_nrf_receive[5], buf_nrf_receive[6]);
		  }
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
	__HAL_RCC_RTC_ENABLE();
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  __HAL_RCC_RTC_ENABLE();
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 45;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void read_temp_humi()
{
	uint8_t buf[4] = {0};

	buf[0] = 0;
    HAL_I2C_Master_Transmit(&hi2c1, 0x50, buf, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, 0x50, buf, 4, HAL_MAX_DELAY);

    humidity = (float)(((buf[0] & 0x3F ) << 8) + buf[1]) / 16384.0 * 100.0;
    temperature = (float)((unsigned)(buf[2]  * 64) + (unsigned)(buf[3] >> 2 )) / 16384.0 * 165.0 - 40.0;
}


float read_adc()
{
	static uint8_t i = 0;
	uint32_t temp = 0;
	float ret = 0;
	i++;

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i*15);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    temp = HAL_ADC_GetValue(&hadc1);
    HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);

    ret = (3.3f / 4095) * temp;

    return ret;
}



void rtc_set(
        uint8_t year, uint8_t month, uint8_t day, uint8_t week_day,
        uint8_t hour, uint8_t min, uint8_t sec
        )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    memset(&time, 0, sizeof(time));
    memset(&date, 0, sizeof(date));

    date.WeekDay = week_day;
    date.Year = year;
    date.Month = month;
    date.Date = day;

    HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);

    time.Hours = hour;
    time.Minutes = min;
    time.Seconds = sec;

    HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
}


void init_timer(uint32_t sec, uint32_t aver)
{
	uint32_t micro_sec = 0;
	average = aver;
	seconds = sec;

	HAL_TIM_Base_Stop_IT(&htim2);
	timer_flag = 0;
	timer_counter = 0;
	humi_sum = 0;
	temp_sum = 0;
	adc_sum = 0;

	micro_sec = (1000000 * seconds) / average;
	TIM2->CNT = micro_sec;
	TIM2->ARR = micro_sec;

	HAL_TIM_Base_Start_IT(&htim2);
}



void step_timer()
{
	read_temp_humi();
	humi_sum += humidity;
	temp_sum += temperature;
	adc_sum += read_adc();

	timer_counter++;

	if (timer_counter == average)
	{
		humi = humi_sum / average;
		temp = temp_sum / average;
		adc = adc_sum / average;

		humi_sum = 0;
		temp_sum = 0;
		adc_sum = 0;
		timer_counter = 0;
		step_flag = 1;
	}
}


void prepare_transmission_data()
{
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	*((float*)buf_nrf_transmit) = humi;
	*((float*)(buf_nrf_transmit + 4)) = temp;
	*((float*)(buf_nrf_transmit + 8)) = adc;
	buf_nrf_transmit[12] = rtc_date.Year;
	buf_nrf_transmit[13] = rtc_date.Month;
	buf_nrf_transmit[14] = rtc_date.Date;
	buf_nrf_transmit[15] = rtc_date.WeekDay;
	buf_nrf_transmit[16] = rtc_time.Hours;
	buf_nrf_transmit[17] = rtc_time.Minutes;
	buf_nrf_transmit[18] = rtc_time.Seconds;
	*((uint32_t*)(buf_nrf_transmit + 19)) = seconds;
	*((uint32_t*)(buf_nrf_transmit + 23)) = average;

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
	while(1)
	{
		HAL_Delay(1000);
	}
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
