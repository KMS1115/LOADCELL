/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ADS125X_Init(ADS125X_t *ads, SPI_HandleTypeDef *hspi, uint8_t drate, uint8_t gain, uint8_t buffer_en)
{
  ads->hspix = hspi;
  ads->pga = 1 << gain;

	ADS125X_CS(ads, 1);
	ADS125X_CMD_Send(ads, ADS125X_CMD_RESET);
	HAL_Delay(5);
	ADS125X_CMD_Send(ads, ADS125X_CMD_SDATAC);

	uint8_t tmp[5]; // buffer
	#ifdef DEBUG_ADS1255
		ADS125X_Register_Read(ads, ADS125X_REG_STATUS, tmp, 1);
		printf("STATUS: %#.2x\n", tmp[0]);
	#endif

		// enable clockout | ADS125X_PGA1
		ADS125X_Register_Write(ads, ADS125X_REG_ADCON, ADS125X_CLKOUT_1 | gain);  // enable clockout = clkin/1
	#ifdef DEBUG_ADS1255
		ADS125X_Register_Read(ads, ADS125X_REG_ADCON, tmp, 1);
		printf("ADCON: %#.2x\n", tmp[0]);
	#endif

	  ADS125X_Register_Write(ads, ADS125X_REG_DRATE, drate);
	#ifdef DEBUG_ADS1255
		ADS125X_Register_Read(ads, ADS125X_REG_DRATE, tmp, 1);
		printf("DRATE: %#.2x\n", tmp[0]);
	#endif

	ADS125X_Register_Write(ads, ADS125X_REG_IO, 0x00); // all GPIOs are outputs (do not leave floating) - D0 is CLKOUT
	#ifdef DEBUG_ADS1255
		ADS125X_Register_Read(ads, ADS125X_REG_IO, tmp, 1);
		printf("IO   : %#.2x\n", tmp[0]);
	#endif

		ADS125X_CMD_Send(ads, ADS125X_CMD_SELFCAL);
		ADS125X_CS(ads, 0);
	  ADS125X_DRDY_Wait(ads);  // wait ADS1256 to settle after self calibration

		return 0;
	}

uint8_t ADS125X_DRDY_Wait(ADS125X_t *ads){ // 데이터 준비 완료 신호(DRDY 핀)를 대기 -> GPIO 핀이 LOW로 내려갈 때까지 대기
  while(HAL_GPIO_ReadPin(ads->drdyPort, ads->drdyPin) == GPIO_PIN_SET);
	   // ADC가 데이터 변환을 완료하기 전까지 추가 명령이나 데이터를 요청하지 않도록 보장

	return 0;
}

uint8_t ADS125X_CS(ADS125X_t *ads, uint8_t on) // 칩 선택 핀을 제어 -> GPIO 핀을 설정(LOW) or 해제(HIGH)
{
  if(on) on = 0;
  else on = 1;
  HAL_GPIO_WritePin( ads->csPort, ads->csPin, on);
	return 0;
}

void ADS125X_ADC_Code2Volt (ADS125X_t *ads, int32_t *pCode, float *pVolt, uint16_t size){ // 2의 보수 형식 ADC 데이터를 전압 값으로 변환 -> PGA 이득 및 참조 전압을 고려
	for(uint8_t i=0; i<size; i++){
		// Vin = Code * 2 * (Vrefp - Vrefn) / ( PGA * 0x7FFFFF )
		if(pCode[i] & 0x800000) pCode[i] |= 0xff000000;  // fix 2's complement
		// do all calculations in float. don't change the order of factors --> (adsCode/0x7fffff) will always return 0
		pVolt[i] = ( (float)pCode[i] * (2.0f * ads->vref) ) / ( ads->pga * 8388607.0f );  // 0x7fffff = 8388607.0f
	}
}

float ADS125X_ADC_ReadVolt (ADS125X_t *ads){ // 현재 입력 채널의 아날로그 값을 전압 값으로 변환하여 반환 -> RDATA 명령을 전송하여 24비트 ADC 데이터를 읽음, 2의 보수 형식 데이터를 부호 있는 32비트 정수로 전환
	uint8_t spiRx[3] = {0,0,0};
	spiRx[0] = ADS125X_CMD_RDATA;

	ADS125X_CS(ads, 1);
	HAL_SPI_Transmit(ads->hspix, spiRx, 1, 10);
	HAL_Delay(1);
	HAL_SPI_Receive(ads->hspix, spiRx, 3, 10);
	ADS125X_CS(ads, 0);

#ifdef DEBUG_ADS1255
	printf("RDATA: %#.2x%.2x%.2x\n", spiRx[0], spiRx[1], spiRx[2]);
#endif

	// must be signed integer for 2's complement to work
	int32_t adsCode = (spiRx[0] << 16) | (spiRx[1] << 8) | (spiRx[2]);
  if(adsCode & 0x800000) adsCode |= 0xff000000;  // fix 2's complement
	// do all calculations in float. don't change the order of factors --> (adsCode/0x7fffff) will always return 0
	return ( (float)adsCode * (2.0f * ads->vref) ) / ( ads->pga * 8388607.0f );  // 0x7fffff = 8388607.0f
}

uint8_t ADS125X_Register_Read(ADS125X_t *ads, uint8_t reg, uint8_t* pData, uint8_t n) // 내부 레지스터 값을 읽음 -> RREG 명령 전송, 명령 수행 후 지연을 추가하여 안정성 확보
{
  uint8_t spiTx[2];
	spiTx[0] = ADS125X_CMD_RREG | reg; // 1st command byte
	spiTx[1] = n-1;                    // 2nd command byte = bytes to be read -1

  ADS125X_CS(ads, 1);
  ADS125X_DRDY_Wait(ads);
  HAL_SPI_Transmit(ads->hspix, spiTx, 2, 1);
  HAL_Delay(1); // t6 delay (50*tCLKIN)
  HAL_SPI_Receive(ads->hspix, pData, n, 1);
  HAL_Delay(1); // t11 delay
  ADS125X_CS(ads, 0);
	return 0;
}

uint8_t ADS125X_Register_Write(ADS125X_t *ads, uint8_t reg, uint8_t data) // 내부 레지스터 값을 설정 -> WREG 명령과 데이터를 SPI로 전송, 설정값을 레지스터에 기록
{
	uint8_t spiTx[3];
	spiTx[0] = ADS125X_CMD_WREG | reg; // 1st command byte
	spiTx[1] = 0;                      // 2nd command byte = payload length = 1 bytes -1 = 0
	spiTx[2] = data;

  ADS125X_CS(ads, 1);
	ADS125X_DRDY_Wait(ads);
	HAL_SPI_Transmit(ads->hspix, spiTx, 3, 10);
	HAL_Delay(1);
  ADS125X_CS(ads, 0);
	return 0;
}

void ADS125X_Channel_Set(ADS125X_t *ads, int8_t channel) // 싱글 엔드 채널 설정
{
  ADS125X_ChannelDiff_Set(ads, channel, ADS125X_MUXN_AINCOM);
}

/**
  * @brief  set internal multiplexer to differential input channel
  * @param  *ads pointer to ads handle
  * @param  p_chan positive analog input
  * @param  n_chan negative analog input
  * @see    Datasheet p. 31 MUX : Input Multiplexer Control Register (Address 01h)
  */
uint8_t ADS125X_ChannelDiff_Set(ADS125X_t *ads, int8_t p_chan, int8_t n_chan) // 차동 입력 채널 설정 -> MUX 레지스터를 설정하여 채널 선택, SYNC 및 WAKEUP 명령 실행
{
  // uint8_t channels = ((p_chan << 4)&0xF0) | (n_chan & 0x0F);

  ADS125X_Register_Write(ads, ADS125X_REG_MUX, p_chan | n_chan);
  ADS125X_CMD_Send(ads, ADS125X_CMD_SYNC);
  ADS125X_CMD_Send(ads, ADS125X_CMD_WAKEUP);
#ifdef DEBUG_ADS1255
	uint8_t tmp = 0;
	ADS125X_Register_Read(ads, ADS125X_REG_MUX, &tmp, 1);
#endif
  // ADS125X_CMD_Send(ads, ADS125X_CMD_SYNC);
  // ADS125X_CMD_Send(ads, ADS125X_CMD_WAKEUP);
  return 0;
}
uint8_t ADS125X_CMD_Send(ADS125X_t *ads, uint8_t cmd) // 단일 명령을 ADC로 전송 -> SDATAC, RESET, SYNC와 같은 명령 전송
{
  uint8_t spiTx = cmd;

  ADS125X_CS(ads, 1);
  ADS125X_DRDY_Wait(ads);
  HAL_SPI_Transmit(ads->hspix, &spiTx, 1, 1);
  ADS125X_CS(ads, 0);
	return 0;
}

int _write(int file, char *data, int len){
	HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
	return len;
}

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
	float s1, s2, s3, s4;
		float w1, w2, w3, w4;
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
  ADS125X_t adc;
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  adc.csPort = CS_GPIO_Port;
    	  adc.csPin = CS_Pin;
    	  adc.drdyPort = DRDY_GPIO_Port;
    	  adc.drdyPin = DRDY_Pin;
    	  adc.vref = 2.5f;
    	  ADS125X_Init(&adc, &hspi1, ADS125X_DRATE_500SPS, ADS125X_PGA8, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t timeStamp = HAL_GetTick();
	  ADS125X_ChannelDiff_Set(&adc, ADS125X_MUXP_AIN0, ADS125X_MUXN_AIN1);
	  HAL_Delay(10); // 안정화 대기
	  s1 = ADS125X_ADC_ReadVolt(&adc);
	  w1 = -5665.2 * s1 + 0.2402;

	  ADS125X_ChannelDiff_Set(&adc, ADS125X_MUXP_AIN2, ADS125X_MUXN_AIN3);
	  HAL_Delay(10); // 안정화 대기
	  s2 = ADS125X_ADC_ReadVolt(&adc);
	  w2 = -5639.2 * s2 + 0.2324;

	  ADS125X_ChannelDiff_Set(&adc, ADS125X_MUXP_AIN4, ADS125X_MUXN_AIN5);
	  HAL_Delay(10); // 안정화 대기
	  s3 = ADS125X_ADC_ReadVolt(&adc);
	  w3 = -5532.6 * s3 + 0.5163;

	  ADS125X_ChannelDiff_Set(&adc, ADS125X_MUXP_AIN6, ADS125X_MUXN_AIN7);
	  HAL_Delay(10); // 안정화 대기
	  s4 = ADS125X_ADC_ReadVolt(&adc);
	  w4 = -5614.7 * s4 + 0.4523;

	  printf("%lu,%.2f,%.2f,%.2f,%.2f*\r\n", timeStamp, w1, w2, w3, w4);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
