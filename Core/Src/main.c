/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "myheader.h"

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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
//HTS221



uint8_t HTTS221_CTR_REG_1_settings = (0b10000110);
uint8_t HTS221_WHOAMI_buf = 0;
uint8_t HTTS221_CTR_CALIBRATION_REG[6];
uint8_t HTS221_Data_buff[4];

//data to interpolation TEMP
uint8_t HTTS221_T0_degC = 0;
uint8_t HTTS221_T1_degC = 0;
uint8_t HTTS221_T_MSB = 0;
int16_t HTTS221_T0_OUT = 0;
int16_t HTTS221_T1_OUT = 0;
int16_t HTTS221_T_OUT = 0;

//data to interpolation HUMI
uint8_t HTTS221_H0_rH_x2 = 0;
uint8_t HTTS221_H1_rH_x2 = 0;
int16_t HTTS221_H0_T0_OUT = 0;
int16_t HTTS221_H1_T0_OUT = 0;


//LSM6DSL
uint8_t LSM6DSL_CTRL1_XL=(0b01001110);
uint8_t LSM6DSL_GYRRO_CTRL2_G_sett=(0b01001000);
uint8_t LSM6DSL_WHOAMI_buf = 0;



//LSM303AGR

uint8_t LSM303AGR_ACCE_WHOAMI_buff = 0;
uint8_t LSM303AGR_GYRO_WHOAMI_buff = 0;

uint8_t LSM303AGR_CTRL_REG1_A_settings = 0x57;
uint8_t LSM303AGR_CTRL_REG2_A_settings = 0x00;
uint8_t LSM303AGR_CTRL_REG3_A_settings = 0x00;
uint8_t LSM303AGR_CTRL_REG4_A_settings = 0b10100000;
uint8_t LSM303AGR_TEMP_CFG_REG_A_settings = 0xC0;

uint8_t LSM303AGR_DATA_ACCE_buff[6];

//LPS22HB

uint8_t LPS22HB_WHOAMI_buf = 0;
uint8_t LPS22HB_CTRL_REG1_A_settings = 0b00111000;

uint8_t UART_BUFFOR[100];
uint8_t UART_RECEIVE_BUFFOR = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  //Read WHOAMI for HTS221 - temperature, humidity
      HAL_I2C_Mem_Read(&hi2c1, HTS221_ADR, HTS221_ADR_WHO_AM_I,1, &HTS221_WHOAMI_buf, 1, 5);
    //Read WHOAMI for LSM6DSL -  accelerometer,  gyroscope
      HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_ADR, LSM6DSL_ADR_WHOAMI, 1, &LSM6DSL_WHOAMI_buf, 1, 5);
  	  //Read WHOAMI for LSM303AGR - accelerometer
  		HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_ADR_WHOAMI_ACCE, 1, &LSM303AGR_ACCE_WHOAMI_buff, 1, 5);
  	  //Read WHOAMI for LSM303AGR - gyroscope
  		HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ADR_GYRO, LSM303AGR_ADR_WHOAMI_GYRO, 1, &LSM303AGR_GYRO_WHOAMI_buff, 1, 5);
    //Read WHOAMI for LPS22HB MEMS pressure sensor, 260-1260 hPa absolute digital output barometer
      HAL_I2C_Mem_Read(&hi2c1, LPS22HB_ADR, LPS22HB_ADR_WHOAMI, 1, &LPS22HB_WHOAMI_buf, 1, 50);

    //LSM6DSL SET REGISTER
      HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADR, LSM6DSL_ADR_CTRL1_XL, 1, &LSM6DSL_CTRL1_XL, 1, 5);
      HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADR, LSM6DSL_GYRRO_CTRL2_G, 1, &LSM6DSL_GYRRO_CTRL2_G_sett, 1, 5);
    //LSM303AGR SET REGISTER
      HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_TEMP_CFG_REG_A, 1, &LSM303AGR_TEMP_CFG_REG_A_settings, 1, 5); //ON TEMPERATURE
      HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_CTRL_REG2_A, 1, &LSM303AGR_CTRL_REG2_A_settings, 1, 5);
      HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_CTRL_REG3_A, 1, &LSM303AGR_CTRL_REG3_A_settings, 1, 5);
      HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_CTRL_REG4_A, 1, &LSM303AGR_CTRL_REG4_A_settings, 1, 5);
      HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_CTRL_REG1_A, 1, &LSM303AGR_CTRL_REG1_A_settings, 1, 5);

      //LPS22HB SET REGISTER
      HAL_I2C_Mem_Write(&hi2c1, LPS22HB_ADR, LPS22HB_LPS22HB_CTRL_REG1, 1, &LPS22HB_CTRL_REG1_A_settings, 1, 5);

     //HTS221 SET REGISTER

      HAL_I2C_Mem_Write(&hi2c1, HTS221_ADR, HTS221_CTR_REG_1, 1,  &HTTS221_CTR_REG_1_settings, 1, 5);
     //HTS221 READ CONTROL DATA Interpolation TEMP
		  HAL_I2C_Mem_Read(&hi2c1, HTS221_ADR, HTS221_T0_degC_x8, 1, HTS221_Data_buff, 2, 5);
		  HTTS221_T0_degC = HTS221_Data_buff[0];
		  HTTS221_T1_degC = HTS221_Data_buff[1];
		  HAL_I2C_Mem_Read(&hi2c1, HTS221_ADR, HTS221_T1T0_MSB, 1, HTS221_Data_buff, 1, 5);
		  HTTS221_T_MSB = HTS221_Data_buff[0];
		  HAL_I2C_Mem_Read(&hi2c1, HTS221_ADR, HTS221_TEMP_T0_OUT, 1,  HTS221_Data_buff, 4, 5);
		  HTTS221_T0_OUT = (HTS221_Data_buff[1] << 8) + HTS221_Data_buff[0];
		  HTTS221_T1_OUT = (HTS221_Data_buff[3] << 8) + HTS221_Data_buff[2];
		  HTTS221_T0_degC = HTTS221_T0_degC / 8;
		  HTTS221_T1_degC = HTTS221_T1_degC / 8;

	 //HTS221 READ CONTROL DATA Interpolation HUMI
		  HAL_I2C_Mem_Read(&hi2c1, HTS221_ADR, HTS221_H0_rH_x2, 1, HTS221_Data_buff, 2, 5);
		  HTTS221_H0_rH_x2 =  HTS221_Data_buff[0] / 2;
		  HTTS221_H1_rH_x2 =  HTS221_Data_buff[1] / 2;

		  HAL_I2C_Mem_Read(&hi2c1, HTS221_ADR, HTS221_H0_T0_OUT, 1, HTS221_Data_buff, 2, 5);
		  HTTS221_H0_T0_OUT = (HTS221_Data_buff[1] << 8) + HTS221_Data_buff[0];
		  HAL_I2C_Mem_Read(&hi2c1, HTS221_ADR, HTS221_H1_T0_OUT, 1, HTS221_Data_buff, 2, 5);
		  HTTS221_H1_T0_OUT = (HTS221_Data_buff[1] << 8) + HTS221_Data_buff[0];

//      HTTS221_H0_rH_x2_sett = HTTS221_CTR_CALIBRATION_REG[0];
//      HTTS221_H1_rH_x2_sett = HTTS221_CTR_CALIBRATION_REG[1];
//      HTTS221_H0_T0_OUT_sett = (HTTS221_CTR_CALIBRATION_REG[3] << 8) + HTTS221_CTR_CALIBRATION_REG[2];
//      HTTS221_H1_T0_OUT_sett = (HTTS221_CTR_CALIBRATION_REG[5] << 8) + HTTS221_CTR_CALIBRATION_REG[4];

      //First Interrupt
      HAL_I2C_Mem_Read_IT(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_OUT_X_L_A, 1, LSM303AGR_DATA_ACCE_buff, 6);

      //START UART_DMA
      HAL_UART_Transmit_DMA(&huart2, (uint8_t*) UART_BUFFOR, 70);
      HAL_UART_Receive_DMA(&huart2, (uint8_t*) &UART_RECEIVE_BUFFOR, 1);
      //uart_send_string_DMA( (char*) UART_BUFFOR, 50); //test
      //uart_send_string_DMA((char*) "HELLO", 50);   //test


  while (1)
  {

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE END RTC_Init 0 */

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
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

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
