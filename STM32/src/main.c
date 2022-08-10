/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * Copyright (c) 2022 Makermax Systems Inc..
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

#include "oled.h"
#include "adc.h"
#include "dac.h"

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
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


typedef enum {
	CHG_ENABLE = 1, CHG_DISABLE = 0
} CHG_EN;

typedef enum {
	IDLE = 0, CHG = 1, DCHG = 2
} STATE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_DAC1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */


STATE currentState = IDLE;
uint8_t currentDchgPct = 0;
uint8_t MCP342x_ADDR = 0x68;

static float lastReadBattV = 0;
static float lastReadCurr_mA = 0;
static float currentCellSOC = 0;
static float currChargeRemaining = 0; //Ah how much charge is remaining inside the cell
static const float fullChargeCapacity = 3.0 ; //Ah //TODO: Change this as per your cell
static float lastComputedPower = 0;
static float dchgCurrVal = 0.1;
static float chgCurrVal = 0.1;

void Charging_Enable(CHG_EN chg_en);
void Change_State(STATE new_state);
void Discharging_Set(uint8_t pct);
void Safety_Loop();
void calcSOC(float ocv_V, float chargeRemain_Ah);
void getLatestADCValues();
static void updateOLED();


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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_DAC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  //Initialize OLED display
  	ssd1306_Init();
  	ssd1306_Fill(Black);
  	ssd1306_WriteString("MakerMax", Font_7x10, White);
  	ssd1306_UpdateScreen();

  	//DAC code
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, 0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

    // ADC code
    mcp342x_return_code_t ret = MCP342x_RET_OK;

    // Create an instance of our mcp342x device
    mcp342x_dev_t dev;

    int8_t usr_i2c_write(const uint8_t busAddr, const uint8_t *data, const uint32_t len) {
	mcp342x_return_code_t ret = MCP342x_RET_OK;

	// Transmit the data to the specified device from the provided
	// data buffer.
	//HAL_I2C_Mem_Write(&hi2c1, 0b11010000, 0x0, I2C_MEMADD_SIZE_8BIT, data, len, 1500);

	HAL_I2C_Master_Transmit(&hi2c1, 0b11010000, data, len, 1500);

	return ret;
    }

    int8_t usr_i2c_read(const uint8_t busAddr, uint8_t *data, const uint32_t len) {
        mcp342x_return_code_t ret = MCP342x_RET_OK;

    // Received the specified amount of data from the device and store it
    // in the data buffer
    //HAL_I2C_Mem_Read(&hi2c1, 0b11010000, 0x0, I2C_MEMADD_SIZE_8BIT, data, len, 1500);

     HAL_I2C_Master_Receive(&hi2c1, 0b11010000, data, len, 1500);


        return ret;
    }

    void usr_delay_us(uint32_t period) {
        // Delay for the requested period
    	HAL_Delay(period/1000);
    }

    // Provide the hardware abstraction functions for
    // I2c Read/Write and a micro-second delay function
    dev.intf.i2c_addr = MCP342x_ADDR;
    dev.intf.write = usr_i2c_write;
    dev.intf.read = usr_i2c_read;
    dev.intf.delay_us = usr_delay_us;

    // Init our desired config
    dev.registers.bits.config.bits.conv_mode = MCP342x_CM_CONT;
    dev.registers.bits.config.bits.gain = MCP342x_GAIN_x1;
    dev.registers.bits.config.bits.sample_rate = MCP342x_SR_15SPS;

    ret = mcp342x_writeConfig(&dev);

    if( MCP342x_RET_OK != ret ) {
           return 0;
       }

	char currentString[10];
	float chgcurrent = 0.0;
	float dchgcurrent = 0.0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ret = mcp342x_sampleChannel(&dev, MCP342x_CH_1);
	  ret = mcp342x_sampleChannel(&dev, MCP342x_CH_2);
	  ret = mcp342x_sampleChannel(&dev, MCP342x_CH_3);
	  ret = mcp342x_sampleChannel(&dev, MCP342x_CH_4);

	  usr_delay_us(1000);

	if( MCP342x_RET_OK == ret ) {

		ssd1306_SetCursor(0, 0);
		sprintf(currentString, "Makermax: %d", currentState);
		ssd1306_WriteString(currentString, Font_7x10, White);
		ssd1306_UpdateScreen();

		ssd1306_SetCursor(15, 14);

		if(dev.results[0].voltage < 4.0)
		{
		chgcurrent = (float)(fabs(dev.results[0].voltage));
		chgcurrent = chgcurrent*1000.0;
		chgcurrent = chgcurrent/0.2;

		sprintf(currentString, "CHG_C: %0.1fmA", chgcurrent);
		ssd1306_WriteString(currentString, Font_7x10, White);
		}

		ssd1306_SetCursor(15, 25);

		if(dev.results[1].voltage < 4.0)
		{
		dchgcurrent = (float)(fabs(dev.results[1].voltage));
		dchgcurrent = dchgcurrent*1000.0;
		dchgcurrent = dchgcurrent/0.2;

		sprintf(currentString, "DCHG_C: %0.1fmA", dchgcurrent);
		ssd1306_WriteString(currentString, Font_7x10, White);
		}

		ssd1306_SetCursor(15, 37);
		sprintf(currentString, "BATT_V: %0.2fV", (dev.results[2].voltage*14700/4700));
		ssd1306_WriteString(currentString, Font_7x10, White);

		ssd1306_SetCursor(15, 49);
		sprintf(currentString, "TEMP: %0.2fV", dev.results[3].voltage);
		ssd1306_WriteString(currentString, Font_7x10, White);

		ssd1306_UpdateScreen();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_IN_Pin */
  GPIO_InitStruct.Pin = SW1_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_IN_Pin */
  GPIO_InitStruct.Pin = SW2_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW2_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_LED_Pin */
  GPIO_InitStruct.Pin = USR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USR_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(SW1_IN_Pin == GPIO_Pin)
	{
		if(HAL_GPIO_ReadPin(SW2_IN_GPIO_Port,SW2_IN_Pin) == GPIO_PIN_RESET)
		{
			//Change mode
			currentState++;
			if (currentState>2)
			{
				currentState=0;
			}
		}
		HAL_GPIO_TogglePin(USR_LED_GPIO_Port,USR_LED_Pin);

		if(currentState == DCHG)
		{
			chgCurrVal = 0.0;
			dchgCurrVal -= 100.0;
			if(dchgCurrVal < 0)
			{
				dchgCurrVal = 0;
			}
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dchgCurrVal/8.77);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, chgCurrVal/8.77);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

		}
		else if(currentState == CHG)
		{
			dchgCurrVal = 0.0;
			chgCurrVal -= 100.0;
			if(chgCurrVal < 0)
			{
				chgCurrVal = 0;
			}
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, chgCurrVal/8.77);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dchgCurrVal/8.77);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

		}
		else
		{
			dchgCurrVal = 0.0;
			chgCurrVal = 0.0;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, chgCurrVal);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dchgCurrVal);
			HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);

		}
	}
	else if(SW2_IN_Pin == GPIO_Pin)
	{
		if(HAL_GPIO_ReadPin(SW1_IN_GPIO_Port,SW1_IN_Pin) == GPIO_PIN_RESET)
		{
			//Change mode
			currentState++;
			if (currentState>2)
			{
				currentState=0;
			}
		}
		HAL_GPIO_TogglePin(USR_LED_GPIO_Port,USR_LED_Pin);

		chgCurrVal += 100.0;
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, chgCurrVal/8.77);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dchgCurrVal/8.77);
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

		if(currentState == DCHG)
		{
			chgCurrVal = 0.0;
			dchgCurrVal += 100.0;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, chgCurrVal/8.77);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dchgCurrVal/8.77);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		}
		else if(currentState == CHG)
		{
			dchgCurrVal = 0.0;
			chgCurrVal += 100.0;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, chgCurrVal/8.77);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dchgCurrVal/8.77);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

		}
		else
		{
			dchgCurrVal = 0.0;
			chgCurrVal = 0.0;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, chgCurrVal);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dchgCurrVal);

			HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
		}
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
