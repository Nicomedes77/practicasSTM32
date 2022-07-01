/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>
#include "math.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define termistorNominalRes 100000
#define termistorNominalTemp 25
#define termistorBValue 3950
#define voltageDividerResistor 100000
#define N_SAMPLES_ADC 16
#define	CLEAR_LCD	0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//variables and/temp
uint32_t adc_val = 50;
uint32_t bufferDMA;
uint32_t tempConv;
//uint32_t n_sample_adc = 0;
uint32_t i = 0;
float_t Vsupply = 3.3;
float_t Vout;
float_t R_NTC;
float_t B_param = 3950;
float_t T0 = 298.15;
float_t Temp_K;
float_t Temp_C;
uint32_t tempCint;

//variables control PID
uint32_t PWM_pin;	//esta variable seria el pin por donde sale el PWM controlado
float_t temperature_read; //esta variable vendria a ser la que guarda el resultado de getTemp()
uint32_t set_temperature = 70;
float_t PID_error = 0;
float_t previous_error = 0;
float_t elapsedTime, Time, timePrev;
uint32_t PID_value = 0;

//PID constants
#define kp  9.1
#define ki  0.3
#define kd  1.8
float_t PID_p = 0;
float_t PID_i = 0;
float_t PID_d = 0;

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  Time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  controlPID();
//	  HAL_Delay(300);
//	  updateLCD();

//	  lcd_send_cmd(0x80);	//primer linea
//	  lcd_send_string ("CH0- ");  // send string
//	  lcd_send_cmd(0x80|0x40);	//segunda linea
//	  sprintf(val,"%lu C / %lu C",bufferDMA,bufferDMA);
//	  lcd_send_string(val);
//	  HAL_Delay (200); // 200ms delay
/*
	  lcd_send_data ((bufferDMA/1000)+48);  // print the 1000th value of value[0]
	  lcd_send_data (((bufferDMA%1000)/100)+48);  // print the 100th value of value[0]
	  lcd_send_data (((bufferDMA%100)/10)+48);  // print the 10th value of value[0]
	  lcd_send_data (((bufferDMA%10))+48);  // print the 1st value of value[0]
*/

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//
////	  adc_val = bufferDMA;
////	  n_sample_adc++;
//}


float_t getTemp(void)
{

	uint32_t adc_val_average = 0;

	for(uint8_t i = 0 ; i < N_SAMPLES_ADC ; i++)
	{
		HAL_ADC_Start_DMA(&hadc1, &adc_val, 1);	//star in DMA mode and we are reading only 1 channel or 1 word
		adc_val_average += adc_val;
	}
	adc_val_average = (uint32_t)(adc_val_average/N_SAMPLES_ADC);

/*
		Vout = adc_val_average * (3.3/4095);
		R_NTC = (Vout * 100000) / (Vsupply - Vout);
		Temp_K = (T0 * B_param) / (T0 * log(R_NTC / 100000) + B_param);
		Temp_C = Temp_K - 273.15;
		return	Temp_C;
*/


		while(tempNTC100K[i].adc < adc_val_average) i++;

		if(i == NUMTEMPS)	Temp_C = tempNTC100K[i-1].temp;
		else if(i == 0) Temp_C = tempNTC100K[i].temp;
		else
		{
			Temp_C = tempNTC100K[i-1].temp + (adc_val_average - tempNTC100K[i-1].adc) * (float)(tempNTC100K[i].temp - tempNTC100K[i-1].temp) / (float)(tempNTC100K[i].adc - tempNTC100K[i-1].adc);

			//aux = (tempNTC100K[i].temp - tempNTC100K[i-1].temp) / ( tempNTC100K[i].adc - tempNTC100K[i-1].adc);
			//tempC = aux * (adc_val - tempNTC100K[i].adc) + tempNTC100K[i].temp;
		}

		//tempCint = (uint32_t)Temp_C;
		return	Temp_C;


}

void controlPID(void)
{
	temperature_read = getTemp();  // First we read the real value of temperature
	tempConv = (uint32_t)temperature_read;
//	updateLCD();

	PID_error = set_temperature - temperature_read; //Next we calculate the error between the setpoint and the real value
	PID_p = kp * PID_error;   //Calculate the P value
	if((-3 < PID_error ) && ( PID_error < 3 ))	PID_i = PID_i + (ki * PID_error); //Calculate the I value in a range on +-3

	//For derivative we need real time to calculate speed change rate
	timePrev = Time; // the previous time is stored before the actual time read
	Time = HAL_GetTick();	// actual time read
	elapsedTime = (Time - timePrev) / 1000;
	PID_d = kd * ((PID_error - previous_error)/elapsedTime); //Now we can calculate the D value
	PID_value =(uint32_t)(PID_p + PID_i + PID_d); //Final total PID value is the sum of P + I + D

	//We define PWM range between 0 and 255
	if(PID_value < 0) PID_value = 0;
	else if(PID_value > 255)  PID_value = 255;
	updateLCD();
	//Now we can write the PWM signal to the mosfet on digital pin D3
	htim1.Instance->CCR1 = PID_value;
	//ESCRIBE EL VALOR DEL PWM

	previous_error = PID_error;     //Remember to store the previous error for next loop.

}

void updateLCD(void)
{
	char val1[20];
	char val2[20];
	sprintf(val1,"%lu C / %lu C",tempConv,set_temperature);
	sprintf(val2,"%lu",PID_value);
	lcd_send_cmd(0x80|0x00);	//primer linea
	lcd_send_string("T actual / T target");
  	lcd_send_cmd(0x80|0x40);	//segunda linea
  	lcd_send_string(val1);
	lcd_send_cmd(0x80|0x14);	//tercera linea
	lcd_send_string(val2);
	  /*
	  	  lcd_send_cmd(0x80|0x00);	//primer linea
	  	  lcd_send_string("PlasticBot");
	  	  lcd_send_cmd(0x80|0x40);	//segunda linea
	  	  lcd_send_string("v0.1");
	  	  lcd_send_cmd(0x80|0x14);	//tercera linea
	  	  lcd_send_string("TF CESE");
	  	  lcd_send_cmd(0x80|0x54);	//cuarta linea
	  	  lcd_send_string("2022");
	  */
  	HAL_Delay(1000);
  	lcd_send_cmd (CLEAR_LCD);  // clear display
  	HAL_Delay(1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
