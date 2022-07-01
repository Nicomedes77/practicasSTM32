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
#include "i2c-lcd.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum Button {Ok, Left, Right, NoPressed};

typedef struct PETfilConv{
	bool colecPET_state;
	bool colecFil_state;
	bool filDetector_state;
	bool soundAlarm_state;
	bool lightAlarm_state;
	uint32_t ExtTemp;
}PETfilConv;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	CLEAR_LCD		0x01
#define LINE1_LCD 	0x80|0x00
#define LINE2_LCD	0x80|0x40
#define LINE3_LCD	0x80|0x14
#define LINE4_LCD	0x80|0x54
#define C_LINE1_LCD	0x80|0x13
#define C_LINE2_LCD	0x80|0x53
#define C_LINE3_LCD	0x80|0x27
#define C_LINE4_LCD	0x80|0x67
#define LINES		4

int reg_lines[LINES] = {
				0x80|0x00,
				0x80|0x40,
				0x80|0x14,
				0x80|0x54
			};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char *CURSOR[] = {">"};
char *CHAR_VACIO[] = {" "};

const char *ScreenStart[] = {
    "     PlasticBOT     ",
    "        v0.1        ",
	"                    ",
	"   CFP nro 8 SMATA  "

};

const char *ScreenWorking[] = {
    "  T act/T trab   ",
	"                   ",
    "   Extruyendo...   ",
    "                   "
};

const char *PrinMenu1[] = {
    "Pantalla Principal ",
    "Extrusores         ",
    "Colectores PET     ",
    "Colectores fil.    "
};

const char *PrinMenu2[] = {
    "Alarmas            ",
    "Historial          ",
    "Acerca de...       ",
	"                   "
};

const char *SubMenu1[] = {
    "Volver...          ",
    "Extrusor 1         ",
	"                   ",
	"                   "
};

const char *SubMenu1_2[] = {
    "Elegir T trabajo:  ",
	"                    ",
	"                    ",
	"                    "
};

const char *SubMenu2[] = {
    "Volver...          ",
    "Colector PET 1     ",
	"                   ",
	"                   "
};

const char *SubMenu3[] = {
    "Volver...          ",
    "Colector fil 1     ",
	"                   ",
	"                   "
};

const char *SubMenu4[] = {
    "Volver...          ",
    "Luminica           ",
    "Sonora             ",
	"                   "
};

const char *SubMenu5[] = {
    "Volver...          ",
    "Extrusor 1         ",
    "Borrar historial   ",
	"                   "
};

const char *SubMenu6[] = {
    "Proyecto final de   ",
    "Esp. en Sistemas    ",
    "Embebidos (LSE-UBA) ",
	"                    "
};

const char *SubMenu7[] = {
    "Ing.Nicolas Vargas  ",
    "Alice. Año 2022     ",
	"                    ",
	"                    "
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/*
pantalla 0 - pantalla de trabajo
pantalla 11 - menu principal (parte 1)
pantalla 12 - menu principal (parte 2)
pantalla 2 - sub menu 1 (EXTRUSORES)
pantalla 3 - sub menu 2 (COLECTORES PET)
pantalla 4 - sub menu 3 (COLECTORES fil.)
pantalla 5 - sub menu 4 (ALARMAS)
pantalla 6 - sub menu 5 (HISTORIAL)
pantalla 7 - sub menu 6 (ACERCA DE...) (parte 1)
pantalla 8 - sub menu 6 (ACERCA DE...) (parte 2)
pantalla 9 - sub sub menu 1 (Seteo temp Extrusor)
*/
#define workingScreen 			0
#define mainMenu_part1 			11
#define mainMenu_part2 			12
#define subMenuExtrusores 		2
#define subMenuColectoresPET 	3
#define subMenuColectoresFil 	4
#define subMenuAlarmas			5
#define subMenuHistorial		6
#define subMenuAcercaDe_part1	7
#define menuSetTempExt			8
#define cantItemsPrinMenu 		7
#define cantItemsSubMenu1 		2
#define cantItemsSubMenu2 		2
#define cantItemsSubMenu3 		2
#define cantItemsSubMenu4 		3
#define cantItemsSubMenu5 		3
#define cantItemsSubMenu6 		2

uint8_t currentScreen = 0;
uint8_t previousScreen = 0;
uint32_t previousTick = 0;
uint32_t currentTick = 0;
enum Button btnPressed = Ok;
uint8_t indexes[9] = {0,0,0,0,0,0,0,0,0};
uint8_t previousIndexes[9] = {1,1,1,1,1,1,1,1,1};

/*
indexes[0] -> NO SE UTILIZA
indexes[1] -> pantalla 1 - menu principal
indexes[2] -> pantalla 2 - sub menu 1
indexes[3] -> pantalla 3 - sub menu 2
indexes[4] -> pantalla 4 - sub menu 3
indexes[5] -> pantalla 5 - sub menu 4
indexes[6] -> pantalla 6 - sub menu 5
indexes[7] -> pantalla 7 - sub menu 6
indexes[8] -> pantalla 8 - sub sub menu 1
*/

PETfilConv PETfilConv1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
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
	PETfilConv1.colecFil_state = false;
	PETfilConv1.colecPET_state = false;
	PETfilConv1.filDetector_state = false;
	PETfilConv1.lightAlarm_state = false;
	PETfilConv1.soundAlarm_state = false;
	PETfilConv1.ExtTemp = 25;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  HAL_Delay(100);
  printScreen(ScreenStart);
  HAL_Delay(4000);
  clearScreen();
  //lcd_send_cmd (CLEAR_LCD);  // clear display
  //HAL_Delay(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  updateDataGUI();
	  updateLCD();
	  updateCursor();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : RotaryDT_Pin RotaryCLK_Pin Button_Pin */
  GPIO_InitStruct.Pin = RotaryDT_Pin|RotaryCLK_Pin|Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	currentTick = HAL_GetTick();
	//Rotate encoder CCKW or CKW
	if(currentTick - previousTick > 300)
	{
		if(GPIO_Pin == RotaryDT_Pin)
		{
	    	//HAL_GPIO_TogglePin(GPIOB, LED_ROJO_Pin);
//	    	if(PrinMenuIndex == 1)PrinMenuIndex = 1;
//	    	else PrinMenuIndex--;
	    	btnPressed = Left;
	    	previousTick = currentTick;
		}

		else if(GPIO_Pin == RotaryCLK_Pin)
		{
	    	//HAL_GPIO_TogglePin(GPIOB, LED_VERDE_Pin);
//	    	if(PrinMenuIndex == 4) PrinMenuIndex = 4;
//	    	else PrinMenuIndex++;
	    	btnPressed = Right;
	    	previousTick = currentTick;
		}

		else if(GPIO_Pin == Button_Pin)
		{
//	    	if(PrinMenuIndex == 4) PrinMenuIndex = 4;
//	    	else PrinMenuIndex++;
	    	btnPressed = Ok;
	    	previousTick = currentTick;
		}
	}
}

void updateDataGUI(void)
{
	if(btnPressed != NoPressed)
	{
		switch(btnPressed)
		{
			case Ok:
				switch(currentScreen)
				{
					case workingScreen:	//pantalla de trabajo
						currentScreen = mainMenu_part1;
						indexes[1] = 0;
					break;

					case mainMenu_part1: //menu principal (parte 1)
						if(indexes[1] == 0) currentScreen = workingScreen;
						else if(indexes[1] == 1) currentScreen = subMenuExtrusores;
						else if(indexes[1] == 2) currentScreen = subMenuColectoresPET;
						else if(indexes[1] == 3) currentScreen = subMenuColectoresFil;
					break;

					case mainMenu_part2: //menu principal (parte 2)
						if(indexes[1] == 4) currentScreen = subMenuAlarmas;
						else if(indexes[1] == 5) currentScreen = subMenuHistorial;
						else if(indexes[1] == 6) currentScreen = subMenuAcercaDe_part1;
					break;

					case subMenuExtrusores: //sub menu 1 (EXTRUSORES)
						if(indexes[2] == 0) currentScreen = mainMenu_part1;
						else if (indexes[2] == 1) currentScreen = menuSetTempExt;
					break;

					case subMenuColectoresPET:	//sub menu 2 (COLECTORES PET)
						if(indexes[3] == 0) currentScreen = mainMenu_part1;
						else if(indexes[3] == 1)
						{
							if(PETfilConv1.colecPET_state == false)
							{
								PETfilConv1.colecPET_state = true;
								//imprime un OK en esa linea
							}
							else
							{
								PETfilConv1.colecPET_state = false;
								//borra OK en esa linea
							}
						}
					break;

					case subMenuColectoresFil:	//sub menu 3 (COLECTORES fil.)
						if(indexes[4] == 0) currentScreen = mainMenu_part1;
						else if(indexes[4] == 1)
						{
							if(PETfilConv1.colecFil_state == false)
							{
								PETfilConv1.colecFil_state = true;
								//imprime un OK en esa linea
							}
							else
							{
								PETfilConv1.colecFil_state = false;
								//borra OK en esa linea
							}
						}

					break;

					case subMenuAlarmas:	//sub menu 4 (ALARMAS)
						if(indexes[5] == 0) currentScreen = mainMenu_part2;
						else if(indexes[5] == 1)
						{
							if(PETfilConv1.lightAlarm_state == false)
							{
								PETfilConv1.lightAlarm_state = true;
								//imprime un OK en esa linea
							}
							else
							{
								PETfilConv1.lightAlarm_state = false;
								//borra OK en esa linea
							}
						}
						else if(indexes[5] == 2)
						{
							if(PETfilConv1.soundAlarm_state == false)
							{
								PETfilConv1.soundAlarm_state = true;
								//imprime un OK en esa linea
							}
							else
							{
								PETfilConv1.soundAlarm_state = false;
								//borra OK en esa linea
							}
						}

						if((PETfilConv1.soundAlarm_state == true)||(PETfilConv1.lightAlarm_state == true))	PETfilConv1.filDetector_state = true;
						else PETfilConv1.filDetector_state = false;
					break;

					case subMenuHistorial:	//sub menu 5 (HISTORIAL)
						if(indexes[6] == 0) currentScreen = mainMenu_part2;
						else{} //borra el valor acumulador del extrusor
					break;

					case subMenuAcercaDe_part1:	//sub menu 6 (ACERCA DE...)
						currentScreen = mainMenu_part2;
					break;

					case menuSetTempExt:	//menu seteo de temperatura COLECTORES
						currentScreen = subMenuColectoresPET;
					break;
				}
				break;

				case Left:
					switch(currentScreen)
					{
						case workingScreen:	//pantalla de trabajo
							//NADA
						break;

						case mainMenu_part1:
							if(indexes[1] == 0) indexes[1] = 0;
							else	indexes[1]--;
						break;

						case mainMenu_part2:
							if(indexes[1] <= 4)
							{
								currentScreen = mainMenu_part1;
								indexes[1]--;
							}
							else	indexes[1]--;
						break;

						case menuSetTempExt:	//menu seteo de temperatura COLECTORES
							if(PETfilConv1.ExtTemp == 0) PETfilConv1.ExtTemp = 0;
							else PETfilConv1.ExtTemp--;
						break;

						default:
							if(indexes[currentScreen] == 0) indexes[currentScreen] = 0;
							else indexes[currentScreen]--;
					}
				break;

				case Right:
					switch(currentScreen)
					{
						case workingScreen:	//pantalla de trabajo
							//NADA
						break;

						case mainMenu_part1:
							if(indexes[1] >= 3)
							{
								currentScreen = mainMenu_part2;
								indexes[1]++;
							}
							else	indexes[1]++;
						break;

						case mainMenu_part2:
							if(indexes[1] >= 6) indexes[1] == 6;
							else	indexes[1]++;
						break;

						case subMenuExtrusores:
							if(indexes[currentScreen] == (cantItemsSubMenu1-1)) indexes[currentScreen] = cantItemsSubMenu1-1;
							else indexes[currentScreen]++;
						break;

						case subMenuColectoresPET:
							if(indexes[currentScreen] == (cantItemsSubMenu2-1)) indexes[currentScreen] = cantItemsSubMenu2-1;
							else indexes[currentScreen]++;
						break;

						case subMenuColectoresFil:
							if(indexes[currentScreen] == (cantItemsSubMenu3-1)) indexes[currentScreen] = cantItemsSubMenu3-1;
							else indexes[currentScreen]++;
						break;

						case subMenuAlarmas:
							if(indexes[currentScreen] == (cantItemsSubMenu4-1)) indexes[currentScreen] = cantItemsSubMenu4-1;
							else indexes[currentScreen]++;
						break;

						case subMenuHistorial:
							if(indexes[currentScreen] == (cantItemsSubMenu5-1)) indexes[currentScreen] = cantItemsSubMenu5-1;
							else indexes[currentScreen]++;
						break;

						case subMenuAcercaDe_part1:
							if(indexes[currentScreen] == (cantItemsSubMenu6-1)) indexes[currentScreen] = cantItemsSubMenu6-1;
							else indexes[currentScreen]++;
						break;

						case menuSetTempExt:	//menu seteo de temperatura COLECTORES
							if(PETfilConv1.ExtTemp == 250) PETfilConv1.ExtTemp = 250;
							else PETfilConv1.ExtTemp++;	//Temp++
						break;
					}
				break;
		}

//		updateLCD();
		btnPressed = NoPressed;
	}
}

void updateLCD(void)
{
	if(currentScreen != previousScreen)
	{
//		previousScreen = currentScreen;

		switch(currentScreen)
		{
			case workingScreen: printScreen(ScreenWorking);
			break;

			case mainMenu_part1: printScreen(PrinMenu1);
			break;

			case mainMenu_part2: printScreen(PrinMenu2);
			break;

			case subMenuExtrusores: printScreen(SubMenu1);
			break;

			case subMenuColectoresPET: printScreen(SubMenu2);
			break;

			case subMenuColectoresFil: printScreen(SubMenu3);
			break;

			case subMenuAlarmas: printScreen(SubMenu4);
			break;

			case subMenuHistorial: printScreen(SubMenu5);
			break;

			case subMenuAcercaDe_part1: printScreen(SubMenu6);
			break;

			case menuSetTempExt: printScreenSettingTemp(PETfilConv1);
			break;
		}
	}
}

void printScreenSettingTemp(PETfilConv conversorPETfil)
{
	char aux = 0;
	  //lcd_send_cmd (CLEAR_LCD);  // clear display
	  //HAL_Delay(1);
	clearScreen();
	lcd_send_cmd(reg_lines[0]);
	lcd_send_string(SubMenu1_2[0]);
	lcd_send_cmd(reg_lines[2]);
	//REVISAR
	aux = '0'+ conversorPETfil.ExtTemp;
	lcd_send_string(aux);
}

void updateCursor(void)
{
  if(
		(previousIndexes[1] != indexes[1])||	//cambio indice en menu principal?
		(previousIndexes[2] != indexes[2])||	//cambio indice en sub menu 1?
		(previousIndexes[3] != indexes[3])||	//cambio indice en sub menu 2?
		(previousIndexes[4] != indexes[4])||	//cambio indice en sub menu 3?
		(previousIndexes[5] != indexes[5])||	//cambio indice en sub menu 4?
		(previousIndexes[6] != indexes[6])||	//cambio indice en sub menu 5?
		(previousIndexes[7] != indexes[7])||	//cambio indice en sub menu 6?
		(previousIndexes[8] != indexes[8] ||
		     previousScreen != currentScreen)		//cambio indice en sub sub menu 1?
	)
  {
	  previousScreen = currentScreen;
	  for(int i = 1 ; i < 9; i++) previousIndexes[i] = indexes[i];

	  clearAllCursor();

	switch(currentScreen)
	{
		case workingScreen:	//NO MUESTRA CURSOR
		break;

		case mainMenu_part1: printCursor(indexes[1]);
		break;

		case mainMenu_part2: printCursor(indexes[1]);
		break;

		case subMenuExtrusores: printCursor(indexes[2]);
		break;

		case subMenuColectoresPET: printCursor(indexes[3]);
		break;

		case subMenuColectoresFil: printCursor(indexes[4]);
		break;

		case subMenuAlarmas: printCursor(indexes[5]);
		break;

		case subMenuHistorial: printCursor(indexes[6]);
		break;

		case subMenuAcercaDe_part1:
		break;

		case menuSetTempExt:
		break;
	}
  }
}

void printCursor(uint8_t index)
{
	switch(index)
	{
		case 0:
			lcd_send_cmd(C_LINE1_LCD);	//primer linea
			lcd_send_string(CURSOR[0]);
		break;

		case 1:
			lcd_send_cmd(C_LINE2_LCD);	//primer linea
			lcd_send_string(CURSOR[0]);
		break;

		case 2:
			lcd_send_cmd(C_LINE3_LCD);	//primer linea
			lcd_send_string(CURSOR[0]);
		break;

		case 3:
			lcd_send_cmd(C_LINE4_LCD);	//primer linea
			lcd_send_string(CURSOR[0]);
		break;

		case 4:
			lcd_send_cmd(C_LINE1_LCD);	//primer linea
			lcd_send_string(CURSOR[0]);
			break;

		case 5:
			lcd_send_cmd(C_LINE2_LCD);	//primer linea
			lcd_send_string(CURSOR[0]);
			break;

		case 6:
			lcd_send_cmd(C_LINE3_LCD);	//primer linea
			lcd_send_string(CURSOR[0]);
			break;
	}
}

void clearAllCursor(void)
{
	lcd_send_cmd(C_LINE1_LCD);	//primer linea
	lcd_send_string(CHAR_VACIO[0]);
	lcd_send_cmd(C_LINE2_LCD);	//primer linea
	lcd_send_string(CHAR_VACIO[0]);
	lcd_send_cmd(C_LINE3_LCD);	//primer linea
	lcd_send_string(CHAR_VACIO[0]);
	lcd_send_cmd(C_LINE4_LCD);	//primer linea
	lcd_send_string(CHAR_VACIO[0]);
}

void printScreen(const char *screen[])
{
	  //lcd_send_cmd (CLEAR_LCD);  // clear display
	  //HAL_Delay(1);
	clearScreen();
	  for(int i = 0 ; i < LINES ; i++)
	  {
		  lcd_send_cmd(reg_lines[i]);
		  lcd_send_string(screen[i]);
	  }
}

void clearScreen(void)
{
	  lcd_send_cmd (CLEAR_LCD);  // clear display
	  HAL_Delay(100);
}

void getTemp(void)
{

}

void PIDTemp(void)
{

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
