/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "UartControl.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"

#include <stdio.h>

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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern int timer4_int;
extern int timer1_int;

char TmpBuffer[102];
int len;
char Rx_index, Rx_data[2], Rx_Buffer[100], EndOfTrans;

//0519
uint8_t rxBuffer[2];

//ADC
__IO uint16_t ADCvalue[2];

float adc_data[2];

// uart receive  // 0518 i dont know
//int data;
//uint8_t data;  //0519

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void delay_10ms(uint8_t cnt);
void LedProc(void);
void ssd1306_InitMsg(void);
void ShellProcess(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float floatmap(float x, float inMin, float inMax, float outMin, float outMax ) {
	return (x-inMin)*(outMax-outMin)/(inMax-inMin)+outMin;
} // transfer short -> float
  // ADCvalue[] -> adc_data[]
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
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Board LED OFF
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);

  // UART setting
  UartReceiveStart(&huart2);

  // OLED setting
  ssd1306_InitMsg();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  int i;
	  int adc_motor;
	  int adc_led;

	  uint8_t buffer[1000];  //ADC DMA sprintf

	  //ShellProcess();

	  /////////////////////////////////////////////////////////////////////////////////////
	  // ADC value to Voltage
//	  while(1) {
//		  printf("==================================\r\n");
//		  printf("Convert 'ADC value' to '0V~3.3V'  \r\n");
//		  printf("==================================\r\n");
//		  HAL_Delay(1000);
//
//		  printf("ADC test\r\n");
//		  for(i=30;i>0;i--){
//			  HAL_ADC_Start_DMA(&hadc1, ADCvalue, 2); // jumper 2&3 range is 0 to 3.3V
//			  adc_data[0] = floatmap(ADCvalue[0], 0, 4095, 0, 3.3);
//			  printf("%f\r\n",adc_data[0]);  // by 'printf'
//			  HAL_Delay(1000);
//		  }
//		  printf("******************\r\n");
//	  }
	  /////////////////////////////////////////////////////////////////////////////////////



	  /////////////////////////////////////////////////////////////////////////////////////
	  // LED ON/OFF
//  	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
//  	  		HAL_Delay(2000);
//  	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
//  	  		HAL_Delay(2000);
  	  /////////////////////////////////////////////////////////////////////////////////////



	  /////////////////////////////////////////////////////////////////////////////////////
	  // ADC DMA and uart display
//	  HAL_ADCEx_Calibration_Start(&hadc1);
//	  while(1) {
//		  HAL_ADC_Start_DMA(&hadc1, ADCvalue, 2);
//		  printf("ADC value: \r\n");
//		  sprintf(buffer, "%d \r\n", ADCvalue[0]);
//		  HAL_UART_Transmit(&huart2, buffer, strlen(buffer),10);
//		  HAL_Delay(1000);
//
//		  adc_data[0] = floatmap(ADCvalue[0], 0, 4095, 0, 3.3);
//		  printf("Voltage: \r\n");
//		  sprintf(buffer, "%f \r\n", adc_data[0]);
//		  HAL_UART_Transmit(&huart2, buffer, strlen(buffer),10);
//		  HAL_Delay(1000);
//	  }
	  /////////////////////////////////////////////////////////////////////////////////////



	  /////////////////////////////////////////////////////////////////////////////////////
	  // motor speed ex.1
//	   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,00);  // change timer pulse value
//	   HAL_Delay(10);
//	   while(1);
	  /////////////////////////////////////////////////////////////////////////////////////



	  /////////////////////////////////////////////////////////////////////////////////////
	  //motor speed ex.2
//		ssd1306_Init();
//		ssd1306_Fill(Black);
//		ssd1306_SetCursor(2, 0);
//		ssd1306_WriteString("ElecAcademy", Font_11x18, White);
//		ssd1306_SetCursor(2, 22);
//		ssd1306_WriteString("Motor test ", Font_11x18, White);
//
//		ssd1306_SetCursor(2, 22+22);
//		ssd1306_WriteString("PWM change ", Font_11x18, White);
//		ssd1306_UpdateScreen();
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//		while(1) {
//			for(i=600;i>100;i--){
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,i); //pulse change
//			HAL_Delay(10);
//			}
//		for(i=100;i<600;i++){
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,i); //pulse change
//		  	HAL_Delay(10);
//			}
//		}
	  /////////////////////////////////////////////////////////////////////////////////////



	  /////////////////////////////////////////////////////////////////////////////////////
	  // ADC DMA and OLED display
//	  	HAL_ADCEx_Calibration_Start(&hadc1);
//	  	ssd1306_Init();
//	  	ssd1306_Fill(Black);
//	  	ssd1306_SetCursor(2, 0);
//	  	ssd1306_WriteString("MOTOR TEST", Font_11x18, White);
//	  	ssd1306_SetCursor(2, 22);
//	  	ssd1306_WriteString("PWM control", Font_11x18, White);
//	  	while(1) {
//	  		HAL_ADC_Start_DMA(&hadc1, ADCvalue, 2); // jumper 2&3 range is 0 to 3.3V
//	  		adc_data[0] = floatmap(ADCvalue[0], 0, 4095, 0, 3.3);
//	  		sprintf(buffer, "adc_data : %d \r\n", ADCvalue[0]); // jumper 2&3 range is 0 to 4095
//	  		HAL_UART_Transmit(&huart2, buffer, strlen(buffer),10);  // by 'UART_Transmit'
//	  		HAL_Delay(100);
//
//	  		adc_motor = adc_data[0]*300;
//	  		sprintf(buffer, "%d\r\n", adc_motor); // jumper 2&3 range is 0 to 4095
//	  		HAL_UART_Transmit(&huart2, buffer, strlen(buffer),10);
//			HAL_Delay(100);
//	  		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	  		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,adc_motor);  // change timer pulse value
//
//	  		ssd1306_SetCursor(2, 22+22);
//	  		ssd1306_WriteString(buffer, Font_11x18, White);
//	  		ssd1306_UpdateScreen();
//	  		HAL_Delay(800);
//	  	  }
	  	///////////////////////////////////////////////////////////////////////////////////////





	  /////////////////////////////////////////////////////////////////////////////////////
	  // LED speed ex.1
//	  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,500);  // change timer pulse value
//	  	__HAL_TIM_SET_PRESCALER(&htim1,8000); // change timer clock div. value
//	  	//(&htim1)->Instance->PSC = 4000;
//	  	HAL_Delay(10);
//	  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	  	HAL_Delay(10);
//	  	while(1);
	  	/////////////////////////////////////////////////////////////////////////////////////




  	  	/////////////////////////////////////////////////////////////////////////////////////
	  	  // ADC DMA and OLED display : pulse duty control
//	  	  HAL_ADCEx_Calibration_Start(&hadc1);
//	  	  __HAL_TIM_SET_PRESCALER(&htim1,8000); // change timer clock div. value
//
//	  	  ssd1306_Init();
//	  	  while(1) {
//	  		  HAL_ADC_Start_DMA(&hadc1, ADCvalue, 2); // jumper 2&3 range is 0 to 3.3V
//			  adc_data[0] = floatmap(ADCvalue[0], 0, 4095, 0, 3.3);
//			  sprintf(buffer, "adc_data : %d \r\n", ADCvalue[0]); // jumper 2&3 range is 0 to 4095
//			  HAL_UART_Transmit(&huart2, buffer, strlen(buffer),10);
//			  HAL_Delay(100);
//			  adc_led = adc_data[0]*300;
//	  	  	  sprintf(buffer, "%d\r\n", adc_led); // jumper 2&3 range is 0 to 4095
//	  	  	  HAL_UART_Transmit(&huart2, buffer, strlen(buffer),10);
//	  	  	  HAL_Delay(100);
//	  	  	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,adc_led);  // change timer pulse value
//	  	      //(&htim1)->Instance->PSC = 4000;
//	  	      HAL_Delay(10);
//	  	      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	  	      HAL_Delay(10);
//
//	  	      ssd1306_Fill(Black);
//	  	      ssd1306_SetCursor(2, 0);
//	  	      ssd1306_WriteString("LED TEST", Font_11x18, White);
//	  	      ssd1306_SetCursor(2, 22);
//	  	      ssd1306_WriteString("PWM control", Font_11x18, White);
//	  	      ssd1306_SetCursor(2, 22+22);
//	  	      ssd1306_WriteString(buffer, Font_11x18, White);
//	  	      ssd1306_UpdateScreen();
//	  	      HAL_Delay(600);
//	  	  	}
	  	/////////////////////////////////////////////////////////////////////////////////////



	  	  /////////////////////////////////////////////////////////////////////////////////////
	  	  // ADC DMA and OLED display : frequency control
//	  	  HAL_ADCEx_Calibration_Start(&hadc1);
//	  	  ssd1306_Init();
//	  	  ssd1306_Fill(Black);
//	  	  ssd1306_SetCursor(2, 0);
//	  	  ssd1306_WriteString("LED TEST", Font_11x18, White);
//	  	  ssd1306_SetCursor(2, 22);
//	  	  ssd1306_WriteString("PWM control", Font_11x18, White);
//
//	  	  while(1) {
//	  		  HAL_ADC_Start_DMA(&hadc1, ADCvalue, 2); // jumper 2&3 range is 0 to 3.3V
//			  adc_data[0] = floatmap(ADCvalue[0], 0, 4095, 0, 3.3);
//			  adc_led = adc_data[0]*3000;
//	  	  	  sprintf(buffer, "%d\r\n", adc_led); // jumper 2&3 range is 0 to 4095
//	  	  	  HAL_UART_Transmit(&huart2, buffer, strlen(buffer),10);
//	  	  	  HAL_Delay(10);
//	  	  	  __HAL_TIM_SET_PRESCALER(&htim1,adc_led); // change timer clock div. value
//	  	  	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,adc_led%2);  // change timer pulse value
//
//	  	      //(&htim1)->Instance->PSC = 4000;
//	  	      HAL_Delay(10);
//	  	      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	  	      HAL_Delay(10);
//
//
//	  	  		ssd1306_SetCursor(2, 22+22);
//	  	  		ssd1306_WriteString(buffer, Font_11x18, White);
//	  	  		ssd1306_UpdateScreen();
//	  	  		HAL_Delay(300);
//	  	  	}
	  	  /////////////////////////////////////////////////////////////////////////////////////



	  /*
	  /////////////////////////////////////////////////////////////////////////////////////
	  // heater control ex.1
	  	ssd1306_Init();
	  	ssd1306_SetCursor(1, 0);
	  	ssd1306_WriteString("ElecAcademy", Font_11x18, White);
	  	ssd1306_SetCursor(2, 20);
	  	ssd1306_WriteString("HEATER TEST", Font_11x18, White);
	  	ssd1306_SetCursor(2, 20+21);
	  	ssd1306_WriteString(": PWM on", Font_11x18, White);
	  	ssd1306_UpdateScreen();
	    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,990);  // change timer pulse value
	  	HAL_Delay(10);
	  	while(1);
	  /////////////////////////////////////////////////////////////////////////////////////
	  */




	  // user LCD
//		ssd1306_Init();
//		ssd1306_Fill(Black);
//	    ssd1306_SetCursor(2, 0);
//	    ssd1306_WriteString("EA 2nd", Font_16x26, White);
//	    ssd1306_SetCursor(2, 20);
//	    ssd1306_WriteString("OLED font", Font_11x18, White);
//	    ssd1306_SetCursor(2, 20+20);
//	    ssd1306_WriteString("Elec_Academy", Font_7x10, White);
//	    ssd1306_UpdateScreen();
//	    //FontDef Font_6x8 = {6,8,Font6x8};
//	    //FontDef Font_7x10 = {7,10,Font7x10};
//	    //FontDef Font_11x18 = {11,18,Font11x18};
//	    //FontDef Font_16x26 = {16,26,Font16x26};
//	    while(1);




  	  	/*
	  // MOTOR on/off
	  	  ssd1306_Init();
	  	  ssd1306_SetCursor(1, 0);
	  	  ssd1306_WriteString("ElecAcademy", Font_11x18, White);
	  	  while(1) {
	  		ssd1306_SetCursor(2, 20);
	  		ssd1306_WriteString("MOTOR  ON", Font_11x18, White);
	  		ssd1306_SetCursor(2, 20+21);
	  		ssd1306_WriteString("LED OFF", Font_11x18, White);
	  		ssd1306_UpdateScreen();
	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1); // LED on
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);  // Motor on
	  		HAL_Delay(800);
	  		ssd1306_SetCursor(2, 20);
	  		ssd1306_WriteString("MOTOR OFF", Font_11x18, White);
	  		ssd1306_SetCursor(2, 20+21);
	  		ssd1306_WriteString("LED  ON", Font_11x18, White);
	  		ssd1306_UpdateScreen();
	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);  // LED off
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);   // Motor off
	  		HAL_Delay(800);
	  	  }
		*/
	  //


	  /*
	  if(timer4_int==1)
	  {
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //SW1
	  }
	  else
	  {
		  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	  }
	  HAL_Delay(800);
	  if(timer1_int==1)
	  {
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //SW2
	  }
	  else
	  {
	  	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	  }
	  HAL_Delay(800);
	  */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */


  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 48-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	data[0] = floatmap(ADCvalue[0], 0, 4095, 0, 3.3);
	sprintf(currentline, "s%f", data[0]);
	//HAL_UART_Transmit_DMA(&huart2, currentline, 9);
}
*/

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
