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
#include "tim.h"
#include "gpio.h"

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
enum abeceda{
	A = 0b1110111,
	a = 0b1111101,
	b = 0b0011111,
	C = 0b1001110,
	c = 0b0001101,
	d = 0b0111101,
	E = 0b1001111,
	F = 0b1000111,
	G = 0b1011110,
	H = 0b0110111,
	h = 0b0010111,
	I = 0b0000110,
	J = 0b0111100,
	L = 0b0001110,
	n = 0b0010101,
	O = 0b1111110,
	o = 0b0011101,
	P = 0b1100111,
	q = 0b1110011,
	r = 0b0000101,
	S = 0b1011011,
	t = 0b0001111,
	U = 0b0111110,
	u = 0b0011100,
	y = 0b0111011,

	K = 0b1010111,
	M = 0b1101010,
	W = 0b0111111,
	X = 0b1001001,
	Z = 0b1101101,
	V = 0b0101010,

	num_1 = 0b0110000,
	num_2 = 0b1101101,
	num_3 = 0b1111001,
	num_4 = 0b0010011,
	num_5 = 0b1011011,
	num_6 = 0b1011111,
	num_7 = 0b1110000,
	num_8 = 0b1111111,
	num_9 = 0b1111011,
	num_0 = 0b1111110,
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  turnON_digit(0,0,0,1);
	  write_character(num_2);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 4 */
void write_character(unsigned short ch)
{
static char bin[8];
static char inverted_bin[8];

	// write bin number into an array
	for (int i = 0;i < 8; i++)
	{
		bin[i] = ch & 0x80 ? '1' : '0';
		ch <<= 1;
	}

	// invert characters in bin array
	for (int i = 0;i < 8; i++)
	{
		if (bin[i] == '1'){inverted_bin[i] = 0;}
		else if (bin[i] == '0'){inverted_bin[i] = 1;}
	}


	if(inverted_bin[1] == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
	}
	if(inverted_bin[1] == 0){
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
	}
	//--------------------------------------------------
	if(inverted_bin[1] == 1){
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
		}
	if(inverted_bin[1] == 0){
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
	}
	//--------------------------------------------------
	if(inverted_bin[1] == 1){
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
	}
	if(inverted_bin[1] == 0){
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
	}
	//--------------------------------------------------
	if(inverted_bin[1] == 1){
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
	}
	if(inverted_bin[1] == 0){
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
	}
	//--------------------------------------------------
	if(inverted_bin[1] == 1){
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
	}
	if(inverted_bin[1] == 0){
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
	}
	//--------------------------------------------------
	if(inverted_bin[1] == 1){
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
	}
	if(inverted_bin[1] == 0){
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
	}
	//--------------------------------------------------
	if(inverted_bin[1] == 1){
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
	}
	if(inverted_bin[1] == 0){
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
	}

	/*
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_1, inverted_bin[1]);
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_0, inverted_bin[2]);
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_8, inverted_bin[3]);
	 HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_5, inverted_bin[4]);

	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_11, inverted_bin[5]);
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_3, inverted_bin[6]);
	 HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_4, inverted_bin[7]);
	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // desatinna ciara
	  */
}

void turnON_digit(int seg1,int seg2, int seg3, int seg4)
{
	if (seg1 == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
	}
	if (seg2 == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
	}
	if (seg3 == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
	}
	if (seg4 == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
