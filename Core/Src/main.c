/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ring_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFF_SIZE 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flag_err = 0;
uint8_t flag_tim = 0;
char str[3] = {0,};
RING_buffer_t ring;
uint8_t buff[BUFF_SIZE];
uint8_t brightness = 0;
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
	uint8_t tstring[255];
	uint8_t rstring[BUFF_SIZE + 1];
	char string[10] = {0,0};
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  RING_Init(&ring, buff, sizeof(buff) / sizeof(buff[0])); // Initialize UART receiver ring buffer.
  sprintf((char*)tstring,"\r\nEnter command:\n\r"
  		  "L=xx -- brightness for LEDs, where xx - value of brightness\n\r"
  		  "OR\n\r"
  		  "l=xx -- brightness for LEDs, where xx - value of brightness.\n\r\n\r");
  HAL_UART_Transmit(&huart2,tstring,strlen((char*)tstring), HAL_MAX_DELAY);
  // Start UART receiver in the non blocking mode
  HAL_UART_Receive_IT(&huart2,ring.buffer,1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Ring_GetMessage(&ring, rstring))
	  {
		  sscanf((char*)rstring,"%s", string);
		  for (int i = 0; i < 3; i++)
		  {
			  str[i] = string[i+2];
		  }
		  if (str[1] == '\r' || str[1] == '\n' || str[1] == '\0')
		  {
			  brightness = ((int) str[0]) - 48;
			  flag_err = 1;
		  }
		  else if (str[2] == '\r' || str[2] == '\n' || str[2] == '\0')
		  {
			  brightness = ((((int) str[0]) - 48) * 10) + (((int) str[1]) - 48);
			  flag_err = 1;
		  }
		  else
		  {
			  flag_err = 2;
		  }
		  RING_Clear(&ring);
		  if ((string[0] == 'L' || string[0] == 'l') && (string[1] == '=') && (flag_err == 1))
		  {
			  sprintf((char*)tstring,"\n\rEcho: %s\n\r"
			  				  "Enter command 'L=xx' or 'l=xx'\r\n",string);
		  }
		  else
		  {
			  sprintf((char*)tstring,"\n\rEcho: Wrong command!!!\r\n"
					  	  	  "Enter command 'L=xx' or 'l=xx'\r\n");
		  }
		  HAL_UART_Transmit_IT(&huart2,tstring,strlen((char*)tstring));
		  flag_err = 0;
		  if (flag_tim == 1)
		  {
			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			  TIM1->CCR1 = brightness;
		  	  TIM1->CCR2 = brightness;
		  	  TIM1->CCR3 = brightness;
		  	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		  	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		  	  flag_tim = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Check that interrupt caused by UART2
	if (huart == &huart2)
	{
		flag_tim = 1;
		// Put new character from the UART receiver data register (DR) to the ring buffer
		RING_Put(huart->Instance->DR, &ring);
		// Set the overrun flag if the message is longer than ring buffer can hold
		if (ring.idxOut == ring.idxIn) ring.flag.BufferOverrun = 1;
		// Set the message ready flag if the end of line character has been received
		if ((ring.buffer[ring.idxIn -1] == '\r') ||
				(ring.buffer[ring.idxOut -1] == '\n'))
			ring.flag.MessageReady = 1;
		// Receive the next character from UART in non blocking mode
		HAL_UART_Receive_IT(&huart2,&ring.buffer[ring.idxOut],1);
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
