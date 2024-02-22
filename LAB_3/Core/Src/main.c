/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
  HAL_Init();

	// Enable the timer 2 (TIM2) and 3 (TIM3) peripheral in the RCC
  RCC->APB1ENR	|=	RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR	|=	RCC_APB1ENR_TIM3EN;
	
	// Enable the LEDs
	RCC->AHBENR	|=	RCC_AHBENR_GPIOCEN;
	
	// NVIC setup for TIM2 handler
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// Configure the timer (TIM2) to trigger an update event (UEV) at 4 Hz
	TIM2->PSC		 =	0x1F3F;	// 7999
	TIM2->ARR		 =	0xFA;		// 250
	
	// Configure the timer (TIM3) to trigger an update event (UEV) at 800 Hz
	TIM3->PSC		 =	0xF9;	// 249
	TIM3->ARR		 =	0x28;	// 40
	
	// Configure the timer to generate an interrupt on the UEV event
	TIM2->DIER	|=	TIM_DIER_UIE;
	
	// Configure an enable/start the timer 2
	TIM2->CR1		|=	TIM_CR1_CEN;
	
	// Configure an enable/start the timer 3
	TIM3->CR1		|=	TIM_CR1_CEN;
	
	// Set channels 1 & 2 to output (00)
	TIM3->CCMR1	&=	~(TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1);
	TIM3->CCMR1	&=	~(TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC2S_1);
	
	// Set output channel 1 to PWM Mode 2 (111)
	TIM3->CCMR1	|=	(TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0);
	
	// Set output channel 2 to PWM Mode 1 (110)
	TIM3->CCMR1	|=	(TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
	TIM3->CCMR1	&=	~(TIM_CCMR1_OC2M_0);
	
	// Enable the output compare preload for both channels
	TIM3->CCMR1	|=	(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
	
	// Set the output enable bits for channels 1 & 2 in the CCER register
	TIM3->CCER	|=	(TIM_CCER_CC1E | TIM_CCER_CC2E);
	
	/* Set the capture/compare registers (CCRx) for both channels to 20% of your ARR value */ // /*
	TIM3->CCR1	 =	8; 	// Controls RED  LED; 40*20% = 8
	TIM3->CCR2	 =	8; 	// Controls BLUE LED; 40*20% = 8		*/
	
	/* Both LEDs dim */  /*
	TIM3->CCR1	 =	40;	// Controls RED  LED; 40*100% = 40
	TIM3->CCR2	 =	5;	// Controls BLUE LED; 40*12.5% = 5	*/
	
	/* BLUE LED more dim */  /*
	TIM3->CCR1	 =	5; 	// Controls RED  LED; 40*12.5% = 5
	TIM3->CCR2	 =	5; 	// Controls BLUE LED; 40*12.5% = 5	*/
	
	/* RED LED more dim */  /*
	TIM3->CCR1	 =	40; // Controls RED  LED; 40*100% = 40
	TIM3->CCR2	 =	40; // Controls BLUE LED; 40*100% = 40	*/
	
	
	/* Initialize LEDs: RED (PC6), BLUE (PC7), ORANGE (PC8), and GREEN (PC9)	*/ // /*
	// (Reset state: 00)
	GPIOC->MODER	&=	~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	
	// Push-Pull Output type for PC6, PC7, PC8, and PC9: 00
	GPIOC->OTYPER	&=	~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	
	// No pull-up/down resistors for PC6, PC7, PC8, and PC9: 00
	GPIOC->PUPDR	&=	~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR9_Msk);
	
	// General purpose mode for PC8 and PC9: 01
	GPIOC->MODER	|=	(GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	
	// Alternate function mode for PC6 and PC7: 10
	GPIOC->MODER	|=	(GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
	
	// High speed for PC8 and PC9: 11
	GPIOC->OSPEEDR	&=	~(GPIO_OSPEEDR_OSPEEDR8 | GPIO_OSPEEDR_OSPEEDR9);
	
	// Low speed for PC6 and PC7: 00
	GPIOC->OSPEEDR	&=	~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk);
	
	// Initialize pins to logic high and the other to low.
	GPIOC->BSRR		 =	GPIO_BSRR_BR_8;	// Set PC8 low
	GPIOC->BSRR		 =	GPIO_BSRR_BS_9; // Set PC9 high
	//	*/
	
	SystemClock_Config();
	
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

// Set up the timer's interrupt handler, and enable in the NVIC
void TIM2_IRQHandler(void)
{
	// Toggle between ORANGE and GREEN LEDs
	GPIOC->ODR	^=	(GPIO_ODR_8 | GPIO_ODR_9);
	TIM2->SR		^=	TIM_SR_UIF; // Update interrupt flag
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
