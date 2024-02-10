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
#include "stm32f072xb.h"

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
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
		
	/* Configuring a GPIO Pin to Output and Blink an LED */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	// Enable the GPIOC Clock in RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	// Enable the USER Button PA0
	
	// Use the RCC to enable the peripheral clock to the SYSCFG peripheral.
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	
	// Enable the selected EXTI interrupt
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	/* Set the priority for the interrupt to 1 (high-priority)	*/ // /*
	NVIC_SetPriority(EXTI0_1_IRQn, 1);	// */
	
	/* Set the priority for the interrupt to 3 (low-priority)		*/  /*
	NVIC_SetPriority(EXTI0_1_IRQn, 3);	// */
	
	// Change the Systick interrupt priority to 2 (medium priority)
	//NVIC_SetPriority(SysTick_IRQn, 2);
	
	
	/* Initialize all LEDs: RED (PC6), BLUE (PC7), ORANGE (PC8), GREEN (PC9)	*/ // /*
	// (Reset state: 00)
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	
	// (General purpose: 01) 
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	
	// Configure Push/Pull Output type for PC6, PC7, PC8, and PC9	(00)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	
	// Configure low speed for PC6, PC7, PC8, and PC9	(00)
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk | GPIO_OSPEEDR_OSPEEDR8_Msk | GPIO_OSPEEDR_OSPEEDR9_Msk);
	
	// Configure no pull-up/down resistors for PC6, PC7, PC8, and PC9	(00)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR9_Msk);
	
	// Initialize pins to logic high and the other to low.
	GPIOC->BSRR = GPIO_BSRR_BR_6;	// Set PC6 low
	GPIOC->BSRR = GPIO_BSRR_BR_7; // Set PC7 low
	GPIOC->BSRR = GPIO_BSRR_BR_8;	// Set PC8 low
	GPIOC->BSRR = GPIO_BSRR_BS_9; // Set PC9 high
	//	*/
	
	/* Configuring a GPIO Pin to Input and Reading a Button */
	// Configure input mode for PA0	(00)
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk);
	
	// Configure low speed for PA0	(00)
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk);
	
	// Configure Pull-down for PA0	(10)
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk); // clear bits first
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;			// Set to Pull-down
	
	// Enable/unmask interrupt generation on EXTI input line 0 (EXTI0).
	EXTI->IMR |= EXTI_IMR_MR0_Msk;
	
	// Configure the EXTI input line 0 to have a rising-edge trigger.
	EXTI->RTSR |= EXTI_RTSR_TR0_Msk;
	
	// Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).
	SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR1_EXTI0_PA);
	
	while (1) {
		/* Toggle RED PC6 */ // /*
		HAL_Delay(400); // 400ms delay
		GPIOC->ODR ^= (GPIO_ODR_6); // */
	}
}

volatile uint32_t count2 = 0;	// Declare global count variable
// Use the handler name to declare the handler function in main.c
void EXTI0_1_IRQHandler(void)
{
	// Check triggering of EXTI0
	EXTI->PR |= EXTI_PR_PR0;
	
	// Toggle ORANGE and GREEN LEDs
	GPIOC->ODR ^= (GPIO_ODR_8 | GPIO_ODR_9);
	
	
	/* Second sign-off */  /*
	// Add a delay loop of roughly 1-2 seconds to the EXTI interrupt handler
	for (count2 = 0; count2 < 1500000; count2++){
		// Empty loop that acts as delay.
	}
	
	// Add a second ORANGE and GREEN LED toggle
	GPIOC->ODR ^= (GPIO_ODR_8 | GPIO_ODR_9); // */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
