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
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	/* This example uses HAL library calls to control
	the GPIOC peripheral. You’ll be redoing this code
	with hardware register access. */
	
	//__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC

	// Set up a configuration struct to pass to the initialization function
	/*GPIO_InitTypeDef initStr = {
		GPIO_PIN_8 | GPIO_PIN_9,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL};*/
	//HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high
	
	
	/* Lab 1 - Part 1*/
	// Enable the GPIOC Clock in RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Enable the GPIOA Clock for USER Button PA0
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	
	/* ORANGE (PC8) & GREEN (PC9) LEDs */ // /*
	// Clears the 16th (PC8 [MODER8(17,16)]) and 18th (PC9 [MODER9(19,18)]) bits in the GPIOC_MODER register (Reset state: 00)
	//GPIOC->MODER &= ~((1 << 16) | (1 << 18));
	GPIOC->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	
	// Sets the 16th (PC8 [MODER8(17,16)]) and 18th (PC9 [MODER9(19,18)]) bits in the GPIOC_MODER register (General purpose: 01)
	//GPIOC->MODER |= (1 << 16) | (1 << 18);
	GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0); 
	
	// Configure Push/Pull Output type for PC8 and PC9
	//GPIOC->OTYPER |= 0x0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	
	// Configure low speed for PC8 and PC9
	//GPIOC->OSPEEDR |= 0x0;
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8_Msk | GPIO_OSPEEDR_OSPEEDR9_Msk);
	
	// Configure no pull-up/down resistors for PC8 and PC9
	//GPIOC->PUPDR |= 0x0;
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR9_Msk);
	
	// Initialize pins to logic high and the other to low.
	//GPIOC->ODR |= (1 << 8) | (0 << 9);
  GPIOC->BSRR = GPIO_BSRR_BS_8; // PC8 high
  GPIOC->BSRR = GPIO_BSRR_BR_9; // PC9 low
	// */
	
	
	
	/* RED    (PC6) & BLUE  (PC7) LEDs */  /*
	// Clears the 12th (PC6 [MODER8(13,12)]) and 14th (PC7 [MODER9(15,14)]) bits in the GPIOC_MODER register (Reset state: 00)
	//GPIOC->MODER &= ~((1 << 12) | (1 << 14));
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);
	
	// Sets the 12th (PC6 [MODER8(13,12)]) and 14th (PC7 [MODER9(15,14)]) bits in the GPIOC_MODER register (General purpose: 01)
	//GPIOC->MODER |= (1 << 12) | (1 << 14); 
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
	
	// Configure Push/Pull Output type for PC6 and PC7
	//GPIOC->OTYPER |= 0x0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7);
	
	// Configure low speed for PC6 and PC7
	//GPIOC->OSPEEDR |= 0x0;
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk);
	
	// Configure no pull-up/down resistors for PC6 and PC7
	//GPIOC->PUPDR |= 0x0;
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk);
	
	// Initialize pins to logic high and the other to low.
	//GPIOC->ODR |= (1 << 6) | (0 << 7);
  GPIOC->BSRR = GPIO_BSRR_BS_6; // Set PC6 high
  GPIOC->BSRR = GPIO_BSRR_BR_7; // Set PC7 low
	// */

	
	
	/* Lab 1 - Part 2 */
	// Configure input mode (00) for PA0
	//GPIOA->MODER |= (0 << 0);
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk);
	
	// Configure low speed for PA0
	//GPIOA->OSPEEDR |= 0x0;
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk);
	
	// Configure Pull-down for PA0
	//GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk; // clear bits
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;		// Set to Pull-down (10)
	
	
	

	uint32_t debouncer = 0;
	while (1) {
		HAL_Delay(200); // Delay 200ms
		
		debouncer = (debouncer << 1); // Always shift every loop iteration
		if (GPIOA->IDR & 1) { // If input signal is set/high
			debouncer |= 0x01; // Set lowest bit of bit-vector
		}
		
		if (debouncer == 0xFFFFFFFF) {
		// This code triggers repeatedly when button is steady high!
			
		}

		if (debouncer == 0x00000000) {
		// This code triggers repeatedly when button is steady low!
			
		}
		
		if (debouncer == 0x7FFFFFFF) {
		// This code triggers only once when transitioning to steady high!
			
			
		}
		// When button is bouncing the bit-vector value is random since bits are set when the button is high and not when it bounces low.
		
		
		/* Orange and green
		GPIOC->ODR |= (1 << 8);
		HAL_Delay(200);
		GPIOC->ODR |= (1 << 9); //*/
		
		// Blue and red
		GPIOC->ODR |= (1 << 6);
		HAL_Delay(200);
		GPIOC->ODR |= (1 << 7);
		
		
		
		// Toggle the output state of both PC8 and PC9
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
	}
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
