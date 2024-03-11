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

void initLEDs(void);
void initPeriph(void);


void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Enable GPIOC, GPIOB, and I2C Clock in RCC
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	
	// initialize GPIO modes and I2C peripherals
	initPeriph();
	
	// initialize LEDs
	initLEDs();
	
	

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

/* Initialize all configured peripherals */
void initPeriph(void) {
	/* Setting the GPIO Modes */
	// Set PB11 - AF1
	GPIOB->MODER |= (GPIO_MODER_MODER11_1); // (Alternate function: 10)
	//GPIOB->OTYPER |= (0x1 << GPIO_OTYPER_OT_11); // (Open-drain: 1)
	GPIOB->OTYPER |= (1<<11); // (Open-drain: 1)
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos); // select I2C2_SDA as alternative function
	
	// Set PB13 - AF5
	GPIOB->MODER |= (GPIO_MODER_MODER13_1); // (Alternate function: 10)
	//GPIOB->OTYPER |= (0x1 << GPIO_OTYPER_OT_13); // (Open-drain: 1)
	GPIOB->OTYPER |= (1<<13); // (Open-drain: 1)
	GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos); // select I2C2_SCL as alternative function
	
	// Set PB14 - initialize high
	GPIOB->MODER |= (GPIO_MODER_MODER14_0); // (General purpose: 01)
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14); // (Push-pull: 0)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // set pin high

	// Set PC0 - initialize high
	GPIOC->MODER |= (GPIO_MODER_MODER0_0); // (General purpose: 01)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0); // (Push-pull: 0)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // set pin high

	/* Initializing the I2C Peripheral */
	// Set the parameters in the TIMINGR reg to use 100 kHz (section 26.7.5 Periph manual for mapping)
	I2C2->TIMINGR |= (0x1  << I2C_TIMINGR_PRESC_Pos);  // PRESC = 1    [bits 28-31]
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);   // SCLL = 0x13  [bits 0-7]
	I2C2->TIMINGR |= (0x0F << I2C_TIMINGR_SCLH_Pos);   // SCHL = 0xF   [bits 8-15]
	I2C2->TIMINGR |= (0x2  << I2C_TIMINGR_SDADEL_Pos); // SDADEL = 0x2 [bits 16-19]
	I2C2->TIMINGR |= (0x4  << I2C_TIMINGR_SCLDEL_Pos); // SCLDEL = 0x4 [bits 20-23]
	//I2C2->TIMINGR |= 0x10420F13; // Alternative approach
	
	I2C2->CR1 |= I2C_CR1_PE; // Peripheral enable
}

/* Initialize the LEDs*/
void initLEDs(void) {	
	/* Initialize all LEDs: RED (PC6), BLUE (PC7), ORANGE (PC8), GREEN (PC9)	*/ // /*
	// (Reset state: 00)
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	
	// (General purpose: 01) 
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	
	// Configure Push/Pull Output type for PC6, PC7, PC8, and PC9	(00)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	
	// Configure low speed for PC6, PC7, PC8, and PC9	(00)
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk | GPIO_OSPEEDR_OSPEEDR8_Msk | GPIO_OSPEEDR_OSPEEDR9_Msk);
	
	// Configure no pull-up/down resistors for PC6, PC7, PC8, and PC9	(00)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR9_Msk);
	
	// Initialize pins to logic high and the other to low.
	GPIOC->BSRR = GPIO_BSRR_BS_6;	// Set PC6 high
	GPIOC->BSRR = GPIO_BSRR_BS_7; // Set PC7 high
	GPIOC->BSRR = GPIO_BSRR_BS_8;	// Set PC8 high
	GPIOC->BSRR = GPIO_BSRR_BS_9; // Set PC9 high
	//	*/
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
