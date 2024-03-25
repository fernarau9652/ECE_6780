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

void initLEDs(void);
void initPC1(void);
void initPA4(void);
void initADC(void);
void initDAC(void);
void calADC(void);
void thresLED(void);
void waveDAC(void);

void SystemClock_Config(void);


// Thresholds for Part 1
#define THRESHOLD_0 10
#define THRESHOLD_1 20
#define THRESHOLD_2 30
#define THRESHOLD_3 40


// Generative Waves for DAC
// Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	
// Triangle Wave: 8-bit, 32 samples/cycle
const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};

// Sawtooth Wave: 8-bit, 32 samples/cycle
const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};

// Square Wave: 8-bit, 32 samples/cycle (Don't use)
//const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
//254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	
	
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Enable GPIOC, GPIOA, ADC1, and DAC1 Clock in RCC
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_DAC1_CLK_ENABLE();
	
	// intialize LEDs
	initLEDs();
	
	/* Part 1 Initialization */	// /*
	// Setup for PC1
	initPC1();
	
	// Setup ADC and perform self-calibration
	initADC();
	calADC();		// */
	
	
	/* Part 2 Initialization */	// /*
	// Setup for PC1
	initPA4();
	
	// Setup ADC and perform self-calibration
	initDAC();	// */
	
 
  while (1) {
		/* Part 1 */	 /*
		thresLED();		// */
		
		/* Part 2 */	// /*
		waveDAC();		// */
  }
 
}


/* Initialize the LEDs*/
void initLEDs(void) {	
	/* Initialize all LEDs: RED (PC6), BLUE (PC7), ORANGE (PC8), GREEN (PC9)	*/
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
	GPIOC->BSRR = GPIO_BSRR_BR_6;	// Set PC6 low
	GPIOC->BSRR = GPIO_BSRR_BR_7; // Set PC7 low
	GPIOC->BSRR = GPIO_BSRR_BR_8;	// Set PC8 low
	GPIOC->BSRR = GPIO_BSRR_BR_9; // Set PC9 low
}


/* Select a GPIO pin PC1 to ADC input */
void initPC1(void) {
	// (Reset state: 00)
	GPIOC->MODER &= ~(GPIO_MODER_MODER1_Msk);

	// (Analog function: 11)
	GPIOC->MODER |= (GPIO_MODER_MODER1_Msk);
	
	// Configure Push/Pull Output type for PC1	(00)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_1);
	
	// Configure low speed for PC1	(00)
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR1_Msk);
	
	// Configure no pull-up/down resistors for PC1	(00)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1_Msk);
}


/* Configure ADC */
void initADC(void) {
	// 8-bit resolution
	ADC1->CFGR1 |= (0x2 << ADC_CFGR1_RES_Pos);
	
	// Continuous conversion mode
	ADC1->CFGR1 |= (ADC_CFGR1_CONT_Msk);
	
	// Hardware triggers disabled (software trigger only)
	ADC1->CFGR1 &= ~(ADC_CFGR1_EXTEN_Msk);
	
	// Set channel 11 for PC1
	ADC1->CHSELR |= (ADC_CHSELR_CHSEL11);
}


/* Enable ADC conversion and self-calibrate */
void calADC(void) {
	// Calibration (Peripheral Ref: A.7.1)
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN by setting ADDIS*/
	/* (3) Clear DMAEN */
	/* (4) Launch the calibration by setting ADCAL */
	/* (5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */ {
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0) {
		/* For robust implementation, add here time-out management */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */ {
		/* For robust implementation, add here time-out management */
	}
	
	// Enable (Peripheral Ref: A.7.2)
	/* (1) Ensure that ADRDY = 0 */
	/* (2) Clear ADRDY */
	/* (3) Enable the ADC */
	/* (4) Wait until ADC ready */
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */ {
		ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */ {
		/* For robust implementation, add here time-out management */
	}
	
	// Start ADC
	ADC1->CR |= (ADC_CR_ADSTART);
}


/* Part 1 LED checkoff */
void thresLED(void) {
	// store Data reg in local variable
	uint16_t val = ADC1->DR;
	
	if (val < THRESHOLD_0) {
		GPIOC->ODR &= ~(GPIO_ODR_6);
		GPIOC->ODR &= ~(GPIO_ODR_7);
		GPIOC->ODR &= ~(GPIO_ODR_8);
		GPIOC->ODR &= ~(GPIO_ODR_9);
	} else if ((THRESHOLD_0 < val) & (val < THRESHOLD_1)) {
		GPIOC->ODR |= (GPIO_ODR_6);
		GPIOC->ODR &= ~(GPIO_ODR_7);
		GPIOC->ODR &= ~(GPIO_ODR_8);
		GPIOC->ODR &= ~(GPIO_ODR_9);
	} else if ((THRESHOLD_1 < val) & (val < THRESHOLD_2)) {
		GPIOC->ODR |= (GPIO_ODR_6);
		GPIOC->ODR |= (GPIO_ODR_7);
		GPIOC->ODR &= ~(GPIO_ODR_8);
		GPIOC->ODR &= ~(GPIO_ODR_9);
	} else if ((THRESHOLD_2 < val) & (val < THRESHOLD_3)) {
		GPIOC->ODR |= (GPIO_ODR_6);
		GPIOC->ODR |= (GPIO_ODR_7);
		GPIOC->ODR |= (GPIO_ODR_8);
		GPIOC->ODR &= ~(GPIO_ODR_9);
	} else if (THRESHOLD_3 < val) {
		GPIOC->ODR |= (GPIO_ODR_6);
		GPIOC->ODR |= (GPIO_ODR_7);
		GPIOC->ODR |= (GPIO_ODR_8);
		GPIOC->ODR |= (GPIO_ODR_9);
	}
}


/* Select a GPIO pin PA4 to DAC input */
void initPA4(void) {
	// (Reset state: 00)
	GPIOA->MODER &= ~(GPIO_MODER_MODER4_Msk);

	// (Analog function: 11)
	GPIOA->MODER |= (GPIO_MODER_MODER4_Msk);
	
	// Configure Push/Pull Output type for PA4	(00)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
	
	// Configure low speed for PA4	(00)
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR4_Msk);
	
	// Configure no pull-up/down resistors for PA4	(00)
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4_Msk);
}


/* Configure DAC */
void initDAC(void) {
	// Set software trigger: (111)
	DAC1->CR |= (DAC_CR_TSEL1_Msk);
	
	// DAC enable
	DAC1->CR |= (DAC_CR_EN1_Msk);
}


// global variable index
uint8_t index = 0;

/* DAC wave table */
void waveDAC(void) {
	// Send wave to data register DHR8R1
	//DAC1->DHR8R1 = sine_table[index];
	//DAC1->DHR8R1 = triangle_table[index];
	DAC1->DHR8R1 = sawtooth_table[index];
	
	// Track the index
	index++;
	if (index == 31) {
		index = 0;
	}
	
	// Insert delay 1ms
	HAL_Delay(1);
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
