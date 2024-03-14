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
void readReg(void);
void initGyro(void);
void senseGyro(void);

void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Enable GPIOC, GPIOB, and I2C Clock in RCC (SCL)
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	
	// initialize GPIO modes and I2C peripherals
	initPeriph();
	
	// initialize LEDs
	initLEDs();
	
	// Part 1
	//readReg();
	
	// Part 2 Initialization
	initGyro();
	
  while (1)
  {
		// Part 2 While-loop
    senseGyro();
  }
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
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_11); // (Open-drain: 1)
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos); // select I2C2_SDA as alternative function
	
	// Set PB13 - AF5
	GPIOB->MODER |= (GPIO_MODER_MODER13_1); // (Alternate function: 10)
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_13); // (Open-drain: 1)
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
	
	// Peripheral enable
	I2C2->CR1 |= I2C_CR1_PE; 
	
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
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
	GPIOC->BSRR = GPIO_BSRR_BR_6;	// Set PC6 low
	GPIOC->BSRR = GPIO_BSRR_BR_7; // Set PC7 low
	GPIOC->BSRR = GPIO_BSRR_BR_8;	// Set PC8 low
	GPIOC->BSRR = GPIO_BSRR_BR_9; // Set PC9 low
	//	*/
}


/* Part 1 */
void readReg(void) {
	// Congigure CR2 register
	//I2C2->CR2 |= (1 << 16) | (0x69 << 1); // Alternative approach
	//I2C2->CR2 &= ~(1 << 10);              // Set the RD_WRN to write operation
	//I2C2->CR2 |= (1 << 13);               // Set START bit
	
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001, left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 1
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        // Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit
	
	
	// Wait until TXIS or NACKF flags are set
	while(1) {
		if ((I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF)) {
			break;
		}
	}
	
	// Turn on ORANGE LED for NACKF debugging
	if (I2C2->ISR & I2C_ISR_NACKF) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	}
	
	// Write the address of the "WHO_AM_I" register into TXDR
	I2C2->TXDR = 0x0F;

	// /*
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			// Turn on RED LED for TC debugging
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			
			break;
		}	
	}
	
	// /*
	// Reload CR2 from before, but change to RD_WRN to read
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001; left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 1
	I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         // Set the RD_WRN to read operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit

	// Wait until RXNE or NACKF flags are set
	while(1) {
		if ((I2C2->ISR & I2C_ISR_RXNE) | (I2C2->ISR & I2C_ISR_NACKF)) {
			break;
		}
	}
	
	// Turn on GREEN LED for RXNE debugging
	if (I2C2->ISR & I2C_ISR_RXNE) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);	
	}
	
	// /*
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			// Turn on/off RED LED for TC debugging
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			
			break;
		}	
	} 
	// /*
	// check if RXDR matches 0xD3
	if (I2C2->RXDR == 0xD3) {
		// Turn on/off BLUE LED for TC debugging
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	}
	
	// Set the stop bit in CR2
	I2C2->CR2 |= (I2C_CR2_STOP); // */
} 


/* Part 2 Initialization */
void initGyro(void) {
	/* Initializing the Gyroscope */
	// Enable the X and Y sensing axes in the CTRL_REG1 register
	//I2C2->CR2 |= (2 << 16) | (0x69 << 1); // Alternative approach
	//I2C2->CR2 &= ~(1 << 10);              // Set the RD_WRN to write operation
	//I2C2->CR2 |= (1 << 13);               // Set START bit
	
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001; left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 2
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        // Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit
	
	// Wait until TXIS or NACKF flags are set (1)
	while(1) {
		if ((I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF)) {
			break;
		}
	}
	
	// Write the address of the "CTRL_REG1" register into TXDR
	I2C2->TXDR = 0x20;
	
	// Wait again until TXIS or NACKF flags are set (2)
	while(1) {
		if ((I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF)) {
			break;
		}
	}
	
	// Write the address of the "normal or sleep mode"
	I2C2->TXDR = 0x0B;
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {			
			break;
		}	
	}
	
	// Set the STOP bit in CR2 to release the bus
	I2C2->CR2 |= (I2C_CR2_STOP);
}

/* Global Variables for x-axis */
uint8_t x_byte1;
uint8_t x_byte2;
int16_t x;
int16_t x_dir = 0;

/* Global Variables for y-axis */
uint8_t y_byte1;
uint8_t y_byte2;
int16_t y;
int16_t y_dir = 0;

/* Part 2 While-loop */
void senseGyro(void) {
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	/* Enable the X sensing axis in the CTRL_REG1 register */
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001; left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 2
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        // Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit
	
	// Wait until TXIS or NACKF flags are set
	while(1) {
		// Continue if TXIS flag is set
		if ((I2C2->ISR & I2C_ISR_TXIS)) {
			I2C2->TXDR = 0xA8;
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // debugging
		}
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}	
	}
	
	// Reload CR2 from before, but change to RD_WRN to read
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001; left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 1
	I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         // Set the RD_WRN to read operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit
	
	// Wait until RXNE or NACKF flags are set (1)
	while(1) {
		// Continue if RXNE flag is set
		if ((I2C2->ISR & I2C_ISR_RXNE)) {
			x_byte1 = I2C2->RXDR;
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // debugging
		}
	}
	
	
	// Wait again until RXNE or NACKF flags are set (2)
	while(1) {
		// Continue if RXNE flag is set
		if ((I2C2->ISR & I2C_ISR_RXNE)) {
			x_byte2 = I2C2->RXDR;
			break;
		}
		
		// Light GREEN LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // debugging
		}
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}	
	}
	
	// Set the STOP bit in CR2 to release the bus
	I2C2->CR2 |= (I2C_CR2_STOP);
	
	// store x_byte1 and x_byte2 into x
	x = (x_byte2 << 8) | (x_byte1 << 0);
	
	// x-axis direction
	x_dir += x;
	HAL_Delay(100); // 100 ms delay
	
	/********************************************/
	
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	/* Enable the Y sensing axis in the CTRL_REG1 register */
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001; left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 2
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        // Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit
	
	// Wait until TXIS or NACKF flags are set
	while(1) {
		// Continue if TXIS flag is set
		if ((I2C2->ISR & I2C_ISR_TXIS)) {
			I2C2->TXDR = 0xAA;
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // debugging
		}
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}	
	}
	
	// Reload CR2 from before, but change to RD_WRN to read
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001; left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 1
	I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         // Set the RD_WRN to read operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit
	
	// Wait until RXNE or NACKF flags are set (1)
	while(1) {
		// Continue if RXNE flag is set
		if ((I2C2->ISR & I2C_ISR_RXNE)) {
			y_byte1 = I2C2->RXDR;
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // debugging
		}
	}
	
	
	// Wait again until RXNE or NACKF flags are set (2)
	while(1) {
		// Continue if RXNE flag is set
		if ((I2C2->ISR & I2C_ISR_RXNE)) {
			y_byte2 = I2C2->RXDR;
			break;
		}
		
		// Light GREEN LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // debugging
		}
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}	
	}
	
	// Set the STOP bit in CR2 to release the bus
	I2C2->CR2 |= (I2C_CR2_STOP);
	
	// store y_byte1 and y_byte2 into y
	y = (y_byte2 << 8) | (y_byte1 << 0);
	
	// y-axis direction
	y_dir += y;
	// HAL_Delay(100); // 100 ms delay
	
	
	/* Gyroscope Testing */
	// X-AXIS controls the ORANGE and GREEN LEDS
	if (x_dir < 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	}
	
	// Y-AXIS controls the RED and BLUE LEDS
	if (x_dir < 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	}
	
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
