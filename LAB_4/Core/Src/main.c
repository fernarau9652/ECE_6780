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

// Functions utilized in main
void initLEDs(void);
void initUSART(void);
void charTransmit(char c);
void stringTransmit(const char *text);
void initBlockTransmit(void);
void receiveChar(void);

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Enable GPIOC and USART3 Clock in RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// initialize LEDs
	initLEDs();
	
	// initialize USART
	initUSART();
	
	// initial Block transmit for a string
	initBlockTransmit();
	
	/* Uncomment after check-off 1 */  /*
	// NVIC setup for USART3 handler
	NVIC_EnableIRQ(USART3_4_IRQn);
	
	// Set the priority for the interrupt to 1 (high-priority)
	NVIC_SetPriority(USART3_4_IRQn, 1);	// */
  
  while (1)
  {
		receiveChar();
		
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



// Initialize LEDs function
void initLEDs(void)
{	
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
	GPIOC->BSRR = GPIO_BSRR_BS_6;	// Set PC6 high
	GPIOC->BSRR = GPIO_BSRR_BS_7; // Set PC7 high
	GPIOC->BSRR = GPIO_BSRR_BS_8;	// Set PC8 high
	GPIOC->BSRR = GPIO_BSRR_BS_9; // Set PC9 high
	//	*/
}


// Initialize USART function
void initUSART(void)
{
	// Set Moder 4 and Moder 5 to alternate function mode
	GPIOC->MODER &= ~(GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER5_Msk);	// reset bits
	GPIOC->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);	// set to AF mode (10)
	
	// Set PC4 and PC5 to AF1
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL4_Pos); 
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL5_Pos);
	
	// Configure the USART BRR
	uint32_t baud_rate = 115200;
	uint32_t f_clk = HAL_RCC_GetHCLKFreq();
	USART3->BRR = f_clk/baud_rate;
	
	// Enable the transmitter and receiver
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
	
	// Enable the USART peripheral
	USART3->CR1 |= USART_CR1_UE;
}


void initBlockTransmit(void)
{
	/* Transmit a string test */ // /*
	const char *string = "Toggle LEDs: Press R, G, B, or O keys; ";
	//const char *string = "ABCDEFG NOPQRST abcdefg nopqrst ";
	stringTransmit(string);	// */
}


// Transmit 1 character
void charTransmit(char c)
{
	while ((USART3->ISR & USART_ISR_TXE) == 0){
	}
	USART3->TDR = c;
}	

// initial count to avoid massive putty output files
uint32_t count = 0;

// Transmit a string of characters
void stringTransmit(const char *text)
{
	while(*text != '\0'){
		HAL_Delay(5);
		charTransmit(*text);
		text++;
	}
}


void receiveChar(void)
{
	char ch;
	
	if((USART3->ISR & USART_ISR_RXNE)) {
		ch = USART3->RDR; // & (0xFF); // Bottom 8 bits
		
	/* Checks the character and toggles an LED or outputs error message to console */ // /*
		switch(ch){
			case 'r':
			case 'R':
				GPIOC->ODR ^= GPIO_ODR_6;
				break;
			case 'b':
			case 'B':
				GPIOC->ODR ^= GPIO_ODR_7;
				break;
			case 'o':
			case 'O':
				GPIOC->ODR ^= GPIO_ODR_8;
				break;
			case 'g':
			case 'G':
				GPIOC->ODR ^= GPIO_ODR_9;
				break;
			default:
				stringTransmit("Error: Unsupported Character; ");
				break;
		}	// */
	}
}



// NVIC USART3 handler
void USART3_4_IRQHandler(void)
{


}



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
