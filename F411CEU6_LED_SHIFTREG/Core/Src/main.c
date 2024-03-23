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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t state;
uint16_t state2;
HAL_StatusTypeDef status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Shift out 8 bits to the shift register (LSB will be output A, MSB will be output H as in the datasheet)
// state = [H G F E ... A] in binary
void ShiftOut(uint8_t state) {
	status = HAL_SPI_Transmit(&hspi1, &state, 1, 10);

	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
}

// Various method to test with a single digit 7-segment display

void TestSingleByteShift() {
  state = 0x0f;
  ShiftOut(state);
}

void TestIndividualBit() {
	state = 0x01;
	for(int i = 0; i < 8; i++) {
		ShiftOut(state);
		state = state << 1;
		HAL_Delay(500);
	}
}

// LED edges in the 7-segment display to turn on for the digits
const uint8_t digits[] = {
		0b00111111,  // 0 = {A, B, C, D, E, F}
		0b00000110,  // 1 = {B, C}
		0b01011011,  // 2 = {A, B, D, E, G }
		0b01001111,  // 3 = {A, B, C, D, G }
		0b01100110,  // 4 = {B, C, F, G }
		0b01101101,  // 5 = {A, C, D, F, G }
		0b01111101,  // 6 = {A, C, D, E, F, G }
		0b00000111,  // 7 = {A, B, C }
		0b01111111,  // 8 = {A, B, C, D, E, F, G }
		0b01101111,  // 9 = {A, B, D, E, G }
		// Unfortunately, we cannot do hexadecimal since B is indistinguishable from 8 and D from 0.
};

void TestDigits() {
	for(int d = 0; d < 10; d++) {
		state = digits[d];
		ShiftOut(state);
		HAL_Delay(500);
	}
}

void TestAllByteShift() {
	for(state = 0x00; state <= 0xff; state++) {
	  ShiftOut(state);
	  HAL_Delay(100);
	}
}


// Shift out 2 bytes to the cascaded shift registers (one daisy chained to the another)
// state = [H1 G F E ... A1 | H2 G F E ... A2]
// where Ai, Bi, ..., Hi are the output of the i-th register. The 1st register is the one
// whose data pin 14 (SER) is connected to the MCU's MOSI. The second one received its data
// via the first's pin 9 (QH').
void ShiftOut2(uint16_t state) {
	status = HAL_SPI_Transmit(&hspi1, (uint8_t*)(&state), 2, 10);

	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
}

// Various functions to test daisy chained shift registers with a 4-digit 7-segment display and LED matrix
// Connection assumption:
//   * For 4-digit 7-segment dipslay:
//         1st shift register connects to digit LEDs (A, B, ..., DP)
//         2nd shift register A, B, C, D is connected to digit selection pin (pin 6, 8, 9, 12)
//   * For LED matrix:
//         1st shift register to the row pins
//         2nd shift register to the column pins


// Display the digit d at the digitNum (0..3 instead of 1..4)
void DisplayDigit(uint8_t d, uint8_t digitNum) {
	// We need to turn ONLY the bit for the digit off to ground it.
	// The remaining bits are kept high so that the LEDs are not on for them.
	uint8_t mask = 1 << digitNum;
	mask = ~mask;

	state2 = digits[d];
	state2 = (state2 << 8) | mask;
	ShiftOut2(state2);
}

// Render all digit slot one by one
// For example, 0 is displayed in the first digit slot, then second, then third, then forth.
// Then 1 is displayed in the first, ... etc.
void TestAllDigits() {
	for(uint8_t d = 0; d < 10; d++) {
		for(uint8_t n = 0; n < 4; n++) {
			DisplayDigit(d, n);
			HAL_Delay(1000);
		}
	}
}

// Quickly display 4-digit for "persistence of vision" (switching 1ms between digits)
// Note: d[s] is the most significant digit, d[3] is the least significant one
void Display4DigitNum(uint8_t d[4], uint8_t s) {
	for(uint8_t i = s; i < 4; i++) {
		DisplayDigit(d[i], i);
		HAL_Delay(1);
	}
}

void TestCounting() {
	uint8_t d[4];
	for(int value = 0; value <= 9999; value++) {
		int x = value;
		// Note that the ordering of digits on the 7-segment display is in reverse order
		// with the 1st being the most significant digit and the 4-th the least one.
		d[3] = x % 10; // unit digit
		x = x / 10;
		d[2] = x % 10; // ten digit
		x = x / 10;
		d[1] = x % 10;
		d[0] = x / 10;

		// Find the start digit (the actual most significant digit)
		uint8_t s = 0;
		for(s = 0; s < 3; s++) {
			if (d[s] != 0)
				break;
		}

		// Loop 200 times to keep the digit on display for ~200ms
		for(int c = 0; c < 200; c++) {
			Display4DigitNum(d, s);
		}

		// HAL_Delay(50);
	}
}

// Turn a single pixel on the 8x8 LED matrix
// Row and Column are indexed from 0 to 7 corresponding to 1..8 as in the datasheet of the unit
void DisplaySinglePixel(uint8_t row, uint8_t col) {
	// To turn the row-i column-j pixels on, we need to input HIGH (VCC) to the pin Ri and LOW (GND) to Cj.
	// This way, electricity will flow through the LED at (i, j).

	uint8_t mask = 1 << col;
	mask = ~mask;

	state2 = 1 << row;
	state2 = (state2 << 8) | mask;

	ShiftOut2(state2);
}

void TestPixels() {
	for(uint8_t row = 0; row < 8; row++) {
		for(uint8_t col = 0; col < 8; col++) {
			DisplaySinglePixel(row, col);
			HAL_Delay(200);
		}
	}
}
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // TestSingleByteShift();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// TestDigits();
	// TestAllByteShift();

	// TestAllDigits();
	// TestCounting();

	TestPixels();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LATCH_Pin */
  GPIO_InitStruct.Pin = LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LATCH_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
