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
#include "delay.h"
#include "MFRC522.h"
#include <string.h>
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
u_char status, cardstr[MAX_LEN+1];
u_char card_data[17];
uint32_t delay_val = 1000; //ms

// To keep track of the phases of processing that we pass through
uint16_t result = 0;

// The type of card inserted
u_char TagType[2];

u_char UID[5];

// Select Acknowledge https://www.nxp.com/docs/en/application-note/AN10833.pdf
u_char SAK = 0;

// To read the entire 1KB of the card for inspection
u_char CardContent[64][17];
u_char CardReadStatus;

// a private key to scramble data writing/reading to/from RFID card:
u_char Mx1[7][5]={{0x12,0x45,0xF2,0xA8},{0xB2,0x6C,0x39,0x83},{0x55,0xE5,0xDA,0x18},
				{0x1F,0x09,0xCA,0x75},{0x99,0xA2,0x50,0xEC},{0x2C,0x88,0x7F,0x3D}};

u_char SectorKey[7];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ProcessTag() {
#if 0
	SectorKey[0] = ((Mx1[0][0]) ^ (UID[0])) + ((Mx1[0][1]) ^ (UID[1]))
			+ ((Mx1[0][2]) ^ (UID[2])) + ((Mx1[0][3]) ^ (UID[3])); // 0x11; //KeyA[0]
	SectorKey[1] = ((Mx1[1][0]) ^ (UID[0])) + ((Mx1[1][1]) ^ (UID[1]))
			+ ((Mx1[1][2]) ^ (UID[2])) + ((Mx1[1][3]) ^ (UID[3])); // 0x11; //KeyA[0]
	SectorKey[2] = ((Mx1[2][0]) ^ (UID[0])) + ((Mx1[2][1]) ^ (UID[1]))
			+ ((Mx1[2][2]) ^ (UID[2])) + ((Mx1[2][3]) ^ (UID[3])); // 0x11; //KeyA[0]
	SectorKey[3] = ((Mx1[3][0]) ^ (UID[0])) + ((Mx1[3][1]) ^ (UID[1]))
			+ ((Mx1[3][2]) ^ (UID[2])) + ((Mx1[3][3]) ^ (UID[3])); // 0x11; //KeyA[0]
	SectorKey[4] = ((Mx1[4][0]) ^ (UID[0])) + ((Mx1[4][1]) ^ (UID[1]))
			+ ((Mx1[4][2]) ^ (UID[2])) + ((Mx1[4][3]) ^ (UID[3])); // 0x11; //KeyA[0]
	SectorKey[5] = ((Mx1[5][0]) ^ (UID[0])) + ((Mx1[5][1]) ^ (UID[1]))
			+ ((Mx1[5][2]) ^ (UID[2])) + ((Mx1[5][3]) ^ (UID[3])); // 0x11; //KeyA[0]

	DWT_Delay_ms(1);
	status = MFRC522_Auth(0x60, 3, SectorKey, cardstr);

	if (status == MI_OK) {
		result++;

		if (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == 0) {
			// Clean-Up the Card:
			card_data[0] = 0xFF;
			card_data[1] = 0xFF;
			card_data[2] = 0xFF;
			card_data[3] = 0xFF;
			card_data[4] = 0xFF;
			card_data[5] = 0xFF;
			card_data[6] = 0xFF; //Access_bits[6]
			card_data[7] = 0x07; //Access_bits[7]
			card_data[8] = 0x80; //Access_bits[8]
			card_data[9] = 0x88; //user_byte[9]
			card_data[10] = 0x88; //user_byte[10]
			card_data[11] = 0x88; //user_byte[11]
			card_data[12] = 0x88; //user_byte[12]
			card_data[13] = 0x88; //user_byte[13]
			card_data[14] = 0x88; //user_byte[14]
			card_data[15] = 0x88; //user_byte[15]
			DWT_Delay_ms(1);
			status = MFRC522_Write(3, card_data);
			if (status == MI_OK) {
				result++;
				delay_val = 2000;
			}
		}
	} else {
		for (int i = 0; i < 16; i++) {
			cardstr[i] = 0;
		}
		status = 0;
		// Find cards
		DWT_Delay_ms(1);
		status = MFRC522_Request(PICC_REQIDL, cardstr);
		DWT_Delay_ms(1);
		status = MFRC522_Anticoll(cardstr);
		DWT_Delay_ms(1);
		status = MFRC522_SelectTag(cardstr);
		SectorKey[0] = 0xFF;
		SectorKey[1] = 0xFF;
		SectorKey[2] = 0xFF;
		SectorKey[3] = 0xFF;
		SectorKey[4] = 0xFF;
		SectorKey[5] = 0xFF;
		DWT_Delay_ms(1);
		status = MFRC522_Auth(0x60, 3, SectorKey, cardstr);
		if (status == MI_OK) {
			if (HAL_GPIO_ReadPin(Key1_GPIO_Port, Key1_Pin) == 0) {
				card_data[0] = ((Mx1[0][0]) ^ (UID[0]))
						+ ((Mx1[0][1]) ^ (UID[1])) + ((Mx1[0][2]) ^ (UID[2]))
						+ ((Mx1[0][3]) ^ (UID[3])); // 0x11; //KeyA[0]
				card_data[1] = ((Mx1[1][0]) ^ (UID[0]))
						+ ((Mx1[1][1]) ^ (UID[1])) + ((Mx1[1][2]) ^ (UID[2]))
						+ ((Mx1[1][3]) ^ (UID[3])); // 0x11; //KeyA[0]
				card_data[2] = ((Mx1[2][0]) ^ (UID[0]))
						+ ((Mx1[2][1]) ^ (UID[1])) + ((Mx1[2][2]) ^ (UID[2]))
						+ ((Mx1[2][3]) ^ (UID[3])); // 0x11; //KeyA[0]
				card_data[3] = ((Mx1[3][0]) ^ (UID[0]))
						+ ((Mx1[3][1]) ^ (UID[1])) + ((Mx1[3][2]) ^ (UID[2]))
						+ ((Mx1[3][3]) ^ (UID[3])); // 0x11; //KeyA[0]
				card_data[4] = ((Mx1[4][0]) ^ (UID[0]))
						+ ((Mx1[4][1]) ^ (UID[1])) + ((Mx1[4][2]) ^ (UID[2]))
						+ ((Mx1[4][3]) ^ (UID[3])); // 0x11; //KeyA[0]
				card_data[5] = ((Mx1[5][0]) ^ (UID[0]))
						+ ((Mx1[5][1]) ^ (UID[1])) + ((Mx1[5][2]) ^ (UID[2]))
						+ ((Mx1[5][3]) ^ (UID[3])); // 0x11; //KeyA[0]
				card_data[6] = 0xFF; //Access_bits[6]
				card_data[7] = 0x07; //Access_bits[7]
				card_data[8] = 0x80; //Access_bits[8]
				card_data[9] = 0x88; //user_byte[9]
				card_data[10] = 0x88; //user_byte[10]
				card_data[11] = 0x88; //user_byte[11]
				card_data[12] = 0x88; //user_byte[12]
				card_data[13] = 0x88; //user_byte[13]
				card_data[14] = 0x88; //user_byte[14]
				card_data[15] = 0x88; //user_byte[15]
				DWT_Delay_ms(1);
				status = MFRC522_Write(3, card_data);
				if (status == MI_OK) {
					result++;
					sprintf(str3, "Card Set!");
					delay_val = 2000;
				}
			} else {
				sprintf(str4, "New Card!");
			}
		} else if (status != MI_OK) {
			sprintf(str3, "Auth. Error");
		}
	}
	DWT_Delay_ms(1);
	MFRC522_StopCrypto1();
#endif
	// Prepare the sector key to authenticate before reading/writing the block.
	// This key is stored as the last 6 bytes of the trailer blocks in the sector.
	// So it does not have to be the same for all sector.
	// In a real application, this key can be set once by the user via PIN/password
	// and re-derived from the password/PIN when the card is accessed.
	// The default key is set to 6's 0xFF.
	// WARNING: Once the key is changed, make sure to keep it somewhere so that we can later
	// read the sector!
	for(int i = 0; i < 6; i++) {
		SectorKey[i] = 0xFF;
	}

	// Read the whole 1KB from the card by going through each blocks, authenticate then read.
	// Adapted from https://lastminuteengineers.com/how-rfid-works-rc522-arduino-tutorial/
	for(u_char blockNum = 0; blockNum < 64; blockNum++) {
		status = MFRC522_Auth(0x60, blockNum, SectorKey, UID);

		if (status == MI_OK) {
			result++;

			HAL_Delay(1);
			CardReadStatus = MFRC522_Read(blockNum, CardContent[blockNum]);
		}

		HAL_Delay(10);
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
  // From the datasheet: Bring RST pin HIGH (bring it LOW will power down the module) to reset the module
  HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // Now we can initialize the module
  MFRC522_Init();

  // Put a breakpoint here to see the (module?) version
  status = Read_MFRC522(VersionReg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 1 // During testing: stop for inspection once succeed
	if (result > 0) {
		HAL_Delay(1000);
		continue;
	}
#endif

	// status = 0;
	// Find cards
	status = MFRC522_Request(PICC_REQIDL, TagType);
	if(status == MI_OK) {
		result++;

		HAL_Delay(1);
		status = MFRC522_Anticoll(cardstr);

		if(status == MI_OK) {
			result++;
			memcpy(UID, cardstr, 5);

			HAL_Delay(1);
			SAK = MFRC522_SelectTag(cardstr);

			if (SAK > 0) {
				result++;
				ProcessTag();
			}
		}

		MFRC522_Halt();
	} else {
		// Waiting for card
	}
	HAL_Delay(1000);
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  HAL_GPIO_WritePin(GPIOB, RC522_CS_Pin|RC522_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RC522_CS_Pin RC522_RST_Pin */
  GPIO_InitStruct.Pin = RC522_CS_Pin|RC522_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
