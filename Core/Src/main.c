/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORA_NSS_GPIO_Port    GPIOC
#define LORA_NSS_Pin          GPIO_PIN_4

#define LORA_RESET_GPIO_Port  GPIOC
#define LORA_RESET_Pin        GPIO_PIN_5

#define LORA_DIO0_GPIO_Port   GPIOB
#define LORA_DIO0_Pin         GPIO_PIN_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//LoRa handling functions
void LoRa_Select(void) {
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); // CS LOW
}
void LoRa_Unselect(void) {
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); // CS HIGH
}

// LoRa reset function
void LoRa_Reset(void) {
    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

// Funkcja SPI do komunikacji z LoRa – jeden bajt w obie strony
uint8_t LoRa_SPI_ReadWrite(uint8_t data) {
    uint8_t received;
    HAL_SPI_TransmitReceive(&hspi1, &data, &received, 1, HAL_MAX_DELAY);
    return received;
}

// Zapis do rejestru SX1278
void LoRa_WriteRegister(uint8_t reg, uint8_t value) {
    LoRa_Select();
    LoRa_SPI_ReadWrite(reg | 0x80); // Bit 7 = 1 -> zapis
    LoRa_SPI_ReadWrite(value);
    LoRa_Unselect();
}

// Odczyt z rejestru SX1278
uint8_t LoRa_ReadRegister(uint8_t reg) {
    LoRa_Select();
    LoRa_SPI_ReadWrite(reg & 0x7F); // Bit 7 = 0 -> odczyt
    uint8_t value = LoRa_SPI_ReadWrite(0x00);
    LoRa_Unselect();
    return value;
}


void LoRa_Init(void) {
    LoRa_Reset();

    uint8_t version = LoRa_ReadRegister(0x42);
    if (version != 0x12) {
        // Błąd - moduł nie odpowiada poprawnie
        while(1);
    }

    // Ustaw tryb sleep aby konfigurować
    LoRa_WriteRegister(0x01, 0x00); // RegOpMode = sleep, LoRa mode enabled (bit7=1)

    // Ustaw częstotliwość (np. 433 MHz)
    // Freq = 433000000 Hz, Fstep = 61.035 Hz, RegFrMsb = freq / Fstep >> 16 itd.
    uint32_t frf = (uint32_t)((433000000.0) / 61.03515625);
    LoRa_WriteRegister(0x06, (frf >> 16) & 0xFF);
    LoRa_WriteRegister(0x07, (frf >> 8) & 0xFF);
    LoRa_WriteRegister(0x08, frf & 0xFF);

    // Konfiguracja modułu
    LoRa_WriteRegister(0x09, 0xFF); // RegPaConfig max power
    LoRa_WriteRegister(0x0A, 0x23); // RegOcp (prąd ochrony)
    LoRa_WriteRegister(0x0B, 0x07); // LNA gain max

    // Ustaw parametry modulatora (Bandwidth, CodingRate, SpreadingFactor)
    LoRa_WriteRegister(0x1D, 0x72); // RegModemConfig1 (BW=125kHz, CR=4/5)
    LoRa_WriteRegister(0x1E, 0x74); // RegModemConfig2 (SF=7, CRC enabled)

    LoRa_WriteRegister(0x26, 0x04); // RegModemConfig3 (LowDataRateOptimize off, AGC auto on)

    // Ustaw adres FIFO do Tx i Rx
    LoRa_WriteRegister(0x0F, 0x00); // RegFifoTxBaseAddr
    LoRa_WriteRegister(0x0E, 0x00); // RegFifoRxBaseAddr

    // Ustaw tryb RX Continuous
    LoRa_WriteRegister(0x01, 0x85); // RegOpMode = LoRa mode, RX continuous

    // Wyczyść przerwania
    LoRa_WriteRegister(0x12, 0xFF);
}

void LoRa_SendText(const char *text) {
    uint8_t len = strlen(text);
    if (len > 255) len = 255; // max pakiet

    // Ustaw tryb standby
    LoRa_WriteRegister(0x01, 0x81); // LoRa mode, standby

    // Ustaw wskaźnik FIFO na start transmisji
    LoRa_WriteRegister(0x0D, 0x00); // RegFifoAddrPtr

    // Zapisz bajty do FIFO
    for (uint8_t i = 0; i < len; i++) {
        LoRa_WriteRegister(0x00, text[i]);
    }

    // Ustaw długość pakietu
    LoRa_WriteRegister(0x22, len);

    // Włącz TX
    LoRa_WriteRegister(0x01, 0x83); // LoRa mode, TX

    // Czekaj na przerwanie TX done w DIO0
    while (HAL_GPIO_ReadPin(LORA_DIO0_GPIO_Port, LORA_DIO0_Pin) == GPIO_PIN_RESET);

    // Wyczyść flagi przerwań TX done
    LoRa_WriteRegister(0x12, 0x08);

    // Powrót do RX continuous
    LoRa_WriteRegister(0x01, 0x85);
}

//// Prosty test odczytu wersji SX1278 (powinien zwrócić 0x12) -> zmiana na spi zamiast huart
//void LoRa_TestVersion(void) {
//    uint8_t version = LoRa_ReadRegister(0x42); // RegVersion
//    char msg[32];
//    sprintf(msg, "SX1278 Ver: 0x%02X\r\n", version);
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//}


int LoRa_ReceiveText(char *buffer, int max_len) {
    uint8_t irqFlags = LoRa_ReadRegister(0x12);

    if (irqFlags & 0x40) { // Rx done
        // Sprawdź CRC ok
        if (irqFlags & 0x20) {
            // Błąd CRC
            LoRa_WriteRegister(0x12, 0xFF); // wyczyść przerwania
            return 0;
        }

        // Odczytaj długość pakietu
        uint8_t len = LoRa_ReadRegister(0x13);

        // Ustaw adres FIFO na ostatni pakiet
        uint8_t fifoRxCurrentAddr = LoRa_ReadRegister(0x10);
        LoRa_WriteRegister(0x0D, fifoRxCurrentAddr);

        if (len > max_len - 1) len = max_len - 1;

        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = LoRa_ReadRegister(0x00);
        }
        buffer[len] = 0;

        // Wyczyść przerwania
        LoRa_WriteRegister(0x12, 0xFF);

        return len;
    }
    return 0;
}

volatile uint8_t flag_error = 0;
volatile uint8_t flag_rx_done = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI_interrupt_Pin) { // przerwanie od SX1278
        uint8_t irqFlags = LoRa_ReadRegister(0x12);

        if (irqFlags & 0x20) {  // CRC error
            flag_error = 1;
        //    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET); // zapal diodę
            // wysyłamy komunikat radiowy informujący o błędzie (jednorazowo)
            LoRa_SendText("ERROR: CRC");
        } else if (irqFlags & 0x40) { // Rx done
            flag_rx_done = 1;
        //    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET); // zgaś diodę
            // opcjonalnie potwierdzenie odebrania
            LoRa_SendText("RX_DONE");
        }

        LoRa_WriteRegister(0x12, 0xFF); // czyścimy przerwania
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	    if (flag_error) {
	            flag_error = 0;
	            // obsługa błędu — np. logowanie, powiadomienie na konsoli UART
	            UART_Printf("Błąd CRC w pakiecie LoRa!\r\n");
	            // ewentualnie reset lub inne działania
	        }
	        if (flag_rx_done) {
	            flag_rx_done = 0;
	            // obsługa odebranych danych
	            char buf[256];
	            int len = LoRa_ReceiveText(buf, sizeof(buf));
	            if (len > 0) {
	                UART_Printf("Odebrano: %s\r\n", buf);
	            }
	        }

	  static uint32_t last_send = 0;
	         if (HAL_GetTick() - last_send > 10000) {
	             LoRa_SendText("MENU:\r\n1.Status\r\n2.Config\r\n3.Reset\r\nWpisz nr opcji:\r\n");
	             last_send = HAL_GetTick();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Pin|SPI_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_Reset_Pin */
  GPIO_InitStruct.Pin = SPI_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_interrupt_Pin */
  GPIO_InitStruct.Pin = SPI_interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_interrupt_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
