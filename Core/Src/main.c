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
#include "core_cm4.h"  // lub core_cm3.h/core_cm7.h zależnie od MCU
#include <stdbool.h>
#include <string.h>
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

//USER HELP VARIABLES and debugging functions
const bool BOOL_Log_1 = true;
const bool BOOL_Log_0 = false;

bool my_bit = false;
void ToggleBit(void) {
    my_bit = !my_bit;
}


int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar(ptr[i]); // Wysyłanie znaków do kanału 0 SWV
    }
    return len;
}

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
    HAL_Delay(200);

    uint8_t version = LoRa_ReadRegister(0x42);
    if (version != 0x12) {

    	// Błąd - moduł nie odpowiada poprawnie
    	 for (int i = 0; i < 5; i++) {
    		        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // zapal diodę
    	            HAL_Delay(200);
    	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // zgaś diodę
    	            HAL_Delay(200);
    	 }
    	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        while(1);
    }

    // Sleep + LoRa mode enabled (bit7=1)
    LoRa_WriteRegister(0x01, 0x80);

    // Dla 433.125 MHz
    uint32_t frf = (uint32_t)(433125000.0 / 61.03515625); // = 7094272 ≈ 0x6C40A0

    LoRa_WriteRegister(0x06, (frf >> 16) & 0xFF); // 0x6C
    LoRa_WriteRegister(0x07, (frf >> 8) & 0xFF);  // 0x40
    LoRa_WriteRegister(0x08, frf & 0xFF);         // 0xA0

    // PA config - max power
    LoRa_WriteRegister(0x09, 0x4F);

    // Ochrona prądu
    LoRa_WriteRegister(0x0A, 0x23);

    // LNA gain max
    LoRa_WriteRegister(0x0B, 0x07);

    // Modem config
    LoRa_WriteRegister(0x1D, 0x72); // ModemConfig1: BW=125, CR=4/5
    LoRa_WriteRegister(0x1E, 0x94); // ModemConfig2: SF=9, CRC on
    LoRa_WriteRegister(0x26, 0x04); // ModemConfig3: LowDataRateOptimize = off

    printf("ModemConfig1: 0x%02X\n", LoRa_ReadRegister(0x1D));
    printf("ModemConfig2: 0x%02X\n", LoRa_ReadRegister(0x1E));
    printf("ModemConfig3: 0x%02X\n", LoRa_ReadRegister(0x26));


    // FIFO base address: Tx i Rx poprawnie!
    LoRa_WriteRegister(0x0E, 0x00); // RegFifoTxBaseAddr
    LoRa_WriteRegister(0x0F, 0x00); // RegFifoRxBaseAddr

    // Tryb RX continuous
    LoRa_WriteRegister(0x01, 0x85); // LoRa mode, RX continuous

    // Wyczyść IRQ
    LoRa_WriteRegister(0x12, 0xFF);
}

void LoRa_SendText(const char *text) {

	// 1. Wyczyść flagi IRQ
    LoRa_WriteRegister(0x12, 0xFF);

    // 1. Ustaw FIFO base addr na 0
    LoRa_WriteRegister(0x0E, 0x00);
    LoRa_WriteRegister(0x0D, 0x00);

    // 3. Załaduj dane do FIFO
    LoRa_Select();
    LoRa_SPI_ReadWrite(0x80); // FIFO address with write bit


    printf("parametry przed petla for  do wpisywania do fifo\n");
       LoRa_DumpRegisters();

    for (uint8_t i = 0; i < strlen(text); i++) {
        LoRa_SPI_ReadWrite(text[i]);
    }
    LoRa_Unselect();

    // 4. Ustaw długość wiadomości
    LoRa_WriteRegister(0x22, strlen(text)); // RegPayloadLength

    // 5. Tryb TX (LoRa mode, FSK off, TX on)
    LoRa_WriteRegister(0x01, 0x83); // RegOpMode: LoRa, TX mode

    printf("OpMode=0x%02X, IrqFlags=0x%02X, PayloadLen=%d\r\n",
           LoRa_ReadRegister(0x01),
           LoRa_ReadRegister(0x12),
           LoRa_ReadRegister(0x22));

    printf("parametry przed petla while - sprawdzeniem czy tx done\n");
    LoRa_DumpRegisters();


    // 6. Czekaj na zakończenie transmisji
    while ((LoRa_ReadRegister(0x12) & 0x08) == 0) // TX_DONE = 0x08
    {
    	printf("Still waiting... Irq=0x%02X\r\n", LoRa_ReadRegister(0x12));
    	LoRa_DumpRegisters();
    	HAL_Delay(2000);
    };


    // 🐞 DEBUG: potwierdzenie zakończenia
    printf("TX done: Irq=0x%02X\r\n", LoRa_ReadRegister(0x12));

    for (int i = 0; i < 2; i++) {
       		        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // zapal diodę
       	            HAL_Delay(200);
       	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // zgaś diodę
       	            HAL_Delay(200);
       	 }

    HAL_Delay(400);

    // 7. Wyczyść flagi IRQ
    LoRa_WriteRegister(0x12, 0xFF);

    // 8. Tryb RX (powrót)
    LoRa_WriteRegister(0x01, 0x85); // RX_CONTINUOUS
}

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

void LoRa_ReportEvent(void) {
    uint8_t irqFlags = LoRa_ReadRegister(0x12);   // RegIrqFlags
    LoRa_WriteRegister(0x12, 0xFF);               // Skasuj flagi (1 = clear)

    if (irqFlags & (1 << 6)) {
        LoRa_SendText("CRC ERROR\r\n");
    } else if (irqFlags & (1 << 3)) {
        LoRa_SendText("RX DONE\r\n");
    } else if (irqFlags & (1 << 0)) {
        LoRa_SendText("TX DONE\r\n");
    } else {
        char msg[32];
        sprintf(msg, "IRQ Unknown: 0x%02X\r\n", irqFlags);
        LoRa_SendText(msg);
    }
}

volatile uint8_t flag_error = 0;
volatile uint8_t flag_rx_done = 0;
volatile uint8_t lora_event_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int HUL_TEST = 1;
    if (GPIO_Pin == SPI_interrupt_Pin) { // przerwanie od SX1278
        uint8_t irqFlags = LoRa_ReadRegister(0x12);

        if (irqFlags & 0x20) {  // CRC error
            flag_error = 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // zapal diodę
            // wysyłamy komunikat radiowy informujący o błędzie (jednorazowo)
            LoRa_SendText("ERROR: CRC");
        } else if (irqFlags & 0x40) { // Rx done
            flag_rx_done = 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // zgaś diodę
            // opcjonalnie potwierdzenie odebrania
            LoRa_SendText("RX_DONE");
        }

        LoRa_WriteRegister(0x12, 0xFF); // czyścimy przerwania
    }
}

uint8_t reg_values[15];

void LoRa_DumpRegisters() {
    printf("Version: 0x%02X\n", LoRa_ReadRegister(0x42));
    printf("OpMode: 0x%02X\n", LoRa_ReadRegister(0x01));
    printf("RegFifoAddrPtr: 0x%02X\n", LoRa_ReadRegister(0x0D));
    printf("RegFifoTxBaseAddr: 0x%02X\n", LoRa_ReadRegister(0x0E));
    printf("RegPayloadLength: %d\n", LoRa_ReadRegister(0x22));
    printf("RegModemConfig1: 0x%02X\n", LoRa_ReadRegister(0x1D));
    printf("RegModemConfig2: 0x%02X\n", LoRa_ReadRegister(0x1E));
    printf("RegModemConfig3: 0x%02X\n", LoRa_ReadRegister(0x26));
    printf("Freq (MSB): 0x%02X\n", LoRa_ReadRegister(0x06));
    printf("Freq (MID): 0x%02X\n", LoRa_ReadRegister(0x07));
    printf("Freq (LSB): 0x%02X\n", LoRa_ReadRegister(0x08));
    printf("PaConfig: 0x%02X\n", LoRa_ReadRegister(0x09));
    printf("IrqFlags: 0x%02X\n", LoRa_ReadRegister(0x12));
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

  LoRa_Init();  // konfigurujesz po resecie, nie przed


  LoRa_DumpRegisters();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  LoRa_DumpRegisters();

  while (1)
  {

	  if (lora_event_flag) {
	      lora_event_flag = 0;
	      LoRa_ReportEvent();  // odczytaj rejestr i wyślij informację do PC
	  }

//	  static uint32_t last_send = 0;
//	         if (HAL_GetTick() - last_send > 10000) {
//	             LoRa_SendText("MENU:\r\n1.Status\r\n2.Config\r\n3.Reset\r\nWpisz nr opcji:\r\n");
//	             last_send = HAL_GetTick();

//	             for (int i = 0; i < 3; i++) {
//	                 		        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // zapal diodę
//	                 	            HAL_Delay(200);
//	                 	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // zgaś diodę
//	                 	            HAL_Delay(200);
	                 //	 }

	        // }



	   LoRa_SendText("MENU:\n");



	             HAL_Delay(2000);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Pin|SPI_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D_callback_orange_GPIO_Port, D_callback_orange_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : D_callback_orange_Pin */
  GPIO_InitStruct.Pin = D_callback_orange_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D_callback_orange_GPIO_Port, &GPIO_InitStruct);

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
