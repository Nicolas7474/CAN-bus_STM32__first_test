/* USER CODE BEGIN Header */
/* Nicolas Prata 2026
 * For loopback mode, since Rx and Tx are only internally linked,
 * monitor Tx signal (PB9) at the logic analyser (Rx is still)
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	// Blinking blue led is the "heartbeat" of the system
	GPIOK->ODR ^= GPIO_ODR_OD3; //toggle PK3 (bleu)
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//
CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/*	Once the interrupt occurs, the callback function HAL_CAN_RxFifo0MsgPendingCallback is called.
		In this function, we retrieve the received message header and data, and perform further checks if required.
	*/
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }
  if ((RxHeader.StdId == 0x103))
  {
	 // datacheck = 1; // in main loop
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // Process "Priority" messages here
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error = HAL_CAN_GetError(hcan);

    // 1. Check for ACK Error (Most common)
    if (error & HAL_CAN_ERROR_ACK) {
        // No one is acknowledging your message!
    }

    // 2. Check for Bus-Off
    if (error & HAL_CAN_ERROR_BOF) {
        // The peripheral has shut down due to too many errors.
        // You may need to call HAL_CAN_Stop() and HAL_CAN_Start() to recover.
    }

    // 3. Check for Bit/Stuff Errors
    if (error & (HAL_CAN_ERROR_STF | HAL_CAN_ERROR_FOR)) {
        // Physical layer noise or baud rate mismatch.
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

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x469;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 3;

	TxData[0] = 0x50;
	TxData[1] = 0xAA;
	TxData[2] = 0x86;

	CAN_FilterTypeDef filtercfg;

	/*When a message arrives, the hardware checks it against the active filter banks.*/
	// --- CONFIGURING BANK 0 ---
	filtercfg.FilterActivation = CAN_FILTER_ENABLE;
	filtercfg.SlaveStartFilterBank = 10;  // how many filters to assign to the CAN1 (master can)
	filtercfg.FilterBank = 0;  // which filter bank to use from the assigned ones
	filtercfg.FilterFIFOAssignment = CAN_FILTER_FIFO0; // decide which FIFO (FIFO0 or FIFO1) will store received messages (related interrupt is enabled)
	filtercfg.FilterScale = CAN_FILTERSCALE_32BIT; // use either one 32-bit register or two 16-bit registers.
	filtercfg.FilterMode = CAN_FILTERMODE_IDMASK;
	// In Mask mode, FilterIdHigh is the Target ID. It defines what the bits should look like. The FilterMaskIdHigh then defines which of those bits are mandatory.
	// It List Mode, it is simply ID #1. In this mode, there is no mask. FilterIdHigh is the first ID you want to allow, and FilterMaskIdHigh is actually a second ID you want to allow.
	filtercfg.FilterIdHigh = 0x446 << 5; // Holds the upper 16 bits of the filter. In 32-bit Filter Mode, the first 5 bits of that register are used for RTR, IDE, etc.
	filtercfg.FilterIdLow = 0;
	filtercfg.FilterMaskIdHigh = 0x446<<5; // define which ID bits to compare (e.g., shift the STD ID by 5 because it starts at bit 5).
	filtercfg.FilterMaskIdLow = 0x7FF << 5; // Check all 11 bits, catch only 0x446 ID
	HAL_CAN_ConfigFilter(&hcan1, &filtercfg);

	// --- CONFIGURING BANK 1 (if needed) to FIFO1 ---
	filtercfg.SlaveStartFilterBank = 1;  // how many filters to assign to the CAN1 (master can)
	filtercfg.FilterBank = 1;  // which filter bank to use from the assigned ones
	filtercfg.FilterFIFOAssignment = CAN_FILTER_FIFO1; // decide which FIFO (FIFO0 or FIFO1) will store received messages (related interrupt is enabled)
	filtercfg.FilterScale = CAN_FILTERSCALE_32BIT; // use either one 32-bit register or two 16-bit registers.
	filtercfg.FilterMode = CAN_FILTERMODE_IDMASK;
	filtercfg.FilterIdHigh = 0x100 << 5; // Binary: 001 0000 0000
	filtercfg.FilterIdLow = 0;
	filtercfg.FilterMaskIdHigh = 0xFF0 << 5; // Binary: 111 1111 0000 -> The filter  will accept 0x100, 0x101, 0x102 up to 0x10F.
	filtercfg.FilterMaskIdLow = 0x0000;
	HAL_CAN_ConfigFilter(&hcan1, &filtercfg);

/*1. In Mask Mode (Identifier Mask)
This is the default for most people. Here, FilterIdHigh is the Target, and FilterMaskIdHigh is the Constraint.
    FilterIdHigh: "I am looking for 0x123."
    FilterMaskIdHigh: "I want you to check every single bit of that ID (0x7FF)."
    Result: You get exactly one ID.

2. In List Mode (Identifier List)
In this mode, there is no mask. The hardware stops acting like a "bit-checker" and starts acting like a "phone book." It treats both registers as independent IDs.
    FilterIdHigh: "Accept ID 0x123."
    FilterMaskIdHigh: "Also accept ID 0x456."
    Result: You get two specific IDs per filter bank, but you have no way to use a "wildcard" (mask) to catch a range.

Why the naming is confusing
The HAL uses the variable name FilterMaskIdHigh regardless of which mode you are in.
    In Mask Mode, that variable is a bit-mask.
    In List Mode, that variable is actually a second ID.

How to decide which to use?
If you want to...	Use this Mode	Setup
Catch one specific ID	Mask Mode	ID = 0x123, Mask = 0x7FF
Catch a group of IDs (e.g., 0x100 to 0x10F)	Mask Mode	ID = 0x100, Mask = 0xFF0
Catch two unrelated IDs in one bank	List Mode	ID1 = 0x123, ID2 = 0x456*/

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
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  if (HAL_CAN_ActivateNotification(&hcan1, // --- TX Interrupts ---
		    //CAN_IT_TX_MAILBOX_EMPTY |       // Transmit mailbox becomes empty
		    // --- RX Interrupts ---
		    CAN_IT_RX_FIFO0_MSG_PENDING |  // New message in FIFO 0
			CAN_IT_RX_FIFO1_MSG_PENDING
		    // CAN_IT_RX_FIFO0_FULL |          // FIFO 0 is full
		    // CAN_IT_RX_FIFO1_MSG_PENDING |   // New message in FIFO 1
		    // --- Status Change / Error (SCE) Interrupts ---
		    // CAN_IT_ERROR_WARNING |          // Error counter > 96
		    // CAN_IT_ERROR_PASSIVE |          // Error counter > 127
		    // CAN_IT_BUSOFF |                 // Bus-Off state (counter > 255)
		    // CAN_IT_LAST_ERROR_CODE |        // Type of last error (ACK, Stuff, etc)
			// CAN_IT_ERROR                    // Global Error) != HAL_OK)
			) != HAL_OK) {
 	  Error_Handler();
   }


  /* USER CODE BEGIN 2 */

  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_4);

  // 1. Start the CAN peripheral - This moves it from INIT to NORMAL mode
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_4);
      Error_Handler();
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK)
		{
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);

		}

		HAL_Delay(1000);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 75;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Orange_Led_GPIO_Port, Orange_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_Led_Pin */
  GPIO_InitStruct.Pin = Blue_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Blue_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Orange_Led_Pin */
  GPIO_InitStruct.Pin = Orange_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Orange_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_Led_Pin */
  GPIO_InitStruct.Pin = Green_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_Led_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
