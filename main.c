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
ring_buffer_uart1 rx_buffer_uart1 = {{0}, 0, 0};
ring_buffer_uart1 tx_buffer_uart1 = {{0}, 0, 0};
ring_buffer_uart1 *_rx_buffer_uart1;
ring_buffer_uart1 *_tx_buffer_uart1;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* CMSIS threshold for "QF-aware" interrupts, see NOTE5 */
#define USART_SR_ORE (0x1UL << 3U)

/*Different UART Message Type*/
#define UART_MSG_1_PACKET_LENGTH 1008 /*Fixed Packet size for UART Msg of type 1*/
#define UART_MSG_2_PACKET_LENGTH 1500 /*Fixed Packet size for UART Msg of type 2*/
#define UART_MSG_3_PACKET_LENGTH 1800 /*Fixed Packet size for UART Msg of type 3*/

#define RING_BUFFER_SIZE   2000 /*Max size of the buffer used to store UART RX Data*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
void Ringbuf_UART1_Free(void);
void Store_UART1_Char(unsigned char c, ring_buffer_uart1 *buffer);
void Ringbuf_Init(void);
unsigned char EEPROM_Buff[RING_BUFFER_SIZE] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

uint8_t rxdata[1000] = {0};
uint16_t ui16Index = 0;
uint16_t Baudrate = 0;
uint16_t isr_callback_ctr = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Ringbuf_UART1_Free(void)
{
    memset(_rx_buffer_uart1->buffer_uart1, 0x00, UART1_BUFFER_SIZE);
    _rx_buffer_uart1->head_uart1 = NULL;
    _rx_buffer_uart1->tail_uart1 = NULL;
    memset(_tx_buffer_uart1->buffer_uart1, 0x00, UART1_BUFFER_SIZE);
    _tx_buffer_uart1->head_uart1 = NULL;
    _tx_buffer_uart1->tail_uart1 = NULL;
}

void Store_UART1_Char(unsigned char c, ring_buffer_uart1 *buffer)
{
    uint32_t i = (uint32_t)(buffer->head_uart1 + 1) % UART1_BUFFER_SIZE; /*i will be pointing tail now*/
    /**
   * if we should be storing the received character into the location
   * just before the tail (meaning that the head would advance to the
   * current location of the tail), we're about to overflow the buffer
   * and so we don't write the character or advance the head.
   */

    if (i != buffer->tail_uart1)
    {
        buffer->buffer_uart1[buffer->head_uart1] = c;
        buffer->head_uart1 = i;
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
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  NVIC_SetPriorityGrouping(0U);
  NVIC_EnableIRQ(UART4_IRQn);

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Ringbuf_Init();
  Ringbuf_UART1_Free();
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 2400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);

  __HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_Callback(void){ /*Every 3 second*/
    Baudrate = ((rx_buffer_uart1.ui8NoOfBytesReceivedSofar*8)/3)-312; /*Approx Baudrate value including S.W, Parity,Start(1 bit) and stop(1 or 2 bit) bits delay consideration*/
    if(rx_buffer_uart1.ui8NoOfBytesReceivedSofar == UART_MSG_1_PACKET_LENGTH)
    {
    	EEPROM_Buff[UART_MSG_1_PACKET_LENGTH] = '\n';
    	HAL_UART_Transmit(&huart4,(uint8_t*)EEPROM_Buff,sizeof(EEPROM_Buff),5000);
    	printf("Baudrate is :  %d \n",Baudrate);
    }
    /*If incase you don't want MCU to send the message repeatedly , stop Timer2 Here*/
}

void UART_ISR(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        uint32_t isrflags = READ_REG(huart->Instance->ISR);
        uint32_t cr1its = READ_REG(huart->Instance->CR1);
        if ( ((isrflags & USART_ISR_ORE) == 8U)    || ((isrflags & USART_ISR_PE) == 1U) || ((isrflags & USART_ISR_FE) == 2U) || ((isrflags & USART_ISR_NE) == 4U))            /* Check for Overflow flag */
        {
            huart->Instance->ISR;                   // Read status register
            unsigned char c = huart->Instance->RDR; // Read data register
            rxdata[ui16Index++] = c;
        }
        /* if DR is not empty and the Rx Int is enabled */
        if (((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        {
        	isr_callback_ctr++;
        	if(isr_callback_ctr++ == 1){
            	HAL_TIM_Base_Start_IT(&htim2);
        	}
            /******************
            * @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
            *         error) and IDLE (Idle line detected) flags are cleared by software
            *         sequence: a read operation to USART_SR register followed by a read
            *         operation to USART_DR register.
            * @note   RXNE flag can be also cleared by a read to the USART_DR register.
            * @note   TC flag can be also cleared by software sequence: a read operation to
            *         USART_SR register followed by a write operation to USART_DR register.
            * @note   TXE flag is cleared only by a write to the USART_DR register.
            *********************/
            huart->Instance->ISR;                   // Read status register
            unsigned char c = huart->Instance->RDR; // Read data register

                Store_UART1_Char(c, _rx_buffer_uart1); // store data in buffer
                ++rx_buffer_uart1.ui8NoOfBytesReceivedSofar;

                if(rx_buffer_uart1.ui8NoOfBytesReceivedSofar == UART_MSG_1_PACKET_LENGTH)
                {
                	memcpy(&EEPROM_Buff,&(rx_buffer_uart1.buffer_uart1[0]),UART_MSG_1_PACKET_LENGTH);
                    Ringbuf_UART1_Free();
                }
			    if ((rx_buffer_uart1.ui8NoOfBytesReceivedSofar == RING_BUFFER_SIZE))
                {
			    	ui16Index = 0;
                    rx_buffer_uart1.ui8NoOfBytesReceivedSofar = 0;
                    Ringbuf_UART1_Free();
                }

            return;
        }
        /*If Size of the message is not defined then use this method to send data via UART*/
        /*If interrupt is caused due to Transmit Data Register Empty */
        if (((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
        {
            if (tx_buffer_uart1.head_uart1 == tx_buffer_uart1.tail_uart1)
            {
                // Buffer empty, so disable interrupts
                __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
            }
            else
            {
                // There is more data in the output buffer. Send the next byte
                unsigned char c = tx_buffer_uart1.buffer_uart1[tx_buffer_uart1.tail_uart1];
                tx_buffer_uart1.tail_uart1 = (tx_buffer_uart1.tail_uart1 + 1) % UART1_BUFFER_SIZE;
                /******************
                * @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
                *         error) and IDLE (Idle line detected) flags are cleared by software
                *         sequence: a read operation to USART_SR register followed by a read
                *         operation to USART_DR register.
                * @note   RXNE flag can be also cleared by a read to the USART_DR register.
                * @note   TC flag can be also cleared by software sequence: a read operation to
                *         USART_SR register followed by a write operation to USART_DR register.
                * @note   TXE flag is cleared only by a write to the USART_DR register.
                *********************/
                huart->Instance->ISR;
                huart->Instance->RDR = c;
            }
            return;
        }
    }
}

void Ringbuf_Init(void)
{
    _rx_buffer_uart1 = &rx_buffer_uart1;
    _tx_buffer_uart1 = &tx_buffer_uart1;

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
}

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
