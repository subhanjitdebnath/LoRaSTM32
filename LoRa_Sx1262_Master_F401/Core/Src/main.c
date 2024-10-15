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
#include "sx126x.h"
#include "sx126x_hal.h"
#include "sx1262_B_common.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

LoRaConfig LoRa;

uint8_t txdata[10];
uint8_t rxdata[10];
static volatile bool irq_fired = false;
static sx126x_pkt_type_t pkt_type ;
static sx126x_chip_status_t radio_status;
static sx126x_stats_lora_t stats;
static sx126x_errors_mask_t errors;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /****************************LoRa Module Initialization *******************************/
  printf("Master APPLICATION\n\r");
  init_LoRa_parm();
  sx126x_clear_device_errors( &LoRa);
  sx126x_init(&LoRa );
  Radio_init(&LoRa);
  sx126x_clear_irq_status( &LoRa, SX126X_IRQ_ALL );
  sx126x_set_dio_irq_params(
		  &LoRa, SX126X_IRQ_ALL,
          SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE,
          SX126X_IRQ_NONE, SX126X_IRQ_NONE );

      sx126x_clear_irq_status( &LoRa, SX126X_IRQ_ALL );
  /****************************LoRa Module Initialization END *******************************/
      sx126x_get_pkt_type( &LoRa, &pkt_type );
      sx126x_get_status( &LoRa, &radio_status );
      sx126x_get_lora_stats( &LoRa, &stats );
      sx126x_get_device_errors( &LoRa, &errors);


      HAL_Delay(5000);
      sx126x_get_device_errors( &LoRa, &errors);
      sx126x_set_rx( &LoRa, 0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  apps_common_sx126x_irq_process(&LoRa);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, YELLOW_Pin|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WHITE_GPIO_Port, WHITE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : YELLOW_Pin RED_Pin NSS_Pin */
  GPIO_InitStruct.Pin = YELLOW_Pin|RED_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WHITE_Pin RST_Pin */
  GPIO_InitStruct.Pin = WHITE_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void init_LoRa_parm(void)
{
	txdata[0] = 'M';
	txdata[1] = 'L';
	txdata[2] = 'o';
	txdata[3] = 'R';
	txdata[4] = 'a';

	LoRa.BUSY_port = BUSY_GPIO_Port;
	LoRa.BUSY_pin  = BUSY_Pin;
	LoRa.NSS_port  = NSS_GPIO_Port;
	LoRa.NSS_pin   = NSS_Pin;
	LoRa.RST_port  = RST_GPIO_Port;
	LoRa.RST_pin   = RST_Pin;
	LoRa.DIO1_port = DIO1_GPIO_Port;
	LoRa.DIO1_pin  = DIO1_Pin;

	LoRa.hSPIx     = &hspi2;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == LoRa.DIO1_pin)
  {
	  irq_fired = true;
  }
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
void apps_common_sx126x_irq_process( const void* context )
{
    if( irq_fired == true )
    {
        irq_fired = false;

        sx126x_irq_mask_t irq_regs;
        sx126x_get_and_clear_irq_status( context, &irq_regs );

        if( ( irq_regs & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
        {
            printf( "Tx done\n\r" );
            //on_tx_done( );
        }

        if( ( irq_regs & SX126X_IRQ_RX_DONE ) == SX126X_IRQ_RX_DONE )
        {
        	printf( "Rx done\n\r" );
            sx126x_handle_rx_done( context );
            on_rx_done( );

        }

        if( ( irq_regs & SX126X_IRQ_PREAMBLE_DETECTED ) == SX126X_IRQ_PREAMBLE_DETECTED )
        {
        	printf( "Preamble detected\n\r" );
            //on_preamble_detected( );
        }

        if( ( irq_regs & SX126X_IRQ_SYNC_WORD_VALID ) == SX126X_IRQ_SYNC_WORD_VALID )
        {
        	printf( "Syncword valid\n\r" );
            //on_syncword_valid( );
        }

        if( ( irq_regs & SX126X_IRQ_HEADER_VALID ) == SX126X_IRQ_HEADER_VALID )
        {
        	printf( "Header valid\n\r" );
            //on_header_valid( );
        }

        if( ( irq_regs & SX126X_IRQ_HEADER_ERROR ) == SX126X_IRQ_HEADER_ERROR )
        {
        	printf( "Header error\n\r" );
            //on_header_error( );
        }

        if( ( irq_regs & SX126X_IRQ_CRC_ERROR ) == SX126X_IRQ_CRC_ERROR )
        {
        	printf( "CRC error\n\r" );
            //on_crc_error( );
        }

        if( ( irq_regs & SX126X_IRQ_CAD_DONE ) == SX126X_IRQ_CAD_DONE )
        {
        	printf( "CAD done\n\r" );
            if( ( irq_regs & SX126X_IRQ_CAD_DETECTED ) == SX126X_IRQ_CAD_DETECTED )
            {
            	printf( "Channel activity detected\n\r" );
                //on_cad_done_detected( );
            }
            else
            {
            	printf( "No channel activity detected\n\r" );
                //on_cad_done_undetected( );
            }
        }

        if( ( irq_regs & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT )
        {
        	printf( "Rx timeout\n\r" );
            //on_rx_timeout( );
        }

        if( ( irq_regs & SX126X_IRQ_LR_FHSS_HOP ) == SX126X_IRQ_LR_FHSS_HOP )
        {
        	printf( "FHSS hop done\n\r" );
            //on_fhss_hop_done( );
        }
    }
}
void on_rx_done(void)
{
	sx126x_rx_buffer_status_t rx_buffer_status;
	char received[10];
	int n;

	sx126x_get_rx_buffer_status( &LoRa, &rx_buffer_status );
	sx126x_read_buffer( &LoRa, 0, (uint8_t*)received, rx_buffer_status.pld_len_in_bytes );
	rxdata[rx_buffer_status.pld_len_in_bytes] = '\0';
	printf("%s\n\r",received);
	sx126x_set_rx( &LoRa, 0);
	sscanf(&received[4],"%d",&n);
	UpdateLed(n);

}

void UpdateLed(int Number)
{
	switch(Number)
	{

	case 1:
		//1 on and 2-3 off
		HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 1);
			HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 0);
			HAL_GPIO_WritePin(WHITE_GPIO_Port, WHITE_Pin, 0);
	break;
	case 2:
		//2 on and 1-3 off
			HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);
		HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 1);
			HAL_GPIO_WritePin(WHITE_GPIO_Port, WHITE_Pin, 0);
	break;
	case 3:
		//3 on and 1-2 off
			HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);
			HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 0);
		HAL_GPIO_WritePin(WHITE_GPIO_Port, WHITE_Pin, 1);
	case 0:
	default:
			//ALL off
			HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);
			HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 0);
			HAL_GPIO_WritePin(WHITE_GPIO_Port, WHITE_Pin, 0);
		break;
	}
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
