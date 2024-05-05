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
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mp6570_drv.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_ZEROERR_PACKET 12
#define ZEROERR_SINGLE_TRN_RD 0x02


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t UART_Data[MAX_ZEROERR_PACKET] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  FDCAN_FilterTypeDef sFDCAN_Filter;
  sFDCAN_Filter.FilterIndex = 0;
  sFDCAN_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFDCAN_Filter.FilterID1 = 0x0;
  sFDCAN_Filter.FilterID2 = 0x7FF;
  sFDCAN_Filter.FilterType = FDCAN_FILTER_RANGE;
  sFDCAN_Filter.IdType = FDCAN_STANDARD_ID;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFDCAN_Filter);

  FDCAN_TxHeaderTypeDef sTxHeader;
  sTxHeader.Identifier = 0x333;
  sTxHeader.DataLength = FDCAN_DLC_BYTES_8;
  sTxHeader.FDFormat = FDCAN_CLASSIC_CAN; //CLASSIC or FD
  sTxHeader.IdType = FDCAN_STANDARD_ID; // STANDARD or EXTENDED
  sTxHeader.TxFrameType = FDCAN_DATA_FRAME; //DATA or REMOTE

  sTxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE; //must have
  sTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  sTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  sTxHeader.MessageMarker = 0;

  FDCAN_RxHeaderTypeDef sRxHeader;

  HAL_FDCAN_Start(&hfdcan1);

  uint8_t TxData[8] = "Started", RxData[8] = {0};
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, TxData);
  HAL_Delay(100); //must have


  MP6570_Enable();
  HAL_Delay(200);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_Data, MAX_ZEROERR_PACKET);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  HAL_Delay(100);

  uint8_t temp = ZEROERR_SINGLE_TRN_RD;
  HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);


  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, UART_Data);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, UART_Data+8);



  //uint64_t AnglePosition = 0x00000000004FBBAA;
  //MP6570_SetRelativePosition64(AnglePosition);
  //MP6570_SyncUp(); //sync up

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &sRxHeader, RxData) == HAL_OK)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_Data, MAX_ZEROERR_PACKET);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

		HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start_IT(&htim2);
		MP6570_SetRelativePosition8(RxData);
		MP6570_SyncUp(); //sync up

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, UART_Data);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, UART_Data+8);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Prevent unused argument(s) compilation warning */
  //HAL_UART_Transmit(&huart1, UART_Data, Size, HAL_MAX_DELAY);
  //HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, UART_Data);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
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
