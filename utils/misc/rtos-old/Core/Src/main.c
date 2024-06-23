
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


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOSConfig.h"
#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

uint8_t rxData;
char userMsg[256] = {};

TaskHandle_t xTask1;
TaskHandle_t xTask2;

SemaphoreHandle_t xMutexSemaphore;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

static void vTask1( void* pvParameters );
static void vTask2( void* pvParameters );

void vSoftwareInterruptHandler( void );
void printMsg( char* msg );

void* pMem;

#define HEAP_CHUNK_SIZE         8     // for all archs lower than 64-bit, otherwise it should be 16

void* __wrap_malloc   (size_t Size);
void  __wrap_free     (void* pMem);

// Prototypes of the original stdlib functions symbols created during the linker wrapping functionality
void* __real_malloc       (size_t size);
void  __real_free         (void* p);

uint32_t *__heap_start__ = (uint32_t *)0x20000000;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	BaseType_t xStatus;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Enable the Cortex-M4 CYCCNT counter register.
   * Address: 0xE0001000
   * Access: Read/Write
   * Reset State: 0x40000000
  */
  DWT_CTRL |= (1 << 0);

  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();


  /* In FreeRTOS stack is not in bytes, but in sizeof(StackType_t) which is 4 on ARM ports.       */
  /* Stack size should be therefore 4 byte aligned in order to avoid division caused side effects */

  uint32_t stackSize = (1024 * 1);
  uint32_t stack = stackSize / sizeof(StackType_t);


  xStatus = xTaskCreate(vTask1, "Task1", (uint16_t)stack, NULL, 2, &xTask1);
  configASSERT(xStatus == pdPASS);

  xStatus = xTaskCreate(vTask2, "Task2", (uint16_t)stack, NULL, 2, &xTask2);
  configASSERT(xStatus == pdPASS);

  /* Attempt to create a semaphore. */
  xMutexSemaphore = xSemaphoreCreateMutex();

  /* The semaphore is created in the 'empty' state, meaning the semaphore
  	 * must first be given using the xSemaphoreGive() API function before it
  	 * can subsequently be taken (obtained) using the xSemaphoreTake() function.
  	 * */
  	xSemaphoreGive( xMutexSemaphore );

  // start the freeRTOS scheduler
  vTaskStartScheduler();

  // if the control comes here, then the launch of the scheduler has
  // failed due to insufficient memory in heap

  while (1)
  {

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
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

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


static void vTask1( void* pvParameters )
{
	while(1)
	{
		// before printing, take the semaphore
		xSemaphoreTake( xMutexSemaphore, portMAX_DELAY );

    pMem = malloc(64);
		sprintf(userMsg, "Task 1 is running\r\n");
		printMsg(userMsg);

		xSemaphoreGive( xMutexSemaphore );
		vTaskDelay( pdMS_TO_TICKS(500) );
	}
}

static void vTask2( void* pvParameters )
{

	while(1)
	{
		// before printing, take the semaphore
		xSemaphoreTake( xMutexSemaphore, portMAX_DELAY );

    free(pMem);
		sprintf(userMsg, "Task 2 is running\r\n");
		printMsg(userMsg);

		xSemaphoreGive( xMutexSemaphore );
		vTaskDelay( pdMS_TO_TICKS(500) );
	}
}

void printMsg(char* msg)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen((char*)msg), HAL_MAX_DELAY);
}

/*********************************************************************
*
*       _GetAllocBlockSize
*
*  Function description
*    Returns the real block size internally reserved for the requested
*    allocation size for the 'Basic' SEGGER heap model
*
*  Parameters
*    ReqSize : requested size
*
*  Return value
*    block size
*/
static U32 _GetAllocBlockSize(U32 ReqSize) {

  return (ReqSize + sizeof(unsigned) + HEAP_CHUNK_SIZE - 1) & (unsigned)(-HEAP_CHUNK_SIZE);

}

/*********************************************************************
*
*       __wrap_malloc
*
*  Function description
*    Replaces the stdlib malloc function and dispatches the 
*    SystemView HeapAlloc event.
*
*  Parameters
*    Size : Size of the memory block to be allocated
*
*  Return value
*    Pointer to the allocated memory
*/
void* __wrap_malloc(size_t Size) {
  void* pMem;
  
  pMem = __real_malloc(Size);
  // SystemView needs the real internal heap block size, not the amount requested by the user,
  // hence the call to a helper function _GetAllocBlockSize for the currently selected heap type ('Basic')
  SEGGER_SYSVIEW_HeapAlloc(__heap_start__, pMem, _GetAllocBlockSize(Size));
  return pMem;
}

/*********************************************************************
*
*       __wrap_free
*
*  Function description
*    Replaces the stdlib free function and dispatches the SystemView HeapFree event.
*
*  Parameters
*    pMem : Pointer to be released
*/
void __wrap_free(void* pMem) {
  __real_free(pMem);

  SEGGER_SYSVIEW_HeapFree(__heap_start__, pMem);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
