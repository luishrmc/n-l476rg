/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "mainController.h"
#include "uartDriver.h"
#include "config.h"
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
/* RTT includes. */
#include "SEGGER_RTT.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void vTask1(void *pvParameters);
static void vTask2(void *pvParameters);
static void vTask3(void *pvParameters);
static void vTask4(void *pvParameters);

TaskHandle_t xTask1;
TaskHandle_t xTask2;
TaskHandle_t xTask3;
TaskHandle_t xTask4;
/* Private user code ---------------------------------------------------------*/

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    BaseType_t xStatus;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    traceSTART();

    /* In FreeRTOS stack is not in bytes, but in sizeof(StackType_t) which is 4 on ARM ports.       */
    /* Stack size should be therefore 4 byte aligned in order to avoid division caused side effects */
    uint32_t stackSize = (1024 / 4);
    uint32_t stack = stackSize / sizeof(StackType_t);

    xStatus = xTaskCreate(vTask1, "Task1", (uint16_t)stack, NULL, 2, &xTask1);
    configASSERT(xStatus == pdPASS);

    xStatus = xTaskCreate(vTask2, "Task2", (uint16_t)stack, NULL, 2, &xTask2);
    configASSERT(xStatus == pdPASS);

    xStatus = xTaskCreate(vTask3, "Task3", (uint16_t)stack, NULL, 2, &xTask3);
    configASSERT(xStatus == pdPASS);

    xStatus = xTaskCreate(vTask4, "Task4", (uint16_t)stack, NULL, 2, &xTask4);
    configASSERT(xStatus == pdPASS);

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
    SEGGER_RTT_printf(0, "###### %s - v%s (%s) ######\r\n", CONFIG_PROJECT_NAME, CONFIG_PROJECT_VERSION, CONFIG_PROJECT_BUILD);

    // start the freeRTOS scheduler
    vTaskStartScheduler();

    /* Infinite loop */
    while (1)
    {
    }
}

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

static void vTask1(void *pvParameters)
{
    while (1)
    {
        SEGGER_SYSVIEW_PrintfTarget("T1");
        SEGGER_RTT_WriteString(0, "T1\n");
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void vTask2(void *pvParameters)
{
    while (1)
    {
        SEGGER_RTT_WriteString(0, "T2\n");
        SEGGER_SYSVIEW_PrintfTarget("T2");
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void vTask3(void *pvParameters)
{
    while (1)
    {
        SEGGER_RTT_WriteString(0, "T3\n");
        SEGGER_SYSVIEW_PrintfTarget("T3");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void vTask4(void *pvParameters)
{
    while (1)
    {
        SEGGER_RTT_WriteString(0, "T4\n");
        SEGGER_SYSVIEW_PrintfTarget("T4");
        vTaskDelay(pdMS_TO_TICKS(700));
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
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
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
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
ex:
    printf("Wrong parameters value: file %s on line %d\r\n", file, line) * /
}
#endif /* USE_FULL_ASSERT */
