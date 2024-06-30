#ifndef __STM32L4xx_IT_H
#define __STM32L4xx_IT_H

#ifdef __cplusplus
extern "C"
{
#endif

  /* Private includes ----------------------------------------------------------*/

  /* Exported types ------------------------------------------------------------*/

  /* Exported constants --------------------------------------------------------*/

  /* Exported macro ------------------------------------------------------------*/

  /* Exported functions prototypes ---------------------------------------------*/
  void NMI_Handler(void);
  void HardFault_Handler(void);
  void MemManage_Handler(void);
  void BusFault_Handler(void);
  void UsageFault_Handler(void);
  void SVC_Handler(void);
  void DebugMon_Handler(void);
  void PendSV_Handler(void);
  void SysTick_Handler(void);
  void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_IT_H */
