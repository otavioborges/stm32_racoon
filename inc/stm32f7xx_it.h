#ifndef __STM32F7xx_IT_H
#define __STM32F7xx_IT_H

#include "stm32746g_discovery.h"
#include "stm32f7xx_hal_uart.h"

#ifdef __cplusplus
 extern "C" {
#endif 

UART_HandleTypeDef g_uartComHandle;
SD_HandleTypeDef g_sdHandle;

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F7xx_IT_H */
