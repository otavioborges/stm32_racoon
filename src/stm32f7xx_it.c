#include "stm32f7xx_it.h"
#include "stm32f7xx_hal.h"

void NMI_Handler(void){
	__asm("nop");
}

void HardFault_Handler(void){
	__asm("nop");
}

void MemManage_Handler(void){
	__asm("nop");
}

void BusFault_Handler(void){
	__asm("nop");
}

void UsageFault_Handler(void){
	__asm("nop");
}

void SVC_Handler(void){
	__asm("nop");
}

void DebugMon_Handler(void){
	__asm("nop");
}

void PendSV_Handler(void){
	__asm("nop");
}

void SysTick_Handler(void){
	HAL_IncTick();
}

void BSP_SDMMC_IRQHandler(void)
{
  HAL_SD_IRQHandler(&g_sdHandle);
}

/**
* @brief  This function handles DMA2 Stream 6 interrupt request.
* @param  None
* @retval None
*/
void BSP_SDMMC_DMA_Tx_IRQHandler(void)
{
  HAL_DMA_IRQHandler(g_sdHandle.hdmatx);
}

/**
* @brief  This function handles DMA2 Stream 3 interrupt request.
* @param  None
* @retval None
*/
void BSP_SDMMC_DMA_Rx_IRQHandler(void)
{
  HAL_DMA_IRQHandler(g_sdHandle.hdmarx);
}

