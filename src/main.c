#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "main.h"
#include "stm32f7xx_it.h"
#include "stm32f7xx_hal_uart.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sd.h"

#include "ff_gen_drv.h"
#include "sd_diskio.h"

#include "genesis.h"

char g_sdPath[10];
FATFS g_FatFs;
FIL g_file;

static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void LCD_Config(void);

void UartPrint(const char *message, ...){
	char msg[128];
	va_list args;
	uint16_t length;

	va_start(args, message);
	vsprintf(msg, message, args);
	va_end(args);

	length = strlen(msg);
	HAL_UART_Transmit(&g_uartComHandle, (uint8_t *)msg, length, 100);
}

uint8_t inc = 0;
uint32_t nextColor(uint32_t color){
	switch(inc){
		case 0:
			if((color & 0xFF) == 0xF0)
				inc ++;
			color += 0x8;
			break;
		case 1:
			if((color & 0xFF00) == 0xF000)
				inc++;
			color += 0x800;
			color -= 0x8;
			break;
		case 2:
			if((color & 0xFF0000) == 0xF00000)
				inc++;
			color += 0x80000;
			color -= 0x800;
			break;
		case 3:
			if((color & 0xFF0000) == 0x080000)
				inc = 0;

			color -= 0x80000;
			break;
	}

	return color;
}

#define MSB2LSBDW( x )  (\
              ( ( x & 0x000000FF ) << 24 ) \
            | ( ( x & 0x0000FF00 ) << 8 ) \
            | ( ( x & 0x00FF0000 ) >> 8 ) \
            | ( ( x & 0xFF000000 ) >> 24 ) \
             )

int main(){
	CPU_CACHE_Enable();
	HAL_Init();
	SystemClock_Config();
	LCD_Config();
	BSP_SDRAM_Init();

	FATFS_LinkDriver(&SD_Driver, g_sdPath);
	FRESULT res;
	res = f_mount(&g_FatFs, (TCHAR const *)g_sdPath, 0);
	res = f_open(&g_file, "0:/sonic.bin", FA_READ);
	uint32_t remainder = f_size(&g_file);
	f_read(&g_file, (uint32_t *)SDRAM_DEVICE_ADDR, remainder, &remainder);

	uint32_t *rom = (uint32_t *)SDRAM_DEVICE_ADDR;
//	for(uint32_t byte = 0; byte < (remainder >> 2); byte++)
//		rom[byte] = MSB2LSBDW(rom[byte]);

	remainder = f_size(&g_file);
	f_close(&g_file);


	GENESIS_CInit(SDRAM_DEVICE_ADDR, remainder);

	g_uartComHandle.Init.BaudRate = 115200;
	g_uartComHandle.Init.WordLength = UART_WORDLENGTH_8B;
	g_uartComHandle.Init.StopBits = UART_STOPBITS_1;
	g_uartComHandle.Init.Parity = UART_PARITY_NONE;
	g_uartComHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	g_uartComHandle.Init.Mode = UART_MODE_RX | UART_MODE_TX;

	BSP_COM_Init(COM1, &g_uartComHandle);

	uint32_t color = 0xFF000000;
	uint8_t count = 0;
	for(;;){
		BSP_LCD_Clear(color);
		UartPrint("Teste %i\r\n", count);
		HAL_Delay(50);

		color = nextColor(color);
		count++;
	}
}

static void SystemClock_Config(void){
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 200 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

static void CPU_CACHE_Enable(void){
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

static void LCD_Config(void){
  /* LCD Initialization */
  BSP_LCD_Init();

  /* LCD Initialization */
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));

  /* Enable the LCD */
  BSP_LCD_DisplayOn();

  /* Select the LCD Background Layer  */
  BSP_LCD_SelectLayer(0);

  /* Clear the Background Layer */
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  /* Select the LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);

  /* Clear the Foreground Layer */
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  /* Configure the transparency for foreground and background :
     Increase the transparency */
  BSP_LCD_SetTransparency(0, 0);
  BSP_LCD_SetTransparency(1, 100);
}
