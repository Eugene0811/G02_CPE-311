/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#include "string.h"

//PB6 TX PB7 RX USART1
void SystemClock_Config(void);
void GPIO_USART_Config(void);
void USART_Config(void);
void USART_SendStream(uint8_t* d, uint16_t s);

uint8_t recv_buffer[10] = {0};
uint8_t show_str[6] = {0};
uint16_t w_index = 0;

int main(void)
{

  SystemClock_Config();
	USART_Config();
	LCD_GLASS_Init();
	
	while(1)
	{
		while(LL_USART_IsActiveFlag_RXNE(USART1) == RESET);
		recv_buffer[w_index] = LL_USART_ReceiveData8(USART1);
		w_index = (w_index+1) % 10;
		if(recv_buffer[w_index - 1] == '\n')
		{
			for(int i = 0; i < 6; i++)
			{
				show_str[i] = recv_buffer[i+1];
			}
			LL_LCD_Clear();
			LCD_GLASS_DisplayString((uint8_t*)show_str);
			LL_mDelay(100);
			memset (recv_buffer,NULL,10);
			memset (show_str,NULL,10);
			//w_index = 0;
		}
		else {
			w_index = w_index % 10;
		}
	}
}

void GPIO_USART_Config(void)
{
	LL_GPIO_InitTypeDef gpio_conf;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	gpio_conf.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_conf.Pull = LL_GPIO_PULL_NO;
	gpio_conf.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_conf.Pin = LL_GPIO_PIN_6; 
	gpio_conf.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_conf.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	
	gpio_conf.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOB, &gpio_conf);
}

void USART_Config(void)
{
	LL_USART_InitTypeDef usart_conf;
	GPIO_USART_Config();
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	
	usart_conf.BaudRate = 9600;
	usart_conf.DataWidth = LL_USART_DATAWIDTH_8B;
	usart_conf.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	usart_conf.Parity = LL_USART_PARITY_NONE;
	usart_conf.StopBits = LL_USART_STOPBITS_1;
	usart_conf.OverSampling = LL_USART_OVERSAMPLING_16;
	usart_conf.TransferDirection = LL_USART_DIRECTION_TX_RX;
	LL_USART_Init(USART1, &usart_conf);
	
	LL_USART_Enable(USART1);
}

void USART_SendStream(uint8_t* d, uint16_t s)
{
	uint16_t i;
	
	for(i = 0; i < s; ++i)
	{
		while(LL_USART_IsActiveFlag_TXE(USART1) == RESET);
		LL_USART_TransmitData8(USART1, d[i]);
	}

}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}
