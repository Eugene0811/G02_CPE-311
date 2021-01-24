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
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_exti.h"
#include "math.h"  


void SystemClock_Config(void);
void ltc4727js_Config(void);
void ShowOn7Segment(uint8_t  x);

uint8_t i;
uint32_t pin7seg = LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
uint32_t pinDrain = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
uint32_t number[10] = {LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
											 LL_GPIO_PIN_7 | LL_GPIO_PIN_8,
											 LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_12,
											 LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_12,
											 LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12,
											 LL_GPIO_PIN_6 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12,
											 LL_GPIO_PIN_6 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12,
											 LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8,
											 LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12,
											 LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12
									    };
uint32_t highLow[2] = { LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
												LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12
											};

uint32_t digit[3] = {LL_GPIO_PIN_8 , LL_GPIO_PIN_9 , LL_GPIO_PIN_10};
uint32_t seg[3];

int main(void)
{
  SystemClock_Config();
	ltc4727js_Config();

	while(1)
	{ 
		ShowOn7Segment(1);
	}
}

void ltc4727js_Config(void)
{
	LL_GPIO_InitTypeDef ltc4727_initstruct;
	//config ltc4727js
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	
	ltc4727_initstruct.Mode = LL_GPIO_MODE_OUTPUT;
	ltc4727_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ltc4727_initstruct.Pull = LL_GPIO_PULL_NO;
	ltc4727_initstruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	ltc4727_initstruct.Pin = pin7seg;
	LL_GPIO_Init(GPIOC, &ltc4727_initstruct);
	
	ltc4727_initstruct.Pin = pinDrain;
	LL_GPIO_Init(GPIOA, &ltc4727_initstruct);
}

void ShowOn7Segment(uint8_t  level)
{
	for(i = 0; i < 3; ++i)
		{
			LL_GPIO_ResetOutputPin(GPIOC, pin7seg);
			LL_GPIO_ResetOutputPin(GPIOA, pinDrain);
			LL_GPIO_SetOutputPin(GPIOC, highLow[level]);
			if((i == 1 & level == 0)|(i == 2 & level == 1))
				LL_GPIO_SetOutputPin(GPIOA, digit[i]);
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
