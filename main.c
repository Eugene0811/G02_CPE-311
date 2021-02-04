#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_adc.h" 
#include "stm32l1xx_ll_dac.h" 
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_lcd.h" 
#include "stm32l152_glass_lcd.h"
#include "stdlib.h"
#include "stdio.h"

uint16_t adc_data = 0, i = 0;
uint16_t data_list[1000];
uint16_t sound, sound_min, sound_max, sound_amplitude;
uint16_t danger_cnt = 0;
uint8_t blink_state = 0;
int decibel, showDecibel = 0;
void SystemClock_Config(void);
void ADC_Configuration(void);
void TIM_BASE_DurationConfig(void);
void DisplayOnLCD(void);
void TIM4_R_OC_GPIO_Config(void);
void TIM4_R_BASE_Config(void);
void TIM4_R_OC_Config(void);
void ADC_G_Config(void);

int check;


int main()
{
	SystemClock_Config();
	LCD_GLASS_Init();
	ADC_Configuration();
	TIM_BASE_DurationConfig();
	TIM4_R_OC_Config();
	ADC_G_Config();
	

	while(1)
	{
		sound_min = 1024;
    sound_max = 0;
		
		//LL_DAC_ConvertData12RightAligned(DAC1,LL_DAC_CHANNEL_1, 0x00FF);
		//DAC1->DHR12R1 = (255-25)*16;
		//LL_TIM_OC_SetCompareCH1(TIM4, 25);
		/*
		LL_TIM_OC_SetCompareCH1(TIM4, i);
		LL_TIM_OC_SetCompareCH1(TIM3, 255-i);
		i = (i+1)%256;
		LL_mDelay(10);
		*/
//////////////////////////////////////////////////////////////////////////		
		
		while(LL_TIM_IsActiveFlag_UPDATE(TIM2) == RESET)
		{
			LL_ADC_REG_StartConversionSWStart(ADC1);
			while(LL_ADC_IsActiveFlag_EOCS(ADC1)==0);
			sound = ADC1->DR;
			if(sound > sound_max)
				sound_max = sound;
			else if(sound < sound_min)
				sound_min = sound;
		}
		LL_TIM_ClearFlag_UPDATE(TIM2);
		
		sound_amplitude = abs(sound_max - sound_min);
		
		decibel = sound_amplitude*(110-40)/(1024-0)+40; // dB range = 40 - 110 dB
		if(showDecibel != decibel)
		{
			showDecibel = decibel;
			DisplayOnLCD();
			//LL_TIM_OC_SetCompareCH1(TIM4, 255-sound_amplitude);
			LL_TIM_OC_SetCompareCH1(TIM4, (decibel)*(4095-0)/(110-40));
			//DAC->DHR12R1 = sound_amplitude*(4095-2304)/(1024-0)+2304;
			DAC->DHR12R1 = (110-decibel)*(4095-2304)/(110-40)+2304;
		}

		
////////////////////////////////////////////////////////////////////////////////////////////////////
		if(decibel > 85)
		{
			blink_state = 1;
			danger_cnt++;
			if(danger_cnt==100)
			{
				LCD_GLASS_Clear();
				LCD_GLASS_DisplayString((uint8_t*)"danger");
				LL_mDelay(500);
				danger_cnt = 0;	
			}	
		}
		else
			danger_cnt = 0;		
	}
}

void DisplayOnLCD(void)
{
	char showLCD[6];
	LCD_GLASS_Clear();
	sprintf(showLCD,"%d dB", showDecibel);
	LCD_GLASS_DisplayString((uint8_t*)showLCD);
}

void ADC_Configuration(void)
{ 
  RCC->AHBENR |= (1<<0);
	GPIOA->MODER |= (3<<10);
	RCC->CR |= (1<<0);
	//while(((RCC->CR & 0x02) >> 1) == 1);
	RCC->APB2ENR |= (1<<9);
	
	ADC1-> CR1 |= (1<<24) | (1<<11);
	ADC1-> CR1 &= ~(7<<13);
	ADC1-> CR2 &= ~(1<<11);
	ADC1-> SMPR3 |= (7<<15);
	ADC1-> SQR5 |= (5<<0);
	ADC1-> CR2 |= (1<<0);
	
	
}

void ADC_G_Config(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
	
	LL_GPIO_InitTypeDef led_g_initstructure;
	
	led_g_initstructure.Mode = LL_GPIO_MODE_ANALOG;
	led_g_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	led_g_initstructure.Pin = LL_GPIO_PIN_4;
	led_g_initstructure.Pull = LL_GPIO_PULL_NO;
	led_g_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOA, &led_g_initstructure);
	
	LL_DAC_InitTypeDef dac_g_initstructure;
	
	
	LL_DAC_Enable(DAC1,LL_DAC_CHANNEL_1);
	
	
}

void TIM_BASE_DurationConfig(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 100 - 1;
	timbase_initstructure.Prescaler =  32000 - 1;
	LL_TIM_Init(TIM2, &timbase_initstructure);
	
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM2_IRQn, 1);
	NVIC_EnableIRQ(TIM2_IRQn);
	LL_TIM_EnableIT_CC1(TIM2);
	
	LL_TIM_EnableCounter(TIM2); 
	LL_TIM_ClearFlag_UPDATE(TIM2); //Force clear update flag
}

void TIM2_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM2) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM2);
		if(blink_state == 1)
			LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_6);
		else
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
	}
}
	
void TIM4_R_BASE_Config(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 1024 - 1;
	timbase_initstructure.Prescaler = 314 - 1;
	
	LL_TIM_Init(TIM4, &timbase_initstructure);

}

void TIM4_R_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstructure.Alternate = LL_GPIO_AF_2;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pin = LL_GPIO_PIN_6;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

void TIM4_R_OC_Config(void)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM4_R_BASE_Config();
	TIM4_R_OC_GPIO_Config();
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4);
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}


void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM4);
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
