#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_adc.h" 
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
uint8_t danger_cnt = 0;
int decibel, showDecibel = 0;
void SystemClock_Config(void);
void ADC_Configuration(void);
void TIM_BASE_DurationConfig(void);
void DisplayOnLCD(void);

int check;


int main()
{
	SystemClock_Config();
	LCD_GLASS_Init();
	ADC_Configuration();
	TIM_BASE_DurationConfig();

	while(1)
	{
		sound_min = 1024;
    sound_max = 0;
		
		
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
		}
		/*
		if(decibel > 85)
		{
			danger_cnt++;
			if(danger_cnt==20)
			{
				LCD_GLASS_Clear();
				LCD_GLASS_DisplayString((uint8_t*)"danger");
				LL_mDelay(500);
				LCD_GLASS_Clear();
				danger_cnt = 0;	
			}	
		}
		else
			danger_cnt = 0;	
		*/
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
	/*
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_InitTypeDef GPIO_ADC_InitStructure;
	
	GPIO_ADC_InitStructure.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_ADC_InitStructure.Pull = LL_GPIO_PULL_NO;
	GPIO_ADC_InitStructure.Pin = LL_GPIO_PIN_5;
	GPIO_ADC_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOA, &GPIO_ADC_InitStructure);
	
	LL_ADC_SetCommonClock(ADC1_COMMON, LL_ADC_CLOCK_ASYNC_DIV2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_ADC_InitTypeDef  ADC_InitStructure;
	
	LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_1RANK);

	ADC_InitStructure.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStructure.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStructure.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;

  
  LL_ADC_Init(ADC1, &ADC_InitStructure);
	
	
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_48CYCLES);
	
	LL_ADC_REG_InitTypeDef ADC_REG_InitStructure;
	ADC_REG_InitStructure.ContinuousMode = LL_ADC_REG_SEQ_SCAN_DISABLE;
	ADC_REG_InitStructure.SequencerLength = LL_ADC_REG_RANK_5;
	
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStructure);
	
  LL_ADC_Enable(ADC1);
	*/
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
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStructure.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void TIM_BASE_DurationConfig(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 500 - 1;
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
		//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_6);
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
