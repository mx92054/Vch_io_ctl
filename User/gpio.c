#include "gpio.h"


extern short wReg[] ;
extern short coils[] ;

//-------------------------------------
void GPIO_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE) ;

	//----------------control input singal----------------------------------------------
	GPIO_InitStructure.GPIO_Pin = LED1_PIN ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(LED1_GPIO, &GPIO_InitStructure);		

	GPIO_InitStructure.GPIO_Pin = LED2_PIN ;
  GPIO_Init(LED2_GPIO, &GPIO_InitStructure);	
	
	LED1(0) ;
	LED2(0) ;
}


//--------------------------------------------------------------
//      Watchdong config
//			cycle = load_val*Prescaler/40k
//---------------------------------------------------------------
void IWDG_Configuration(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 
    IWDG_SetPrescaler(IWDG_Prescaler_16); 
    IWDG_SetReload(0xFFF);    // 0xfff*16/40k = 1.6s
    IWDG_ReloadCounter(); 
    IWDG_Enable(); 
}

//--------------------------------------------------------------
//      feed dog function
//---------------------------------------------------------------
void IWDG_Feed(void)   
{
    IWDG->KR=0XAAAA;                                  
}




//----------------------end of file-----------------------------
