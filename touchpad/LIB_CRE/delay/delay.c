#include "ht32_cm0plus_misc.h"
#include "delay.h"
//mS microsecond delay program
void delay_us(u32 us)
{
	u32 i;
	SYSTICK_ClockSourceConfig(SYSTICK_SRC_STCLK);          //Select the external reference clock as the SysTick clock source. 8MHZ
	SYSTICK_SetReloadValue(SystemCoreClock / 8 / 1000000); // Initial reload count
	SYSTICK_IntConfig(DISABLE);                            // Whether to enable interrupt
	SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);             //Clear timer
	SYSTICK_CounterCmd(SYSTICK_COUNTER_ENABLE);            //Enable
	for( i = 0;i < us;i++ )
	{
		while( !( (SysTick->CTRL) & (1<<16) ) ); 
	}
 
	SYSTICK_CounterCmd(SYSTICK_COUNTER_DISABLE); //shut down
	SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);	 //Reset to zero
}

void delay_ms(u16 ms){ //mS millisecond delay program 	  
	while( ms-- != 0){
		delay_us(1000);	//Call a delay of 1000 microseconds
	}
}
 
void delay_s(u16 s){ //S seconds delay program	 		  	  
	while( s-- != 0){
		delay_ms(1000);	//Call a delay of 1000 ms
	}
} 


