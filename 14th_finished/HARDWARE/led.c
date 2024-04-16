#include "led.h"

uint8_t old_status=0x00;
uint8_t new_status=0x00;

//#define D1  	   
//#define D2	GPIO_PIN_9   
//#define D3	GPIO_PIN_10  
//#define D4	GPIO_PIN_11  
//#define D5	GPIO_PIN_12  
//#define D6	GPIO_PIN_13  
//#define D7	GPIO_PIN_14  
//#define D8	GPIO_PIN_15  
uint32_t led_pin[8] = {GPIO_PIN_8,GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15};

void led_col(uint32_t led,uint8_t on_off)
{
	uint8_t i=0,j=0;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	if(on_off == LED_ON)//÷√0
	{
		new_status &= ~led;
	}else//÷√1
	{
		new_status |= led;
	}
	for(i=0;i<8;i++)
	{
		j = new_status&(1<<i);
		HAL_GPIO_WritePin(GPIOC,led_pin[i],j);
	}
	old_status = new_status;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	
}