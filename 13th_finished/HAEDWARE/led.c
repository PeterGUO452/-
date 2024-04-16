#include "led.h"
uint8_t old_status = 0x00;
uint8_t new_status  = 0x00;

uint16_t led_pin[] = {GPIO_PIN_8,GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15};

void led_cotl(uint8_t led,uint8_t on_off)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	if(on_off)
	{
		//¹ØµÆ ÖÃ1
		new_status |= led;
		
	}else
	{
		//¿ªµÆ ÖÃ0
		new_status &= ~led;
	}
	uint8_t i=0,j=0;
	for(i=0;i<8;i++)
	{
		j = new_status & (1<<i);
		if(j)
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		}else
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
		}
	}
	old_status = new_status;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}