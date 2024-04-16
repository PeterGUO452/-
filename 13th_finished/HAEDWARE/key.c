#include "key.h"

uint8_t  key_scan(void)
{	
	if( !HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) )
	{
		HAL_Delay(10);
		if( !HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) )
		{
			while(1)
			{
				
			}
			return B4;
		}
	}
	if( !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) )
	{
		HAL_Delay(10);
		if( !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) )
		{
			while(1)
			{
				
			}
			return B1;
		}
	}
	if( !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) )
	{
		HAL_Delay(10);
		if( !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) )
		{
			while(1)
			{
				
			}
			return B2;
		}
	}
	if( !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) )
	{
		HAL_Delay(10);
		if( !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) )
		{
			while(1)
			{
				
			}
			return B3;
		}
	}
	return 0;
}