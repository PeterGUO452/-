#include "key.h"
#include "main.h"

extern uint32_t k4_down_cnt;
extern uint8_t long_down_flag;
extern uint8_t duty_lock_flag;
extern uint8_t show_flag ;
uint8_t key_scan(void)
{
	//K4 PA0
	if(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
	{
		HAL_Delay(10);
		k4_down_cnt = 1;
		while(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
		{
			
		}
		
		if(k4_down_cnt >=200)
		{
			//长按了两秒以上
			long_down_flag=1;
		}else
		{
			long_down_flag=0;
		}
		
		
			return K4;
	}
	//K1 PB0
	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0))
	{
		HAL_Delay(10);
		while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0))
		{
		}
			return K1;
	}
	//K2 PB1
	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1))
	{
		HAL_Delay(10);
		while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1))
		{
		}
			return K2;
	}
	//K3 PB2
	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2))
	{
		HAL_Delay(10);
		while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2))
		{
		}
			return K3;
	}
	return 0;

}