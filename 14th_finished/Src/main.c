/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "led.h"
#include "key.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t read_r37(void);
void key_proc(uint8_t key);
void duty_proc(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DATA 1
#define PARA 2
#define RECD 3

uint8_t show_flag = DATA;

#define H_MODE 1
#define L_MODE 2
#define TO_L 1
#define TO_H 2
uint8_t cur_mode = L_MODE;
uint8_t mode_switch_dir=0;
uint8_t had_switch=0;
uint32_t mode_cnt=0;

uint8_t S5_flag=0;

uint8_t R_flag=0,K_flag=0;
uint8_t R=1,K=1;
uint8_t N=0;

uint8_t long_down_flag = 0;
uint8_t duty_lock_flag=0;

uint8_t led2_status = 0;

uint32_t k4_down_cnt=0;

float duty=0;
float V,pi=3.14,f,Max_H=0,Max_L=0;
extern uint16_t ccr1,ccr2;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
  LCD_Init();

  
  HAL_UART_Receive_IT(&huart1,&ch,1);
  
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	printf("READY\r\n");

	uint8_t key;
	uint16_t tim2_ccr=htim2.Instance->CCR2;
	uint16_t tim2_arr=htim2.Instance->ARR + 2;
	
	float F,D;
	
	char lcd_data[40];
//	led_col(D1,LED_ON);
//	led_col(D2,LED_OFF);
//	led_col(D3,LED_OFF);
//	led_col(D4,LED_OFF);
//	led_col(D5,LED_OFF);
//	led_col(D6,LED_OFF);
//	led_col(D7,LED_OFF);
//	led_col(D8,LED_OFF);

	GPIOC->BSRR = 0xFFFFFFFF;
	
	HAL_Delay(500);
	
	GPIOC->BSRR = 0x00;
	while (1)
	{
		
		
		key = key_scan();
		key_proc(key);
		duty_proc();
		
		if(show_flag == DATA)
		{
			sprintf(lcd_data,"        DATA     ");
			LCD_DisplayStringLine(Line4,(uint8_t*)lcd_data);
			
			if(cur_mode == H_MODE)
			{
				sprintf(lcd_data,"     M = H     ");
			}else
			{
				sprintf(lcd_data,"     M = L     ");
			}
			LCD_DisplayStringLine(Line5,(uint8_t*)lcd_data);
			
			sprintf(lcd_data,"     P = %d%%     ",(int)(duty*100));
			LCD_DisplayStringLine(Line6,(uint8_t*)lcd_data);
			
			sprintf(lcd_data,"     V = %.1f     ",V);
			LCD_DisplayStringLine(Line7,(uint8_t*)lcd_data);

		}else if(show_flag == PARA)
		{
			
			
			sprintf(lcd_data,"        PARA     ");
			LCD_DisplayStringLine(Line4,(uint8_t*)lcd_data);
			
			sprintf(lcd_data,"     R = %d     ",R);
			LCD_DisplayStringLine(Line5,(uint8_t*)lcd_data);
			
			sprintf(lcd_data,"     K = %d     ",K);
			LCD_DisplayStringLine(Line6,(uint8_t*)lcd_data);
		
		
		}else
		{
			sprintf(lcd_data,"        RECD     ");
			LCD_DisplayStringLine(Line4,(uint8_t*)lcd_data);
			
			sprintf(lcd_data,"     N = %d     ",N);
			LCD_DisplayStringLine(Line5,(uint8_t*)lcd_data);
			
			sprintf(lcd_data,"     MH = %.1f     ",Max_H);
			LCD_DisplayStringLine(Line6,(uint8_t*)lcd_data);
			
			sprintf(lcd_data,"     ML = %.1f     ",Max_L);
			LCD_DisplayStringLine(Line7,(uint8_t*)lcd_data);
		
		}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t read_r37(void)
{
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,50);
	return HAL_ADC_GetValue(&hadc2);
}

void key_proc(uint8_t key)
{
	if(key)
	{
		switch(key)
		{
			

			case K1:
				{
					if(show_flag == DATA)
					{
						show_flag = PARA;
						R_flag = 1;
						K_flag = 0;

					}else if(show_flag == PARA)
					{

						show_flag = RECD;
						
						//从参数界面转数据界面时要使所有参数生效
					}else
					{
						show_flag = DATA;
						
					}
					if(show_flag == DATA)
					{
						led_col(D1,LED_ON);
					}else
					{
						led_col(D1,LED_OFF);
					}
					LCD_Clear(Black);
				}break;
			case K2:
				{
					
					if(show_flag == DATA)
					{
						//高低频模式选择界面
						if(cur_mode == H_MODE)
						{
							cur_mode = L_MODE;
							mode_switch_dir = TO_L;//高变低
						}else
						{
							mode_switch_dir = TO_H;//低变高
							cur_mode = H_MODE;
						}
						
						
					}else if(show_flag == PARA)
					{
						//参数选择界面
						if(R_flag)
						{
							R_flag = 0;
							K_flag = 1;
						}else
						{
							R_flag = 1;
							K_flag = 0;
						}
					
					}
				}break;	
			case K3:
				{
					if(show_flag == PARA)
					{
						//参数加键
						if(R_flag)
						{
							R = ((R)%11)+1;
						}else
						{
							K = ((K)%11)+1;
						}
					}
						
						
				}break;
			case K4:
				{
					if(show_flag == PARA)
					{
						//参数减键
						if(R_flag)
						{
							R-=1;
							if(!R)
								R=10;
						}else
						{
							K-=1;
							if(!K)
								K=10;
						}
					}else if(show_flag == DATA)
					{
						//长按B4 2s锁定占空比
						if(long_down_flag)	
						{
							duty_lock_flag = 1;
						}else
						{
							duty_lock_flag = 0;
						}
						

					}
				}break;

		}
		
		
		if(duty_lock_flag)
		{
			led_col(D3,LED_ON);
		}else
		{
			led_col(D3,LED_OFF);
		}
	}
	
}

void duty_proc(void)
{
	float vR37;
	if(!duty_lock_flag)
	{
		vR37 = read_r37()*3.3/4096;
		
		if(vR37 <= 1)
		{
			duty = 0.1;
			
		}
		else if(vR37 >= 3)
		{
			duty = 0.85;
		}
		else
		{
			duty = 0.375*(vR37 - 1.0) + 0.1;
		}
		TIM2->CCR2 = (uint16_t)(duty * TIM2->ARR);
		//printf("Vr37:%.1f, CCR2:%d\r\n",vR37,TIM2->CCR2);
	}
}

uint32_t cnt=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		//定时器溢出中断，中断时间是10ms
		cnt++;
		k4_down_cnt+=1;
		if(had_switch)
		{
			if(mode_switch_dir == 0)
			{
				mode_cnt++;
			}
			
			if((mode_cnt>=200))
			{
				N+=1;
				if(cur_mode == H_MODE)
				{
					if(Max_H < V)
						Max_H = V;
					
				}else
				{
					if(Max_L < V)
						Max_L = V;
					
				}
				mode_cnt = 0;
				had_switch =0;
			}

			
		}


	}
	if((cnt%10) == 0)
	{
		//100ms
		if(mode_switch_dir)
		{
			if(mode_switch_dir == TO_L)
			{
				
				//TIM2->PSC = (TIM2->PSC>=79) ? 79 : (TIM2->PSC++);
				if(TIM2->PSC<=79)
				{
					TIM2->PSC+=1;
				}else
				{
					TIM2->PSC=79;
				}
				
			}else if(mode_switch_dir == TO_H)
			{
//				TIM2->PSC = (TIM2->PSC<=39) ? 39 : (TIM2->PSC--);
				if(TIM2->PSC>=39)
				{
					TIM2->PSC-=1;
				}else
				{
					TIM2->PSC=39;
				}
			}
			
			if((TIM2->PSC == 39) || TIM2->PSC == 79)
			{
				led_col(D2,LED_OFF);
				mode_switch_dir = 0;
				mode_cnt=0;
				had_switch = 1;
			}
			//printf("PSC:%d, dir:%d,had:%d\r\n",TIM2->PSC,mode_switch_dir,had_switch);
		}
		
		if(mode_switch_dir)
		{
			led2_status = !led2_status;
			if(led2_status)
			{
				led_col(D2,LED_ON);
			}else
			{
				led_col(D2,LED_OFF);
			}
		}
	}
	

}


uint16_t ccr1,ccr2;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		
		ccr1 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		ccr2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		
		f = 10000000*1.0/ccr2;
		V = (f*2*pi*R) / (100 * K);
		
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
