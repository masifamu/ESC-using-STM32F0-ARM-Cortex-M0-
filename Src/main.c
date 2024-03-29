/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bldc.h"
#include "stdio.h"
#include "string.h"
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
#ifdef UART_COMM_DEBUG
char printDataString[100] = "buffer here\r\n";//{'\0',};
#endif

uint16_t ADCBuffer[6]={0,};
extern uint32_t time;
extern uint8_t toUpdate;
uint32_t localTime=0;
#ifdef UART_COMM_DEBUG
extern uint16_t noOfHSCuts;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t pwmWidth=0;
	uint16_t throtle=0;
	float filterPWMWidth=0.0f;
#ifdef MEASURE_POWER
	uint16_t battVoltage=0,currentDrawn=0;
	float powerConsumed=0.0f;
#endif
#ifdef UART_COMM_DEBUG
	uint8_t hour=0,minute=0,second=0,rpm;
	uint16_t procTemp=0,heatSinkTemp=0,procVolt=0;
	uint32_t msStampS=0;
#endif
#ifdef MEASURE_POWER
	uint32_t msStampV=0,timeStampPower=0;
#endif
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc,(uint32_t*)&ADCBuffer,6);
	
	BLDC_Init();
  /* USER CODE END 2 */
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	
#ifdef UART_COMM_DEBUG
	msStampS=time;
#endif
#ifdef MEASURE_POWER
	msStampV=time;
	timeStampPower = time;
#endif
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		throtle=ADCBuffer[0];
		//Checking for throttle accident management
		if(!isThrotleProperlyConnected(time,throtle)){
			while(1){
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
				HAL_Delay(200);
			}
		}
#ifdef MEASURE_POWER
		//measuring battery voltage
		//anding is to avoid the error due to two lower bits. 3.3*(15.6+1)*1000=54780, shifting right by 12bit = div by 4096
		battVoltage = (uint16_t)(((uint32_t)ADCBuffer[1] * 54780)>>12)/1000 ;
		if(battVoltage < minBattThreVolt | battVoltage > maxBattThreVolt){
			if(time-msStampV >= waitAftLowVoltDet){//if voltage is out of range for 5sec then turn everything off and blink PB3 and PB4 together at 500ms.
				BLDC_MotorStop();
				while(1){
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);
					HAL_Delay(500);
				}
			}
			msStampV=time;
		}
		
		//measuring current
		currentDrawn = getCurrentDrawn(ADCBuffer[2]);
		
		//measuring power consumed
		
		powerConsumed += ((((float)battVoltage*currentDrawn)*(float)(time-timeStampPower))/3600000.0f);
		timeStampPower = time;
#endif
#ifdef UART_COMM_DEBUG
		//measuring heatSink temperature
		heatSinkTemp = getHeatSinkTemp(ADCBuffer[3]);
		
		//measuring processor temperature 
		procTemp = getProcTemp(ADCBuffer[4]);

		//measuring internal processor voltage
		procVolt = getProcVoltage(ADCBuffer[5]);
		
		//measuring ON-time
		hour = (uint8_t)(((time/1000)/60)/60);
		minute = (uint8_t)(((time/1000)/60)%60);
		second = (uint8_t)((time/1000)%60);
		
		//meauring RPM at every 1sec interval
		if(time-msStampS >=1000){
			rpm=(uint16_t)((noOfHSCuts*60)/HSCutsInOneCycle);
			noOfHSCuts=0;
			msStampS=time;
		}
	
		snprintf(printDataString,100, "PWM=%3d TH=%4d V=%2d %1d:%2d:%2d RPM=%3d C=%5d PC=%8.1f PT=%2d PV=%4d HST=%2d\n\r", pwmWidth,throtle, battVoltage,hour,minute,second,rpm,currentDrawn,powerConsumed,procTemp,procVolt,heatSinkTemp);
		HAL_UART_Transmit(&huart1, (uint8_t*)printDataString, strlen(printDataString), HAL_MAX_DELAY);
#endif
		//HAL_Delay(150);//to avoid the sound problem//check why?
		//motor control block
    if (throtle > BLDC_ADC_START) {
			if (BLDC_MotorGetSpin() == BLDC_STOP) {
				// Check Reverse pin
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != 0) {
					// Forward
					BLDC_MotorSetSpin(BLDC_CW);
					BLDC_MotorSetStopDirection(BLDC_CW);
				}else{
					// Backward
					BLDC_MotorSetSpin(BLDC_CCW);
					BLDC_MotorSetStopDirection(BLDC_CCW);
				}
				pwmWidth=8;//BLDC_ADCToPWM(throtle);
				BLDC_SetPWM(pwmWidth);
				BLDC_MotorCommutation(BLDC_HallSensorsGetPosition());
				BLDC_UpdatePWMWidth(toUpdate);
				for(int i=0;i<10;i++){
					pwmWidth += 1;
					BLDC_SetPWM(pwmWidth);
					HAL_Delay(10);
				}
				//HAL_Delay(1000);
				filterPWMWidth=pwmWidth;//to maintain smooth transition in pwmwidth value
			}
			
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			if(filterPWMWidth < pwmWidth) filterPWMWidth += 0.05f; else filterPWMWidth -= 0.05f;
    	pwmWidth=BLDC_ADCToPWM(throtle);
			BLDC_SetPWM((uint16_t)filterPWMWidth);
    }else{
			if (BLDC_MotorGetSpin() != BLDC_STOP) {
				//meaning motor is still running
				if (throtle < BLDC_ADC_STOP) {
					BLDC_MotorStop();
					pwmWidth = 0;
					filterPWMWidth = 0.0f;
					BLDC_MotorSetSpin(BLDC_STOP);
				}
			}
			toggleGreenLED();
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
