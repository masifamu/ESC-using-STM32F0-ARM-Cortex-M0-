#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"
#include "stdio.h"
#include "bldc.h"
#include "math.h"


#define TIM1CH1(x) TIM1->CCR1=x
#define TIM1CH2(x) TIM1->CCR2=x
#define TIM1CH3(x) TIM1->CCR3=x

#define CH1 1
#define CH2 2
#define CH3 3

#ifdef UART_HALL_DEBUG
char printDataString1[50] = "buffer here\r\n";
#endif

uint8_t BLDC_MotorSpin = 0,toUpdate=0;
uint8_t BLDC_STATE[6] = {0,0,0,0,0,0};
uint16_t PWMWIDTH=0;

#ifdef UART_COMM_DEBUG
uint16_t noOfHSCuts=0;
#endif

#ifndef BLDC_PWMCOMPLEMENTARYMODE
uint8_t BLDC_STATE_PREV[6] = {0,0,0,0,0,0};
#endif

// BLDC motor steps tables
static const uint8_t BLDC_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	1,0	,	0,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

// BLDC motor backwared steps tables
static const uint8_t BLDC_BRIDGE_STATE_BACKWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,1	,	1,0	,	0,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

uint16_t getCurrentDrawn(uint16_t adcBuffer2){
	#ifdef DO_CURRENT_AVGING
	static uint32_t avg2=0;
	static uint8_t count2=0;
	static uint16_t current=0;
	if(++count2 <= 4){
		avg2 += adcBuffer2;
	}else{
		count2=0;
		avg2 = avg2>>2;//5 right shift = div by 32
		current = (uint16_t)(((uint32_t)((float)avg2*((Rlow+Rup)/Rlow)*3300.0f))>>12);
		current = (uint16_t)((float)(current - 2550)/sensitivity*1000);
		avg2=0;
	}
	#endif
	#ifndef DO_CURRENT_AVGING
	uint16_t current=0;
	current = (uint16_t)(((uint32_t)((float)adcBuffer2*((Rlow+Rup)/Rlow)*3300.0f))>>12);
	current = (uint16_t)((float)(current - 2550)/sensitivity*1000);
	#endif
	return current;
}

#ifdef UART_COMM_DEBUG
uint16_t getHeatSinkTemp(uint16_t adcBuffer3){
	static uint32_t sum=0;
	static uint8_t count=0;
	static uint16_t temperature=0;
	if(++count <= 200){
		sum += adcBuffer3;
	}else{
		count=0;
		sum /=200;
		float HSVoltK=0, Vth=0.0,Rth=0.0;
		Vth = 3300 - ((sum*3300)>>12);
		Rth = (R1 * Vth)/(3300 - Vth);
		Vth = log(Rth/rTherm25C);//using Vth agian to save memory
		HSVoltK = 1.0/(coeffA+coeffB*Vth+coeffC*pow(Vth,2)+coeffD*pow(Vth,3));
		
		sum=0;
		temperature = ((uint16_t)(HSVoltK-273.15));
	}
	return temperature;
}

uint16_t getProcVoltage(uint16_t adcBuffer5){
	// 1530 is corresponding to the 1.2326v
	return ((uint16_t)((uint32_t)(3300*1530)/adcBuffer5));
}
uint16_t getProcTemp(uint16_t adcBuffer4){
	static uint32_t avg4=0;
	static uint8_t count4=0;
	static uint16_t pTemp=0;
	if(++count4 <= 16){
		avg4 += adcBuffer4;
	}else{
		count4 = 0;
		avg4 = avg4 >> 4;
		//sensitivity=4.3mV/C=5steps, 1770 is default value at 25C, 25 is addedd to get the actual value
		pTemp = ((uint16_t)((1770-(uint16_t)avg4)/5+25));
		avg4 = 0;
	}
	return pTemp;
}
#endif

void toggleGreenLED(void){
	static uint16_t counter=0;
	if(++counter > 32){
		counter=0;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	}
}

void BLDC_Init(void) {
	BLDC_MotorStart();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
  //interrupt on pins are being reset from the EXTI interrupt handler in stm32f0xx_it.c  
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
	BLDC_MotorCommutation(BLDC_HallSensorsGetPosition());
	
	#ifdef UART_COMM_DEBUG
	noOfHSCuts++;
	#endif
}

uint8_t BLDC_HallSensorsGetPosition(void) {
	uint8_t temp=(uint8_t)((GPIOB->IDR) & (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7))>>5;
	return temp;
}

uint8_t BLDC_MotorGetSpin(void) {
	return BLDC_MotorSpin;
}

void BLDC_MotorSetSpin(uint8_t spin) {
	BLDC_MotorSpin = spin;
}

void BLDC_MotorStop(void)
{
	BLDC_SetPWM(1);
}

void BLDC_MotorStart(void)
{
		BLDC_SetPWM(0);
	
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); 

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

		BLDC_MotorSpin = 0;
		#ifndef BLDC_PWMCOMPLEMENTARYMODE
			memset(BLDC_STATE_PREV, 0, sizeof(BLDC_STATE_PREV));
		#endif
}

#ifdef BLDC_PWMTOPBOTTOMKEYS
void BLDC_MotorCommutation(uint16_t hallpos)
{
	if (BLDC_MotorSpin == BLDC_CW) {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_FORWARD[hallpos], sizeof(BLDC_STATE));
	}
	else {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_BACKWARD[hallpos], sizeof(BLDC_STATE));
	}

	// Disable if need
	if (!BLDC_STATE[UH]) TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	if (!BLDC_STATE[UL]) TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	if (!BLDC_STATE[VH]) TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	if (!BLDC_STATE[VL]) TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	if (!BLDC_STATE[WH]) TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	if (!BLDC_STATE[WL]) TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (BLDC_STATE[UH] & !BLDC_STATE[UL] & !BLDC_STATE_PREV[UH]) TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	if (BLDC_STATE[UL] & !BLDC_STATE[UH] & !BLDC_STATE_PREV[UL]) TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	if (BLDC_STATE[VH] & !BLDC_STATE[VL] & !BLDC_STATE_PREV[VH]) TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	if (BLDC_STATE[VL] & !BLDC_STATE[VH] & !BLDC_STATE_PREV[VL]) TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	if (BLDC_STATE[WH] & !BLDC_STATE[WL] & !BLDC_STATE_PREV[WH]) TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	if (BLDC_STATE[WL] & !BLDC_STATE[WH] & !BLDC_STATE_PREV[WL]) TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

	memcpy(BLDC_STATE_PREV, BLDC_STATE, sizeof(BLDC_STATE));
}
#endif

#ifdef BLDC_PWMTOPKEYS
void BLDC_MotorCommutation(uint16_t hallpos)
{
	if (BLDC_MotorSpin == BLDC_CW) {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_FORWARD[hallpos], sizeof(BLDC_STATE));
	}
	else {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_BACKWARD[hallpos], sizeof(BLDC_STATE));
	}

	// Disable if need
	if (!BLDC_STATE[UH]) TIM1CH3(0);
	if (!BLDC_STATE[UL]) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	if (!BLDC_STATE[VH]) TIM1CH2(0);
	if (!BLDC_STATE[VL]) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	if (!BLDC_STATE[WH]) TIM1CH1(0);
	if (!BLDC_STATE[WL]) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (BLDC_STATE[UH] & !BLDC_STATE[UL] & !BLDC_STATE_PREV[UH]) {
		TIM1CH3(PWMWIDTH); 
		toUpdate=CH3;
	}
	if (BLDC_STATE[UL] & !BLDC_STATE[UH] & !BLDC_STATE_PREV[UL]) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	if (BLDC_STATE[VH] & !BLDC_STATE[VL] & !BLDC_STATE_PREV[VH]) {
		TIM1CH2(PWMWIDTH); 
		toUpdate=CH2;
	}
	if (BLDC_STATE[VL] & !BLDC_STATE[VH] & !BLDC_STATE_PREV[VL]) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	if (BLDC_STATE[WH] & !BLDC_STATE[WL] & !BLDC_STATE_PREV[WH]) {
		TIM1CH1(PWMWIDTH); 
		toUpdate=CH1;
	}
	if (BLDC_STATE[WL] & !BLDC_STATE[WH] & !BLDC_STATE_PREV[WL]) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	memcpy(BLDC_STATE_PREV, BLDC_STATE, sizeof(BLDC_STATE));
}
#endif

#ifdef BLDC_PWMBOTTOMKEYS
void BLDC_MotorCommutation(uint16_t hallpos)
{
	if (BLDC_MotorSpin == BLDC_CW) {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_FORWARD[hallpos], sizeof(BLDC_STATE));
	}
	else {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_BACKWARD[hallpos], sizeof(BLDC_STATE));
	}

	// Disable if need
	if (!BLDC_STATE[UH]) TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	if (!BLDC_STATE[UL]) TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	if (!BLDC_STATE[VH]) TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	if (!BLDC_STATE[VL]) TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	if (!BLDC_STATE[WH]) TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	if (!BLDC_STATE[WL]) TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (BLDC_STATE[UH] & !BLDC_STATE[UL] & !BLDC_STATE_PREV[UH]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	}
	if (BLDC_STATE[UL] & !BLDC_STATE[UH] & !BLDC_STATE_PREV[UL]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	}
	if (BLDC_STATE[VH] & !BLDC_STATE[VL] & !BLDC_STATE_PREV[VH]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	}
	if (BLDC_STATE[VL] & !BLDC_STATE[VH] & !BLDC_STATE_PREV[VL]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	}
	if (BLDC_STATE[WH] & !BLDC_STATE[WL] & !BLDC_STATE_PREV[WH]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	}
	if (BLDC_STATE[WL] & !BLDC_STATE[WH] & !BLDC_STATE_PREV[WL]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	}

	memcpy(BLDC_STATE_PREV, BLDC_STATE, sizeof(BLDC_STATE));
}
#endif



#ifdef BLDC_PWMCOMPLEMENTARYMODE
void BLDC_MotorCommutation(uint16_t hallpos) {
	if (BLDC_MotorSpin == BLDC_CW) {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_FORWARD[hallpos], sizeof(BLDC_STATE));
	}
	else {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_BACKWARD[hallpos], sizeof(BLDC_STATE));
	}

	// PWM at low side FET of bridge U
	// active freewheeling at high side FET of bridge U
	// if low side FET is in PWM off mode then the hide side FET
	// is ON for active freewheeling. This mode needs correct definition
	// of dead time otherwise we have shoot-through problems

	if (BLDC_STATE[UH]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	} else {
		// Low side FET: OFF
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		if (BLDC_STATE[UL]){
			// High side FET: ON
			TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
			TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
		} else {
			// High side FET: OFF
			TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		}
	}

	if (BLDC_STATE[VH]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	} else {
		// Low side FET: OFF
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		if (BLDC_STATE[VL]){
			// High side FET: ON
			TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);
			TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
		} else {
			// High side FET: OFF
			TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		}
	}

	if (BLDC_STATE[WH]) {
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	} else {
		// Low side FET: OFF
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		if (BLDC_STATE[WL]){
			// High side FET: ON
			TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
			TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
		} else {
			// High side FET: OFF
			TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		}
	}

}
#endif

uint16_t BLDC_ADCToPWM(uint16_t ADC_VALUE) {
	uint32_t tmp;

	if (ADC_VALUE < BLDC_ADC_STOP) {
		return 0;
	}

	if (ADC_VALUE > BLDC_ADC_MAX) {
		return (BLDC_CHOPPER_PERIOD * BLDC_SPEEDING_FACTOR)+1;
	}

	tmp = (uint32_t)(ADC_VALUE-BLDC_ADC_STOP) * (uint32_t)(BLDC_CHOPPER_PERIOD * BLDC_SPEEDING_FACTOR) / (uint32_t)(BLDC_ADC_MAX - BLDC_ADC_START);

	//to maintain the lower PWM width
	return (uint16_t) tmp-3;

}

void BLDC_SetPWM(uint16_t PWM)
{
	PWMWIDTH=PWM;
	if(toUpdate == CH1){
		TIM1CH1(PWMWIDTH);
	}else if(toUpdate == CH2){
		TIM1CH2(PWMWIDTH);
	}else if(toUpdate == CH3){
		TIM1CH3(PWMWIDTH);
	}
	#ifdef UART_HALL_DEBUG
  snprintf(printDataString1,50, "PWMWIDTH = %d\n\r", (uint16_t)PWMWIDTH);
  HAL_UART_Transmit(&huart1, (uint8_t*)printDataString1, strlen(printDataString1), HAL_MAX_DELAY);
	#endif
}
