#ifndef _BLDC_LIB_H_
#define _BLDC_LIB_H_

#include "stdint.h"
// PWM Frequency = 72000000/BLDC_CHOPPER_PERIOD
#define BLDC_CHOPPER_PERIOD 480
#define BLDC_SPEEDING_FACTOR 0.5
// Dead time = BLDC_NOL/72000000  (on 72MHz: 7 is 98ns)
// (on 72MHz: 72 is 1000ns)
//#define BLDC_NOL 72
#define BLDC_NOL 72

#define BLDC_PWMTOPKEYS
//#define BLDC_PWMBOTTOMKEYS
//#define BLDC_PWMTOPBOTTOMKEYS
//#define BLDC_PWMCOMPLEMENTARYMODE

//#define UART_COMM_DEBUG
//#define UART_HALL_DEBUG


/* when working with variable resistor
#define BLDC_ADC_START 15
#define BLDC_ADC_STOP 5
#define BLDC_ADC_MAX 4000
*/
//when working with throtle
#define BLDC_ADC_START 1150
#define BLDC_ADC_STOP 1090
#define BLDC_ADC_MAX 4000
/////////////////////////////////////////////////////////////////////////
#define BLDC_STOP	0
#define BLDC_CW		1
#define BLDC_CCW	2

//motor parameter macros
#define minBattThreVolt           36
#define maxBattThreVolt           56
#define waitAftLowVoltDet         5000//in msec
#define wheelDia                  0.65
#define HSCutsInOneCycle          266

//thermistor parameter
#define R1										100710//100k
#define coeffA                0.003354016
#define coeffB								0.0002569850
#define coeffC								0.000002620131
#define coeffD								0.00000006383091
#define rTherm25C							100000


#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

void BLDC_Init(void);
void BLDC_HallSensorsInit(void);
void BLDC_PWMTimerInit(void);
uint8_t BLDC_HallSensorsGetPosition(void);
void BLDC_MotorSetSpin(uint8_t spin);
uint8_t BLDC_MotorGetSpin(void);
void BLDC_MotorStop(void);
void BLDC_MotorStart(void);
void BLDC_MotorCommutation(uint16_t hallpos);
uint16_t BLDC_ADCToPWM(uint16_t ADC_VALUE);
void BLDC_SetPWM(uint16_t PWM);
void usart_init(void);
void USARTSend(char *);
uint16_t getHeatSinkTemp(uint16_t adcBuffer3);
uint16_t getProcVoltage(uint16_t adcBuffer5);
uint16_t getProcTemp(uint16_t adcBuffer4);
uint16_t getCurrentDrawn(uint16_t adcBuffer2);
void toggleGreenLED(void);
#endif
