/*
 * adc.h
 *
 *  Created on: 06-Feb-2019
 *      Author: simmu
 */

#ifndef ADC_EXTERNAL_H_
#define ADC_EXTERNAL_H_

uint32_t ui32ADC0Value[4];
volatile uint32_t ui32Avg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;

#define ch_0 ADC_CTL_CH0   //PE3
#define ch_1 ADC_CTL_CH1   //PE2
#define ch_2 ADC_CTL_CH2   //PE1
#define ch_3 ADC_CTL_CH3   //PE0
#define ch_temp ADC_CTL_TS   //temprature sensor


void ADC_config(char channel);
unsigned int ADC_conversion(char channel_number);


#endif /* ADC_EXTERNAL_H_ */
