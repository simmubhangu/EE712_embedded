/*
 * adc.c
 *
 *  Created on: 06-Feb-2019
 *      Author: simmu
 */
#include "commonheader.h"
#include "adc_external.h"

void ADC_config(char channel)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, channel);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, channel);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, channel);
    ADCSequenceStepConfigure(ADC0_BASE,1,3,channel|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
}

unsigned int ADC_conversion(char channel_number)
{
    ADCIntClear(ADC0_BASE, 1);
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false));
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    ui32Avg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
    return ui32Avg;
}


