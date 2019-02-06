
#include "commonheader.h"
#include "adc_external.h"
#include "lcd.h"


#define NUM_SSI_DATA 2
const uint8_t DataTx[NUM_SSI_DATA] ={0x80, 0xD0};


void SSI_configure(void)

{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
//    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 10000, 16);
    SSIEnable(SSI0_BASE);
}


int main(void)
{
  //uint32_t ui32Index;
    uint8_t Data;
    uint8_t i=0;
    SSI_configure();
    lcd_init();
    ADC_config(ch_0);

    while(1)
    {


        for( i= 0; i < NUM_SSI_DATA; i++)
        {
            if (i==1)
                Data = DataTx[i];
            else
                Data = DataTx[i];
            SSIDataPut(SSI0_BASE, Data);
            while(SSIBusy(SSI0_BASE));

        }
        uint16_t value = ADC_conversion(ch_0);
        lcd_print(2,1,value,4);
    }
}
