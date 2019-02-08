
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

    SSIEnable(SSI0_BASE);
}

void write_data(bool mode, unsigned int value)

{
    uint8_t configure_bits = 0x03 | mode << 3;

    uint8_t firstbyte = configure_bits << 4 | (value & 0x0F00) >> 8;

    uint8_t secondByte = (value & 0xFF);

    SSIDataPut(SSI0_BASE, firstbyte);
    SSIDataPut(SSI0_BASE, secondByte);
    while(SSIBusy(SSI0_BASE));


}


int main(void)
{
  //uint32_t ui32Index;
//    uint8_t Data;
//    uint8_t i=0;
    SSI_configure();
    lcd_init();
    ADC_config(ch_0);

    while(1)
    {

        //write_data(1,100);

        uint16_t value = ADC_conversion(ch_0);

        lcd_print(2,1,value,4);
        write_data(1,value);
        SysCtlDelay(67000);
    }
}
