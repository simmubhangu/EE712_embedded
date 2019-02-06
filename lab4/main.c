#include"commonheader.h"
#include "lcd.h"
#include "adc_external.h"


int main(void)
{

    ADC_config(ch_0);
    lcd_init();
    lcd_string("Pot");
    while(1)
    {
        uint32_t value = ADC_conversion(ch_0);
//        ui32TempValueC = (147.5 - ((75 * 3.3 * value)) / 4096);
//        ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
        lcd_print(2,1,value,4);
    }
}
