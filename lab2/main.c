/*

* Author: Texas Instruments

* Editted by: Simranjeet singh
          Wel Lab, EE Department, IIT Bombay

* Description: This code will test file to check software and hardware setup. This code will blink three colors of RGB LED present of Launchpad board in sequence.

* Filename: lab-02.c

* Functions: setup(), led_pin_config(), main()

* Global Variables: none

*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"


unsigned int x[8];
unsigned int y[8];


/*

* Function Name: setup()

* Input: none

* Output: none

* Description: Set crystal frequency and enable GPIO Peripherals

* Example Call: setup();

*/
void setup(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
}

/*

* Function Name: led_pin_config()

* Input: none

* Output: none

* Description: Set PORTF Pin 1, Pin 2, Pin 3 as output. On this pin Red, Blue and Green LEDs are connected.

* Example Call: led_pin_config();

*/

void led_pin_config(void)
{
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
}

/*

* Function Name: led_pin_config()

* Input: none

* Output: none

* Description: Set PORTF Pin 1, Pin 2, Pin 3 as output. On this pin Red, Blue and Green LEDs are connected.

* Example Call: led_pin_config();

*/

void switch_pin_config(void)
{
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}
/*

* Function Name: led_glow()

* Input: Porta portb

* Output: none

* Example Call: led_pin_config();

*/

void led_glow(uint8_t porta, uint8_t portb)
{
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0,porta);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0, portb);

}

void read_switch(void)
{
   x[0] =  GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_3);
   x[1] =  GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_4);
   x[2] =  GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_5);
   x[3] =  GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3);
   x[4] =  GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_4);
   x[5] =  GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_5);
   x[6] =  GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6) ;
   x[7] =  GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_7) ;
}

void alu_logic(void)

{
    uint8_t i=0;

    for (i=6;i<=7;i++)
    {
        y[i] = x[i] >>3;
    }

    if (x[7] == 0x00 && x[6] == 0x00) //copy
    {
         uint8_t i=0;

        for (i=0;i<=5;i++)
        {
            y[i] = x[i] >>3;
        }
    }
    else if (x[7] == 0x00 && x[6] != 0x00) //add
    {
        uint8_t i=0, first_oprand = 0, second_oprand=0;

        for (i=0;i<=2;i++)
        {
            first_oprand =(first_oprand | x[i] >>3);

        }
        uint8_t j=0, add_result;
        for (j=3;j<=5;j++)
        {
            second_oprand =(second_oprand | x[j] >>3);

        }

        add_result = first_oprand + second_oprand;

        led_glow(((add_result & 0x38) >>3),(add_result & 0x07));

    }
    else if (x[7] != 0x00 && x[6] == 0x00) //substract
    {
        uint8_t i=0, first_oprand = 0, second_oprand=0;

        for (i=0;i<=2;i++)
        {
            first_oprand =(first_oprand | x[i] >>3);

        }
        uint8_t j=0, sub_result;
        for (j=3;j<=5;j++)
        {
            second_oprand =(second_oprand | x[j] >>3);

        }

        sub_result = first_oprand + (~second_oprand +1);

        led_glow(((sub_result & 0x38) >>3),(sub_result & 0x07));

    }
    else //multiply
    {
        uint8_t i=0, first_oprand = 0, second_oprand=0;

        for (i=0;i<=2;i++)
        {
           first_oprand =(first_oprand | x[i] >>3);

        }
        uint8_t j=0, mul_result;
        for (j=3;j<=5;j++)
        {
           second_oprand =(second_oprand | x[j] >>3);

        }

        mul_result = first_oprand + (~second_oprand +1);

        led_glow(((mul_result & 0x38) >>3),(mul_result & 0x07));


    }


}
int main(void)
{
    uint8_t ui8LED = 2;

    setup();
    led_pin_config();
    switch_pin_config();

    while(1)
    {
        read_switch();
        alu_logic();
        // Turn on the LED
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8LED);
        //led_glow(0x00,0x00);
        // Delay for a bit
        SysCtlDelay(2000);
        // Cycle through Red, Green and Blue LEDs
        if (ui8LED == 8)
        {
            ui8LED = 2;
        }
        else
        {
            ui8LED =ui8LED*2;
        }
    }
}
