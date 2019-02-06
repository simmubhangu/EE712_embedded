/*
 * switch_handler.c
 *
 *  Created on: 25-Jan-2019
 *      Author: simmu
 */


#include "commonheader.h"
#include "lcd.h"
#include "switch_handler.h"

#define read_sw1 0x01
#define read_sw2 0x10

uint32_t ui32Period;
uint32_t time_delay=0;

char attempt[5];


void timer_configuration(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

}

void time_calculation(void)
{
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ui32Period = (SysCtlClockGet()/1000)/3;
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period - 1);
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
    TimerEnable(TIMER2_BASE, TIMER_A);

}

void Timer2IntHandler(void)
{
 // Clear the timer interrupt
 TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
 // Read the current state of the GPIO pin and
 // write back the opposite state
 time_delay++;
// if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1))
//  {
//   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, 0);
//  }
//  else
//  {
//   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2);
//  }
}

void ledPinConfig(void)
{
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3);  // Pin-1 of PORT F set as output. Modifiy this to use other 2 LEDs.
}

void led_write(char value)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3,value);
}

int main(void)
{
    uint32_t i = 0,sw1,sw2;
    lcd_init();     // initialize LCD. Set required pin for interfacing LED. Configure LCD in 8-bit mode. Refer to lcd.c file for details
    switch_setup();
    switchPinConfig();
    ledPinConfig();
    timer_configuration();
    time_calculation();
    uint32_t average_time,j=0;
    uint32_t delay_measure;
    //lcd_cursor(1,2);
 while(1)
 {
     for(i=1;i<=5;i++)
     {   lcd_command(0x01);
         lcd_cursor(1,1);
         lcd_string("Press switch SW1");
         lcd_cursor(2,3);
         lcd_string("LED glows");
         led_write(0x02);
    do
    {
        sw1 = read_switch(read_sw1);

    }
    while (sw1 !=0x00);
        led_write(0x00);
        delay_measure = time_delay;
        time_delay =0;
        lcd_command(0x01);
        led_write(0x00);
        lcd_cursor(1,2);
        lcd_string("Attempt no: ");
        lcd_print(1,14,i,1);
        attempt[i]= delay_measure;
        lcd_print(2,1,delay_measure,6);
        lcd_string(" ms");
        SysCtlDelay(6700000);
   }

        lcd_command(0x01);
        lcd_cursor(1,1);
        lcd_string("Press switch SW2");
        lcd_cursor(2,1);
        lcd_string("Average T: ");
        led_write(0x02);
     do
     {
         sw2 = read_switch(read_sw2);
     }
     while (sw2 !=0x00);


     for(j=1;j<=5;j++)
     {
         average_time = average_time + attempt[j];
     }
     average_time = average_time/5;
     lcd_print(2,11,average_time,4);
     SysCtlDelay(6700000);

 }

 }
