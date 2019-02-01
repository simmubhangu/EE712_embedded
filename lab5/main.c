
#include "commonheader.h"

#define NUM_SSI_DATA 8
const uint8_t pui8DataTx[NUM_SSI_DATA] =
{0x88, 0xF8, 0xF8, 0x88, 0x01, 0x1F, 0x1F, 0x01};

int main(void)
{
    uint32_t ui32Index;
    uint32_t ui32Data;
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
    while(1)
    {
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        //ui32Data = (Reverse(pui8DataTx[ui32Index]) << 8) + (1 << ui32Index);
        SSIDataPut(SSI0_BASE, ui32Data);
        while(SSIBusy(SSI0_BASE));
    }
    }
}
