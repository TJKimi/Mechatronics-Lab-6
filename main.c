#include "msp.h"
#include "driverlib.h"
#include "stdlib.h"
#include "stdio.h"
#include "uart.h"
#include "stdint.h"
#include "stdbool.h"

#define MAIN_1_SECTOR_31 0x0003F000

//volatile char receiveBuffer[500]="";
volatile float data[30] = {};
char outputString[10];
volatile int timer = 0;
volatile int i = 0;
void MAP_FPU_enbleModule();
float outArray[30] = {};

const Timer_A_UpModeConfig upConfig_0 = {
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_32,
    46875,
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
    TIMER_A_DO_CLEAR
};

const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
    19,                                     // BRDIV = 19
    8,                                       // UCxBRF = 8
    85,                                       // UCxBRS = 85
    EUSCI_A_UART_NO_PARITY, // No Parity
    EUSCI_A_UART_LSB_FIRST, // LSB First
    EUSCI_A_UART_ONE_STOP_BIT, // One stop bit
    EUSCI_A_UART_MODE, // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
    /*clock prescalar: 19, firstModReg: 8, secondModReg: 85, oversampling: 1*/
};
void main(void)
{

    MAP_WDT_A_holdTimer();//stop watchdog timer

    /*Configure frequency and clock*/
    //MAP_CS_setDCOFrequency(3E+6); //set frequency
    //MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);

    /*Set input pin 1.1 1.2 1.3*/


    /*initialize UART A0*/
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);//configure UART
    MAP_UART_enableModule(EUSCI_A0_BASE); //enable UART Module

    /*enable UART interrupts
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableMaster();*/

    /*set S1 and S2 (p1.1 and p1.4)*/
    MAP_GPIO_setAsOutputPin                 (GPIO_PORT_P1, GPIO_PIN0); //p1.1
    MAP_GPIO_setAsOutputPin                 (GPIO_PORT_P1, GPIO_PIN4); //p1.4
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1); //S1 pull up
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN4); //S2 pull down
    MAP_GPIO_setAsOutputPin     (GPIO_PORT_P2,GPIO_PIN1); //p2.1
    MAP_GPIO_setOutputLowOnPin  (GPIO_PORT_P2, GPIO_PIN1);//turn off p2.1
    unsigned short usiButton1 = 0;
    unsigned short usiButton2 = 0;

    while(1)
        {
            usiButton1 = MAP_GPIO_getInputPinValue ( GPIO_PORT_P1, GPIO_PIN1 );// Read button p1.1
            usiButton2 = MAP_GPIO_getInputPinValue ( GPIO_PORT_P1, GPIO_PIN4 );// Read button p1.4

            if ( usiButton1 == GPIO_INPUT_PIN_LOW) //data acquisition mode S1
                {
                    MAP_CS_setDCOFrequency(1.5E+6);
                    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
                    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig_0);
                    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);

                    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);//P1.1,P1.2,P1.3

                    /*enable ADC converter*/
                    MAP_ADC14_enableModule();
                    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION); // P5.5 for 5V connection
                    MAP_ADC14_setResolution(ADC_10BIT); // 10 bit resolution
                    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_MAPINTCH0); // initialize module
                    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, false);
                    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);
                    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);//manual iteration

                    /*enable timer interrupts*/
                    MAP_Interrupt_enableInterrupt(INT_TA0_0);
                    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
                    MAP_Interrupt_enableMaster();

                    /*ADC start sampling and ADC conversion*/
                    MAP_ADC14_enableConversion(); //Enable conversion
                    MAP_ADC14_toggleConversionTrigger(); //Initiates a single conversion (trigger)

                    while (timer < 30){};
                    MAP_Interrupt_disableMaster();//stop sampling immediately
                    while (ADC14_isBusy()){};

                    /*write data into flash memory*/
                    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31);//enable writing
                    if(!FlashCtl_eraseSector(MAIN_1_SECTOR_31))//erase whatever is in sector 31
                        {
                         printf("\nErase failed\r\n");
                        }

                    //FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE);
                    //char myString[] = {'F','l','a','s','h',' ','E','x','a','m','p','l','e','\0'} ;
                    //FlashCtl_programMemory(data, (void*)MAIN_1_SECTOR_31, 30);

                    if(!MAP_FlashCtl_programMemory(data, (void*)MAIN_1_SECTOR_31, 200))
                        {
                         printf("\nWrite failed\r\n");
                        }

                    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31);

                    int j = 0;

                    //float convertedData[30];
                    for (j = 0; j < 30; j++)
                    {
                        float *convertedData = (float*)MAIN_1_SECTOR_31+j;
                        outArray[j] = *convertedData;
                        printf("\n %f", outArray[j]);

                    }

                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);//turn on LED P2.1
                }

            if ( usiButton2 == GPIO_INPUT_PIN_LOW) //data output mode S2
                {
                    unsigned int dcoFrequency = 3E+6; // configure clock
                    MAP_CS_setDCOFrequency(dcoFrequency);
                    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
                    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
                    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);//P1.1,P1.2,P1.3

                    /*initialize UART A0*/
                    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);//configure UART
                    MAP_UART_enableModule(EUSCI_A0_BASE); //enable UART Module


                    int e = 0;
                    int d = 0;

                    for (e = 0; e<30; e++)
                    {
                        sprintf(outputString, "%2.1f \r\n", outArray[e]);
                        for (d = 0; d < 10; d++)
                            {
                                MAP_UART_transmitData(EUSCI_A0_BASE, outputString[d]);

                            }
                    }

        }//end of infinite while loop

}

/* EUSCI A0 UART ISR - Echoes data back to PC host
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        int ASCII = MAP_UART_receiveData(EUSCI_A0_BASE);

        if (ASCII != 13){
            receiveBuffer[array] = MAP_UART_receiveData(EUSCI_A0_BASE);
            array++;
        } else {
            edge = 1;
        }

    }

} */
}
void TA0_0_IRQHandler(void)
{

    //MAP_CS_setDCOFrequency(1.5E+6);
    float volt = MAP_ADC14_getResult(ADC_MEM0);
    volt = volt*1.25/510; //convert to voltage
    float temp = volt * 150/2.5;  // temp interpolated at 150 F at 2.5 V
    data[timer] = temp;//store temperatures into floating array
    MAP_ADC14_toggleConversionTrigger();
    //char *string_from_flash = (char*)data[timer];
    printf("\nTemp: %f, %i" , data[timer], timer);

    timer = timer + 1;
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Timer_A_clearInterruptFlag(TIMER_A0_BASE);

};
