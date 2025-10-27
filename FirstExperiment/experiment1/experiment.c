/*
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                  Experiment 1
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *      # Connections
 *      ---------------------------------------------
 *      * Button 1 -> PIO0_21
 *      * Button 2 -> PIO0_20
 *      * Red Led -> PIO0_17
 *      * Yellow Led -> PIO0_18
 *      * Green Led -> PIO0_19
 *
 *      # Scenario
 *      ---------------------------------------------
 *      * Start red LED will be ON (initial state).
 *      *  When B1 is pressed, the LEDs will cycle as:
 *          - Red ON, Yellow OFF, Green OFF
 *          - Red OFF, Yellow ON, Green OFF
 *          - Red OFF, Yellow OFF, Green ON
 *          - Red OFF, Yellow OFF, Green OFF
 *          - Red ON, Yellow ON, Green ON
 *      * When B2 is pressed, the LEDs will cycle in the
 *      same sequence but will skip two places:
 *          - Red ON, Yellow OFF, Green OFF
 *          - Red OFF, Yellow OFF, Green ON
 *          - Red ON, Yellow ON, Green ON
 *          - Red OFF, Yellow ON, Green OFF
 *          - Red OFF, Yellow OFF, Green OFF
 *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include "lpc824.h"

void delay(int counts)
{
    int wait;
    for (wait=counts; wait>0; --wait){}
}

int main(void)
{
    // System Clock Control Register
    // This register enables the clocks to individual system and peripheral blocks.
    // Bit 6 -> GPIO
    // Bit 7 -> SWM (Switch Matrix)
    // Bit 18 -> IOCON
    // 0100 0000 0000 1100 0000 -> 0x400C0
//    SYSCON_BASE Adress -> 0x40048000
//    SYSCON_SYSAHBCLKCTRL -> SYSCON_BASE + 0x80
    volatile int state = 0;
    volatile int button1Pressed = 0;
    volatile int button2Pressed = 0;
    *((volatile unsigned int *)(0x40048080)) |= 0x00000040; // only GPIO


    // Peripheral Reset Control Register
    // Bit 10 -> GPIO_RST_N
    // 0100 0000 0000
//    SYSCON_BASE -> 0x40048000
//    SYSCON_PRESETCTRL -> 0x04

    *((volatile unsigned int *)(0x40048004)) &= ~(0x400); // Peripheral reset control to gpio/gpio int
    *((volatile unsigned int *)(0x40048004)) |= 0x400; // AO: Check.


    // GPIO Direction Port Register
    // 0 input - 1 output
//    GPIO BASE -> 0xa0000000
//    GPIO_DIR0 -> GPIO_BASE + 0x2000

    *((volatile unsigned int *)(0xA0002000)) &= ~(0x00300000); // Pin 20 (B2) and 21 (B1) input.
    *((volatile unsigned int *)(0xA0002000)) |= (0x000E0000); // Pin 17, 18, 19 output(LEDS).


//    GPIO BASE -> 0xa0000000
//    GPIO_Bn -> GPIO_BASE + n

    // *((volatile unsigned int *)(0xA0002000)) &= ~(0x00300000); // Pin 20 (B2) and 21 (B1) input.
    // *((volatile unsigned int *)(0xA0002000)) |= (0x000E0000); // Pin 17, 18, 19 output(LEDS).
    // *((volatile unsigned char *)(0xA0000011)) = 1; // GPIO_B17 = 0; // red on
    // *((volatile unsigned char *)(0xA0000012)) = 0; // GPIO_B18 = 1; // yellow off
    // *((volatile unsigned char *)(0xA0000013)) = 0; // GPIO_B19 = 1; // green off

    volatile unsigned char* RED =  (volatile unsigned char *) 0xA0000011;//pin = 17
    volatile unsigned char* YELLOW = (volatile unsigned char *) 0xA0000012;//pin = 18
    volatile unsigned char* GREEN = (volatile unsigned char *) 0xA0000013;//pin = 19
    volatile unsigned char* Button1 = (volatile unsigned char *) 0xA0000015;//pin = 21
    volatile unsigned char* Button2 = (volatile unsigned char *) 0xA0000014;//pin = 20
    *RED = 1;    // red on
    *YELLOW = 0; // yellow off
    *GREEN = 0;  // green off

    while (1)
    {
        if (*Button1 == 1) //when button 1 is pressed
        {
            if(!button1Pressed)
            {
                button1Pressed = 1;
                state++;
            }

            if (state > 4)
            {
                state = 0;
            }
        }
        else{
            button1Pressed = 0;
        }

        if (*Button2 == 1) //when button 2 is pressed
        {
            if(!button2Pressed)
            {
                button2Pressed = 1;
                state += 2;
            }

            if (state == 5)
            {
                state = 0;
            }
            else if (state == 6)
            {
                state = 1;
            }
        }
        else
        {
            button2Pressed = 0;
        }

        //    GPIO_B17 -> 0xa0000011 -> *((volatile unsigned int *)(0xa0000011))
        //    GPIO_B18 -> 0xa0000012 -> *((volatile unsigned int *)(0xa0000012))
        //    GPIO_B19 -> 0xa0000013 -> *((volatile unsigned int *)(0xa0000013))

       switch(state)
        {
            case 0:
                *RED = 1;
                *YELLOW = 0;
                *GREEN = 0;
                break;
            case 1:
                *RED = 0;
                *YELLOW = 1;
                *GREEN = 0;
                break;
            case 2:
                *RED = 0;
                *YELLOW = 0;
                *GREEN = 1;
                break;
            case 3:
                *RED = 0;
                *YELLOW = 0;
                *GREEN = 0;
                break;
            case 4:
                 *RED = 1;
                 *YELLOW = 1;
                 *GREEN = 1;
                break;
            default:
                break;
        }
    }
}



