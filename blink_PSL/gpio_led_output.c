#include "fsl_gpio.h"
#include "fsl_power.h"
#include "fsl_clock.h"
#include "fsl_swm.h"
#include "fsl_swm_connections.h"

#include <stdint.h>
#include <stdio.h>

#define GRN_LED_PORT 0
#define GRN_LED_PIN  16

#define CORE_CLOCK   30000000U  // Set CPU Core clock frequency (Hz)

void delay(uint32_t counts); // The busy delay routine.
void clock_init(void);


int main(void) {
  gpio_pin_config_t led_config = {
				  kGPIO_DigitalOutput,
				  0,
  };
  gpio_pin_config_t button_config = {
      kGPIO_DigitalInput,
      0,
  };
  clock_init();
  GPIO_PortInit(GPIO, GRN_LED_PORT);
  GPIO_PinInit(GPIO, GRN_LED_PORT, GRN_LED_PIN, &led_config);
  GPIO_PinInit(GPIO, 0, 17, &led_config);  // RED
  GPIO_PinInit(GPIO, 0, 18, &led_config);  // YELLOW  
  GPIO_PinInit(GPIO, 0, 19, &led_config);  // GREEN
  GPIO_PinInit(GPIO, 0, 20, &button_config);  // Button 2
  GPIO_PinInit(GPIO, 0, 21, &button_config);  // Button 1
  for(int i = 0; i < 3; i++) {
    GPIO_PinWrite(GPIO, 0, 17, 0);  // All LEDs ON
    GPIO_PinWrite(GPIO, 0, 18, 0);
    GPIO_PinWrite(GPIO, 0, 19, 0);
    delay(300000);
    GPIO_PinWrite(GPIO, 0, 17, 1);  // All LEDs OFF  
    GPIO_PinWrite(GPIO, 0, 18, 1);
    GPIO_PinWrite(GPIO, 0, 19, 1);
    delay(300000);
}
  volatile int state = 0;
    volatile int button1Pressed = 0;
    volatile int button2Pressed = 0;
    unsigned char RED =  17;
    unsigned char YELLOW =  18;
    unsigned char GREEN =  19;
    unsigned char Button1 =  21;
    unsigned char Button2 =  20;
    while (1)
    {
        if (GPIO_PinRead(GPIO, 0, Button1)  == 0) //when button 1 is pressed
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

        if (GPIO_PinRead(GPIO, 0, Button2)==0) //when button 2 is pressed
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
       switch(state)
        {
            case 0:
              /* Red ON, Yellow OFF, Green OFF */
              GPIO_PinWrite(GPIO, 0, RED, 0);
              GPIO_PinWrite(GPIO, 0, YELLOW, 1);
              GPIO_PinWrite(GPIO, 0, GREEN, 1);
              break;
            case 1:
              /* Red OFF, Yellow ON, Green OFF */
              GPIO_PinWrite(GPIO, 0, RED, 1);
              GPIO_PinWrite(GPIO, 0, YELLOW, 0);
              GPIO_PinWrite(GPIO, 0, GREEN, 1);
              break;
            case 2:
              /* Red OFF, Yellow OFF, Green ON */
              GPIO_PinWrite(GPIO, 0, RED, 1);
              GPIO_PinWrite(GPIO, 0, YELLOW, 1);
              GPIO_PinWrite(GPIO, 0, GREEN, 0);
              break;
            case 3:
              /* All OFF */
              GPIO_PinWrite(GPIO, 0, RED, 1);
              GPIO_PinWrite(GPIO, 0, YELLOW, 1);
              GPIO_PinWrite(GPIO, 0, GREEN, 1);
              break;
            case 4:
              /* All ON */
              GPIO_PinWrite(GPIO, 0, RED, 0);
              GPIO_PinWrite(GPIO, 0, YELLOW, 0);
              GPIO_PinWrite(GPIO, 0, GREEN, 0);
              break;
            default:
              break;
        }
        delay(1000000);
    }
}


// Delay timing is provided by a busy wait:
void delay(uint32_t counts) {

  uint32_t wait;
  for (wait=counts; wait>0; --wait){
  }

}

void clock_init(void) {    // Set up the clock source

  // Set up IRC
  POWER_DisablePD(kPDRUNCFG_PD_IRC_OUT);        // Turn ON IRC OUT
  POWER_DisablePD(kPDRUNCFG_PD_IRC);            // Turn ON IRC
  //POWER_DisablePD(kPDRUNCFG_PD_SYSOSC);       // In Alakart SYSOSC is not used.
  CLOCK_Select(kSYSPLL_From_Irc);               // Connect IRC to PLL input.
  clock_sys_pll_t config;
  config.src = kCLOCK_SysPllSrcIrc;             // Select PLL source as IRC. 
  config.targetFreq = CORE_CLOCK*2;             // set pll target freq
  CLOCK_InitSystemPll(&config);                 // set parameters
  //CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcIrc);  // Select IRC as main clock source.
  CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcSysPll); // Select PLL as main clock source.
  CLOCK_Select(kCLKOUT_From_Irc);               // select IRC for CLKOUT
  CLOCK_SetCoreSysClkDiv(1U);
  //CLOCK_UpdateClkOUTsrc();
  // Set SystemCoreClock variable.
  //    SystemCoreClock = CORE_CLOCK;

  // Check processor registers and calculate the
  // Actual clock speed. This is stored in the
  // global variable SystemCoreClock
  SystemCoreClockUpdate ();

  // The following is for convenience and not necessary. AO.
  // It outputs the system clock on Pin 27
  //    so that we can check using an oscilloscope:
  // First activate the clock out function:
  SYSCON->CLKOUTSEL = (uint32_t)3; //set CLKOUT source to main clock.
  SYSCON->CLKOUTUEN = 0UL;
  SYSCON->CLKOUTUEN = 1UL;
  // Divide by a reasonable constant so that it is easy to view on an oscilloscope:
  //SYSCON->CLKOUTDIV = 100;
  SYSCON->CLKOUTDIV = 2000; 

  // Using the switch matrix, connect clock out to Pin 27:
  CLOCK_EnableClock(kCLOCK_Swm);     // Enables clock for switch matrix.
  SWM_SetMovablePinSelect(SWM0, kSWM_CLKOUT, kSWM_PortPin_P0_27);
  CLOCK_DisableClock(kCLOCK_Swm); // Disable clock for switch matrix.

}




//===================================================================
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
// #include "lpc824.h"

// void delay(int counts)
// {
//     int wait;
//     for (wait=counts; wait>0; --wait){}
// }

// int main(void)
// {
//     volatile int state = 0;
//     volatile int button1Pressed = 0;
//     volatile int button2Pressed = 0;
//     unsigned char RED =  17;
//     unsigned char YELLOW =  18;
//     unsigned char GREEN =  19;
//     unsigned char Button1 =  21;
//     unsigned char Button2 =  20;
//     while (1)
//     {
//         if (*Button1 == 1) //when button 1 is pressed
//         {
//             if(!button1Pressed)
//             {
//                 button1Pressed = 1;
//                 state++;
//             }

//             if (state > 4)
//             {
//                 state = 0;
//             }
//         }
//         else{
//             button1Pressed = 0;
//         }

//         if (*Button2 == 1) //when button 2 is pressed
//         {
//             if(!button2Pressed)
//             {
//                 button2Pressed = 1;
//                 state += 2;
//             }

//             if (state == 5)
//             {
//                 state = 0;
//             }
//             else if (state == 6)
//             {
//                 state = 1;
//             }
//         }
//         else
//         {
//             button2Pressed = 0;
//         }

//         //    GPIO_B17 -> 0xa0000011 -> *((volatile unsigned int *)(0xa0000011))
//         //    GPIO_B18 -> 0xa0000012 -> *((volatile unsigned int *)(0xa0000012))
//         //    GPIO_B19 -> 0xa0000013 -> *((volatile unsigned int *)(0xa0000013))

//        switch(state)
//         {
//             case 0:
//               /* Red ON, Yellow OFF, Green OFF */
//               GPIO_PinWrite(GPIO, 0, RED, 0);
//               GPIO_PinWrite(GPIO, 0, YELLOW, 1);
//               GPIO_PinWrite(GPIO, 0, GREEN, 1);
//               break;
//             case 1:
//               /* Red OFF, Yellow ON, Green OFF */
//               GPIO_PinWrite(GPIO, 0, RED, 1);
//               GPIO_PinWrite(GPIO, 0, YELLOW, 0);
//               GPIO_PinWrite(GPIO, 0, GREEN, 1);
//               break;
//             case 2:
//               /* Red OFF, Yellow OFF, Green ON */
//               GPIO_PinWrite(GPIO, 0, RED, 1);
//               GPIO_PinWrite(GPIO, 0, YELLOW, 1);
//               GPIO_PinWrite(GPIO, 0, GREEN, 0);
//               break;
//             case 3:
//               /* All OFF */
//               GPIO_PinWrite(GPIO, 0, RED, 1);
//               GPIO_PinWrite(GPIO, 0, YELLOW, 1);
//               GPIO_PinWrite(GPIO, 0, GREEN, 1);
//               break;
//             case 4:
//               /* All ON */
//               GPIO_PinWrite(GPIO, 0, RED, 0);
//               GPIO_PinWrite(GPIO, 0, YELLOW, 0);
//               GPIO_PinWrite(GPIO, 0, GREEN, 0);
//               break;
//             default:
//               break;
//         }
//         delay(1000000);
//     }
// }


