// Heavily modified from NXP Xpresso_SDK, driver_examples 16bit_counter
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "fsl_sctimer.h"
#include "fsl_swm.h"
#include "fsl_swm_connections.h"
#include "fsl_power.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "fsl_pint.h"
#include "fsl_clock.h"
#include "fsl_syscon.h"
#include "fsl_mrt.h"
#define BUTTON_PORT 0U
#define BUTTON_PIN_ONE 10U
#define BUTTON_PIN_TWO 11U
#define BUTTON_PIN_THREE 18U

status_t uart_init(void);
void clock_init(void);
void SysTick_Handler(void);
void SysTick_DelayTicks(uint32_t n);
volatile uint32_t g_systickCounter;
void SCTimerL_init(sctimer_config_t* sctimerConfig);
#define CORE_CLOCK   6000000U
#define LED_PIN_ONE 24
#define LED_PIN_TWO 25
#define LED_PIN_THREE 19
#define DESIRED_INT_FREQ 2
#define USART_INSTANCE   0U
#define USART_BAUDRATE 115200
uint32_t buttonPressEventId = 0;
uint32_t eventCounterL; 
volatile uint16_t matchValueL = 60000;
bool InAction = false;
volatile int action = -1;
volatile uint8_t state = 0;
uint32_t mrt_clock;
uint32_t mrt_count_val;
// state FSM:
// 0: idle
// 1: button 1 pressed
// 11: button 1 pressed again
// 113: button 3 pressed
// 1132: button 2 pressed
// 11321: button 1 pressed
// the End
// * from state 11 we stay in it after button 1 press
void MRT0_IRQHandler(void);
void SCT0_IRQHandler(void);
void pint_intr_callback_one(pint_pin_int_t pintr, uint32_t pmatch_status);
void pint_intr_callback_two(pint_pin_int_t pintr, uint32_t pmatch_status);  
void pint_intr_callback_three(pint_pin_int_t pintr, uint32_t pmatch_status);
void SetInterrupt();

void SCT0_IRQHandler(void)
{
    uint32_t status = SCTIMER_GetStatusFlags(SCT0);
    SCTIMER_ClearStatusFlags(SCT0, status);
    if((!GPIO_PinRead(GPIO, BUTTON_PORT, BUTTON_PIN_ONE) && action == 1)
    || (!GPIO_PinRead(GPIO, BUTTON_PORT, BUTTON_PIN_TWO) && action == 2)
    || (!GPIO_PinRead(GPIO, BUTTON_PORT, BUTTON_PIN_THREE) && action == 3))
    {
        //button still pressed
        MRT_StartTimer(MRT0, kMRT_Channel_0, mrt_count_val);
    }
    else
        action = -1;
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_L);
}

void MRT0_IRQHandler(void) {
// Clear interrupt flag:
    MRT_ClearStatusFlags(MRT0,
		       kMRT_Channel_0,
		       kMRT_TimerInterruptFlag);

    //FSM handling
    if(action == 1){
        if(state == 0){
            state = 1;
            GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 1);
            GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 0);
        }
        else if(state == 1 || state == 2){
            state = 2;
        }
        else if(state == 4){
            state = 0;
            GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 1);
            GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 1);
            GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 1);
        }
        else{
            state = 0;
            GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 0);
        }
    }
    else if(action == 2){
        if(state == 3){
            state = 4;
            GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 1);
            GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 0);
        }
        else{
            state = 0;
            GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 0);
        }
    }
    else if(action == 3){
        if(state == 2){
            state = 3;
            GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 1);
        }
        else{
            state = 0;
            GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 0);
            GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 0);
        }
    }
    SysTick_DelayTicks(500U);
    GPIO_PinWrite(GPIO, 0, LED_PIN_ONE, 0);
    GPIO_PinWrite(GPIO, 0, LED_PIN_TWO, 0);
    GPIO_PinWrite(GPIO, 0, LED_PIN_THREE, 0);
    action = -1;
}
// Setup interrupt after button press to check after 1 sec if button still pressed
void pint_intr_callback_one(pint_pin_int_t pintr, uint32_t pmatch_status) {
    //in 1 sec sct0 interrupt will happen 
    action = 1;
    SetInterrupt();
    return;
    }

void pint_intr_callback_two(pint_pin_int_t pintr, uint32_t pmatch_status) {
    //in 1 sec sct0 interrupt will happen 
    action = 2;
    SetInterrupt();
    return;
    }

void pint_intr_callback_three(pint_pin_int_t pintr, uint32_t pmatch_status) {
    //in 1 sec sct0 interrupt will happen 
    action = 3;
    SetInterrupt();
    return;
    }

void SetInterrupt(){
    //setting up interrupt to check after 1 sec if button still pressed
    SCTIMER_CreateAndScheduleEvent(SCT0,
            kSCTIMER_MatchEventOnly,
            matchValueL,
            0,
            kSCTIMER_Counter_L,
            &eventCounterL);
    SCTIMER_SetupEventActiveDirection(SCT0, kSCTIMER_ActiveIndependent, eventCounterL);
    SCTIMER_EnableInterrupts(SCT0, (1U << eventCounterL));
    EnableIRQ(SCT0_IRQn);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
    return;
}

int main(void)
{
    CLOCK_EnableClock(kCLOCK_Uart0);               // Enable clock of uart0
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U);   // Ser DIV of uart0.

    InitPins();
    clock_init();
    uart_init();

    mrt_config_t mrtConfig;
    gpio_pin_config_t led_pin_conf ={kGPIO_DigitalOutput, 0};
    CLOCK_EnableClock(kCLOCK_Gpio0);



    CLOCK_EnableClock(kCLOCK_Mrt);
    MRT_GetDefaultConfig(&mrtConfig);

    MRT_Init(MRT0, &mrtConfig);
    MRT_SetupChannelMode(MRT0, kMRT_Channel_0, kMRT_OneShotMode);

    mrt_clock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    mrt_count_val= mrt_clock/DESIRED_INT_FREQ;
    MRT_EnableInterrupts(MRT0, kMRT_Channel_0, kMRT_TimerInterruptEnable);
    EnableIRQ(MRT0_IRQn);
    // Connect PIO_10 11 16 as a source to PIN INT 1 2 3:
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt1, kSYSCON_GpioPort0Pin10ToPintsel);//button1
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt2, kSYSCON_GpioPort0Pin11ToPintsel);//button2
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt3, kSYSCON_GpioPort0Pin18ToPintsel);//button3

    PINT_Init(PINT);

    // Setup Pin Interrupt 1:
    //butt1
    PINT_PinInterruptConfig(PINT,
            kPINT_PinInt1,
            kPINT_PinIntEnableFallEdge,
            pint_intr_callback_one); // Name of the callback function
    //butt2      
    PINT_PinInterruptConfig(PINT,
            kPINT_PinInt2,
            kPINT_PinIntEnableFallEdge,
            pint_intr_callback_two);
    //butt3
    PINT_PinInterruptConfig(PINT,
            kPINT_PinInt3,
            kPINT_PinIntEnableFallEdge,
            pint_intr_callback_three);

    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt1);
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt2);
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt3);

    EnableIRQ(PIN_INT1_IRQn);
    EnableIRQ(PIN_INT2_IRQn);
    EnableIRQ(PIN_INT3_IRQn);

    sctimer_config_t sctimerConfig;
    SCTimerL_init(&sctimerConfig);
    SCTIMER_Init(SCT0, &sctimerConfig);
    GPIO_PinInit(GPIO, 0, LED_PIN_ONE, &led_pin_conf);
    GPIO_PinInit(GPIO, 0, LED_PIN_TWO, &led_pin_conf);
    GPIO_PinInit(GPIO, 0, LED_PIN_THREE, &led_pin_conf);
    SysTick_Config(SystemCoreClock / 1000U);    
    while (1) {
        __WFI();
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
    // It outputs the system clock on Pin 26
    //    so that we can check using an oscilloscope:
    // First activate the clock out function:
    SYSCON->CLKOUTSEL = (uint32_t)3; //set CLKOUT source to main clock.
    SYSCON->CLKOUTUEN = 0UL;
    SYSCON->CLKOUTUEN = 1UL;
    // Divide by a reasonable constant so that it is easy to view on an oscilloscope:
    //SYSCON->CLKOUTDIV = 100;
    SYSCON->CLKOUTDIV = 2000; 

    // Using the switch matrix, connect clock out to Pin 26:
    CLOCK_EnableClock(kCLOCK_Swm);     // Enables clock for switch matrix.
    SWM_SetMovablePinSelect(SWM0, kSWM_CLKOUT, kSWM_PortPin_P0_26);
    CLOCK_DisableClock(kCLOCK_Swm); // Disable clock for switch matrix.
    }

void SCTimerL_init(sctimer_config_t* sctimerConfig) {

    SCTIMER_GetDefaultConfig(sctimerConfig);

    sctimerConfig->enableCounterUnify = false;            /* use two 16-bit timers */
    sctimerConfig->clockMode = kSCTIMER_System_ClockMode; /* use system clock */
    sctimerConfig->enableBidirection_l = false;
    sctimerConfig->enableBidirection_h = false;
    sctimerConfig->prescale_l = 249; /* prescaler value (used as value+1) */
    sctimerConfig->prescale_h = 249;
    }

    status_t uart_init(void) {

    uint32_t uart_clock_freq;
    status_t result;

    uart_clock_freq=CLOCK_GetMainClkFreq(); // Read UART clock frequency. 

    CLOCK_EnableClock(kCLOCK_Uart0);               // Enable clock of UART0.
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U);   // Set prescaler of UART0.
    RESET_PeripheralReset(kUART0_RST_N_SHIFT_RSTn);// Reset UART0

    // See:
    //Xpresso_SDK/devices/LPC824/utilities/debug_console_lite/fsl_debug_console.c
    result = DbgConsole_Init(USART_INSTANCE,
            USART_BAUDRATE,
            kSerialPort_Uart,
            uart_clock_freq);
    // assert(kStatus_Success == result);
    return result;
    }


void SysTick_Handler(void) {
    if (g_systickCounter != 0U) {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n){
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {}
}
