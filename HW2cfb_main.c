//#############################################################################
// FILE:   HWstarter_main.c
//
// TITLE:  HW Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCA_ISR(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
//global variables for photoresistor
uint16_t ADCINA4_raw = 0; // takes input from ADCA4
float ADCINA4_result = 0; // discretized converted voltage
uint32_t ADCA_interruptcount = 0; // clock counter for the ADCA interrupt function
uint16_t xaxis = 0; // takes input from joystick
uint16_t yaxis = 0; // takes input from joystick
float x_voltage = 0; // 
float y_voltage = 0; // 

// Function Variables
int16_t updown =1; // For cpu timer 2 LED dimming function, when updown = 1, counts up, when updown = 0 count down - cfb

// global function

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5); // Sets GPIO22 to EPwm12A
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCA1_INT = &ADCA_ISR; // allows us to create an interrupt
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);


    //command ADCA peripheral to sample ADCINA4 ########################################################################################################################
    EALLOW;
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 01; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1  50Mhz Clock
    EPwm4Regs.TBPRD = 50000;  // Set Period to 1ms sample.  Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 00; //unfreeze, and enter up count mode
    EDIS;

    // Setup ADCA to use SOC0 ########################################################################################################################
    // ADCA1 interrupt is setup to be called when one channel ADCINA4 is finished converting
    // ADCA input channels :ADCINA0, ADCINA1, etc.
    // ADCA interrupts: ADCA1, ADCA2, etc.

    EALLOW;
    //write configurations for  ADCA
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x4;//SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0B;// EPWM4 ADCSOCA
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x2;//SOC1 will conv Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0B;// EPWM4 ADCSOCA
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 0x3;//SOC2 will conv Channel you choose Does not have to be A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0B;// EPWM4 ADCSOCA
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL=0x2;//set to last or only SOC that is converted and it will set INT1 flag ADCA1. Only looking at SOC0, the only soc setup is 0.
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    // Setting EPWM12A ########################################################################################################################
     EPwm12Regs.TBCTL.bit.CTRMODE=0; //Sets Counte up mode
     EPwm12Regs.TBCTL.bit.FREE_SOFT=2; //Sets the counter to continue to run (Free Run) even when there is a break point - cfb
     EPwm12Regs.TBCTL.bit.PHSEN=0; // Time-base counter is not loaded from the phase register - cfb
     EPwm12Regs.TBCTL.bit.CLKDIV=0; // Sets clockdivide to 1 - cfb
     EPwm12Regs.TBCTR=0; // Initializes Time-base counter to zero which is default setting - cfb
     EPwm12Regs.TBPRD=5000; // Sets period (carrier frequency of the PWM signal to 5KHz which is a period of 200ms.
     EPwm12Regs.CMPA.bit.CMPA=2500; // Starts duty cycle at 50%
     EPwm12Regs.AQCTLA.bit.CAU=1; // When count up reaches CMPA (TBCTR = 2500), forces output of pin to low - cfb
     EPwm12Regs.AQCTLA.bit.ZRO=2; // When count reaches 0, forces output of pin to high so that the duty cycle starts on at the beginning of the period -cfb
     EPwm12Regs.TBPHS.bit.TBPHS=0; // Sets phase to zero, assumed to be default setting - cfb

     // Disable pull-up resistor when an I/O pin is set as a PWm output pin for power consump[tion reasons.
     EALLOW;  // Below are protected registers
     GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
     EDIS;
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;


    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enagle PIE interrupt 1.1, I'm a little confused of the notation of this.FPI
    PieCtrlRegs.PIEIER1.bit.INTx1=1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
                //serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            // When UARTPRINT ==1, print ADCINA4_result
            // serial_printf(&SerialA,"ADCA1 V: %f \r\n", ADCINA4_result);
            serial_printf(&SerialA,"Vx: %f, Vy: %f \r\n", x_voltage, y_voltage);
            UARTPrint = 0;
        }
    }
}

//adca1 pie interrupt
__interrupt void ADCA_ISR (void) {
    // Here covert ADCINA4 to volts
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    // read ADCINA4 into ADCINA4_raw, read ADAC1 and ADC2 into xaxis yaxis
    ADCINA4_raw = AdcaResultRegs.ADCRESULT0;
    xaxis = AdcaResultRegs.ADCRESULT2;
    yaxis = AdcaResultRegs.ADCRESULT1;


    // Convert ADCINA4 to voltage and store into ADCINA4_ result;
    ADCINA4_result = ADCINA4_raw * (3.0/4096.0);
    // Converts yaxis and xaxis to voltage
    x_voltage = xaxis * (3/4096.0);
    y_voltage = yaxis * (3/4096.0);
    // Print ADCINA4’s voltage value to TeraTerm every 100ms by setting UARTPrint to one every 100th  time in this function.
    if ((ADCA_interruptcount % 100) == 0) {
            UARTPrint = 1;
        }

    // Turn voltage into descretized value for CMPA
    EPwm12Regs.CMPA.bit.CMPA = ADCINA4_result * (5000/3);
    // Turns LED 8 on if there is no input to the joystick
    if (x_voltage <= 1.7 && x_voltage >= 1.5 && y_voltage >= 1.4 && y_voltage <= 1.6) {
        GpioDataRegs.GPASET.bit.GPIO25 = 1;  // 8 on
        GpioDataRegs.GPECLEAR.bit.GPIO130 = 1; // 6 off
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // 10 off
        GpioDataRegs.GPECLEAR.bit.GPIO157 = 1; // 13 off
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1; // 3 off
    }
    // turns LED 6 on if there is an input to the positive x direction
    if (x_voltage >  1.7) {
        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;  // 8 off
        GpioDataRegs.GPESET.bit.GPIO130 = 1; // 6 on
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // 10 off
        GpioDataRegs.GPECLEAR.bit.GPIO157 = 1; // 13 off
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1; // 3 off
    }
    // turns LED 10 on if there is an input to the negative direction
    if (x_voltage < 1.5) {
        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;  // 8 off
        GpioDataRegs.GPECLEAR.bit.GPIO130 = 1; // 6 off
        GpioDataRegs.GPASET.bit.GPIO27 = 1; // 10 on
        GpioDataRegs.GPECLEAR.bit.GPIO157 = 1; // 13 off
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1; // 3 off
    }
    // turns LED 3 on if there is an input to the positive y direction
    if (y_voltage > 1.6) {
        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;  // 8 off
        GpioDataRegs.GPECLEAR.bit.GPIO130 = 1; // 6 off
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // 10 off
        GpioDataRegs.GPECLEAR.bit.GPIO157 = 1; // 13 off
        GpioDataRegs.GPCSET.bit.GPIO95 = 1; // 3 on
    }
    // turns LED 13 on if there is an input to the negative y direction
    if (y_voltage < 1.4) {
        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;  // 8 off
        GpioDataRegs.GPECLEAR.bit.GPIO130 = 1; // 6 off
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // 10 off
        GpioDataRegs.GPESET.bit.GPIO157 = 1; // 13 on
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1; // 3 off
    }

    ADCA_interruptcount++; // increments clock
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // ready for more interrupts

}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
       // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
        // commenting out Letter display to see the LED1's dimming function
       // displayLEDletter(LEDdisplaynum);
                  //LEDdisplaynum++;
                 //if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
                 //     LEDdisplaynum = 0;
                 // }
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    if ((numTimer0calls%250) == 0) {
        // Blink LaunchPad Blue LED
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
        }

    /*if (updown == 1) {
        EPwm12Regs.CMPA.bit.CMPA++;

        if(EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD) {

            updown = 0;

        }
    }
        if (updown == 0 ) {
                    EPwm12Regs.CMPA.bit.CMPA--;
                    if(EPwm12Regs.CMPA.bit.CMPA == 0) {
                        updown =1;
                    }
                }


    */
    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}
