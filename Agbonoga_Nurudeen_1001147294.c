// Final Project
// Nurudeen Agbonoga

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
/*
// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
//
SSI Interface Pinouts:
CLK PB4
CS PB5
TX PB7
LDAC PA7
ADC Pinouts:
AIN0 PE3
AIN1 PE2

*/
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
//#define DAC_CS (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))    
#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define PD1 (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))


#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define AIN0_MASK 8
#define AIN1_MASK 4
#define DAC_CS_MASK 32 //connected to PB5
#define TX_MASK 128 //PB7
#define CLK_MASK 16 //PB4
#define LDAC_MASK 128 //PA7
#define PD1_MASK 2

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
#define MAXCHAR 80
#define MAXFIELDS 6
#define MAXCHARS 80
#define LUTSIZE 4096
//commands by index
#define RESET 1
#define DC 2
#define CYCLES 3
#define SINE 4
#define SQUARE 5
#define SAWTOOTH 6
#define TRIANGLE 7
#define RUN 8
#define STOP 9
#define VOLTAGE 10
#define OUTPUT 11
#define HILBERT 12
#define DIFFERENTIAL 13
#define ALC 14
#define DOWNLOAD 15
#define ARB 16

//GLobal variables
char strIn[MAXCHAR];//the user input
char strOUT[MAXCHAR];
uint16_t LUT[2][LUTSIZE];
uint16_t savedLUT[LUTSIZE];
uint32_t phi0 = 0;
uint32_t delta_phi0 = 0;
uint32_t phi1 = 0;
uint32_t delta_phi1 = 0;
float delta_phi_f0 = 0;
float delta_phi_f1 = 0;
uint8_t pos[MAXFIELDS]; //stores the postion of the words in the user input string
uint8_t argCount; //number of arguments in the user input
uint8_t load = 0;
uint16_t idle;
uint8_t LUTOut1 = 0;
uint8_t LUTOut2 = 1;
int cycles = -1;
int cyclescount = 0;
uint8_t type; //type can be any command
uint32_t tailrOn; //to be used to set how long the timer runs for the square wave function
uint32_t tailrOff;
bool on = false;
uint16_t dataDc = 0;
uint8_t alcStat = 0;
uint8_t downCom = 0;
int downChan = 0;
float downFreq = 0;
float downAmp = 0;
float downOfs = 0;
float Tcor0 = 1;
float Tcor1 = 1;

//Prototypes
void initHw();
void getString();
void step1();
void waitMicrosecond(uint32_t us);
void parseString();
void reset();
uint8_t isCommand(char strCmd[],uint8_t count); //count is the minimum number of arguments we need
float readADC0voltage(uint8_t ain);//input will be 0 or 1 depending on the ADC we want to read from
void dacWrite(uint16_t data);
void dcOut(uint8_t out, float voltage);
void makeLut(uint8_t com, uint8_t out, float freq, float amp, float ofs);
void modLut(uint8_t com, int arg1);
void resetLUT();
void alcOn();
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable ADC 0, GPIO port A, B, D, E,F and SSI2 peripherals
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGCSSI_R = SYSCTL_RCGCSSI_R2;            // turn-on SSI2 clocking
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB;// | SYSCTL_RCGC2_GPIOD;

    // Configure LED pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs
  
    
    // Configure AIN0 and AN1 as an analog inputs
	GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3
    GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN1 (PE2)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2
    
    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN2;                // disable sample sequencer 2 (SS2) for programming
    ADC0_SAC_R = ADC_SAC_AVG_64X;
    ADC0_EMUX_R = ADC_EMUX_EM2_PROCESSOR;            // select SS2 bit in ADCPSSI as trigger
    ADC0_SSMUX2_R = 16;                               // set first sample to AIN0 and second sample to AN1
    ADC0_SSCTL2_R = ADC_SSCTL2_END1;                 // mark second sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN2;                 // enable SS2 for operation
    
    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
     
     
    //configure LDAC
    GPIO_PORTA_DIR_R |= LDAC_MASK;
    GPIO_PORTA_DR2R_R |= LDAC_MASK;
    GPIO_PORTA_DEN_R |= LDAC_MASK; 
    //GPIO_PORTA_PUR_R |= LDAC_MASK; 
    
    //Configure SSI2 pins for SPI configuration
    GPIO_PORTB_DIR_R |= TX_MASK | DAC_CS_MASK | CLK_MASK;   //make SSI1 TX, FSS, and CLK outputs
    GPIO_PORTB_DR2R_R |= TX_MASK | DAC_CS_MASK | CLK_MASK;                       // set drive strength to 2mA
	GPIO_PORTB_AFSEL_R |= TX_MASK | DAC_CS_MASK | CLK_MASK;                      // select alternative functions for TX, SCLK and CS pins
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= TX_MASK | DAC_CS_MASK | CLK_MASK;   // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= CLK_MASK;  // must be disabled when SPO=0
    GPIO_PORTB_PUR_R |= DAC_CS_MASK;

    // Configure the SSI2 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
  
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 10;                                // set bit rate to 4 MHz (if SR=0 in CR0)
    SSI2_CR0_R &= ~SSI_CR0_SPO; //SPO = 0 and SPH = 0
    SSI2_CR0_R &= ~SSI_CR0_SPH;
    SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2
  
    //Configure timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;                       // set load value for 100KHz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer

}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
	UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char *str)
{
	uint8_t i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
	return UART0_DR_R & 0xFF;                        // get character from fifo
}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
	__asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
//This function will get an input string from the user
void getString()
{
    
    int count = 0;
    char c = 0;
    int stay = 1;
    for (count = 0; count < MAXCHAR; count++)//resets the global variable 'str'
    {
        strIn[count] = 0;
    }
    count = 0;
    //while (c != 10 || c!= 13)//loop will run until enter is pressed or the string is full
    while(stay)
    {
            c = getcUart0();
            if ((c == 8 || c == 127) && count == 1) //if backspace or delete was entered as the first character in the string
                strIn[count] = 0;
                //count--;
            else if(c == 10 || c == 13) //if enter was entered
            stay = 0;
            
        else
        {
            if (c >= 32) //if c is greater than space
                {
                strIn[count] = c; //store characters in the input string
                count++;
                    
                }
            else; //ignore non printable characters
        }
    }
    
    
}

//This will cause a rolling reset of the device 
void step1()
{
    GREEN_LED = 1;
    waitMicrosecond(500000);
    GREEN_LED = 0;
    waitMicrosecond(500000);
    
    
}
//This function processes the user input
void parseString()
{
    uint8_t count = 0;
    argCount = 0;
    for(count = 0;count < MAXCHAR;count++)//the putrpose of the code in the for loop is to convert all delimiters to \0
    {
        if((strIn[count] >= 48 && strIn[count] <= 57) || (strIn[count] >= 65 && strIn[count] <= 90) || (strIn[count] >= 97 && strIn[count] <= 122) || strIn[count] == '.' || strIn[count] =='-')
        {
           strIn[count] = strIn[count]; 
        }
        else
        strIn[count] = 0;
    }

    for (count = 0;count < MAXCHAR;count++)//the code in the for loop stores the index of the words in the user input to the 'pos' array
    {
        if(count == 0 && strIn[count] != 0)
        {
            pos[argCount] = count;
            argCount++;
        }
        else if(count == 0 && strIn[count] == 0 );
        else
        {
            if(strIn[count - 1] == 0 && strIn[count] != 0)
            {
                pos[argCount] = count;
                argCount++;
            }
            else;
        }
    }

}

//This function performs a software reset
void reset()
{
    NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
}

//This function verifirs that the first argument is the verb and the correct number of arguments have been entered by the user
uint8_t isCommand(char strCmd[],uint8_t count)
{
    if (strCmd[0] == 'r' && strCmd[1] == 'e' && strCmd[2] == 's' && strCmd[3] == 'e' && strCmd[4] == 't' && count >= 0) //reset command
    return RESET;
    
    else if (strCmd[0] == 'v' && strCmd[1] == 'o' && strCmd[2] == 'l' && strCmd[3] == 't' && strCmd[4] == 'a' && strCmd[5] =='g' && strCmd[6] == 'e' && count >= 2) //voltage in command
    return VOLTAGE;

    else if (strCmd[0] == 'd' && strCmd[1] == 'c' && count >= 3) //dc command
    return DC;
    
    else if(strCmd[0] == 's' && strCmd[1] == 'i' && strCmd[2] == 'n' && strCmd[3] == 'e' && count >= 4)
    return SINE;
    
    else if(strCmd[0] == 'r' && strCmd[1] == 'u' && strCmd[2] == 'n' && count >=0)
    return RUN;
    
    else if(strCmd[0] == 's' && strCmd[1] == 't' && strCmd[2] == 'o' && strCmd[3] == 'p' && count >=0)
    return STOP;
    
    else if(strCmd[0] == 'c' && strCmd[1] == 'y' && strCmd[2] == 'c' && strCmd[3] == 'l' && strCmd[4] == 'e' && strCmd[5] == 's' && count >=0)
    return CYCLES;
    
    else if(strCmd[0] == 's' && strCmd[1] == 'q' && strCmd[2] == 'u' && strCmd[3] == 'a' && strCmd[4] == 'r' && strCmd[5] == 'e' && count >=4)
    return SQUARE;
    
    else if(strCmd[0] == 't' && strCmd[1] == 'r' && strCmd[2] == 'i' && strCmd[3] == 'a' && strCmd[4] == 'n' && strCmd[5] == 'g' && strCmd[6] == 'l' && strCmd[7] == 'e' && count >=4)
    return TRIANGLE;
    
    else if(strCmd[0] == 's' && strCmd[1] == 'a' && strCmd[2] == 'w' && strCmd[3] == 't' && strCmd[4] == 'o' && strCmd[5] == 'o' && strCmd[6] == 't' && strCmd[7] == 'h' && count >=4)
    return SAWTOOTH;
    
    else if(strCmd[0] == 'h' && strCmd[1] == 'i' && strCmd[2] == 'l' && strCmd[3] == 'b' && strCmd[4] == 'e' && strCmd[5] == 'r' && strCmd[6] == 't' && count >=2)
    return HILBERT;
    
    else if(strCmd[0] == 'd' && strCmd[1] == 'i' && strCmd[2] == 'f' && strCmd[3] == 'f' && strCmd[4] == 'e' && strCmd[5] == 'r' && strCmd[6] == 'e' && strCmd[7] == 'n' && strCmd[8] == 't' && strCmd[9] == 'i' && strCmd[10] == 'a' && strCmd[11] == 'l' && count >=2)
    return DIFFERENTIAL;
    
    else if(strCmd[0] == 'a' && strCmd[1] == 'l' && strCmd[2] == 'c' && count >=2)
    return ALC;
    
    else if(strCmd[0] == 'd' && strCmd[1] == 'o' && strCmd[2] == 'w' && strCmd[3] == 'n' && strCmd[4] == 'l' && strCmd[5] == 'o' && strCmd[6] == 'a' && strCmd[7] == 'd' && count >=2)
    return DOWNLOAD;
    
    else if(strCmd[0] == 'a' && strCmd[1] == 'r' && strCmd[2] == 'b' && count >=1)
    return ARB;
    
    else
    return 0;
}

//This function returns the value of the ADC result on the specified AIN
float readADC0voltage(uint8_t ain)
{
    int16_t ain0;
    float v;
    int16_t ain1;
    ADC0_PSSI_R |= ADC_PSSI_SS2;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    ain0 = ADC0_SSFIFO2_R;                           // get single result from the FIFO
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    ain1 = ADC0_SSFIFO2_R;                           // get single result from the FIFO
    
    if(ain == 0)
    {
        v = ((3.3*ain0+0.5)/4096)+0.05; 
    }
    else if(ain == 1)
    {
        v = ((3.3*ain1 + 0.5)/4096)+0.05; 
    }
    return v;
}

//this function writes 16 bits to the DAC
void dacWrite(uint16_t data)
{
	SSI2_DR_R = data;               // write data
	while (SSI2_SR_R & SSI_SR_BSY); //wait for transmission to end
	LDAC = 0;
    __asm (" NOP");                    // allow line to settle
	__asm (" NOP");
	__asm (" NOP");
    LDAC = 1; //pulse LDAC to load the values to the register
    
}

//This function calculates the value of R to be written to the DAC
void dcOut(uint8_t out, float voltage)
{
    uint16_t R = 0;
    uint16_t data = 0;
    float vout = 0;
    if(out == 1)
    voltage = voltage*Tcor0;
    else if(out == 2)
    voltage = voltage*Tcor1;


    if (out == 1) {
        vout = ((voltage-0.02) * 2.048/-10) + 1.024;
        R = vout * 4096 / 2.048;
        data = 12288| R; //0011
        dacWrite(data); //write to DAC A
    }
    else if(out == 2){
        vout = ((voltage-0.01) * 2.048/-10) + 1.024;
        R = vout * 4096 / 2.048;
        data = 45056| R; //1011
        dacWrite(data);//write to DAC B
    }
    dataDc = data;
    //sprintf(strOUT,"\r\n>the value of R is %d/r/n",R);
    //putsUart0(strOUT);
}

//Timer 1 ISR
void timer1Isr()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer

    if(load == 1)
    {
        if (cycles > 0){ //if the user enters a number using the cycles command
            cyclescount = cyclescount + 1;
        if (cyclescount == cycles){
                cycles = -1;
                cyclescount = 0;
                load = 0;
                //cycles = -1; //default value for cycles to continuous 
            }
        }
            RED_LED = 0;
        LDAC = 0;
        __asm (" NOP");                    // allow line to settle
        __asm (" NOP");
        __asm (" NOP");
        LDAC = 1; //pulse LDAC to load the values to the register
        
           phi0 += delta_phi0;
           phi1 += delta_phi1;
            SSI2_DR_R = LUT[0][phi0>>20];               // write data 
            //while (SSI2_SR_R & SSI_SR_BSY);
            SSI2_DR_R = LUT[1][phi1>>20];
        
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
    
}

//This function generates the look up table
void makeLut(uint8_t com, uint8_t out, float freq, float amp, float ofs)
{
    int i;
    uint16_t peak = 0;
    uint16_t base = 0;
    int increment = 0;
    /*if(out == 1)
    amp = amp*Tcor0;
    else if(out == 2)
    amp = amp*Tcor1;*/
   // delta_phi = (pow(2,32)*(freq*1.8)/100000);
   if(out == 1)
   {
        delta_phi0 = (pow(2,32)*(freq*1.2)/100000);
        delta_phi_f0 = delta_phi0;
   }
   else if (out == 2)
   {
        delta_phi1 = (pow(2,32)*(freq*1.2)/100000);
        delta_phi_f1 = delta_phi1;
   }
    LUTOut1 = 0;//LUTOut1 + 1; //initially 0
    LUTOut2 = 1; //LUTOut2 + 1; //initially 1
    float vout;

    if (com == SINE)
    {
        for (i = 0; i < 4096; i++)
        {
          // LUT[0][i] = (2048 + (ofs * 384 or 409)) + (2047*(amp/5)*sin(2*M_PI*i/4096)); 
            if(out == 1){
            LUT[LUTOut1][i] = (2097 + (ofs * -410)) + (2096*(amp/5)*sin(2*M_PI*i/4096)); 
            LUT[LUTOut1][i] = 12288|LUT[LUTOut1][i];
            }
            else if(out == 2){
            LUT[LUTOut2][i] = (2088 + (ofs * -410)) + (2087*(amp/5)*sin(2*M_PI*i/4096)); 
            LUT[LUTOut2][i] = 45056|LUT[LUTOut2][i];
            }
        }
    }
    
    else if (com == TRIANGLE)
    {
        //peak = (((amp + ofs - 0.12) * 2.048/-10) + 1.024) * 4096 / 2.048;
        vout = ((amp + ofs - 0.12) * 2.048/-10) + 1.024;
        peak = vout * 4096 / 2.048;
        //vout = ((ofs - 0.12) * 2.048/-10) + 1.024;
        //base = vout * 4096 / 2.048;
        //trough = (((-amp + offset - 0.12) * 2.048/-10) + 1.024) * 4096 / 2.048;
       // increment = peak/(4095/2 - 1);
       increment = 10;
        if(out == 1)
            LUT[LUTOut1][2047] = peak;
        else if(out == 1)
            LUT[LUTOut2][2047] = peak;
        
        for (i = 2046; i >= 0; i--)
        {
            if(out == 1){
            LUT[LUTOut1][i] = LUT[LUTOut1][i+1] + increment;
            LUT[LUTOut1][i] = 12288|LUT[LUTOut1][i];}
            else if(out == 2){
            LUT[LUTOut2][i] = LUT[LUTOut2][i+1] + increment;
            LUT[LUTOut2][i] = 45056|LUT[LUTOut2][i];}
        }
        
        for (i = 2048; i < 4096; i++)
        {
            if(out == 1){
            LUT[LUTOut1][i] = LUT[LUTOut1][i-1] + increment;
            LUT[LUTOut1][i] = 12888|LUT[LUTOut1][i];}
            else if(out == 2){
            LUT[LUTOut2][i] = LUT[LUTOut2][i-1] + increment;
            LUT[LUTOut2][i] = 45056|LUT[LUTOut2][i];}
        }
        

    }
    
        else if (com == SAWTOOTH)
    {
        //peak = (((amp + ofs - 0.12) * 2.048/-10) + 1.024) * 4096 / 2.048;
        vout = ((amp + ofs - 0.12) * 2.048/-10) + 1.024;
        peak = vout * 4096 / 2.048;
        //increment = peak/4096;
        increment = 10;
        if(out == 1)
            LUT[LUTOut1][0] = peak;
        else if(out == 1)
            LUT[LUTOut2][0] = peak;
        for (i = 1; i < 4096; i++)
        {
            if(out == 1){
                LUT[LUTOut1][i] = LUT[LUTOut1][i-1] + increment;
                LUT[LUTOut1][i] = 12288|LUT[LUTOut1][i];
            }
            else if(out == 2){
                LUT[LUTOut2][i] = LUT[LUTOut2][i-1] + increment; 
                LUT[LUTOut2][i] = 45056|LUT[LUTOut2][i];            
            }
        }
        
           
    }
    else if(com == SQUARE){
        vout = ((amp + ofs - 0.12) * 2.048/-10) + 1.024;
        peak = vout * 4096 / 2.048;
        for (i = 0; i < 2048;i++){
            if(out == 1){
                LUT[LUTOut1][i] = peak;//(((amp + ofs - 0.12) * 2.048/-10) + 1.024) * 4096 / 2.048;
                LUT[LUTOut1][i] = 12288|LUT[LUTOut1][i];
            }
            else if(out == 2){
                LUT[LUTOut2][i] = peak; //(((amp + ofs - 0.12) * 2.048/-10) + 1.024) * 4096 / 2.048;
                LUT[LUTOut2][i] = 45056|LUT[LUTOut1][i];            
            }
        }
        for (i = 2048; i < 4096;i++){
            if(out == 1){
                LUT[LUTOut1][i] = 2096;
                LUT[LUTOut1][i] = 12288 | LUT[LUTOut2][i];
            }
            else if(out == 2){
                LUT[LUTOut2][i] = 2088;
                LUT[LUTOut2][i] = 45056 | LUT[LUTOut2][i];            
            }
        }
        

    }
    else
    putsUart0("\r\n>Something has gone awry\r\n>");
}

void modLut(uint8_t com, int arg1)
{
    int i = 0;
    int j = 0;
   if(com == HILBERT)
   {
       j = 0;
       for (i = 3584; i < 4096; i++)
       {
           LUT[LUTOut2][j] = LUT[LUTOut1][i];
           LUT[LUTOut2][j] = 0x0FFF & LUT[LUTOut2][j];
            LUT[LUTOut2][j] = 45056|LUT[LUTOut2][j];
           j++;
       }
       j = 513;
       for (i = 0; i < 3584; i++)
       {
           LUT[LUTOut2][j] = LUT[LUTOut1][i];
           LUT[LUTOut2][j] = 0x0FFF & LUT[LUTOut2][j];
            LUT[LUTOut2][j] = 45056|LUT[LUTOut2][j];
           j++;
       }
   }
   else if (com == DIFFERENTIAL)
   {
       j = 0;
       for (i = 2048; i < 4096; i++)
       {
            LUT[LUTOut2][j] = LUT[LUTOut1][i];
            LUT[LUTOut2][j] = 0x0FFF & LUT[LUTOut2][j];
            LUT[LUTOut2][j] = 45056|LUT[LUTOut2][j];
           j++;
       }
       j = 2049;
       for (i = 0; i < 2048; i++)
       {
            LUT[LUTOut2][j] = LUT[LUTOut1][i];
            LUT[LUTOut2][j] = 0x0FFF & LUT[LUTOut2][j];
            LUT[LUTOut2][j] = 45056|LUT[LUTOut2][j];
           j++;
       }
      /* for (i = 0; i < LUTSIZE; i++)
       {
           LUT[LUTOut2][i] = -LUT[LUTOut1][i];
       }*/
   }
   else if (com == DOWNLOAD)
   {
       for (i = 0; i< LUTSIZE; i++)
       {
           savedLUT[i] = LUT[arg1][i];
       }
   }
   
}

void resetLUT()
{
    int i = 0;
    for(i = 0; i < 4096; i++)
    {
        LUT[LUTOut2][i] = 0;
    }
}

void alcOn()
{
    float voltageInt; //intended voltage
    float voltageAc0; //measured voltage
    float voltageAc1;
    float newVoltage;
    float voltIntWv; 
    float Z = 0;
    float T = 0;
    float Z0 = 0; //unknown impedance
    float T0 = 0;
    float Z1 = 0; //unknown impedance
    float T1 = 0;
    int i;
    //if((dataDc & 0x1000) == 12288) //if channel A
    //calibration
    //load = 0;
    dcOut(1,0);
    dcOut(2,0);
    voltageInt = 2;
    dcOut(1,voltageInt);
    dcOut(2,voltageInt);
    voltageAc0 = readADC0voltage(0);
    voltageAc1 = readADC0voltage(1);
    
   /* Z0 = (49.9*voltageAc0)/(voltageInt - voltageAc0);
    T0 = Z0/(49.9 + Z0);
    
    dcOut(2,voltageInt);
    voltageAc1 = readADC0voltage(1);
    Z1 = (49.9*voltageAc1)/(voltageInt - voltageAc1);
    T1 = Z1/(49.9 + Z1);*/
    //Z = (49.9*voltageAc0)/(voltageInt - voltageAc0);
    Tcor0 = 2/voltageAc0;
    Tcor1 = 2/voltageAc1;
    //sprintf(strOUT,"Cor 1 is %3.3f Cor 2 is: %3.3f\r\n>",Tcor0,Tcor1);
    //putsUart0(strOUT);
   /* if(downCom == DC)
    {
        putsUart0("Here");
        voltageInt = -0.002441*((dataDc & 0x0111) - 2043.9);
        if((dataDc & 0xF000) == 0x3000) //if DAC channel A
        {
            voltageAc0 = readADC0voltage(0);
            Z = (49.9*voltageAc0)/(voltageInt - voltageAc0);
            T = Z/(49.9 + Z);
            newVoltage = voltageAc0/T;
            dcOut(1,downAmp*Tcor0);
        }
        else if((dataDc & 0xF000) == 0xB000) //if DAC channel B
        {
            voltageAc1 = readADC0voltage(1);
            Z = (49.9*voltageAc1)/(voltageInt - voltageAc1);
            T = Z/(49.9 + Z);
            newVoltage = voltageAc1/T;
            dcOut(2,downAmp*Tcor1);
        }
    }*/
    /*else
    {
        load = 0;
        for (i = 0; i < LUTSIZE; i++)
        {
            voltageInt = -0.002441*((LUT[LUTOut1][i] & 0x0111) - 2043.9);
            if((dataDc & 0xF000) == 0x3000) //if DAC channel A
            {
                voltageAc0 = readADC0voltage(0);
                Z = (49.9*voltageAc0)/(voltageInt - voltageAc0);
                T = Z/(49.9 + Z);
                newVoltage = voltageAc0/T; //dividing by the transfer function of the circuit with the unknown load
                LUT[LUTOut1][i] = 12288|(((newVoltage - 0.12) * 2.048/-10) + 1.024);

            }
            else if((dataDc & 0xF000) == 0xB000) //if DAC channel B
            {
                voltageAc1 = readADC0voltage(1);
                Z = (49.9*voltageAc1)/(voltageInt - voltageAc1);
                T = Z/(49.9 + Z);
                newVoltage = voltageAc1/T;
                LUT[LUTOut2][i] = 45056|(((newVoltage - 0.12) * 2.048/-10) + 1.024);

            }
        }
        load = 1;
       // makeLut(downCom,downChan,downFreq,downAmp,downOfs);
    }*/
    /*else if(downCom == SINE)
    {
        
        voltageInt = -0.002441*((LUT[LUTOut1][1024] & 0x0111) - 2043.9);
        if((dataDc & 0xF000) == 0x3000) //if DAC channel A
        {
            voltageAc0 = readADC0voltage(0);
            Z = (49.9*voltageAc0)/(voltageInt - voltageAc0);
            T = Z/(49.9 + Z);
            newVoltage = voltageAc0/T;
            dcOut(1,newVoltage);
        }
        else if((dataDc & 0xF000) == 0xB000) //if DAC channel B
        {
            voltageAc1 = readADC0voltage(1);
            Z = (49.9*voltageAc1)/(voltageInt - voltageAc1);
            T = Z/(49.9 + Z);
            newVoltage = voltageAc1/T;
            dcOut(2,newVoltage);
        }
    }*/
   /* else if(downCom == SINE)
    {
        //voltageInt = -0.002441*((LUT[LUTOut1][1024] & 0x0111) - 2043.9);
        if(downChan == 1)
        {
            newVoltage = downAmp/T0;
            makeLut(downCom,downChan,downFreq,newVoltage,downOfs);
        }
        else if(downChan == 2)
        {
            newVoltage = downAmp/T1;
            makeLut(downCom,downChan,downFreq,newVoltage,downOfs);
        }
    }*/
    
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    /*To seperate argument string
    1. use getString() function
    2. use parseString() function
    3. type &strIn[pos[x]] where x is the argument number (1st argument, 2nd argument, etc)
    4. use atoi(&strIn[pos[x]]) to make the argument an integer
    5. use atof(&strIn[pos[x]]) to make the argument a float
    */
    //char* add;
	// Initialize hardware
	initHw();
	int x;
	int i;
	uint8_t com;
	int arg1 = 0;
	float arg2 = 0;
	float arg3 = 0;
	float arg4 = 0;
	float arg5 = 0;

	uint8_t downL = 1;

	LDAC = 1;
	//DAC_CS = 1;
	RED_LED = 1;
    
	// Display greeting
    putsUart0("Project\r\n Enter a command\r\n");
    putcUart0('>');
    step1();
    while(1)
    {
        getString();
        //putsUart0(strIn);
        //putsUart0("\r\n >");
        parseString();
        com = isCommand(&strIn[pos[0]],argCount);

        switch(com){
            case 0:
                putsUart0("\r\n>invalid command\r\n>");
            break;
            
            case RESET:
                reset();
            break;
            
            case VOLTAGE:
            
                arg1 = atoi(&strIn[pos[1]]);
                putsUart0("\r\n>");
                sprintf(strOUT,"The voltage on %d is: %3.3f\r\n>",arg1,readADC0voltage(arg1));
                putsUart0(strOUT);
            break;
            
            case DC:
                downCom = com;
                
                arg1 = atoi(&strIn[pos[1]]);
                arg2 = atof(&strIn[pos[2]]);
                dcOut(arg1,arg2);
                //downAmp = arg2;
                putsUart0("\r\n>");
            break;
            
            case SINE:

                arg1 = atoi(&strIn[pos[1]]);
                arg2 = atof(&strIn[pos[2]]);
                arg3 = atof(&strIn[pos[3]]);
                if(argCount == 5)
                arg4 = atof(&strIn[pos[4]]);
                type = SINE;
               
                if(downL == 1)
                {
                    downCom = com;
                    downChan = arg1;
                    downFreq = arg2;
	                downAmp = arg3;
    	            downOfs = arg4;
    	            //downL = 0;
                }
                else;
                makeLut(com,arg1,arg2,arg3,arg4);
                if (load == 0)
                putsUart0("\r\n>Enter 'run' to play sine\r\n>");
                else
                putsUart0("\r\n>Now playing Sine\r\n>");
            break;
            
            case RUN:
                load = 1;
                putsUart0("\r\n>");
            break;
            
            case STOP:
                load = 0;
                putsUart0("\r\n>");
            break;
            
            case CYCLES:
                arg1 = atoi(&strIn[pos[1]]);
                
               // cycles = atoi(&strIn[pos[1]]) * 4096/(((delta_phi>>20)+1));
                if (arg1 == 1)
                    cycles = atoi(&strIn[pos[2]]) * 4096/(((delta_phi_f0/pow(2,20))+1));
                else if (arg1 == 2)
                    cycles = atoi(&strIn[pos[2]]) * 4096/(((delta_phi_f1/pow(2,20))+1));
               
                putsUart0("\r\n>");
            break;
            
            case SQUARE:
                downCom = com;
                arg1 = atoi(&strIn[pos[1]]);
                arg2 = atof(&strIn[pos[2]]);
                arg3 = atof(&strIn[pos[3]]);
                if(argCount == 5)
                arg4 = atof(&strIn[pos[4]]);
                if(argCount == 6)
                arg5 = atof(&strIn[pos[5]]);
                type = SQUARE;
                //square(arg1,arg2,arg3,arg4,arg5);

                if(downL == 1)
                {
                    downCom = com;
                    downChan = arg1;
                    downFreq = arg2;
	                downAmp = arg3;
    	            downOfs = arg4;
    	            //downL = 0;
                }
                else;
                makeLut(com,arg1,arg2,arg3,arg4);
                if (load == 0)
                putsUart0("\r\n>Enter 'run' to play square\r\n>");
                else
                putsUart0("\r\n>Now playing Square\r\n>");
            break;
            
            case TRIANGLE:
                downCom = com;
                arg1 = atoi(&strIn[pos[1]]);
                arg2 = atof(&strIn[pos[2]]);
                arg3 = atof(&strIn[pos[3]]);
                if(argCount == 5)
                arg4 = atof(&strIn[pos[4]]);
                type = TRIANGLE;
                
                if(downL == 1)
                {
                    downCom = com;
                    downChan = arg1;
                    downFreq = arg2;
	                downAmp = arg3;
    	            downOfs = arg4;
    	            //downL = 0;
                }
                else;
                makeLut(com,arg1,arg2,arg3,arg4);
                if (load == 0)
                putsUart0("\r\n>Enter 'run' to play triangle\r\n>");
                else
                putsUart0("\r\n>Now playing Triangle\r\n>");
            break;
            
            case SAWTOOTH:
                downCom = com;
                arg1 = atoi(&strIn[pos[1]]);
                arg2 = atof(&strIn[pos[2]]);
                arg3 = atof(&strIn[pos[3]]);
                if(argCount == 5)
                arg4 = atof(&strIn[pos[4]]);
                type = SAWTOOTH;
                
                if(downL == 1)
                {
                    downCom = com;
                    downChan = arg1;
                    downFreq = arg2;
	                downAmp = arg3;
    	            downOfs = arg4;
    	            //downL = 0;
                }
                else;
                makeLut(com,arg1,arg2,arg3,arg4);
                if (load == 0)
                putsUart0("\r\n>Enter 'run' to play sawtooth\r\n>");
                else
                putsUart0("\r\n>Now playing Sawtooth\r\n>");
            break;
            
            case HILBERT:
                if (strIn[pos[1]] == 'o' && strIn[pos[1] + 1] == 'n' )
                {
                    putsUart0("\r\n>Hilbert on\r\n>");
                    modLut(com,arg1);
                }
                else{
                    resetLUT();
                    putsUart0("\r\n>Hilbert off\r\n>");
                }
            break;
            
            case DIFFERENTIAL:
                if (strIn[pos[1]] == 'o' && strIn[pos[1] + 1] == 'n' )
                {
                    putsUart0("\r\n>Differential on\r\n>");
                    modLut(com,arg1);
                }
                else{
                    resetLUT();
                    putsUart0("\r\n>Differential off\r\n>");
                }
            break;
            
            case ALC:
            
                if (strIn[pos[1]] == 'o' && strIn[pos[1] + 1] == 'n' )
                {
                    load = 0;
                    waitMicrosecond(500000);
                    alcOn();
                    if(downCom != DC)
                    {
                        if(downChan == 1)
                        makeLut(downCom,downChan,downFreq,downAmp*Tcor0,downOfs);
                        else if(downChan == 2)
                        makeLut(downCom,downChan,downFreq,downAmp*Tcor1,downOfs);
                    }
                    load = 1;
                    putsUart0("\r\n>ALC on\r\n>");
                }
                else
                {
                    Tcor0 = 1;
                    Tcor1 = 1;
                    if(downChan == 1)
                        makeLut(downCom,downChan,downFreq,downAmp*Tcor0,downOfs);
                    else if(downChan == 2)
                        makeLut(downCom,downChan,downFreq,downAmp*Tcor1,downOfs);

                    putsUart0("\r\n>ALC off\r\n>");
                }
            break;
            
            case DOWNLOAD:
                //arg1 = atoi(&strIn[pos[1]]);
                
                if (strIn[pos[1]] == 'o' && strIn[pos[1] + 1] == 'n' )
                {
                    downL = 1;
                    putsUart0("\r\n>Download On!\r\n>The next waveform you create can played later by entering 'arb'\r\n>");
                }
                else
                {
                    downL = 0;
                    putsUart0("\r\n>Download Off!\r\n>");
                }
            break;
            
            case ARB:
                /*for (i = 0; i< LUTSIZE; i++)
                {
                    LUT[downChan][i] = savedLUT[i]; 
                }*/
                load = 0;
                makeLut(downCom,downChan,downFreq,downAmp,downOfs);
                putsUart0("\r\n>Enter 'run' to play previously downloaded waveform\r\n>");
            break;
            
                
           

               
        }
    }
}




