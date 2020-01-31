
// Project      :   Function generator
// Student Name :   Rahul Kidecha
// Student ID   :   1001735009


// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) //  PF3
#define CS_N          (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4))) //PB5 ~CS SSI2FSS
#define LDAC_N        (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) //PA5 ~LDAC
//#define LDAC_N          (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) //PF1 ~LDAC

#define GREEN_LED_MASK 8
#define AIN0_MASK 8      //PE3
#define AIN1_MASK 4      //PE2
//#define LDAC_N_MASK 2
#define LDAC_N_MASK 32

#define Buffer_Max  80  // Maximum size of input command buffer.
#define MAX_Args 6      // Maximum number of arguments for any available commands.

char input[Buffer_Max]; // Input buffer
char type [MAX_Args];   // Type of arguments in input buffer.
char* commands[14] = {"reset","dc","sine","square","sawtooth","triangle","run","stop","voltage","cycle","gain","differential","hilbert","alc"}; // Currently available commands.
char* mode[2]={"on","off"};
char str[100];
char cmd;
uint8_t Min_Args[14] = {0,2,3,3,3,3,0,0,1,2,2,1,1,1};

uint8_t argc,i,j,k,count;              // Argument count and general purpose variables.
uint8_t pos[MAX_Args];                // Position of arguments in input buffer.


//Parameter and voltage calibration
uint8_t Channel;
float Voltage,Freq;
float Offset=0;
float duty=0;
float Freq_1,Freq_2;
float Vin,Vin0,Vin1;
float Freq_E,Freq_S;
int16_t m1=-383;
int16_t m2=-385;
uint16_t data =0;
uint16_t ldacDC1 =2070+12288;
uint16_t ldacDC2 = 2075+45056;

//lookuptable variable
uint16_t L;
uint16_t LUT1[4095];
uint16_t LUT2[4095];
uint16_t LUT3[4095];
uint8_t dc_on1,dc_on2=1;

//Frequency variables
uint32_t Phi1=0;
uint32_t Phi2=0;
uint32_t dPhi1,dPhi2;

//Run variable
uint8_t run=0;
uint32_t cycle1=0;
uint32_t cycle2=0;
uint8_t cycle1_Mode=0;
uint8_t cycle2_Mode=0;
int16_t raw,raw0,raw1;


//Duty
uint16_t T;
uint8_t Square_on,Sawtooth_on,Sine_on,Triangle_on=0;
uint8_t Sine_on2=0;

/****************************************************************************************************/

char getcUart0()
{

    while (UART0_FR_R & UART_FR_RXFE);   // Checking UART RX flag.
    return UART0_DR_R & 0xFF;
}


void initHw()
{
// Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F,A,B and E peripherals
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK;  // enable LEDs and pushbuttons

    // Configure UART0 pins

       SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // Turn-on UART0, leave other uarts in same status
       GPIO_PORTA_DEN_R |= 3;                           // Default, added for clarity
       GPIO_PORTA_AFSEL_R |= 3;                         // Default, added for clarity
       GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)

       UART0_CTL_R |= 0;                                 // Turn-off UART0 to allow safe programming
       UART0_CC_R |= UART_CC_CS_SYSCLK;                  // Use system clock (40 MHz)
       UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
       UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
       UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // Configure for 8N1 w/ 16-level FIFO
       UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // Enable TX, RX, and module
}

void initSSI()

{
    //SSI configuration
    SYSCTL_RCGCSSI_R = SYSCTL_RCGCSSI_R2;            // turn-on SSI2 clocking


//        Configure DAC (for port F)
//        GPIO_PORTF_DIR_R |= LDAC_N_MASK ;       // make bit 1 outputs PF1
//        GPIO_PORTF_DR2R_R |=LDAC_N_MASK;      // set drive strength to 2mA
//        GPIO_PORTF_DEN_R |= LDAC_N_MASK;       // enable bit 1 digital

        // Configure DAC (for port A)
        GPIO_PORTA_DIR_R |= LDAC_N_MASK ;       // make bit 5 output PA5
        GPIO_PORTA_DR2R_R |=LDAC_N_MASK;      // set drive strength to 2mA
        GPIO_PORTA_DEN_R |= LDAC_N_MASK;       // enable bit 5 digital


        // Configure SSI2 pins for SPI configuration
         GPIO_PORTB_DIR_R |= 0xB0;                        // make bits pb4(ssi2clk) and pb7(ssi2tx) outputs pb5(ssi2fss)
         GPIO_PORTB_DR2R_R |= 0xB0;                       // set drive strength to 2mA
         GPIO_PORTB_AFSEL_R |= 0xB0;                      // select alternative functions for MOSI, SCLK pins
         GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK|GPIO_PCTL_PB5_SSI2FSS  ; // map alt fns to SSI2
         GPIO_PORTB_DEN_R |= 0xB0;                        // enable digital operation on TX, CLK pins
         GPIO_PORTB_PUR_R |= 0x30;                        // must be enabled when SPO=1

        // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
            SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
            SSI2_CR1_R = 0;                                  // select master mode
            SSI2_CC_R = 0;                                   // select system clock as the clock source
            SSI2_CPSR_R = 10;                                // set bit rate to 1 MHz (if SR=0 in CR0)
            SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
            SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

         // LDAC_N=1;

    }

void initTimer()
{
    // Configure Timer 1 as the time base


       TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
       TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
       TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
       TIMER1_TAILR_R = 400;                            // set load value to 40e6 for 1 Hz interrupt rate
       TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
       NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
       TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}

void initADC()
{
        SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;

    //configure AINO and AIN1
        GPIO_PORTE_AFSEL_R |= AIN1_MASK | AIN0_MASK;                 // select alternative functions for AN10 and AIN1
        GPIO_PORTE_DEN_R &= ~(AIN1_MASK | AIN0_MASK);                // turn off digital operation on pin PE2 and PE3
        GPIO_PORTE_AMSEL_R |= AIN1_MASK | AIN0_MASK;                 // turn on analog operation on pin PE2 and PE3


        // Configure ADC
        ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
        ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
        ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
        ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
        ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}


//Function to generate wait states.

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

/******************************************************************************************************/


//Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);   // Checking UART TX Flag
    UART0_DR_R = c;
}


// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}


//Function to get character from the user with backspace support and store it in input buffer if it is printable.

void getsUart0()
{

     uint8_t c;
     uint8_t count = 0;
A1:  c = getcUart0();
     putcUart0(c);
     if (c == 127) { //check backspace
         if (count == 0)// Checking count
             goto A1;

         else if (count > 0)
         {
              count = count-1;
              goto A1;
         }
     }
      else
          {
         if(c != 0x0D) //  Enter terminates entry mode.
         {
             if(c < 0x20)  // Check for printability of character.
                 goto A1;
             else
            {
             input[count] = tolower(c); // All entries to the input buffer are converted to lowercase
             count++;
             if (count < Buffer_Max)     // Checking if buffer is full.
             {
                 goto A1;

             }
             else {
                 putsUart0("Buffer is full\r\n\r\n");
                 return;
             }
            }
         }
         else
         {
             input[count] = '\0';
             return;
         }
     }
}


// Parse string function
void parsestring()
{

j = 0;
k = 0;
bool flag = true;
uint8_t l = strlen(input);


for(i=0;i<l;i++)     // Loop to parse the entries after the first entry.
{
if(isspace(input[i]) || input[i] ==',')
{
    input[i] = '\0';
    flag = true;
    continue;
}
if(isalpha(input[i]) || isdigit(input[i]) || input[i] == '.'||input[i] == '-')
{
    if (flag)
    {
        argc++;
        pos[j++] = i;
        flag = false;
    }
}
}
}


// Reset function
void Reset()
{
    putsUart0("\r\n Reset... \r\n");
    waitMicrosecond(2000000);
    NVIC_APINT_R = 0x04 | (0x05FA << 16);
 }


// Send data into DAC
void loadLDAC(float v,uint8_t Chn)
{

      if (Chn == 1)
    {
        putsUart0("Channel 1 selected");
        putsUart0("\r\n");
        data=v*m1+2070+12288;
        ldacDC1 = data;
    }
    else if (Chn == 2)
    {
        putsUart0("Channel 2 selected");
        putsUart0("\r\n");
        data=v*m2+2075+45056;
        ldacDC2 = data;
    }
    }


// DC function
void DCout()
{
   // putsUart0("\r\n DC... \r\n");
    Channel =atoi(&input[pos[1]]);
    Voltage =atof(&input[pos[2]]);
    if(Channel==1){
    dc_on1=1;
    }if(Channel==2){
        dc_on2= 1;
    }
    loadLDAC(Voltage,Channel);
 }


// Sine function
void Sineout()
{
  //     putsUart0("\r\n Sine... \r\n");
       Channel =atoi(&input[pos[1]]);
       Voltage=atof(&input[pos[2]]);
       Freq=atof(&input[pos[3]]);
       Offset=atof(&input[pos[4]]);


       if (Channel==1)
       {dPhi1=pow(2,32)*Freq/100000;
       dc_on1=0;
       Sine_on=1;
       Sawtooth_on=0;
       Square_on=0;
       Triangle_on=0;
       cycle1_Mode=0;
       Freq_1=Freq;
           for (L=0;L<4096;L++)
           {
               LUT1[L]=m1*((Voltage)*(sin(2*3.14*L/4096))+Offset)+2070+12288;
           }
       }
       else if(Channel==2)
       {dPhi2=pow(2,32)*Freq/100000;
       dc_on2=0;
       Sine_on2=1;
       cycle2_Mode=0;
       Freq_2=Freq;
           for (L=0;L<4096;L++)
           {
               LUT2[L]=m2*((Voltage)*(sin(2*3.14*L/4096))+Offset)+2075+45056;

           }
       }
}


// Square function
void Sqaureout()
{
   //  putsUart0("\r\n Square... \r\n");
       Channel =atoi(&input[pos[1]]);
       Voltage=atof(&input[pos[2]]);
       Freq=atof(&input[pos[3]]);
       Offset=atof(&input[pos[4]]);
       duty=atof(&input[pos[5]]);


if(duty==0.0)

    {
           if (Channel==1)
       {dPhi1=pow(2,32)*Freq/100000;
       dc_on1=0;
       Sine_on=0;
       Sawtooth_on=0;
       Square_on=1;
       Triangle_on=0;
       cycle1_Mode=0;
       Freq_1=Freq;
           for (L=0;L<4096;L++)
           {
               if(L<2048)
               {
                   LUT1[L]=m1*(Voltage+Offset)+2070+12288;
               }
               else
               {
                   LUT1[L]=m1*(-Voltage+Offset)+2070+12288;
               }
           }
       }
       else if(Channel==2)
       {dPhi2=pow(2,32)*Freq/100000;
       dc_on2=0;
       cycle2_Mode=0;
       Freq_2=Freq;
           for (L=0;L<4096;L++)
           {
               if(L<2048)
               {
                   LUT2[L]=m2*(Voltage+Offset)+2075+45056;
               }
               else
               {
                   LUT2[L]=m2*(-Voltage+Offset)+2075+45056;
               }

           }
       }
  }

else
{
    T=4096*duty/100;

    if (Channel==1)
    {dPhi1=pow(2,32)*Freq/100000;
    dc_on1=0;
    Sine_on=0;
    Sawtooth_on=0;
    Square_on=1;
    Triangle_on=0;
    cycle1_Mode=0;
    Freq_1=Freq;
        for (L=0;L<4096;L++)
        {
            if(L<T)
            {
                LUT1[L]=m1*(Voltage+Offset)+2070+12288;
            }
            else
            {
                LUT1[L]=m1*(-Voltage+Offset)+2070+12288;
            }
        }
    }
    else if(Channel==2)
    {dPhi2=pow(2,32)*Freq/100000;
    dc_on2=0;
    cycle2_Mode=0;
    Freq_2=Freq;
        for (L=0;L<4096;L++)
        {
            if(L<T)
            {
                LUT2[L]=m2*(Voltage+Offset)+2075+45056;
            }
            else
            {
                LUT2[L]=m2*(-Voltage+Offset)+2075+45056;
            }

        }
    }
}



}


// Sawtooth function
void Sawtoothout()
{
   //  putsUart0("\r\n Sawtooth... \r\n");
       Channel =atoi(&input[pos[1]]);
       Voltage=atof(&input[pos[2]]);
       Freq=atof(&input[pos[3]]);
       Offset=atof(&input[pos[4]]);


       if (Channel==1)
       {dPhi1=pow(2,32)*Freq/100000;
       dc_on1=0;
       Sine_on=0;
       Sawtooth_on=1;
       Square_on=0;
       Triangle_on=0;
       cycle1_Mode=0;
       Freq_1=Freq;


       for (L=0;L<4096;L++)
           {
           LUT1[L]=m1*((Voltage*L/4096)+Offset)+2070+12288;
           }
       }
       else if(Channel==2)

       {dPhi2=pow(2,32)*Freq/100000;
       dc_on2=0;
       cycle2_Mode=0;
       Freq_2=Freq;

           for (L=0;L<4096;L++)
           {
            LUT2[L]=m2*((Voltage*L/4096)+Offset)+2075+45056;
           }
       }
}


// Triangle function
void Triangleout()
{
    // putsUart0("\r\n Triangle... \r\n");
       Channel =atoi(&input[pos[1]]);
       Voltage=atof(&input[pos[2]]);
       Freq=atof(&input[pos[3]]);
       Offset=atof(&input[pos[4]]);


       if (Channel==1)

       {dPhi1=pow(2,32)*Freq/100000;
       dc_on1=0;
       Sine_on=0;
       Sawtooth_on=0;
       Square_on=0;
       Triangle_on=1;
       cycle1_Mode=0;
       Freq_1=Freq;

       for (L=0;L<4096;L++)
           {
           if (L<2048)
           {
           LUT1[L]=m1*((Voltage*L/2048)+Offset)+2070+12288;
           } else
           {
           LUT1[L]=m1*((-Voltage*L/2048)+(2*Voltage)+Offset)+2070+12288;
           }
           }
       }
       else if(Channel==2)

       {dPhi2=pow(2,32)*Freq/100000;
       dc_on2=0;
       cycle2_Mode=0;
       Freq_2=Freq;

       for (L=0;L<4096;L++)
           {
           if (L<2048)
           {
           LUT2[L]=m2*((Voltage*L/2048)+Offset)+2075+45056;
           } else
           {
           LUT2[L]=m2*((-Voltage*L/2048)+(2*Voltage)+Offset)+2075+45056;
           }
           }

       }

}

//Run function
void Run()
{
run=1;
}


//Stop function
void Stop()
{
run=0;
}

// Request and read one sample from SS3
int16_t readAdc0ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}


//ADC function
void ADC()
{
    uint8_t IN;
  //  int16_t raw;

    IN=atoi(&input[pos[1]]);

    if (IN==0)
    {
        ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
    }
    else if(IN==1)
    {
        ADC0_SSMUX3_R = 1;                               // set first sample to AIN1
    }else
    {
        putsUart0("Invalid pin!\r\n\r\n");
    }

   raw=readAdc0ss3();
   Vin = (3.3 * (raw+0.5) / 4096);
   sprintf(str, "\r\n Voltage ADC:    %4.1f\r\n", Vin);
   putsUart0(str);

}

// Cycle count function
void Cycle()
{
uint8_t Number=0;
Channel=atoi(&input[pos[1]]);
Number=atoi(&input[pos[2]]);
if (Channel==1)
{
    cycle1=100000/Freq_1*Number;
    cycle1_Mode=1;

}else if(Channel==2)
{

    cycle2=100000/Freq_2*Number;
    cycle2_Mode=1;

}else
{
 cycle1_Mode=0;
 cycle2_Mode=0;
 putsUart0("\n\r cycle mode is continous \n\r");
}
}


// Gain calculation

void gain()
{
    float g;
    float gain=0.0;
    Freq_S=atof(&input[pos[1]]);
    Freq_E=atof(&input[pos[2]]);

    for (g=Freq_S;g<=Freq_E;g=g+500)
    {
       dPhi1=pow(2,32)*g/100000;
        dc_on1=0;
     //   Sine_on=1;

        for (L=0;L<4096;L++)
           {
             LUT1[L]=m1*((4)*(sin(2*3.14*L/4096))+0)+2070+12288;
           }
       waitMicrosecond(2000000); //2 second display

       ADC0_SSMUX3_R = 0;
       raw0=readAdc0ss3();
       Vin0 = (3.3 * (raw0+0.5) / 4096);
       ADC0_SSMUX3_R = 1;
       raw1=readAdc0ss3();
       Vin1 = (3.3 * (raw1+0.5) / 4096);
       gain=Vin1/Vin0;

       sprintf(str,"\r\n voltage at AIN0:    %4.1f\r\n", Vin0);
       putsUart0(str);
       sprintf(str,"\r\n voltage at AIN1:    %4.1f\r\n", Vin1);
        putsUart0(str);
        sprintf(str, "\r\n Gain is:    %4.1f\r\n", gain);
        putsUart0(str);
        putsUart0("\r\n");

       dc_on1=1;
    }

}

//Differential mode
void Differential()
{

    if(strcmp(&input[pos[1]],mode[0])==0)  // on mode
    {
        if (Offset==0)
        {
         if(Sine_on==1)
            {dPhi2=0;
             dPhi2=dPhi1;
            dc_on2=0;
            for (L=0;L<4096;L++)
            {
                              LUT2[L]=m2*((-1)*(Voltage)*(sin(2*3.14*L/4096))+Offset)+2075+45056;

            }
            }else if (Sawtooth_on==1)
            {
            dPhi2=0;
            dPhi2=dPhi1;
            dc_on2=0;
                for (L=0;L<4096;L++)
                {
                 LUT2[L]=m2*((-1)*(Voltage*L/4096)+Offset)+2075+45056;
                }
            }else if (Square_on==1)
            {
            dPhi2=0;
            dc_on2=0;
            dPhi2=dPhi1;
                for (L=0;L<4096;L++)
                {
                    if(L<2048)
                    {
                        LUT2[L]=m2*((-1)*Voltage+Offset)+2075+45056;
                    }
                    else
                    {
                        LUT2[L]=m2*(Voltage+Offset)+2075+45056;
                    }
                }

        }else if (Triangle_on==1)
        {
            dPhi2=0;
            dc_on2=0;
            dPhi2=dPhi1;

            for (L=0;L<4096;L++)
                {
                if (L<2048)
                {
                 LUT2[L]=m2*((-Voltage*L/2048)+Offset)+2075+45056;

                } else
                {

                LUT2[L]=m2*((Voltage*L/2048)+(2*-Voltage)+Offset)+2075+45056;
                }
                }
        }
        }else
        {
         putsUart0("\n\r Invalid command \n\r");
        }
    }else if((strcmp(&input[pos[1]],mode[1])==0))  // off mode
    {

        for (L=0;L<4096;L++)
    {
        LUT2[L]=2075+45056;  // pass zero voltage
    }
     Sine_on=0;
     Sawtooth_on=0;
     Square_on=0;
     Triangle_on=0;
    }else
     {
        putsUart0("\n\r Invalid command \n\r");
     }

}


//hilbert function

void hilbert()

{
    if(strcmp(&input[pos[1]],mode[0])==0)  // on mode
    {if(Offset==0)
    {if (Sine_on==1)
    {
    {dPhi2=0;
    dPhi2=dPhi1;
    dc_on2=0;
    for (L=0;L<4096;L++)
    {
        LUT2[L]=m2*((-1)*(Voltage)*(cos(2*3.14*L/4096))+Offset)+2075+45056;

    }
    }
    }
    }

    }else if(strcmp(&input[pos[1]],mode[1])==0)  // off mode
    {
        for (L=0;L<4096;L++)
    {
        LUT2[L]=2075+45056;  // pass zero voltage
    }
        Sine_on=0;
    }else
     {
        putsUart0("\n\r Invalid command \n\r");
     }


}

//ALC


 void ALC()
 {
       float Re=0.0;
       float E=0.0;
       Vin0=0;

     if(strcmp(&input[pos[1]],mode[0])==0)
       {
       ADC0_SSMUX3_R = 0;
       raw0=readAdc0ss3();
       Vin0 = (3.3 * (raw0+0.5) / 4096);
       sprintf(str,"\r\n voltage at AIN0:    %4.1f\r\n", Vin0);
       putsUart0(str);

       Re=(49*Vin0)/(Voltage-Vin0);

       E=Re/(50+Re);

       if (Sine_on==1)

       {
           dPhi1=pow(2,32)*Freq/100000;
           for (L=0;L<4096;L++)
           {
               LUT1[L]=m1*((Voltage/E)*(sin(2*3.14*L/4096))+Offset)+2070+12288;
           }


       }else if (Sine_on2==1)

       { dPhi2=pow(2,32)*Freq/100000;
       dc_on2=0;
           for (L=0;L<4096;L++)
           {
               LUT2[L]=m2*((Voltage/E)*(sin(2*3.14*L/4096))+Offset)+2075+45056;

           }
        }
        }
        else if(strcmp(&input[pos[1]],mode[1])==0)  // off mode
        {
            if(Sine_on==1)
            {
        for (L=0;L<4096;L++)
        {
            LUT1[L]=m1*((Voltage)*(sin(2*3.14*L/4096))+Offset)+2070+12288;  // Channel 1
        }
        Sine_on=0;
            }else if(Sine_on2==1)
            {
            for (L=0;L<4096;L++)
            {
                LUT2[L]=m2*((Voltage)*(sin(2*3.14*L/4096))+Offset)+2075+45056;  // Channel 2
            }
            Sine_on2=0;
        }

        }else
         {
        putsUart0("\n\r Invalid command \n\r");
         }
 }





// Timer ISR function
void timer1Isr()
{

    if (run==1)   // run mode on
    {
       if(dc_on1==0)  // other wave for channel 1
       {
     if(cycle1_Mode==0){
           {
           Phi1=Phi1+dPhi1;        // channel 1

    LDAC_N=1;
    SSI2_DR_R=LUT1[Phi1>>20];
    LDAC_N=0;
       }
     }
     else if (cycle1_Mode==1){
         if (cycle1!=0)         // check cycle count not equal to zero
         { cycle1=cycle1-1;
             Phi1=Phi1+dPhi1;        // channel 1

         LDAC_N=1;
         SSI2_DR_R=LUT1[Phi1>>20];
         LDAC_N=0;
     }if(cycle1==0)
     {
         LDAC_N=1;
         SSI2_DR_R=2070+12288;
         LDAC_N=0;
         }
         }
       }

  //__asm("             NOP");

    if(dc_on2==0)     // Other wave for channel 2
    {
    if(cycle2_Mode==0){
          {
          Phi2=Phi2+dPhi2;        // channel 1

   LDAC_N=1;
   SSI2_DR_R=LUT2[Phi2>>20];
   LDAC_N=0;
      }
    }
    else if (cycle2_Mode==1){
        if (cycle2!=0)
        { cycle2=cycle2-1;
            Phi2=Phi2+dPhi2;        // channel 1

        LDAC_N=1;
        SSI2_DR_R=LUT2[Phi2>>20];
        LDAC_N=0;
    }if(cycle1==0){
        LDAC_N=1;
        SSI2_DR_R=2075+45056;
        LDAC_N=0;
        }
    }
}

if(dc_on1==1)   // for DC channel 1
{
    LDAC_N =1;
    SSI2_DR_R = ldacDC1;
    LDAC_N =0;
  //  __asm("             NOP");
   // __asm("             NOP");
  //  __asm("             NOP");
   // __asm("             NOP");
}

if(dc_on2==1)  // for DC channel 2
{

    LDAC_N =1;
    SSI2_DR_R = ldacDC2;
    LDAC_N =0;
  //  __asm("             NOP");
}
    }

    else if (run==0) // stop mode on
    {
        LDAC_N=1;
        SSI2_DR_R=2070+12288;
        LDAC_N=0;

     // __asm("             NOP");

        LDAC_N=1;
        SSI2_DR_R=2075+45056;
        LDAC_N=0;
    }

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}


//Function to find the operation
char IsCommand(){

    if(strcmp(&input[pos[0]],commands[0])==0) // Checking if operation is  available in  command database
      {

        putsUart0("Reset Selected!\r\n\r\n");
             Reset();
      }
     else if (strcmp(&input[pos[0]],commands[1])==0)

     {
         if(argc-1>=Min_Args[1])
         {
         putsUart0("DC Selected!\r\n\r\n");
               DCout();  }      // DC
         else
         {
         putsUart0("Insufficient Data!\r\n\r\n");
         }
     }
     else if (strcmp(&input[pos[0]],commands[2])==0)
     {
         if(argc-1>=Min_Args[2])
         {
         putsUart0("Sine Selected!\r\n\r\n");
               Sineout();         // Sine
         } else
         {
          putsUart0("Insufficient Data!\r\n\r\n");
         }
     }
     else if (strcmp(&input[pos[0]],commands[3])==0)
     {
         if(argc-1>=Min_Args[3])
         {
             putsUart0("Sqaure Selected!\r\n\r\n");
             Sqaureout();        // square

         } else
         {
             putsUart0("Insufficient Data!\r\n\r\n");
         }

     }
     else if (strcmp(&input[pos[0]],commands[4])==0)
     {
        if(argc-1>=Min_Args[4])
        {
         putsUart0("Sawtooth Selected!\r\n\r\n");
                Sawtoothout();         // Sawtooth
        } else
        {
            putsUart0("Insufficient Data!\r\n\r\n");
        }
     }
     else if (strcmp(&input[pos[0]],commands[5])==0)
     {
        if(argc-1>=Min_Args[5])
        {
         putsUart0("Triangle Selected!\r\n\r\n");
         Triangleout();       // triangle
         } else
         {
             putsUart0("Insufficient Data!\r\n\r\n");
         }

     }
     else if (strcmp(&input[pos[0]],commands[6])==0)
     {
         putsUart0("Run waveforms \r\n\r\n");
             Run();       // run
     }
     else if (strcmp(&input[pos[0]],commands[7])==0)
     {
         putsUart0("Stop waveforms!\r\n\r\n");
             Stop();       // stop

     }else if (strcmp(&input[pos[0]],commands[8])==0)
     {
         if(argc-1>=Min_Args[8])
         {
         putsUart0("Read input voltage!\r\n\r\n");
            ADC();       // ReadADC
         } else
         {
             putsUart0("Insufficient Data!\r\n\r\n");
         }

     }else if (strcmp(&input[pos[0]],commands[9])==0)
     {
         if(argc-1>=Min_Args[9])
         {
         putsUart0("Cycle!\r\n\r\n");
            Cycle();       // cycle count
         }else
         {
         putsUart0("Insufficient Data!\r\n\r\n");
         }
     }else if (strcmp(&input[pos[0]],commands[10])==0)
     {
         if(argc-1>=Min_Args[10])
         {
         putsUart0("Gain!\r\n\r\n");
            gain();       // gain calculation
         }else
         {
         putsUart0("Insufficient Data!\r\n\r\n");
         }
     } else if (strcmp(&input[pos[0]],commands[11])==0)
     {
         putsUart0("Differential selected!\r\n\r\n");
         Differential();       // Differential calculation

     }else if (strcmp(&input[pos[0]],commands[12])==0)
     {
         putsUart0("Hilbert selected!\r\n\r\n");
         hilbert();       // hilbert calculation
     }else if (strcmp(&input[pos[0]],commands[13])==0)
     {
         putsUart0("ALC selected!\r\n\r\n");
         ALC();       // ALC calculation
     }else
     {
         putsUart0("Invalid Command!\r\n\r\n"); // Operation not supported by program
      }
return 0;
}

/*****************************************************************************************/


//Main Program

int main(void)

{
 // Initialize hardware
   initHw();
   initTimer();
   initSSI();
   initADC();

// Routine to check if device is working
   GREEN_LED = 1;
   waitMicrosecond(1000000);// Wait of 100ms (40 clocks/us)
   GREEN_LED = 0;
   waitMicrosecond(1000000); // Wait of 100ms (40 clocks/us)

         putsUart0("Use space and ; delimiter between arguments!\r\n\r\n");
         putsUart0("Reset,DC,Sine,Square,Sawtooth,triangle,run,stop,voltagein,cycle,gain \r\n\r\n");
         putsUart0("\r\n");

      while(1) //Continue asking for commands until reset.
      {
      argc = 0;

      getsUart0();

      putsUart0("Input string is\r\n");
      putsUart0("\r\n");

      putsUart0(input);
      putsUart0("\r\n");

      parsestring(); // Parse input string

//        for (i=0;i<j;i++) // to display the Arguments
//       {
//          putsUart0("Argument is :");
//          putsUart0(&input[pos[i]]);
//          putsUart0("\r\n");
//          putsUart0("\r\n");
//
//        }

        cmd = IsCommand(); // Compare command

        timer1Isr();
      }
}
