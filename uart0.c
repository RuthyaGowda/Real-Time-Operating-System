
// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
//parse
#define MAX_CHARS 80
char strbuffer [MAX_CHARS + 1];
#define MAX_ARGS 5
#define CHECK_NUM(x) ((x>=48 && x<=57) || (x==45) || (x==46) ? 1:0)
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DATA_R |= UART_TX_MASK;
    GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                  // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
   // UART0_IM_R |= UART_IM_TXIM;
   //NVIC_EN0_R |= 1<<(INT_UART0-16);
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART0_IBRD_R = (divisorTimes128 + 1) >> 7;          // set integer value to floor(r)
    UART0_FBRD_R = (((divisorTimes128 + 1)) >> 1) & 63; // set fractional value to round(fract(r)*64)
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full

    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}


 void getsUart0(char strbuffer [])
 {
     uint8_t count=0;
     uint8_t asc=0;
     while(true)
     {

         char c=getcUart0();
         yield();
         strbuffer[count]=c;
         asc=(uint8_t)c;

        //checking for backspace
         if(asc==8 || asc==127)
         {
                if(count>0)
                {
                    count=count-1;
                    continue;
                }
                else
                {
                    continue;
                 }
         }

         // checking for enter

        if(asc==13 || asc==10)
        {
            strbuffer[count]='\0';
            break;
        }

        if(asc>=32)
        {
            //strbuffer[count++]=c;    // changed temp
            count++;
        }
        else
        {
           continue;
        }


        if(count>=MAX_CHARS)
        {
            strbuffer[count]=0;
            putsUart0("Length exceeded \n");
        }
    }

}


uint8_t parse(const char strbuffer[], uint8_t index[], char type[])
{
    int i=0;
    int j=0;
    uint8_t asc=0;
    char types[90];
    types[0]='d';
    char *str_dummy=(char*)strbuffer;


    while(strbuffer[i]!='\0')

    {
        asc=(uint8_t)strbuffer[i];
        if(asc>=48 && asc<=57)
         {
             types[i+1] ='n';

         }
       else if((asc>=65 && asc<=90)||(asc>=97 && asc<=122)||(asc==38))
             {
                 types[i+1] ='a';
             }
       else{
           types[i+1]='d';

       }

           i++;
    }

    types[i+1]='\0';
    i=1;

    while(types[i]!='\0')

    {

        if(types[i-1]=='d' && types[i]!='d')
        {
            index[j]=i-1;
            type[j]=types[i];
            j++;
        }
        i++;
    }

    i=0;
    while(types[i]!='\0'){
        if(types[i]=='d' && i>0){
            str_dummy[i-1]='\0';
        }
        i++;
    }

    if (j>MAX_ARGS)
    {
        j=0;
    }
        return j;
}

bool iscommand(char strmatch[], uint8_t args, const char strbuffer[], uint8_t argcount, uint8_t index[], char type[] )

    {
//        if(args<argscount)
//        {
            int i=0;
            int ref=0;
            while(strbuffer[i]!='\0' && strmatch[i]!='\0')
            {
                if(strbuffer[i]!=strmatch[i])
                {
                    ref=1;
                    break;

                }
                i++;
            }

            if(ref==0 && strbuffer[i]=='\0' && strmatch[i]=='\0' && args<argcount )
                return true;
            else
                return false;

    }

int Atoi(const char *str)
{
    int i=0;
    int res = 0;
    for(i=0;str[i]!='\0';i++)
    {
        if(CHECK_NUM(str[i]))
        {
            res=((res*10)+(str[i]-48));

        }
    }
    return res;
}

int32_t getvalue(uint8_t args, const char strbuffer[], uint8_t argcount, uint8_t index[], char type[])
    {
        if(type[args+1]=='n')
        {
            return Atoi(&strbuffer[index[args+1]]);
        }
        else
        {
            return -1;
        }
    }

void itoa(char str[], int num)
{
    int rem, len = 0, n;
    n = num;
    if(n>0)
    {
    while (n != 0)
    {
     len++;
     n /= 10;
    }
    }
    else
    {
     len++;
    }
    int i;
    for ( i = 0; i < len; i++)
    {
     rem = num % 10;
     num = num / 10;
     str[len - (i + 1)] = rem + '0';
    }
    str[len] = '\0';
}

char* getstring(uint8_t args, const char strbuffer[], uint8_t argcount, uint8_t index[], char type[])
    {
        return (&strbuffer[index[args+1]]);
    }

int stringcmp(char mj1[], char mj2[])
{

    int i = 0, flag = 0;
    while(mj1[i] != '\0' && mj2[i] != '\0') // until atleast one string ends
    {

        if(mj1[i] != mj2[i])
        {
            flag = 1;
            break;
        }
        i++;
    }

    if(flag == 0 && mj1[i] == '\0' && mj2[i] == '\0')
        return 1;
    else
        return 0;
}

