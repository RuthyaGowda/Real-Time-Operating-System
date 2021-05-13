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

#ifndef UART0_H_
#define UART0_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart0();
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
bool kbhitUart0();
void getsUart0(char strbuffer []);
uint8_t parse(const char strbuffer[], uint8_t index[], char type[]);
bool iscommand(char strmatch[], uint8_t args, const char strbuffer[], uint8_t argscount, uint8_t index[], char type[] );
int32_t getvalue(uint8_t args, const char strbuffer[], uint8_t argscount, uint8_t index[], char type[]);
char* getstring(uint8_t args, const char strbuffer[], uint8_t argcount, uint8_t index[], char type[]);
//void displayUart0(char str[]);
void yield(void);
int intoAs(int numb);
void itoa(char str[], int num);
int Atoi(const char* str);
int stringcmp(char mj1[], char mj2[]);




#endif
