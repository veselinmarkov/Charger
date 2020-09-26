
#include "p33FJ16GS502.h"

unsigned int TimerInterruptCount = 0;
char UART_char, UART_CR =0;          //UART_CR CR is pressed and the command must be processed in the main loop

char UART_buff[16];         //UART buffer
int i, UART_pointer =0;       //buffer pointer
extern int PID_enable;

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt()
{
  TimerInterruptCount ++; 	/* Increment interrupt counter */
  IFS0bits.T1IF = 0; 		/* Clear Interrupt Flag */
}

void __attribute__((__interrupt__, no_auto_psv)) _PWM1Interrupt()
{
    LATBbits.LATB10 =0;         //Activate the Over current/Temp out flag
    IFS5bits.PWM1IF = 0;        //Clear Interrupt flag
    PID_enable =0;
 }

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    UART_char =U1RXREG;
    if (UART_char =='\r') {
        UART_buff[UART_pointer] =0;
        UART_pointer =0;
        UART_CR =1;
        while(!U1STAbits.TRMT);
        U1TXREG = '\n';
    } else {
        if (UART_pointer <15)
            UART_buff[UART_pointer++] =UART_char;
    }
    while(!U1STAbits.TRMT);
    U1TXREG = UART_char;
    IFS0bits.U1RXIF = 0;
}

void UART_string_out(char buff[]) {              //zero terminated string
    while(!U1STAbits.TRMT);
        U1TXREG = '\r';
    while(!U1STAbits.TRMT);
        U1TXREG = '\n';
    for (i =0; buff[i] !=0 && i < 150; i++) {
        while(!U1STAbits.TRMT);
        U1TXREG = buff[i];
    }
    while(!U1STAbits.TRMT);
        U1TXREG = '\r';
    while(!U1STAbits.TRMT);
        U1TXREG = '\n';
}