/**********************************************************************
* © 2008 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
*******************************************************************************/

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