//functions to communicate with MCP2515 CAN controller
#include "p33FJ16GS502.h"

char rd;

char exch_byte(char b) {
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = b;
    while (SPI1STATbits.SPIRBF != 1);
    return SPI1BUF;
}

void CAN_reset() {
    LATBbits.LATB11 = 0;
    exch_byte(0b11000000);   //reset the 2515 controller
    LATBbits.LATB11 = 1;
}

//load byte at the addr
void CAN_write(char addr, char b) {
    LATBbits.LATB11 = 0;
    exch_byte(0b01000010);   //write command
    exch_byte(addr);         //send the address
    exch_byte(b);         //send the byte
    LATBbits.LATB11 = 1;
}

//read 1 byte at the specified address
char CAN_read(char addr) {
    LATBbits.LATB11 = 0;
    exch_byte(0b00000011);   //read command
    exch_byte(addr);         //send at address
    rd =exch_byte(0);         //send 0 and clock out the result
    LATBbits.LATB11 = 1;
    return rd;
}

//load 1-st TX buffer with 1 byte of data
void load_TX_buffer(char b) {
    LATBbits.LATB11 = 0;
    exch_byte(0b01000001);   //write TX buf TXB0D0
    exch_byte(b);
    LATBbits.LATB11 = 1;
}
//request buffer number to be send [321] encoded in the last 3 bits of the command
void request_to_send(char n_buf) {
    LATBbits.LATB11 = 0;
    exch_byte((0b111 & n_buf) | (0b10000000));   //send RTS
    LATBbits.LATB11 = 1;
}

char read_status(void) {
    LATBbits.LATB11 = 0;
    exch_byte(0b00000011);   //read status command
    rd =exch_byte(0);         //send 0 and clock out the result
    LATBbits.LATB11 = 1;
    return rd;
}