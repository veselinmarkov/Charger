/* 
 * File:   CAN_transmit.h
 * Author: Markov
 *
 * Created on May 3, 2016, 3:48 PM
 */
//usefull MCP2515 registers
#define CANCTRL 0x1F    //send byte= 0b00001111; clock/8; clock out pin enabled; one-shot mode; set normal operation mode
#define CANSTAT 0x1E    //can read the actual operation mode
#define TXB1SIDH 0x31   //trnsmit buffer 1 standard 11bit identifier high (3-10bits) could send 0x00
#define TXB1SIDL 0x32   //trnsmit buffer 1 standard 11bit identifier low (0-2bits) could send 0b11100000 for 0-2bits=111
#define TXB1DLC 0x35    //data length code 0-3bits least significant bits
#define TXB0D0 0x36     //first byte of data to be transmitted


extern void CAN_reset();
extern void CAN_write(char addr, char b);
extern char CAN_read(char addr);
extern void load_TX_buffer(char b);
extern void request_to_send(char n_buf);
extern char read_status(void);


