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
#include "Functions.h"
// #include "dsp.h"
#include "CAN_transmit.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON)
_FWDT(FWDTEN_OFF)
_FPOR(FPWRT_PWR128)
_FICD(ICS_PGD2 & JTAGEN_OFF)

//#define INPUTUNDERVOLTAGE 391					/* Input voltage <7V --> 2.2k/(10k+2.2k)*7V = 1.2623V
//												Now calculate the ADC expected value = 1.2623/3.3*1023 = 391 */
//#define INPUTOVERVOLTAGE 839						/* Input voltage >15V --> 2.2k/ (10k+2.2k)*15 = 2.70492V
//												Now calculate the ADC expected value  = 2.70492/3.3*1023 = 839 */

#define OutputChargeFinish 784                                  /* Out >4.2V --> 5k/(5k +3.3k)*4.2 = 2.53V
                                                                    ADC value = 2.53/3.3*1023 = 784*/

#define OutputBatteryPresent 280                                  /* Out >1.5 --> 0.9V
                                                                    ADC value = 1.5/3.3*1023 = 280 */
// extern tPID Buck1VoltagePID;
extern int zero_transducer, average, vout_average;
double AlphaOutVolt =0.0278, BetaOutVolt = 0.0;   //Alpha and Beta coefficients in the equation OutVolt =Alpha*steps +Beta for the out voltage
int PotVoltage, CurrentTarget, OutVolt1, OutVolt1steps, InVoltage, n;
char UART_logger_interval = -1;        //Return pressed via UART; if -1 the logger is switched off
extern char UART_CR;
extern char UART_buff[16];
char UART_logger_buff[150];
int PID_enable =0;
char CAN_result;

double StepToFloat(int current) {
    //return (double)0.01052*current; //95 steps per amper
    return (double)1.0 * current;
}

//current state Vout measure: R7+R8 = 3.29kOm; R9 = 33 Om; gain =1/100; max input = 30V
//              In volt=5mV; in ACPL-7900 = 1mV; out ACPL = 7mV; ADC AN1 = 50mV (noise); steps=14  
//              In volt=4.09V; in ACPL-7900 = 42mV; out ACPL = 104mV; ADC AN1 = 490mV (amp=4.54); steps =147; alpha = 0.0278
double returnOutVolt(int voltage) {
    return (double)AlphaOutVolt*voltage +BetaOutVolt;
}

void measureIdleCurrent(void) {  // PWM must be switched off before invoke
    ADCONbits.ADON = 0;
    Delay_ms(100);                                        //wait some time to measure zero current offset
    ADSTATbits.P0RDY = 0; 				  /* Clear Pair 0 data ready bit */
    ADCPC0bits.TRGSRC0 = 1;                               /* ADC Pair 0 triggered by soft trigger */
    ADCONbits.ADON = 1;                                   //enable temporary the ADC
    ADCPC0bits.SWTRG0 =1;                                 // new sample for pair 0
    while (ADSTATbits.P0RDY == 0);                         // wait to finish the zero measurement
    zero_transducer = ADCBUF0;
    ADCPC0bits.TRGSRC0 = 4; 			  /* ADC Pair 0 triggered by PWM1 */
    ADCONbits.ADON = 1;
}

int measureInputVolt(void) {
    ADSTATbits.P1RDY = 0;						// Clear the ADC pair 1 ready bit
    ADCPC0bits.SWTRG1 =1;                       // new sample for pair 1
    while (ADSTATbits.P1RDY == 0);                         // wait to finish the zero measurement
    return ADCBUF2;
 }

// Pins definitions
//Current Buck1 - AN0/RA0 (pin2) = FB0 
//Volt Buck1 out - AN1/RA1 (pin3) = FB1
//Volt Buck1 in - AN2 (pin4) = FB4
//Over current/over temp signal in - AN3/RP0/RB0 (pin5)
//Over current/over temp status active low out - AN5/RB10 (pin7)
//Charging status active low out- AN7/RB2 (pin10)
//Charging current 0-100% external drive PWM signal in - AN6/RB1 (pin9)  
//start/stop switch in - RB8/RP8 (pin14)
//SPI signals: CS - RP11 (pin21); SCK - RP12 (pin22); SI - RP5 (pin16); SO - RP15 (pin15)
//UART new: TX - RP7 (pin18) RX - RP6 (pin17)
int main(void)
{
	
	/* Configure Oscillator to operate the device at 40Mhz
	   Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
 	   Fosc= 7.37*(43)/(2*2)=80Mhz for Fosc, Fcy = 40Mhz */

	/* Configure PLL prescaler, PLL postscaler, PLL divisor */
	PLLFBD = 41; 							/* M = PLLFBD + 2 */
	CLKDIVbits.PLLPOST = 0;   				/* N1 = 2 */
	CLKDIVbits.PLLPRE = 0;    				/* N2 = 2 */

    //remappable inputs
    RPINR29bits.FLT1R =0;                   // RP0/RB0 is PWM fault/current source
    TRISBbits.TRISB0 =1;                    //config RB0(RP0) as an input
    ADPCFGbits.PCFG3 =1;                    //config RP0 = AN3 AS DIGITAL/NOT ANALOG
    RPINR20bits.SDI1R =5;                   // Pin RP5 SPI:SI
    TRISBbits.TRISB5 =1;                    //config RB5(RP5) as an input
    RPINR18bits.U1RXR = 6;			// Make Pin RP6 U1RX
    TRISBbits.TRISB6 =1;                    //config RB6(RP6) as an input
//        RPINR18bits.U1RXR = 0;			// Make Pin RP0 U1RX
//        TRISBbits.TRISB0 =1;                    //config RB0(RP0) as an input
    //RPINR20bits.SCK1R =12;
    //RPINR21bits.SS1R =11;
    
    //remappable outputs
    RPOR7bits.RP15R = 0b000111;  //SDO
    RPOR6bits.RP12R = 0b001000; //SCK
    RPOR5bits.RP11R = 0b001001; //CS
	RPOR3bits.RP7R = 3;		// Make Pin RB7 (RP7) U1TX
//	RPOR4bits.RP9R = 3;		// Make Pin RB9 (RP9) U1TX
    TRISBbits.TRISB15 =0;
    TRISBbits.TRISB12 =0;
    TRISBbits.TRISB11 =0;
    TRISBbits.TRISB10 =0;           //Digital out fault indication signal  AN5/RB10 (pin7)
    ADPCFGbits.PCFG5 =1;            //config  AN5/RB10 (pin7) AS DIGITAL/NOT ANALOG
    LATBbits.LATB10 =1;             //switch if off
    TRISBbits.TRISB2 =0;           //Digital out signal charging indication AN7/RB2 (pin10)
    ADPCFGbits.PCFG7 =1;            //config AN7/RB2 (pin10) AS DIGITAL/NOT ANALOG
    LATBbits.LATB2 =1;             //switch if off             

    __builtin_write_OSCCONH(0x01);			/* New Oscillator selection FRC w/ PLL */
    __builtin_write_OSCCONL(0x01);  		/* Enable Switch */
      
	while(OSCCONbits.COSC != 0b001);		/* Wait for Oscillator to switch to FRC w/ PLL */  
        while(OSCCONbits.LOCK != 1);			/* Wait for Pll to Lock */

	/* Now setup the ADC and PWM clock for 120MHz
	   ((FRC * 16) / APSTSCLR ) = (7.37MHz * 16) / 1 = 117.9MHz*/
	
	ACLKCONbits.FRCSEL = 1;					/* FRC provides input for Auxiliary PLL (x16) */
	ACLKCONbits.SELACLK = 1;				/* Auxiliary Ocillator provides clock source for PWM & ADC */
	ACLKCONbits.APSTSCLR = 7;				/* Divide Auxiliary clock by 1 */
	ACLKCONbits.ENAPLL = 1;					/* Enable Auxiliary PLL */
	
	while(ACLKCONbits.APLLCK != 1);			/* Wait for Auxiliary PLL to Lock */

    Buck1Drive();							/* PWM Setup */
        /* Complementary mode, 400kHz, dead time 25ns, ALT dt 50ns, primary timebase, disable immediate updates, trigger every cycle, fault disabled */
    CurrentandVoltageMeasurements();		/* ADC Setup */
    measureIdleCurrent();                   // Measure the Idle current and store in zero_transducer
    Buck1VoltageLoop();    					/* Initialize Buck1 PID */

    SPI_setup();
    UART_setup();
    
    IEC0bits.U1RXIE =1;                                         // Enable UART RX interrupt
    IFS0bits.U1RXIF = 0;                                        // Clear UART RX interrupt
    ADCONbits.ADON = 1;						/* Enable the ADC */
    PTCONbits.PTEN = 1;						/* Enable the PWM */
    //ADCPC1bits.SWTRG2 =1;                                               // new sample for pair 2; POT Voltage

     /* Setup RB12 to enable Buck 1 Output load */
//    TRISBbits.TRISB12 = 0;                  /* Set RB12 as digital output */
//    LATBbits.LATB12 = 0;                    /* If set this turns ON output load connected to Buck1 */

    //Stop the PWM
    IOCON1bits.OVRENH = 1;
    IOCON1bits.OVRENL = 1;
    IOCON2bits.OVRENH = 1;
    IOCON2bits.OVRENL = 1;

    CurrentTarget =40;              // 1Amp current target
    while(1)
    {
                    if (UART_CR ==1) {                                                       // process UART commands
                        UART_CR =0;
                        if (strncmp("ATLOGON", UART_buff, 7) ==0) {                          //start the logger
                            UART_string_out("OK");                                           
                            UART_logger_interval =0;
                        }
                        else if (strncmp("ATLOGOFF", UART_buff, 8) ==0) {                    //stop the logger                                                  // process UART commands
                            UART_string_out("OK");
                            UART_logger_interval =-1;
                        }
                        else if (strncmp("ATON", UART_buff, 4) ==0) { 
                            n =atoi(UART_buff +4);
                            if (strlen(UART_buff) ==4)
                                n = 2;
                            if (n >0) {
                                IEC6bits.ADCP0IE = 1;
                                average =0;
                                vout_average =0;                       //start the charging process
                                UART_string_out("Measurements active");
                            }
                            if (n >1) {                                            /* Enable the ADC Pair 0 interrupt current measurement*/
                                IOCON1bits.OVRENH = 0;                                           //Start the PWM
                                IOCON1bits.OVRENL = 0;
                                IOCON2bits.OVRENH = 0;
                                IOCON2bits.OVRENL = 0;                       //start the charging process
                                UART_string_out("PWM output active");
                            }
                            if (n >2) {
                                PID_enable = 1;
//                              Buck1SoftStartRoutine(5*PotVoltage);                           /* Initiate Buck 1 soft start to 5V */
                                Buck1SoftStartRoutine(CurrentTarget *32);                       //start the charging process
                                UART_string_out("PID activated");
                            }
                        }
                        else if (strncmp("ATOFF", UART_buff, 5) ==0) {                       //stop the charging                                                  // process UART commands
                            UART_string_out("OK");
                            IEC6bits.ADCP0IE = 0;                                            /* Disable the ADC Pair 0 interrupt current measurement*/
                            IOCON1bits.OVRENH = 1;                                           //Stop the PWM
                            IOCON1bits.OVRENL = 1;
                            IOCON2bits.OVRENH = 1;
                            IOCON2bits.OVRENL = 1;
                            average =0;
                            vout_average =0;
                            UpdateControlReference(0);
                        }
                        else if (strncmp("ATCURR", UART_buff, 6) ==0) {                          //set or query current target value
                            if (strlen(UART_buff) ==6) {
                                sprintf(UART_logger_buff, "Current target: %d", CurrentTarget);
                                UART_string_out(UART_logger_buff);
                            } else {
                                n =atoi(UART_buff +6);
                                if (n >0) {
                                    CurrentTarget =n;
                                    UpdateControlReference(n *32);
                                    sprintf(UART_logger_buff, "New current target set: %d", n);
                                    UART_string_out(UART_logger_buff);
                                } else {
                                    sprintf(UART_logger_buff, "Not correct value: %s", UART_buff +6);
                                    UART_string_out(UART_logger_buff);
                                }
                            }
                        }
                        else if (strncmp("ATDTR", UART_buff, 5) ==0) {                          //set or query DTR
                            if (strlen(UART_buff) ==5) {
                                sprintf(UART_logger_buff, "DTR: %d", DTR1);
                                UART_string_out(UART_logger_buff);
                            } else {
                                n =atoi(UART_buff +5);
                                if (n >0) {
                                    UpdateDTR(n);
                                    sprintf(UART_logger_buff, "New DTR set: %d", n);
                                    UART_string_out(UART_logger_buff);
                                } else {
                                    sprintf(UART_logger_buff, "Not correct value: %s", UART_buff +5);
                                    UART_string_out(UART_logger_buff);
                                }
                            }
                        }
                        else if (strncmp("ATTRG", UART_buff, 5) ==0) {                          //set or query ADC Trigger
                            if (strlen(UART_buff) ==5) {
                                sprintf(UART_logger_buff, "Trigger: %d", TRIG1);
                                UART_string_out(UART_logger_buff);
                            } else {
                                n =atoi(UART_buff +5);
                                if (n >0) {
                                    updateTrigPeriodPercentage(n);
                                    sprintf(UART_logger_buff, "New Trigger percent set: %d", n);
                                    UART_string_out(UART_logger_buff);
                                } else {
                                    sprintf(UART_logger_buff, "Not correct value: %s", UART_buff +5);
                                    UART_string_out(UART_logger_buff);
                                }
                            }
                        }
                        else if (strncmp("ATPWM", UART_buff, 5) ==0) {                          //set or query ADC Trigger
                            if (strlen(UART_buff) ==5) {
                                sprintf(UART_logger_buff, "PWM value %d", PHASE2);
                                UART_string_out(UART_logger_buff);
                                sprintf(UART_logger_buff, "PID state: %d", PID_enable);
                                UART_string_out(UART_logger_buff);
                            } else {
                                n =atoi(UART_buff +5);
                                if (n >0) {
                                    PID_enable = 0;
                                    PHASE2 = n;
                                    sprintf(UART_logger_buff, "PID disabled. New PHASE2 value set: %d", n);
                                    UART_string_out(UART_logger_buff);
                                } else {
                                    sprintf(UART_logger_buff, "Not correct value: %s", UART_buff +5);
                                    UART_string_out(UART_logger_buff);
                                }
                            }
                        }
                        else if (strncmp("ATCALIB", UART_buff, 7) ==0) {                       //calibrate zero current
                            if (IOCON1bits.OVRENH ==0) {
                                UART_string_out("Error. Switch the charging off first");
                                continue;
                            }
                            measureIdleCurrent();
                            sprintf(UART_logger_buff, "Zero current calibrated: %d\n", zero_transducer);
                            UART_string_out(UART_logger_buff);
                        }
                        else if (strncmp("ATVO1CAL", UART_buff, 8) ==0) {                       //calibrate output voltage step1
                            n =atoi(UART_buff +8);
                            if (n >0) {
                                OutVolt1 =n;
                                OutVolt1steps =ADCBUF1;
                                if (OutVolt1steps ==0) {
                                    sprintf(UART_logger_buff, "No out voltage detected. Please connect a battery");
                                    UART_string_out(UART_logger_buff);
                                    continue;
                                }
                                AlphaOutVolt =(double)OutVolt1 /OutVolt1steps;
                                BetaOutVolt =0.0;
                                sprintf(UART_logger_buff, "Please use ATVO2CAL to improve the precision");
                                UART_string_out(UART_logger_buff);
                            } else {
                                sprintf(UART_logger_buff, "Not correct value for voltage (ATVO1CAL[1 - 999]): %s", UART_buff +8);
                                UART_string_out(UART_logger_buff);
                            }
                        }
                        else if (strncmp("ATVO2CAL", UART_buff, 8) ==0) {                       //calibrate output voltage step2
                            if (OutVolt1 ==0 || OutVolt1steps ==0) {
                                sprintf(UART_logger_buff, "Please issue ATVO1CAL first");
                                UART_string_out(UART_logger_buff);
                                continue;
                            }
                            n =atoi(UART_buff +8);
                            if (n >0) {
                                AlphaOutVolt =(double)(n -OutVolt1) /(ADCBUF1 -OutVolt1steps);
                                BetaOutVolt =OutVolt1 -AlphaOutVolt*OutVolt1steps;
                                sprintf(UART_logger_buff, "Second step completed for better volt precision");
                                UART_string_out(UART_logger_buff);
                            } else {
                                sprintf(UART_logger_buff, "Not correct value for voltage (ATVO2CAL[1 - 999): %s", UART_buff +8);
                                UART_string_out(UART_logger_buff);
                            }
                        }
                    }

                    if (UART_logger_interval >=0) {
                        UART_logger_interval++;
                        if (UART_logger_interval >10) {                                   //output current +voltages +temp
                            UART_logger_interval =0;
            //                UART_logger_buff[0] =0;
                            InVoltage = measureInputVolt();
                            sprintf(UART_logger_buff, "Out current: %.2fA; Current target: %.2fA; Zero calibrate: %d; Out voltage: %.2fV; In voltage: %d steps\n", StepToFloat(average/32), StepToFloat(CurrentTarget), zero_transducer, returnOutVolt(vout_average/32), InVoltage);
                            UART_string_out(UART_logger_buff);
                        }
                    }

                    if (U1STAbits.OERR ==1) {
                        U1STAbits.OERR =0;
                    }
/*
                    if (ADSTATbits.P2RDY ==1)
                    {
                            PotVoltage = ADCBUF5;						// Read POT Voltage
                            if (IOCON1bits.OVRENH ==0)                                          //if only the PWM is active
                                UpdateDTR(PotVoltage);
//                                UpdateControlReference(5*PotVoltage);
                            ADSTATbits.P2RDY = 0;						// Clear the ADC pair ready bit
                            ADCPC1bits.SWTRG2 =1;                                               // new sample for pair 2

                    }
*/
                    if (!(PORTBbits.RB8) )  //&& (OutVoltage >= OutputBatteryPresent) && (OutVoltage <= OutputChargeFinish)) /* if input voltage is less than
                                          //                                 ChargeFinish or greater than BatteryPresent then activate the PWM output */
                    {
                            PID_enable =1; 
                            IOCON1bits.OVRENH = 0;			//Start the PWM
                            IOCON1bits.OVRENL = 0;
                            IOCON2bits.OVRENH = 0;
                            IOCON2bits.OVRENL = 0;
                            IEC6bits.ADCP0IE = 1;
                            average =0;
                            vout_average =0;
//                            Buck1SoftStartRoutine(5*PotVoltage);    			/* Initiate Buck 1 soft start to 5V */
                            Buck1SoftStartRoutine(CurrentTarget *32);
                            CAN_result =CAN_read(CANCTRL);
                    }
                    

                    //for the mcp2515 to transmit data must load al least TXBnDLC-data lenght code, TXB0SIDH-standard dientifier high, TXB0SIDL-SID low

                    /*
                    if ((OutVoltage < OutputBatteryPresent) || (OutVoltage > OutputChargeFinish)) // if input voltage is less than
                                                                           BatteryPresent or greater than ChargeFinish then deactivate the PWM output 
                    {
                            IOCON1bits.OVRENH = 1;			//Stop the PWM
                            IOCON1bits.OVRENL = 1;
                    }
                     */
//        LATBbits.LATB12 =~PORTBbits.RB8;
                    Delay_ms(200);          //delay the main loop at 0.5 s. intervals
                    LATBbits.LATB10 = PORTBbits.RB0;    //indicate overcurrent/temperature fault LED
                    PID_enable = PORTBbits.RB0; //switch on/off PID when overcurrent/temperature fault
                    //LATBbits.LATB10 =1;     //Reset the overcurrent/temp fail out
                    LATBbits.LATB2 = IOCON1bits.OVRENH;     //indicate the charging status LED
    }

}



