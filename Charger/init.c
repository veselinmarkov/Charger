
#include "p33FJ16GS502.h"
#include "Functions.h"
#include "dsp.h"
#include "CAN_transmit.h"

/* Variable Declaration required for each PID controller in the application. */
tPID Buck1VoltagePID; 

/* These data structures contain a pointer to derived coefficients in X-space and 
   pointer to controler state (history) samples in Y-space. So declare variables 
   for the derived coefficients and the controller history samples */
fractional Buck1VoltageABC[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional Buck1VoltageHistory[3] __attribute__ ((section (".ybss, bss, ymemory")));

#define PID_BUCK1_KP 0.23
#define PID_BUCK1_KI 0.0016
#define PID_BUCK1_KD 0

/* These macros are used for faster calculation in the PID routine and are used for limiting the PID Coefficients */
#define PID_BUCK1_A Q15(PID_BUCK1_KP + PID_BUCK1_KI + PID_BUCK1_KD) //A = 0.2316
#define PID_BUCK1_B Q15(-1 *(PID_BUCK1_KP + 2 * PID_BUCK1_KD))      //B = -0.23
#define PID_BUCK1_C Q15(PID_BUCK1_KD)                               //C = 0

#define TRIG_OFFSET             140      // small offset for the ADC trigger

/* This is increment rate to give us the sofstart */
 #define BUCK1_SOFTSTART_INCREMENT	 40

int zero_transducer, average, vout_average;       //zero value; running average of the measured current
extern unsigned int TimerInterruptCount;
char CAN_stat_result;

void SPI_setup(void)
{
    //__builtin_write_OSCCONL(0x46);
    //__builtin_write_OSCCONH(0x57);
    //OSCCONbits.IOLOCK =0; // enable changing RPIN and RPOR registers
 
    //__builtin_write_OSCCONL(0x46);
    //__builtin_write_OSCCONH(0x57);
    //OSCCONbits.IOLOCK =1; // prevent changing RPIN and RPOR registers

    //Prescalers: in case Fck =40MHz, the baud rate is 104.17KHz
    SPI1CON1bits.PPRE =0b00;    //64:1
    SPI1CON1bits.SPRE =0b010;   //6:1
    
    //setup the SPI1
    IFS0bits.SPI1IF = 0; //Clear the Interrupt Flag
    IEC0bits.SPI1IE = 0; //disable the Interrupt
    SPI1CON1bits.DISSCK = 0; //Internal Serial Clock is Enabled.
    SPI1CON1bits.DISSDO = 0; //SDOx pin is controlled by the module.
    SPI1CON1bits.MODE16 = 0; //Communication is word-wide (16 bits).
    SPI1CON1bits.SMP = 0; //Input Data is sampled at the middle of data output time.
    SPI1CON1bits.CKE = 1; //Serial output data changes on transition from
                           //Idle clock state to active clock state
    SPI1CON1bits.CKP = 0; //Idle state for clock is a low level;
                        //active state is a high level
    SPI1CON1bits.MSTEN = 1; //Master Mode Enabled
    SPI1STATbits.SPIEN = 1; //Enable SPI Module

    Delay_ms(1);
    //issue reset command
    CAN_reset();
    Delay_ms(1);

    CAN_write(CANCTRL, 0b00001110); // clock/4; clock out pin enabled; one-shot mode; set normal operation mode
    CAN_stat_result =CAN_read(CANSTAT);
}

void UART_setup(void) {
    #define FCY             39613000        //Instruction Cycle Frequency
    #define BAUDRATE         9600
    #define BRGVAL          ((FCY/BAUDRATE)/16)-1 // calculate UART prescaler value
    U1BRG  = BRGVAL;
    U1MODEbits.UARTEN = 1; /* Reset UART to 8-n-1, alt pins, and enable */
    U1STAbits.UTXEN  = 1; /* Reset status register and enable TX & RX*/
//    U1MODEbits.ABAUD =1;
}

void Buck1Drive(void)
{
    #define PERIOD 4800 // for 100KHz PWM period
    PTCON2bits.PCLKDIV =1;                  //master PWM clock divider; changes the PWM freq. from 200kHz to 100kHz period
    PTPER = PERIOD;

//  PWM#1
    IOCON1bits.PENH = 1;                  /* PWM1H is controlled by PWM module */
    IOCON1bits.PENL = 1;                  /* PWM1L is controlled by PWM module */

    IOCON1bits.PMOD = 0;                  /* Complementary Mode */
    
    IOCON1bits.POLH = 0;                  /* Drive signals are active-high */
    IOCON1bits.POLL = 0;                  /* Drive signals are active-high */

    IOCON1bits.OVRENH = 0;				  /* Disable Override feature for shutdown PWM */
    IOCON1bits.OVRENL = 0;				  /* Disable Override feature for shutdown PWM */
    IOCON1bits.OVRDAT = 0b00;			  /* Shut down PWM with Over ride 0 on PWMH and PWML */

    PWMCON1bits.DTC = 0;                  /* Positive Deadtime enabled */
    
    DTR1    = 80;                        // DT is about 100ns
    ALTDTR1 = 80;                        // alt DT is about 150ns

    PWMCON1bits.IUE = 0;                  /* Disable Immediate duty cycle updates */
    PWMCON1bits.ITB = 0;                  /* Select Primary Timebase mode */
    PWMCON1bits.CLIEN = 1;                // Enable Current limit interrupt
            
    TRGCON1bits.TRGDIV = 0b1111;               /* Trigger interrupt generated every 16-th PWM cycle */
    TRGCON1bits.TRGSTRT = 0;              /* Trigger generated after waiting 0 PWM cycles */

    PDC1 = PERIOD /2;                            /* 50% Initial pulse-width = minimum deadtime required (DTR1 + ALDTR1)*/
    TRIG1 = PERIOD -TRIG_OFFSET;                            /* Trigger generated at beginning of PWM active period */

    FCLCON1bits.CLSRC = 0;                // Current source 0
    FCLCON1bits.CLPOL = 1;                // Invert current source = active low
    FCLCON1bits.CLMOD = 1;                // Enable current limiting
    FCLCON1bits.FLTMOD = 3;               /* Fault Disabled */

//  PWM#2
    IOCON2bits.PENH = 1;                  /* PWM1H is controlled by PWM module */
    IOCON2bits.PENL = 1;                  /* PWM1L is controlled by PWM module */

    IOCON2bits.PMOD = 0;                  /* Complementary Mode */

    IOCON2bits.POLH = 0;                  /* Drive signals are active-high */
    IOCON2bits.POLL = 0;                  /* Drive signals are active-high */

    IOCON2bits.OVRENH = 0;				  /* Disable Override feature for shutdown PWM */
    IOCON2bits.OVRENL = 0;				  /* Disable Override feature for shutdown PWM */
    IOCON2bits.OVRDAT = 0b00;			  /* Shut down PWM with Over ride 0 on PWMH and PWML */

    PWMCON2bits.DTC = 0;                  /* Positive Deadtime enabled */

    DTR2    = 80;                        // DT is about 100ns

    ALTDTR2 = 80;                       // alt DT is about 150ns

    PWMCON2bits.IUE = 0;                  /* Disable Immediate duty cycle updates */
    PWMCON2bits.ITB = 0;                  /* Select Primary Timebase mode */

    PDC2 =PERIOD /2;
    PHASE2 = 1200;
    
    FCLCON2bits.CLSRC = 0;                // Current source 0
    FCLCON2bits.CLPOL = 1;                // Invert current source = active low
    FCLCON2bits.CLMOD = 1;                // Enable current limiting
    FCLCON2bits.FLTMOD = 3;               /* Fault Disabled */

    IEC5bits.PWM1IE =1;                   //Enable PWM1 interrupt (capture Current limit)
}

void updateTrigPeriodPercentage(unsigned int perc) {
    TRIG1 = (double)perc/100*PERIOD;
}
void CurrentandVoltageMeasurements(void)
{
    ADCONbits.ADON = 0;                   //Stop the ADC
    ADCONbits.FORM = 0;                   /* Integer data format */
    ADCONbits.EIE = 0;                    /* Early Interrupt disabled */
    ADCONbits.ORDER = 0;                  /* Convert even channel first */
    ADCONbits.SEQSAMP = 0;                /* Select simultaneous sampling */
    ADCONbits.ADCS = 5;                   /* ADC clock = FADC/6 = 120MHz / 6 = 20MHz, 
											12*Tad = 1.6 MSPS, two SARs = 3.2 MSPS */
	//ADC Pair0 PWM interrupt config
    IFS6bits.ADCP0IF = 0;		    	  /* Clear ADC Pair 0 interrupt flag */ 
    IPC27bits.ADCP0IP = 5;			      /* Set ADC interrupt priority */ 
    /*IEC6bits.ADCP0IE = 1;				  Enable the ADC Pair 0 interrupt. 
                                          Disable now, to use it later for enable/disable the measurements */

    ADPCFGbits.PCFG0 = 0; 				  /* Current Measurement for Buck 1 */ 
    ADPCFGbits.PCFG1 = 0; 				  /* Voltage Measurement for Buck 1 */
    
    ADCPC0bits.IRQEN0 = 1; 				  /* Enable ADC Interrupt, pair 0*/ 
    ADCPC0bits.TRGSRC0 = 4; 			  /* ADC Pair 0 triggered by PWM1 */

    //ADC Pair 1 In voltage manually start
    ADPCFGbits.PCFG2 = 0; 				  /* Input voltage Measurement for Buck 1 at AN2*/
    ADSTATbits.P1RDY = 0; 				  /* Clear Pair 1 data ready bit */
    ADCPC0bits.IRQEN1 = 0;                /* Disable ADC pair 1 Interrupt after input voltage measurement */
    ADCPC0bits.TRGSRC1 = 1; 			  /* ADC Pair 1 triggered by individual soft trigger */
}

void Buck1VoltageLoop(void)
{
    Buck1VoltagePID.abcCoefficients = Buck1VoltageABC;      /* Set up pointer to derived coefficients */
    Buck1VoltagePID.controlHistory = Buck1VoltageHistory;   /* Set up pointer to controller history samples */
    
    PIDInit(&Buck1VoltagePID); 
                             
	if ((PID_BUCK1_A == 0x7FFF || PID_BUCK1_A == 0x8000) || 
		(PID_BUCK1_B == 0x7FFF || PID_BUCK1_B == 0x8000) ||
		(PID_BUCK1_C == 0x7FFF || PID_BUCK1_C == 0x8000))
	{
		while(1);											/* This is a check for PID Coefficients being saturated */ 
	}																	

	Buck1VoltagePID.abcCoefficients[0] = PID_BUCK1_A;		/* Load calculated coefficients */	
	Buck1VoltagePID.abcCoefficients[1] = PID_BUCK1_B;
	Buck1VoltagePID.abcCoefficients[2] = PID_BUCK1_C;
   
	Buck1VoltagePID.controlReference = 0;   //Start with 0 current

	Buck1VoltagePID.measuredOutput = 0;       				/* Measured Output is 0 volts before softstart */       	 

}

void UpdateControlReference(unsigned int POT) {
    Buck1VoltagePID.controlReference = POT;
}

void UpdateDTR(unsigned int POT) {
    DTR1    = POT;
    ALTDTR1 = POT;

    DTR2    = POT;
    ALTDTR2 = POT;
}

void Buck1SoftStartRoutine(unsigned int POT)
{
  /* This routine increments the control reference until the reference reaches 
     the desired output voltage reference. In this case the we have a softstart of 50ms */
        Buck1VoltagePID.controlReference = 0;   //Start with 0 current

	while (Buck1VoltagePID.controlReference <= POT)
	{
		Delay_ms(1);
		Buck1VoltagePID.controlReference += BUCK1_SOFTSTART_INCREMENT;
	}

	Buck1VoltagePID.controlReference = POT;
}


void Delay_ms (unsigned int delay)
{
	TimerInterruptCount = 0;			 /* Clear Interrupt counter flag */

	PR1 = 0x9C40;						 /* (1ms / 25ns) = 40,000 = 0x9C40 */ 
	IPC0bits.T1IP = 4;					 /* Set Interrupt Priority lower then ADC */
	IEC0bits.T1IE = 1;				   	 /* Enable Timer1 interrupts */

	T1CONbits.TON = 1;					 /* Enable Timer1 */

	while (TimerInterruptCount < delay); /* Wait for Interrupt counts to equal delay */

	T1CONbits.TON = 0;					 /* Disable the Timer */
}
