/*
 * File:   newmainXC16.c
 * Author: samue
 *
 * Created on May 13, 2017, 3:02 PM
 */

#include "xc.h"

//Configuration Register - FOSCSEL
#pragma config  IESO    = 0     //Disable any internal to external osc switch config
#pragma config  FNOSC   = 1     //Select FRC osc with PLL at reset

//Configuration Register - FOSC
#pragma config  FCKSM       = 3 //Osc switch fail safe modes disabled
#pragma config  OSCIOFNC    = 0 //OSCO pin is now a general purpose IO pin
#pragma config  POSCMD      = 3 //Disable primary osc (HS, XT, EC)

//Configure In-Circuit Programmer, WDT
#pragma config  FWDTEN      = 0 //Watchdog Timer is disabled
#pragma config  ICS         = 3 //Program through PGEC1/PGED1 ports

#define PPSUnLock   __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock     __builtin_write_OSCCONL(OSCCON | 0x40)

volatile unsigned int z;
volatile unsigned int y;
    
//How long to read from mic 5000ms = 5s
#define SAMPLE_TIME		5000

unsigned int counter = 0;
unsigned int gWord[SAMPLE_TIME];
	
unsigned int isRecording = 1;
unsigned int isPlaybackStart = 1;
unsigned int isPlayback = 1;
unsigned int replayCount = 0;


void delay(int limit1, int limit2){
    unsigned int i, j;
    for(i = 0; i < limit1; i++){
        for(j = 0; j < limit2; j++){       
        }
    }
}

/*
 *	ISR For Right Channel DAC
 *		TODO: When does this interrupt activate?
 *		W
 */
void __attribute__((interrupt, no_auto_psv))_DAC1RInterrupt(void){
	IFS4bits.DAC1RIF = 0;		//Clear Right Channel Interrupt Flag
	/*
	//Ping pong between the two statements as the FIFO is NOT FULL
	if(z == 0){
		z = 1;
		//0xFFFF	2.34V
		//0x8FFF	1.79V
		//0x8000	1.73V
		//0x1111	1.18V
		//0x0000	1.09V
		//DAC1RDAT = 0xFFFF;
		DAC1RDAT = gWord[counter];
	}
	else{
		z = 0;
		//DAC1RDAT = 0x1111;
		DAC1RDAT = 0x0000;
	}
	*/
	
	//Only intake data when not recording	
	if(isRecording == 0){	//Only activate when not recording	
		if(isPlaybackStart == 1){
			LATAbits.LATA2 = 1;		//RA2	p30
			LATAbits.LATA1 = 1;		//GREEN
			//Entering the playback stage
			isPlayback = 1;
		}
		//How to handle out of bounds?
		DAC1RDAT = gWord[counter];
		counter = counter + 1;
		if(counter >= SAMPLE_TIME - 10){	//Only when all of the array has been pushed out
			replayCount++;
			counter = 0;
			LATAbits.LATA1 = 0;		//GREEN			
			LATAbits.LATA2 = 0;		//RA2	p30
			//Handle control logic outside of ISR
			isPlaybackStart = 0;
			isPlayback = 0;
		}	
	}	
}

unsigned int SPI_Receive(){
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x0000;
    while(SPI1STATbits.SPIRBF == 0);
    
    return SPI1BUF;
}

void clockSetup(){
     //Clock source definition - A single FRC internal clock
    OSCTUNbits.TUN      = 0;    //Select FRC = 7.37 MHz
    
    //CLKDIV register
    CLKDIVbits.FRCDIV   = 0;    //FRCDIVN = FRC/1 = 7.37MHz = FIN
    CLKDIVbits.PLLPRE   = 0;    //FIN/2 = 7.37/2 = 3.685MHz
    PLLFBDbits.PLLDIV   = 38;   //FVCO = 3.68*40 = 147.4MHz
    CLKDIVbits.PLLPOST  = 3;    //FRCPLL = 147.4/8 = 18.42MHz = FOSC and 
                                    //FP = 18.42/2 = 9.21MHz
	//The following controls the speed of the program loop....
		//0 is faster than 3
	//CLKDIVbits.PLLPOST  = 0;    //FRCPLL = 147.4/8 = 18.42MHz = FOSC and 
    CLKDIVbits.DOZE     = 0;    //FCY = FP/1 = 9.21MHz
    CLKDIVbits.DOZEN    = 1;    //DOZE is the ratio between CPU and periph clk
    
    
    //ACLKCON Register = Auxiliary clock control for DAC
    ACLKCONbits.AOSCMD	 = 0;    //Aux clock is disabled
	ACLKCONbits.SELACLK  = 0;    //Select PLL Output
    ACLKCONbits.APSTSCLR = 7;    //FA = FOSC/1 = 18.42 MHz
	//Causes infinite loop when play back??
	//ACLKCONbits.APSTSCLR = 0;    //FA = FOSC/1 = 18.42 MHz
}

void spiSetup(){
    SPI1CON1bits.DISSCK = 0;    //SPI1 clock on SCK1 pin is enabled
    SPI1CON1bits.DISSDO = 0;    //SDO1 pin is controlled by module
    SPI1CON1bits.MODE16 = 1;    //Communication is word wide (16 bits)
    SPI1CON1bits.SMP    = 0;    //Sample input at middle of data output time
    SPI1CON1bits.CKE    = 1;    //Output data changes when SCK goes ACTIVE to IDLE
    SPI1CON1bits.SSEN   = 0;    //Slave select pin is controlled by PORT function
    SPI1CON1bits.CKP    = 0;    //Idle state for clock is a low level, active is high 
    SPI1CON1bits.MSTEN  = 1;    //Enable MASTER mode
	/*
		SPI Clock Frequency:
			FSCK = (FCY)/(Primary Prescale * Secondary Prescale)
		FSCK is the serial clock and is provided to external devices through SCKx pin
		Path:
			FP -> Primary Prescale -> Secondary Prescale -> SCKx
	*/
	//Primary prescale bits		64:1
    SPI1CON1bits.PPRE   = 0;    //Primary SPI clock = 9.21/64 = 143.9KHz
	//Secondary prescale bits	3:1
    SPI1CON1bits.SPRE   = 5;    //Second SPI clock  = 143.9/3 = 48KHz 
	//SCK = 48KHz
    
    SPI1STATbits.SPIROV = 0;    //Clear initial overflow bit 
    SPI1STATbits.SPIEN  = 1;    //SPI1 module is enabled, SCK1, SDO1 and SS1' are PORT pins
}

void pinSelect(){
    PPSUnLock;
    RPOR4bits.RP8R      = 8;    //RP8 output for SCK1
    RPINR20bits.SCK1R   = 8;    //RP8  input for SCK1
    RPINR20bits.SDI1R   = 9;    //RP9  input for SDI1
    //No SDO is needed, MIC is not expecting input...
    RPOR3bits.RP6R      = 0;    //RP6 output for SS1
								//No peripheral mapped to RP6
								//RP6 output is controlled by LATB
    RPOR6bits.RP12R     = 0;    //RP12 analog output for DAC1 (DAC1RP)
	RPOR6bits.RP13R     = 0;    //RP13 analog output for DAC1 (DAC1RN)

    PPSLock;
}

void ioSetup(){
	//Microphone pins
    TRISBbits.TRISB6    = 0;    //RB6 output for SS                 p42
    TRISBbits.TRISB8    = 0;    //RB8 output for SCK1               p44    
	TRISBbits.TRISB9    = 1;    //RB9 input for MISO                p01
	
	//DAC Pin
	TRISBbits.TRISB12   = 0;    //RB12 output for DAC1RP            p10
	TRISBbits.TRISB13   = 0;    //RB13 output for DAC1RN            p11

    //LED Debugging
    AD1PCFGL = 0xFFFF;          //AN0-AN8 are now GPIO
    //Debugging recording/playback states
	TRISAbits.TRISA0 = 0;       //RA0 = output      p19
    TRISAbits.TRISA1 = 0;       //RA1 = output      p20    
	//Debugging misc.
    TRISAbits.TRISA8 = 0;       //RA8 = output      p32    
	TRISAbits.TRISA4 = 0;       //RA4 = output      p34
	TRISAbits.TRISA9 = 0;       //RA9 = output      p35		
	TRISAbits.TRISA2 = 0;       //RA2 = output      p30
	TRISAbits.TRISA3 = 0;       //RA3 = output      p31	
}

void setupDAC(){
    //Configure DAC
	//Status Register:
    DAC1STAT = 0;				//Set status of both DAC1RDAT and DAC1LDAT to be EMPTY
	
	//Enabling right channel
    DAC1STATbits.ROEN   = 1;
	
	//Set the DAC default value: (Shows when there is no  value in DAC1RDAT FIFO)
		//Use the midpoint value
	DAC1DFLT = 0x7FFF;		//This makes no difference....
	
	//Setup for both channels:
    DAC1CONbits.DACFDIV = 28;   //FDAC = FOUT/AB = 18.42MHz/28 = 635KHz
									//Equivalent to DAC sample rate 2.5KHz	?
	//DAC1CONbits.DACFDIV = 72 - 1;	//FDAC = FOUT/AB = 147MHz/72 = 2MHz
										//From Table, 2MHz == 8KHz Sample Data Rate
    DAC1CONbits.FORM    = 0;    //DAC data in format is unsigned integer
	
	//Clear interrupt flags
	IFS4bits.DAC1RIF = 0;
		
	//Interrupt type.. LEAVE AT DEFAULT
	//DAC1STATbits.RITYPE = 1;	//Fire interrupt when FIFO Full
	//DAC1STATbits.LITYPE = 0;	//FIFO Not Full, FIFO is empty or has data
	
	//Enable interrupts
	IEC4bits.DAC1RIE = 1;
	//DAC1RDAT = 0; //?

	DAC1CONbits.AMPON = 0;
	    
	DAC1CONbits.DACEN   = 1;    //Enable DAC
}

/*
 *	Interpreting the digital data to voltages...
 *	When FORM = 0 (unsigned integer):
 *		0xFFFF		Most positive output voltage
 *		0x8000		Midpoint output voltage
 *		0x7FFF		Value just below midpoint
 *		0x0000		Minimum output voltage
 */
int main(void) {
    unsigned int i, Delay;
    unsigned int delayLimit = 60;
    unsigned int replayLimit = 3;

	//Setups
    clockSetup();
    spiSetup();
    pinSelect();
    ioSetup();
    setupDAC();
    
	//Debug LEDS
    LATAbits.LATA0 = 0;		//RA0	p19
    LATAbits.LATA1 = 0;		//RA1	p20
	
    LATAbits.LATA8 = 0;		//RA8	p32
    LATAbits.LATA4 = 0;		//RA4	p34
    LATAbits.LATA9 = 0;		//RA9	p35
    LATAbits.LATA2 = 0;		//RA2	p30
    LATAbits.LATA3 = 0;		//RA3	p31
	
    i = 0;
    while(1){
		//Record for some predetermined time length
		if(isRecording){
			//Turn ON RED LED (p19)= Recording
			LATAbits.LATA0 = 1;
			//What does this delay do?
			delay(400, 400);
			// Record for some length of time, feed contents into global array
			for(i = 0; i < SAMPLE_TIME; i++){		
				LATBbits.LATB6 = 0;					//Turn ON SS pin
				gWord[i] = (SPI_Receive() << 4);	//Intake data from SPI bus
				LATBbits.LATB6 = 1;					//Turn OFF SS pin
				//What does this delay do?
				for(Delay = 0; Delay < delayLimit; Delay++);
			}
			//No longer recording
			isRecording = 0;
			//Turn OFF RED LED (p19)= Not Recording
			LATAbits.LATA0 = 0;	
			//Start the playback
			isPlaybackStart = 1;
			
			//Put something in the FIFO to activate the interrupts?
				//TODO: Adjust interrupt config in setupDAC()
			DAC1RDAT = 0x1111;
		}
		//USE INTERRUPTS INSTEAD OF STATUS REGISTER*********************
		//Toggle debug LED to visualize steps...
        switch(replayCount){
			case 0:			
				LATAbits.LATA8 = 1;		//RA8	p32
				delay(100,1000);
				LATAbits.LATA8 = 0;		//RA8	p32
				break;
			case 1:
				LATAbits.LATA4 = 1;		//RA4	p34
				delay(100,1000);
				LATAbits.LATA4 = 0;		//RA4	p34
				break;
			case 2:				
				LATAbits.LATA9 = 1;		//RA9	p35
				delay(100,1000);
				LATAbits.LATA9 = 0;		//RA9	p35
				break;
			default:
				break;
		}
		
		if(replayCount >= replayLimit){
			//Restart the process
			isRecording = 1;
			replayCount = 0;
        }
    }
    	
	//Adding this while statement will cause main() to loop infinitely thus only running once.
	//while(1);
	
	//Remove return otherwise main() will infinite loop
    //return 0;
}
