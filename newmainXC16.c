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
    
//How long to read from mic 5000ms = 5s
#define SAMPLE_TIME		5000

unsigned int counter = 0;
unsigned int gWord[SAMPLE_TIME];
unsigned int dacBuf[SAMPLE_TIME];
	
unsigned int delayLimit = 60;    

unsigned int playDac = 0;


void delay(int limit1, int limit2){
    unsigned int i, j;
    for(i = 0; i < limit1; i++){
        for(j = 0; j < limit2; j++){       
        }
    }
}

/*
 *	ISR For Right Channel DAC
 *	Interrupt will constantly activate
 *	However, it'll only function when flag is active
 */
void __attribute__((interrupt, no_auto_psv))_DAC1RInterrupt(void){
	IFS4bits.DAC1RIF = 0;		//Clear Right Channel Interrupt Flag
	
	if(playDac == 1){
		DAC1RDAT = dacBuf[counter];
		counter = counter + 1;
		if(counter >= SAMPLE_TIME - 10){
			counter = 0;
			playDac = 0;
		}	
	}	
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
}

void spiSetup(){
    SPI1CON1bits.DISSCK = 0;    //SPI1 clock on SCK1 pin is enabled
    SPI1CON1bits.DISSDO = 0;    //SDO1 pin is controlled by module
	SPI1CON1bits.MODE16 = 0;    //Communication is 8 bit wide
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
    //SPI1CON1bits.SPRE   = 5;    //Second SPI clock  = 143.9/3 = 48KHz     
	SPI1CON1bits.SPRE   = 7;    //Second SPI clock
	//SCK = XXKHz
    
    SPI1STATbits.SPIROV = 0;    //Clear initial overflow bit 
    SPI1STATbits.SPIEN  = 1;    //SPI1 module is enabled, SCK1, SDO1 and SS1' are PORT pins
}

void pinSelect(){	    
	PPSUnLock;
    RPOR4bits.RP8R = 8;     //RP8 is an output	for SCK1
    RPINR20bits.SCK1R = 8;  //RP8 is an input	for SCK1
    
	RPOR4bits.RP9R = 7;     //RP9 is an output	for SDO1	
    RPINR20bits.SDI1R = 7;  //RP7 is an input	for SDI1
	
	RPOR2bits.RP5R	= 9;	//RP5 output for SS1	for MEM    
	RPOR3bits.RP6R	= 0;    //RP6 output for SS2	for MIC

	RPOR6bits.RP12R	= 0;    //RP12 analog output for DAC1 (DAC1RP)
	RPOR6bits.RP13R	= 0;    //RP13 analog output for DAC1 (DAC1RN)
	PPSLock;
}

void ioSetup(){
	//SPI Pins
    TRISBbits.TRISB8    = 0;    //RB8 output for SCK1               p44    
	TRISBbits.TRISB9    = 1;    //RB9 output for MOSI               p01
	TRISBbits.TRISB7	= 1;	//RB7 input  for MISO    
	TRISBbits.TRISB5	= 0;	//RB5 output for SS1 (MEM)
	TRISBbits.TRISB6    = 0;    //RB6 output for SS2 (MIC)			p42
	
	//DAC Pin
	TRISBbits.TRISB12   = 0;    //RB12 output for DAC1RP            p10
	TRISBbits.TRISB13   = 0;    //RB13 output for DAC1RN            p11

    AD1PCFGL = 0xFFFF;          //AN0-AN8 are now GPIO
	
	//Button inputs
    TRISAbits.TRISA0=1; //Pin RA0 is assigned as input for record
    TRISAbits.TRISA1=1; //Pin RA1 is assigned as input for play
    TRISAbits.TRISA4=1; //Pin RA4 is assigned as input for erase
	
	//LED Outputs 
	TRISAbits.TRISA2=0; //Pin RA2 is assigned as output for Red LED
    TRISAbits.TRISA3=0; //Pin RA3 is assigned as output for Green LED
	TRISAbits.TRISA8=0; //Pin RA8 is assigned as output for Yellow LED
	
	//Debug
	TRISAbits.TRISA7 = 0; //RA7 output debug LED
}

void setupDAC(){
    //Configure DAC
	//Status Register
    DAC1STAT = 0;
	
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
	//DAC1RDAT = 0; //Some data is required to initialize interrupts?

	DAC1CONbits.AMPON = 0;
	    
	DAC1CONbits.DACEN   = 1;    //Enable DAC
}

/*
unsigned int SPI_Receive(){
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x0000;
    while(SPI1STATbits.SPIRBF == 0);
    
    return SPI1BUF;
}
 * */

unsigned char SPI_Transmit(unsigned char TxValue)
{
    while(SPI1STATbits.SPITBF==1);//Wait until the TX buffer is empty due to a prior process
    SPI1BUF = TxValue;            //When empty, send the byte to the TX buffer
    while(SPI1STATbits.SPIRBF==0);//As valid bits shifts out of SDO, junk bits are received from SDI
                                  //Wait until the RX buffer is full of junk data
    return SPI1BUF;               //When full, read the junk data in RX buffer through SPI1BUF
}

unsigned char SPI_Receive()
{
    while(SPI1STATbits.SPITBF==1);//Wait until the TX buffer is empty due to a prior process
    SPI1BUF = 0xBB;               //When empty, send the junk byte (0x00) to the TX buffer
    while(SPI1STATbits.SPIRBF==0);//As junk bits shifts out of SDO, valid bits are received from the SDI
                                  //Wait until the RX buffer is full of valid data
    return SPI1BUF;               //When full, read the valid data in RX buffer through SPI1BUF
}

void WREN()
{
    LATBbits.LATB5 = 0; //Lower SS for instruction delivery
    SPI_Transmit(0x06); //Send the WREN instruction
    LATBbits.LATB5 = 1; //Raise SS for instruction completion
}

unsigned char Read_Status_Reg()
{
    LATBbits.LATB5 = 0;                  //Lower SS for instruction delivery
    SPI_Transmit(0x05);                  //Send the Read status register instruction
    unsigned char status = SPI_Receive();//Read the status register
    LATBbits.LATB5 = 1;                  //Raise SS for instruction completion
    return status;
}

unsigned char Write_Byte (unsigned char command, long int address, unsigned char SentByte)
{
    LATBbits.LATB5 = 0;                  //Lower SS to start PP
    SPI_Transmit(command);               //Send the PP instruction
    SPI_Transmit((address >> 16) & 0xFF);//Send the  MS byte of the 24-bit address
    SPI_Transmit((address >> 8) & 0xFF); //Send the mid byte of the 24-bit address
    SPI_Transmit(address & 0xFF);        //Send the LS byte of the 24-bit address
    SPI_Transmit(SentByte);
    unsigned char JunkByte = SPI_Receive();
    LATBbits.LATB5 = 1;                  //Raise SS for instruction completion
    return JunkByte;
}

unsigned char Read_Byte (unsigned char command, long int address)
{
    LATBbits.LATB5 = 0;                  //Lower SS to start PP
    SPI_Transmit(command);               //Send the PP instruction
    SPI_Transmit((address >> 16) & 0xFF);//Send the MS byte of the 24-bit address
    SPI_Transmit((address >> 8) & 0xFF); //Send the mid byte of the 24-bit address
    SPI_Transmit(address & 0xFF);        //Send the LS byte of the 24-bit address
    unsigned char ReceivedByte = SPI_Receive();
    LATBbits.LATB5 = 1;                  //Raise SS for instruction completion
    return ReceivedByte;
}

void clear_memory()
{
	//Process to clear FLASH memory
	LATAbits.LATA8=1;   //Yellow LED on
	LATBbits.LATB5=0;  //Lower SS for instruction delivery
	SPI_Transmit(0x50);//Enable-Write_Status-Register opcode
	LATBbits.LATB5=1;  //Raise SS for instruction completion

	LATBbits.LATB5=0;  //Lower SS for instruction delivery
	SPI_Transmit(0x01);//Write-Status-Register opcode
	SPI_Transmit(0x00);//Status Register:0x00
					   //Must set BP2,BP1,BP0 to 0 in order to take off write protect
	LATBbits.LATB5=1;

	LATBbits.LATB5=0;
	SPI_Transmit(0x05);//Read status register Opcode
	SPI_Receive();     //Read status register
	LATBbits.LATB5=1;


	WREN();
	LATBbits.LATB5=0;
	SPI_Transmit(0xC7);//Opcode for chip erase
	LATBbits.LATB5=1;

	while((Read_Status_Reg() & 0x01) == 0x01);//Wait for memory to be clear

	Read_Byte(0x03,0x020000);//Check when data is 0xFF

	Read_Status_Reg();            
	delay(100,1000);
	LATAbits.LATA8=0;   //Yellow LED off
	LATAbits.LATA2=1;   //Red LED on
	LATAbits.LATA3=1;   //Green LED on
	delay(100,100);
}
void flash_setup()
{
	WREN();
	while((Read_Status_Reg() & 0x02)==0x00);//While WEL=0 keep reading the status register Read_Status_Reg();
	LATBbits.LATB5=0;
	SPI_Transmit(0xAD);//AAI Word Program instruction opcode
	SPI_Transmit(0x02);//Address 0x020000
	SPI_Transmit(0x00);
	SPI_Transmit(0x00);
	SPI_Transmit(0xAB);//Word Byte Data: 0xABCD//This will be junk
	SPI_Transmit(0xCD);
	LATBbits.LATB5=1;
	while((Read_Status_Reg()& 0x01) == 0x01);//Polling Busy bit in the status register

	LATAbits.LATA8=1;   //Yellow LED on
	LATAbits.LATA2=0;   //Red LED off
	LATAbits.LATA3=0;   //Green LED off
	delay(100,1000);
}
void record()
{
	LATAbits.LATA2=1;   //Red LED on
	while((Read_Status_Reg()& 0x01) == 0x01);//Polling Busy bit in the status register

	LATBbits.LATB5=0;
	SPI_Transmit(0xAD);//AAI opcode
	SPI_Transmit(0xEF);//Word Byte Data: 0xEFBA Actual Data
	SPI_Transmit(0xBA);
	LATBbits.LATB5=1;
}

void recordMicData(){	
	unsigned int i;
	
	LATAbits.LATA2=1;   //Red LED on
	while((Read_Status_Reg()& 0x01) == 0x01);//Polling Busy bit in the status register

	LATBbits.LATB5=0;
	SPI_Transmit(0xAD);//AAI opcode
	//Loop through the collected samples
	for(i = 0; i < SAMPLE_TIME; i++){
		//Each element is 16 bits, split into two 8 bits and transfer out
		SPI_Transmit((gWord[i] >> 8) & 0xFF);	//High
		SPI_Transmit((gWord[i] >> 0) & 0xFF);	//Low
	}
	LATBbits.LATB5=1;
}

void exit_record()
{
	LATAbits.LATA3=1;   //Green LED on

	LATBbits.LATB5=0;    
	SPI_Transmit(0x04);//Opcode WRDI to exit AAI Mode
	LATBbits.LATB5=1;
}
void play()//read data
{
	Read_Status_Reg();//Read status register
	LATBbits.LATB5=0;
	SPI_Transmit(0x03);//Read opcode
	SPI_Transmit(0x02);//3-byte start address
	SPI_Transmit(0x00);
	SPI_Transmit(0x00);
	while(SPI_Receive()!=0xFF)//Read all the data that was written until we reach the area of memory that is clear
	{
		SPI_Receive();
	}
	LATBbits.LATB5=1;
}

void playMicData(){
	unsigned int i = 0;
		
	Read_Status_Reg();//Read status register
	LATBbits.LATB5=0;
	SPI_Transmit(0x03);//Read opcode
	SPI_Transmit(0x02);//3-byte start address
	SPI_Transmit(0x00);
	SPI_Transmit(0x00);
	while(SPI_Receive()!=0xFF)//Read all the data that was written until we reach the area of memory that is clear
	{
		//Fetch 8 bit data from memory twice, store into 16 bit element
		dacBuf[i] = ((SPI_Receive() << 8) | (SPI_Receive()));
		i = i + 1;
	}
	//All of the flash data has been pushed to DAC buffer array, signal to push to DAC
	playDac = 1;
	LATBbits.LATB5=1;
}

void getMicData(){
    unsigned int i, Delay;
	
	for(i = 0; i < SAMPLE_TIME; i++){		
		LATBbits.LATB6 = 0;					//Turn ON SS pin
		//gWord[i] = (SPI_Receive() << 4);	//Intake data from SPI bus
		//Receive 16 bit data in 8 bit mode...
		//Store two 8 bit data into 16 bit element
		gWord[i] = ((SPI_Receive() << 8) | (SPI_Receive())) << 4;
		LATBbits.LATB6 = 1;					//Turn OFF SS pin
		//What does this delay do?
		for(Delay = 0; Delay < delayLimit; Delay++);
	}
	//Put something in the DAC FIFO to activate interrupts?	
	DAC1RDAT = 0x1111;
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
	//Setups
    clockSetup();
    spiSetup();
    pinSelect();
    ioSetup();
    setupDAC();
    
	//Debug LED
    LATAbits.LATA7 = 0;
	
	while(1){
		//Press erase button and wait for LED to turn off
		//When the LED goes from ON to OFF, this indicates that the memory is cleared
		if(PORTAbits.RA4 == 1){	//Erase button is pressed
			clear_memory();
			flash_setup();
		}
		else{					//Erase button is not pressed
			LATAbits.LATA8 = 0;	//Yellow LED off
		}
		
		//Press and Hold Record button
		if(PORTAbits.RA0 == 1){	//Record button is pressed
			getMicData();		//Obtain data from microphone
			recordMicData();	//Write microphone data onto memory
			//record();			//Write data onto flash memory
		}
		else{					//Record button not pressed
			LATAbits.LATA2 = 0;	//Red LED off
		}
		
		if(PORTAbits.RA1 == 1){	//Play button is pressed
			exit_record();		//Function to exit AAI
			playMicData();		//Function to read microphone data from flash memory
			//play();				//Function to read data from flash memory
		}
		else{					//Play button is not pressed
			LATAbits.LATA3 = 0;	//Green LED off
		}
	}
    	
	//Remove return otherwise main() will infinite loop
    //return 0;
}
