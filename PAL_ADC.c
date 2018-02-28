/**********************************************************************

**********************************************************************/

#include "system.h"

#include "p33fj128gp802.h"
#include "PAL_Main.h"
#include "PAL_ADC.h"

extern int ADCDone;

int ADCBuffer[BUFFLEN] __attribute__((space(dma)));
float win[BUFFLEN];

/*=============================================================================
InitADC() is used to configure A/D to convert channel 5 on Timer event. 
It generates event to DMA on every sample/convert sequence. ADC clock is configured at 625Khz 
=============================================================================*/
void InitADC(void)
{

//		AD1CON1bits.FORM   = 3;		// Data Output Format: Signed Fraction (Q15 format)
		AD1CON1bits.FORM   = 0;		// Data Output Format: Integer
		AD1CON1bits.SSRC   = 2;		// Sample Clock Source: GP Timer starts conversion
		AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
		AD1CON1bits.AD12B  = 1;		// 12-bit ADC operation

		AD1CON2bits.CHPS  = 0;		// Converts CH0
     
		AD1CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
		AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
									// ADC Conversion Time for 12-bit Tc=14*Tad = 22.4us 
					
		AD1CON1bits.ADDMABM = 1; 	// DMA buffers are built in conversion order mode
		AD1CON2bits.SMPI    = 0;	// SMPI must be 0


        //AD1CHS0: A/D Input Select Register
        AD1CHS0bits.CH0SA=0;		// MUXA +ve input selection (AN0) for CH0
		AD1CHS0bits.CH0NA=0;		// MUXA -ve input selection (Vref-) for CH0

        //AD1PCFGL: Port Configuration Register
		AD1PCFGL=0xFFFF;
        AD1PCFGLbits.PCFG0 = 0;		// AN0 as Analog Input

        
        IFS0bits.AD1IF = 0;			// Clear the A/D interrupt flag bit
        IEC0bits.AD1IE = 0;			// Do Not Enable A/D interrupt 
        AD1CON1bits.ADON = 1;		// Turn on the A/D converter	

}
// DMA0 configuration
// Direction: Read from peripheral address 0-x300 (ADC1BUF0) and write to DMA RAM 
// AMODE: Register indirect with post increment
// MODE: Continuous, no Ping-Pong Mode
// IRQ: ADC Interrupt
// ADC stores results stored in ADCBuffer[]

void InitDMA0(void)
{
	DMA0CONbits.AMODE = 0;			// Configure DMA for Register indirect with post increment
	DMA0CONbits.MODE  = 0;			// Configure DMA for Continuous NO Ping-Pong mode

	DMA0PAD=(int)&ADC1BUF0;
	DMA0CNT=(BUFFLEN-1);				
	
	DMA0REQ=13;						// ADC1 convert done
	
	DMA0STA = __builtin_dmaoffset(ADCBuffer);		

	IFS0bits.DMA0IF = 0;			//Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1;			//Set the DMA interrupt enable bit
	
	DMA0CONbits.CHEN=1;

}


/*=============================================================================
_DMA0Interrupt(): ISR name is chosen from the device linker script.
=============================================================================*/
unsigned int DmaBuffer = 0;

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{


	ADCDone = 1;				// set Flag, buffer full
	T3CONbits.TON = 0;			// stop TMR3
	TMR3 = 0;
//		LATBbits.LATB3 ^= 1;		// RB3, pin7 flash LED


    IFS0bits.DMA0IF = 0;		//Clear the DMA0 Interrupt Flag
}

////////////////////////////////////////////////////////////////
// Timer3 Servo Timer Service Routine ADC 50uS
//
// Timer3 Prescaler = 
// 
////////////////////////////////////////////////////////////////
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
{
	IFS0bits.T3IF = 0;			// clear interrupt flag
	IEC0bits.T3IE = 0;			// disable interrupt

//		LATBbits.LATB3 ^= 1;		// RB3, pin7 flash LED

	IEC0bits.T3IE = 1;			// enable interrupt
}







