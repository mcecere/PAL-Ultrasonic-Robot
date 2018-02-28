
////////////////////////////////////////////////////////////////
// Timer1 Fast Timer Service Routine
//
// Prescaler = 1
// Timer1 Prescaler = 1; give 40KHz UltraSonic Pulses
// 1 x 25nS x (499+1) = 12.500uS = 40KHz
//////////////////////////////////////////	//////////////////////
#include "p33fj128gp802.h"

extern int PulseCount;
extern int Count256uS;
extern int BunchCount;
extern int MaxBunch;
extern int PulseSep;
extern int PulseCount;
extern int MaxPulses;
extern int ServoSet1;
extern int ServoSet2;
extern int ServoSet3;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
	IFS0bits.T1IF = 0;			// clear interrupt flag
	IEC0bits.T1IE = 0;			// disable interrupt

	if (PulseCount>0) {
		LATBbits.LATB15 ^= 1;	// toggle differential US xmtr drive
		LATBbits.LATB14 ^= 1;	//
		PulseCount = PulseCount - 1;	// only do this MaxPulses times
	}	// if PulseCount > 0;

	IEC0bits.T1IE = 1;			// enable interrupt
}
////////////////////////////////////////////////////////////////
// Timer2 Slow Timer Service Routine
//
// Timer2 Prescaler = 256 x 25nS = 6.4uS x 40 = 0.256mS
// 
////////////////////////////////////////////////////////////////
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
	IFS0bits.T2IF = 0;			// clear interrupt flag
	IEC0bits.T2IE = 0;			// disable interrupt
	
	Count256uS = Count256uS + 1;

// start pulse trains this often
	if (Count256uS> 400) {	// every 102.4mS
		Count256uS = 0;
		BunchCount = MaxBunch;	// start a string of bunches
	}
// finish off the pulse train
	if ((Count256uS > (PulseSep *(MaxBunch - BunchCount)))& (BunchCount>0)) {
		BunchCount = BunchCount - 1;	// one less bunch to make
		PulseCount = MaxPulses;			// start pulse generation
	}
// Start ADC and record data, when Count

// analyze data for amplitude and distance

// 
	IEC0bits.T2IE = 1;			// enable interrupt
}
////////////////////////////////////////////////////////////////
// Timer3 Servo Timer Service Routine
//
// Timer3 Prescaler = 256 x 25nS x 3125 = 20.0mS
// 
////////////////////////////////////////////////////////////////
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
{
	IFS0bits.T3IF = 0;			// clear interrupt flag
	IEC0bits.T3IE = 0;			// disable interrupt
//TMR3 = 0x00; 				// Clear timer register

OC1RS = ServoSet1;	// Write Duty Cycle value for Servo1
OC2RS = ServoSet2;	// Write Duty Cycle value for Servo2
OC3RS = ServoSet3;	// Write Duty Cycle value for Servo3

	IEC0bits.T3IE = 1;			// enable interrupt
}



// OC1 Output Comparator Servo1 control
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt( void)
{
IFS0bits.OC1IF	= 0;	// clear output compare 1 interrupt flag
}
// OC2 Output Comparator Servo2 control
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt( void)
{
IFS0bits.OC2IF	= 0;	// clear output compare 2 interrupt flag
}
// OC3 Output Comparator Servo3 control
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt( void)
{
IFS1bits.OC3IF	= 0;	// clear output compare 3 interrupt flag
}

