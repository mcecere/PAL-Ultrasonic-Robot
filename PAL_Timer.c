#include "system.h"
//
#include "p33fj128gp802.h"

extern int PulseCount;
extern int ACQCount256uS;
extern int BunchCount;
int BunchSep256uS;
extern int MaxBunch;
extern int PulseSep;
extern int PulseCount;
extern int MaxPulses;
extern int ServoSet1;
extern int ServoSet2;
extern int ServoSet3;
extern int Servo_Period;
extern int StartADC;
extern int EnableMotion;

void NumNops(unsigned count) {
int i;
// for 40MHz clock cycle:
// count = 1   --> 0.6uS
// count = 2   --> 0.85uS
// count = 5   --> 1.5uS
// count = 10  --> 2.66uS
// count = 100 --> 22.9uS
// count = 1000 -> 226uS
// for 14.75MHz clock cycle:
// count = 1   --> 0.6uS
// count = 2   --> 0.85uS
// count = 5   --> 1.5uS
// count = 10  --> 2.66uS
// count = 23  --> 12.5uS
// count = 100 --> 49.5uS
// count = 1000 -> 480uS
for (i=0;i<count;i++) Nop();
}

void Delay_mS(unsigned count) {
// count = # mS to wait
int i;
for (i=0;i<count;i++) NumNops(5832);
}
	
///////////////////////////////////////////////////////////////////////
// Timer1 Prescaler = 1:1	Ultrasonic 40 KHz Transmitter timing
// Timer1 is dedicated to generating the 2-bit quadrature outputs
// to drive the Ultrasonic transmitter at exactly 40KHz
//
void InitTimer1() 		
{
T1CONbits.TON = 0x0000;		// stop timer1
T1CONbits.TCS = 0; 			// Select internal instruction cycle clock
T1CONbits.TGATE = 0; 		// Disable Gated Timer mode
T1CONbits.TCKPS = 0b00;		// sets prescale to 1:1
TMR1 = 0x00;				// Clear contents of the timer register
PR1 = 499;					// load period register
							// (499.0 + 1) * 25nS = 12.5uS exactly
IPC0bits.T1IP = 0x01;		// interrupt priority level
IFS0bits.T1IF = 0;			// Clear timer1 interrupt status flag
IEC0bits.T1IE = 1;			// Enable timer1 interrupts
T1CONbits.TON = 1;			// Start TMR1
}


///////////////////////////////////////////////////////////////////////////////
// Timer2 Prescaler = 256 x 25nS x 3125 = 20.0mS
// forms the basis for the servo control pulses at 20mS rep rate
//
// OC1 OutputComparator is used for servo1 control
// OC2 OutputComparator is used for servo2 control
// OC3 OutputComparator is used for servo3 control
void InitTimer2() 		
{
// Initialize and enable Timer2
T2CONbits.TON = 0; 			// Disable Timer
T2CONbits.TCS = 0; 			// Select internal instruction cycle clock
T2CONbits.TGATE = 0; 		// Disable Gated Timer mode
T2CONbits.TCKPS = 0b11; 	// Select 1:256 Prescaler
TMR2 = 0x00; 				// Clear timer register
PR2 = Servo_Period - 1;		// Load the period value (3124+1) x 6.4uS = 20.0mS
IPC1bits.T2IP = 0x03; 		// Set Timer 2 Interrupt Priority Level
IFS0bits.T2IF = 0; 			// Clear Timer 2 Interrupt Flag
IEC0bits.T2IE = 1; 			// Enable Timer 2 interrupt
T2CONbits.TON = 1; 			// Start Timer

// Servo1 output comparator
OC1CONbits.OCM = 0b000; 	// Disable Output Compare Module
OC1RS = ServoSet1; 			// Write the duty cycle for the second PWM pulse
OC1CONbits.OCTSEL = 0; 		// 
OC1CONbits.OCM = 0b110; 	// Select the PWM Output Compare mode 

// Servo2 output comparator
OC2CONbits.OCM = 0b000; 	// Disable Output Compare Module
OC2RS = ServoSet1; 			// Write the duty cycle for the second PWM pulse
OC2CONbits.OCTSEL = 0; 		// Select Timer 2 as output compare time base
OC2CONbits.OCM = 0b110; 	// Select the PWM Output Compare mode 

// Servo3 output comparator
OC3CONbits.OCM = 0b000; 	// Disable Output Compare Module
OC3RS = ServoSet1; 			// Write the duty cycle for the second PWM pulse
OC3CONbits.OCTSEL = 0; 		// Select Timer 2 as output compare time base
OC3CONbits.OCM = 0b110; 	// Select the PWM Output Compare mode 

RPOR5 = 0x1400;				// RB11 is output of OC3
RPOR6 = 0x1213;				// RB13 is output of OC1, RB12 as output of OC2
}
///////////////////////////////////////////////////////////////////////
// Timer3 ADC Timer Control for acquisition rate
// Prescaler = 8 x 25nS = 200nS * 250 = 50uS
// 
// 
void InitTimer3() 		
{
T3CONbits.TON = 0x0000;		// stop timer3
T3CONbits.TCS = 0; 			// Select internal instruction cycle clock
T3CONbits.TGATE = 0; 		// Disable Gated Timer mode
T3CONbits.TCKPS = 0b001;	// sets prescale to 8
TMR3 = 0x00;				// Clear contents of the timer register
PR3 = 249;					// load period register
							// (249 + 1) * 8 * 25nS = 50uS
IPC2bits.T3IP = 0x01;		// interrupt priority level
IFS0bits.T3IF = 0;			// Clear timer3 interrupt status flag
IEC0bits.T3IE = 1;			// Enable timer3 interrupts
T3CONbits.TON = 0;			// don't start TMR3

}
///////////////////////////////////////////////////////////////////////
// Timer4 Prescaler = 256  MAIN TIMER FOR UltraSonic loop
// 
// interrupts every 1/4mS or 256uS, 
//
void InitTimer4() 		
{
T4CONbits.TON = 0x0000;		// stop timer4
T4CONbits.TCS = 0; 			// Select internal instruction cycle clock
T4CONbits.TGATE = 0; 		// Disable Gated Timer mode
T4CONbits.TCKPS = 0b11;		// sets prescale to 256
TMR4 = 0x00;				// Clear contents of the timer register
PR4 = 39;					// load period register
							// (39 + 1) * 6.4000uS = 256uS = 1/4mS
IPC6bits.T4IP = 0x04;		// interrupt priority level
IFS1bits.T4IF = 0;			// Clear timer4 interrupt status flag
IEC1bits.T4IE = 1;			// enable timer 4
T4CONbits.TON = 0;			// don't start timer 4

}

////////////////////////////////////////////////////////////////
// Timer1 Fast Timer Service Routine
//
// Prescaler = 1
// Timer1 Prescaler = 1; give 40KHz UltraSonic Pulses
// 1 x 25nS x (499+1) = 12.500uS = 40KHz
//////////////////////////////////////////	//////////////////////
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
	IFS0bits.T1IF = 0;			// clear interrupt flag
	IEC0bits.T1IE = 0;			// disable interrupt
//	TMR1 = 0;
	if (PulseCount>0) {
		LATBbits.LATB15 ^= 1;	// toggle differential US xmtr drive
		LATBbits.LATB14 ^= 1;	//
		PulseCount = PulseCount - 1;	// only do this MaxPulses times
	}	// if PulseCount > 0;

	IEC0bits.T1IE = 1;			// enable interrupt
}
////////////////////////////////////////////////////////////////
// Timer2 Servo Timer Service Routine
//
// Timer2 Prescaler = 256 x 25nS x 3125 = 20.0mS
// 
////////////////////////////////////////////////////////////////
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
	IFS0bits.T2IF = 0;			// clear interrupt flag
	IEC0bits.T2IE = 0;			// disable interrupt
//		LATBbits.LATB3 ^= 1;		// RB3, pin7

if (EnableMotion) {
	OC1RS = ServoSet1;	// Write Duty Cycle value for Servo1
	OC2RS = ServoSet2;	// Write Duty Cycle value for Servo2
	OC3RS = ServoSet3;	// Write Duty Cycle value for Servo3
}
	IEC0bits.T2IE = 1;			// enable interrupt
}

////////////////////////////////////////////////////////////////
// Timer4 Slow Timer Service Routine
//
// Timer4 Prescaler = 256 x 25nS = 6.4uS x 40 = 0.256mS
// 
////////////////////////////////////////////////////////////////
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void)
{
	IFS1bits.T4IF = 0;			// clear interrupt flag
	IEC1bits.T4IE = 0;			// disable interrupt
	
	ACQCount256uS = ACQCount256uS + 1;

// make pulse bunchs
	if (BunchCount == MaxBunch) {		// initiates first pulse and marks this point
		BunchCount--;					// one less bunch to make
		LATBbits.LATB15 = 1;			// pre-stagger the diff-pair going to the RS-232 chip
		LATBbits.LATB14 = 0;			// 
		PulseCount = MaxPulses;			// start pulse generation
		BunchSep256uS = ACQCount256uS - 1;		// set origin of first pulse bunch
	}
	if (((ACQCount256uS - BunchSep256uS) > (PulseSep *(MaxBunch - BunchCount)))& (BunchCount>0)) {
		BunchCount--;					// one less bunch to make
		LATBbits.LATB15 = 1;			// pre-stagger the diff-pair going to the RS-232 chip
		LATBbits.LATB14 = 0;			// 
		PulseCount = MaxPulses;			// start pulse generation
	}

	IEC1bits.T4IE = 1;			// enable interrupt
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

/*********************************************************************
 * EOF
 ********************************************************************/
