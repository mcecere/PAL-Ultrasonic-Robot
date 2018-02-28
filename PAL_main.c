////////////////////////////////////////////////////////////////////
//
//   PAL_main.c and PAL_main.h
//
// Main program file for Pal the Robotic Dog, 
//
// Michael Cecere, Nov. 2008
//  
////////////////////////////////////////////////////////////////////

#include "p33fj128gp802.h"
#include "PAL_Main.h"
#include "PAL_Timer.h"
#include "PAL_ADC.h"
#include "Pal_Algo.h"

// Primary Oscillator without PLL'
_FOSCSEL(FNOSC_PRIPLL);
//OSC2 Pin Function: OSC2 is clock output
// Primary osc mode: HS crystal
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT);
// Watchdog timer disabled
_FWDT(FWDTEN_OFF);
// Code Protect OFF; Write Protect OFF
_FGS( GCP_OFF & GWRP_OFF);
//Enable ICSP on PGD1
//_FICD(BKBUG_OFF & COE_OFF & JTAGEN_OFF & ICS_PGD1);

// EchoAmp holds the amplitude of each scan
// EchoInches holds the distance of scan
// see PAL_Algo.c for details
float EchoAmp[HISTORYLEN];		// fills up from the last position to the first
float EchoInches[HISTORYLEN];

// these set the number of pulses of 40KHz
int MaxPulses = 12;			// # of ultrasonic cycles in pulse, 8
int MakePulse = 0;			// Enables/disables generation of US pulses
							// When enabled, generates MaxPulses pulses, then ->0
int PulseCount = 0;			// Present # of cycles to make still

// these set the number of bunches of MaxPulse pulses
int MaxBunch = 3;			// # of pulses in a bunch
int BunchCount = 0;			// Present # of bunchs
int PulseSep = 10;			// number of Timer2 int to seperate pulses by
							// 4 * 250uS = 1mS;
int PulseSep50uS;

int ACQCount256uS = 0;			// counter updates every 256uS, used for bunch counting
int StartADC = 0;			// # of 256uS counts to start listening for echo

long int	Servo_Period = 3125;	// 20mS/12.8uS, update rate of servos
float ServoCon = .001/6.4e-6;		// counts/mS, 1/6.4e-6 = 156.25
float ServoPan = .5;				// Pan servo position, 0 - 1.0
float ServoPan2 = 0.5;
float ServoMin = -90.;				// min servo swing, cw
float ServoMax = 90.;				// max servo swing cw, 
							
long int ServoSet1 = 234;			// actual PWM setting of ServoPan
long int ServoSet2 = 234;			// 1mS pulse width = ServoCon = 156.25
long int ServoSet3 = 234;			// 1.5mS pulse width = ServoCon *1.5 = 234.375
int PanDir = 1;						// to the right = 1, to the left = -1
float PanInc = 1.;					// 

int ADCDone = 0;					// Flags when ADC DMA buffer, ADCBuffer[] is full

long int LoopCount = 0;
long int StopCount = 0;

int ACQIndex = 0;					// holds the # of ACQCount256uS since last dir change
float f1, f2;

int FirstRun = 1;
int StartPulse = 8;					// time to wait after moving pan servo

int EnableMotion = 1;				// enable servo control

float PkAmp = -1000.;
int ACQindex = 0;
float HeadAngle[20], HeadAmp[20];
float PkInches;
int	PkIndex, PkAngle;
int PkOverShoot = 1;				// number acquisitions to shoot past peak
float AmpAvg;
float HighSpeed, LowSpeed;			// scanning rate can have two speeds
float EchoNextAvg, EchoPrevAvg;

/////////////////////////////////////////////////////////////////////
//   Convert Degrees to servo setting value, but doesn't set servo
float deg2servo(float deg)
// range:
// deg  =  servo setting
// -180 = .86
//    0 = 1.5
//  180 = 2.14
{
float setting;
//setting = ServoCon*(1.28*(deg+180.)/360.-0.14+1.0);
setting = ServoCon*(1.335*(deg+180.)/360.-0.25+1.0);	// calibrated for a Futaba S3003
return (setting);
}

int main(void)
{
int i;
int ContrastIndex = 0;
// for an 8MHz crystal and 40MIPS
	PLLFBD=38;									// M=40
	CLKDIVbits.PLLPOST=0;						// N1=2
	CLKDIVbits.PLLPRE=0;						// N2=2

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};


	TRISAbits.TRISA0 = 1;	// make chan AN0/RA0/Pin2 as input
//	TRISAbits.TRISA1 = 1;
//	TRISAbits.TRISA3 = 0;

	TRISBbits.TRISB15 = 0;	// Ultrasonic differential drive signal (+)
	TRISBbits.TRISB14 = 0;	// Ultrasonic differential drive signal (-)
	TRISBbits.TRISB13 = 0;	// Servo1 output
	TRISBbits.TRISB12 = 0;	// Servo2 output
	TRISBbits.TRISB11 = 0;	// Servo3 output
	TRISBbits.TRISB10 = 0;	// un-used
	TRISBbits.TRISB3 = 0;	// On-Board LED, pin7, RB3
	
	LATBbits.LATB15 = 1;	// pre-stagger the diff-pair going to the RS-232 chip
	LATBbits.LATB14 = 0;	// for XORing later
	LATBbits.LATB10 = 1;

	// # of samples between each pulse in a pulse bunch
	// PulseSep is in units of 256uS, Timer 2
	PulseSep50uS = PulseSep * 256. / 50.;

	InitTimer1();				// 12.5uS timer for UltraSonic Pulse Generation
	MakePulse = 0;				// disable pulse generation

	InitTimer2();				// RC servo 20mS timer, for all servos
	BunchCount = 0;				// set Bunch counter

	InitTimer3();				// Timer for ADC conversions, 50uS

	InitTimer4();				// 250uS timer for main sensing loop, don't start

	InitADC();					// set up ADC stuff

	InitDMA0();					// DMA pipline for ADC data

	LoopCount = 0;

	for (i=0;i<HISTORYLEN;i++) {
		EchoAmp[i] = 1.;
		EchoInches[i] = 0;
	}

i=0;
i=0;
/*
for (i=0;i<101;i++) {
	ServoSet1 = deg2servo(i*360./100.-180.);
	ServoSet2 = ServoSet1;
	ServoSet3 = ServoSet1;
	Delay_mS(100);
}
i=0;
i=0;
*/

	ServoMax = 180;					// max servo swing in degrees
	ServoMin = -180;				// minimum servo swing in degrees
	HighSpeed = 3.0;				// high speed servo scanning, in degrees
	LowSpeed = 1.;					// low speed servo scanning, in degrees
	PanInc = HighSpeed;				// degree increment for each step
	PanDir = -1;					// start turning left-CCW, right-CW = -1
	StopCount = (ServoMax - ServoMin)/PanInc;
	ServoPan =-180.-PanDir*PanInc;			// start scanning from here
	ServoSet1 = deg2servo(ServoPan);
	ServoSet2 = deg2servo(ServoPan);
	ServoSet3 = deg2servo(ServoPan);
	Delay_mS(500);

	StartPulse = 75./0.05;					// time to wait after moving pan servo mS/samplerate
	StartADC = StartPulse + 5;				// when to start ADC converi
	ADCDone = 0;
	PkAmp = -1000;
	ACQindex = 0;
	HeadAngle[0] = 0.;
	HeadAmp[0] = -1000;
	ACQCount256uS = 0;				// incremented by timer4 every 256uS
	FirstRun = 1;

	T4CONbits.TON = 1;				// start timer 4, State machine clock

//////////////////////////////////////////////////////////////////////////////
	while(1) {

// STATE1: first move pan servo an increment
	if ((ACQCount256uS == 0) & (FirstRun)) {
		FirstRun = 0;
// autospeed adjustment
//		if ((PkAmp<30)&(PkAmp>0)) PanInc = HighSpeed;
//		else PanInc = LowSpeed;
	
		ServoPan = ServoPan + PanDir*PanInc;
		if ((ServoPan>ServoMax)|(ServoPan<ServoMin)) {
			PanDir = -1 * PanDir;
			if (ServoPan > ServoMax) ServoPan = ServoMax;
			if (ServoPan < ServoMin) ServoPan = ServoMin;
			PkAmp = -1000;
			HeadAngle[0] = 0.;
			HeadAmp[0] = -1000.;	// some impossible amplitude
			ACQindex = 0;
			ACQCount256uS = 0;		// should be the last two statements since it
			FirstRun = 1;		// starts the process over again
		}
		ServoSet1 = deg2servo(ServoPan);		// (var_mS + 1mS) x counts/mS
		ServoSet2 = ServoSet1;
		ServoSet3 = ServoSet1;
		
			// adding 18mS shifts this to a negative going pulse of 1-2mS
			// useful when output is being inverted/buffered perhaps
			// adding 1mS instead of 18 makes for a normal positive going PWM
	}	// end STATE1, servo movement

// STATE2:start ultrasonic pulse sequence at StartPulse
	if ((ACQCount256uS == StartPulse) & (!FirstRun))  {
		FirstRun = 1;
		BunchCount = MaxBunch;		// start bunches of pulses
	}	// end STATE2, the starting of the US pulses

// STATE3: start ADC when counter reaches StartADC
	if ((ACQCount256uS == StartADC) & (FirstRun)) {
		FirstRun = 0;
		TMR3 = 0;
		T3CONbits.TON = 1;			// start ADC
	}	// end STATE3 starting of the ADC 

// STATE4: when the ADC buffer is full 
	if ((ACQCount256uS > StartADC) & (ADCDone)) {
		ACQindex = ACQindex + 1;
		ADCDone = 0;
		// shift the results buffer stack
		for (i=0;i<HISTORYLEN;i++) {
			EchoAmp[i] = EchoAmp[i+1];
			EchoInches[i] = EchoInches[i+1];
		}

		// get the Amplitude and distance for the last acquisition
		ExtractData(EchoAmp, EchoInches, HISTORYLEN-1);	// Puts results at end of array
		LoopCount = LoopCount + 1;
		if (LoopCount>StopCount) {
			LoopCount = 0;
		}
		EchoPrevAvg=(EchoAmp[HISTORYLEN-3]+EchoAmp[HISTORYLEN-4])/2.;
		EchoNextAvg=(EchoAmp[HISTORYLEN-1]+EchoAmp[HISTORYLEN-2])/2.;
//		EchoInches[HISTORYLEN-1]=(EchoPrevAvg-EchoNextAvg)/(EchoPrevAvg+EchoNextAvg);

		// capture highest amplitude value and position it occured at
		if (EchoNextAvg > PkAmp) {
			PkAmp = EchoNextAvg;
			PkInches = EchoInches[HISTORYLEN-1];
			PkIndex = ACQindex;					// this index is 0 when direction reversed
			PkAngle = ServoPan;
		}	// end lockin peak amplitude
/*
		// now follow peak signal
//		if ((EchoAmp[HISTORYLEN-1]<EchoAmp[HISTORYLEN-2]) & (ACQindex>(PkIndex+PkOverShoot)) & (PkAmp>50)) {
		// follow positive gradient 
			if ((PkAmp>30) & (EchoNextAvg<0.95*PkAmp)) {
				for (i=0;i<19;i++) {
					HeadAngle[19-i] = HeadAngle[18-i];
					HeadAmp[19-i] = HeadAmp[18-i];
				}
				HeadAngle[0] = PkAngle;
				HeadAmp[0] = EchoAmp[HISTORYLEN-1];
//				HeadAmp[0] = (EchoAmp[HISTORYLEN-1]+EchoAmp[HISTORYLEN-2])/2.;
				PanDir = -1*PanDir;
//				PanInc = 
				PkAmp = -1000;
				ACQindex = 0;
			}	// end after switching direction
//		}	end following gradient
*/
		// haven't found a peak, so keep looking...
		ACQCount256uS = 0;
		FirstRun = 1;
		
		LATBbits.LATB3 ^= 1;		// RB3, pin7 flash LED

	}	// end STATE, when ADC buffer has been filled

	}		//  end ADC processing
return 0;

/*			ServoPan2 = ServoPan2 + Servoinc2;
			if ((ServoPan2>1)|(ServoPan2<0.00001)) Servoinc2 = -1 * Servoinc2;
			// adding 18mS shifts this to a negative going pulse of 1-2mS
			// useful when output is being inverted/buffered perhaps
			// adding 1mS instead of 18 makes for a normal positive going PWM
			ServoSet2 = (ServoPan2 + 1) * ServoCon; // (var_mS + 1mS) x counts/mS
*/
}
