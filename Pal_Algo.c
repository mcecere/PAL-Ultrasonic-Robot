//////////////////////////////////////////////////////
//
// Pal Algorythms
//////////////////////////////////////////////////////

#include "system.h"
//#include "p24fj64ga002.h"
#include "p33fj128gp802.h"
#include "PAL_Main.h"

extern int ADCBuffer[];
extern int PulseSep;
extern int PulseSep50uS;
extern int StartADC;

static float EchoDelay = 9;			// 
/////////////////////////////////////////////////////
//  Analyze acquired waveform for Amplitude and Delay
//  
int ExtractData(float *EAmp, float *Ein, int Count)
{
int i,j;
int Win_Width = 3;		// # pnts to avg in each window

extern float win[];

float PeakuS = 0;
float PeakValue = -1e6;

for (i=0;i<(BUFFLEN-2*PulseSep50uS);i++) {
//	win[i] = 0;
//	for (j=0;j<Win_Width;j++) {
		win[i] = ADCBuffer[i] + ADCBuffer[i+PulseSep50uS] + ADCBuffer[i+2*PulseSep50uS];
		if (win[i]>PeakValue) {
			PeakValue = win[i];
			PeakuS = i;
		}	// ends if peak value
//	}	// end averaging each point
}	// end scanning input array
EAmp[Count] = (int) (PeakValue / 3.);			// average the values
if (EAmp[Count] < 0) EAmp[Count] = 0;
Ein[Count] = (int)((PeakuS + EchoDelay)*50./SPEEDSOUND);	// 50uS per point, 74.6uS/inch
if (Ein[Count] < 0) Ein[Count] = 0;

i = 0;
i=0;

return (1);

}

