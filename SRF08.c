////////////////////////////////////////////////////////////////////////////
//
//	SRF08 Ultrasonic rangefinder Software - Preliminary
//
//	Written by Gerald Coe - November 2001
//
// (C) Copyright Devantech Ltd 2001
//	Commercial use of this software is prohibited.
//	Private and Educational use only is permitted
//
////////////////////////////////////////////////////////////////////////////
//
// Sonar uses one of 16 addresses -> 0xe0 - 0xfe
// Bit 0 is always zero - its the i2c rd/wr bit
//
////////////////////////////////////////////////////////////////////////////
//
// This software is written for the HITECH PICC C compiler
//
////////////////////////////////////////////////////////////////////////////


#include "pic.h"

#define version	1			// software version
#define echo		RA2		// 1st stage echo line
#define led			RB4		// low to light led
#define pot_ud		RC0		// Pot up/dw control
#define pot_cs		RC1		// Pot chip select control
#define anpower	RC2		// analog power - low on
#define txpower 	RC5		// Tx power - low on
#define clamp		RC6		// comparator clamp
#define clamp_en	TRISC6	// comparator clamp enable
#define detect		RC7


// initialise the eeprom with 0xea i2c address
// the default shipping address is 0xe0, our test jig
// will change the address to 0xe0
__EEPROM_DATA (0xff, 0xff, 0xff, 0xff, 0xff, 0xea, 0xff, 0xff);


// prototypes
void setup(void);
void burst(void);
void multi_range(void);
void ann_range(void);
void set_bit(unsigned char idx);
void flash_addr(void);
void convert(unsigned char cmd, unsigned char idx);


// global variables
char buffer[36];
char loop, dlyctr;
bit timeout;
unsigned char command, index;
unsigned char gain, gaincnt;


// the interrupt
void interrupt the_only_one(void)
{
static char idx=0, wr_addr=0;
char i2c_data;

	if(SSPIF) {							// I2C interrupt
		SSPIF = 0;

		if(!STAT_DA) {					// low = address
			wr_addr=0;
		}
		if(STAT_RW) {					// high = read from this program
			SSPBUF = buffer[idx];	// send data
			if(idx<36) ++idx;			// limit index to 32 bytes
			CKP = 1;						// release I2C clock line
		}
		else {
			i2c_data = SSPBUF;		// read incoming data
			wr_addr++;
			if(wr_addr==2) {			// 1st byte written is internal location
				idx = i2c_data;		// lower 4 bits only (0-35 index)
				if(idx>35) idx=35;	// limit index
			}
			else {
				if(idx==0 && wr_addr==3) { // register 0 is start ping command
					command=i2c_data;
				}
			}
		}
		SSPOV = 0;
	}

	if(TMR1IF==1) {			// timer1 is the echo timer
		timeout = 1;			// end of echo timing when it rolls over
		TMR1ON = 0;		
		TMR1IF = 0;
	}
}



void main(void)
{
static unsigned char seq=0;

	setup();						// initialise the peripherals
	flash_addr();				// flash the I2C address on LED

	while(1) {
		while(!command);		// wait for start command

		timeout = 1;			// end of echo timing when new command arrives
		TMR1ON = 0;		
		TMR1IF = 0;

		switch(command) {
			case 0x00:			// Gain commands to limit max. gain in
			case 0x01:			// Range Mode
			case 0x02:
			case 0x03:
			case 0x04:
			case 0x05:
			case 0x06:
			case 0x07:
			case 0x08:
			case 0x09:
			case 0x0A:
			case 0x0B:
			case 0x0C:
			case 0x0D:
			case 0x0E:
			case 0x0F:
			case 0x10:
			case 0x11:
			case 0x12:
			case 0x13:
			case 0x14:
			case 0x15:
			case 0x16:
			case 0x17:
			case 0x18:
			case 0x19:
			case 0x1A:
			case 0x1B:
			case 0x1C:
			case 0x1D:
			case 0x1E:
			case 0x1F:	gain = command;
							break;
			
			case 0x80:								// inches, centimetres or uS	
			case 0x81:
			case 0x82:	multi_range();			// 2byte multi-ping data
							seq = 0;					// reset address change sequence
							break;
			case 0x83:
			case 0x84:
			case 0x85:	ann_range(); 			// 2byte 1st, 1byte multi-pings
							seq = 0;					// reset address change sequence
							break;
			case 0xa0:	seq = 1;					// start of sequence to change address
							break;
			case 0xaa:	if(seq==1) ++seq;		// 2nd of sequence to change address
							else seq = 0;
							break;
			case 0xa5:	if(seq==2) ++seq;		// 3rd of sequence to change address
							else seq = 0;
							break;
			case 0xe0:			// if seq=3 user is changing sonar I2C address
			case 0xe2:
			case 0xe4:
			case 0xe6:
			case 0xe8:
			case 0xea:
			case 0xec:
			case 0xee:
			case 0xf0:
			case 0xf2:
			case 0xf4:
			case 0xf6:
			case 0xf8:
			case 0xfa:
			case 0xfc:
			case 0xfe:	if(seq==3) {
								EEPROM_WRITE(5, command);
								SSPADD = command;
								led = 0;
							}
							seq = 0;
							break;
		}			

		command = 0;
		anpower = 1;								// analog power off
	} 
}



////////////////////////////////////////////////////////////////////////////

// The burst routine generates an acurately timed 40khz burst of 8 cycles.
// Timing assumes an 8Mhz PIC (500nS instruction rate) 
// I drop down to assembler here because I don't trust the compiler to 
// always generate accurately timed code with different versions or
// optimisation settings
//
void burst(void) {

char x;
	clamp = 0;
	clamp_en = 0;				// force low on clamp line
		
	pot_cs = 1;					// deselect pot
	led = 0;						// on
	GIE = 0;						// disable interrupts for timing accuracy
	txpower = 0;				// turn st232 on
	anpower = 0;				// turn analog power on
	loop = 8; 					// number of cycles in burst
	pot_ud = 1;					// select pot inc mode
	x = 0;	
	while(--x);					// wait for +/- 10v to charge up.
	pot_cs = 0;					// enable pot
	for(x=2; x<36; x++) {	// and take opportunity to clear echo buffer
		pot_ud = 0;				// and reset pot wiper
		buffer[x] = 0;
		pot_ud = 1;
	}
	clamp_en = 1;				// release clamp line
		
	ADGO = 1;					// convert light sensor
	pot_cs = 1;					// deselect pot
	while(ADGO);
	pot_ud = 0;					// select pot dec mode
	buffer[1] = ADRESH; 		// store light sensor reading
		
#asm
burst1:	movlw		0x14			; 1st half cycle
			movwf		_PORTB
			nop
	
			movlw		7				; (7 * 3inst * 500nS) -500nS = 10uS 
			movwf		_dlyctr		; 10uS + (5*500nS) = 12.5uS
burst2:	decfsz	_dlyctr,f
			goto		burst2
		
			movlw		0x18			; 2nd half cycle
			movwf		_PORTB
		
			movlw		6				; (6 * 3inst * 500nS) -500nS = 8.5uS 
			movwf		_dlyctr		; 8.5uS + (8*500nS) = 12.5uS
burst3:	decfsz	_dlyctr,f
			goto		burst3
			nop
			decfsz	_loop,f
			goto		burst1
		
			movlw		0x10			; set both drives low
			movwf		_PORTB
#endasm
	GIE = 1;
	txpower = 1;				// turn st232 off
	led = 1;						// Led off
	pot_cs = 0;					// enable pot
}


////////////////////////////////////////////////////////////////////////////

void multi_range(void) {

unsigned char tone_cnt, period, cmd;

	burst();				// send 40khz burst, reset pot wiper and clear buffer

	cmd = command;		// save cmd so we know how to convert result	
	TMR0 = 0;
	TMR1H = 0;
	TMR1L = 0;
	timeout = 0;
	tone_cnt = 3;
	index = 2;
	TMR1ON = 1;
	TMR2 = 0;
	TMR2IF = 0;
	gaincnt = gain;
		
	while(timeout==0) {							// while still timing stage3
		while(timeout==0 && echo==0) {		// wait for high
			if(TMR2IF && gaincnt) {
				pot_ud = 1;
				--gaincnt;
				TMR2IF = 0;
				pot_ud = 0;
			}
		}				
		while(timeout==0 && echo==1) {		// wait for low
			if(TMR2IF && gaincnt) {
				pot_ud = 1;
				--gaincnt;
				TMR2IF = 0;
				pot_ud = 0;
			}
		}				
		
		if(timeout==0) {
			period = TMR0;
			TMR0 = 0;
			if(period>40 && period<60) {
				if(!(--tone_cnt)) {
					do {
						buffer[index] = TMR1H;
						buffer[index+1] = TMR1L;
					}while(buffer[index] != TMR1H);
					
					convert(cmd, index);		// convert to in, cm or uS
					if(index == 36) return;
					index += 2;
					tone_cnt = 3;
					period = 0;
					while(--period){				// delay about 5 inches of range
						if(TMR2IF && gaincnt) {
							pot_ud = 1;
							--gaincnt;
							TMR2IF = 0;
							pot_ud = 0;
						}
					}
					while(--period){
						if(TMR2IF && gaincnt) {
							pot_ud = 1;
							--gaincnt;
							TMR2IF = 0;
							pot_ud = 0;
						}
					}
					while(--period){
						if(TMR2IF && gaincnt) {
							pot_ud = 1;
							--gaincnt;
							TMR2IF = 0;
							pot_ud = 0;
						}
					}
				}
			}
			else tone_cnt=3;
		}
	}
}				

						
////////////////////////////////////////////////////////////////////


void ann_range(void) {

unsigned char tone_cnt, period, index, cmd;

	burst();				// send 40khz burst and clear buffer
	
	cmd = command;		// save cmd so we know how to convert result	
	TMR0 = 0;
	TMR1H = 0;
	TMR1L = 0;
	timeout = 0;
	tone_cnt = 3;
	index = 2;
	TMR1ON = 1;

	while(timeout==0) {							// while still timing stage3
		while(timeout==0 && echo==0) {		// wait for high
			if(TMR2IF) {
				pot_ud = 1;
				TMR2IF = 0;
				pot_ud = 0;
			}
		}				
		while(timeout==0 && echo==1) {		// wait for low
			if(TMR2IF) {
				pot_ud = 1;
				TMR2IF = 0;
				pot_ud = 0;
			}
		}				
		
		if(timeout==0) {
			period = TMR0;
			TMR0 = 0;
			if(period>40 && period<60) {
				if(!(--tone_cnt)) {
					set_bit(TMR1H);
					if(index==2) {				// only 1st echo in ann mode
						do {
							buffer[index] = TMR1H;
							buffer[index+1] = TMR1L;
						}while(buffer[index] != TMR1H);
						convert(cmd, index);		// convert to in, cm or uS
						index += 2;
					}
					tone_cnt = 1;				// to detect continuing echo
				}
			}
			else tone_cnt=3;
		}
	}
}				
				


void set_bit(unsigned char idx)
{
char pos;

	pos = idx&7;				// lower 3 bits indicate bit position
	idx = (idx>>3)+4;			// index into buffer
	switch(pos) {
		case 0:	buffer[idx] |= 0x01;
					break;
		case 1:	buffer[idx] |= 0x02;
					break;
		case 2:	buffer[idx] |= 0x04;
					break;
		case 3:	buffer[idx] |= 0x08;
					break;
		case 4:	buffer[idx] |= 0x10;
					break;
		case 5:	buffer[idx] |= 0x20;
					break;
		case 6:	buffer[idx] |= 0x40;
					break;
		case 7:	buffer[idx] |= 0x80;
					break;					
	}
}


////////////////////////////////////////////////////////////////////


void convert(unsigned char cmd, unsigned char idx)
{
unsigned int x;

	x = (buffer[idx]<<8) + buffer[idx+1];
	switch(cmd) {
		case 0x80:
		case 0x83:	x /= 148;	// convert to inches
						break;
		case 0x81:
		case 0x84:	x /= 58;		// convert to cm
	}
	buffer[idx] = x>>8;
	buffer[idx+1] = x&0xff;		// replace uS with inches, cm or uS
}



////////////////////////////////////////////////////////////////////


void setup(void)
{
//	_CONFIG(0x0d42);			// code protected, hs osc
	__CONFIG(0x3d72);			// code not protected, hs osc
	
	ADCON1 = 0x0e;				// PortA 0 is analog, rest are digital		
	ADCON0 = 0x41;				// convert ch0
	PORTC = 0xff;				// nothing powered at start
	TRISA = 0xff;				// All inputs
	TRISB = 0xc3;				// 11000011 PB7,6,1,0 are inputs, rest are outputs
	TRISC = 0x18;				// 00011000 RC3,4 are inputs
	OPTION = 0x08;				// portb pullups on, prescaler to wdt
	T1CON = 0x10;				// timer1 prescale 1:2, but not started yet
	T2CON = 0x04;				// 1:4 prescale and running
//	T2CON = 0x06;				// 1:16 prescale and running
	PR2 = 140;					// set TMR2IF every 280uS at 8MHz
	SSPSTAT = 0x80;				// slew rate disabled
	SSPCON  = 0x36;				// enable port in 7 bit slave mode
	SSPCON2 = 0x80;				// enable general call (address 0)
	SSPADD  = EEPROM_READ(5);	// address 0xE0 - 0xFE
	if(SSPADD<0xE0)
		SSPADD=0xE0;			// protection against corrupted eeprom
	else SSPADD &= 0xfe;
	buffer[0] = version;		// software revision
	SSPIE = 1;					// enable I2C interrupts
	TMR1IE = 1;					// enable timer1 interrupts
	TMR1IF = 0;
	SSPIF = 0;
	PEIE = 1;					// enable peripheal interrupts
	GIE = 1;						// enable global interrupts
	gain = 32;					// maximum gain at power-up
}


////////////////////////////////////////////////////////////////////


void flash_addr(void)
{
unsigned char count;
long delay, on, off;

	on = off = 30000;
	
	count = ((SSPADD>>1)&0x0f)+1;
	do {
		delay = on;
		on = 10000;
		led = 0;					// led on
		while(--delay) if(command) return;
		delay = off;
		off = 20000;
		led = 1;					// led off
		while(--delay) if(command) return;
	}
	while(--count);	
}



////////////////////////////////////////////////////////////////////

