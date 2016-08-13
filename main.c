//******************************************************************************
//	MSP430G2xx1 - ten independent Delta-Sigma modulators (not PWM channels),
//		one for each available pin.
//
//	Description:
//		An algorithm similar to Delta-Sigma conversion is calculating
//		for each "clock" (1 clock at each N'th WDT interrupt) the value of
//		each output bit. All bits are outputted to LED's during WDT interrupt.
//
//		The algorithm is used once for each channel, in order to calculate
//		the output value of each Delta-Sigma ADC using "synthetic division",
//		and it simulates a first-order Delta-Sigma modulator. The algorithm can
//		be found in the last two instructions of calc_output_bits() function.
//
//		All bits in P1 and P2 ports are configured as digital outputs.
//		Each bit modulates the intensity of one LED, using 0 and 1 (not PWM).
//
//		The equivalent of the analog inputs for the Delta-Sigma modulators
//		are the numbers stored in the req[] array. Values in the req[] array
//		are calculated by the calc_CH_x_TO_y() functions.
//
//		The main differences of Delta-Sigma vs. PWM are:
//			- bigger output refresh rate (switching frequency) then in PWM
//			- output frequency spectrum is more spread in Delta-Sigma
//
//	Application:
//		Four multicolour LED's, two RGB and two RG, are shifting their colour
//		with independent speed, resolution and number of steps for each LED.
//
//	Schematics: MSP430G2211 at 3.6V with
//		P1.0 - RGB_LED_1 Red bit	(100 ohm, common anode)
//		P1.1 - RGB_LED_1 Green bit	(100 ohm, common anode)
//		P1.2 - RGB_LED_1 Blue bit	( 33 ohm, common anode)
//
//		P1.3 - RGB_LED_2 Red bit	(100 ohm, common anode)
//		P1.4 - RGB_LED_2 Green bit	(100 ohm, common anode)
//		P1.5 - RGB_LED_2 Blue bit	( 33 ohm, common anode)
//
//		P1.6 - RG_LED_1 Red bit		(100 ohm, common cathode)
//		P1.7 - RG_LED_1 Green bit	(100 ohm, common cathode)
//
//		P2.6 - RG_LED_2 Red bit		(100 ohm, common cathode)
//		P2.7 - RG_LED_2 Green bit	(100 ohm, common cathode)
//
//	Bibliography:
//		Jason Sachs, Modulation Alternatives for the Software Engineer
//		http://www.embeddedrelated.com/showarticle/107.php
//
//  Author: RoGeorge
//
//  2012.06.02	v1.0
//  Built with CCS Version 4.2.4
//******************************************************************************

#include "msp430g2211.h"

//------------------------------------------------------------------------------
// Hardware related definitions
//------------------------------------------------------------------------------
#define P1_COMM_ANOD	0x3F	// Mask to negate bits for common
#define P2_COMM_ANOD	0x00	//		anode LED's

#define N_CH			10		// Number of modulator channels

#define LOOP_SPEED		6		// Calc envelope on each 2**LOOP_SPEED interrupts

//------------------------------------------------------------------------------
// Delta-Sigma related definitions	// STEPS must be <= MAX
//------------------------------------------------------------------------------
#define MAX_CH_0_2		200	// Distinct possible steps between 0-100%
#define MAX_CH_3_5		200	// Distinct possible steps between 0-100%
#define MAX_CH_6_7		100	// Distinct possible steps between 0-100%
#define MAX_CH_8_9		150	// Distinct possible steps between 0-100%

#define STEPS_CH_0_2		100		    // Nr of equidistant used steps
#define INC_CH_0_2		MAX_CH_0_2/STEPS_CH_0_2	// Increment for one step

#define STEPS_CH_3_5		100			// Nr of equidistant used steps
#define INC_CH_3_5		MAX_CH_3_5/STEPS_CH_3_5	// Increment for one step

#define STEPS_CH_6_7		50			// Nr of equidistant used steps
#define INC_CH_6_7		MAX_CH_6_7/STEPS_CH_6_7	// Increment for one step

#define STEPS_CH_8_9		150			// Nr of equidistant used steps
#define INC_CH_8_9		MAX_CH_8_9/STEPS_CH_8_9	// Increment for one step

//------------------------------------------------------------------------------
// Global variables used for each software channel of a Delta-Sigma modulator
//------------------------------------------------------------------------------
unsigned int max[N_CH];		// Maxim level (resolution) for each channel
unsigned char req[N_CH];	// Requested levels, 0 <= req <= max
unsigned int sum[N_CH];		// Integrators value, 0 <= sum < 2*sum

unsigned int outBits;		// Each bit store the output value of one modulator

//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void init_all_CH_arrays();
void calc_CH_0_TO_2();
void calc_CH_3_TO_5();
void calc_CH_6_TO_7();
void calc_CH_8_TO_9();
void calc_output_bits();



void main(void) {
//------------------------------------------------------------------------------
// main()
//------------------------------------------------------------------------------
	volatile int intCnt;			// WDT interrupt counter

	WDTCTL = WDTPW + WDTHOLD;		// Stop watchdog timer

	DCOCTL |= DCO2;					// DCO ~ 16 MHz?
	BCSCTL1 |= RSEL3;				// DCO ~ 16 MHz?

	P1OUT = 0x00;					// Initialize P1OUT
	P1DIR = 0xFF;					// Set all P1 pins as outputs

	P2OUT = 0x00;					// Initialize P2OUT
	P2SEL = 0x00;					// Set all P2 pins as outputs
	P2DIR = 0xFF;					// Set all P2 pins as outputs

	WDTCTL = WDT_MDLY_8;			// Start WDT+ in timer mode, 8ms/16
	IE1 |= WDTIE;					// Enable WDT+ interrupts

	init_all_CH_arrays();			// Initialize modulators

	__enable_interrupt();			// Global interrupt enable

	for(;;) {						// Infinite main loop
		LPM0;						// Wait for a WDT+ interrupt

		if (++intCnt & (1 << LOOP_SPEED)) {	// Color envelope calculation for
			intCnt = 0;			//   every 2**LOOP_SPEED interrupts
			calc_CH_0_TO_2();		// Calculate next RGB_LED_1 color
			calc_CH_3_TO_5();		// Calculate next RGB_LED_2 color
			calc_CH_6_TO_7();		// Calculate next RG_LED_1 color
			calc_CH_8_TO_9();		// Calculate next RG_LED_2 color
		}
		calc_output_bits();	// Calculate next values for the modulators outputs
	}
}

void init_all_CH_arrays() {
//------------------------------------------------------------------------------
// Initialize all 10 channels arrays with different periods and initial values
//------------------------------------------------------------------------------
	max[0] = MAX_CH_0_2;	// Set maxim value (resolution) for each modulator
	max[1] = MAX_CH_0_2;
	max[2] = MAX_CH_0_2;
	max[3] = MAX_CH_3_5;
	max[4] = MAX_CH_3_5;
	max[5] = MAX_CH_3_5;
	max[6] = MAX_CH_6_7;
	max[7] = MAX_CH_6_7;
	max[8] = MAX_CH_8_9;
	max[9] = MAX_CH_8_9;

	req[0] = MAX_CH_0_2;	// Initial RGB_LED_1 Red value
	req[1] = 0;				// Initial RGB_LED_1 Green value
	req[2] = 0;				// Initial RGB_LED_1 Blue value

	req[3] = 0;				// Initial RGB_LED_2 Red value
	req[4] = MAX_CH_3_5;	// Initial RGB_LED_2 Green value
	req[5] = MAX_CH_3_5;	// Initial RGB_LED_2 Blue value

	req[6] = 0;				// Initial RG_LED_1 Red value
	req[7] = 0;				// Initial RG_LED_1 Green value

	req[8] = MAX_CH_8_9;	// Initial RG_LED_2 Red value
	req[9] = MAX_CH_8_9;	// Initial RG_LED_2 Green value
}

void calc_CH_0_TO_2() {
//------------------------------------------------------------------------------
// Calculate next input values for modulators 0, 1, 2 (RGB_LED_1 color envelope)
//------------------------------------------------------------------------------
	static int stCnt = 0;		//envelope state counter

	if ((stCnt >= 0*STEPS_CH_0_2) && (stCnt < 1*STEPS_CH_0_2))
		req[1] += INC_CH_0_2;	//increase Green
	if ((stCnt >= 1*STEPS_CH_0_2) && (stCnt < 2*STEPS_CH_0_2))
		req[0] -= INC_CH_0_2;	//decrease Red
	if ((stCnt >= 2*STEPS_CH_0_2) && (stCnt < 3*STEPS_CH_0_2))
		req[2] += INC_CH_0_2;	//increase Blue
	if ((stCnt >= 3*STEPS_CH_0_2) && (stCnt < 4*STEPS_CH_0_2))
		req[1] -= INC_CH_0_2;	//decrease Green
	if ((stCnt >= 4*STEPS_CH_0_2) && (stCnt < 5*STEPS_CH_0_2))
		req[0] += INC_CH_0_2;	//increase Red
	if ((stCnt >= 5*STEPS_CH_0_2) && (stCnt < 6*STEPS_CH_0_2))
		req[2] -= INC_CH_0_2;	//decrease Blue

	if (++stCnt >= 6*STEPS_CH_0_2) stCnt = 0;    //++stCnt modulo 6*STEPS_CH_0_2
}

void calc_CH_3_TO_5() {
//------------------------------------------------------------------------------
// Calculate next input values for modulators 3, 4, 5 (RGB_LED_2 color envelope)
//------------------------------------------------------------------------------
	static int stCnt = 3*STEPS_CH_3_5;	    //envelope state counter

	if ((stCnt >= 0*STEPS_CH_3_5) && (stCnt < 1*STEPS_CH_3_5))
		req[4] += INC_CH_3_5;	//increase Green
	if ((stCnt >= 1*STEPS_CH_3_5) && (stCnt < 2*STEPS_CH_3_5))
		req[3] -= INC_CH_3_5;	//decrease Red
	if ((stCnt >= 2*STEPS_CH_3_5) && (stCnt < 3*STEPS_CH_3_5))
		req[5] += INC_CH_3_5;	//increase Blue
	if ((stCnt >= 3*STEPS_CH_3_5) && (stCnt < 4*STEPS_CH_3_5))
		req[4] -= INC_CH_3_5;	//decrease Green
	if ((stCnt >= 4*STEPS_CH_3_5) && (stCnt < 5*STEPS_CH_3_5))
		req[3] += INC_CH_3_5;	//increase Red
	if ((stCnt >= 5*STEPS_CH_3_5) && (stCnt < 6*STEPS_CH_3_5))
		req[5] -= INC_CH_3_5;	//decrease Blue

	if (++stCnt >= 6*STEPS_CH_3_5) stCnt = 0;    //++stCnt modulo 6*STEPS_CH_3_5
}

void calc_CH_6_TO_7() {
//------------------------------------------------------------------------------
// Calculate next input values for modulators 6, 7 (RG_LED_1 color envelope)
//------------------------------------------------------------------------------
	static int stCnt = 0;		            //envelope state counter

	if ((stCnt >= 0*STEPS_CH_6_7) && (stCnt < 1*STEPS_CH_6_7))
		req[7] += INC_CH_6_7;	//increase Green
	if ((stCnt >= 1*STEPS_CH_6_7) && (stCnt < 2*STEPS_CH_6_7))
		req[6] += INC_CH_6_7;	//increase Red
	if ((stCnt >= 2*STEPS_CH_6_7) && (stCnt < 3*STEPS_CH_6_7))
		req[7] -= INC_CH_6_7;	//decrease Green
	if ((stCnt >= 3*STEPS_CH_6_7) && (stCnt < 4*STEPS_CH_6_7))
		req[6] -= INC_CH_6_7;	//decrease Red

	if (++stCnt >= 4*STEPS_CH_6_7) stCnt = 0;    //++stCnt modulo 4*STEPS_CH_6_7
}

void calc_CH_8_TO_9() {
//------------------------------------------------------------------------------
// Calculate next input values for modulators 8, 9 (RG_LED_2 color envelope)
//------------------------------------------------------------------------------
	static int stCnt = 2*STEPS_CH_8_9;	    //envelope state counter

	if ((stCnt >= 0*STEPS_CH_8_9) && (stCnt < 1*STEPS_CH_8_9))
		req[9] += INC_CH_8_9;	//increase Green
	if ((stCnt >= 1*STEPS_CH_8_9) && (stCnt < 2*STEPS_CH_8_9))
		req[8] += INC_CH_8_9;	//increase Red
	if ((stCnt >= 2*STEPS_CH_8_9) && (stCnt < 3*STEPS_CH_8_9))
		req[9] -= INC_CH_8_9;	//decrease Green
	if ((stCnt >= 3*STEPS_CH_8_9) && (stCnt < 4*STEPS_CH_8_9))
		req[8] -= INC_CH_8_9;	//decrease Red
	
	if (++stCnt >= 4*STEPS_CH_8_9) stCnt = 0;    //++stCnt modulo 4*STEPS_CH_8_9
}

void calc_output_bits() {
//------------------------------------------------------------------------------
// Calculate the output bit for each Delta-Sigma modulator
//------------------------------------------------------------------------------
	int n;						// Modulator (channel) number

	for (n = N_CH - 1; n >= 0; --n) {	// For each Delta-Sigma modulator
		outBits <<= 1;			// Shift previously calculated bits
// Sigma delta modulation algorithm using "synthetic division"
		sum[n] += req[n];		// Update integrator value
		if (sum[n] < max[n])
			outBits++;			// LSB = 1
		else
			sum[n] -= max[n];	// LSB = 0 (untouched) and adjust integrator
	}
}

#pragma vector = WDT_VECTOR
__interrupt void Watchdog_Timer(void) {
//------------------------------------------------------------------------------
// Watchdog Timer ISR - Write the modulators outputs to P1 and P2 (all LED's)
//------------------------------------------------------------------------------
	P1OUT = outBits ^ P1_COMM_ANOD;			// Negate common anode LED's bits
	P2OUT = (outBits >> 2) ^ P2_COMM_ANOD;	// Negate common anode LED's bits

	_BIC_SR_IRQ(LPM0_bits);					// Clear LPM0 bits from 0(SR)
}
