
#define F_CPU 8000000UL
#define _BV(bit) (1 << (bit))
#define setBit(x,y) x |= _BV(y) //set bit
#define resetBit(x,y) x &= ~(_BV(y)) //clear bit
#define tbi(x,y) x ^= _BV(y) //toggle bit
#define is_high(x,y) ((x & _BV(y)) == _BV(y))

#include <util/delay.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


const float ADC_MAX			= 0x3FF;	// 10 bit resolution
const float PWM_RANGE		= 0xFF;		// FF ~ 250hz, 

// Prototypes

uint8_t readADC(void);
void initTimer0PWM (void);
void initADC(void);
void initTimer1(void);

int main (void) {


	// Save power, turn off digital inputs
	
	
	
	DIDR0 = (0 << AIN0D) |
	(1 << AIN1D) |
	(1 << ADC0D) |
	(1 << ADC1D) |
	(1 << ADC2D) |
	(1 << ADC3D);


	TIMSK = 0;

	// Enable outputs.
	
	DIDR0 = 0;
	
	
	setBit(DDRB, PB0);
	setBit(DDRB, PB1); // Output (LED)

	// PB3 analog input.
	
	initADC();

	initTimer0PWM();
	
	sei();

	for (;;) {
		
		sleep_mode();

	}

	return (0);
	
}



ISR (ADC_vect)  {

	OCR0B = PWM_RANGE - readADC();
	
}


uint8_t readADC(void) {
	
	// 'L' must be read before 'H'
	
	uint8_t LOWORD = ADCL;
	uint16_t analogRaw = ADCH;
	
	analogRaw <<=8;

	analogRaw |= LOWORD;
	
	
	analogRaw = analogRaw + (analogRaw % 2); // Smooth out (Round to 2)
	
	uint8_t pwm = (uint8_t)((analogRaw / (float)ADC_MAX) * PWM_RANGE);
	

	
	
	//pwm = pwm - (pwm % 2);
	
	return pwm;
	
	
	
}

void initTimer0PWM (void) {
	
	
	TCCR0A =  _BV(COM0B0) | _BV(COM0B1)		// Set OC0A/OC0B on Compare Match when up-counting.
											// Clear OC0A/OC0B on Compare Match when down-counting
	| _BV(WGM00); 					// Mode: 5 - PWM, Phase Correct OCRA TOP BOTTOM
	
	
	TCCR0B = _BV(CS00) | _BV(CS01)			// clkI/O/256 (From prescaler)
	| _BV(WGM02); 							// Mode: 5 - PWM, Phase Correct OCRA TOP BOTTOM
	
	
	
	OCR0A = PWM_RANGE;  // Frequency
	OCR0B = 0x00;	// Pulse width
	
	
	
}

void initADC(void) {


	/*
	
	Clock   Available prescaler values
	---------------------------------------
	1 MHz   8 (125kHz), 16 (62.5kHz)
	4 MHz   32 (125kHz), 64 (62.5kHz)
	8 MHz   64 (125kHz), 128 (62.5kHz)
	16 MHz   128 (125kHz)

	MUX 3210
	0000 ADC0 (PB5)
	0001 ADC1 (PB2)
	0010 ADC2 (PB4)
	0011 ADC3 (PB3)

	*/

	ADMUX =
	(0 << ADLAR) |     // left shift result
	(0 << REFS1) |     // Sets ref. voltage to VCC, bit 1
	(0 << REFS0) |     // Sets ref. voltage to VCC, bit 0
	(0 << MUX3)  |     //
	(0 << MUX2)  |     //
	(1 << MUX1)  |     // PB3
	(1 << MUX0);       // PB3
	
	
	

	//ADCSRA &= ~(ADATE | ADSC); 
	
	//ADCSRB |=
	//		(0 << ADTS2) |
	//		(0 << ADTS1) |
	//		(0 << ADTS0);

	ADCSRA |=
	(1 << ADEN)  |     // Tunr off ADC - Turn it in in TIMER1_OVF_vect
	(0 << ADPS2) |     // set prescaler to 64, bit 2
	(1 << ADPS1) |     // set prescaler to 64, bit 1
	(1 << ADPS0) |      // set prescaler to 64, bit 0
	(1 << ADIF) |      // Interupt flag
	(1 << ADIE);		// Interupt enable.
	
	
	// The successive approximation circuitry requires an input clock frequency between 50 kHz and 200 kHz
	// ADPSn = cpu_clk/8 == 125 kHz
	
	// Turn off the Analog Comparator.
	
	ACSR |= ACD;
	
}



