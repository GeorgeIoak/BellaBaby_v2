/*
 * BellaBaby_v2.c
 *
 * Datasheet for ATtiny20: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8235-8-bit-AVR-Microcontroller-ATtiny20_Datasheet.pdf
 *
 * Pin Configuration:
 * PA0      : Use as GND for voltage divider
 * PA1/ADC1 : BatLevel, voltage divider on battery
 * PA2      : PowerGood signal from BQ24040
 * PA3      : ChargeOK signal from BQ24040
 * PA4      : RED LED anode status indicator
 * PA5      : GREEN LED anode status indicator
 * PA6      : NC
 * PA7/OC0B : AP5724 Enable pin for brightness control
 * PB0      : TPICLK, TPI Header H3-3
 * PB1      : TPIDATA, TPI Header H3-1
 * PB2      : PwrButton, pulled high, active low
 * PB3      : Reset, pulled high, TPI header H3-5
 *
 */ 

#define F_CPU 8000000 //8MHz, default /8 =>1MHz operation

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

#define LED_RED_ON       PORTA |= (1<<4) 	// Red LED, PA4
#define LED_RED_OFF      PORTA &= ~(1<<4)
#define LED_RED_SWAP     PORTA ^= (1<<4)

#define LED_GREEN_ON     PORTA |= (1<<5) 	// Green LED, PA5
#define LED_GREEN_OFF    PORTA &= ~(1<<5)
#define LED_GREEN_SWAP   PORTA ^= (1<<5)

#define LED_LIGHT_ON     PORTA |= (1<<7) 	// White LEDs, PA7
#define LED_LIGHT_OFF    PORTA &= ~(1<<7)
#define LED_LIGHT_SWAP   PORTA ^= (1<<7)

#define LED_MASK		 ((1 << PINA4) | (1 << PINA5) | (1 << PINA7))

#define KEY_PIN			 PINB
#define KEY1			 2 					// Switch, PB2

#define REPEAT_MASK   	(1<<KEY1)			// repeat: key1
#define REPEAT_START  	100        			// after 1s
#define REPEAT_NEXT   	20        			// every 200ms

#define BAUD 			9600
#define BIT_TIME_US 	(1000000/BAUD)
#define SUART_PORT 		PORTB
#define SUART_DDR  		DDRB
#define SUART_PIN  		0					// Debug Serial pin PB0
#define SUART_PIN_m 	(1<<SUART_PIN)

void suart_init(void)
{
	SUART_DDR |= SUART_PIN_m;
}

void suart_putc(char c)
{
	uint8_t i;

	// Start bit
	SUART_PORT &= ~SUART_PIN_m;
	_delay_us(BIT_TIME_US);

	// Shift out. LSB first.
	for (i = 0; i < 8; i++) {
		if (c & 1)
			SUART_PORT |= SUART_PIN_m;
		else
			SUART_PORT &= ~SUART_PIN_m;
			c >>= 1;
			_delay_us(BIT_TIME_US);
	}
	// Stop bit
	SUART_PORT |= SUART_PIN_m;
	_delay_us(BIT_TIME_US);
}

void suart_puts(char *s)
{
	while(*s)
	suart_putc(*s++);
}

void byte_to_usart_in_decimal(uint16_t val)
{
	uint8_t thousands = val / 1000;
	uint8_t hundreds = (val % 1000) / 100;
	uint8_t tens = (val % 100) / 10;
	uint8_t ones = (val % 100) % 10;
	if (thousands) {
		suart_putc(thousands + '0');
	}
	if (thousands || hundreds) {
		suart_putc(hundreds + '0');
	}
	if (hundreds || tens) {
		suart_putc(tens + '0');
	}
	suart_putc(ones + '0');
}


//Global variables
volatile unsigned char 	key_state;		// debounced & inverted key state:bit = 1: key pressed
volatile unsigned char 	key_press;		// key press detect
volatile unsigned char 	key_rpt;		// key long press and repeat
volatile uint8_t 		tick;			// Used for longer delay
volatile uint8_t 		charging;		// Charging Flag
volatile uint8_t 		asleep;			// Asleep Flag
volatile uint8_t 		gotosleep;		// Go to sleep Flag


// Init hardware.
// Function is called only once so well give compiler a hint to inline it. This saves us few bytes

static inline void init_hardware(void)
{
	// Main Clock source: Calibrated Internal 8 MHz Osc.
	CCP=0xd8;
	CLKMSR=(0<<CLKMS1) | (0<<CLKMS0);
	// Clock Prescaler division factor: 8
	CCP=0xd8;
	CLKPSR=(0<<CLKPS3) | (0<<CLKPS2) | (1<<CLKPS1) | (1<<CLKPS0);

	// Port A initialization

	// Pull-up initialization:
	// Bit7=Off Bit6=Off Bit5=Off Bit4=Off Bit3=On Bit2=On Bit1=Off Bit0=Off
	PUEA = (1<<PUEA3) | (1<<PUEA2) | (0<<PUEA1) | (0<<PUEA0);

	// PA4, PA5, PA7 as digital out
	DDRA |= (1 << PINA4) | (1 << PINA5) | (1 << PINA7) | (1 << PINA6); // PA6 is test pin
	PORTA &= ~(1 << PINA4) | ~(1 << PINA5) | ~(1 << PINA7); //Start OFF

	// Port B initialization

	// Pull-up initialization:
	// Bit3=Off Bit2=Off Bit1=Off Bit0=Off
	PUEB = (0 << PUEB3) | (0 << PUEB2) | (0 << PUEB1) | (0 << PUEB0);

	// Function: Bit3=In Bit2=In Bit1=In Bit0=In
	// DDRB &= ~(1<<KEY1); // PB2 As Input pin
	DDRB = (0 << DDB3) | (0 << DDB2) | (0 << DDB1) | (0 << DDB0);
	// State: Bit3=T Bit2=T Bit1=T Bit0=T
	PORTB = (0 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

	// Break Before Make Mode PORTA: Off
	// Break Before Make Mode PORTB: Off
	PORTCR=(0 << BBMB) | (0 << BBMA);

	//Enable PCINT0 Interrupt
	GIMSK |= (1 << PCIE0); 		// Respond to Power Good & Charge Done Changes

	//Set Interrupt on PA2 & PA3
	PCMSK0 = (1 << PCINT2) | (1 << PCINT3); // Power Good & Charge Done

} //End init_hardware

void init_button_int(void)
{
	//Enable PCTINT1 Interrupt
	GIMSK |= (1 << PCIE1);		// Respond to button press
	//Set Interrupt on PB2
	PCMSK1 = (1 << PCINT10);	//Button connected to PB2
}

void tim0_setup(void) // Timer0 setup, not used
{
	TCCR0A |= (1 << WGM01); 				// Mode 2: CTC
	TCCR0B |= (1 << CS01) | (1 << CS00); 	// 8MHz/8/64 = 15,625Hz
	OCR0A = 75; 							// ->> 10ms timer
	TIMSK |= (1 << OCIE0A); 				// enable T0 interrupt
}

void tim1_setup(void) // Timer1 setup for debounce
{
	if (asleep)
	{
		TCCR1B = (1 << CS10); 				// 8MHz/256/1 = 31,250Hz
		OCR1A = 130; 						// 312/2 - fudge->> 10ms timer
	}
	else {
		TCCR1B = (1 << CS11); 				// 8MHz/8/8 = 125kHz
		OCR1A = 612; 						// 1249/2 - fudge->> 10ms timer
	}

	TIMSK |= (1 << OCIE1A); 				// enable T1 interrupt
	TCCR1B |= (1 << WGM12); 				// Mode 4: CTC
}

void adc_setup(void) 						// Used to check Vcc level
{
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	ADMUX =  (1 << MUX3) | (1 << MUX0);

	// Set the prescaler to clock/128
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0);	// Don't enable yet to save power
}

uint16_t readVcc()
{
	uint8_t loop;
	uint16_t aveVcc = 0;
	//PORTA ^= (1 << PINA6);     			// Test signal on pin 7 of ATtiny20

	ADCSRA |= (1 << ADEN );					// Enable ADC
	_delay_us(30);							// Wait for Vref to settle
	ADCSRA |= (1 << ADSC);					// Start 1st A2D Conversion -discard it
	while (ADCSRA & (1 << ADSC));			// Wait for it to finish - blocking

	for (loop=0; loop < 10; loop++)
	{
		ADCSRA |= (1 << ADSC);				// Start A2D Conversion
		while (ADCSRA & (1 << ADSC));		// Wait for it to finish - blocking
		aveVcc = aveVcc + ADCW;
	}
	aveVcc = aveVcc / 10;					// Calculate average of 10 readings
	/*	
		Note that the ADC will not automatically be turned off when entering other sleep modes than Idle
		mode and ADC Noise Reduction mode. The user is advised to write zero to ADEN before entering such
		sleep modes to avoid excessive power consumption.
	*/
	
	ADCSRA &= ~(1<<ADEN);					// Disable ADC to save power

	return (aveVcc); 						// Vcc level in counts
}

void set_LEDs (uint16_t Vcc)
{
	if (!(charging)) 						// If not charging set RED LED for low battery
	{
		if (Vcc >= 330)						// 332 = 3.4V, ~10 counts/100mV
		{
			byte_to_usart_in_decimal(Vcc);
			byte_to_usart_in_decimal(9); 	// End of xfer flag
			LED_RED_ON; 					// Low Battery Warning
		}
		else {
			byte_to_usart_in_decimal(Vcc);
			byte_to_usart_in_decimal(7); 	// End of xfer flag
			LED_RED_OFF;
		}
	}
}

void pwm_setup (void)
{
	// Set Timer 0 prescaler to clock/8
	// 8MHz / (CLKPSR=/8) / 8 = 125kHz
	// See ATtiny20 datasheet, Table 11.9.
	TCCR0B |= (1 << CS01);

	// Set to 'PWM Phase Correct' Mode 5
	TCCR0A |= (1 << WGM00); 
	TCCR0B |= (1 << WGM02);
	
	// Clear OC0B/PA7 output on compare match, upwards counting.
	// In Mode 5, set OC0B on match when down counting
	TCCR0A |= (1 << COM0B1); 
}

void pwm_write (int val)
{
	OCR0B = val;  							// LED Control signal on PA7
}

// Service routine called by a timer 1 interrupt
void DebounceSwitch(void) {
	
	static unsigned char ct0, ct1, rpt;
	unsigned char i;

	i = key_state ^ ~KEY_PIN;			 	// Has key changed?
	ct0 = ~( ct0 & i );                 	// reset or count ct0
	ct1 = ct0 ^ (ct1 & i);               	// reset or count ct1
	i &= ct0 & ct1;                      	// count until roll over ?
	key_state ^= i;                      	// then toggle debounced state
	key_press |= key_state & i;          	// 0->1: key press detect
	
	if( (key_state & REPEAT_MASK) == 0 ) 	// check repeat function
		rpt = REPEAT_START;                	// start delay
	
	if( --rpt == 0 ){                    	// Decrease every 10ms until
		rpt = REPEAT_NEXT;                  // repeat delay
		key_rpt |= key_state & REPEAT_MASK;
	}
} // End debounceSwitch


unsigned char get_key_press( unsigned char key_mask )
{
	cli();                  				// read and clear atomic !
	key_mask &= key_press;  				// read key(s)
	key_press ^= key_mask;  				// clear key(s)
	sei();
	return key_mask;
}


unsigned char get_key_rpt( unsigned char key_mask )
{
	cli();                					// read and clear atomic !
	key_mask &= key_rpt;  					// read key(s)
	key_rpt ^= key_mask;  					// clear key(s)
	sei();
	return key_mask;
}

unsigned char get_key_short(unsigned char key_mask )
{
	cli();         // read key state and key press atomic !
	return get_key_press( ~key_state & key_mask );
}

unsigned char  get_key_long(unsigned char key_mask )
{
	return get_key_press( get_key_rpt( key_mask ));
}

void sleepy_time(void)
{
	cli();									// Disable global interrupts
	DDRA  = 0b10000000;						// All inputs except PA7
	//PUEA &= ~(1 << PUEA3) | ~(1 << PUEA2);	// Disable pull-ups to save power
	PUEA |= (1 << PUEA6) | (1 << PUEA3) | (1 << PUEA2) | (1 << PUEA1); // Enable Pull-ups on unused pins
	PORTA &= ~(LED_MASK) | ~(1 << PINA6) | ~(1 << PINA1) | ~(1 << PINA0);	// Set pins low
	//DDRA  &= ~(LED_MASK) | ~(1 << PINA6);	// Change to inputs

	PUEB = (1 << PUEB3) | (1 << PUEB2) | (1 << PUEB1) | (1 << PUEB0); // Enabling lowers power for some reason
	PORTB = (0 << PORTB3) | (1 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);	// Don't pull PB2 low it has external Pull-up
	DDRB = 0;								// All Port B to Inputs
	DIDR0 = 0xFF; 							// Disable Digital Input Buffers to conserve power
	CCP = 0xD8;								// Write signature to allow frequency change
	CLKPSR = (1 << CLKPS3);					// Set CPU Clock to /256, must be within 4 clocks of CCP

	PRR |= (1 << PRTWI) | (1 << PRSPI) | (0 << PRTIM1) | (1 << PRTIM0) | (1 << PRADC);	//Shut 'em down
	ADCSRA &= ~(1 << ADEN);					// Disable ADC to save power
	ACSRA |= (1 << ACD);					// Switch off the Analog Comparator
	asleep = 1;								// I'm going to sleep now
	init_button_int();						// Need to enable again
	tim1_setup();	
	set_sleep_mode(SLEEP_MODE_IDLE);		// 673uA power level @ F_CPU=8MHz/8
	sleep_enable();
	sei();									// Enable global interrupts
	CCP = 0xD8;								// Write signature to allow frequency change
	MCUCR |= (1 << BODS);					// Disable Brown Out Detect, but didn't lower power
	sleep_cpu();
	sleep_disable();
}

void wakeup_time(void)
{
	init_hardware();						// General I/O Configuration
	asleep = 0;
	DIDR0 = 0;								// Enable Digital Input Buffers
	PRR = (1 << PRTWI) | (1 << PRSPI) | (0 << PRTIM1) | (0 << PRTIM0) | (0 << PRADC);	// 0 Enables them
	CCP = 0xD8;								// Write signature to allow frequency change
	CLKPSR = (1 << CLKPS1) | (1 << CLKPS0);	// Set CPU Clock to /8, must be within 4 clocks of CCP
	tim1_setup();							// Used for the debounce routine
	adc_setup();							// Measure Vcc for battery check
	pwm_setup();							// LED String Brightness Control
	suart_init();							// Used for debugging
}

int main (void)
{
	wakeup_time();							// Initialize everything
	init_button_int();

	// Start with LEDs OFF
	LED_RED_OFF;							// Low Battery Indicator
	LED_GREEN_OFF;							// Charging
	LED_LIGHT_OFF;							// LED Light String
	int16_t step = 1;						// Used to cycle through brightness
	uint16_t brightness[4] = {0,85,170,255};// Won't use 0 for now
	uint16_t counter = 12000;				// Together w/ 10ms gives 1min

	OCR0B = 0;								// Controls the duty cycle
	OCR0A = 255;							// Sets the total PWM cycle	

	sei();									// Enable global interrupts
	sleepy_time();							// Start off sleeping

	while (1)
	{
		if (!asleep)						// Ignore short if asleep
		{
			if (get_key_short(1 << KEY1))
			{
				step++;
				byte_to_usart_in_decimal(step);
				byte_to_usart_in_decimal(9); 	// End of xfer flag

				if (step == 4)
				{
					//step = 0;
					step = 1;
					pwm_write(brightness[step]);
					//_delay_ms(50);				// Need time to turn OFF LEDs
					//gotosleep = 1;
				}
				else {
					pwm_write(brightness[step]);
				}
			}
		}

		if (get_key_long(1 << KEY1))
		{
			if (asleep) 					// Was I asleep?
			{
				wakeup_time();				// Time to wake up
				//LED_GREEN_ON;				// Just testing
				pwm_write(brightness[step]);// Turn on LEDs at the last level
			}
			else { 
				pwm_write(0);				// Turn OFF the LEDs
				_delay_ms(50);				// Need time to turn OFF LEDs
				gotosleep = 1;
			}
		}

		if (tick)
		{
			tick = 0;
			if(--counter == 0)				// Time to check battery level
			{
				//set_LEDs(readVcc());
				counter = 12000;			// Reset 1min timer
			}

		}

		if (gotosleep)
		{
			gotosleep = 0;
			sleepy_time();
		}
	}// End while
	return 0;
}// End main()


// Timer1 Compare match A interrupt
ISR(TIM1_COMPA_vect)						// Button Debouncer
{
	TCNT1 = 0;
	PORTA ^= (1 << PINA6);					//Toggle test pin
	DebounceSwitch();
	tick = 1;								// 10ms passed flag
}

ISR(PCINT0_vect)							// Battery Charger Response
{
	/*
	 Power Good:  PA2
	 Charge Done: PA3
	 Status Conditions:
	                			PA2  PA3  PA4  PA5
	                 	 		 PG  CHG  RED  GRN
	 Not Charging, Bat OK    	 H    H    L    L
	 Not Charging, Bat LOW    	 H    H    H    L
	 Charging        	 		 L    L    H    H
	 Charge Done     	 		 L    H    L    H
	*/

	if (!(PINA & (1 << PINA2)))				// Check if PA2 is low
	{
		LED_GREEN_ON; 						// Good power plugged in
		LED_RED_ON; 						// Indicate charging
		charging = 1;
		wakeup_time();
	}
	else
	{
		LED_GREEN_OFF; 						// Power unplugged
		charging = 0;
		sleepy_time();						// Back on battery power, time to save power
	}
	
	// Check if PA3 is high & PA2 is low => Done Charging
	if ((PINA & (1 << PINA3)) && !(PINA & (1 << PINA2)))
	{
		LED_RED_OFF;						// Charge complete
	}
}

ISR(PCINT1_vect)
{
	if (asleep)								// Were we asleep
	{
		//asleep = 0;							// You woke me up!
		//DebounceSwitch();
		//wakeup_time();
		//LED_GREEN_ON;
	}
	else {
		//DebounceSwitch();
	}
	//wakeup_time();
	//LED_RED_SWAP;					// Just testing
		
	GIMSK &= ~(1<<PCIE1); 			// Disable the interrupt so it doesn't keep flagging
  	PCMSK1 &= ~(1<<PCINT10);
}