/*
 * Tinyg_tc.c - TinyG temperature controller device
 * Part of TinyG project
 * Based on Kinen Motion Control System 
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
 *
 * The Kinen Motion Control System is licensed under the OSHW 1.0 license
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>				// for memset
#include <avr/io.h>
#include <avr/interrupt.h>

#include "kinen_core.h"
#include "tinyg_tc.h"

// static functions 
static void _controller(void);

// static data
static struct DeviceSingleton {
	double temperature_reading;
	double temperature_set_point;

	double pwm_freq;			// save it for stopping and starting PWM

	uint8_t rtc_flag;			// true = the timer interrupt fired
	uint8_t rtc_100ms_count;	// 100ms down counter
	uint8_t rtc_1sec_count;		// 1 second down counter

} dev;

static uint8_t device_array[DEVICE_ADDRESS_MAX];

// FROM MightyBoardFirmware:
// Number of bad sensor readings we need to get in a row before shutting off the heater
const uint8_t SENSOR_MAX_BAD_READINGS = 15;

// Number of temp readings to be at target value before triggering newTargetReached
// with bad seating of thermocouples, we sometimes get innacurate reads
const uint16_t TARGET_CHECK_COUNT = 5;

// If we read a temperature higher than this, shut down the heater
const uint16_t HEATER_CUTOFF_TEMPERATURE = 300;

// temperatures below setting by this amount will flag as "not heating up"
const uint16_t HEAT_FAIL_THRESHOLD = 30;

// if the starting temperature is less than this amount, we will check heating progress
// to get to this temperature, the heater has already been checked.
const uint16_t HEAT_CHECKED_THRESHOLD = 50;

// timeout for heating all the way up
const uint32_t HEAT_UP_TIME = 300000000;  //five minutes

// timeout for showing heating progress
const uint32_t HEAT_PROGRESS_TIME = 90000000; // 90 seconds

// threshold above starting temperature we check for heating progres
const uint16_t HEAT_PROGRESS_THRESHOLD = 10;


/****************************************************************************
 * main
 *
 *	Device and Kinen initialization
 *	Main loop handler
 */
int main(void)
{
	cli();						// initializations
	kinen_init();				// do this first
	device_init();				// handles all the device inits
	sei(); 						// enable interrupts

	DEVICE_UNITS;				// uncomment __UNIT_TEST_DEVICE to enable unit tests

	while (true) {				// go to the controller loop and never return
		_controller();
	}
	return (false);				// never returns
}

/*
 * Dispatch loop
 *
 *	The dispatch loop is a set of pre-registered callbacks that (in effect) 
 *	provide rudimentry multi-threading. Functions are organized from highest
 *	priority to lowest priority. Each called function must return a status code
 *	(see kinen_core.h). If SC_EAGAIN (02) is returned the loop restarts at the
 *	start of the list. For any other status code exceution continues down the list
 */

#define	DISPATCH(func) if (func == SC_EAGAIN) return; 
static void _controller()
{
	DISPATCH(kinen_callback());	// intercept low-level communication events
	DISPATCH(rtc_callback());	// real-time clock handler
	DISPATCH(pid_controller());	// main controller task
}

/**** PID Controller Functions *************************/
/*
 * pid_controller()
 */

uint8_t pid_controller()
{
	dev.temperature_set_point = 512;
	dev.temperature_reading = (double)adc_read(ADC_CHANNEL);

	pwm_set_duty(dev.temperature_reading / 10.1);

	if (dev.temperature_reading > dev.temperature_set_point) {
		led_on();
	} else {
		led_off();
	}
	return (SC_OK);
}

/**** Device Init ****
 */
void device_init(void)
{
	DDRB = PORTB_DIR;			// initialize all ports for proper IO function
	DDRC = PORTC_DIR;
	DDRD = PORTD_DIR;

	rtc_init();
	pwm_init();
	adc_init();
	led_on();					// put on the red light [Sting, 1978]

	pwm_set_freq(PWM_FREQUENCY);
}

/**** ADC - Analog to Digital Converter for thermocouple reader ****/
/*
 * adc_init() - initialize ADC. See tinyg_tc.h for settings used
 */
void adc_init(void)
{
	ADMUX  = (ADC_REFS | ADC_CHANNEL);	 // setup ADC Vref and channel 0
	ADCSRA = (ADC_ENABLE | ADC_PRESCALE);// Enable ADC (bit 7) & set prescaler
}

double adc_read(uint8_t channel)
{
	ADMUX &= 0xF0;						// clobber the channel
	ADMUX |= 0x0F & channel;			// set the channel

	ADCSRA |= ADC_START_CONVERSION;		// start the conversion
	while (ADCSRA && (1<<ADIF) == 0);	// wait about 100 uSec
	ADCSRA |= (1<<ADIF);				// clear the conversion flag
	return ((double)ADC);
}

/**** PWM - Pulse Width Modulation Functions ****/
/*
 * pwm_init() - initialize RTC timers and data
 *
 * 	Configure timer 2 for extruder heater PWM
 *	Mode: 8 bit Fast PWM Fast w/OCR2A setting PWM freq (TOP value)
 *		  and OCR2B setting the duty cycle as a fraction of OCR2A seeting
 */
void pwm_init(void)
{
	TCCR2A  = PWM_INVERTED;		// alternative is PWM_NON_INVERTED
	TCCR2A |= 0b00000011;		// Waveform generation set to MODE 7 - here...
	TCCR2B  = 0b00001000;		// ...continued here
	TCCR2B |= PWM_PRESCALE_SET;	// set clock and prescaler
	TIMSK1 = 0; 				// disable PWM interrupts
	OCR2A = 0;					// clear PWM frequency (TOP value)
	OCR2B = 0;					// clear PWM duty cycle as % of TOP value
	dev.pwm_freq = 0;
}

/*
 * pwm_set_freq() - set PWM channel frequency
 *
 *	At current settings the range is from about 500 Hz to about 6000 Hz  
 */

uint8_t pwm_set_freq(double freq)
{
	dev.pwm_freq = F_CPU / PWM_PRESCALE / freq;
	if (dev.pwm_freq < PWM_MIN_RES) { 
		OCR2A = PWM_MIN_RES;
	} else if (dev.pwm_freq >= PWM_MAX_RES) { 
		OCR2A = PWM_MAX_RES;
	} else { 
		OCR2A = (uint8_t)dev.pwm_freq;
	}
	return (SC_OK);
}

/*
 * pwm_set_duty() - set PWM channel duty cycle 
 *
 *	Setting duty cycle between 0 and 100 enables PWM channel
 *	Setting duty cycle to 0 disables the PWM channel with output low
 *	Setting duty cycle to 100 disables the PWM channel with output high
 *
 *	The frequency must have been set previously.
 *
 *	Since I can't seem to get the output pin to work in non-inverted mode
 *	it's done in software in this routine.
 */

uint8_t pwm_set_duty(double duty)
{
	if (duty <= 0) { 
		OCR2B = 255;
	} else if (duty > 100) { 
		OCR2B = 0;
	} else {
		OCR2B = (uint8_t)(OCR2A * (1-(duty/100)));
	}
	OCR2A = (uint8_t)dev.pwm_freq;
	return (SC_OK);
}

/**** RTC - Real Time Clock Functions ****
 * rtc_init() 	  - initialize RTC timers and data
 * ISR()		  - RTC interrupt routine 
 * rtc_callback() - run RTC from dispatch loop
 * rtc_10ms()	  - tasks that run every 10 ms
 * rtc_100ms()	  - tasks that run every 100 ms
 * rtc_1sec()	  - tasks that run every 100 ms
 */
void rtc_init(void)
{
	TCCR0A = 0x00;				// normal mode, no compare values
	TCCR0B = 0x05;				// normal mode, internal clock / 1024 ~= 7800 Hz
	TCNT0 = (256 - RTC_10MS_COUNT);	// set timer for approx 10 ms overflow
	TIMSK0 = (1<<TOIE0);		// enable overflow interrupts
	dev.rtc_100ms_count = 10;
	dev.rtc_1sec_count = 10;	
}

ISR(TIMER0_OVF_vect)
{
	TCNT0 = (256 - RTC_10MS_COUNT);	// reset timer for approx 10 ms overflow
	dev.rtc_flag = true;
}

uint8_t rtc_callback(void)
{
	if (dev.rtc_flag == false) { return (SC_NOOP);}
	dev.rtc_flag = false;

	rtc_10ms();

	if (--dev.rtc_100ms_count != 0) { return (SC_OK);}
	dev.rtc_100ms_count = 10;
	rtc_100ms();

	if (--dev.rtc_1sec_count != 0) { return (SC_OK);}
	dev.rtc_1sec_count = 10;
	rtc_1sec();

	return (SC_OK);
}

void rtc_10ms(void)
{
	return;
}

void rtc_100ms(void)
{
	return;
}

void rtc_1sec(void)
{
//	led_toggle();
	return;
}

/**** LED Functions ****
 * led_on()
 * led_off()
 * led_toggle()
 */

void led_on(void) 
{
	LED_PORT &= ~(LED_PIN);
}

void led_off(void) 
{
	LED_PORT |= LED_PIN;
}

void led_toggle(void) 
{
	if (LED_PORT && LED_PIN) {
		led_on();
	} else {
		led_off();
	}
}

/****************************************************************************
 *
 * Kinen Callback functions - mandatory
 *
 *	These functions are called from Kinen drivers and must be implemented 
 *	at the device level for any Kinen device
 *
 *	device_reset() 		- reset device in response tro Kinen reset command
 *	device_read_byte() 	- read a byte from Kinen channel into device structs
 *	device_write_byte() - write a byte from device to Kinen channel
 */

void device_reset(void)
{
	return;
}

uint8_t device_read_byte(uint8_t addr, uint8_t *data)
{
	addr -= KINEN_COMMON_MAX;
	if (addr >= DEVICE_ADDRESS_MAX) return (SC_INVALID_ADDRESS);
	*data = device_array[addr];
	return (SC_OK);
}

uint8_t device_write_byte(uint8_t addr, uint8_t data)
{
	addr -= KINEN_COMMON_MAX;
	if (addr >= DEVICE_ADDRESS_MAX) return (SC_INVALID_ADDRESS);
	// There are no checks in here for read-only locations
	// Assumes all locations are writable.
	device_array[addr] = data;
	return (SC_OK);
}


//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TEST_DEVICE

void device_unit_tests()
{

// PWM tests
	
	pwm_set_freq(50000);
	pwm_set_freq(10000);
	pwm_set_freq(5000);
	pwm_set_freq(2500);
	pwm_set_freq(1000);
	pwm_set_freq(500);
	pwm_set_freq(250);
	pwm_set_freq(100);

	pwm_set_freq(1000);
	pwm_set_duty(1000);
	pwm_set_duty(100);
	pwm_set_duty(99);
	pwm_set_duty(75);
/*
	pwm_set_duty(50);
	pwm_set_duty(20);
	pwm_set_duty(10);
	pwm_set_duty(5);
	pwm_set_duty(2);
	pwm_set_duty(1);
	pwm_set_duty(0.1);

	pwm_set_freq(5000);
	pwm_set_duty(1000);
	pwm_set_duty(100);
	pwm_set_duty(99);
	pwm_set_duty(75);
	pwm_set_duty(50);
	pwm_set_duty(20);
	pwm_set_duty(10);
	pwm_set_duty(5);
	pwm_set_duty(2);
	pwm_set_duty(1);
	pwm_set_duty(0.1);
*/
// exception cases

}

#endif // __UNIT_TEST_DEVICE
