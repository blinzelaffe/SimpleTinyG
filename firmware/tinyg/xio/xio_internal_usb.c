/*
 * xio_internal_usb.c	- internal usb stack (CDC) for xmega family
 *						- using open source LUFA libary for usb cdc stack
 * 						- works with avr-gcc stdio library
 *
 * Part of SimpleTinyG (fork of TinyG project
 *
 * Copyright (c) 2014 Marcus Drobisch
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <string.h>						// for memset
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking character writes

// include LUFA libary
#include <LUFADescriptors.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Board/Joystick.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>
	
#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

// application specific stuff that's littered into the USB handler
#include "../tinyg.h"					// needed for AXES definition
#include "../gpio.h"					// needed for XON/XOFF LED indicator
#include "../util.h"					// needed to pick up __debug defines
#include "../config.h"					// needed to write back usb baud rate
#include "../network.h"
#include "../controller.h"
#include "../canonical_machine.h"		// trapped characters communicate directly with the canonical machine


/** LUFA CDC Class driver configuration 
 *  the structure is necessary for LUFA to setup and work correctly
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
	.Config = 
	{
		.ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
		.DataINEndpoint           =	
		{
			.Address          = CDC_TX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.DataOUTEndpoint = 
		{
			.Address          = CDC_RX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.NotificationEndpoint = 
		{
			.Address          = CDC_NOTIFICATION_EPADDR,
			.Size             = CDC_NOTIFICATION_EPSIZE,
			.Banks            = 1,
		},
	},
};



struct cfgINTERNALUSB {
	x_open_t x_open;
	x_ctrl_t x_ctrl;
	x_gets_t x_gets;
	x_getc_t x_getc;
	x_putc_t x_putc;
	x_flow_t x_flow;
	PORT_t *port;			// port binding
};

static struct cfgINTERNALUSB const cfginternalusb PROGMEM = {
	xio_open_internal_usb,			// USB config record
	xio_ctrl_generic,
	xio_gets_internal_usb,
	xio_getc_internal_usb,
	xio_putc_internal_usb,
	xio_fc_null,
	&INTERNAL_USB_PORT
};


/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/


void xio_init_internal_usb(void)
{
	// theres only one usb module per device, open generic
	xio_open_generic(XIO_DEV_INTERNAL_USB,
		(x_open_t)pgm_read_word(&cfginternalusb.x_open),
		(x_ctrl_t)pgm_read_word(&cfginternalusb.x_ctrl),
		(x_gets_t)pgm_read_word(&cfginternalusb.x_gets),
		(x_getc_t)pgm_read_word(&cfginternalusb.x_getc),
		(x_putc_t)pgm_read_word(&cfginternalusb.x_putc),
		(x_flow_t)pgm_read_word(&cfginternalusb.x_flow));
}

FILE *xio_open_internal_usb(const uint8_t dev, const char *addr, const flags_t flags)
{
	xioDev_t *d = &ds[dev];							// setup device struct pointer
	d->x = &uis;								// bind extended struct to device
	xioUSB_t *dx = &uis;

	memset (dx, 0, sizeof(xioUSB_t));				// clear all values
	xio_reset_working_flags(d);
	xio_ctrl_generic(d, flags);						// setup control flags

	// setup internal RX/TX control buffers
	dx->rx_buf_head = 1;		// can't use location 0 in circular buffer
	dx->rx_buf_tail = 1;
	dx->tx_buf_head = 1;
	dx->tx_buf_tail = 1;

	Delay_MS(100);
	USB_Init();
	Delay_MS(100);
	xio_sync_internal_usb();
	Delay_MS(100);
	xio_sync_internal_usb();
	Delay_MS(100);
	

	return (&d->file);		// return FILE reference
}

static int _gets_usb_helper(xioDev_t *d, xioUSB_t *dx)
{
	char c = NUL;

	if (uis.rx_buf_head == uis.rx_buf_tail) {	// RX ISR buffer empty
		uis.rx_buf_count = 0;					// reset count for good measure
		return(XIO_BUFFER_EMPTY);				// stop reading
	}
	
	advance_buffer(dx->rx_buf_tail, RX_BUFFER_SIZE);
	dx->rx_buf_count--;
	d->x_flow(d);								// run flow control

	//	c = dx->rx_buf[dx->rx_buf_tail];			// get char from RX Q
	c = (dx->rx_buf[dx->rx_buf_tail] & 0x007F);	// get char from RX Q & mask MSB
	//if (d->flag_echo) d->x_putc(c, stdout);		// conditional echo regardless of character
	if (d->flag_echo) 
		d->x_putc(c, stdout);		// conditional echo regardless of character

	if (d->len >= d->size) {					// handle buffer overruns
		d->buf[d->size] = NUL;					// terminate line (d->size is zero based)
		d->signal = XIO_SIG_EOL;
		return (XIO_BUFFER_FULL);
	}
	if ((c == CR) || (c == LF)) {				// handle CR, LF termination
		d->buf[(d->len)++] = NUL;
		d->signal = XIO_SIG_EOL;
		d->flag_in_line = false;				// clear in-line state (reset)
		return (XIO_EOL);						// return for end-of-line
	}
	d->buf[(d->len)++] = c;						// write character to buffer
	return (XIO_EAGAIN);
}

/* 
 *	xio_gets_usart() - read a complete line from the usart device
 * _gets_helper() 	 - non-blocking character getter for gets
 *
 *	Retains line context across calls - so it can be called multiple times.
 *	Reads as many characters as it can until any of the following is true:
 *
 *	  - RX buffer is empty on entry (return XIO_EAGAIN)
 *	  - no more chars to read from RX buffer (return XIO_EAGAIN)
 *	  - read would cause output buffer overflow (return XIO_BUFFER_FULL)
 *	  - read returns complete line (returns XIO_OK)
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 *	Note: This function assumes ignore CR and ignore LF handled upstream before the RX buffer
 */
int xio_gets_internal_usb(xioDev_t *d, char *buf, const int size)
{
	xioUSB_t *dx = &uis;					// USB pointer

	if (d->flag_in_line == false) {				// first time thru initializations
		d->flag_in_line = true;					// yes, we are busy getting a line
		d->len = 0;								// zero buffer
		d->buf = buf;
		d->size = size;
		d->signal = XIO_SIG_OK;					// reset signal register
	}
	while (true) {
		switch (_gets_usb_helper(d,dx)) {
			case (XIO_BUFFER_EMPTY): 
				return (XIO_EAGAIN); // empty condition
			case (XIO_BUFFER_FULL): 
				return (XIO_BUFFER_FULL);// overrun err
			case (XIO_EOL): 
				return (XIO_OK);			  // got complete line
			case (XIO_EAGAIN): 
				break;					  // loop for next character
		}
	}
	return (XIO_OK);
}

/*
 *  xio_getc_internal_usb() - generic char reader for USART devices
 *
 *	Compatible with stdio system - may be bound to a FILE handle
 *
 *  Get next character from RX buffer.
 *  See https://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Module-Details#Notes_on_Circular_Buffers
 *  for a discussion of how the circular buffers work
 *
 *	This routine returns a single character from the RX buffer to the caller.
 *	It's typically called by fgets() and is useful for single-threaded IO cases.
 *	Cases with multiple concurrent IO streams may want to use the gets() function
 *	which is incompatible with the stdio system. 
 *
 *  Flags that affect behavior:
 *
 *  BLOCKING behaviors
 *	 	- execute blocking or non-blocking read depending on controls
 *		- return character or -1 & XIO_SIG_WOULDBLOCK if non-blocking
 *		- return character or sleep() if blocking
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 *		- Note: putc is responsible for expanding newlines to <cr><lf> if needed
 */
int xio_getc_internal_usb(FILE *stream)
{
	// these convenience pointers optimize faster than resolving the references each time
	xioDev_t *d = (xioDev_t *)stream->udata;
	xioUSB_t *dx = &uis;
	char c;

	while (dx->rx_buf_head == dx->rx_buf_tail) {	// RX ISR buffer empty
		dx->rx_buf_count = 0;						// reset count for good measure
		if (d->flag_block) {
			sleep_mode();
			} else {
			d->signal = XIO_SIG_EAGAIN;
			return(_FDEV_ERR);
		}
	}
	advance_buffer(dx->rx_buf_tail, RX_BUFFER_SIZE);
	dx->rx_buf_count--;
	d->x_flow(d);									// flow control callback
	c = (dx->rx_buf[dx->rx_buf_tail] & 0x007F);		// get char from RX buf & mask MSB

	// Triage the input character for handling. This code does not handle deletes
	if (d->flag_echo) d->x_putc(c, stdout);			// conditional echo regardless of character
	if (c > CR) return(c); 							// fast cutout for majority cases
	if ((c == CR) || (c == LF)) {
		if (d->flag_linemode) return('\n');
	}
	return(c);
}


/* 
 * xio_putc_internal_usb() - stdio compatible char writer for usart devices
 *	This routine is not needed at the class level. 
 *	See xio_putc_usb() and xio_putc_rs485() 
 */
int xio_putc_internal_usb(const char c, FILE *stream)
{
	CDC_Device_SendByte(&VirtualSerial_CDC_Interface,(uint8_t)c);	
	return (XIO_OK);
}

void xio_handle_received_internal_usb_byte(char c)
{
	if (tg.network_mode == NETWORK_MASTER) {	// forward character if you are a master
		net_forward(c);
	}
	// trap async commands - do not insert character into RX queue
	if (c == CHAR_RESET) {	 					// trap Kill signal
		tg_request_reset();
		return;
	}
	if (c == CHAR_FEEDHOLD) {					// trap feedhold signal
		cm_request_feedhold();
		return;
	}
	if (c == CHAR_QUEUE_FLUSH) {				// trap queue flush signal
		cm_request_queue_flush();
		return;
	}
	if (c == CHAR_CYCLE_START) {				// trap cycle start signal
		cm_request_cycle_start();
		return;
	}

	// filter out CRs and LFs if they are to be ignored
	if ((c == CR) && (uis.flag_ignorecr)) return;
	if ((c == LF) && (uis.flag_ignorelf)) return;

	// normal character path
	advance_buffer(uis.rx_buf_head, RX_BUFFER_SIZE);
	if (uis.rx_buf_head != uis.rx_buf_tail) {	// buffer is not full
		uis.rx_buf[uis.rx_buf_head] = c;		// write char unless full
		//CDC_Device_SendByte(&VirtualSerial_CDC_Interface,(uint8_t)c);	
		uis.rx_buf_count++;
	} else {		
		// buffer-full - toss the incoming character
		if ((++uis.rx_buf_head) > RX_BUFFER_SIZE-1) {	// reset the head
			uis.rx_buf_count = RX_BUFFER_SIZE-1;		// reset count for good measure
			uis.rx_buf_head = 1;
		}
	}
}

int xio_sync_internal_usb()
{
	int16_t i;
	int16_t received_byte;
	int16_t bytes_received = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);
	for(i=0;i< bytes_received;i++)
	{
		received_byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		if(received_byte > 0)
		{			
			//CDC_Device_SendByte(&VirtualSerial_CDC_Interface,(uint8_t)received_byte);	
			xio_handle_received_internal_usb_byte((char)received_byte);
		}
	}
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();

	return (XIO_OK);
}

/* USB Device Connect event handler. */
void EVENT_USB_Device_Connect(void)
{
	// todo
}

/* USB Device Disconnect event handler. */
void EVENT_USB_Device_Disconnect(void)
{
	// todo
}

/* USB Control Configuration event handler. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** USB Control Request event handler. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

