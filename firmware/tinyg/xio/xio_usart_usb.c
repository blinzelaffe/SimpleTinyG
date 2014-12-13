/*
 * xio_usb.c - FTDI USB device driver for xmega family
 * 			 - works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking TX

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

// application specific stuff that's littered into the USB handler
#include "../tinyg.h"
#include "../config.h"
#include "../network.h"
#include "../controller.h"
#include "../canonical_machine.h"		// trapped characters communicate directly with the canonical machine

// Fast accessors
#define UARTUSB ds[XIO_DEV_UART_USB]
#define UARTUSBu us[XIO_DEV_UART_USB - XIO_DEV_USART_OFFSET]

/*
 * xio_putc_usb() 
 * USB_TX_ISR - USB transmitter interrupt (TX) used by xio_usb_putc()
 *
 * 	These are co-routines that work in tandem.
 * 	xio_putc_usb() is a more efficient form derived from xio_putc_usart()
 * 
 *	The TX interrupt dilemma: TX interrupts occur when the USART DATA register is 
 *	empty (and the ISR must disable interrupts when nothing's left to read, or they 
 *	keep firing). If the TX buffer is completely empty (TXCIF is set) then enabling
 *	interrupts does no good. The USART won't interrupt and the TX circular buffer 
 *	never empties. So the routine that puts chars in the TX buffer must always force
 *	an interrupt.
 */

int xio_putc_usb(const char c, FILE *stream)
{
	buffer_t next_tx_buf_head = UARTUSBu.tx_buf_head-1;		// set next head while leaving current one alone
	if (next_tx_buf_head == 0)
		next_tx_buf_head = TX_BUFFER_SIZE-1; 			// detect wrap and adjust; -1 avoids off-by-one
	while (next_tx_buf_head == UARTUSBu.tx_buf_tail) 
		sleep_mode(); 									// sleep until there is space in the buffer
	UARTUSBu.usart->CTRLA = CTRLA_RXON_TXOFF;				// disable TX interrupt (mutex region)
	UARTUSBu.tx_buf_head = next_tx_buf_head;				// accept next buffer head
	UARTUSBu.tx_buf[UARTUSBu.tx_buf_head] = c;					// write char to buffer

	// expand <LF> to <LF><CR> if $ec is set
	if ((c == '\n') && (UARTUSB.flag_crlf)) {
		UARTUSBu.usart->CTRLA = CTRLA_RXON_TXON;			// force interrupt to send the queued <CR>
		buffer_t next_tx_buf_head = UARTUSBu.tx_buf_head-1;
		if (next_tx_buf_head == 0) next_tx_buf_head = TX_BUFFER_SIZE-1;
		while (next_tx_buf_head == UARTUSBu.tx_buf_tail) sleep_mode();
		UARTUSBu.usart->CTRLA = CTRLA_RXON_TXOFF;			// MUTEX region
		UARTUSBu.tx_buf_head = next_tx_buf_head;
		UARTUSBu.tx_buf[UARTUSBu.tx_buf_head] = CR;
	}
	// finish up
	UARTUSBu.usart->CTRLA = CTRLA_RXON_TXON;			// force interrupt to send char(s) - doesn't work if you just |= it
	return (XIO_OK);
}

ISR(USB_TX_ISR_vect) //ISR(USARTC0_DRE_vect)		// USARTC0 data register empty
{
	// If the CTS pin (FTDI's RTS) is HIGH, then we cannot send anything, so exit
//	if ((USBu.port->IN & USB_CTS_bm)) {
	if ((cfg.enable_flow_control == FLOW_CONTROL_RTS) && (UARTUSBu.port->IN & USB_CTS_bm)) {
		UARTUSBu.usart->CTRLA = CTRLA_RXON_TXOFF;		// force another TX interrupt
		return;
	}

	// Send an RX-side XON or XOFF character if queued
	if (UARTUSBu.fc_char_rx != NUL) {					// an XON/ of XOFF needs to be sent
		UARTUSBu.usart->DATA = UARTUSBu.fc_char_rx;			// send the XON/XOFF char and exit
		UARTUSBu.fc_char_rx = NUL;
		return;
	}

	// Halt transmission while in TX-side XOFF
	if (UARTUSBu.fc_state_tx == FC_IN_XOFF) {
		return;
	}

	// Otherwise process normal TX transmission
	if (UARTUSBu.tx_buf_head != UARTUSBu.tx_buf_tail) {		// buffer has data
		advance_buffer(UARTUSBu.tx_buf_tail, TX_BUFFER_SIZE);
		UARTUSBu.usart->DATA = UARTUSBu.tx_buf[UARTUSBu.tx_buf_tail];
	} else {
		UARTUSBu.usart->CTRLA = CTRLA_RXON_TXOFF;		// force another interrupt
	}
} 

/*
 * Pin Change (edge-detect) interrupt for CTS pin.
 */

ISR(USB_CTS_ISR_vect)	
{
	UARTUSBu.usart->CTRLA = CTRLA_RXON_TXON;		// force another interrupt
}

/* 
 * USB_RX_ISR - USB receiver interrupt (RX)
 *
 * RX buffer states can be one of:
 *	- buffer has space	(CTS should be asserted)
 *	- buffer is full 	(CTS should be not_asserted)
 *	- buffer becomes full with this character (write char and assert CTS)
 *
 * Signals:
 *	- Signals are captured at the ISR level and either dispatched or flag-set
 *	- As RX ISR is a critical code region signal handling is stupid and fast
 *	- signal characters are not put in the RX buffer
 *
 * Flow Control:
 *	- Flow control is not implemented. Need to work RTS line.
 *	- Flow control should cut off at high water mark, re-enable at low water mark
 *	- High water mark should have about 4 - 8 bytes left in buffer (~95% full) 
 *	- Low water mark about 50% full
 *
 *  See https://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Module-Details#Notes_on_Circular_Buffers
 *  for a discussion of how the circular buffers work
 */

ISR(UARTUSB_RX_ISR_vect)	//ISR(USARTC0_RXC_vect)	// serial port C0 RX int 
{
	char c = UARTUSBu.usart->DATA;					// can only read DATA once

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
	if (UARTUSB.flag_xoff) {
		if (c == XOFF) {						// trap incoming XON/XOFF signals
			UARTUSBu.fc_state_tx = FC_IN_XOFF;
			return;
		}
		if (c == XON) {
			UARTUSBu.fc_state_tx = FC_IN_XON;
			UARTUSBu.usart->CTRLA = CTRLA_RXON_TXOFF;// force a TX interrupt
			return;
		}
	}

	// filter out CRs and LFs if they are to be ignored
	if ((c == CR) && (UARTUSB.flag_ignorecr)) return;
	if ((c == LF) && (UARTUSB.flag_ignorelf)) return;

	// normal character path
	advance_buffer(UARTUSBu.rx_buf_head, RX_BUFFER_SIZE);
	if (UARTUSBu.rx_buf_head != UARTUSBu.rx_buf_tail) {	// buffer is not full
		UARTUSBu.rx_buf[UARTUSBu.rx_buf_head] = c;		// write char unless full
		UARTUSBu.rx_buf_count++;
		if ((UARTUSB.flag_xoff) && (xio_get_rx_bufcount_usart(&UARTUSBu) > XOFF_RX_HI_WATER_MARK)) {
			xio_xoff_usart(&UARTUSBu);
		}
	} else { 											// buffer-full - toss the incoming character
		if ((++UARTUSBu.rx_buf_head) > RX_BUFFER_SIZE-1) {	// reset the head
			UARTUSBu.rx_buf_count = RX_BUFFER_SIZE-1;		// reset count for good measure
			UARTUSBu.rx_buf_head = 1;
		}
	}
}

/*
 * xio_get_usb_rx_free() - returns free space in the USB RX buffer
 *
 *	Remember: The queues fill from top to bottom, w/0 being the wrap location
 */
buffer_t xio_get_usb_rx_free(void)
{
	return (RX_BUFFER_SIZE - xio_get_rx_bufcount_usart(&UARTUSBu));
}

/*
 * xio_reset_usb_rx_buffers() - clears the USB RX buffer
 */
void xio_reset_usb_rx_buffers(void)
{
	// reset xio_gets() buffer
	UARTUSB.len = 0;
	UARTUSB.flag_in_line = false;

	// reset interrupt circular buffer
	UARTUSBu.rx_buf_head = 1;		// can't use location 0 in circular buffer
	UARTUSBu.rx_buf_tail = 1;
	UARTUSBu.tx_buf_head = 1;
	UARTUSBu.tx_buf_tail = 1;
}
