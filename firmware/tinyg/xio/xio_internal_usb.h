/*
 * xio_internal_usb.h - Common usb stack (cdc) definitions 
 * Part of SimpleTinyG (Fork of the TinyG project)
 *
 * Copyright (c) 2014 Marcus Drobisch.
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


#ifndef xio_internal_usb_h
#define xio_internal_usb_h

#define INTERNAL_USB_PORT					PORTD		// port where the USART is located
#define INTERNAL_USB_FLAGS					0x00
#define INTERNAL_USB_FREQUENCY				48000000UL
#define INTERNAL_USB_DFLL_COMPARE_VALUE		(INTERNAL_USB_FREQUENCY / 1000)

/******************************************************************************
 * USB CDC CLASS AND DEVICE FUNCTION PROTOTYPES AND ALIASES
 ******************************************************************************/
FILE *xio_open_internal_usb(const uint8_t dev, const char *addr, const flags_t flags);
int xio_gets_internal_usb(xioDev_t *d, char *buf, const int size);
int xio_getc_internal_usb(FILE *stream);
int xio_putc_internal_usb(const char c, FILE *stream);
int xio_sync_internal_usb();
void xio_init_internal_usb();

// prototypes for LUFA events
void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);



/******************************************************************************
 * STRUCTURES 
 ******************************************************************************/
/* 
 * USART extended control structure 
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable
 */
typedef struct xioUSB {
	volatile buffer_t rx_buf_tail;			// RX buffer read index
	volatile buffer_t rx_buf_head;			// RX buffer write index (written by ISR)
	volatile buffer_t rx_buf_count;			// RX buffer counter for flow control

	volatile buffer_t tx_buf_tail;			// TX buffer read index  (written by ISR)
	volatile buffer_t tx_buf_head;			// TX buffer write index
	volatile buffer_t tx_buf_count;

	PORT_t	*port;							// corresponding port

	volatile char rx_buf[RX_BUFFER_SIZE];	// (written by ISR)
	volatile char tx_buf[TX_BUFFER_SIZE];
	
	// device configuration flags
	uint8_t flag_block;
	uint8_t flag_echo;
	uint8_t flag_crlf;
	uint8_t flag_ignorecr;
	uint8_t flag_ignorelf;
	uint8_t flag_linemode;
	
} xioUSB_t;


#endif