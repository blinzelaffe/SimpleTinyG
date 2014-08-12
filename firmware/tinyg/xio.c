/*
 * xio.c - Xmega IO devices - common code file
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart Jr.
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
/* ----- XIO - Xmega Device System ----
 *
 * XIO provides common access to native and derived xmega devices (see table below)
 * XIO devices are compatible with avr-gcc stdio and also provide some special functions
 * that are not found in stdio.
 *
 * Stdio support:
 *	- http://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
 * 	- Stdio compatible putc() and getc() functions provided for each device
 *	- This enables fgets, printf, scanf, and other stdio functions
 * 	- Full support for formatted printing is provided (including floats)
 * 	- Assignment of a default device to stdin, stdout & stderr is provided
 *	- printf() and printf_P() send to stdout, so use fprintf() to stderr
 *		for things that should't go over RS485 in SLAVE mode
 *
 * Facilities provided beyond stdio:
 *	- Supported devices include:
 *		- USB (derived from USART)
 *		- RS485 (derived from USART)
 *		- SPI devices and slave channels
 *		- Program memory "files" (read only)
 *	- Stdio FILE streams are managed as bindings to the above devices
 *	- Additional functions provided include:
 *		- open() - initialize parameters, addresses and flags
 *		- gets() - non-blocking input line reader - extends fgets
 *		- ctrl() - ioctl-like knockoff for setting device parameters (flags)
 *		- signal handling: interrupt on: feedhold, cycle_start, ctrl-x software reset
 *		- interrupt buffered RX and TX functions
 *		- XON/XOFF software flow control
 */
/* ----- XIO - Some Internals ----
 *
 * XIO layers are: (1) xio virtual device (root), (2) xio device type, (3) xio devices
 *
 * The virtual device has the following methods:
 *	xio_init() - initialize the entire xio system
 *	xio_open() - open a device indicated by the XIO_DEV number
 *	xio_ctrl() - set control flags for XIO_DEV device
 *	xio_gets() - get a string from the XIO_DEV device (non blocking line reader)
 *	xio_getc() - read a character from the XIO_DEV device (not stdio compatible)
 *	xio_putc() - write a character to the XIO_DEV device (not stdio compatible)
 *  xio_set_baud() - set baud rates for devices for which this is meaningful
 *
 * The device type layer currently knows about USARTS, SPI, and File devices. Methods are:
 *	xio_init_<type>() - initializes the devices of that type
 *
 * The device layer currently supports: USB, RS485, SPI channels, PGM file reading. methods:
 *	xio_open<device>() - set up the device for use or reset the device
 *	xio_ctrl<device>() - change device flag controls
 *	xio_gets<device>() - get a string from the device (non-blocking)
 *	xio_getc<device>() - read a character from the device (stdio compatible)
 *	xio_putc<device>() - write a character to the device (stdio compatible)
 *
 * The virtual level uses XIO_DEV_xxx numeric device IDs for reference.
 * Lower layers are called using the device structure pointer xioDev_t *d
 * The stdio compatible functions use pointers to the stdio FILE structs.
 */
#include <string.h>					// for memset()
#include <stdio.h>					// precursor for xio.h
#include <avr/pgmspace.h>			// precursor for xio.h

#include "xio.h"					// all device includes are nested here
#include "tinyg.h"					// needed by init() for default source
#include "config.h"					// needed by init() for default source
#include "controller.h"				// needed by init() for default source

#ifndef MAX_ULONG
#define MAX_ULONG (4294967295)
#endif

typedef struct readlineSlot {		// input buffer slots
	uint8_t status;					// status of slot
	uint32_t seqnum;				// sequence number of slot
	char_t buf[READLINE_SLOT_SIZE];	// allocated buffer for slot
} slot_t;

typedef struct xioSingleton {
	magic_t magic_start;
	FILE * stderr_shadow;			// used for stack overflow / memory integrity checking

	// readline sliding window
	uint8_t slots_free;
	uint32_t next_seqnum;
	slot_t slot[READLINE_SLOTS];

	magic_t magic_end;
} xioSingleton_t;
xioSingleton_t xio;

/********************************************************************************
 * XIO Initializations, Resets and Assertions
 */
/*
 * xio_init() - initialize entire xio sub-system
 */
void xio_init()
{
	// set memory integrity check
//	xio_set_stderr(0);				// set a bogus value; may be overwritten with a real value

	memset(&xio, 0, sizeof(xioSingleton_t));	// clear all values

	xio.slots_free = READLINE_SLOTS;

	// setup device types
	xio_init_usart();
	xio_init_spi();
	xio_init_file();

	// open individual devices (file device opens occur at time-of-use)
	xio_open(XIO_DEV_USB,  0, USB_FLAGS);
	xio_open(XIO_DEV_RS485,0, RS485_FLAGS);
	xio_open(XIO_DEV_SPI1, 0, SPI_FLAGS);
	xio_open(XIO_DEV_SPI2, 0, SPI_FLAGS);

	xio_init_assertions();
}

/*
 * xio_init_assertions()
 * xio_test_assertions() - validate operating state
 *
 * NOTE: xio device assertions are set up as part of xio_open_generic()
 *		 This system is kind of brittle right now because if a device is
 *		 not set up then it will fail in the assertions test. Need to fix this.
 */

void xio_init_assertions()
{
	xio.magic_start = MAGICNUM;
	xio.magic_end = MAGICNUM;
}

uint8_t xio_test_assertions()
{
	if (xio.magic_start					!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (xio.magic_end					!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_USB].magic_start		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_USB].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_RS485].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_RS485].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI1].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI1].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI2].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI2].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
//	if (ds[XIO_DEV_PGM].magic_start		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
//	if (ds[XIO_DEV_PGM].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (stderr != xio.stderr_shadow) 				 return (STAT_XIO_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * xio_isbusy() - return TRUE if XIO sub-system is busy
 *
 *	This function is here so that the caller can detect that the serial system is active
 *	and therefore generating interrupts. This is a hack for the earlier AVRs that require
 *	interrupts to be disabled for EEPROM write so the caller can see if the XIO system is
 *	quiescent. This is used by the G10 deferred writeback persistence functions.
 *
 *	Idle conditions:
 *	- The serial RX buffer is empty, indicating with some probability that data is not being sent
 *	- The serial TX buffers are empty
 */

uint8_t xio_isbusy()
{
	if (xio_get_rx_bufcount_usart(&USBu) != 0) return (false);
	if (xio_get_tx_bufcount_usart(&USBu) != 0) return (false);
	return (true);
}

/*
 * xio_reset_working_flags()
 */

void xio_reset_working_flags(xioDev_t *d)
{
	d->signal = 0;
	d->flag_in_line = 0;
	d->flag_eol = 0;
	d->flag_eof = 0;
}

/*
 * xio_init_device() - generic initialization function for any device
 *
 *	This binds the main fucntions and sets up the stdio FILE structure
 *	udata is used to point back to the device struct so it can be gotten
 *	from getc() and putc() functions.
 *
 *	Requires device open() to be run prior to using the device
 */
void xio_open_generic(uint8_t dev, x_open_t x_open,
								   x_ctrl_t x_ctrl,
								   x_gets_t x_gets,
								   x_getc_t x_getc,
								   x_putc_t x_putc,
								   x_flow_t x_flow)
{
	xioDev_t *d = &ds[dev];
	memset (d, 0, sizeof(xioDev_t));
	d->magic_start = MAGICNUM;
	d->magic_end = MAGICNUM;
	d->dev = dev;

	// bind functions to device structure
	d->x_open = x_open;
	d->x_ctrl = x_ctrl;
	d->x_gets = x_gets;
	d->x_getc = x_getc;	// you don't need to bind getc & putc unless you are going to use them directly
	d->x_putc = x_putc;	// they are bound into the fdev stream struct
	d->x_flow = x_flow;

	// setup the stdio FILE struct and link udata back to the device struct
	fdev_setup_stream(&d->file, x_putc, x_getc, _FDEV_SETUP_RW);
	fdev_set_udata(&d->file, d);		// reference yourself for udata
}

/********************************************************************************
 * PUBLIC ENTRY POINTS - access the functions via the XIO_DEV number
 * xio_open() - open function
 * xio_gets() - entry point for non-blocking get line function
 * xio_getc() - entry point for getc (not stdio compatible)
 * xio_putc() - entry point for putc (not stdio compatible)
 *
 * It might be prudent to run an assertion such as below, but we trust the callers:
 * 	if (dev < XIO_DEV_COUNT) blah blah blah
 *	else  return (_FDEV_ERR);	// XIO_NO_SUCH_DEVICE
 */
FILE *xio_open(uint8_t dev, const char *addr, flags_t flags)
{
	return (ds[dev].x_open(dev, addr, flags));
}

int xio_gets(const uint8_t dev, char *buf, const int size)
{
	return (ds[dev].x_gets(&ds[dev], buf, size));
}

int xio_getc(const uint8_t dev)
{
	return (ds[dev].x_getc(&ds[dev].file));
}

int xio_putc(const uint8_t dev, const char c)
{
	return (ds[dev].x_putc(c, &ds[dev].file));
}

/*
 * xio_ctrl() - PUBLIC set control flags (top-level XIO_DEV access)
 * xio_ctrl_generic() - PRIVATE but generic set-control-flags
 */
int xio_ctrl(const uint8_t dev, const flags_t flags)
{
	return (xio_ctrl_generic(&ds[dev], flags));
}

#define SETFLAG(t,f) if ((flags & t) != 0) { d->f = true; }
#define CLRFLAG(t,f) if ((flags & t) != 0) { d->f = false; }

int xio_ctrl_generic(xioDev_t *d, const flags_t flags)
{
	SETFLAG(XIO_BLOCK,		flag_block);
	CLRFLAG(XIO_NOBLOCK,	flag_block);
	SETFLAG(XIO_XOFF,		flag_xoff);
	CLRFLAG(XIO_NOXOFF,		flag_xoff);
	SETFLAG(XIO_ECHO,		flag_echo);
	CLRFLAG(XIO_NOECHO,		flag_echo);
	SETFLAG(XIO_CRLF,		flag_crlf);
	CLRFLAG(XIO_NOCRLF,		flag_crlf);
	SETFLAG(XIO_IGNORECR,	flag_ignorecr);
	CLRFLAG(XIO_NOIGNORECR,	flag_ignorecr);
	SETFLAG(XIO_IGNORELF,	flag_ignorelf);
	CLRFLAG(XIO_NOIGNORELF,	flag_ignorelf);
	SETFLAG(XIO_LINEMODE,	flag_linemode);
	CLRFLAG(XIO_NOLINEMODE,	flag_linemode);
	return (XIO_OK);
}

/*
 * xio_set_baud() - PUBLIC entry to set baud rate
 *	Currently this only works on USART devices
 */
int xio_set_baud(const uint8_t dev, const uint8_t baud)
{
	xioUsart_t *dx = (xioUsart_t *)&us[dev - XIO_DEV_USART_OFFSET];
	xio_set_baud_usart(dx, baud);
	return (XIO_OK);
}

/*
 * xio_fc_null() - flow control null function
 */
void xio_fc_null(xioDev_t *d)
{
	return;
}

/*
 * xio_set_stdin()  - set stdin from device number
 * xio_set_stdout() - set stdout from device number
 * xio_set_stderr() - set stderr from device number
 *
 *	stderr is defined in stdio as __iob[2]. Turns out stderr is the last RAM
 *	allocated by the linker for this project. We use that to keep a shadow
 *	of __iob[2] for stack overflow detection and other memory corruption.
 */
void xio_set_stdin(const uint8_t dev) { stdin  = &ds[dev].file; }
void xio_set_stdout(const uint8_t dev) { stdout = &ds[dev].file; }
void xio_set_stderr(const uint8_t dev)
{
	stderr = &ds[dev].file;
	xio.stderr_shadow = stderr;		// this is the last thing in RAM, so we use it as a memory corruption canary
}

/*
 * readline() - sliding window read reader
 *
 *	Reads characters from one or more input devices and attempts to assemble a completed
 *	line ready for execution by the controller. Uses an array of input slots to implement a
 *	sliding window protocol - i.e. buffers multiple input lines. Accepts CR or LF as line
 *	terminator. Replaces CR or LF with NUL in the returned string.
 *
 *	This function reads characters from a device (e.g. USB) into the array of input slots
 *	and returns the next terminated buffer according to the rules below.
 *
 *	Single Device Reads (reads from USB port only)
 *	 This case shows reading both ctrl and data from the USB device
 *	  - Step 1) Read all data:
 *		- Read from the USB's RX queue into the currently loading slot buffer. Keep reading
 *		  and filling up slot buffers until the USB device has no more characters or there
 *		  are no more slots to load.
 *		- As lines are read  parse and label them as a control or data line.
 *		- Mark stored lines with an incrementing sequence number.
 *	  - Step 2) When done reading:
 *		- Return the control line with the lowest sequence number
 *		- If there are no control lines return the data line with the lowest sequence number
 *		- Return with no data if there are no pending control or data buffers
 *
 *	Multiple Device Reads (USB port and FLASH or other mass storage device)
 *	 In this case the USB port is treated as ctrl and the mass storage port is treated as data
 *	  - Step 1) Read from USB:
 *		- Read from the USB's RX queue into the currently loading slot buffer. Keep reading
 *		  and filling up slot buffers until the USB device has no more characters or there
 *		  are no more slots to load.
 *		- As lines are read  parse them as a control or data line. Discard data lines, possibly
 *		  with some kind of warning.
 *		- Mark stored lines with an incrementing sequence number.
 *	  - Step 2) When done reading:
 *		- Return the control line with the lowest sequence number
 *		- If there are no control lines read and return a line from the data device
 *		- Return with no data and an EOF flag if there are no pending control or data buffers
 *		- When EOF is encountered revert to Single Device mode (USB only)
 *
 *	ARGS:
 *
 *	 flags - Bitfield used as an input argument and as a return indicating the type of line read.
 *			 Set to DEV_IS_BOTH on input to read from the USB device for control and data
 *		     (Single Device read). Set to DEV_IS_CTRL for multiple device read. In this case
 *			 will read the current mass storage device from it's current read pointer.
 *
 *			 Returns the type of line returned - DEV_IS_CTRL, DEV_IS_DATA, or 0 (DEV_FLAGS_CLEAR)
 *			 if no line is returned.
 *
 *   size -  Does nothing. Returns zero.
 *			 // size of the completed buffer, including the NUL termination character.
 *			 // Lines may be returned truncated to the length of the slot buffer if the text
 *			 // from the device is longer than the slot buffer. The size value provided as a calling
 *			 // argument is ignored (size doesn't matter).
 *
 *	 char_t * Returns a pointer to the buffer containing the line, or NULL (*0) if no text
 *
 ***** NOTE: Only Single Device Read is currrently implemented ****
 ***** NOTE: This function assumes synchronous operation. That is, the readline() caller (controller)
 *			 will completely use the returned line before calling readline() again.
 ***** NOTE: This is rather brute force for now. If it tests out it can be optimized ****
 */

// HELPERS

int8_t _get_next_slot(int8_t s, uint8_t status)  // returns index of first slot with a given status
{
	while (s < READLINE_SLOTS) {
		if (xio.slot[s].status == status) return (s);
		s++;
	}
	return (-1);
//	for (; s<READLINE_SLOTS; s++) {
//		if (xio.slot[s].status == status) {
//			return (s);
//		}
//	}
}

int8_t _get_lowest_seqnum_slot(uint8_t status)	// returns the slot with the lowest non-zero sequence number for a given status
{
	int8_t slot = -1;
	uint32_t seqnum = MAX_ULONG;

	for (uint8_t s=0; s<READLINE_SLOTS; s++) {
		if ((xio.slot[s].seqnum != 0) && (xio.slot[s].status == status) && (xio.slot[s].seqnum < seqnum)) {
			seqnum = xio.slot[s].seqnum;
			slot = s;
		}
	}
	return (slot);
}

int8_t _discard_nul_slot(int8_t s) { // discard blank lines and return -1 if so
	if (xio.slot[s].buf[0] == NUL) {
		xio.slot[s].status = SLOT_IS_FREE;
		xio.slot[s].seqnum = 0;
		return (-1);
	}
	return (0);
}

void _mark_slot(int8_t s)	// reads slot contents and marks as CTRL or DATA, sets seqnum
{
	xio.slot[s].seqnum = ++xio.next_seqnum;					// mark w/sequence number
	if (strchr("{$?!~%Hh", xio.slot[s].buf[0]) != NULL) {	// a match indicates control
		xio.slot[s].status = SLOT_IS_CTRL;
	} else {
		xio.slot[s].status = SLOT_IS_DATA;
	}
}

char_t *_return_slot(devflags_t *flags)
{
	int8_t s;
	*flags = DEV_FLAGS_CLEAR;

	if ((s = _get_lowest_seqnum_slot(SLOT_IS_CTRL)) != -1) {
		xio.slot[s].status = SLOT_IS_PROCESSING;
		xio.slots_free++;
		*flags = DEV_IS_CTRL;
		return (xio.slot[s].buf);
	}
	if ((s = _get_lowest_seqnum_slot(SLOT_IS_DATA)) != -1) {
		xio.slot[s].status = SLOT_IS_PROCESSING;
		xio.slots_free++;
		*flags = DEV_IS_DATA;
		return (xio.slot[s].buf);
	}
	return ((char_t *)NULL);			// there was no slot to return
}

char_t *readline(devflags_t *flags, uint16_t *size)
{
	int8_t s=0;							// slot index

	// free a previously processing slot
	if ((s = _get_next_slot(0, SLOT_IS_PROCESSING)) != -1) {
		xio.slot[s].status = SLOT_IS_FREE;
		xio.slot[s].seqnum = 0;
	}

	// Look for a partially filled slot if one exists
	// NB: xio_gets_usart() can return overflowed lines, these aretruncated and terminated
	if ((s = _get_next_slot(0, SLOT_IS_FILLING)) != -1) {
		if (xio_gets_usart(&ds[XIO_DEV_USB], xio.slot[s].buf, READLINE_SLOT_SIZE) == STAT_EAGAIN) {
			return (_return_slot(flags));			// no more characters to read. Return an available slot
		}
		if (_discard_nul_slot(s) != -1) {
			_mark_slot(s);							// mark as ctrl or data & set seqnum
		}
	}

	// Now fill free slots until you run out of slots or characters
	s=0;
	while ((s = _get_next_slot(s, SLOT_IS_FREE)) != -1) {
		if (xio_gets_usart(&ds[XIO_DEV_USB], xio.slot[s].buf, READLINE_SLOT_SIZE) == STAT_EAGAIN) {
			xio.slot[s].status = SLOT_IS_FILLING;	// break out on first partially filled line
			break;
		}
		// we got a completed line
		if (_discard_nul_slot(s) == -1) continue;
		xio.slots_free--;
		_mark_slot(s);
	}
	return (_return_slot(flags));
}