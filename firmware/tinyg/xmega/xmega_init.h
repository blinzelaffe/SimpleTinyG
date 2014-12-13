/*
 * xmega_init.h - general init and support functions for xmega family
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

#ifndef xmega_support_h
#define xmega_support_h

/*
 * Global Scope Functions
 */

/* Offset of member MEMBER in a struct of type TYPE. */
#define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)


enum XMEGA_System_UsbDFLLReference_t
{
	DFLL_REF_INT_RC32KHZ   = 0, /**< Reference clock sourced from the Internal 32KHz RC Oscillator clock. */
	DFLL_REF_EXT_RC32KHZ   = 1, /**< Reference clock sourced from the External 32KHz RC Oscillator clock connected to TOSC pins. */
	DFLL_REF_INT_USBSOF    = 2, /**< Reference clock sourced from the USB Start Of Frame packets. */
};


void xmega_init(void);
void CCPWrite( volatile uint8_t * address, uint8_t value );

#endif
