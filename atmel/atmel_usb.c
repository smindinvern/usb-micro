/**
 * Copyright 2018 Nickolas T Lloyd <ultrageek.lloyd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.hh"
#include "sam.hh"

void *malloc(unsigned int);
void free(void *);
void memset(char *, int, unsigned int);
void *memcpy(void *, void *, unsigned int);

void usb_in_token(unsigned int ep);
void usb_out_token(unsigned int ep);
int usb_setup_token(unsigned int ep);

void atmel_usb_set_address(unsigned int address)
{
	sam_usb_set_address(address);
}

void atmel_usb_set_configured(_Bool configured)
{
	/* set CONFG bit in global state register */
	sam_usb_set_configured(configured);
}

/* DEPRECATED */
void atmel_usb_init_usb(void)
{
	// Configure and enable EP{1,2}
	sam_usb_init(ATMEL_USB_EP1 | ATMEL_USB_EP2);
}

void atmel_usb_ep_isr(unsigned int ep)
{
	if (sam_usb_ep_stalled(ep)) {
		sam_usb_reset_endpoint(0);
		sam_usb_clear_ep_stall(ep);
		do_dmb();
	}
	if (sam_usb_setup_rxd(ep)) { /* setup packet received */
		usb_setup_token(ep);
	}
	if (sam_usb_out_rxd(ep)) { /* OUT packet received */
		usb_out_token(ep);
	}
	if (sam_usb_tx_completed(ep)) { /* IN packet received */
		if (ep == 1)
			usb_in_token(ep);
	}
}

/* DEPRECATED */
void atmel_usb_enter_default_state(void)
{
	sam_usb_enter_default_state(ATMEL_USB_EP1 | ATMEL_USB_EP2);
}

void atmel_usb_enable(void)
{
	sam_usb_enable();
}
