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

#ifndef SAM_HH_
#define SAM_HH_

#define ATMEL_USB_EP(n) (1 << n)
#define ATMEL_USB_EP0   ATMEL_USB_EP(0)
#define ATMEL_USB_EP1   ATMEL_USB_EP(1)
#define ATMEL_USB_EP2   ATMEL_USB_EP(2)
#define ATMEL_USB_EP3   ATMEL_USB_EP(3)
#define ATMEL_USB_EP4   ATMEL_USB_EP(4)
#define ATMEL_USB_EP5   ATMEL_USB_EP(5)
#define ATMEL_USB_EP6   ATMEL_USB_EP(6)
#define ATMEL_USB_EP7   ATMEL_USB_EP(7)

#ifdef __cplusplus
extern "C" {
#endif
	void sam_usb_reset_endpoint(const unsigned int ep);
	void sam_usb_set_ep_in_dir(const unsigned int ep);
	void sam_usb_set_ep_out_dir(const unsigned int ep);
	void sam_usb_wait_for_tx_ready(const unsigned int ep);
	void sam_usb_set_tx_ready(const unsigned int ep);
	void sam_usb_load_from_fifo(const unsigned int ep, char * const data,
				    const unsigned int length);
	void sam_usb_load_to_fifo(const unsigned int ep, char * const data,
				  const unsigned int length);
	bool sam_usb_tx_completed(const unsigned int ep);
	void sam_usb_ack_tx_completed(const unsigned int ep);
	void sam_usb_ack_rx_completed(const unsigned int ep, const bool bank);
	bool sam_usb_setup_rxd(const unsigned int ep);
	bool sam_usb_out_rxd(const unsigned int ep);
	void sam_usb_ack_setup_completed(const unsigned int ep);
	void sam_usb_stall_ep(const unsigned int ep);
	bool sam_usb_ep_stalled(const unsigned int ep);
	void sam_usb_clear_ep_stall(const unsigned int ep);
	unsigned int sam_usb_out_token_length(const unsigned int ep);
	unsigned int sam_usb_setup_token_length(const unsigned int ep);
	void sam_usb_set_address(const unsigned int address);
	void sam_usb_configure_endpoint(const unsigned int ep, const bool dir,
					const unsigned char type, const bool enabled);
	void sam_usb_set_configured(const bool configured);
	void sam_usb_init(const unsigned int enabled_eps);
	void sam_usb_enter_default_state(const unsigned int endpoints);
	void sam_usb_enable();
#ifdef __cplusplus
}
#endif

#endif
