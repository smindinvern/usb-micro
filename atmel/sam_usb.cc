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

#include "sam_usb.hh"
#include "sam.hh"

AtmelSAMUSBEndpointImpl::AtmelSAMUSBEndpointImpl(unsigned char epNumber,
						 USBEndpoint::ep_type epType,
						 ep_dir epDir)
    : AtmelUSBEndpointImplBase(epNumber, epType, epDir)
{
    // configure endpoint here
    reset();
    sam_usb_configure_endpoint(ep_number, dir, (unsigned char)(type), true);
}

void AtmelSAMUSBEndpointImpl::reset()
{
    sam_usb_reset_endpoint(ep_number);
    rx_ppb = 0;
}

void AtmelSAMUSBEndpointImpl::stall()
{
    sam_usb_stall_ep(ep_number);
}

void AtmelSAMUSBEndpointImpl::unstall()
{
    sam_usb_clear_ep_stall(ep_number);
}

int AtmelSAMUSBEndpointImpl::send_data(char* data, unsigned int length, bool buffered)
{
    if (type == USBEndpoint::control_ep) {
	sam_usb_set_ep_in_dir(ep_number);
	wait(5);
    }

    // wait for endpoint to become free
    sam_usb_wait_for_tx_ready(ep_number);

    sam_usb_load_to_fifo(ep_number, data, length);
	
    do_dmb();
    sam_usb_set_tx_ready(ep_number);
    wait(10);
    do_dmb();

    if (type == USBEndpoint::control_ep || !buffered) {
	/* wait for transaction to finish so our buffer doesn't go away */
	while (!sam_usb_tx_completed(ep_number)) { // wait for TXCOMP to be set
	    wait(5);
	}
	sam_usb_ack_tx_completed(ep_number);
	wait(5);
    }
    return 0;
}

char* AtmelSAMUSBEndpointImpl::read_data(unsigned int& length)
{
    length = sam_usb_out_token_length(ep_number);
    if (!length) {
	return nullptr;
    }
	
    char* data{ new(std::nothrow) char[length] };
    if (!data) {
	return nullptr;
    }

    sam_usb_load_from_fifo(ep_number, data, length);

    if (type != USBEndpoint::control_ep) {
	sam_usb_ack_rx_completed(ep_number, rx_ppb);
	rx_ppb = !rx_ppb;
    }
	
    return data;
}

char* AtmelSAMUSBEndpointImpl::read_setup(unsigned int& length)
{
    length = sam_usb_setup_token_length(ep_number);
    if (!length) {
	return nullptr;
    }

    char* data{ new(std::nothrow) char[length] };
    if (!data) {
	return nullptr;
    }

    sam_usb_load_from_fifo(ep_number, data, length);
    sam_usb_ack_setup_completed(ep_number);
	
    return data;
}
