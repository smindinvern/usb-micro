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
#include "ra6m1.hh"

extern "C" {
    void nmi()
    {
	return;
    }

    void hard_fault()
    {
	return;
    }

    void mem_manage_fault()
    {
	while (1);
    }

    void bus_fault()
    {
	if (1 == 1)
	    while (1);
	
	return;
    }

    void usage_fault()
    {
	while (1);
	
	return;
    }

    void svcall()
    {
	return;
    }

    void pendsv()
    {
	return;
    }

    void systick()
    {
	return;
    }

    void unused_interrupt()
    {
	return;
    }

    void usb_device_state_change()
    {
	// Clear interrupt flag
	Reg16 intsts0{ USBFS_INTSTS0 };
	intsts0 = (unsigned short)~USBFS_INTSTS0_DVST;

	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	// When an interrupt occurs, check for a reset before anything else.
	// This way we can be assured that if a reset occurs, all endpoints
	// are immediately reset without attempting to send/receive any more
	// data.
	if (UsbDeviceBusResetStatus::reset_in_progress())
	{
	    usb_status->device->reset();
	}
	else if (ra6m1_usb_get_device_state() == UsbDeviceState::ConfiguredState)
	{
	    usb_status->configured = 1;
	}
    }

    void usb_resume()
    {
	// Clear interrupt flag
	Reg16 intsts0{ USBFS_INTSTS0 };
	intsts0 = (unsigned short)~USBFS_INTSTS0_RESM;
    }

    void usb_control_transfer_state_transition()
    {
	// Clear interrupt flag.
	Reg16 intsts0{ USBFS_INTSTS0 };
	intsts0 = (unsigned short)~USBFS_INTSTS0_CTRT;
	
	ra6m1_usb_control_transfer(ra6m1_usb_get_control_transfer_stage());
    }

    void usb_pipe_buffer_ready()
    {
	// Pipe buffer ready for read/write.
	// OUT token received
	Reg16 brdysts{ USBFS_BRDYSTS };
	unsigned short temp = *brdysts;
	for (unsigned int i = 0; i <= 9; i++)
	{
	    if ((temp & (1U << i)) != 0)
	    {
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
		unsigned char epnum = 0;
		if (i != 0)
		{
		    // Figure out which endpoint this is.
		    ra6m1_usb_select_pipe(i);
		    epnum = ra6m1_usb_get_pipe_epnum__();
		}
		// Clear BRDY interrupt before accessing FIFO
		brdysts = (unsigned short)~(1U << i);
		usb_status->device->out_token_received(epnum);
		// Service only one endpoint at a time, so that we are sure to
		// catch a reset as soon as it happens.
		// TODO: This means that higher-numbered endpoints could be starved.
		return;
	    }
	}
    }

    void usb_pipe_buffer_empty()
    {
	// Buffer empty after FIFO data transmitted
	// IN token received
	Reg16 bempsts{ USBFS_BEMPSTS };
	unsigned short temp = *bempsts;
	for (unsigned int i = 0; i <= 9; i++)
	{
	    if ((temp & (1U << i)) != 0)
	    {
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
		// Clear BEMP interrupt before accessing FIFO
		bempsts = (unsigned short)~(1U << i);
		unsigned char epnum = 0;
		if (i != 0)
		{
		    // Figure out which endpoint this is.
		    ra6m1_usb_select_pipe(i);
		    epnum = ra6m1_usb_get_pipe_epnum__();
		    RA6M1USBInEndpoint* ep = static_cast<RA6M1USBInEndpoint*>(usb_status->device->getEndpoint(epnum));
		    ep->send_data();
		}
		else
		{
		    RA6M1USBControlEndpoint* ep = static_cast<RA6M1USBControlEndpoint*>(&usb_status->device->ep0);
		    ep->send_data();
		}
		// TODO: this should be immediately after an IN token is received.
		usb_status->device->in_token_received(epnum);
		// Service only one endpoint at a time, so that we are sure to
		// catch a reset as soon as it happens.
		// TODO: This means that higher-numbered endpoints could be starved.
		return;
	    }
	}
    }
    
    void usb_interrupt()
    {
	// Determine cause of interrupt
	Reg16 intsts0{ USBFS_INTSTS0 };
	Reg16 intenb0{ USBFS_INTENB0 };
	unsigned short mask = *intenb0;
	unsigned short flags = 0;
	while (flags = *intsts0, (flags & mask) != 0)
	{
	    // NB: Check for reset before anything else!
	    if (flags & USBFS_INTSTS0_DVST)
	    {
		usb_device_state_change();
	    }
	    else if (flags & USBFS_INTSTS0_RESM)
	    {
		usb_resume();
	    }
	    else if (flags & USBFS_INTSTS0_CTRT)
	    {
		usb_control_transfer_state_transition();
	    }
	    else if (flags & USBFS_INTSTS0_BRDY)
	    {
		usb_pipe_buffer_ready();
	    }
	    else if (flags & USBFS_INTSTS0_BEMP)
	    {
		usb_pipe_buffer_empty();
	    }
	}
	icu_clear_interrupt(0);
    }
}
