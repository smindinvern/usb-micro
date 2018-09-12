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
#include "samd21_usb.hh"

extern "C" {
	void usb_ep_isr(unsigned int);
	void usb_enable();
  
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

	void watchdog_interrupt()
	{
		return;
	}

	void uart0_interrupt()
	{
		return;
	}

	void timer_interrupt()
	{
		volatile unsigned int *pio_odsr = (unsigned int *)(0x400e1238);
	
		*pio_odsr ^= (1 << 10);
	
		return;
	}

	void udp_interrupt()
	{
		// Check if this is an EORST interrupt
		Reg16 intflag{ USB_INTFLAG };
		if (intflag & (1 << 3)) {
			volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
			usb_status->device->reset();
			// Clear interrupt
			intflag = 1 << 3;
			return;
		}
		Reg16 epintsmry{ USB_EPINTSMRY };
		while (epintsmry) {
			for (int i = 0; i < 16; i++) {
				if ((epintsmry & (1 << i)) != 0) {
					usb_ep_isr(i);
				}
			}
		}
		return;
	}
}
