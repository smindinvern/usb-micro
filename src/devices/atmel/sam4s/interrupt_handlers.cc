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

#include "sam4s.hh"
#include "main.hh"

extern "C" {
	void usb_ep_isr(unsigned int);
	void usb_enable();
  
	void nmi(void)
	{
        return;
	}

	void hard_fault(void)
	{
        return;
	}

	void mem_manage_fault(void)
	{
        while (1);
	}

	void bus_fault(void)
	{
        if (1 == 1)
			while (1);

        return;
	}

	void usage_fault(void)
	{
        while (1);

        return;
	}

	void svcall(void)
	{
        return;
	}

	void pendsv(void)
	{
        return;
	}

	void systick(void)
	{
        return;
	}

	void unused_interrupt(void)
	{
        return;
	}

	void watchdog_interrupt(void)
	{
        return;
	}

	void uart0_interrupt(void)
	{
        return;
	}

	void timer_interrupt(void)
	{
        volatile unsigned int *pio_odsr = (unsigned int *)(PIO_ODSR(1));

        *pio_odsr ^= (1 << 10);

        return;
	}

	void sam_usb_enter_default_state(const unsigned int);
	
	void udp_interrupt(void)
	{
        volatile unsigned int *udp_isr = (unsigned int *)(UDP_ISR);
        volatile unsigned int *udp_icr = (unsigned int *)(UDP_ICR);
		volatile unsigned int *udp_imr = (unsigned int *)(UDP_IMR);

        while (unsigned int active = *udp_isr & *udp_imr) {
			if (active & (1 << 12)) {
				volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
				sam_usb_enter_default_state(1 << 3);
				usb_status->device->reset();
				// usb_enter_default_state();
				usb_enable();
				*udp_icr = (1 << 12);
			}
			else if (active & 0xff) {
				for (int i = 0; i < 8; i++) {
					if (active & (1 << i)) {
						usb_ep_isr(i);
						*udp_icr = (1 << i);
					}
				}
			}
			else {
				*udp_icr = 0x00003f00;
			}
        }

        return;
	}
}
