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

	void udp_interrupt(void)
	{
        volatile unsigned int *udp_isr = (unsigned int *)(UDP_ISR);
        volatile unsigned int *udp_icr = (unsigned int *)(UDP_ICR);

        while (*udp_isr) {
			if (*udp_isr & (1 << 12)) {
				// usb_enter_default_state();
				usb_enable();
				*udp_icr = (1 << 12);
			}
			else if (*udp_isr & 0xff) {
				if (*udp_isr & 1) {
					usb_ep_isr(0);
					*udp_icr = 1;
				}
				if (*udp_isr & (1 << 1)) {
					usb_ep_isr(1);
					*udp_icr = (1 << 1);
				}
				if (*udp_isr & (1 << 2)) {
					usb_ep_isr(2);
					*udp_icr = (1 << 2);
				}
			}
			else {
				*udp_icr = 0x00003f00;
			}
        }

        return;
	}
}
