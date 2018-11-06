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

#include "arm.hh"
#include "Registers.hh"

extern "C" {
	void enable_interrupt(unsigned int interrupt)
	{
		Reg32 nvic_iser{ ARM_NVIC_ISER(interrupt / 32) };

		interrupt %= 32;
		nvic_iser |= (1 << interrupt);
	}

    void disable_interrupt(unsigned int interrupt)
    {
    	Reg32 nvic_icer{ ARM_NVIC_ICER(interrupt / 32) };
    
    	interrupt %= 32;
    	nvic_icer |= (1 << interrupt);
    }

	void set_interrupt_priority(unsigned int interrupt,
								unsigned char priority)
	{
		Reg32 nvic_ipr{ ARM_NVIC_IPR((int)(interrupt / 4)) };
		interrupt %= 4;
		nvic_ipr = (nvic_ipr & ~(0b11 << (8*interrupt + 6))) | ((priority & 0b11) << (8*interrupt + 6));
	}

	void set_systick_rvr(unsigned int n)
	{
		Reg32 syst_rvr{ ARM_SYST_RVR };
		syst_rvr = n & 0x00FFFFFF;
	}

	void reset_systick()
	{
		Reg32 syst_cvr{ ARM_SYST_CVR };
		syst_cvr = 0;
		Reg32 syst_csr{ ARM_SYST_CSR };
		syst_csr = (1 << 2) | 1;
	}

	unsigned int get_systick_cvr()
	{
		Reg32 syst_cvr{ ARM_SYST_CVR };
		Reg32 syst_csr{ ARM_SYST_CSR };
		bool a{ syst_csr & (1 << 16) };
		unsigned int cvr = syst_cvr;
		bool b{ syst_csr & (1 << 16) };
		if (b) {
			cvr = syst_cvr;
		}
		if (a || b) {
			cvr |= 0x80000000;
		}
		return cvr;
	}

	void wait_n_systicks(unsigned int n)
	{
		set_systick_rvr(n);
		reset_systick();
		while (!(get_systick_cvr() & 0x80000000));
	}

	unsigned int systick_get_tenms()
	{
		Reg32 syst_calib{ ARM_SYST_CALIB };
		return syst_calib & 0x00FFFFFF;
	}

	void wait_n_10ms_periods(unsigned short n)
	{
		unsigned int tenms{ systick_get_tenms() };
		for (unsigned short i = 0; i < n; i++) {
			wait_n_systicks(tenms);
		}
	}
}
