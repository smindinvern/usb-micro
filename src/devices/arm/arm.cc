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

	unsigned int get_interrupt_mask(unsigned int offset)
	{
		Reg32 nvic_iser{ ARM_NVIC_ISER(offset) };
		return (unsigned int)nvic_iser;
	}

	void set_interrupt_mask(unsigned int offset, unsigned int mask)
	{
		Reg32 nvic_iser{ ARM_NVIC_ISER(offset) };
		nvic_iser = mask;
	}

	void disable_interrupt(unsigned int interrupt)
	{
		Reg32 nvic_icer{ ARM_NVIC_ICER(interrupt / 32) };

		interrupt %= 32;
		nvic_icer |= (1 << interrupt);
	}

	void clear_interrupt(unsigned int interrupt)
	{
		Reg32 nvic_icpr{ ARM_NVIC_ICPR(interrupt / 32) };
		interrupt %= 32;
		nvic_icpr = (1U << interrupt);
	}
  
	void set_interrupt_priority(unsigned int interrupt,
				    unsigned char priority)
	{
		Reg32 nvic_ipr{ ARM_NVIC_IPR(interrupt / 4) };
		interrupt %= 4;
		// bit-fields are 8 bits wide
		unsigned int shift = 8U * interrupt;
		// Clear this interrupt's priority field
		unsigned int temp = nvic_ipr & ~(0xFF << shift);
		// Set the new priority
		temp |= priority << shift;
		nvic_ipr = temp;
	}

	void systick_use_external_clock()
	{
		Reg32 syst_csr{ ARM_SYST_CSR };
		syst_csr &= (1U << 2);
	}

	void systick_use_cpu_clock()
	{
		Reg32 syst_csr{ ARM_SYST_CSR };
		syst_csr |= (1U << 2);
	}

	void systick_set_count(unsigned int n)
	{
		// Cortex-M4 Devices Generic User Guide
		// 4.4.2: SysTick Reload Value Register
		Reg32 syst_rvr{ ARM_SYST_RVR };
		syst_rvr = n & 0x00FFFFFF;
	}

	void systick_set_tickint(bool enabled)
	{
		// 4.4.1: SysTick Control and Status Register
		Reg32 syst_csr{ ARM_SYST_CSR };
		// Set/clear TICKINT flag
		unsigned int tickint = ((enabled ? 1U : 0U) << 1);
		unsigned int temp = syst_csr & ~(1U << 1);
		syst_csr = temp | tickint;
	}

	void systick_enable_tickint()
	{
		systick_set_tickint(true);
	}

	void systick_disable_tickint()
	{
		systick_set_tickint(false);
	}

	void systick_set_enabled(bool enabled)
	{
		// 4.4.1: SysTick Control and Status Register
		Reg32 syst_csr{ ARM_SYST_CSR };
		// Set ENABLE flag to start counting.
		if (enabled)
		{
			syst_csr |= 1U;
		}
		else {
			syst_csr &= ~1U;
		}
	}
	
	void systick_start_counting()
	{
		systick_set_enabled(true);
	}

	void systick_stop_counting()
	{
		systick_set_enabled(false);
	}
	
	void wait_n_systicks(unsigned int n)
	{
		// Temporarily mask SysTick interrupt
		unsigned int mask = get_interrupt_mask(0);
		unsigned int mask_temp = mask & ~(1U << SYSTICK_VECTOR);
		set_interrupt_mask(0, mask_temp);
		systick_set_count(n);
		// Cortex-M4 Devices Generic User Guide
		// 4.4.3: SysTick Current Value Register
		Reg32 syst_cvr{ ARM_SYST_CVR };
		// Reset current value to 0.
		syst_cvr = 0;
		// 4.4.1: SysTick Control and Status Register
		Reg32 syst_csr{ ARM_SYST_CSR };
		systick_start_counting();
		// Wait for COUNTFLAG to be set, indicating the counter has
		// reached 0.
		while ((syst_csr & (1 << 16)) == 0);
		// Restore SysTick interrupt state.  If the interrupt is enabled,
		// it will fire immediately.
		set_interrupt_mask(0, mask);
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
