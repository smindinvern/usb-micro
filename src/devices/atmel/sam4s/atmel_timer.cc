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

void wait_n_ticks(unsigned short ticks)
{
	Reg32 tc_ccr0{ TC_CCR(0) };
	Reg32 tc_cv0{ TC_CV(0) };

	tc_ccr0 = (1 << 2);

	while ((unsigned int)tc_cv0 < ticks);
}

void setup_timer(unsigned int timer)
{
	Reg32 tc_wpmr{ TC_WPMR };
	Reg32 tc_ccr{ TC_CCR(timer) };
	Reg32 tc_cmr{ TC_CMR(timer) };
	Reg32 tc_ier{ TC_IER(timer) };
	Reg32 pmc_pcer0{ PMC_PCER0 };

	pmc_pcer0 = (1 << (23 + timer));  // enable TC peripheral clock
	tc_wpmr = 0x54494d00;  // disable write protection on TC registers
	tc_cmr = (1 << 15);  // set TC to waveform mode and MC/128
	tc_ier = 1;  // enable TC CV overflow interrupt
	tc_ccr = 1;  // enable the clock
	tc_ccr = (1 << 2);  // trigger the TC
}
