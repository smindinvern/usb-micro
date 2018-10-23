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
#include "sam4s.hh"
#include "main.hh"
#include "usb.hh"
#include "mm.hh"
#include "atmel_timer.hh"

void setup_clocks()
{
	Reg32 pmc_wpmr{ PMC_WPMR };
	Reg32 ckgr_mor{ CKGR_MOR };
	Reg32 pmc_sr{ PMC_SR };
	Reg32 ckgr_pllar{ CKGR_PLLAR };
	Reg32 ckgr_pllbr{ CKGR_PLLBR };
	Reg32 pmc_mckr{ PMC_MCKR };
	Reg32 pmc_usb{ PMC_USB };
	Reg32 pmc_scer{ PMC_SCER };

	pmc_wpmr = 0x504d4300; /* turn PMC write protect off */
	ckgr_mor = 0x370000 | ((ckgr_mor & ~(1 << 1)) | 1); /* clear main xtal oscillator bypass and set main xtal oscillator enable */
	do_dmb();

	ckgr_mor = 0x370000 | (ckgr_mor | (1 << 24));
	
	while (!(pmc_sr & 1)) {} /* wait for main xtal oscillator to lock */
	Reg32 eefc_fmr{ EEFC_FMR(0) };
	eefc_fmr = 0x04000200;
	do_dmb();
	ckgr_pllar = (1 << 29) | (9 << 16) | (0x3f << 8) | 2; /* set PLLA multiplier to 2 and divider to 6 */
	while (!(pmc_sr & (1 << 1))) {} /* wait for PLLA to lock */
	
	ckgr_pllbr = (7 << 16) | (0x3f << 8) | 2; /* set PLLB multiplier to 4 and divider to 1 */
	while (!(pmc_sr & (1 << 2))) {} /* wait for PLLB to lock */
	
	pmc_mckr = 2; /* switch to PLLA */
	pmc_scer = (1 << 7); /* enable UDP 48MHz clock */
	pmc_usb = 1; /* set USB clock source to PLLB */
}

void init()
{
	Reg32 scb_actlr{ ARM_ACTLR };
	Reg32 scb_shcsr{ ARM_SCB_SHCSR };
	Reg32 eefc_fmr{ EEFC_FMR(0) };
	Reg32 wdt_mr{ WDT_MR };

	wdt_mr |= (1 << 15);  /* disable the watchdog timer */
	eefc_fmr = (5 << 8);  /* set wait states to 5 */
	scb_shcsr = (1 << 18) | (1 << 17) | (1 << 16);  /* enable usage fault, bus fault, and mem fault */

	setup_clocks();
	init_mm();
	setup_timer(0);

	init_usb();
}
