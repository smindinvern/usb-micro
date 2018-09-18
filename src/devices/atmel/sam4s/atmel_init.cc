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
#include "usb.hh"
#include "mm.hh"
#include "atmel_timer.hh"

void setup_clocks()
{
	Reg32 pmc_wpmr{ 0x400e04e4 };
	Reg32 ckgr_mor{ 0x400e0420 };
	Reg32 pmc_sr{ 0x400e0468 };
	Reg32 ckgr_pllar{ 0x400e0428 };
	Reg32 ckgr_pllbr{ 0x400e042c };
	Reg32 pmc_mckr{ 0x400e0430 };
	Reg32 pmc_usb{ 0x400e0438 };
	Reg32 pmc_scer{ 0x400e0400 };

	pmc_wpmr = 0x504d4300; /* turn PMC write protect off */
	ckgr_mor = 0x370000 | ((ckgr_mor & ~(1 << 1)) | 1); /* clear main xtal oscillator bypass and set main xtal oscillator enable */
	do_dmb();

	ckgr_mor = 0x370000 | (ckgr_mor | (1 << 24));
	
	while (!(pmc_sr & 1)) {} /* wait for main xtal oscillator to lock */
	
	ckgr_pllar = (1 << 29) | (1 << 16) | (0x3f << 8) | 2; /* set PLLA multiplier to 2 and divider to 6 */
	while (!(pmc_sr & (1 << 1))) {} /* wait for PLLA to lock */
	
	ckgr_pllbr = (3 << 16) | (0x3f << 8) | 1; /* set PLLB multiplier to 4 and divider to 1 */
	while (!(pmc_sr & (1 << 2))) {} /* wait for PLLB to lock */
	
	pmc_mckr = 2; /* switch to PLLA */
	pmc_scer = (1 << 7); /* enable UDP 48MHz clock */
	pmc_usb = 1; /* set USB clock source to PLLB */
}

void init()
{
	Reg32 scb_actlr{ 0xe000e008 };
	Reg32 scb_shcsr{ 0xe000ed24 };
	Reg32 eefc_fmr{ 0x400e0a00 };
	Reg32 wdt_mr{ 0x400e1454 };

	wdt_mr |= (1 << 15);  /* disable the watchdog timer */
	eefc_fmr = (5 << 8);  /* set wait states to 5 */
	scb_shcsr = (1 << 18) | (1 << 17) | (1 << 16);  /* enable usage fault, bus fault, and mem fault */

	setup_clocks();
	init_mm();
	setup_timer(0);

	init_usb();
}
