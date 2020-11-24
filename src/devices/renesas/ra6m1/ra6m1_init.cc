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
#include "main.hh"
#include "mm.hh"
#include "ra6m1.hh"

void setup_main_clock(
    unsigned int modrv0_bits,
    unsigned int moscwtcr_bits,
    unsigned int ick_div_bits,
    unsigned int pcka_div_bits,
    unsigned int pckb_div_bits,
    unsigned int pckc_div_bits,
    unsigned int pckd_div_bits,
    unsigned int bck_div_bits,
    unsigned int fck_div_bits,
    unsigned int pli_div_bits,
    unsigned int pll_mul_bits,
    unsigned int flwt_bits,
    unsigned int sramwtsc_bits)
{
    // 9.2.3: The clock sources should be switched when there are no occurring
    // internal asynchronous interrupts.
    // Ensure all interrupts are disabled before switching clock sources
    mask_interrupts();
  
    // 13.1: Register Write Protection is enabled by default for everything,
    // which includes clock generation circuit registers.
    set_registers_protect(false, true, true);
    
    // 9.4.2:
    // If a sub-clock oscillator is not conected, set SOSCCR.SOSTP to 1 to
    // stop the oscillator.
    Reg8 sosccr { SOSCCR };
    sosccr = SOSTP;

    // After POR, system clock source is MOCO
    // Configure and start the main oscillator
    enable_main_clock_oscillator(
	modrv0_bits,
	moscwtcr_bits);

    // Set clock dividers so that we don't exceed limits when we switch to PLL @ 240MHz.
    set_ck_div_(ick_div_bits, ICK_DIV_SHIFT);
    set_ck_div_(pcka_div_bits, PCKA_DIV_SHIFT);
    set_ck_div_(pckb_div_bits, PCKB_DIV_SHIFT);
    set_ck_div_(pckc_div_bits, PCKC_DIV_SHIFT);
    set_ck_div_(pckd_div_bits, PCKD_DIV_SHIFT);
    set_ck_div_(bck_div_bits, BCK_DIV_SHIFT);
    set_ck_div_(fck_div_bits, FCK_DIV_SHIFT);

    // Set PLL source to main oscillator
    Reg16 pllccr{ PLLCCR };
    pllccr = pli_div_bits | PLSRCSEL_MAIN | pll_mul_bits;
    Reg8 pllcr{ PLLCR };
    // Start the PLL
    pllcr = 0;
    // Wait for PLL to stabilize
    while (!pll_is_stable());
    // 9.2.1: Note 5: The frequency of ICLK is limited by the FLWT register.
    // 50.3.2.3: Flash Wait Cycle Register (FLWT)
    //   FLWT[2:0]  0b000: 0 wait (ICLK <= 40MHz)
    //              0b001: 1 wait (40MHz < ICLK <= 80MHz)
    //              0b010: 2 waits (80MHz < ICLK <= 120MHz)
    Reg8 flwt{ FLWT };
    flwt = flwt_bits;
    // 48.4.1:
    // Set the number of SRAM wait cycles in the SRAMWTSC register based on the
    // following:
    // - SRAM0
    //   1 wait: 60MHz < ICLK <= 120MHz
    //   No wait: ICLK <= 60MHz
    // Unlock SRAMWTSC register
    Reg8 sramprcr{ 0x40002004U };
    sramprcr = (0x78U << 1) | 1U;
    Reg8 sramwtsc{ 0x40002008U };
    sramwtsc = (sramwtsc_bits << 1);
    // Lock SRAMWTSC register
    sramprcr = 0;

    // Switch system clock source to PLL
    // 9.2.3: When switching from non-PLL to PLL source, wait at least 250ns after
    // changing the value before starting subsequent processing.
    Reg8 sckscr{ SCKSCR };
    sckscr = CKSEL_PLL;
    // LOCO sources the SysTick clock by default.  One tick is ~30us.
    wait_n_systicks(1);

    set_registers_protect(true, true, true);
    
    // Restore interrupts
    unmask_interrupts();
}

void init(
    unsigned int modrv0_bits,
    unsigned int moscwtcr_bits,
    unsigned int ick_div_bits,
    unsigned int pcka_div_bits,
    unsigned int pckb_div_bits,
    unsigned int pckc_div_bits,
    unsigned int pckd_div_bits,
    unsigned int bck_div_bits,
    unsigned int fck_div_bits,
    unsigned int pli_div_bits,
    unsigned int pll_mul_bits,
    unsigned int flwt_bits,
    unsigned int sramwtc_bits)
{
    setup_main_clock(
	modrv0_bits,
	moscwtcr_bits,
	ick_div_bits,
	pcka_div_bits,
	pckb_div_bits,
	pckc_div_bits,
	pckd_div_bits,
	bck_div_bits,
	fck_div_bits,
	pli_div_bits,
	pll_mul_bits,
        flwt_bits,
	sramwtc_bits);
    init_mm();

    // init_usb();
}
