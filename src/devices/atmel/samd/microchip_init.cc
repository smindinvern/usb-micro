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

#include "samd21_usb.hh"

#include "main.hh"
#include "usb.hh"
#include "mm.hh"

void setup_clocks()
{
	/**
	 * At POR, the clock system is configured as follows:
	 *
	 *        /8             GCLK_MAIN        /1
	 * OSC8M------>GCLKGEN0------------->PM-------->CLK_CPU
	 *                                   ||
	 *                                   ||
	 *              USB<=================++ /1
	 *                                   ||
	 *                                   ||
	 *                                   ||
	 *                                AHB/APB clocks
	 *
	 * The SYSCTRL, GCLK and PM modules are already powered and clocked.
	 *
	 * In order to operate the USB, we need a 48MHz clock.  In order
	 * to respond to USB events with minimal latency, we also clock
	 * the CPU and AHB/APB buses at 48MHz.
	 *
	 *          /1            GCLK_MAIN       /1
	 * DFLL48M----->GCLKGEN0------------>PM-------->CLK_CPU
	 *    ^               |              ||
	 *    +------------+  +-----+        ||
	 *          /1     |        |        || /1
	 * OSC8M------->GCLKGEN1    |        ||
	 *                          V        ||
	 *                         USB<======++
	 *                                   ||
	 *                                AHB/APB clocks
	 */

	// Enable internal 32K oscillator.
	Reg32 osc32k{ SYSCTRL_OSC32K };
	osc32k |= (0x7 << 8) | (1 << 2) | (1 << 1);

	setup_gclkgen(GCLKGEN2, OSC32K, 1);
	setup_gclk(GCLKGEN2, GCLK_DPLL_32K);
	setup_gclk(GCLKGEN2, GCLK_DPLL);

	// Make sure that the DPLL is disabled before we reconfigure it.
	Reg8 dpllctrla{ SYSCTRL_DPLLCTRLA };
	dpllctrla = 0;

	// For a 32kHz * 1500 = 48MHz.  Fclk_fdpll96m = Fclk_fdpll96m_ref * (LDR + 1 + (LDRFRAC/16))
	// So DPLLRATIO.LDR = 1499 yields a multiplier of 1500.
	Reg32 dpllratio{ SYSCTRL_DPLLRATIO };
	dpllratio = 1499;

	// Set DPLL ref clock to GCLK_DPLL.
	Reg32 dpllctrlb{ SYSCTRL_DPLLCTRLB };
	dpllctrlb = (2 << 4);

	// Enable the DPLL output.
	dpllctrla = (1 << 7) | (1 << 1);

	// Before switching CPU to 48MHz clock, configure NVM controller to use 1 wait-state,
	// as per section 37.12 of the SAMD21 datasheet.
	Reg32 nvmctrlb{ NVMCTRL_CTRLB };
	nvmctrlb = (nvmctrlb & ~(0xf << 1)) | (1 << 1);
	// Feed GCLKGEN0 with DPLL.  This sets GCLK_MAIN to 48MHz.
	setup_gclkgen(GCLKGEN0, FDPLL96M, 1);
}

void init()
{
    // Disable watchdog timer
	Reg32 wdt_ctrl{ WDT_CTRL };
	// CTRL.ALWAYSON = 0
	// CTRL.ENABLE = 0
	wdt_ctrl = 0;
	// Enable usage fault, bus fault, and mem fault
	Reg32 scb_shcsr{ ARM_SCB_SHCSR };
	scb_shcsr = (1 << 18) | (1 << 17) | (1 << 16);

	setup_clocks();
	init_mm();
}
