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

#include "samd_usb.hh"

#include "main.hh"
#include "usb.hh"
#include "mm.hh"

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
 */


void setup_xosc32k()
{
	Reg16 xosc32k{ SYSCTRL_XOSC32K };
	// Clear the enable bit first before we do any configuration.
	// XOSC32K.ENABLE = 0
	xosc32k = 0;
	// XOSC32K.STARTUP = 0x3; ~1s startup time
	const unsigned short startup{ 0x5U << 8U };
	// XOSC32K.ONDEMAND = 0; run the oscillator in all sleep modes.
	const unsigned short ondemand{ 0U };
	// XOSC32K.RUNSTDBY = 1; run the oscillator in standby mode.
	const unsigned short runstdby{ 1U << 6U };
	// Silicon errata: automatic amplitude control doesn't work
	// XOSC32K.AAMPEN = 0; disable automatic amplitude control.
	const unsigned short aampen{ 0U };
	// XOSC32K.EN32K = 1; enable 32kHz output.
	const unsigned short en32k{ 1U << 3U };
	// XOSC32K.XTALEN = 1; crystal connected to XIN32/XOUT32.
	const unsigned short xtalen{ 1U << 2U };
	xosc32k = startup | ondemand | runstdby | aampen | en32k | xtalen;
	// XOSC32K.ENABLE = 1; enable oscillator.
	xosc32k |= 1U << 1U;
	// Wait for oscillator to stabilize.
	Reg32 pclksr{ SYSCTRL_PCLKSR };
	while (!(pclksr & (1U << 1U)));
}

void configure_dfll48m(unsigned short multiplier)
{
	Reg16 dfllctrl{ SYSCTRL_DFLLCTRL };
	// DFLL must be enabled before configuring.  Due to silicon errata, ensure
	// that DFLLCTRL.ONDEMAND is cleared.
	dfllctrl = (1U << 1U);

	// Wait for PCLKSR.DFLLRDY
	Reg32 pclksr{ SYSCTRL_PCLKSR };
	while (!(pclksr & (1U << 4U)));

	// Write DFLLMUL
	Reg32 dfllmul{ SYSCTRL_DFLLMUL };
	// DFLLMUL.CSTEP = 1
	// DFLLMUL.FSTEP = 1
	// DFLLMUL.MUL = multiplier
	dfllmul = (1U << 26U) | (1U << 16U) | multiplier;

	// Write DFLLVAL
	// Set DFLLVAL.COURSE to value from OTP *before* entering closed-loop mode.
	Reg32 dfllval{ SYSCTRL_DFLLVAL };
	// DFLLVAL.COURSE from NVMOTP
	const unsigned char* otp = (const unsigned char* )NVM_OTP;
	// DFLL48M COURSE CAL = bits 63:58
	dfllval = ((otp[7] & 0xFC) >> 2) << 10;
	// DFLLCTRL.MODE = 1; enter closed-loop mode.
	// dfllctrl |= (1U << 2U);
	// DFLLCTRL.WAITLOCK = 1
	// DFLLCTRL.QLDIS = 1
	// DFLLCTRL.ONDEMAND = 0
	// DFLLCTRL.RUNSTDBY = 1
	// DFLLCTRL.USBCRM = 1
	dfllctrl = (1U << 11U) | (1U << 10U) | (1U << 9U) | (1U << 8U) | (1U << 6U) | (1U << 5U) | (1U << 2U) | (1U << 1U);
	// Wait for DFLL48M to lock.
	while (!(pclksr & (1U << 4U)));
}

// Fclk_fdpll96m = Fclk_fdpll96m_ref * (ratio + (LDRFRAC/16))
// Fclk_fdpll96m_ref = f_xosc * (1 / (2 * div))
// For a 32kHz oscillator, x1500 = 48MHz.
void configure_dpll(
    unsigned short ratio,
    unsigned char sixteenths,
    unsigned char ref_clk_bits,
    unsigned short div)
{
    // Make sure that the DPLL is disabled before we reconfigure it.
    Reg8 dpllctrla{ SYSCTRL_DPLLCTRLA };
    dpllctrla = 0;

    Reg32 dpllratio{ SYSCTRL_DPLLRATIO };
    dpllratio = (ratio - 1) | (sixteenths << 16);

    // Set DPLL reference clock source and division factor.
    Reg32 dpllctrlb{ SYSCTRL_DPLLCTRLB };
    dpllctrlb = (ref_clk_bits << 4) | ((div - 1) << 16);

    // Enable the DPLL output.
    dpllctrla = (1 << 7) | (1 << 1);
}


// 37.12
// +-------------+---------------+---------------------------+
// |Vdd range    |NVM wait states|Maximum operating frequency|
// +-------------+---------------+---------------------------+
// |1.62V to 2.7V|0              |14MHz                      |
// |             +---------------+---------------------------+
// |             |1              |28MHz                      |
// |             +---------------+---------------------------+
// |             |2              |42MHz                      |
// |             +---------------+---------------------------+
// |             |3              |48MHz                      |
// +-------------+---------------+---------------------------+
// |2.7V to 3.63V|0              |24MHz                      |
// |             +---------------+---------------------------+
// |             |1              |48MHz                      |
// +-------------+---------------+---------------------------+
void set_cpu_clock(
    unsigned char gclk_source,
    unsigned char nvm_wait_states)
{
    // Before switching CPU clock, configure NVM controller wait-states,
    // as per section 37.12 of the SAMD21 datasheet.
    Reg32 nvmctrlb{ NVMCTRL_CTRLB };
    nvmctrlb = (nvmctrlb & ~(0xf << 1)) | (nvm_wait_states << 1);
    // Feed GCLKGEN0 with DPLL.  This sets GCLK_MAIN clock.
    setup_gclkgen(GCLKGEN0, gclk_source, 1);
}

void init(
    unsigned char gclk_source,
    unsigned char nvm_wait_states)
{
    // Disable watchdog timer
    Reg32 wdt_ctrl{ WDT_CTRL };
    // CTRL.ALWAYSON = 0
    // CTRL.ENABLE = 0
    wdt_ctrl = 0;
    // Enable usage fault, bus fault, and mem fault
    Reg32 scb_shcsr{ ARM_SCB_SHCSR };
    scb_shcsr = (1 << 18) | (1 << 17) | (1 << 16);

    set_cpu_clock(gclk_source, nvm_wait_states);
    init_mm();
}
