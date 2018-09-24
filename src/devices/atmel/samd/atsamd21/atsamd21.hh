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

#ifndef ATSAMD21_HH_
#define ATSAMD21_HH_

#include "arm.hh"
#include "main.hh"

#define CHIP_FAMILY SAMD21

#define NVM_OTP 0x806020

#define SAMD21_SCS_BASE          (0xE000E000UL)
#define SAMD21_SYSTICK_BASE      (0xE000E010UL)
#define SAMD21_NVIC_BASE         (0xE000E100UL)
#define SAMD21_SCB_BASE          (0xE000ED00UL)

#define SAMD21_CODE_BASE         (0x00000000UL)
#define SAMD21_SRAM_BASE         (0x20000000UL)
#define SAMD21_PERIPHERALS_BASE  (0x40000000UL)
#define SAMD21_IOBUS_BASE        (0x60000000UL)
#define SAMD21_SYSTEM_BASE       (0xE0000000UL)

#define SAMD21_AHB_APB_BRIDGE_A  (SAMD21_PERIPHERALS_BASE + 0x00000000UL)
#define SAMD21_AHB_APB_BRIDGE_B  (SAMD21_PERIPHERALS_BASE + 0x01000000UL)
#define SAMD21_AHB_APB_BRIDGE_C  (SAMD21_PERIPHERALS_BASE + 0x02000000UL)

// BRIDGE "A" PERIPHERALS
#define SAMD21_PAC0_OFFSET     (0x0000UL)
#define SAMD21_PM_OFFSET       (0x0400UL)
#define SAMD21_SYSCTRL_OFFSET  (0x0800UL)
#define SAMD21_GCLK_OFFSET     (0x0C00UL)
#define SAMD21_WDT_OFFSET      (0x1000UL)
#define SAMD21_RTC_OFFSET      (0x1400UL)
#define SAMD21_EIC_OFFSET      (0x1800UL)

#define SAMD21_PAC0            (SAMD21_AHB_APB_BRIDGE_A + SAMD21_PAC0_OFFSET)
#define SAMD21_PM              (SAMD21_AHB_APB_BRIDGE_A + SAMD21_PM_OFFSET)
#define SAMD21_SYSCTRL		   (SAMD21_AHB_APB_BRIDGE_A + SAMD21_SYSCTRL_OFFSET)
#define SAMD21_GCLK   		   (SAMD21_AHB_APB_BRIDGE_A + SAMD21_GCLK_OFFSET)
#define SAMD21_WDT    		   (SAMD21_AHB_APB_BRIDGE_A + SAMD21_WDT_OFFSET)
#define SAMD21_RTC    		   (SAMD21_AHB_APB_BRIDGE_A + SAMD21_RTC_OFFSET)
#define SAMD21_EIC    		   (SAMD21_AHB_APB_BRIDGE_A + SAMD21_EIC_OFFSET)

// BRIDGE "B" PERIPHERALS
#define SAMD21_PAC1_OFFSET     (0x0000UL)
#define SAMD21_DSU_OFFSET      (0x2000UL)
#define SAMD21_NVMCTRL_OFFSET  (0x4000UL)
#define SAMD21_PORT_OFFSET     (0x4400UL)
#define SAMD21_DMAC_OFFSET     (0x4800UL)
#define SAMD21_USB_OFFSET      (0x5000UL)
#define SAMD21_MTB_OFFSET      (0x6000UL)

#define SAMD21_PAC1            (SAMD21_AHB_APB_BRIDGE_B + SAMD21_PAC1_OFFSET)
#define SAMD21_DSU    		   (SAMD21_AHB_APB_BRIDGE_B + SAMD21_DSU_OFFSET)
#define SAMD21_NVMCTRL		   (SAMD21_AHB_APB_BRIDGE_B + SAMD21_NVMCTRL_OFFSET)
#define SAMD21_PORT   		   (SAMD21_AHB_APB_BRIDGE_B + SAMD21_PORT_OFFSET)
#define SAMD21_DMAC   		   (SAMD21_AHB_APB_BRIDGE_B + SAMD21_DMAC_OFFSET)
#define SAMD21_USB    		   (SAMD21_AHB_APB_BRIDGE_B + SAMD21_USB_OFFSET)
#define SAMD21_MTB    		   (SAMD21_AHB_APB_BRIDGE_B + SAMD21_MTB_OFFSET)

// BRIDGE "C" PERIPHERALS
#define SAMD21_PAC2_OFFSET     (0x0000UL)
#define SAMD21_EVSYS_OFFSET    (0x0400UL)
#define SAMD21_SERCOM0_OFFSET  (0x0800UL)
#define SAMD21_SERCOM1_OFFSET  (0x0C00UL)
#define SAMD21_SERCOM2_OFFSET  (0x1000UL)
#define SAMD21_SERCOM3_OFFSET  (0x1400UL)
#define SAMD21_SERCOM4_OFFSET  (0x1800UL)
#define SAMD21_SERCOM5_OFFSET  (0x1C00UL)
#define SAMD21_TCC0_OFFSET     (0x2000UL)
#define SAMD21_TCC1_OFFSET     (0x2400UL)
#define SAMD21_TCC2_OFFSET     (0x2800UL)
#define SAMD21_TC3_OFFSET      (0x2C00UL)
#define SAMD21_TC4_OFFSET      (0x3000UL)
#define SAMD21_TC5_OFFSET      (0x3400UL)
#define SAMD21_TC6_OFFSET      (0x3800UL)
#define SAMD21_TC7_OFFSET      (0x3C00UL)
#define SAMD21_ADC_OFFSET      (0x4000UL)
#define SAMD21_AC_OFFSET       (0x4400UL)
#define SAMD21_DAC_OFFSET      (0x4800UL)
#define SAMD21_PTC_OFFSET      (0x4C00UL)
#define SAMD21_I2S_OFFSET      (0x5000UL)
#define SAMD21_AC1_OFFSET      (0x5400UL)

#define SAMD21_PAC2            (SAMD21_AHB_APB_BRIDGE_C + SAMD21_PAC2_OFFSET)
#define SAMD21_EVSYS  		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_EVSYS_OFFSET)
#define SAMD21_SERCOM0		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_SERCOM0_OFFSET)
#define SAMD21_SERCOM1		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_SERCOM1_OFFSET)
#define SAMD21_SERCOM2		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_SERCOM2_OFFSET)
#define SAMD21_SERCOM3		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_SERCOM3_OFFSET)
#define SAMD21_SERCOM4		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_SERCOM4_OFFSET)
#define SAMD21_SERCOM5		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_SERCOM5_OFFSET)
#define SAMD21_TCC0   		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TCC0_OFFSET)
#define SAMD21_TCC1   		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TCC1_OFFSET)
#define SAMD21_TCC2   		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TCC2_OFFSET)
#define SAMD21_TC3    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TC3_OFFSET)
#define SAMD21_TC4    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TC4_OFFSET)
#define SAMD21_TC5    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TC5_OFFSET)
#define SAMD21_TC6    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TC6_OFFSET)
#define SAMD21_TC7    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_TC7_OFFSET)
#define SAMD21_ADC    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_ADC_OFFSET)
#define SAMD21_AC     		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_AC_OFFSET)
#define SAMD21_DAC    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_DAC_OFFSET)
#define SAMD21_PTC    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_PTC_OFFSET)
#define SAMD21_I2S    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_I2S_OFFSET)
#define SAMD21_AC1    		   (SAMD21_AHB_APB_BRIDGE_C + SAMD21_AC1_OFFSET)

#include "samd.hh"

// Watchdog timer
#define WDT_CTRL        (WDT + 0x00UL)
#define WDT_CONFIG      (WDT + 0x01UL)
#define WDT_EWCTRL      (WDT + 0x02UL)
#define WDT_INTENCLR    (WDT + 0x04UL)
#define WDT_INTENSET    (WDT + 0x05UL)
#define WDT_INTFLAG     (WDT + 0x06UL)
#define WDT_STATUS      (WDT + 0x07UL)
#define WDT_CLEAR       (WDT + 0x08UL)

// Sysctrl
#define SYSCTRL_INTENCLR (SYSCTRL + 0x00UL)
#define SYSCTRL_INTENSET (SYSCTRL + 0x04UL)
#define SYSCTRL_INTFLAG  (SYSCTRL + 0x08UL)
#define SYSCTRL_PCLKSR   (SYSCTRL + 0x0CUL)
#define SYSCTRL_XOSC     (SYSCTRL + 0x10UL)
#define SYSCTRL_XOSC32K  (SYSCTRL + 0x14UL)
#define SYSCTRL_OSC32K   (SYSCTRL + 0x18UL)
#define SYSCTRL_OSCULP32K (SYSCTRL + 0x1CUL)
#define SYSCTRL_OSC8M    (SYSCTRL + 0x20UL)
#define SYSCTRL_DFLLCTRL (SYSCTRL + 0x24UL)
#define SYSCTRL_DFLLVAL  (SYSCTRL + 0x28UL)
#define SYSCTRL_DFLLMUL  (SYSCTRL + 0x2CUL)
#define SYSCTRL_DFLLSYNC (SYSCTRL + 0x30UL)
#define SYSCTRL_BOD33    (SYSCTRL + 0x34UL)
#define SYSCTRL_VREG     (SYSCTRL + 0x3CUL)
#define SYSCTRL_VREF     (SYSCTRL + 0x40UL)
#define SYSCTRL_DPLLCTRLA (SYSCTRL + 0x44UL)
#define SYSCTRL_DPLLRATIO (SYSCTRL + 0x48UL)
#define SYSCTRL_DPLLCTRLB (SYSCTRL + 0x4CUL)
#define SYSCTRL_DPLLSTATUS (SYSCTRL + 0x50UL)

// Power manager registers
#define PM_CTRL         (PM + 0x00UL)
#define PM_SLEEP        (PM + 0x01UL)
#define PM_CPUSEL       (PM + 0x08UL)
#define PM_APBASEL      (PM + 0x09UL)
#define PM_APBBSEL      (PM + 0x0AUL)
#define PM_APBCSEL      (PM + 0x0BUL)
#define PM_AHBMASK      (PM + 0x14UL)
#define PM_APBAMASK     (PM + 0x18UL)
#define PM_APBBMASK     (PM + 0x1CUL)
#define PM_APBCMASK     (PM + 0x20UL)
#define PM_INTENCLR     (PM + 0x34UL)
#define PM_INTENSET     (PM + 0x35UL)
#define PM_INTFLAG      (PM + 0x36UL)
#define PM_RCAUSE       (PM + 0x38UL)

// PORT registers
#define PORT_BASE(n)     (PORT + (n * 0x80UL))
#define PORT_DIR(n)      (PORT_BASE(n) + 0x00UL)
#define PORT_DIRCLR(n)   (PORT_BASE(n) + 0x04UL)
						 
#define PORT_DIRSET(n)   (PORT_BASE(n) + 0x08UL)
#define PORT_DIRTGL(n)   (PORT_BASE(n) + 0x0CUL)
#define PORT_OUT(n)      (PORT_BASE(n) + 0x10UL)
#define PORT_OUTCLR(n)   (PORT_BASE(n) + 0x14UL)
#define PORT_OUTSET(n)   (PORT_BASE(n) + 0x18UL)
#define PORT_OUTTGL(n)   (PORT_BASE(n) + 0x1CUL)
#define PORT_IN(n)       (PORT_BASE(n) + 0x20UL)
#define PORT_CTRL(n)     (PORT_BASE(n) + 0x24UL)
#define PORT_WRCONFIG(n) (PORT_BASE(n) + 0x28UL)
#define PORT_PMUX(n, m)  (PORT_BASE(n) + 0x30UL + m)

// NVMCTRL registers
#define NVMCTRL_CTRLA    (NVMCTRL + 0x00UL)
#define NVMCTRL_CTRLB    (NVMCTRL + 0x04UL)
#define NVMCTRL_PARAM    (NVMCTRL + 0x08UL)
#define NVMCTRL_INTENCLR (NVMCTRL + 0x0CUL)
#define NVMCTRL_INTENSET (NVMCTRL + 0x10UL)
#define NVMCTRL_INTFLAG  (NVMCTRL + 0x14UL)
#define NVMCTRL_STATUS   (NVMCTRL + 0x18UL)
#define NVMCTRL_ADDR     (NVMCTRL + 0x1CUL)
#define NVMCTRL_LOCK     (NVMCTRL + 0x20UL)

// GCLK registers
#define GCLK_CTRL        (GCLK + 0x00UL)
#define GCLK_STATUS      (GCLK + 0x01UL)
#define GCLK_CLKCTRL     (GCLK + 0x02UL)
#define GCLK_GENCTRL     (GCLK + 0x04UL)
#define GCLK_GENDIV      (GCLK + 0x08UL)

// GCLK constants

// GCLK clock generator IDs
#define GCLKGEN0  (0)
#define GCLKGEN1  (1)
#define GCLKGEN2  (2)
#define GCLKGEN3  (3)
#define GCLKGEN4  (4)
#define GCLKGEN5  (5)
#define GCLKGEN6  (6)
#define GCLKGEN7  (7)

// GCLK IDs
#define GCLK_DFLL48M_REF      (0x00UL)
#define GCLK_DPLL             (0x01UL)
#define GCLK_DPLL_32K         (0x02UL)
#define GCLK_WDT              (0x03UL)
#define GCLK_RTC              (0x04UL)
#define GCLK_EIC              (0x05UL)
#define GCLK_USB              (0x06UL)
#define GCLK_EVSYS_CHANNEL_0  (0x07UL)
#define GCLK_EVSYS_CHANNEL_1  (0x08UL)
#define GCLK_EVSYS_CHANNEL_2  (0x09UL)
#define GCLK_EVSYS_CHANNEL_3  (0x0AUL)
#define GCLK_EVSYS_CHANNEL_4  (0x0BUL)
#define GCLK_EVSYS_CHANNEL_5  (0x0CUL)
#define GCLK_EVSYS_CHANNEL_6  (0x0DUL)
#define GCLK_EVSYS_CHANNEL_7  (0x0EUL)
#define GCLK_EVSYS_CHANNEL_8  (0x0FUL)
#define GCLK_EVSYS_CHANNEL_9  (0x10UL)
#define GCLK_EVSYS_CHANNEL_10 (0x11UL)
#define GCLK_EVSYS_CHANNEL_11 (0x12UL)
#define GCLK_SERCOMx_SLOW     (0x13UL)
#define GCLK_SERCOM0_CORE     (0x14UL)
#define GCLK_SERCOM1_CORE     (0x15UL)
#define GCLK_SERCOM2_CORE     (0x16UL)
#define GCLK_SERCOM3_CORE     (0x17UL)
#define GCLK_SERCOM4_CORE     (0x18UL)
#define GCLK_SERCOM5_CORE     (0x19UL)
#define GCLK_TCC0_TCC1        (0x1AUL)
#define GCLK_TCC2_TC3         (0x1BUL)
#define GCLK_TC4_TC5          (0x1CUL)
#define GCLK_TC6_TC7          (0x1DUL)
#define GCLK_ADC              (0x1EUL)
#define GCLK_AC_DIG_AC1_DIG   (0x1FUL)
#define GCLK_AC_ANA_AC1_ANA   (0x20UL)
#define GCLK_DAC              (0x21UL)
#define GCLK_PTC              (0x22UL)
#define GCLK_I2S_0            (0x23UL)
#define GCLK_I2S_1            (0x24UL)

// GCLK source IDs
#define XOSC                  (0x00UL)
#define GCLKIN                (0x01UL)
#define GCLKGEN1_OUTPUT       (0x02UL)
#define OSCULP32K             (0x03UL)
#define OSC32K                (0x04UL)
#define XOSC32K               (0x05UL)
#define OSC8M                 (0x06UL)
#define DFLL48M               (0x07UL)
#define FDPLL96M              (0x08UL)

#ifdef __cplusplus
inline void setup_gclkgen(const unsigned char gclkgen_id,
						  const unsigned char source_clock_id,
						  const unsigned short div,
						  const bool oe = false)
{
	// Set GENDIV division factor
	Reg32 gendiv{ GCLK_GENDIV };
	gendiv = (div << 8) | gclkgen_id;
	// Wait for synchronization to complete
	Reg8 status{ GCLK_STATUS };
	while ((status & (1 << 7)) != 0);
	// Write GENCTRL to enable generator
	Reg32 genctrl{ GCLK_GENCTRL };
	// GENCTRL.IDC = 1
	// GENCTRL.GENEN = 1
	// GENCTRL.SRC = source_clock_id
	// GENCTRL.ID = gclkgen_id
	bool idc = (div > 1 && (div % 2) != 0);
	genctrl = (1 << 21) | (1 << 19) | (idc << 17) | (1 << 16) | (source_clock_id << 8) | gclkgen_id;
	// Wait for synchronization to complete
	// while ((status & (1 << 7)) != 0);
}

inline void setup_gclk(const unsigned char gclkgen_id,
					   const unsigned char gclk_id)
{
	// Write CLKCTRL to configure generic clock
	Reg16 clkctrl{ GCLK_CLKCTRL };
	// CLKCTRL.CLKEN = 1
	// CLKCTRL.GEN = gclkgen_id
	// CLKCTRL.ID = gclk_id
	clkctrl = (1 << 14) | (gclkgen_id << 8) | gclk_id;
	// Wait for synchronization to complete
	Reg8 status{ GCLK_STATUS };
	while ((status & (1 << 7)) != 0);
}

inline void set_pins_peripheral(const unsigned int group, const unsigned short pins,
								const bool hw, const unsigned char pmux)
{
	Reg32 wrconfig{ PORT_WRCONFIG(group) };
	wrconfig = (int(hw) << 31) | (1 << 30) | (1 << 28) | ((pmux & 0x0f) << 24) | (1 << 16) | (pins & 0xffff);
}

inline void set_pin_peripheral(const unsigned int group, const unsigned int pin,
							   const unsigned char pmux)
{
	const bool hw{ pin > 15 };
	const unsigned int pin2{ 1U << (hw ? (pin - 16U) : pin) };
	set_pins_peripheral(group, pin2, hw, pmux);
}
#endif  // __cplusplus


#endif  // ATSAMD21_HH_
