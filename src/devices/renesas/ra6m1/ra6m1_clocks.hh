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

#ifndef RA6M1_CLOCKS_HH_
#define RA6M1_CLOCKS_HH_

#include "arm.hh"
#include "Registers.hh"

// Clock Generation Circuit

#define RA6M1_CGC_BASE           (RA6M1_PERIPHERALS_BASE + 0x1E000UL)

// System Clock Division Control Register
#define SCKDIVCR                 (RA6M1_CGC_BASE + 0x20UL)
// Division ratios
#define _CK_X1_1   (0b000)
#define _CK_X1_2   (0b001)
#define _CK_X1_4   (0b010)
#define _CK_X1_8   (0b011)
#define _CK_X1_16  (0b100)
#define _CK_X1_32  (0b101)
#define _CK_X1_64  (0b110)
#define _CK_X1_128 (0b111)
#define CLK_DIV(n) ( _CK_X1_ ## n )

// Peripheral Module Clock D Select
#define PCKD_DIV_SHIFT 0
#define PCKD_DIV(n) CK(n << PCKD_DIV_SHIFT)

// Peripheral Module Clock C Select
#define PCKC_DIV_SHIFT 4
#define PCKC_DIV(n) (CK(n) << PCKC_DIV_SHIFT)

// Peripheral Module Clock B Select
#define PCKB_DIV_SHIFT 8
#define PCKB_DIV(n) (CK(n) << PCKB_DIV_SHIFT)

// Peripheral Module Clock A Select
#define PCKA_DIV_SHIFT 12
#define PCKA_DIV(n) (CK(n) << PCKA_DIV_SHIFT)

// External Bus Clock Select
#define BCK_DIV_SHIFT 16
#define BCK_DIV(n) (CK(n) << BCK_DIV_SHIFT)

// System Clock Select
#define ICK_DIV_SHIFT 24
#define ICK_DIV(n) (CK(n) << ICK_DIV_SHIFT)

// Flash Interface Clock Select
#define FCK_DIV_SHIFT 28
#define FCK_DIV(n) (CK(n) << FCK_DIV_SHIFT)

void set_ck_div_(const unsigned int bits, const unsigned int shift);

template<unsigned int divider>
void set_ck_div(const unsigned int shift);

template<>
inline void set_ck_div<1>(const unsigned int shift)
{
    set_ck_div_(0, shift);
}
template<>
inline void set_ck_div<2>(const unsigned int shift)
{
    set_ck_div_(1, shift);
}
template<>
inline void set_ck_div<4>(const unsigned int shift)
{
    set_ck_div_(2, shift);
}
template<>
inline void set_ck_div<8>(const unsigned int shift)
{
    set_ck_div_(3, shift);
}
template<>
inline void set_ck_div<16>(const unsigned int shift)
{
    set_ck_div_(4, shift);
}

#define set_pckd_div(div) set_ck_div<div>(0)
#define set_pckc_div(div) set_ck_div<div>(4)
#define set_pckb_div(div) set_ck_div<div>(8)
#define set_pcka_div(div) set_ck_div<div>(12)
#define set_bck_div(div) set_ck_div<div>(16)
#define set_ick_div(div) set_ck_div<div>(24)
#define set_fck_div(div) set_ck_div<div>(28)

// System Clock Division Control Register 2
#define SCKDIVCR2                (RA6M1_CGC_BASE + 0x24UL)
// Division ratios
#define UCK_X1_3  (0b010)
#define UCK_X1_4  (0b011)
#define UCK_X1_5  (0b100)

// USB Clock Select
#define UCK(n)  ( ( UCK_X1_ # n ) << 4)

// System Clock Source Control Register
#define SCKSCR                   (RA6M1_CGC_BASE + 0x26UL)
// Clock Source Select
#define CKSEL_HOCO  (0b000)
#define CKSEL_MOCO  (0b001)
#define CKSEL_LOCO  (0b010)
#define CKSEL_MAIN  (0b011)
#define CKSEL_SUB   (0b100)
#define CKSEL_PLL   (0b101)

// PLL Clock Control Register
#define PLLCCR                   (RA6M1_CGC_BASE + 0x28UL)
// PLL Input Frequency Division Ratio Select
#define PLIDIV_1   (0b00)
#define PLIDIV_1_2 (0b01)
#define PLIDIV_1_3 (0b10)

// PLL Clock Source Select
#define PLSRCSEL_MAIN  0UL
#define PLSRCSEL_HOCO  (1U << 4)

// PLL Frequency Multiplication Factor Select
#define PLLMUL(mul)   ((((unsigned long)((2 * (mul)) + 0.5)) - 1UL) << 8)

// PLL Control Register
#define PLLCR                    (RA6M1_CGC_BASE + 0x2AUL)

// PLL Stop Control
#define PLLSTP  1U

// External Bus Clock Control Register
#define BCKCR                    (RA6M1_CGC_BASE + 0x30UL)

// EBCLK Pin Output Select
#define BCLKDIV_1   0U
#define BCLKDIV_1_2 1U

// Main Clock Oscillator Control Register
#define MOSCCR                   (RA6M1_CGC_BASE + 0x32UL)

// Main Clock Oscillator Stop
#define MOSTP  1U

// Subclock Oscillator Control Register
#define SOSCCR                   (RA6M1_CGC_BASE + 0x480UL)
// Sub-Clock Oscillator Stop
#define SOSTP  1U

// Low-Speed On-Chip Oscillator Control Register
#define LOCOCR                   (RA6M1_CGC_BASE + 0x490UL)
// LOCO Stop
#define LCSTP  1U

// High-Speed On-Chip Oscillator Control Register
#define HOCOCR                  (RA6M1_CGC_BASE + 0x36UL)
// HOCO Stop
#define HCSTP  1U

// High-Speed On-Chip Oscillator Wait Control Register
#define HOCOWTCR                (RA6M1_CGC_BASE + 0xA5UL)

// Middle-Speed On-Chip Oscillator Control Register
#define MOCOCR                  (RA6M1_CGC_BASE + 0x38UL)
// MOCO Stop
#define MCSTP  1U

// FLL Control Register 1
#define FLLCR1                  (RA6M1_CGC_BASE + 0x39UL)
// FLL Enable
#define FLLEN  1U

// FLL Control Register 2
#define FLLCR2                  (RA6M1_CGC_BASE + 0x3AUL)

// Oscillation Stabilization Flag Register
#define OSCSF                   (RA6M1_CGC_BASE + 0x3CUL)
// HOCO Clock Oscillation Stabilization Flag
#define HOCOSF  1U
inline bool hoco_is_stable()
{
    Reg8 oscsf { OSCSF };
    return (oscsf & HOCOSF) == HOCOSF;
}

// Main Clock Oscillation Stabilization Flag
#define MOSCSF  (1U << 3)
inline bool main_clock_is_stable()
{
    Reg8 oscsf { OSCSF };
    return (oscsf & MOSCSF) == MOSCSF;
}

// PLL Clock Oscillation Stabilization Flag
#define PLLSF  (1U << 5)
inline bool pll_is_stable()
{
    Reg8 oscsf { OSCSF };
    return (oscsf & PLLSF) == PLLSF;
}

// Oscillation Stop Detection Control Register
#define OSTDCR                  (RA6M1_CGC_BASE + 0x40UL)
// Oscillation Stop Detection Interrupt Enable
#define OSTDIE  1U

// Oscillation Stop Detection Function Enable
#define OSTDE  (1U << 7)

// Oscillation Stop Detection Status Register
#define OSTDSR                  (RA6M1_CGC_BASE + 0x41UL)
// Oscillation Stop Detection Flag
#define OSTDF  1U

// Main Clock Oscillator Wait Control Register
#define MOSCWTCR                (RA6M1_CGC_BASE + 0xA2UL)
// Main Clock Oscillator Wait Time Setting
// TODO: This gives the wrong result!
constexpr inline int main_clock_wait_time_setting(
    const unsigned int cycles,
    const unsigned int accumulator = 0)
{
    if (cycles >= 35)
    {
	return main_clock_wait_time_setting((cycles + 3) / 2, accumulator + 1);
    }
    return accumulator;
}

void set_main_clock_wait_time(unsigned int moscwtcr_bits);

template<unsigned int cycles>
constexpr inline unsigned int main_clock_wait_time()
{
    static_assert(cycles >= 35, "Main clock wait cycles must be >= 35");
    static_assert(cycles <= 8164, "Main clock wait cycles must be <= 8164");
    return main_clock_wait_time_setting(cycles);
}

template<unsigned int cycles>
inline void set_main_clock_wait_time()
{
    set_main_clock_wait_time(main_clock_wait_time<cycles>());
}

// Main Clock Oscillator Mode Oscillation Control Register
#define MOMCR                   (RA6M1_CGC_BASE + 0x413UL)
// Main Clock Oscillator Drive Capability 0 Switching
#define MODRV0_20_24M  (0b00 << 4)
#define MODRV0_16_20M  (0b01 << 4)
#define MODRV0_8_16M   (0b10 << 4)
#define MODRV0_8M      (0b11 << 4)
// Main Clock Oscillator Switching
#define MOSEL_RESONATOR  0U
#define MOSEL_EXT_CLK    (1U << 6)
// Main Clock Oscillator Drive Capability Auto Switching Enable
#define AUTODRVEN  (1U << 7)

template<unsigned long osc_f_hz>
constexpr inline unsigned int modrv0_bits()
{
    static_assert(osc_f_hz >= 8000000, "Main oscillator frequency must be >= 8MHz");
    static_assert(osc_f_hz <= 24000000, "Main oscillator frequency must be <= 24MHz");
    if (osc_f_hz == 8000000) {
	return MODRV0_8M;
    }
    else if (osc_f_hz < 16000000) {
	return MODRV0_8_16M;
    }
    else if (osc_f_hz < 20000000) {
	return MODRV0_16_20M;
    }
    else {
	return MODRV0_20_24M;
    }
}

void enable_main_clock_oscillator(
    unsigned int modrv0_bits,
    unsigned int moscwtcr_bits);

template<unsigned long osc_f_hz, unsigned int wait_cycles>
constexpr inline void enable_main_clock_oscillator()
{
    enable_main_clock_oscillator(
	modrv0_bits<osc_f_hz>(),
	main_clock_wait_time<wait_cycles>());
}

// Subclock Oscillator Mode Control Register
#define SOMCR                   (RA6M1_CGC_BASE + 0x481UL)
// Sub-Clock Oscillator Drive Capability Switching
#define SODRV1_STANDARD  0
#define SODRV1_LOW       (1U << 1)

// Clock Out Control Register
#define CKOCR                   (RA6M1_CGC_BASE + 0x03EUL)
// Clock Out Source Select
#define CKOSEL_HOCO  (0b000)
#define CKOSEL_MOCO  (0b001)
#define CKOSEL_LOCO  (0b010)
#define CKOSEL_MOSC  (0b011)
#define CKOSEL_SOSC  (0b100)
// Clock Out Input Frequency Division Select
#define CKODIV(n)  CK(n, 4)
// Clock Out Enable
#define CKOEN  (1U << 7)

// External Bus Clock Output Control Register
#define EBCKOCR                 (RA6M1_CGC_BASE + 0x52UL)
// EBCLK Pin Output Control
#define EBCKOEN  1U

// LOCO User Trimming Control Register
#define LOCOUTCR                (RA6M1_CGC_BASE + 0x492UL)

// MOCO User Trimming Control Register
#define MOCOUTCR                (RA6M1_CGC_BASE + 0x61UL)

// HOCO User Trimming Control Register
#define HOCOUTCR                (RA6M1_CGC_BASE + 0x62UL)

// Trace Clock Control Register
#define TRCKCR                  (RA6M1_CGC_BASE + 0x3FUL)
// Trace Clock Operating Frequency Select
#define TRCK(n)  CK(n, 0)

// Trace Clock Operation Enable
#define TRCKEN  (1U << 7)

#endif // RA6M1_CLOCKS_HH_
