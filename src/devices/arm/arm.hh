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

#ifndef ARM_HH_
#define ARM_HH_

#ifdef __cplusplus
extern "C" {
#endif
	void enable_interrupt(unsigned int interrupt);
	void set_interrupt_priority(unsigned int interrupt,
								unsigned char priority);
	void wait_n_systicks(unsigned int n);
	unsigned int systick_get_tenms();
	void wait_n_10ms_periods(unsigned short n);
#ifdef __cplusplus
}
#endif

#define ARM_SCS_BASE       (0xE000E000UL)
#define ARM_SYSTICK_BASE   (ARM_SCS_BASE + 0x0010UL)
#define ARM_NVIC_BASE      (ARM_SCS_BASE + 0x0100UL)
#define ARM_MPU_BASE       (ARM_SCS_BASE + 0x0D90UL)

// SCB registers
#define ARM_SCB_BASE       (0xE000ED00UL)
#define ARM_SCB_CPUID      (ARM_SCB_BASE + 0x0000UL)
#define ARM_SCB_ICSR       (ARM_SCB_BASE + 0x0004UL)
#define ARM_SCB_VTOR       (ARM_SCB_BASE + 0x0008UL)
#define ARM_SCB_AIRCR      (ARM_SCB_BASE + 0x000CUL)
#define ARM_SCB_SCR        (ARM_SCB_BASE + 0x0010UL)
#define ARM_SCB_CCR        (ARM_SCB_BASE + 0x0014UL)
#define ARM_SCB_SHPR1      (ARM_SCB_BASE + 0x0018UL)
#define ARM_SCB_SHPR2      (ARM_SCB_BASE + 0x001CUL)
#define ARM_SCB_SHPR3      (ARM_SCB_BASE + 0x0020UL)
#define ARM_SCB_SHCSR      (ARM_SCB_BASE + 0x0024UL)
#define ARM_SCB_CFSR       (ARM_SCB_BASE + 0x0028UL)
#define ARM_SCB_HFSR       (ARM_SCB_BASE + 0x002CUL)
#define ARM_SCB_DFSR       (ARM_SCB_BASE + 0x0030UL)
#define ARM_SCB_MMFAR      (ARM_SCB_BASE + 0x0034UL)
#define ARM_SCB_BFAR       (ARM_SCB_BASE + 0x0038UL)
#define ARM_SCB_AFSR       (ARM_SCB_BASE + 0x003CUL)
#define ARM_SCB_CPACR      (ARM_SCB_BASE + 0x0088UL)

// SCB registers for FP extension
#define ARM_SCB_FPCCR      (ARM_SCB_BASE + 0x0234UL)
#define ARM_SCB_FPCAR      (ARM_SCB_BASE + 0x0238UL)
#define ARM_SCB_FPDSCR     (ARM_SCB_BASE + 0x023CUL)
#define ARM_SCB_MVFR0      (ARM_SCB_BASE + 0x0240UL)
#define ARM_SCB_MVFR1      (ARM_SCB_BASE + 0x0244UL)
#define ARM_SCB_MVFR2      (ARM_SCB_BASE + 0x0248UL)

// SCS registers not in the SCB
#define ARM_MCR            (0xE000E000UL)
#define ARM_ICTR           (0xE000E004UL)
#define ARM_ACTLR          (0xE000E008UL)

// SysTick registers
#define ARM_SYST_CSR       (ARM_SYSTICK_BASE + 0x00UL)
#define ARM_SYST_RVR       (ARM_SYSTICK_BASE + 0x04UL)
#define ARM_SYST_CVR       (ARM_SYSTICK_BASE + 0x08UL)
#define ARM_SYST_CALIB     (ARM_SYSTICK_BASE + 0x0CUL)

// NVIC registers
#define ARM_NVIC_ISER(n)   (ARM_NVIC_BASE + 0x0000UL + (4UL * ((unsigned int)(n / 32))))
#define ARM_NVIC_ICER(n)   (ARM_NVIC_BASE + 0x0080UL + (4UL * ((unsigned int)(n / 32))))
#define ARM_NVIC_ISPR(n)   (ARM_NVIC_BASE + 0x0100UL + (4UL * ((unsigned int)(n / 32))))
#define ARM_NVIC_ICPR(n)   (ARM_NVIC_BASE + 0x0180UL + (4UL * ((unsigned int)(n / 32))))
#define ARM_NVIC_IPR(n)    (ARM_NVIC_BASE + 0x0300UL + (4UL * ((unsigned int)(n / 4))))

#endif
