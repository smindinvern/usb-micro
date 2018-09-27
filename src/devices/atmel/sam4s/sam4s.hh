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

#ifndef _SAM4S_HH
#define _SAM4S_HH

#include "arm.hh"

#define CHIP_FAMILY SAM4S

#define SAM4S_CODE_BASE                (0x00000000UL)
#define SAM4S_SRAM_BASE                (0x20000000UL)
#define SAM4S_PERIPHERALS_BASE         (0x40000000UL)
#define SAM4S_EXTERNAL_SRAM_BASE       (0x60000000UL)
#define SAM4S_SYSTEM_BASE              (0xE0000000UL)

#define SAM4S_HSMCI                    (SAM4S_PERIPHERALS_BASE + 0x00000UL)
#define SAM4S_SSC                      (SAM4S_PERIPHERALS_BASE + 0x04000UL)
#define SAM4S_SPI                      (SAM4S_PERIPHERALS_BASE + 0x08000UL)
#define SAM4S_TC_BASE                  (SAM4S_PERIPHERALS_BASE + 0x10000UL)
#define SAM4S_TC(n)                    (SAM4S_TC_BASE + n * 0x40UL)
#define SAM4S_TWI0                     (SAM4S_PERIPHERALS_BASE + 0x18000UL)
#define SAM4S_TWI1                     (SAM4S_PERIPHERALS_BASE + 0x1C000UL)
#define SAM4S_PWM                      (SAM4S_PERIPHERALS_BASE + 0x20000UL)
#define SAM4S_USART0                   (SAM4S_PERIPHERALS_BASE + 0x24000UL)
#define SAM4S_USART1                   (SAM4S_PERIPHERALS_BASE + 0x28000UL)
#define SAM4S_UDP                      (SAM4S_PERIPHERALS_BASE + 0x34000UL)
#define SAM4S_ADC                      (SAM4S_PERIPHERALS_BASE + 0x38000UL)
#define SAM4S_DACC                     (SAM4S_PERIPHERALS_BASE + 0x3C000UL)
#define SAM4S_ACC                      (SAM4S_PERIPHERALS_BASE + 0x40000UL)
#define SAM4S_CRCCU                    (SAM4S_PERIPHERALS_BASE + 0x44000UL)
#define SAM4S_SYSCTRL_BASE             (SAM4S_PERIPHERALS_BASE + 0xE0000UL)
#define SAM4S_SMC                      (SAM4S_SYSCTRL_BASE + 0x0000UL)
#define SAM4S_MATRIX                   (SAM4S_SYSCTRL_BASE + 0x0200UL)
#define SAM4S_PMC                      (SAM4S_SYSCTRL_BASE + 0x0400UL)
#define SAM4S_UART0                    (SAM4S_SYSCTRL_BASE + 0x0600UL)
#define SAM4S_CHIPID                   (SAM4S_SYSCTRL_BASE + 0x0740UL)
#define SAM4S_UART1                    (SAM4S_SYSCTRL_BASE + 0x0800UL)
#define SAM4S_EEFC(n)                  (SAM4S_SYSCTRL_BASE + 0x0A00UL + n * 0x200UL)
#define SAM4S_EEFC0                    SAM4S_EEFC(0)
#define SAM4S_EEFC1                    SAM4S_EEFC(1)
#define SAM4S_PIO_BASE                 (SAM4S_SYSCTRL_BASE + 0x0E00UL)
#define SAM4S_PIO(n)                   (SAM4S_PIO_BASE + n * 0x0200UL)
#define SAM4S_PIOA                     SAM4S_PIO(0)
#define SAM4S_PIOB                     SAM4S_PIO(1)
#define SAM4S_PIOC                     SAM4S_PIO(2)
#define SAM4S_RSTC                     (SAM4S_SYSCTRL_BASE + 0x1400UL)
#define SAM4S_SUPC                     (SAM4S_SYSCTRL_BASE + 0x1410UL)
#define SAM4S_RTT                      (SAM4S_SYSCTRL_BASE + 0x1430UL)
#define SAM4S_WDT                      (SAM4S_SYSCTRL_BASE + 0x1450UL)
#define SAM4S_RTC                      (SAM4S_SYSCTRL_BASE + 0x1460UL)
#define SAM4S_GPBR                     (SAM4S_SYSCTRL_BASE + 0x1490UL)

// PMC registers
#define PMC_SCER        (SAM4S_PMC + 0x0000UL)
#define PMC_SCDR        (SAM4S_PMC + 0x0004UL)
#define PMC_SCSR        (SAM4S_PMC + 0x0008UL)
#define PMC_PCER0       (SAM4S_PMC + 0x0010UL)
#define PMC_PCDR0       (SAM4S_PMC + 0x0014UL)
#define PMC_PCSR0       (SAM4S_PMC + 0x0018UL)
#define CKGR_MOR        (SAM4S_PMC + 0x0020UL)
#define CKGR_MCFR       (SAM4S_PMC + 0x0024UL)
#define CKGR_PLLAR      (SAM4S_PMC + 0x0028UL)
#define CKGR_PLLBR      (SAM4S_PMC + 0x002CUL)
#define PMC_MCKR        (SAM4S_PMC + 0x0030UL)
#define PMC_USB         (SAM4S_PMC + 0x0038UL)
#define PMC_PCK0        (SAM4S_PMC + 0x0040UL)
#define PMC_PCK1        (SAM4S_PMC + 0x0044UL)
#define PMC_PCK2        (SAM4S_PMC + 0x0048UL)
#define PMC_IER         (SAM4S_PMC + 0x0060UL)
#define PMC_IDR         (SAM4S_PMC + 0x0064UL)
#define PMC_SR          (SAM4S_PMC + 0x0068UL)
#define PMC_IMR         (SAM4S_PMC + 0x006CUL)
#define PMC_FSMR        (SAM4S_PMC + 0x0070UL)
#define PMC_FSPR        (SAM4S_PMC + 0x0074UL)
#define PMC_FOCR        (SAM4S_PMC + 0x0078UL)
#define PMC_WPMR        (SAM4S_PMC + 0x00E4UL)
#define PMC_WPSR        (SAM4S_PMC + 0x00E8UL)
#define PMC_PCER1       (SAM4S_PMC + 0x0100UL)
#define PMC_PCDR1       (SAM4S_PMC + 0x0104UL)
#define PMC_PCSR1       (SAM4S_PMC + 0x0108UL)
#define PMC_OCR         (SAM4S_PMC + 0x0110UL)

// EEFC registers
#define EEFC_FMR(n)     (SAM4S_EEFC(n) + 0x00UL)
#define EEFC_FCR(n)     (SAM4S_EEFC(n) + 0x04UL)
#define EEFC_FSR(n)     (SAM4S_EEFC(n) + 0x08UL)
#define EEFC_FRR(n)     (SAM4S_EEFC(n) + 0x0CUL)

// WDT registers
#define WDT_CR          (SAM4S_WDT + 0x00UL)
#define WDT_MR          (SAM4S_WDT + 0x04UL)
#define WDT_SR          (SAM4S_WDT + 0x08UL)

// TC registers
#define TC_CCR(n)       (SAM4S_TC(n) + 0x00UL)
#define TC_CMR(n)       (SAM4S_TC(n) + 0x04UL)
#define TC_SMMR(n)      (SAM4S_TC(n) + 0x08UL)
#define TC_CV(n)        (SAM4S_TC(n) + 0x10UL)
#define TC_RA(n)        (SAM4S_TC(n) + 0x14UL)
#define TC_RB(n)        (SAM4S_TC(n) + 0x18UL)
#define TC_RC(n)        (SAM4S_TC(n) + 0x1CUL)
#define TC_SR(n)        (SAM4S_TC(n) + 0x20UL)
#define TC_IER(n)       (SAM4S_TC(n) + 0x24UL)
#define TC_IDR(n)       (SAM4S_TC(n) + 0x28UL)
#define TC_IMR(n)       (SAM4S_TC(n) + 0x2CUL)
#define TC_BCR          (SAM4S_TC_BASE + 0xC0UL)
#define TC_BMR          (SAM4S_TC_BASE + 0xC4UL)
#define TC_QIER         (SAM4S_TC_BASE + 0xC8UL)
#define TC_QIDR         (SAM4S_TC_BASE + 0xCCUL)
#define TC_QIMR         (SAM4S_TC_BASE + 0xD0UL)
#define TC_QISR         (SAM4S_TC_BASE + 0xD4UL)
#define TC_FMR          (SAM4S_TC_BASE + 0xD8UL)
#define TC_WPMR         (SAM4S_TC_BASE + 0xE4UL)

// PIO registers
#define PIO_PER(n)      (SAM4S_PIO(n) + 0x0000UL)
#define PIO_PDR(n)      (SAM4S_PIO(n) + 0x0004UL)
#define PIO_PSR(n)      (SAM4S_PIO(n) + 0x0008UL)
#define PIO_OER(n)      (SAM4S_PIO(n) + 0x0010UL)
#define PIO_ODR(n)      (SAM4S_PIO(n) + 0x0014UL)
#define PIO_OSR(n)      (SAM4S_PIO(n) + 0x0018UL)
#define PIO_IFER(n)     (SAM4S_PIO(n) + 0x0020UL)
#define PIO_IFDR(n)     (SAM4S_PIO(n) + 0x0024UL)
#define PIO_IFSR(n)     (SAM4S_PIO(n) + 0x0028UL)
#define PIO_SODR(n)     (SAM4S_PIO(n) + 0x0030UL)
#define PIO_CODR(n)     (SAM4S_PIO(n) + 0x0034UL)
#define PIO_ODSR(n)     (SAM4S_PIO(n) + 0x0038UL)
#define PIO_PDSR(n)     (SAM4S_PIO(n) + 0x003CUL)
#define PIO_IER(n)      (SAM4S_PIO(n) + 0x0040UL)
#define PIO_IDR(n)      (SAM4S_PIO(n) + 0x0044UL)
#define PIO_IMR(n)      (SAM4S_PIO(n) + 0x0048UL)
#define PIO_ISR(n)      (SAM4S_PIO(n) + 0x004CUL)
#define PIO_MDER(n)     (SAM4S_PIO(n) + 0x0050UL)
#define PIO_MDDR(n)     (SAM4S_PIO(n) + 0x0054UL)
#define PIO_MDSR(n)     (SAM4S_PIO(n) + 0x0058UL)
#define PIO_PUDR(n)     (SAM4S_PIO(n) + 0x0060UL)
#define PIO_PUER(n)     (SAM4S_PIO(n) + 0x0064UL)
#define PIO_PESR(n)     (SAM4S_PIO(n) + 0x0068UL)
#define PIO_ABCDSR1(n)  (SAM4S_PIO(n) + 0x0070UL)
#define PIO_ABCDSR2(n)  (SAM4S_PIO(n) + 0x0074UL)
#define PIO_IFSCDR(n)   (SAM4S_PIO(n) + 0x0080UL)
#define PIO_IFSCER(n)   (SAM4S_PIO(n) + 0x0084UL)
#define PIO_IFSCSR(n)   (SAM4S_PIO(n) + 0x0088UL)
#define PIO_SCDR(n)     (SAM4S_PIO(n) + 0x008CUL)
#define PIO_PPDDR(n)    (SAM4S_PIO(n) + 0x0090UL)
#define PIO_PPDER(n)    (SAM4S_PIO(n) + 0x0094UL)
#define PIO_PPDSR(n)    (SAM4S_PIO(n) + 0x0098UL)
#define PIO_OWER(n)     (SAM4S_PIO(n) + 0x00A0UL)
#define PIO_OWDR(n)     (SAM4S_PIO(n) + 0x00A4UL)
#define PIO_OWSR(n)     (SAM4S_PIO(n) + 0x00A8UL)
#define PIO_AIMER(n)    (SAM4S_PIO(n) + 0x00B0UL)
#define PIO_AIMDR(n)    (SAM4S_PIO(n) + 0x00B4UL)
#define PIO_AIMMR(n)    (SAM4S_PIO(n) + 0x00B8UL)
#define PIO_ESR(n)      (SAM4S_PIO(n) + 0x00C0UL)
#define PIO_LSR(n)      (SAM4S_PIO(n) + 0x00C4UL)
#define PIO_ELSR(n)     (SAM4S_PIO(n) + 0x00C8UL)
#define PIO_FELLSR(n)   (SAM4S_PIO(n) + 0x00D0UL)
#define PIO_REHLSR(n)   (SAM4S_PIO(n) + 0x00D4UL)
#define PIO_FRLHSR(n)   (SAM4S_PIO(n) + 0x00D8UL)
#define PIO_LOCKSR(n)   (SAM4S_PIO(n) + 0x00E0UL)
#define PIO_WPMR(n)     (SAM4S_PIO(n) + 0x00E4UL)
#define PIO_WPSR(n)     (SAM4S_PIO(n) + 0x00E8UL)
#define PIO_SCMITT(n)   (SAM4S_PIO(n) + 0x0100UL)
#define PIO_PCMR(n)     (SAM4S_PIO(n) + 0x0150UL)
#define PIO_PCIER(n)    (SAM4S_PIO(n) + 0x0154UL)
#define PIO_PCIDR(n)    (SAM4S_PIO(n) + 0x0158UL)
#define PIO_PCIMR(n)    (SAM4S_PIO(n) + 0x015CUL)
#define PIO_PCISR(n)    (SAM4S_PIO(n) + 0x0160UL)
#define PIO_PCRHR(n)    (SAM4S_PIO(n) + 0x0164UL)

// UDP registers
#define UDP_FRM_NUM     (SAM4S_UDP + 0x000UL)
#define UDP_GLB_STAT    (SAM4S_UDP + 0x004UL)
#define UDP_FADDR       (SAM4S_UDP + 0x008UL)
#define UDP_IER         (SAM4S_UDP + 0x010UL)
#define UDP_IDR         (SAM4S_UDP + 0x014UL)
#define UDP_IMR         (SAM4S_UDP + 0x018UL)
#define UDP_ISR         (SAM4S_UDP + 0x01CUL)
#define UDP_ICR         (SAM4S_UDP + 0x020UL)
#define UDP_RST_EP      (SAM4S_UDP + 0x028UL)
#define UDP_CSR(n)      (SAM4S_UDP + 0x030UL + n * 0x4UL)
#define UDP_FDR(n)      (SAM4S_UDP + 0x050UL + n * 0x4UL)
#define UDP_TXVC        (SAM4S_UDP + 0x074UL)

// Bus MATRIX registers
#define MATRIX_MCFG0    (SAM4S_MATRIX + 0x0000UL)
#define MATRIX_MCFG1    (SAM4S_MATRIX + 0x0004UL)
#define MATRIX_MCFG2    (SAM4S_MATRIX + 0x0008UL)
#define MATRIX_MCFG3    (SAM4S_MATRIX + 0x000CUL)
#define MATRIX_SCFG0    (SAM4S_MATRIX + 0x0040UL)
#define MATRIX_SCFG1    (SAM4S_MATRIX + 0x0044UL)
#define MATRIX_SCFG2    (SAM4S_MATRIX + 0x0048UL)
#define MATRIX_SCFG3    (SAM4S_MATRIX + 0x004CUL)
#define MATRIX_SCFG4    (SAM4S_MATRIX + 0x0050UL)
#define MATRIX_PRAS0    (SAM4S_MATRIX + 0x0080UL)
#define MATRIX_PRAS1    (SAM4S_MATRIX + 0x0088UL)
#define MATRIX_PRAS2    (SAM4S_MATRIX + 0x0090UL)
#define MATRIX_PRAS3    (SAM4S_MATRIX + 0x0098UL)
#define MATRIX_PRAS4    (SAM4S_MATRIX + 0x00A0UL)
#define CCFG_SYSIO      (SAM4S_MATRIX + 0x0114UL)
#define CCFG_SMCNFCS    (SAM4S_MATRIX + 0x011CUL)
#define MATRIX_WPMR     (SAM4S_MATRIX + 0x01E4UL)
#define MATRIX_WPSR     (SAM4S_MATRIX + 0x01E8UL)

#endif
