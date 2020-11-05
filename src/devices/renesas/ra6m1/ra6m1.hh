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

#ifndef RA6M1_HH_
#define RA6M1_HH_

#include "arm.hh"
#include "Registers.hh"

// Bus addresses
#define RA6M1_MEM_BUS_1_3       (0x00000000UL)
#define RA6M1_MEM_BUS_2_3       (0x1FFE0000UL)
#define RA6M1_MEM_BUS_4         (0x20000000UL)
#define RA6M1_MEM_BUS_5         (0x20040000UL)
#define RA6M1_PERIPH_BUS_1      (0x40000000UL)
#define RA6M1_PERIPH_BUS_3      (0x40040000UL)
#define RA6M1_PERIPH_BUS_4      (0x40060000UL)
#define RA6M1_PERIPH_BUS_5      (0x40080000UL)
#define RA6M1_PERIPH_BUS_7      (0x400C0000UL)
#define RA6M1_PERIPH_BUS_9      (0x40100000UL)
#define RA6M1_QSPI_BUS          (0x60000000UL)
#define RA6M1_CS_BUS            (0x80000000UL)

// Address space

#define RA6M1_CODE_BASE         (0x00000000UL)
#define RA6M1_OPTION_MEM_BASE   (0x01007000UL)
#define RA6M1_OPTION_MEM2_BASE  (0x0100A150UL)
#define RA6M1_MEM_MAP_BASE      (0x02000000UL)
#define RA6M1_SRAMHS_BASE       (0x1FFE0000UL)
#define RA6M1_SRAM0_BASE        (0x20000000UL)
#define RA6M1_STANDBY_RAM_BASE  (0x20100000UL)
#define RA6M1_PERIPHERALS_BASE  (0x40000000UL)
#define RA6M1_DATA_BASE         (0x40100000UL)
#define RA6M1_FLASH_IO_BASE     (0x407E0000UL)
#define RA6M1_OPTION_MEM3_BASE  (0x407FB17CUL)
#define RA6M1_FLASH_IO2_BASE    (0x40800000UL)
#define RA6M1_EXT_SPI_ADDR_BASE (0x60000000UL)
#define RA6M1_EXT_CS_ADDR_BASE  (0x80000000UL)
#define RA6M1_SYSTEM_BASE       (0xE0000000UL)

// External (CS) address space

#define RA6M1_QSPI_IO_BASE      (0x64000000UL)
#define RA6M1_EXT_CSN_BASE(n)   (RA6M1_EXT_CS_ADDR_BASE + ((unsigned long)(n) * 0x01000000UL)
#define RA6M1_EXT_CS0_BASE      RA6M1_EXT_CSN_BASE(0)
#define RA6M1_EXT_CS1_BASE      RA6M1_EXT_CSN_BASE(1)
#define RA6M1_EXT_CS4_BASE      RA6M1_EXT_CSN_BASE(4)
#define RA6M1_EXT_CS5_BASE      RA6M1_EXT_CSN_BASE(5)
#define RA6M1_EXT_CS6_BASE      RA6M1_EXT_CSN_BASE(6)
#define RA6M1_EXT_CS7_BASE      RA6M1_EXT_CSN_BASE(7)
#undef RA6M1_EXT_CSN_BASE

// Memory Mirror Function (MMF)

#define RA6M1_MMF_BASE           (RA6M1_PERIPHERALS_BASE + 0x1000UL)

// Register Write Protection
#define PRCR  (0x4001E3FE)
#define PRC0  1U
#define PRC1  (1U << 1)
#define PRC3  (1U << 3)
#define PRKEY (0xA5 << 8)
inline void set_registers_protect(bool cgc, bool lpm, bool lvd)
{
    Reg16 prcr{ PRCR };
    unsigned short val = 0;
    if (!cgc) val |= PRC0;
    if (!lpm) val |= PRC1;
    if (!lvd) val |= PRC3;
    prcr = val | PRKEY;
}

// Interrupt Controller Unit (ICU)

#define RA6M1_ICU_BASE          (RA6M1_PERIPHERALS_BASE + 0x6000UL)

// IRQ Control Register i
#define RA6M1_IRQCRi(i)         (RA6M1_ICU_BASE + ((unsigned long)(i)))
// IRQi Detection Sense Select
#define IRQMD_FALLING_EDGE            (0b00)
#define IRQMD_RISING_EDGE             (0b01)
#define IRQMD_RISING_AND_FALLING_EDGE (0b10)
#define IRQMD_LOW_LEVEL               (0b11)
// IRQi Digital Filter Sampling Clock Select
#define FCLKSEL_PCLKB                 (0b00 << 4)
#define FCLKSEL_PCLKB_8               (0b01 << 4)
#define FCLKSEL_PCLKB_32              (0b10 << 4)
#define FCLKSEL_PCLKB_64              (0b11 << 4)
// IRQi Digital Filter Enable
#define FLTEN                         (0b1 << 7)

// Non-Maskable Interrupt Status Register
#define RA6M1_NMISR             (RA6M1_ICU_BASE + 0x140UL)
// IWDT Underflow/Refresh Error Status Flag
#define IWDTST   (1U)
// WDT Underflow/Refresh Error Status Flag
#define WDTST    (1U << 1)
// Voltage Monitor 1 Interrupt Status Flag
#define LVD1ST   (1U << 2)
// Voltage Monitor 2 Interrupt Status Flag
#define LVD2ST   (1U << 3)
// Oscillation Stop Detection Interrupt Status Flag
#define OSTST    (1U << 6)
// NMI Status Flag
#define NMIST    (1U << 7)
// SRAM Parity Error Interrupt Status Flag
#define RPEST    (1U << 8)
// SRAM ECC Error Interrupt Status Flag
#define RECCST   (1U << 9)
// MPU Bus Slave Error Interrupt Status Flag
#define BUSSST   (1U << 10)
// MPU Bus Master Error Interrupt Status Flag
#define BUSMST   (1U << 11)
// CPU Stack Pointer Monitor Interrupt Status Flag
#define SPEST    (1U << 12)

// Non-Maskable Interrupt Enable Register
#define RA6M1_NMIER             (RA6M1_ICU_BASE + 0x120UL)
// IWDT Underflow/Refresh Error Interrupt Enable
#define IWDTEN   (1U)
// WDT Underflow/Refresh Error Interrupt Enable
#define WDTEN    (1U << 1)
// Voltage Monitor 1 Interrupt Enable
#define LVD1EN   (1U << 2)
// Voltage Monitor 2 Interrupt Enable
#define LVD2EN   (1U << 3)
// Oscillation Stop Detection Interrupt Enable
#define OENEN    (1U << 6)
// NMI Pin Interrupt Enable
#define NMIEN    (1U << 7)
// SRAM Parity Error Interrupt Enable
#define RPEEN    (1U << 8)
// SRAM ECC Error Interrupt Enable
#define RECCEN   (1U << 9)
// MPU Bus Slave Error Interrupt Enable
#define BUSSEN   (1U << 10)
// MPU Bus Master Error Interrupt Enable
#define BUSMEN   (1U << 11)
// CPU Stack Pointer Monitor Interrupt Enable
#define SPEEN    (1U << 12)

// Non-Maskable Interrupt Status Clear Register
#define RA6M1_NMICLR            (RA6M1_ICU_BASE + 0x130UL)
// IWDT Clear
#define IWDTCLR   (1U)
// WDT Clear
#define WDTCLR    (1U << 1)
// LVD1 Clear
#define LVD1CLR   (1U << 2)
// LVD2 Clear
#define LVD2CLR   (1U << 3)
// OST Clear
#define OCLRCLR    (1U << 6)
// NMI Clear
#define NMICLR    (1U << 7)
// SRAM Parity Error Clear
#define RPECLR    (1U << 8)
// SRAM ECC Error Clear
#define RECCCLR   (1U << 9)
// MPU Bus Slave Error Clear
#define BUSSCLR   (1U << 10)
// MPU Bus Master Error Clear
#define BUSMCLR   (1U << 11)
// SPEST Clear
#define SPECLR    (1U << 12)

// NMI Pin Interrupt Control Register
#define RA6M1_NMICR             (RA6M1_ICU_BASE + 0x100UL)
// NMI Detection Set
#define NMIMD_FALLING_EDGE      (0U)
#define NMIMD_RISING_EDGE       (1U)
// NMI Digital Filter Sampling Clock Select
#define NFCLKSEL_PCLKB          (0b00 << 4)
#define NFCLKSEL_PCLKB_8        (0b01 << 4)
#define NFCLKSEL_PCLKB_32       (0b10 << 4)
#define NFCLKSEL_PCLKB_64       (0b11 << 4)
// NMI Digital Filter Enable
#define NFLTEN                  (1U << 7)

inline void ra6m1_event_link_select(unsigned long reg_addr,
				    unsigned short event)
{
    Reg16 reg { reg_addr };
    reg = event;
}

// ICU Event Link Setting Register n
#define RA6M1_IELSR(n)          (RA6M1_ICU_BASE + 0x300UL + ((unsigned long)(n) * 0x4UL))
// ICU Event Link Select
inline void ra6m1_icu_event_link_select(unsigned char irq, unsigned short event)
{
    ra6m1_event_link_select(RA6M1_IELSR(irq), event);
}
inline void ra6m1_icu_event_link_disable(unsigned char irq)
{
    ra6m1_icu_event_link_select(irq, 0);
}
// Interrupt Status Flag
#define IELSR_IR                (1U << 16)
// DTC Activation Enable
#define DTCE                    (1U << 24)

// DMAC Event Link Setting Register n
#define RA6M1_DELSR(n)          (RA6M1_ICU_BASE + 0x280UL + ((unsigned long)(n) * 0x4UL))
// DMAC Event Link Select
inline void ra6m1_dmac_event_link_select(unsigned char dmac, unsigned short event)
{
    ra6m1_event_link_select(RA6M1_DELSR(dmac), event);
}
inline void ra6m1_dmac_event_link_disable(unsigned char dmac)
{
    ra6m1_dmac_event_link_select(dmac, 0);
}
// Interrupt Status Flag for DMAC
#define DELSR_IR                (1U << 16)

// SYS Event Link Setting Register
#define RA6M1_SELSR0            (RA6M1_ICU_BASE + 0x200UL)
// SYS Event Link Select
inline void ra6m1_sys_event_link_select(unsigned short event)
{
    ra6m1_event_link_select(RA6M1_SELSR0, event);
}
inline void ra6m1_sys_event_link_disable()
{
    ra6m1_sys_event_link_select(0);
}

// Wake Up Interrupt Enable Register
#define RA6M1_WUPEN             (RA6M1_ICU_BASE + 0x1A0UL)
// IRQ Interrupt Software Standby Returns Enable
#define IRQWUPEN(n)             (1U << (n))
// IWDT Interrupt Software Standby Returns Enable
#define IWDTWUPEN               (1U << 16)
// Key Interrupt Software Standby Returns Enable
#define KEYWUPEN                (1U << 17)
// LVD1 Interrupt Software Standby Returns Enable
#define LVD1WUPEN               (1U << 18)
// LVD2 Interrupt Software Standby Returns Enable
#define LVD2WUPEN               (1U << 19)
// ACMPHS0 Interrupt Software Standby Returns Enable
#define ACMPHS00WUPEN           (1U << 22)
// RTC Alarm Interrupt Software Standby Returns Enable
#define RTCALMWUPEN             (1U << 24)
// RTC Period Interrupt Software Standby Returns Enable
#define RTCPRDWUPEN             (1U << 25)
// USBFS Interrupt Software Standby Returns Enable
#define USBFSWUPEN              (1U << 27)
// AGT1 Underflow Interrupt Software Standby Returns Enable
#define AGT1UDWUPEN             (1U << 28)
// AGT1 Compare Match A Interrupt Software Standby Returns Enable
#define AGT1CAWUPEN             (1U << 29)
// AGT1 Compare Match B Interrupt Software Standby Returns Enable
#define AGT1CBWUPEN             (1U << 30)
// IIC0 Address Match Interrupt Software Standby Returns Enable
#define IIC0WUPEN               (1U << 31)

// Flash Memory
#define RA6M1_FCACHE_BASE       (0x4001C000UL)
#define FLWT                    (RA6M1_FCACHE_BASE + 0x11C)

#define FLWT_ICLK_LT_40M        (0b000)
#define FLWT_ICLK_40M_TO_80M    (0b001)
#define FLWT_ICLK_80M_TO_120M   (0b010)



// Automatically include these headers
#include "ra6m1_clocks.hh"
#include "ra6m1_ports.hh"


#endif // RA6M1_HH_

