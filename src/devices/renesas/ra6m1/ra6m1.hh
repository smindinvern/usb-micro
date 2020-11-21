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

// Flash Memory
#define RA6M1_FCACHE_BASE       (0x4001C000UL)
#define FLWT                    (RA6M1_FCACHE_BASE + 0x11C)

#define FLWT_ICLK_LT_40M        (0b000)
#define FLWT_ICLK_40M_TO_80M    (0b001)
#define FLWT_ICLK_80M_TO_120M   (0b010)



// Automatically include these headers
#include "ra6m1_clocks.hh"
#include "ra6m1_ports.hh"
#include "ra6m1_icu.hh"
#include "ra6m1_usb.hh"

#endif // RA6M1_HH_

