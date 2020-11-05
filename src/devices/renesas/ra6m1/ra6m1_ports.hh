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

#ifndef RA6M1_PORTS_HH_
#define RA6M1_PORTS_HH_

#include "ra6m1.hh"

#define RA6M1_PORT_BASE        (0x40040000UL)


// Port Control Register 1 (port direction and output data)
#define PCNTR1(n)              (RA6M1_PORT_BASE + (0x20UL * (unsigned long)(n)))
// 16bit port direction register
#define PODR(n)                PCNTR1(n)
// 16bit port output data register
#define PDR(n)                 (PCNTR1(n) + 2UL)

// Port Control Register 2 (logic state and event input data)
#define PCNTR2(n)              (RA6M1_PORT_BASE + 4UL + (0x20UL * (unsigned long)(n)))
// 16bit event input data register
#define EIDR(n)                PCNTR2(n)
// 16bit pin state register
// 20.2.2:
// PIDR is not updated when any of the following are enabled:
//   Main clock oscillator (MOSC)
//   CS area controller (CSC)
//   Analog function (ASEL = 1)
//   Capacitive Touch Sensing Unit (CTSU)
//   USB 2.0 Full-Speed Module (USBFS)
#define PIDR(n)                (PCNTR2(n) + 2UL)

// Port Control Register 3 (output set and output reset)
// 20.2.3:
// Note: When EORRn or EOSRn is set, writing is prohibited to PODRn, PORRn, and POSRn.
// Note: PORRn and POSRn should not be set at the same time.
#define PCNTR3(n)              (RA6M1_PORT_BASE + 8UL + (0x20UL * (unsigned long)(n)))
// 16bit output reset register
// PORR(m) |= (1 << n) -> Pmn set low
#define PORR(n)                PCNTR3(n)
// 16bit output set register
// POSR(m) |= (1 << n) -> Pmn set high
#define POSR(n)                (PCNTR3(n) + 2UL)

// Port Control Register 4
// 20.2.4:
// Note: When EORRn or EOSRn is set, writing is prohibited to PODRn, PORRn, and POSRn.
// Note: EORRn and EOSRn should not be set at the same time.
#define PCNTR4(n)              (RA6M1_PORT_BASE + 0xCUL + (0x20UL * (unsigned long)(n)))
// 16bit event output reset register
// EORR(m) |= (1 << n) -> Pmn set low on ELC_PORTx
#define EORR(n)                PCNTR4(n)
// 16bit event output set register
// EOSR(m) |= (1 << n) -> Pmn set high on ELC_PORTx
#define EOSR(n)                (PCNTR4(n) + 2UL)

// Port mn Pin Function Select Register
#define PFS(m, n)              (RA6M1_PORT_BASE + 0x800UL + (0x40UL * (unsigned long)(m)) + (4UL * (unsigned long)(n)))
// PmnPFS half-word access
#define PFS_HA(m, n)           (PFS(m, n) + 2UL)
// PmnPFS byte access
#define PFS_BY(m, n)           (PFS(m, n + 3UL)

#define PFS_PODR_SHIFT 0U
#define PFS_PIDR_SHIFT 1U
#define PFS_PDR_SHIFT 2U
#define PFS_PCR_SHIFT 4U
#define PFS_NCODR_SHIFT 6U
#define PFS_DSCR_SHIFT 10U
#define PFS_EOF_EOR_SHIFT 12U
#define PFS_ISEL_SHIFT 14U
#define PFS_ASEL_SHIFT 15U
#define PFS_PMR_SHIFT 16U
#define PFS_PSEL_SHIFT 24U

// Write-Protect Register
#define PWPR                  (RA6M1_PORT_BASE + 0xD03U)
// PFSWE Bit Write Disable
#define B0WI                  (1U << 7)
// PmnPFS Register Write Enable
#define PFSWE                 (1U << 6)

inline void port_set_write_protect()
{
    Reg8 pwpr{ PWPR };
    pwpr = B0WI;
}

inline void port_clear_write_protect()
{
    Reg8 pwpr{ PWPR };
    pwpr = 0;
    pwpr = PFSWE;
}

inline bool port_get_write_protect()
{
    Reg8 pwpr{ PWPR };
    return (pwpr & PFSWE) == 0;
}

inline bool port_pop_write_protect()
{
    bool wp = port_get_write_protect();
    if (wp)
    {
	port_clear_write_protect();
    }
    return wp;
}

inline void port_push_write_protect(bool wp)
{
    if (wp)
    {
	port_set_write_protect();
    }
}

void pin_set_gpio(
    const unsigned int port,
    const unsigned int pin);

void pin_set_peripheral(
    const unsigned int port,
    const unsigned int pin,
    const unsigned int peripheral_id);

enum PinDrive
{
    LowDrive = 0b00,
    MiddleDrive = 0b01,
    HighDrive = 0b11
};

void pin_configure_output(
    const unsigned int port,
    const unsigned int pin,
    const bool open_drain,
    const PinDrive drive_level,
    const bool value);

enum PinEventEdge
{
    NoEdge = 0b00,
    RisingEdge = 0b01,
    FallingEdge = 0b10,
    BothEdges = 0b11
};

void pin_configure_input(
    const unsigned int port,
    const unsigned int pin,
    const bool pullup,
    const bool irq,
    const bool analog,
    const PinEventEdge edge);

inline void set_pin_output_level(
    const unsigned int port,
    const unsigned int pin,
    const bool level)
{
    if (level)
    {
	Reg16 posr{ POSR(port) };
	posr = 1U << pin;
    }
    else
    {
	Reg16 porr{ PORR(port) };
	porr = 1U << pin;
    }
}

#endif // RA6M1_PORTS_HH_
