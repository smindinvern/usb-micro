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

#include "ra6m1_ports.hh"

void pin_set_gpio(
    const unsigned int port,
    const unsigned int pin)
{
    bool wp = port_pop_write_protect();
    Reg32 pfs{ PFS(port, pin) };
    pfs &= ~(1 << PFS_PMR_SHIFT);
    port_push_write_protect(wp);
}

void pin_set_peripheral(
    const unsigned int port,
    const unsigned int pin,
    const unsigned int peripheral_id)
{
    bool wp = port_pop_write_protect();
    Reg32 pfs{ PFS(port, pin) };
    unsigned int temp = pfs;
    // Clear peripheral selection bits.
    temp &= ~(0b11111 << PFS_PSEL_SHIFT);
    // Set pin to peripheral control.
    temp |= 1U << PFS_PMR_SHIFT;
    // Set peripheral selection.
    temp |= peripheral_id << PFS_PSEL_SHIFT;
    // Update register.
    pfs |= temp;
    port_push_write_protect(wp);
}

void pin_write_pfs(
    const unsigned int port,
    const unsigned int pin,
    const unsigned int bits)
{
    bool wp = port_pop_write_protect();
    Reg32 pfs{ PFS(port, pin) };
    pfs = bits;
    port_push_write_protect(wp);
}

void pin_configure_output(
    const unsigned int port,
    const unsigned int pin,
    const bool open_drain,
    const PinDrive drive_level,
    const bool value)
{
    const unsigned int open_drain_bits =
	open_drain ? (1U << PFS_NCODR_SHIFT) : 0;
    const unsigned int value_bits =
	value ? 1 : 0;
    const unsigned int drive_bits = ((unsigned int)drive_level << PFS_DSCR_SHIFT);
    const unsigned int bits = (1U << PFS_PDR_SHIFT) | open_drain_bits | value_bits | drive_bits;

    pin_write_pfs(port, pin, bits);
}

void pin_configure_input(
    const unsigned int port,
    const unsigned int pin,
    const bool pullup,
    const bool irq,
    const bool analog,
    const PinEventEdge edge)
{
    const unsigned int pullup_bits =
	(pullup && !analog) ? (1U << PFS_PCR_SHIFT) : 0;
    const unsigned int irq_bits =
	irq ? (1U << PFS_ISEL_SHIFT) : 0;
    const unsigned int analog_bits =
	analog ? (1U << PFS_ASEL_SHIFT) : 0;
    const unsigned int edge_bits = ((unsigned int)edge << PFS_EOF_EOR_SHIFT);
    const unsigned int bits = pullup_bits | edge_bits | irq_bits | analog_bits;

    pin_write_pfs(port, pin, bits);
}
