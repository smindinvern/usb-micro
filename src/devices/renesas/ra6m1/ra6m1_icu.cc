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

#include "ra6m1.hh"
#include "arm.hh"

inline void icu_link_event(
    unsigned short event,
    unsigned char vector,
    bool dtc)
{
    // 14.4.1
    enable_interrupt(vector);
    Reg32 ielsr{ IELSR(vector) };
    ielsr = event | (dtc ? DTCE : 0);
}

void icu_link_event_to_irq(
    unsigned short event,
    unsigned char vector)
{
    icu_link_event(event, vector, false);
}

void icu_link_event_to_dtc(
    unsigned short event,
    unsigned char vector)
{
    icu_link_event(event, vector, true);
}

void icu_unlink_event(unsigned char vector)
{
    // 14.4.1
    Reg16 ielsr{ IELSR(vector) };
    ielsr = 0;
    disable_interrupt(vector);
    clear_interrupt(vector);
}

void icu_clear_interrupt(unsigned char vector)
{
    // 14.2.6:
    // DTCE must be set to 0 before writing 0 to the IR flag.
    // TODO: Why would we manually clear the IR flag if DTC is servicing events?
    // First clear input to NVIC
    Reg32 ielsr{ IELSR(vector) };
    ielsr &= ~DTCE;
    ielsr &= ~IELSR_IR;
    // Now clear NVIC interrupt pending flag
    clear_interrupt(vector);
}

void icu_link_snooze_cancel_event(unsigned short event)
{
    Reg16 selsr0{ SELSR0 };
    selsr0 = event;
}

void icu_unlink_snooze_cancel_event()
{
    icu_link_snooze_cancel_event(0U);
}

void ra6m1_wfi()
{
    // 14.7: Whenever a WFI instruction is executed, confirm that all status flags
    // in the NMISR register are 0.
    Reg16 nmisr{ NMISR };
    if (nmisr == 0)
    {
	do_wfi();
    }
}

void ra6m1_nmi_clear(unsigned short flags)
{
    Reg16 nmiclr{ NMICLR };
    nmiclr = flags;
}

void ra6m1_nmi_clear()
{
    ra6m1_nmi_clear(0x1FCFU);
}

