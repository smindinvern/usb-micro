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

#include "atsamd21/atsamd21.hh"
#include "samd_usb.hh"
#include "primitives.hh"

#include "usb.hh"

extern "C" {
	void samd_usb_init_usb()
	{
		// Enable AHB clock for USB
		Reg32 pm_ahbmask{ PM_AHBMASK };
		// AHBMASK.USB = 1
		pm_ahbmask |= (1 << 6);
		// Enable APB clock for USB and PORT (needed to assign I/O lines to USB)
		Reg32 pm_apbbmask{ PM_APBBMASK };
		// APBBMASK.USB = 1
		// APBBMASK.PORT = 1
		pm_apbbmask |= (1 << 5) | (1 << 3);
		// Assign I/O lines to peripheral (USB) function
		set_pin_peripheral(0, 23, 6);
		set_pin_peripheral(0, 24, 6);
		set_pin_peripheral(0, 25, 6);

		// Wait for register synchronization
		Reg32 sysctrl_pclksr{ SYSCTRL_PCLKSR };
		while ((sysctrl_pclksr & (1 << 4)) == 0);
	
		// Load USB PAD calibration values from NVM OTP CAL region
		const unsigned int* nvm_otp{ (const unsigned int*)NVM_OTP };
		const unsigned char usb_transn = (nvm_otp[1] & 0x03E000) >> 13;
		const unsigned char usb_transp = (nvm_otp[1] & 0x7C0000) >> 18;
		const unsigned char usb_trim = (nvm_otp[1] &  0x3800000) >> 21;
		Reg16 padcal{ USB_PADCAL };
		padcal = (usb_trim << 12) | (usb_transn << 6) | usb_transp;

		// Set RAM QoS to highest priority.
		Reg8 qosctrl{ USB_QOSCTRL };
		qosctrl = (0x3 << 2) | 0x3;

		Reg16 intenclr{ USB_INTENCLR };
		intenclr = 0xFFFF;
		Reg16 intenset{ USB_INTENSET };
		intenset = 1 << 3;

		Reg16 usb_ctrlb{ USB_CTRLB };
		usb_ctrlb = 1;

		do_dmb();

		Reg8 usb_ctrla{ USB_CTRLA };
		usb_ctrla = (1 << 2) | (1 << 1);

		Reg8 usb_sync{ USB_SYNCBUSY };
		while ((usb_sync & (1 << 1)) != 0);
	}

	void samd_usb_enable()
	{
		Reg16 usb_ctrlb{ USB_CTRLB };
		usb_ctrlb = 0;
	}
}

void create_samd21_usb_device(const unsigned int n_eps)
{
	samd21_ep_desc* ep_ram = new(std::nothrow) samd21_ep_desc[n_eps];
	Reg32 descadd{ USB_DESCADD };
	descadd = reinterpret_cast<unsigned int>(ep_ram);
	
	samd_usb_init_usb();
}

extern "C" {
	void samd_usb_set_address(const unsigned int address)
	{
		Reg8 dadd{ USB_DADD };
		dadd = address & 0xFF;
		if (address != 0) {
			do_dmb();
			dadd |= 1 << 7;
		}
	}

	void samd_usb_set_configured(bool)
	{
	}

	void samd_usb_enter_default_state()
	{
	}

	bool samd_usb_setup_rxd(unsigned int ep)
	{
		Reg8 epintflag{ USB_EPINTFLAG(ep) };
		return !!(epintflag & (1 << 4));
	}

	bool samd_usb_out_rxd(unsigned int ep)
	{
		Reg8 epintflag{ USB_EPINTFLAG(ep) };
		return !!(epintflag & 1);
	}

	bool samd_usb_tx_completed(unsigned int ep)
	{
		Reg8 epintflag{ USB_EPINTFLAG(ep) };
		return !!(epintflag & (1 << 1));
	}

	void samd_usb_ep_isr(unsigned int ep)
	{
		if (samd_usb_setup_rxd(ep)) {
			usb_setup_token(ep);
		}
		if (samd_usb_out_rxd(ep)) {
			usb_out_token(ep);
		}
		if (samd_usb_tx_completed(ep)) {
			usb_in_token(ep);
		}
	}
}
