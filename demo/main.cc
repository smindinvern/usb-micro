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

#include "std.hh"
#include "mm.hh"
#include "samd_usb.hh"
#include "usb.hh"
#include "usbtmc.hh"
#include "usbtmc488.hh"
#include "main.hh"
#include "Invokable.hh"
#include "arm.hh"
#include "primitives.hh"

// define this to make the compiler happy
extern "C" {
	void __cxa_pure_virtual()
	{

	}
}

extern "C" {
	void wait(unsigned short ticks)
	{
		wait_n_systicks(ticks);
	}
}

void init();

extern "C" {
	void start()
	{
		unmask_interrupts();
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
		init();

#define MAX_PACKET_SIZE (64U)
#define VENDOR_ID (0U)
#define PRODUCT_ID (0U)
#define BCD_DEVICE_VERSION (0x0100)

		const USBTMCDeviceDescriptor device_descriptor = {
			MAX_PACKET_SIZE,
			VENDOR_ID,
			PRODUCT_ID,
			BCD_DEVICE_VERSION,
			4, // iManufacturer
			2, // iProduct
			3, // iSerialNumber
			1  // 1 configuration
		};

		samd_usb_init_usb();
		samd_usb_enable();
		samd21_ep_desc* descs = new(std::nothrow) samd21_ep_desc[3];
		Reg32 descadd{ USB_DESCADD };
		descadd = reinterpret_cast<unsigned int>(descs);

		USBDeviceFactory cstr = {
			[&](auto&& fact) -> auto
			{
				USBDevice dev{ SAMDUSBControlEndpoint{ 0, 64, descs },
							   device_descriptor, std::move(fact),
							   new(std::nothrow) USBDeviceGenericImpl() };
				return dev;
			}
		};
		Invokable<std::exclusive_ptr<USBInEndpoint>()> get_in_ep = {
			[&]() -> auto
			{
				return std::exclusive_ptr<USBInEndpoint>(new(std::nothrow) SAMDUSBInEndpoint{ 2, USBEndpoint::bulk_ep, 64, descs });
			}
		};
		Invokable<std::exclusive_ptr<USBOutEndpoint>()> get_out_ep = {
			[&]() -> auto
			{
				return std::exclusive_ptr<USBOutEndpoint>(new(std::nothrow) SAMDUSBOutEndpoint{ 1, USBEndpoint::bulk_ep, 64, descs });
			}
		};

		const wchar_t* manufacturer_name{ L"Touch Technologies" };
		const wchar_t* product_name{ L"USB Toucher" };
		const wchar_t* serial_number{ L"1337" };
		USBTMCDevice tmc_dev = create_usbtmc488_device(manufacturer_name, product_name,
													   serial_number, cstr, get_out_ep,
													   get_in_ep);
		usb_status->device = &tmc_dev;
		// Configure and enable USB peripheral interrupts.
		set_interrupt_priority(7, 2);
		enable_interrupt(7);

		while (1) {
			if (usb_status->configured == 1) {
				do_wfi();
			}
		}
	}
}
