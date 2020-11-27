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
#include "ra6m1.hh"
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

void init(
    unsigned int modrv0_bits,
    unsigned int moscwtcr_bits,
    unsigned int ick_div_bits,
    unsigned int pcka_div_bits,
    unsigned int pckb_div_bits,
    unsigned int pckc_div_bits,
    unsigned int pckd_div_bits,
    unsigned int bck_div_bits,
    unsigned int fck_div_bits,
    unsigned int pli_div_bits,
    unsigned int pll_mul_bits,
    unsigned int flwt_bits,
    unsigned int sramwtsc_bits);

extern "C" {
	void start()
	{
	    init(
		modrv0_bits<16000000>(),      // oscillator drive bits
		main_clock_wait_time<8163>(), // oscillator wait time
		CLK_DIV(2),                   // ICLK divider
		CLK_DIV(2),                   // PCLKA divider
		CLK_DIV(4),                   // PCLKB divider
		CLK_DIV(4),                   // PCLKC divider
		CLK_DIV(2),                   // PCLKD divider
		CLK_DIV(2),                   // BCLK divider
		CLK_DIV(4),                   // FCLK divider
		PLIDIV_1,                     // PLL divider
		PLLMUL(15),                   // PLL multiplier
		FLWT_ICLK_80M_TO_120M,        // flash wait cycles
		0b11);                        // 1 wait state on SRAM access.

	    // Enable trace pins
	    const unsigned int trace_id = 0b11010;
	    pin_set_peripheral(2, 8, trace_id);
	    pin_set_peripheral(2, 9, trace_id);
	    pin_set_peripheral(2, 10, trace_id);
	    pin_set_peripheral(2, 11, trace_id);
	    pin_set_peripheral(2, 14, trace_id);
	    
	    unmask_interrupts();
	    volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

#define MAX_PACKET_SIZE (8U)
#define VENDOR_ID (0x1209U) // pid.codes VID
#define PRODUCT_ID (1U) // pid.codes Test PID
#define BCD_DEVICE_VERSION (0x0100) // 1.00

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

	    ra6m1_usb_init_usb();

	    USBDeviceFactory cstr = {
		[&](auto&& fact) -> auto
		    {
			USBDevice dev{ RA6M1USBControlEndpoint{ MAX_PACKET_SIZE },
			    device_descriptor, std::move(fact),
			    new(std::nothrow) USBDeviceGenericImpl() };
			return dev;
		    }
	    };
	    Invokable<std::exclusive_ptr<USBInEndpoint>()> get_in_ep = {
		[&]() -> auto
		    {
			return std::exclusive_ptr<USBInEndpoint>(new(std::nothrow) RA6M1USBInEndpoint{ 2, 2, USBEndpoint::bulk_ep, 64, 0 });
		    }
	    };
	    Invokable<std::exclusive_ptr<USBOutEndpoint>()> get_out_ep = {
		[&]() -> auto
		    {
			return std::exclusive_ptr<USBOutEndpoint>(new(std::nothrow) RA6M1USBOutEndpoint{ 1, 1, USBEndpoint::bulk_ep, 64, 0 });
		    }
	    };

	    char idn_string[] = "Touch Technologies,USB Toucher,1337,0\n";
	    
	    USBTMCInterface::in_msg_handler dev_dep_in =
		[=](USBInEndpoint& in_ep,
		   unsigned char MsgId,
		   unsigned char bTag,
		   unsigned int transferSize,
		   bool useTermChar,
		   char termChar) -> int
		    {
			// Handle device-dependent request messages sent from host here.
			unsigned int size = (transferSize > sizeof(idn_string))
			    ? sizeof(idn_string)
			    : transferSize;
			USBTMCDevDepMsgIn msg{ bTag, size, useTermChar, true };
			char* packet = new(std::nothrow) char[size + USBTMCDevDepMsgIn::size()];
			msg.copyTo(packet, USBTMCDevDepMsgIn::size());
			memcpy(packet + USBTMCDevDepMsgIn::size(), idn_string, sizeof(idn_string));
			in_ep.queue_data(packet, size + USBTMCDevDepMsgIn::size());
			// Return 0 to indicate no error.
			return 0;
		    };
	    USBTMCInterface::out_msg_handler dev_dep_out =
		[](unsigned char MsgID,
		   unsigned char bTag,
		   unsigned int transferSize,
		   char* msgBytes) -> int
		    {
			return 0;
		    };

	    const wchar_t* manufacturer_name{ L"Touch Technologies" };
	    const wchar_t* product_name{ L"USB Toucher" };
	    const wchar_t* serial_number{ L"1337" };
	    USBTMCDevice tmc_dev = create_usbtmc488_device(manufacturer_name, product_name,
							   serial_number, &cstr, &get_out_ep,
							   &get_in_ep, &dev_dep_out, &dev_dep_in);
	    usb_status->device = &tmc_dev;
	    ra6m1_usb_enable();

	    while (1) {
		if (usb_status->configured == 1) {
		    ra6m1_wfi();
		}
	    }
	}
}
