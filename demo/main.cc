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
#include "usb.hh"
#include "usb_private.hh"
#include "main.hh"
#include "Invokable.hh"
#include "Pointers.hh"
#include "arm.hh"
#include "primitives.hh"
#include "usbhid.hh"
#include "gpio.hh"
#include "samd_usb.hh"

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

// Translates key numbers to their corresponding Usage IDs.
extern const unsigned char ATkey_usage_translation_table[130];

// Translates key make codes to their corresponding key number.
extern const unsigned char keycode_ATkey_translation_table[0x85];

unsigned short get_keycode()
{
	union {
		unsigned char bytes[2];
		unsigned short value;
	} keycode;

	Reg32 data{ SERCOM5 + 0x28 };
	Reg8 intflag{ SERCOM5 + 0x18 };
	// Send keyboard query: 0x110
	data = 0x110;
	// Set a timer for 5 ms and wait for response.
	set_systick_rvr(systick_get_tenms());
 try_again:
	reset_systick();
	while (!(get_systick_cvr() & 0x80000000) && !(intflag & (1 << 2)));
	if (!(intflag & (1 << 2))) {
		goto try_again;
	}
	keycode.bytes[0] = data & 0xFF;
	while (!(intflag & (1 << 2)));
	keycode.bytes[1] = data & 0xFF;
	return keycode.value;
}

extern int usb_hid_class_request_handler(USBControlEndpoint* ep0, char* buf);

extern "C" {

	void init_uart()
	{
#define tx_pin PB00
#define rx_pin PB01
#define txpo (0x1)
#define rxpo (0x3)
		// Assign rx_pin and tx_pin to SERCOM
		set_pin_peripheral(PIN_GROUP(rx_pin), PIN_N(rx_pin), 'D' - 'A');
		set_pin_peripheral(PIN_GROUP(tx_pin), PIN_N(tx_pin), 'D' - 'A');

		// Enable SERCOM0 clock
		Reg32 apbcmask{ PM_APBCMASK };
		apbcmask |= 1 << 2;

		// Route GCLK_MAIN to GCLK_SERCOM5_CORE
		setup_gclk(GCLKGEN0, GCLK_SERCOM5_CORE);

		// Disable and configure SERCOM5
		Reg8 syncbusy{ SERCOM5 + 0x1c };
		Reg32 ctrla{ SERCOM5 };
		// CTRLA.SWRST = 1
		ctrla = 1;
		while (syncbusy & 1);
		// CTRLA.MODE = 1 (internal clock)
		const int MODE = 1 << 2;
		// CTRLA.CMODE = 0 (asynchronous)
		// Configure CTRLA.RXPO
		const int RXPO = rxpo << 20;
		// Configure CTRLA.TXPO
		const int TXPO = txpo << 16;
		// Configure CTRLA.DORD for LSB-first transfers
		const int DORD = 1 << 30;
		ctrla = MODE | RXPO | TXPO | DORD;
		// Configure CTRLB.CHSIZE for 9-bit characters
		const int CHSIZE = 1;
		// CTRLB.SBMODE = 0 (1 stop bit)
		// CTRLB.RXEN = 1
		const int RXEN = 1 << 17;
		// CTRLB.TXEN = 1
		const int TXEN = 1 << 16;
		Reg32 ctrlb{ SERCOM5 + 0x04 };
		ctrlb = CHSIZE | RXEN | TXEN;
		// Configure BAUD register
		// As per 25.6.2.3 of the SAMD21 datasheet:
		// BAUD = 65536 * (1 - 16*(f_baud/f_ref))
		// with f_baud = 1/54us ~= 18.519kHz and f_ref = 48MHz
		// BAUD = 65536 * (1 - 16*0.0003858) = 65536 * 0.99382716 ~= 65131
		Reg16 baud{ SERCOM5 + 0x0c };
		baud = 65131;
		// Disable all interrupts
		Reg8 intenclr{ SERCOM5 + 0x14 };
		intenclr = 0xFF;
		// Enable UART
		// CTRLA.ENABLE = 1
		ctrla |= 1 << 1;
		while (syncbusy);
	}
	
	void start()
	{
		unmask_interrupts();
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
		init();

		samd_usb_init_usb();
		samd_usb_enable();
		samd21_ep_desc* descs = new(std::nothrow) samd21_ep_desc[7];
		Reg32 descadd{ USB_DESCADD };
		descadd = reinterpret_cast<unsigned int>(descs);
		
#define MAX_PACKET_SIZE (64U)
#define VENDOR_ID (0U)
#define PRODUCT_ID (0U)
#define BCD_DEVICE_VERSION (0x0100)

#define ERROR_ROLL_OVER 0x01
		char report[8]{};
		char*  pressed_keys{ &report[2] };
		char& modifiers_state(report[0]);

		auto hid_class_request_handler =
			[&](USBControlEndpoint* ep0, char* request) -> int
			{
				switch (request[1]) {
				case USBHID_GET_REPORT:
				{
					char report[8]{};
					report[0] = static_cast<char>(modifiers_state);
					memcpy(&report[2], pressed_keys, 6);
					ep0->sendData(report, request[6] < 8 ? request[6] : 8);
					return true;
				}
				default:
				return usb_hid_class_request_handler(ep0, request);
				}
			};

		auto interface_request_handler =
			[&](USBControlEndpoint* ep0, char* buf) -> int
			{
				USBStandardDeviceRequest request{ buf };
				if (request.bRequest() == GET_DESCRIPTOR) {
					unsigned char descriptorType = (request.wValue() & 0xFF00) >> 8;
					switch (descriptorType) {
					case USBHID_CLASS_DESCRIPTOR:
					{
						USBHIDDescriptor hid_desc{ 0x111, USBHID_CC_US, 1,
								USBHID_REPORT_DESCRIPTOR, USBKeyboardReportDescriptor::size() };
						size_t length = (request.wLength() > hid_desc.size()) ? hid_desc.size() : request.wLength();
						char* buffer = new(std::nothrow) char[length];
						hid_desc.copyTo(buffer, length);
						ep0->sendData(buffer, length);
						delete[] buffer;
						return true;
					}
					case USBHID_REPORT_DESCRIPTOR:
					{
						size_t length = (request.wLength() > USBKeyboardReportDescriptor::size()) ? USBKeyboardReportDescriptor::size() : request.wLength();
						ep0->sendData(kb_descriptor, length);
						return true;
					}
					case USBHID_PHYSICAL_DESCRIPTOR:
					default:
						return false;
					}
				}
				return usb_hid_class_request_handler(ep0, buf);
			};
		
		USBDeviceDescriptor desc{ 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 1 };
		
		auto configFactory =
			[&](unsigned char configN) -> USBConfiguration*
			{
				if (configN != 1) {
					return nullptr;
				}
				USBConfiguration* new_config =
				    new(std::nothrow) USBConfiguration(configN);
				if (!new_config) {
					return nullptr;
				}
				USBInEndpoint* interrupt_in_ep =
				    new(std::nothrow) SAMDUSBInEndpoint(3, USBEndpoint::ep_type::interrupt_ep,
														8, 1, descs);
				if (!interrupt_in_ep) {
					delete new_config;
					return nullptr;
				}
				auto report_request_handler =
				[=]() -> int
				{
					// Send the report again.
					interrupt_in_ep->sendData(report, 8, true);
					// Reg32 udp_idr{ UDP_IDR };
					// udp_idr = 1 << 3;
					return true;
				};

				USBInterface* iface =
				new(std::nothrow) USBInterface(USBHID_CLASS,
											   USBHID_BOOT_INTERFACE_SUBCLASS,
											   USBHID_PROTOCOL_KEYBOARD);
				if (!iface) {
					delete new_config;
					delete interrupt_in_ep;
					return nullptr;
				}

				iface->addClassRequestHandler(interface_request_handler);
				iface->addClassRequestHandler(hid_class_request_handler);
				iface->addInEndpoint(interrupt_in_ep, report_request_handler);
				USBHIDDescriptor hid_desc{ 0x111, USBHID_CC_US, 1,
						USBHID_REPORT_DESCRIPTOR, USBKeyboardReportDescriptor::size() };
				char* buf = new(std::nothrow) char[hid_desc.size()];
				hid_desc.copyTo(buf, hid_desc.size());
				Vector<char> vec{ hid_desc.size() };
				for (unsigned int i = 0; i < vec.size(); i++) {
					vec[i] = buf[i];
				}
				delete[] buf;
				iface->classDescriptors = std::move(vec);
				new_config->interfaces.push_back(iface);
				return new_config;
			};

		init_uart();
		
		// Configure and enable USB peripheral interrupts.
		USBDevice dev{ SAMDUSBControlEndpoint{ 0, 64, descs },
				       desc, std::move(configFactory),
					   new(std::nothrow) USBDeviceGenericImpl() };
		usb_status->device = &dev;

		enable_interrupt(7);
		
		// Set up to send the initial report.
		while (!usb_status->configured);
		wait_n_10ms_periods(100);
		while (1) {
			unsigned short keycode = get_keycode();
#if 0
			// First determine if this is a modifier key or not.
			unsigned char make_code = keycode & 0x00FF;
			unsigned char is_break = (keycode & 0xFF00) == 0xF000;
			switch (make_code) {
			case 0x11:  // Left CTRL
				modifiers_state = (modifiers_state & ~is_break) | !is_break;
				break;
			case 0x12:  // Left SHIFT
				modifiers_state = (modifiers_state & ~(is_break << 1)) | (!is_break << 1);
				break;
			case 0x19:  // Left ALT
				modifiers_state = (modifiers_state & ~(is_break << 2)) | (!is_break << 2);
				break;
			case 0x58:  // Right CTRL
				modifiers_state = (modifiers_state & ~(is_break << 4)) | (!is_break << 4);
				break;
			case 0x59:  // Right SHIFT
				modifiers_state = (modifiers_state & ~(is_break << 5)) | (!is_break << 5);
				break;
			case 0x39:  // Right ALT
				modifiers_state = (modifiers_state & ~(is_break << 6)) | (!is_break << 6);
				break;
			default:
				unsigned char usage = ATkey_usage_translation_table[keycode_ATkey_translation_table[make_code]];
				if (is_break) {
					remove_keycode(pressed_keys, usage);
				}
				else {
					insert_keycode(pressed_keys, usage);
				}
			}
			static_cast<USBInEndpoint*>(dev.getEndpoint(3))->sendData(report, 8, true);
#endif
		}
	}
}
