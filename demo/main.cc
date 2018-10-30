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
#include "sam.hh"
#include "sam_usb.hh"
#include "usb.hh"
#include "usb_private.hh"
#include "main.hh"
#include "Invokable.hh"
#include "Pointers.hh"
#include "arm.hh"
#include "primitives.hh"
#include "usbhid.hh"
#include "gpio.hh"

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

const unsigned char serial_data_pin = PA03;
const unsigned char serial_clock_pin = PA04;

bool insert_keycode(char* keys, unsigned char keycode)
{
	int freeIndex = -1;
	for (unsigned int i = 0; i < 6; i++) {
		if (keys[i] == 0) {
			freeIndex = i;
		}
		else if (keys[i] == keycode) {
			return true;
		}
	}
	if (freeIndex == -1) {
		return false;
	}
	keys[freeIndex] = keycode;
	return true;
}

void remove_keycode(char* keys, unsigned char keycode)
{
	for (unsigned int i = 0; i < 6; i++) {
		if (keys[i] == keycode) {
			keys[i] = 0;
			return;
		}
	}
}

bool get_pin_state_debounced(unsigned char pin)
{
	bool state = get_pin_state(pin);
	for (int i = 0; i < 5; i++) {
		if (get_pin_state(pin) != state) {
			state = !state;
			i = -1;
		}
	}
	return state;
}

/**
 * @data_pin and @clock_pin are open-collector devices on the keyboard.  When
 * the keyboard is ready to send data, if we are not holding @clock_pin low, it
 * drives @clock_pin low and immediately starts transmitting.
 *
 * Data is transmitted as an 11-bit stream consisting of one start bit, 8 data
 * bits, one parity bit, and one stop bit.  Data bits are transmitted
 * LSb-first.  The start bit is always a 0, the stop bit is always a 1, and the
 * parity bit is odd parity.  Data is sampled on the falling-edge of @clock_pin.
 *
 * Both pins are already configured as inputs, with no pull-up/down resistors
 * connected, meaning the pin is tri-stated.  In this function we don't need to
 * send any data, we just poll the status of the pins.
 */
bool rx_bit(unsigned char data_pin, unsigned char clock_pin)
{
	// Wait for clock line to go low.
	while (get_pin_state_debounced(clock_pin) == true);
	bool state = get_pin_state_debounced(data_pin);
	// Wait for the clock line to go high again.  This ensures that this
	// function isn't called again during the same clock cycle.
	while (get_pin_state_debounced(clock_pin) == false);
	return state;
}

void tx_bit(unsigned char data_pin, unsigned char clock_pin, bool bit)
{
	// Wait for clock line to go low.
	while (get_pin_state_debounced(clock_pin) == true);
	configure_pin_direction(data_pin, !bit);
	// Wait for the clock line to go high again.
	while (get_pin_state_debounced(clock_pin) == false);
}

bool rx_byte(unsigned char data_pin, unsigned char clock_pin, unsigned char& rxd_byte)
{
	// Wait for start bit.
	bool start_bit = rx_bit(data_pin, clock_pin);
	if (start_bit != 0) {
		// Start bit is always 0.
		return false;
	}
	rxd_byte = 0;
	unsigned char ones = 0;
	for (int i = 0; i < 8; i++) {
		if (rx_bit(data_pin, clock_pin) == true) {
			ones++;
			rxd_byte |= 1 << i;
		}
	}
	bool parity_bit = rx_bit(data_pin, clock_pin);
	bool stop_bit = rx_bit(data_pin, clock_pin);
	if (stop_bit == false) {
		// Stop bit is always 1.
		return false;
	}
	if ((ones & 0x1) == parity_bit) {
		// The 8 data bits plus the parity bit always have an odd number of
		// ones.
		// (ones & 0x1) => the 8 data bits has an odd number of one bits
		return false;
	}
    return true;
}

void tx_byte(unsigned char data_pin, unsigned char clock_pin, unsigned char byte)
{
	// Assert I/O inhibit.
	configure_pin_direction(clock_pin, true);
	wait_n_systicks(1000);
	// Assert RTS.  This is the start bit.
	configure_pin_direction(data_pin, true);
	configure_pin_direction(clock_pin, false);
	// Data bits.
	unsigned char ones = 0;
	for (int i = 0; i < 8; i++) {
		bool bit = (byte >> i) & 0x01;
		tx_bit(data_pin, clock_pin, bit);
		if (bit) {
			ones++;
		}
	}
	// Parity bit.
	bool parity_bit = !(ones & 0x1);
	tx_bit(data_pin, clock_pin, parity_bit);
	// Stop bit.
	tx_bit(data_pin, clock_pin, 1);
	// Wait for the line-control bit.
	rx_bit(data_pin, clock_pin);
}

// Send one byte, receive one byte.
unsigned char send_command(unsigned char data_pin, unsigned char clock_pin, unsigned char command)
{
	tx_byte(data_pin, clock_pin, command);
	if (!rx_byte(data_pin, clock_pin, command)) {
		return -1;
	}
	return command;
}

unsigned short get_keycode(unsigned char data_pin, unsigned char clock_pin)
{
	union {
		unsigned char bytes[2];
		unsigned short value;
	} keycode;
	if (!rx_byte(data_pin, clock_pin, keycode.bytes[0])) {
		return -1;
	}
	if (keycode.bytes[0] == 0xF0) {
		// This is a break-code which takes up 2 bytes.
		keycode.bytes[1] = keycode.bytes[0];
		if (!rx_byte(data_pin, clock_pin, keycode.bytes[0])) {
		return -1;
	}
	}
	return keycode.value;
}

extern int usb_hid_class_request_handler(USBControlEndpoint* ep0, char* buf);

extern "C" {
	bool attach_keyboard()
	{
		unsigned char rxd_byte;
		#if 0
		// Wait for keyboard attach.
		while (get_pin_state_debounced(serial_clock_pin) == false);
		// Wait for BAT completion code.
		while (!rx_byte(serial_data_pin, serial_clock_pin, rxd_byte));
		if (rxd_byte != 0xAA) {
			return false;
		}
		#endif
		// Switch to scan-code set 3
		rxd_byte = send_command(serial_data_pin, serial_clock_pin, 0xF0);
		if (rxd_byte != 0xFA) {
			return false;
		}
		rxd_byte = send_command(serial_data_pin, serial_clock_pin, 0x03);
		if (rxd_byte != 0xFA) {
			return false;
		}
		// Disable type-matic.
		rxd_byte = send_command(serial_data_pin, serial_clock_pin, 0xF8);
		if (rxd_byte != 0xFA) {
			return false;
		}
		return true;
	}

	void start()
	{
		unmask_interrupts();
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
		init();
		// Enable PIOA peripheral clock.
		Reg32 pmc_pcer0{ PMC_PCER0 };
		pmc_pcer0 = 1 << 11;
		// Make sure PIO registers are not write-locked.
		Reg32 pio_wpmr{ PIO_WPMR(0) };
		pio_wpmr = 0x50494F00;

		// Configure PORT X for GPIO use.
		enable_pin_pullup(serial_clock_pin);
		set_pin_value(serial_clock_pin, false);
		configure_pin_as_gpio(serial_clock_pin);
		enable_pin_pullup(serial_data_pin);
		set_pin_value(serial_data_pin, false);
		configure_pin_as_gpio(serial_data_pin);

		// Assert I/O inhibit until we've enumerated.
		configure_pin_direction(serial_clock_pin, true);
		
#define MAX_PACKET_SIZE (64U)
#define VENDOR_ID (0U)
#define PRODUCT_ID (0U)
#define BCD_DEVICE_VERSION (0x0100)

//		sam_usb_init(ATMEL_USB_EP0 | ATMEL_USB_EP1);


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
				    new(std::nothrow) AtmelSAMUSBInEndpoint(3, USBEndpoint::ep_type::interrupt_ep,
															8, 1);
				if (!interrupt_in_ep) {
					delete new_config;
					return nullptr;
				}
				auto report_request_handler =
				[=]() -> int
				{
					// Send the report again.
					interrupt_in_ep->sendData(report, 8, true);
					Reg32 udp_idr{ UDP_IDR };
					udp_idr = 1 << 3;
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

		// Configure and enable USB peripheral interrupts.
		USBDevice dev{ create_sam4s_usb_device(desc, configFactory) };
		usb_status->device = &dev;

		Reg32 udp_ier{ UDP_IER };
		Reg32 udp_idr{ UDP_IDR };
		// Disable ep3 interrupt until we have data to send.
		udp_idr = 1 << 3;

		enable_interrupt(34);
		
		// Set up to send the initial report.
		while (!usb_status->configured);
//		static_cast<USBInEndpoint*>(dev.getEndpoint(3))->sendData(report, 8, true);
		// De-assert I/O inhibit and wait for keyboard to signal OK.
		configure_pin_direction(serial_clock_pin, false);
		wait_n_10ms_periods(100);
		if (!attach_keyboard()) {
			while (1);
		}
		while (1) {
			unsigned short keycode = get_keycode(serial_data_pin, serial_clock_pin);
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
		}
	}
}
