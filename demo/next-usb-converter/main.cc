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
#include "samd_init.hh"

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

// Translates key numbers to their corresponding Usage IDs.
extern const unsigned char ATkey_usage_translation_table[130];

// Translates key make codes to their corresponding key number.
extern const unsigned char keycode_ATkey_translation_table[0x85];

extern const unsigned char NEXTmodifier_table[7];
extern const unsigned char NEXTkeycode_usage_translation_table[0x51];

bool insert_keycode(char* keys, unsigned char keycode)
{
    int freeIndex = -1;
    for (unsigned int i = 0; i < 6; i++) {
	if (keys[i] == 0) {
	    freeIndex = i;
	}
	else if (keys[i] == keycode) {
	    return false;
	}
    }
    if (freeIndex == -1) {
	return false;
    }
    keys[freeIndex] = keycode;
    return true;
}

bool remove_keycode(char* keys, unsigned char keycode)
{
    for (unsigned int i = 0; i < 6; i++) {
	if (keys[i] == keycode) {
	    keys[i] = 0;
	    return true;
	}
    }
    return false;
}

#define tx_pin PA18
#define rx_pin PA19

void send_reset()
{
    Reg32 data{ SERCOM3 + 0x28 };
    Reg8 intflag{ SERCOM3 + 0x18 };
    // send reset
    data = 0x1ef;
    while (!(intflag & (1 << 1)));
    data = 0x000;
    while (!(intflag & (1 << 1)));
}

void send_query()
{
    Reg32 data{ SERCOM3 + 0x28 };
    Reg8 intflag{ SERCOM3 + 0x18 };
    data = 0x110;
    while (!(intflag & (1 << 1)));
}

unsigned short get_keycode()
{
    union {
	unsigned char bytes[2];
	unsigned short value;
    } keycode;

    Reg32 data{ SERCOM3 + 0x28 };
    Reg8 intflag{ SERCOM3 + 0x18 };
    // Send keyboard query: 0x110
    send_query();
    // Set a timer for 5 ms and wait for response.
    systick_use_cpu_clock();
    systick_set_count(212766);
    systick_reset_value();
    systick_start_counting();
    while (!systick_has_overflowed() && !(intflag & (1 << 2)));
    if (!(intflag & (1 << 2))) {
	send_reset();
	return 0;
    }
    keycode.bytes[0] = data & 0xFF;
    while (!(intflag & (1 << 2)));
    keycode.bytes[1] = data & 0xFF;
    return keycode.value;
}

void set_leds()
{
    Reg32 data{ SERCOM3 + 0x28 };
    Reg8 intflag{ SERCOM3 + 0x18 };

    data = 0x100;
    while (!(intflag & (1 << 1)));
    data = 0x003;
}

extern int usb_hid_class_request_handler(USBControlEndpoint* ep0, char* buf);

extern "C" {
    void init_uart()
    {
#define rxpo (0x3)
#define txpo (0x1)
	// Assign rx_pin to SERCOM3
	set_pin_peripheral(PIN_GROUP(rx_pin), PIN_N(rx_pin), 'D' - 'A');
	set_pin_peripheral(PIN_GROUP(tx_pin), PIN_N(tx_pin), 'D' - 'A');

	// Enable SERCOM3 clock
	Reg32 apbcmask{ PM_APBCMASK };
	apbcmask |= 1 << 5;

	// Route GCLK_MAIN to GCLK_SERCOM3_CORE
	setup_gclk(GCLKGEN0, GCLK_SERCOM3_CORE);

	// Disable and configure SERCOM3
	Reg8 syncbusy{ SERCOM3 + 0x1c };
	Reg32 ctrla{ SERCOM3 };
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
	// CTRLB.SBMODE = 1 (1 stop bit)
	const int SBMODE = 1 << 6;
	// CTRLB.RXEN = 1
	const int RXEN = 1 << 17;
	// CTRLB.TXEN = 1
	const int TXEN = 1 << 16;
	Reg32 ctrlb{ SERCOM3 + 0x04 };
	ctrlb = CHSIZE | SBMODE | RXEN | TXEN;
	// Configure BAUD register
	// As per 25.6.2.3 of the SAMD21 datasheet:
	// BAUD = 65536 * (1 - 16*(f_baud/f_ref))
	// with f_baud = 1/54us ~= 18.519kHz and f_ref = 48MHz
	// BAUD = 65536 * (1 - 16*0.0003858) = 65536 * 0.99382716 ~= 65131
	// fixme
	// the /actual/ clock period is 52.8us.
	Reg16 baud{ SERCOM3 + 0x0c };
	// baud = 65070;
	baud = 65130;
	// Disable all interrupts
	Reg8 intenclr{ SERCOM3 + 0x14 };
	intenclr = 0xFF;
	// Enable UART
	// CTRLA.ENABLE = 1
	ctrla |= 1 << 1;
	while (syncbusy);
    }

	void start()
	{
	    // Setup clocks
	    setup_xosc32k();
	    setup_gclkgen(GCLKGEN2, XOSC32K, 1);
	    setup_gclk(GCLKGEN2, GCLK_DPLL_32K);
	    setup_gclk(GCLKGEN2, GCLK_DPLL);
	    configure_dpll(1500, 0, 2, 1);
	    // Apply main CPU clock and do other initialization.
	    init(FDPLL96M, 1);

	    // Feed GCLKGEN1 with DPLL to clock USB module.
	    setup_gclkgen(GCLKGEN1, FDPLL96M, 1);

	    // Enable GCLK_USB.
	    setup_gclk(GCLKGEN1, GCLK_USB);

	    // USB peripheral initialization.
	    samd_usb_init_usb();
	    samd_usb_enable();

	    samd21_ep_desc* descs = new(std::nothrow) samd21_ep_desc[7];
	    Reg32 descadd{ USB_DESCADD };
	    descadd = reinterpret_cast<unsigned int>(descs);
	    
	    unmask_interrupts();
	    volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

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
			USBStandardDeviceRequest req{ request };
			switch (request[1]) {
			case USBHID_GET_REPORT:
			{
			    char* report = new(std::nothrow) char[8];
			    if (report == nullptr)
			    {
				return -1;
			    }
			    report[0] = static_cast<char>(modifiers_state);
			    memcpy(&report[2], pressed_keys, 6);
			    ep0->queue_response(req.wLength(), report, request[6] < 8 ? request[6] : 8);
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
				ep0->queue_response(request.wLength(), buffer, length);
				delete[] buffer;
				return true;
			    }
			    case USBHID_REPORT_DESCRIPTOR:
			    {
				size_t length = (request.wLength() > USBKeyboardReportDescriptor::size()) ? USBKeyboardReportDescriptor::size() : request.wLength();
				char* desc = new(std::nothrow) char[length];
				if (desc == nullptr)
				{
				    return -1;
				}
				memcpy(desc, kb_descriptor, length);
				ep0->queue_response(request.wLength(), desc, length);
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
								16, 1, descs);
			if (!interrupt_in_ep) {
			    delete new_config;
			    return nullptr;
			}
			// Disable interrupts on interrupt IN endpoint.
			// fixme
			// transmit failures should be handled instead of being ignored.
			Reg8 ep0_intenclr{ USB_EPINTENCLR(0) };
			ep0_intenclr = 0b11001110;
			Reg8 intenclr{ USB_EPINTENCLR(3) };
			intenclr = 0xFF;
			auto report_request_handler =
			    [=]() -> int
				{
				    char* rep = new(std::nothrow) char[8];
				    if (rep == nullptr)
				    {
					return -1;
				    }
				    memcpy(rep, report, 8);
				    // Send the report again.
				    interrupt_in_ep->queue_data(rep, 8);
				    Reg8 intenclr{ USB_EPINTENCLR(3) };
				    intenclr = 0xFF;
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

	    // Reset keyboard and start getting keycodes.
	    get_keycode();
	    send_reset();
	    get_keycode();
	    send_reset();
	    while (1) {
		unsigned short keycode = get_keycode();
		if ((keycode & 0x00FF)==0) {
		    continue;
		}
		bool is_break = (keycode & 0x0080)!=0;
		unsigned char usage = NEXTkeycode_usage_translation_table[keycode&0x007F];
		bool updated = false;
		if (is_break) {
		    updated = remove_keycode(pressed_keys, usage);
		}
		else {
		    updated = insert_keycode(pressed_keys, usage);
		}
		// Handle modifiers
		unsigned char old_modifiers_state = modifiers_state;
		modifiers_state = 0;
		for (unsigned int i = 0; i < sizeof(NEXTmodifier_table)/sizeof(NEXTmodifier_table[0]); i++) {
		    if (keycode & (1 << (i + 8))) {
			modifiers_state |= NEXTmodifier_table[i];
		    }
		}
		if (modifiers_state!=old_modifiers_state) {
		    updated = true;
		}
		if (updated) {
		    char* rep = new(std::nothrow) char[8];
		    if (rep != nullptr)
		    {
			memcpy(rep, report, 8);
			static_cast<USBInEndpoint*>(dev.getEndpoint(3))->queue_data(rep, 8);
		    }
		}
	    }
	}
}
