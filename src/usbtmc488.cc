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
#include "usbtmc488.hh"
#include "Invokable.hh"
#include "String.h"

char idn_string[] = "Touch Technologies,USB Toucher,0,0\n";

int usbtmc488_dev_dep_out_handler(unsigned char MsgID,
				  unsigned char bTag,
				  unsigned int transferSize,
				  char* msgBytes)
{
	// Handle device-dependent messages sent from host here.
	// Return 0 to indicate no error.
	return 0;
}

int usbtmc488_dev_dep_in_handler(USBInEndpoint& in_ep,
				 unsigned char MsgId,
				 unsigned char bTag,
				 unsigned int transferSize,
				 bool useTermChar,
				 char termChar)
{
	// Handle device-dependent request messages sent from host here.
	unsigned int size = (transferSize > sizeof(idn_string))
		? sizeof(idn_string)
		: transferSize;
	USBTMCDevDepMsgIn msg{ bTag, size, useTermChar, true };
	char* packet = new(std::nothrow) char[size + USBTMCDevDepMsgIn::size()];
	msg.copyTo(packet, USBTMCDevDepMsgIn::size());
	memcpy(packet + USBTMCDevDepMsgIn::size(), idn_string, sizeof(idn_string));
	in_ep.sendData(packet, size + USBTMCDevDepMsgIn::size());
	delete[] packet;
	// Return 0 to indicate no error.
	return 0;
}

struct usbtmc_setup_request_handler : public USBSetupRequestHandler
{
protected:
	const size_t manufacturer_name_size;
	const size_t product_name_size;
	const size_t serial_number_size;
	const wchar_t* manufacturer_name_;
	const wchar_t* product_name_;
	const wchar_t* serial_number_;

public:
	virtual int get_status(USBDevice&, char*) { return 0; }
	virtual int usb_clear_feature(USBDevice&, char*) { return 0; }
	virtual int usb_set_feature(USBDevice&, char*) { return 0; }
	virtual int usb_set_address(USBDevice&, char*) { return 0; }
	virtual int usb_get_descriptor(USBDevice& device, char* buf)
	{
		const char STRING = 3;
		char descriptor0[4] = {
			4,
			STRING,
			0x09,
			0x04
		};

		// this handler only handles string descriptors
		// higher-level logic should take care of the rest
		if (buf[3] != STRING) {
			return 0;
		}

		unsigned int length = (buf[7] * 0x100) + buf[6];

		char *bytes = new(std::nothrow) char[length];
		// Assume (length >= 2)
		bytes[1] = STRING;
		switch (buf[2] & 0xff) {
		case 0:
			device.ep0.sendData(descriptor0, 4);
			break;
		case 4:
			bytes[0] = 2 + manufacturer_name_size;
			if (bytes[0] < length) {
				length = bytes[0];
			}
			memcpy(&bytes[2], manufacturer_name_, length - 2);
			device.ep0.sendData(bytes, length);
			break;
		case 2:
			bytes[0] = 2 + product_name_size;
			if (bytes[0] < length) {
				length = bytes[0];
			}
			memcpy(&bytes[2], product_name_, length - 2);
			device.ep0.sendData(bytes, length);
			break;
		case 3:
			bytes[0] = 2 + serial_number_size;
			if (bytes[0] < length) {
				length = bytes[0];
			}
			memcpy(&bytes[2], serial_number_, length - 2);
			device.ep0.sendData(bytes, length);
			break;
		default:
			break;
		}
		delete bytes;
		return true;
	}
	virtual int usb_set_descriptor(USBDevice&, char*) { return 0; }
	virtual int usb_get_configuration(USBDevice&, char*) { return 0; }
	virtual int usb_set_configuration(USBDevice&, char*) { return 0; }
	virtual int usb_get_interface(USBDevice&, char*) { return 0; }
	virtual int usb_set_interface(USBDevice&, char*) { return 0; }
	usbtmc_setup_request_handler(const wchar_t* manufacturer_name,
								 const wchar_t* product_name,
								 const wchar_t* serial_number)
		: USBSetupRequestHandler()
		, manufacturer_name_size{ wcslen(manufacturer_name) * sizeof(wchar_t) }
		, product_name_size{ wcslen(product_name) * sizeof(wchar_t) }
		, serial_number_size{ wcslen(serial_number) * sizeof(wchar_t) }
		, manufacturer_name_{ manufacturer_name }
		, product_name_{ product_name }
		, serial_number_{ serial_number } {}
};

USBTMC488Capabilities interface_capabilities = {
	{
		false,
		false,
		false,
		false
	},     // base_caps
	false, // isUSB488_2
	false, // acceptsGoToLocal
	false, // acceptsTrigMsg
	false, // understandsMandatorySCPI
	false, // sr1Capable
	false, // rl1Capable
	false  // dt1Capable
};

USBTMCDevice create_usbtmc488_device(const wchar_t* manufacturer_name,
									 const wchar_t* product_name,
									 const wchar_t* serial_number,
									 Invokable<USBDevice(Invokable<USBConfiguration*(unsigned char)>&&)>& cstr,
									 Invokable<std::exclusive_ptr<USBOutEndpoint>()>& get_out_ep,
									 Invokable<std::exclusive_ptr<USBInEndpoint>()>& get_in_ep)
{
	USBDevice dev = cstr([&](unsigned char n) -> USBConfiguration*
			     {
				     if (n != 1) {
					     return nullptr;
				     }
				     USBConfiguration* new_config =
					     new(std::nothrow) USBConfiguration(n);
				     if (!new_config) {
					     return nullptr;
				     }

				     // Construct the USBTMC interface.
				     // This will handle sending/receiving of all messages to/from
				     // the host.
				     USBTMCInterface* iface =
					     new(std::nothrow) USBTMCInterface(USBTMC_USB488_interface,
									       &interface_capabilities.base_caps,
										   get_out_ep(),
										   get_in_ep());
				     if (!iface) {
					     delete new_config;
					     return nullptr;
				     }

				     iface->addDevDepMsgOutHandler(usbtmc488_dev_dep_out_handler);
				     iface->addDevDepMsgInReqHandler(usbtmc488_dev_dep_in_handler);

				     new_config->interfaces.push_back(iface);
				     return new_config;
			     });
	USBTMCDevice tmc_dev{ std::move(dev) };

	usbtmc_setup_request_handler* handler =
		new(std::nothrow) usbtmc_setup_request_handler{ manufacturer_name,
														product_name,
														serial_number };
	tmc_dev.addSetupRequestHandler(handler);
	return tmc_dev;
}
