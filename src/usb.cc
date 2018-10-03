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
#include "main.hh"
#include "usb.hh"
#include "usb_private.hh"
#include "mm.hh"

/* USB states */
enum {
	ATTACHED_STATE,
	POWERED_STATE,
	DEFAULT_STATE,
	ADDRESS_STATE,
	CONFIGURED_STATE,
	SUSPENDED_STATE
};

int USBEndpoint::sendData(char* data, unsigned int length, bool buffered)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
	// FIXME: buffering does not currently work
	/* ZLP fast-path */
	if (length == 0) {
		return send_data(data, 0, buffered);
	}
	for (unsigned int size = 0; size < length; size += max_packet_size) {
		unsigned int tx_size{ (length - size > max_packet_size) ? max_packet_size : (length - size) };
		int status{ send_data(data + size, tx_size, buffered) };
		if (status < 0) {
			return status;
		}
		usb_status->eps[ep_number].total_bytes -= tx_size;
	}
	
	return 0;
}

int USBEndpoint::sendZLP(bool wait)
{
	char dummy;
	// FIXME: wait == 0 currently not supported
	return sendData(&dummy, 0, !wait);
}

char* USBEndpoint::receiveData(unsigned int& length)
{
	return read_data(length);
}

char* USBEndpoint::receiveSetup(unsigned int& length)
{
	return read_setup(length);
}

USBDevice::USBDevice(USBControlEndpoint ep0_, USBDeviceDescriptor descriptor,
					 Invokable<USBConfiguration*(unsigned char)> configurationFactory,
					 USBDeviceImpl* privImpl)
	: ep0{ std::move(ep0_) }, // ep0{ 0, descriptor.get<4>() /* bMaxPacketSize0 */ },
	  configFactory{ std::move(configurationFactory) },
	  setup_handlers{ 1 },
	  dev_descriptor{ std::move(descriptor) },
	  impl{ privImpl }
{
	setup_handlers[0] = new(std::nothrow) DefaultSetupRequestHandler{ };
}

void USBDevice::reset() {
	if (current_config) {
		delete current_config;
	}
	if (configFactory) {
		// Endpoints are instantiated here.
		current_config = configFactory(1);
	}
	ep0.reset();
}

int USBDevice::DefaultSetupRequestHandler::get_status(USBDevice& device, char*)
{
	unsigned short status{};
	return !device.ep0.sendData(reinterpret_cast<char*>(&status), sizeof(status)) ? true : -1;
}

int USBDevice::DefaultSetupRequestHandler::usb_clear_feature(USBDevice&, char*)
{
	return false;
}

int USBDevice::DefaultSetupRequestHandler::usb_set_feature(USBDevice&, char*)
{
	return false;
}

int USBDevice::DefaultSetupRequestHandler::usb_set_address(USBDevice& device, char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
	USBStandardDeviceRequest request{ buf };
	const unsigned short& address{ request.wValue() };
	const unsigned short& wIndex{ request.wIndex() };
	const unsigned short& wLength{ request.wLength() };

	if (address > 127 || wIndex != 0 || wLength != 0) {
		return -1;
	}

	if (address == 0) {
		usb_status->state = DEFAULT_STATE;
		device.ep0.sendZLP();
		device.setAddress(0);
		// ordering dependency here with above functions needing address != 0?
		usb_status->address = 0;
		return true;
	}

	usb_status->address = address;
	usb_status->state = ADDRESS_STATE;

	/* make sure that we WAIT for ZLP to be sent before changing address */
	device.ep0.sendZLP();

	device.setAddress(address);

	return true;
}

int USBDevice::DefaultSetupRequestHandler::usb_get_descriptor(USBDevice& device, char* buf)
{
	USBGetDescriptorRequest request{ buf };
	const unsigned char& descriptorType{ request.getDescriptorType() };
	const unsigned char& descriptorIndex{ request.getDescriptorIndex() };
	const unsigned short& descriptorSize{ request.wLength() };
	USBControlEndpoint& ept(device.ep0);

	switch (descriptorType) {
	case DEVICE:
	{
		unsigned int length;
		if (device.dev_descriptor.size() > descriptorSize) {
			length = descriptorSize;
		}
		else {
			length = device.dev_descriptor.size();
		}
		char descriptor[device.dev_descriptor.size()];
		device.dev_descriptor.copyTo(descriptor, length);
		ept.sendData(reinterpret_cast<char*>(descriptor), length);
		break;
	}
	case CONFIGURATION:
	{
		if (descriptorIndex >= device.dev_descriptor.get<11>() /* bNumConfigurations */) {
			ept.stall();
			return true;
		}
		unsigned int total_length{ USBConfigurationDescriptor::size()
				+ device.current_config->interfaces.size() * USBInterfaceDescriptor::size()
				+ ([&]() -> unsigned short
			{
				unsigned short num_endpoints{};
				for (unsigned int i = 0; i < device.current_config->interfaces.size(); i++) {
					num_endpoints += device.current_config->interfaces[i]->inEndpoints.size();
					num_endpoints += device.current_config->interfaces[i]->outEndpoints.size();
				}
				return num_endpoints;
			})() * USBEndpointDescriptor::size()
				};
						
		unsigned int max_size{ total_length > descriptorSize ? descriptorSize : total_length };
		char* data_buffer{ new(std::nothrow) char[max_size] };
		if (!data_buffer) {
			goto error;
		}

		auto copy_descriptor = [&](const auto& descriptor, unsigned int& current_offset) -> int {
			if (current_offset >= descriptorSize) {
				return 0;
			}
			unsigned int copy_size{ ((current_offset + descriptor.size()) <= descriptorSize) ? descriptor.size() : (descriptorSize - current_offset) };
			if (descriptor.copyTo(&data_buffer[current_offset], copy_size) < 0) {
				return -1;
			}
			current_offset += copy_size;
			return 0;
		};
		
		unsigned int offset{};
		if (copy_descriptor(USBConfigurationDescriptor{ (unsigned short)(total_length),
						(unsigned char)(device.current_config->interfaces.size()),
						device.current_config->config_number, 0, 0x80, 225 }, offset) < 0){
			goto error;
		}
		auto copy_ep_descriptor = [&](USBEndpoint* ep) mutable -> int {
			if (copy_descriptor(USBEndpointDescriptor{ ep->ep_number, (unsigned char)(ep->type),
							ep->max_packet_size, 0 }, offset) < 0) {
				return -1;
			}
			return 0;
		};
		for (unsigned char i = 0; i < device.current_config->interfaces.size(); i++) {
			USBInterface* iface{ device.current_config->interfaces[i] };
			if (copy_descriptor(USBInterfaceDescriptor{ i, 0, (unsigned char)(iface->inEndpoints.size() + iface->outEndpoints.size()),
							iface->bInterfaceClass,	iface->bInterfaceSubClass, iface->bInterfaceProtocol, 0 }, offset) < 0) {
				goto error;
			}
			for (unsigned int j = 0; j < iface->inEndpoints.size(); j++) {
				if (copy_ep_descriptor(iface->inEndpoints[j]) < 0) {
					goto error;
				}
			}
			for (unsigned int j = 0; j < iface->outEndpoints.size(); j++) {
				if (copy_ep_descriptor(iface->outEndpoints[j]) < 0) {
					goto error;
				}
			}
		}

		ept.sendData(reinterpret_cast<char*>(data_buffer), max_size);
		delete[] data_buffer;
		
		break;
	}
	default:
		return false;
	}

	return true;

error:
	return -1;
}

int USBDevice::DefaultSetupRequestHandler::usb_set_descriptor(USBDevice&, char*)
{
	return false;
}

int USBDevice::DefaultSetupRequestHandler::usb_get_configuration(USBDevice& device, char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
	char configuration{ device.current_config->config_number };

	if (usb_status->state == DEFAULT_STATE) {
		return -1;
	}
	else if (usb_status->state == ADDRESS_STATE) {
		configuration = 0;
	}
	else if (usb_status->state != CONFIGURED_STATE) {
		return -1;
	}

	return !device.ep0.sendData(&configuration, 1) ? true : -1;
}

/* SetConfiguration request:
 *   Selects a device configuration as reported in the device's configuration descriptors.
 *   A configuration consists of one or more interfaces, which can operate simultaneously.
 */
int USBDevice::DefaultSetupRequestHandler::usb_set_configuration(USBDevice& device, char* buf)
{
	/* TODO: break this register access out into something
	   device-independent */
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	if (usb_status->state == DEFAULT_STATE) {
		return -1;
	}

	USBStandardDeviceRequest request{ buf };
	
	const unsigned short& wValue{ request.wValue() };
	const unsigned short& wIndex{ request.wIndex() };
	const unsigned short& wLength{ request.wLength() };

	if (wIndex != 0 || wLength != 0) {
		device.ep0.stall();
		return -1;
	}

	if (wValue == 0) {
		usb_status->state = ADDRESS_STATE;
	}
	else {
		if (!device.configFactory) {
			return -1;
		}
		delete device.current_config;
		device.current_config = device.configFactory(wValue);
		if (!device.current_config) {
			return -1;
		}
		usb_status->state = CONFIGURED_STATE;
	}
	
	if (usb_status->state == CONFIGURED_STATE) {
		device.setConfigured(true);
	}
	device.ep0.sendZLP();
	usb_status->configured = 1;

	return true;
}

int USBDevice::DefaultSetupRequestHandler::usb_get_interface(USBDevice&, char*)
{
	return false;
}

int USBDevice::DefaultSetupRequestHandler::usb_set_interface(USBDevice&, char*)
{
	return false;
}

int USBDevice::in_token_received(unsigned int ep)
{
	return 0;
}

int USBDevice::out_token_received(unsigned int ep)
{
	char* data{};
	unsigned int length{};
	
	// find out if this is an IN or OUT endpoint
	USBInterface& iface(*current_config->interfaces[0]);  // FIXME: there should be a pointer to the currently selected interface
	for (unsigned int i = 0; i < iface.outEndpoints.size(); i++) {
		if (iface.outEndpoints[i]->ep_number == ep) {
			// this is an OUT endpoint, so we are receiving data
			data = iface.outEndpoints[i]->receiveData(length);
			if (!length || !data) {
				return -1;
			}
			break;
		}
	}

	for (unsigned int i = 0; i < iface.outTokenHandlers.size(); i++) {
		if (iface.outTokenHandlers[i].first == ep) {
			auto& handler(iface.outTokenHandlers[i].second);
			int status{ handler(data, length) };
			if (status < 0) {
				return status;
			}
			else if (status > 0) {
				break;
			}
		}
	}

	delete[] data;
	
	return 0;
}

extern "C" {
	void usb_in_token(unsigned int ep)
	{
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
		USBDevice* device{ usb_status->device };

		device->in_token_received(ep);

		// FIXME: This path is not taken for some reason.
		// Interrupt not raised for EP 1?
	}

	void usb_out_token(unsigned int ep)
	{
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

		USBDevice* device{ usb_status->device };
		device->out_token_received(ep);
	}
}


/**
 * USB HID support functions
 */
struct USBReportDescriptor
{
	unsigned char descriptor[63] = {
		0x05, 0x01,
		0x09, 0x06,
		0xa1, 0x01,
		0x05, 0x07,
		0x19, 0xe0,
		0x29, 0xe7,
		0x15, 0x00,
		0x25, 0x01,
		0x75, 0x01,
		0x95, 0x08,
		0x81, 0x02,
		0x95, 0x01,
		0x75, 0x08,
		0x81, 0x03,
		0x95, 0x05,
		0x75, 0x01,
		0x05, 0x08,
		0x19, 0x01,
		0x29, 0x05,
		0x91, 0x02,
		0x95, 0x01,
		0x75, 0x03,
		0x91, 0x03,
		0x95, 0x06,
		0x75, 0x08,
		0x15, 0x00,
		0x25, 0x65,
		0x05, 0x07,
		0x19, 0x00,
		0x29, 0x65,
		0x81, 0x00,
		0xc0
	};

	static constexpr unsigned int size()
	{
		return sizeof(descriptor);
	}
};

int usb_get_report(volatile char* buf)
{
	char report[8] {};

	if (buf[2] != 0 || buf[3] != 1 || buf[4] != 0 || buf[5] != 0) {
		return -1;
	}

	unsigned int size{ buf[6] < sizeof(report) ? buf[6] : sizeof(report) };
	return getUSBEndpoint<USBControlEndpoint>(0).sendData(report, size);
}

int usb_get_idle(volatile char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
	char idle(usb_status->idle);

	if (buf[2] != 0 || buf[3] != 0) {
		return -1;
	}

	return getUSBEndpoint<USBControlEndpoint>(0).sendData(&idle, 1);
}

int usb_get_protocol(volatile char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
	char protocol(usb_status->protocol);

	if (buf[2] != 0 || buf[3] != 0 || buf[4] != 0 || buf[5] != 0 || buf[6] != 1 || buf[7] != 0) {
		return -1;
	}

	return getUSBEndpoint<USBControlEndpoint>(0).sendData(&protocol, 1);
}

int usb_set_report(volatile char* buf)
{
	return getUSBEndpoint<USBControlEndpoint>(0).sendZLP();
}

int usb_set_idle(volatile char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	if (buf[2] != 0) {
		return -1;
	}

	usb_status->idle = (unsigned char)(buf[3]);

	return getUSBEndpoint<USBControlEndpoint>(0).sendZLP();
}

int usb_set_protocol(volatile char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	if (buf[4] != 0 || buf[5] != 0 || buf[6] != 0 || buf[7] != 0 || buf[3] != 0) {
		return -1;
	}

	usb_status->protocol = buf[2];

	return getUSBEndpoint<USBControlEndpoint>(0).sendZLP();
}

int usb_reserved(volatile char* buf) { return 0; }

int USBDevice::setup_token_received(char* buffer, unsigned int size)
{
	if (!buffer || size < USBStandardDeviceRequest::size()) {
		ep0.stall();
		return -1;
	}

	USBStandardDeviceRequest req{ buffer };

	const unsigned int& bmRequestType{ req.bmRequestType() };
	const unsigned int& bRequest{ req.bRequest() };
	char type{ char((bmRequestType & 0x60) >> 5) };

	int (USBSetupRequestHandler::* usb_functions[13])(USBDevice&, char*) = {
		&USBSetupRequestHandler::get_status,
		&USBSetupRequestHandler::usb_clear_feature,
		nullptr,
		&USBSetupRequestHandler::usb_set_feature,
		nullptr,
		&USBSetupRequestHandler::usb_set_address,
		&USBSetupRequestHandler::usb_get_descriptor,
		&USBSetupRequestHandler::usb_set_descriptor,
		&USBSetupRequestHandler::usb_get_configuration,
		&USBSetupRequestHandler::usb_set_configuration,
		&USBSetupRequestHandler::usb_get_interface,
		&USBSetupRequestHandler::usb_set_interface,
		nullptr
	};

	const char STANDARD = 0;
	const char CLASS = 1;
	
	switch (type) {
	case STANDARD:
		if (bRequest > 12 || usb_functions[bRequest] == nullptr) {
			goto error;
		}
		for (unsigned int i = 0; i < setup_handlers.size(); i++) {
			int handled = (setup_handlers[i]->*usb_functions[bRequest])(*this, buffer);
			if (handled < 0) {
				goto error;
			}
			else if (handled > 0) {
				break;
			}
		}
		break;
	case CLASS:
		for (unsigned int i = 0; i < class_handlers.size(); i++) {
			int handled = class_handlers[i](&ep0, buffer);
			if (handled < 0) {
				goto error;
			}
			else if (handled > 0) {
				break;
			}
		}
		break;
	default:
		goto error;
	}

	return 0;
error:
	ep0.stall();
	return -1;
}

extern "C" {
	int usb_setup_token(unsigned int ep)
	{
		volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
		USBControlEndpoint& ept(usb_status->device->ep0);
		
		unsigned int length;
		char* buffer{ ept.receiveSetup(length) };

		int status{ usb_status->device->setup_token_received(buffer, length) };
		delete[] buffer;
		if (status < 0) {
			return -1;
		}

		return 0;
	}

	void init_usb()
	{
		struct usb_status_info* usb_status{ const_cast<usb_status_info*>(getUSBStatusInfo()) };
		memset(reinterpret_cast<char*>(usb_status), 0, sizeof(*usb_status));
		usb_status->protocol = 1;
		usb_status->rts = 0;
		__usb_init_usb();
	}

	void usb_ep_isr(unsigned int ep)
	{
		__usb_ep_isr(ep);
	}

	void usb_enable()
	{
		__usb_enable();
	}
}
