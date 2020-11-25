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

int USBEndpoint::endpointRequest(USBControlEndpoint* ep0, char* request)
{
	USBStandardDeviceRequest req{ request };
	// Examine bmRequestType to determine if this is a standard, class,
	// or vendor request.
	switch ((req.bmRequestType() & 0x60) >> 5) {
	case 0:  // Standard
		// Only 3 standard requests are allowed for endpoints:
		// * CLEAR_FEATURE (1)
		// * GET_STATUS    (0)
		// * SET_FEATURE   (3)
		// Examine bRequest to determine type of request.
		switch (req.bRequest()) {
		case 0:  // GET_STATUS
		{
		    // For an endpoint, the GET_STATUS response reports
		    // a single status bit:
		    // D15..D1: Reserved
		    // D0: Halt
		    char* response = new(std::nothrow) char[2];
		    if (response == nullptr)
		    {
			return -1;
		    }
		    response[0] = 0;
		    response[1] = stalled ? 1 : 0;
		    ep0->queue_response(req.wLength(), response, 2);
		    return true;
		}
		case 1:  // CLEAR_FEATURE
		case 3:  // SET_FEATURE
			// For an endpoint, the only feature that can be set
			// or cleared is ENDPOINT_HALT (0).
			if (req.wValue() != 0) {
				return -1;
			}
			if (req.bRequest() == 3) {
				stall();
			}
			else {
				unstall();
			}
			return true;
		default:
			return -1;
		}
	case 1:  // Class
		for (int i = 0; i < classRequestHandlers.size(); i++) {
			int status{ classRequestHandlers[i](ep0, request) };
			if (status != false) {
				return status;
			}
		}
		return false;
	case 2:  // Vendor
	case 3:  // Reserved
	default:
		return -1;
	}
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
    unsigned short* status = new(std::nothrow) unsigned short[1];
    if (status == nullptr)
    {
	return -1;
    }
    status[0] = 0;
    device.ep0.queue_data(reinterpret_cast<char*>(status), sizeof(*status));
    return true;
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
		device.ep0.complete_setup(request);
		device.setAddress(0);
		// ordering dependency here with above functions needing address != 0?
		usb_status->address = 0;
		return true;
	}

	usb_status->address = address;
	usb_status->state = ADDRESS_STATE;

	/* make sure that we WAIT for ZLP to be sent before changing address */
	device.ep0.complete_setup(request);

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
		char* descriptor = new(std::nothrow) char[device.dev_descriptor.size()];
		if (descriptor == nullptr)
		{
		    goto error;
		}
		device.dev_descriptor.copyTo(descriptor, length);
		ept.queue_response(request.wLength(), descriptor, length);
		break;
	}
	case CONFIGURATION:
	{
		if (descriptorIndex >= device.dev_descriptor.get<11>() /* bNumConfigurations */) {
			ept.stall();
			return true;
		}
		unsigned int total_length{};
		// One configuration descriptor per configuration.
		total_length += USBConfigurationDescriptor::size();
		for (unsigned int i = 0; i < device.current_config->interfaces.size(); i++) {
			USBInterface* iface{ device.current_config->interfaces[i] };
			// One interface descriptor per interface of the currently-selected
			// configuration.
			total_length += USBInterfaceDescriptor::size();
			// Each interface can have zero or more associated class
			// descriptors.
			total_length += iface->classDescriptors.size();
			// Each interface can have zero or more associated endpoints.
			unsigned short num_endpoints = iface->inEndpoints.size() + iface->outEndpoints.size();
			total_length += num_endpoints * USBEndpointDescriptor::size();
		}
						
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
							ep->max_packet_size, ep->interval_ms }, offset) < 0) {
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
			unsigned int copy_size{ ((offset + iface->classDescriptors.size()) <= max_size) ? iface->classDescriptors.size() : (max_size - offset) };
			memcpy(&data_buffer[offset], iface->classDescriptors.c_ptr(), copy_size);
			offset += copy_size;
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

		ept.queue_response(request.wLength(), reinterpret_cast<char*>(data_buffer), max_size);
		
		break;
	}
	case DEVICE_QUALIFIER:
	        // USB 2.0, 9.6.2:
		// If a full-speed only device (with a device descriptor version number equal to 0200H) receives a
                // GetDescriptor() request for a device_qualifier, it must respond with a request error.
	        return -1;
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

	if (usb_status->state == DEFAULT_STATE) {
		return -1;
	}
	else if (usb_status->state != CONFIGURED_STATE) {
		return -1;
	}

	char* configuration = new(std::nothrow) char[1];
	if (configuration == nullptr)
	{
	    return -1;
	}
	*configuration = device.current_config->config_number;
	
	if (usb_status->state == ADDRESS_STATE) {
		configuration = 0;
	}

	device.ep0.queue_data(configuration, 1);
	return true;
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
	USBInterface& iface(*current_config->interfaces[0]);
	for (unsigned int i = 0; i < iface.inTokenHandlers.size(); i++) {
		if (iface.inTokenHandlers[i].first == ep) {
			auto& handler(iface.inTokenHandlers[i].second);
			int status{ handler() };
			if (status < 0) {
				return status;
			}
			else if (status > 0) {
				break;
			}
		}
	}
	return 0;
}

int USBDevice::out_token_received(unsigned int ep)
{
	char* data{};
	unsigned int length{};

	if (ep == 0) {
		// This is the DATA OUT stage of a SETUP transaction on the default control endpoint.
		data = ep0.read_data(length);
		if (length == 0) {
			return 0;
		}
		if (inProgressSetupTransaction) {
			// A SETUP transaction with a DATA OUT stage is in progress.
			inProgressSetupTransaction.append(data, length);
			delete[] data;
			if (inProgressSetupTransaction) {
				// We still have more data to receive.
				// Return and wait for the next packet to arrive.
				return 0;
			}
			else {
				int status = process_setup_transaction(inProgressSetupTransaction.bytes.get_ptr(),
								       inProgressSetupTransaction.total_bytes);
				inProgressSetupTransaction = {};
				return status;
			}
		}
	}
	
	// find out if this is an IN or OUT endpoint
	USBInterface& iface(*current_config->interfaces[0]);  // FIXME: there should be a pointer to the currently selected interface
	for (unsigned int i = 0; i < iface.outEndpoints.size(); i++) {
		if (iface.outEndpoints[i]->ep_number == ep) {
			// this is an OUT endpoint, so we are receiving data
			data = iface.outEndpoints[i]->read_data(length);
			if (!length || !data) {
				delete[] data;
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
				delete[] data;
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

int usb_reserved(volatile char* buf) { return 0; }

static
int (USBSetupRequestHandler::* const usb_functions[13])(USBDevice&, char*) = {
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

int USBDevice::process_setup_transaction(char* buffer, unsigned int size)
{
	USBStandardDeviceRequest req{ buffer };
	const unsigned char& bmRequestType{ req.bmRequestType() };
	
	// First determine who this request is for.
	const unsigned char recipient = bmRequestType & 0x1F;
	switch (recipient) {
	case 0:  // Device
		break;
	case 1:  // Interface
	{
		unsigned short wIndex{ req.wIndex() };
		// Guaranteed to exist by checks in setup_token_received().
		USBInterface* iface{ getInterface(wIndex) };
		int status{ iface->interfaceRequest(&ep0, buffer) };
		if (status < 0) {
			ep0.stall();
		}
		else {
			ep0.complete_setup(req);
		}
		return status;
	}
	case 2:  // Endpoint
	{
		USBEndpoint* ep{ getEndpoint(req.wIndex()) };
		if (!ep) {
			ep0.stall();
			return -1;
		}
		int status{ ep->endpointRequest(&ep0, buffer) };
		if (status < 0) {
			ep0.stall();
		}
		else {
			ep0.complete_setup(req);
		}
		return status;
	}
	default:
		break;
	}

	const unsigned int& bRequest{ req.bRequest() };
	char type{ char((bmRequestType & 0x60) >> 5) };

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
	if (bRequest != SET_ADDRESS) {
		// SET_ADDRESS is handled specially.  The ZLP needs to be sent *before*
		// the address is changed.
		ep0.complete_setup(req);
	}
	return 0;
error:
	ep0.stall();
	return -1;
}

int USBDevice::setup_token_received(const USBStandardDeviceRequest& req)
{
    char* buffer = new(std::nothrow) char[USBStandardDeviceRequest::size()];
    if (buffer == nullptr)
    {
	ep0.stall();
	return -1;
    }
    req.copyTo(buffer, USBStandardDeviceRequest::size());
        // If there is a DATA-OUT stage, receive the incoming data.
	const unsigned char& bmRequestType{ req.bmRequestType() };
	const unsigned short& wLength{ req.wLength() };
	const bool data_out_stage{ (bmRequestType & 0x80) == 0 && wLength != 0 };
	if (data_out_stage) {
	        inProgressSetupTransaction = InProgressTransfer{ USBStandardDeviceRequest::size() + wLength };
		if (!inProgressSetupTransaction.bytes) {
			inProgressSetupTransaction = {};
			ep0.stall();
			return -1;
		}
		inProgressSetupTransaction.append(buffer, USBStandardDeviceRequest::size());
		delete[] buffer;
		// Fall through and check for errors.
	}
	else {
	        int status = process_setup_transaction(buffer, USBStandardDeviceRequest::size());
		delete[] buffer;
		return status;
	}

	// First determine who this request is for.
	const unsigned char recipient = bmRequestType & 0x1F;
	switch (recipient) {
	case 0:  // Device
		break;
	case 1:  // Interface
	{
		unsigned short wIndex{ req.wIndex() };
		USBInterface* iface{ getInterface(wIndex) };
		if (!iface) {
			ep0.stall();
			return -1;
		}
		return 0;
	}
	case 2:  // Endpoint
		if (!getEndpoint(req.wIndex())) {
			ep0.stall();
			return -1;
		}
		return 0;
	default:
		break;
	}

	const unsigned int& bRequest{ req.bRequest() };
	char type{ char((bmRequestType & 0x60) >> 5) };

	const char STANDARD = 0;
	const char CLASS = 1;
	
	switch (type) {
	case STANDARD:
		if (bRequest > 12 || usb_functions[bRequest] == nullptr) {
			goto error;
		}
		break;
	case CLASS:
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
		
		USBStandardDeviceRequest req{ ept.read_setup() };

		int status{ usb_status->device->setup_token_received(req) };
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
