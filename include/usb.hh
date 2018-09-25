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

#ifndef USB_HH_
#define USB_HH_

#define NUM_ENDPOINTS	3

/**
 * struct endpoint
 *
 * Holds low-level state information about the USB
 * transceiver endpoint, such as the current state
 * of the data toggle bit, data transfers, etc.
 */
struct endpoint {
	unsigned int tx_ppb:1;
	unsigned int rx_ppb:1;
	unsigned int data01:1;
	unsigned int bytes_transmitted:16;
	unsigned int total_bytes:16;
	char *buffer;
};

struct USBEndpoint;
struct USBDevice;

/**
 * struct usb_status_info
 *
 * Holds slightly higher-level information about the
 * USB transceiver as a whole, as well as the user-
 * defined USB device attached to it.
 */
struct usb_status_info {
	unsigned int reset:1;
	unsigned int :0;
	unsigned int configured:1;
	unsigned int :0;
	struct endpoint eps[NUM_ENDPOINTS];
	struct USBDevice* device;
	struct USBEndpoint* usb_eps[NUM_ENDPOINTS];
	unsigned int state:8;
	unsigned int :0;
	unsigned int address:8;
	unsigned int :0;
	unsigned int *bdt;
	unsigned int idle;
	unsigned char protocol;
	unsigned char rts;
};

#ifdef __cplusplus

#include "std.hh"
#include "usb_private.hh"
#include "mm.hh"
#include "Vector.hh"
#include "Invokable.hh"

/**
 * struct USBEndpointImpl
 *
 * Abstract class providing an interface to the hardware-
 * specific implementation behind the USBEndpoint.
 */
struct USBEndpointImpl
{
	virtual void reset() = 0;
	virtual void stall() = 0;
	virtual void unstall() = 0;
	virtual int send_data(char*, unsigned int, bool) = 0;
	virtual char* read_data(unsigned int&) = 0;
	virtual char* read_setup(unsigned int&) = 0;
	virtual ~USBEndpointImpl() = default;
};


/**
 * struct USBEndpoint
 *
 * Provides the interface to a single usb endpoint.  This
 * class hides access to the interface function, as it is
 * intended to be derived by USBOutEndpoint and USBInEndpoint.
 */
struct USBEndpoint
{
protected:
	int sendData(char* data, unsigned int length, bool buffered = false);
	int sendZLP(bool wait = true);
	char* receiveData(unsigned int& length);
	char* receiveSetup(unsigned int& length);
	
	int send_data(char* data, unsigned int length, bool buffered) { return (impl->send_data)(data, length, buffered); }
	char* read_data(unsigned int& length) { return (impl->read_data)(length); }
	char* read_setup(unsigned int& length) { return (impl->read_setup)(length); }
	USBEndpointImpl* impl;
public:
	enum ep_type {
		control_ep,
		isochronous_ep,
		bulk_ep,
		interrupt_ep
	};
	unsigned char ep_number;
	ep_type type;
	unsigned short max_packet_size;
	bool stalled{};

	void reset() { return (impl->reset)(); }
	void stall() { return (impl->stall)();	}
	void unstall() { return (impl->unstall)(); }
	
	// notifiers for in/out/setup tokens?

	/* endpointNumber: bit 8 - 1 for IN endpoints, 0 otherwise */
	USBEndpoint(unsigned char endpointNumber, ep_type endpointType,
		    unsigned short maxPacketSize, USBEndpointImpl* implementation)
		: impl{ implementation },
		  ep_number{ endpointNumber },
		  type{ endpointType },
		  max_packet_size { maxPacketSize } {}
	~USBEndpoint()
	{
		delete impl;
	}
	USBEndpoint(const USBEndpoint&) = delete;
	USBEndpoint(USBEndpoint&& rhs)
		: impl{ rhs.impl },
		  ep_number{ rhs.ep_number },
		  type{ rhs.type },
		  max_packet_size{ rhs.max_packet_size }
	{
		rhs.impl = nullptr;
	}
};

/**
 * struct USBInEndpoint
 *
 * Exposes in-endpoint related functions inherited from USBEndpoint.
 */
struct USBInEndpoint : public USBEndpoint
{
	using USBEndpoint::sendData;
	using USBEndpoint::sendZLP;

	USBInEndpoint(unsigned char endpointNumber, USBEndpoint::ep_type endpointType,
		      unsigned short maxPacketSize, USBEndpointImpl* impl)
		: USBEndpoint{ (unsigned char)(endpointNumber | 0x80), endpointType,
			maxPacketSize, impl } {}
	USBInEndpoint(const USBInEndpoint&) = delete;
	USBInEndpoint(USBInEndpoint&&) = default;
};


/**
 * struct USBOutEndpoint
 *
 * Exposes out-endpoint related functions inherited from USBEndpoint.
 */
struct USBOutEndpoint : public USBEndpoint
{
	unsigned short bytes_transmitted{};
	unsigned short total_bytes{};

	using USBEndpoint::receiveData;

	USBOutEndpoint(unsigned char endpointNumber, USBEndpoint::ep_type endpointType,
		       unsigned short maxPacketSize, USBEndpointImpl* impl)
		: USBEndpoint{ (unsigned char)(endpointNumber & ~0x80), endpointType,
			maxPacketSize, impl } {}
	USBOutEndpoint(const USBOutEndpoint&) = delete;
	USBOutEndpoint(USBOutEndpoint&&) = default;
};

/**
 * struct USBControlEndpoint
 *
 * Exposes control endpoint related functions inherited from USBEndpoint.
 */
struct USBControlEndpoint : public USBEndpoint
{
	using USBEndpoint::sendData;
	using USBEndpoint::sendZLP;
	using USBEndpoint::receiveData;
	using USBEndpoint::receiveSetup;

	USBControlEndpoint(unsigned char endpointNumber, unsigned short maxPacketSize,
			   USBEndpointImpl* impl)
		: USBEndpoint{ (unsigned char)(endpointNumber & ~0x80),
			USBEndpoint::control_ep, maxPacketSize, impl } {}
	USBControlEndpoint(const USBControlEndpoint&) = delete;
	USBControlEndpoint(USBControlEndpoint&&) = default;
};


/**
 * class USBSetupRequestHandler
 *
 * Abstract class defining a uniform interface for USB SETUP token
 * request handlers.  These are intended to be registered with
 * USBDevices to define configurable runtime behavior for USB devices.
 */
struct USBSetupRequestHandler
{
	virtual int get_status(USBDevice& device, char* buf) = 0;
	virtual int usb_clear_feature(USBDevice& device, char* buf) = 0;
	virtual int usb_set_feature(USBDevice& device, char* buf) = 0;
	virtual int usb_set_address(USBDevice& device, char* buf) = 0;
	virtual int usb_get_descriptor(USBDevice& device, char* buf) = 0;
	virtual int usb_set_descriptor(USBDevice& device, char* buf) = 0;
	virtual int usb_get_configuration(USBDevice& device, char* buf) = 0;
	virtual int usb_set_configuration(USBDevice& device, char* buf) = 0;
	virtual int usb_get_interface(USBDevice& device, char* buf) = 0;
	virtual int usb_set_interface(USBDevice& device, char* buf) = 0;
	virtual ~USBSetupRequestHandler() = default;
};


/**
 * USBClassRequestHandler
 *
 * Used for user-defined callbacks for handling and responding to
 * incoming USB class requests.  The handler is expected to send a
 * response to the host if appropriate, and to return true if the
 * request was handled by the callback, false if it was not handled
 * by the callback, and a negative error code if there was an error.
 */
typedef Invokable<int(USBControlEndpoint*, char*)> USBClassRequestHandler;


/**
 * struct USBInterface
 *
 * Contains information describing the defined USB interface, intended
 * to be registered with a USBDevice.  This class can have multiple
 * endpoints of different types attached to it, along with in- and out-
 * token handlers to provide user-defined interface behavior.  This is
 * where most of the behavior of the device should be implemented.
 *
 * USB class request handlers can also be added for interfaces that
 * conform to some USB device class.
 */
struct USBInterface
{
	unsigned char bInterfaceClass;
	unsigned char bInterfaceSubClass;
	unsigned char bInterfaceProtocol;
	Vector<USBInEndpoint*> inEndpoints;
	Vector<USBOutEndpoint*> outEndpoints;
	Vector<Tuple<unsigned int, Invokable<int(char*, unsigned int)>>> outTokenHandlers;
	Vector<Tuple<unsigned int, Invokable<int()>>> inTokenHandlers;
	Vector<Invokable<int(USBControlEndpoint*, char*)>> classRequestHandlers;
	
	void addInEndpoint(USBInEndpoint* ep, Invokable<int()> handler = {})
	{
		unsigned int ep_number{ (unsigned int)(ep->ep_number & ~0x80) };
		inEndpoints.push_back(ep);
		if (handler) {
			inTokenHandlers.push_back({ ep_number, std::move(handler) });
		}
	}
	void addOutEndpoint(USBOutEndpoint* ep, Invokable<int(char*, unsigned int)> handler = {})
	{
		unsigned int ep_number{ (unsigned int)(ep->ep_number & ~0x80) };
		outEndpoints.push_back(ep);
		if (handler) {
			outTokenHandlers.push_back({ ep_number, std::move(handler) });
		}
	}
	void addClassRequestHandler(Invokable<int(USBControlEndpoint*, char*)> handler)
	{
		classRequestHandlers.push_back(std::move(handler));
	}

	USBInterface(unsigned char iface_class,
		     unsigned char iface_subclass, unsigned char iface_protocol)
		: bInterfaceClass{ iface_class },
		  bInterfaceSubClass{ iface_subclass },
		  bInterfaceProtocol{ iface_protocol } {}
	USBInterface(const USBInterface&) = delete;
	USBInterface(USBInterface&&) = default;
	virtual ~USBInterface()
	{
		for (unsigned int i = 0; i < inEndpoints.size(); i++) {
			delete inEndpoints[i];
		}
		for (unsigned int i = 0; i < outEndpoints.size(); i++) {
			delete outEndpoints[i];
		}
	}
};


/**
 * class USBConfiguration
 *
 * Contains a group of USBInterfaces, which are all active simultaneously
 * when the configuration is active.
 *
 * None of the logic of dispatching requests to individual interfaces is
 * done here, but rather in USBDevice.
 */
class USBConfiguration
{
public:
	const unsigned char config_number;
	Vector<USBInterface*> interfaces;

	USBConfiguration(unsigned char number)
		: config_number{ number } {}
	USBConfiguration(USBConfiguration&&) = default;
	virtual ~USBConfiguration()
	{
		for (unsigned int i = 0; i < interfaces.size(); i++) {
			delete interfaces[i];
		}
	}
};

struct USBDeviceImpl
{
	virtual void setAddress(const unsigned short addr) = 0;
	virtual void enterDefaultState() = 0;
	virtual void setConfigured(const bool b) = 0;
	virtual ~USBDeviceImpl() = default;
};

struct USBDeviceGenericImpl : USBDeviceImpl
{
	virtual void setAddress(const unsigned short addr)
	{
		if (addr != 0) {
			__usb_set_address(0);
		}
		__usb_set_address(addr);
	}
	virtual void enterDefaultState()
	{
		__usb_enter_default_state();
	}
	virtual void setConfigured(const bool b)
	{
		__usb_set_configured(b);
	}
};

typedef Invokable<USBConfiguration*(unsigned char)> USBConfigurationFactory;

/**
 * struct USBDevice
 *
 * Contains all information and code necessary to enumerate the specified
 * device and forward all incoming requests and transactions to the appropriate
 * interface.
 *
 * Multiple configurations can be registered via a configuration factory.
 * Device-specific USB setup and class request handlers can also be specified.
 * All USB standard setup requests are handled natively, although additional
 * handlers can be registered for those requests that aren't handled/are rejected
 * by the default handlers.  Class-specific device request handlers can also
 * be registered.
 */
struct USBDevice
{
public:
	USBControlEndpoint ep0;
protected:
    USBConfigurationFactory configFactory;
	USBConfiguration* current_config{};
	Vector<USBSetupRequestHandler*> setup_handlers;
	Vector<USBClassRequestHandler> class_handlers;
	USBDeviceDescriptor dev_descriptor;
	USBDeviceImpl* impl;
	
	struct DefaultSetupRequestHandler : public USBSetupRequestHandler
	{
		virtual int get_status(USBDevice&, char* buf);
		virtual int usb_clear_feature(USBDevice&, char* buf);
		virtual int usb_set_feature(USBDevice&, char* buf);
		virtual int usb_set_address(USBDevice&, char* buf);
		virtual int usb_get_descriptor(USBDevice&, char* buf);
		virtual int usb_set_descriptor(USBDevice&, char* buf);
		virtual int usb_get_configuration(USBDevice&, char* buf);
		virtual int usb_set_configuration(USBDevice&, char* buf);
		virtual int usb_get_interface(USBDevice&, char* buf);
		virtual int usb_set_interface(USBDevice&, char* buf);
	};
	friend struct DefaultSetupRequestHandler;

public:
	USBDevice(USBControlEndpoint ep0_, USBDeviceDescriptor descriptor,
			  USBConfigurationFactory configurationFactory,
			  USBDeviceImpl* privImpl);
	USBDevice(USBDevice&& other)
		: ep0{ std::move(other.ep0) },
		  configFactory{ std::move(other.configFactory) },
		  current_config{ other.current_config },
		  setup_handlers{ std::move(other.setup_handlers) },
		  class_handlers{ std::move(other.class_handlers) },
		  dev_descriptor{ std::move(other.dev_descriptor) },
		  impl{ other.impl }
	{
		other.current_config = nullptr;
		other.impl = nullptr;
	}

	void reset();
	void setAddress(const unsigned short addr) { return impl->setAddress(addr); }
	void enterDefaultState() { return impl->enterDefaultState(); }
	void setConfigured(const bool b) { return impl->setConfigured(b); }

	int setup_token_received(char* buffer, unsigned int size);
	int in_token_received(unsigned int ep);
	int out_token_received(unsigned int ep);
	
	void addSetupRequestHandler(USBSetupRequestHandler* new_handler)
	{
		if (!new_handler) {
			return;
		}
		setup_handlers.push_back(new_handler);
	}
	void addClassRequestHandler(USBClassRequestHandler new_handler)
	{
		class_handlers.push_back(std::move(new_handler));
	}

	USBEndpoint* getEndpoint(unsigned int ep)
	{
		USBConfiguration& config(*current_config);
		for (unsigned int i = 0; i < config.interfaces.size(); i++) {
			for (unsigned int j = 0; j < config.interfaces[i]->inEndpoints.size(); j++) {
				if (config.interfaces[i]->inEndpoints[j]->ep_number == (ep | 0x80)) {
					return config.interfaces[i]->inEndpoints[j];
				}
			}
			for (unsigned int j = 0; j < config.interfaces[i]->outEndpoints.size(); j++) {
				if (config.interfaces[i]->outEndpoints[j]->ep_number == ep) {
					return config.interfaces[i]->outEndpoints[j];
				}
			}
		}

		return nullptr;
	}
	
	virtual ~USBDevice()
	{
		for (unsigned int i = 0; i < setup_handlers.size(); i++) {
			delete setup_handlers[i];
		}
		delete current_config;
		delete impl;
	}
};

typedef Invokable<USBDevice(USBConfigurationFactory&&)> USBDeviceFactory;

#endif  // defined(__cplusplus)

#ifdef __cplusplus
extern "C" {
#endif
	int usb_send_data_chunked(unsigned int ep, char *data, unsigned int length, unsigned int chunk_length, bool buffered);
	void init_usb();
	int usb_setup_token(unsigned int ep);
	void usb_out_token(unsigned int ep);
	void usb_in_token(unsigned int ep);
#ifdef __cplusplus
}
#endif


#endif  // USB_HH_
