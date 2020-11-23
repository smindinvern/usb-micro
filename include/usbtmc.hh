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

#ifndef USBTMC_HH_
#define USBTMC_HH_

#include "std.hh"
#include "usb.hh"
#include "mm.hh"
#include "Tuple.hh"
#include "Vector.hh"
#include "Invokable.hh"
#include "Pointers.hh"

template<class T, class U, class... V> class SerializablePaddedMod4 : public Serializable<T, U, V...>
{
public:
	char* serialize() const
	{
		size_t size{ Serializable<T, U, V...>::size() };
		char* buffer{ new(std::nothrow) char[size + (size % 4)] };
		if (!buffer) {
			return nullptr;
		}
		Serializable<T, U, V...>::copyTo(buffer, size);
		memcpy(buffer + size, 0, size % 4);
		return buffer;
	}
	using Serializable<T, U, V...>::Serializable;
};

/* a USBTMC Bulk OUT header must begin the first transaction
   in ALL Bulk OUT messages to the device */
class USBTMCBulkHeaderBase : public DeSerializable<unsigned char,
						   unsigned char,
						   unsigned char,
						   unsigned char>
{
public:
	USBTMCBulkHeaderBase() = default;
	USBTMCBulkHeaderBase(unsigned char MsgID,
			     unsigned char bTag)
		: DeSerializable{ MsgID, bTag, (unsigned char)(~bTag), 0 } {}
	using DeSerializable::DeSerializable;
};

typedef USBTMCBulkHeaderBase USBTMCBulkOutHeaderBase;
typedef USBTMCBulkHeaderBase USBTMCBulkInHeaderBase;

enum USBTMCBulkMsgIDs {
	/* 0 reserved */
	DEV_DEP_MSG_OUT = 1,                 // OUT | Message is a USBTMC device dependent message.
	                                     // IN  | No defined response for this command message.
	REQUEST_DEV_DEP_MSG_IN = 2,          // OUT | Command message that requests device to send response message
	                                     //       Bulk-IN endpoint.
	DEV_DEP_MSG_IN = 2,                  // IN  | Response message to the REQUEST_DEV_DEP_MSG_IN.
	/* 3-125 reserved for USBTMC use */
	VENDOR_SPECIFIC_OUT = 126,           // OUT | A USBTMC vendor-specific command message.
	                                     // IN  | No defined response for this command message.
	REQUEST_VENDOR_SPECIFIC_IN = 127,    // OUT | Command message that requests device to send vendor-specific
	                                     //       response message on the Bulk-IN endpoint.
	VENDOR_SPECIFIC_IN = 127             // IN  | Response message to REQUEST_VENDOR_SPECIFIC_IN.
	/* 128-191 reserved for USBTMC subclass use */
	/* 192-255 reserved for VISA specification use */
};

/* USBTMC Bulk OUT transactions */
struct USBTMCDevDepMsg : public DeSerializable<USBTMCBulkHeaderBase,  /* Bytes 0-3 */
					       unsigned int,             /* TransferSize */
					       unsigned char,            /* bmTransferAttributes */
					       unsigned char,            /* padding */
					       unsigned char,            /* padding */
					       unsigned char>            /* padding */
{
	USBTMCDevDepMsg(unsigned char MsgID,
			unsigned char bTag,
			unsigned int transferSize,
			unsigned char bmTransferAttributes)
		: DeSerializable{ { MsgID, bTag }, transferSize, bmTransferAttributes, 0, 0, 0 } {}
	using DeSerializable::DeSerializable;
};

struct USBTMCDevDepMsgOut : public USBTMCDevDepMsg
{
	USBTMCDevDepMsgOut(unsigned char bTag,
			   unsigned int transferSize,
			   bool Eom)
		: USBTMCDevDepMsg{ DEV_DEP_MSG_OUT, bTag, transferSize, Eom } {}

	// for direct deserialization
	using USBTMCDevDepMsg::USBTMCDevDepMsg;
};

class USBTMCRequestDevDepMsgIn : public DeSerializable<USBTMCBulkOutHeaderBase,  /* Bytes 0-3 */
						       unsigned int,             /* TransferSize */
						       unsigned char,            /* bmTransferAttributes */
						       unsigned char,            /* TermChar */
						       unsigned char,            /* padding */
						       unsigned char>            /* padding */
{
public:
	USBTMCRequestDevDepMsgIn(unsigned char bTag,
				 unsigned int transferSize,
				 bool termCharEnabled,
				 unsigned char termChar)
		: DeSerializable{ { REQUEST_DEV_DEP_MSG_IN, bTag }, transferSize, termCharEnabled, termChar, 0, 0 } {}
	using DeSerializable::DeSerializable;
};

struct USBTMCVendorSpecific : public DeSerializable<USBTMCBulkHeaderBase,
						    unsigned int,
						    unsigned int>
{
	USBTMCVendorSpecific(unsigned char MsgID,
			     unsigned char bTag,
			     unsigned int transferSize)
		: DeSerializable{ { MsgID, bTag }, transferSize, 0 } {}
	using DeSerializable::DeSerializable;
};

struct USBTMCVendorSpecificOut : public USBTMCVendorSpecific
{
	USBTMCVendorSpecificOut(unsigned char bTag,
				unsigned int transferSize)
		: USBTMCVendorSpecific{ VENDOR_SPECIFIC_OUT, bTag, transferSize } {}
	using USBTMCVendorSpecific::USBTMCVendorSpecific;
};


/* USBTMC Bulk IN transactions */
// sent FROM device TO host in response to a USBTMC Bulk OUT transaction requesting response
// MsgID, bTag, and bTagInverse must match Bulk-OUT header that caused this response

// 2.  If a USBTMC interface receives a Bulk-IN request prior to receiving a USBTMC command message
//     that expects a response, the device must NAK the request.
// 3.  The device must not queue any Bulk-IN DATA until it receives a valid USBTMC command message
//     that expects a response.
// 6.  The device is not required to respond immediately after receiving a USBTMC command message
//     that expects a response. A device must not send a DATA payload until a termination condition is
//     detected (EOM, TermChar, or the maximum number of USBTMC response message data bytes the
//     Host has specified to send are available) or until the device can not buffer any more data.
// 7.  The first USB transaction in a Bulk-IN transfer must begin with a complete Bulk-IN Header.
// 8.  The USBTMC message data bytes must immediately follow the USBTMC Bulk-IN Header in the
//     same USB transaction in the same DATA payload, subject to maximum packet size constraints.
// 9.  A device may return less than the maximum number of USBTMC response message data bytes the
//     Host specified to send. When the Bulk-IN transfer is completed, if more message data bytes are
//     expected, the Host may send a new USBTMC command message to read the remainder of the
//     message.
// 10. The device must always terminate a Bulk-IN transfer by sending a short packet. The short packet
//     may be zero-length or non zero-length. The device may send extra alignment bytes (up to
//     wMaxPacketSize – 1) to avoid sending a zero-length packet. The alignment bytes should be 0x00-
//     valued, but this is not required. A device is not required to send any alignment bytes.
// 11. Once a transfer is terminated, the device must not queue any more Bulk-IN DATA until it receives
//     another USBTMC command message that expects a response.
// 12. A device may defer the parsing and processing of Bulk-OUT data while a Bulk-IN transfer is in
//     progress.
// 13. The device may send a Bulk-IN message using multiple transfers, as the data becomes available. This
//     is illustrated below in Figure 4. This ability is needed because some devices may not have enough
//     memory to buffer a complete USBTMC message. Another benefit of this ability is that some Hosts
//     may make use of USBTMC message content as it is delivered.

struct USBTMCDevDepMsgIn : public USBTMCDevDepMsg
{
	USBTMCDevDepMsgIn(unsigned char bTag,
			  unsigned int transferSize,
			  bool useTermChar,
			  bool Eom)
		: USBTMCDevDepMsg{ DEV_DEP_MSG_IN, bTag, transferSize,
			(unsigned char)((char(useTermChar) << 1) | char(Eom)) } {}
};

struct USBTMCVendorSpecificIn : public USBTMCVendorSpecific
{
	USBTMCVendorSpecificIn(unsigned char bTag,
			       unsigned int transferSize)
		: USBTMCVendorSpecific{ VENDOR_SPECIFIC_IN, bTag, transferSize } {}
	using USBTMCVendorSpecific::USBTMCVendorSpecific;
};

/* USBTMC CONTROL endpoint requests */
// CLEAR_FEATURE wValue=ENDPOINT_HALT:
//   if the reason for the halt no longer exists, the device must clear the
//   halt condition on the specified endpoint
// USBTMC interface Bulk-OUT endpoints:
//   The device, after receiving the CLEAR_FEATURE request, must interpret the first part of the next Bulk-
//   OUT transaction as a new USBTMC Bulk-OUT Header.
// USBTMC interface Bulk-IN endpoints:
//   The device, after receiving the CLEAR_FEATURE request, must not queue any Bulk-IN DATA until it
//   receives a USBTMC command message that expects a response.
// USBTMC class-specific requests:
//   All USBTMC class specific requests must be sent with a Setup packet.

/* Table 15 - USBTMC bRequest values */
/* configuration endpoing requests */
enum USBTMCRequestValues {
	/* 0 reserved */
	INITIATE_ABORT_BULK_OUT = 1,         // aborts a Bulk-OUT transfer
	CHECK_ABORT_BULK_OUT_STATUS = 2,     // returns the status of the previously sent INITIATE_ABORT_BULK_OUT request
	INITIATE_ABORT_BULK_IN = 3,          // aborts a Bulk-IN transfer
	CHECK_ABORT_BULK_IN_STATUS = 4,      // returns the status of the previously sent INITIATE_ABORT_BULK_IN request
	INITIATE_CLEAR = 5,                  // clears all previously sent pending and unprocessed Bulk-OUT message content
	                                     // and clears all pending Bulk-IN transfers from the interface
	CHECK_CLEAR_STATUS = 6,              // returns the status of the previously sent INITIATE_CLEAR request
	GET_CAPABILITIES = 7,                // returns the attributes and capabilities of the USBTMC interface
	/* 8-63 reserved */
	INDICATOR_PULSE = 64,                // a mechanism to turn on an activity indicator for identification purposes.
	                                     // the device indicates whether or not it supports this request in the
	                                     // GET_CAPABILITIES response packet
	/* 65-127 reserved for use by the USBTMC specification */
	/* 128-191 reserved for use by USBTMC subclass specifications */
	/* 192-255 reserved for use by the VISA specification */
};

/* All USBTMC class-specific requests return data to the Host (bmRequestType direction=Device-to-host) and have a data
   payload that begins with a 1 byte USBTMC_status field. */

/* Table 16 - USBTMC_status values */
enum USBTMC_statusValues {
	/* 0 reserved */
	/* Success */
	STATUS_SUCCESS = 0x01,                  // Success.
	/* Warning */
	STATUS_PENDING = 0x02,                  // This status is valid if a device has received a USBTMC split transaction
	                                        // CHECK_STATUS request and the request is still being processed.
	/* 0x03-0x1F reserved for USBTMC use */
	/* 0x20-0x3F reserved for subclass use */
	/* 0x40-0x7F reserved for VISA use */
	/* Failure */
	STATUS_FAILED = 0x80,                   // Failure, unspecified reason, and a more specific USBTMC_status is not defined
	STATUS_TRANSFER_NOT_IN_PROGRESS = 0x81, // This status is only valid if a device has received an INITIATE_ABORT_BULK_OUT
	                                        // or INITIATE_ABORT_BULK_IN request and the specified transfer to abort is
	                                        // not in progress.
	STATUS_SPLIT_NOT_IN_PROGRESS = 0x82,    // This status is valid if the device received a CHECK_STATUS request and the
	                                        // device is not processing an INITIATE request.
	STATUS_SPLIT_IN_PROGRESS = 0x83,        // This status is valid if the device received a new class-specific request and
	                                        // the device is still processing an INITIATE.
	/* 0x84-0x9F reserved for USBTMC use */
	/* 0xA0-0xBF reserved for subclass use */
	/* 0xC0-0xFF reserved for VISA use */
};

// GET_CAPABILITIES

struct USBTMCCapabilities
{
	bool acceptsIndicatorPulse;
	bool talk_only;
	bool listen_only;
	bool supportsTermChar;
};

struct USBTMCGetCapabilitiesResponse : Serializable<unsigned char, /* USBTMC_status */
						    unsigned char, /* Reserved */
						    unsigned short, /* bcdUSBTMC */
						    unsigned char, /* bmUSBTMCifCaps */
						    unsigned char, /* bmUSBTMCdevCaps */
						    unsigned short, /* Reserved */
						    unsigned int, /* Reserved */
						    unsigned int, /* Reserved */
						    unsigned int, /* Reserved */
						    unsigned int /* Reserved */>
{
	USBTMCGetCapabilitiesResponse() = default;
	USBTMCGetCapabilitiesResponse(unsigned char status,
				      unsigned short bcdUSBTMC,
				      bool acceptsIndicatorPulse,
				      bool talk_only,
				      bool listen_only,
				      bool supportsTermChar)
		: Serializable{ status, 0, bcdUSBTMC,
			(unsigned char)((char(acceptsIndicatorPulse) << 2) |
					(char(talk_only) << 1) |
					char(listen_only)),
			(unsigned char)(supportsTermChar), 0, 0, 0, 0, 0 } {}
};

// INDICATOR_PULSE


// USB descriptors

struct USBTMCDeviceDescriptor : public USBDeviceDescriptor
{
	USBTMCDeviceDescriptor(unsigned char bMaxPacketSize0,
			       unsigned short idVendor,
			       unsigned short idProduct,
			       unsigned short bcdDevice,
			       unsigned char iManufacturer,
			       unsigned char iProduct,
			       unsigned char iSerialNumber,
			       unsigned char bNumConfigurations)
		: USBDeviceDescriptor{ 0x00,
			0x00,
			0x00,
			bMaxPacketSize0,
			idVendor,
			idProduct,
			bcdDevice,
			iManufacturer,
			iProduct,
			iSerialNumber,
			bNumConfigurations } {}
};

typedef USBConfigurationDescriptor USBTMCConfigurationDescriptor;

enum USBTMC_bInterfaceProtocol {
	USBTMC_interface = 0,
	USBTMC_USB488_interface
};

struct USBTMCInterfaceDescriptor : public USBInterfaceDescriptor
{
	USBTMCInterfaceDescriptor(unsigned char bInterfaceNumber,
				  unsigned char bNumEndpoints,
				  USBTMC_bInterfaceProtocol bInterfaceProtocol,
				  unsigned char iInterface)
		: USBInterfaceDescriptor{ bInterfaceNumber,
			0x00,
			bNumEndpoints,
			0xfe,
			0x03,
			(unsigned char)bInterfaceProtocol,
			iInterface } {}
};

// USBTMC Bulk-OUT endpoint descriptors must have a wMaxPacketSize
// must be a multiple of 4.
// USBTMC interfaces must implement string descriptors with LANGID = 0x0409.
// string descriptors must contain only the characters 0x20 to 0x7e, excluding these:
//   0x22        '"'
//   0x2a        '*'
//   0x2f        '/'
//   0x3a        ':'
//   0x3f        '?'
//   0x5c        '\' 
// there must not be any leading or trailing space characters

struct USBTMCBulkOutState
{
	enum  {
		IDLE,
		PROCESSING_TRANSACTION,
		RXING_BULK_OUT_TRANSFER,
		TRANSFER_COMPLETED,
		PROCESSING_MESSAGE,
		ABORT_MESSAGE,
		RETURN_STATUS,
		ERROR
	} current_state = IDLE;
	unsigned char last_bTag = 0;

	struct InProgressTransfer current_transfer, current_message;
};

struct USBTMCInEndpoint : public USBInEndpoint
{
	using USBInEndpoint::USBInEndpoint;
	USBTMCInEndpoint(USBInEndpoint&& rhs) : USBInEndpoint(std::move(rhs)) {}
	unsigned char last_bTag{};
};

class USBTMCInterface : public USBInterface
{
	/* NOTE: these endpoints are owned by `USBInterface'! */
	USBOutEndpoint* bulk_out{};
	USBTMCInEndpoint* bulk_in{};
	USBInEndpoint* interrupt_in{};  /* optional */

	USBTMCCapabilities* capabilities;
	USBTMCBulkOutState out_state;
  
	static bool append_transfer_to_message(InProgressTransfer& t, InProgressTransfer& m);

	int handle_current_message();
	int out_token_handler(char* packet_data, unsigned int bytes);
	int in_token_handler();
	int class_request_handler(USBControlEndpoint* ep0, char* bytes);
	int bulk_in_request_handler(USBControlEndpoint* ep0, char* bytes);
	int bulk_out_request_handler(USBControlEndpoint* ep0, char* bytes);
public:
	typedef Invokable<int(unsigned char MsgID,
						  unsigned char bTag,
						  unsigned int transferSize,
						  char* msgBytes)> out_msg_handler;
	typedef Invokable<int(USBInEndpoint& in_ep,
						  unsigned char MsgID,
						  unsigned char bTag,
						  unsigned int transferSize,
						  bool useTermChar,
						  char termChar)> in_msg_handler;

	int handle_dev_dep_msg_out(char* packet_data, unsigned int bytes);
	int handle_dev_dep_msg_in(char* packet_data, unsigned int bytes);
	int handle_vendor_specific_out(char* packet_data, unsigned int bytes);
	int handle_vendor_specific_in(char* packet_data, unsigned int bytes);
	
	USBTMCInterface(USBTMC_bInterfaceProtocol bInterfaceProtocol,
			USBTMCCapabilities* interface_capabilities, /* HACK */
			std::exclusive_ptr<USBOutEndpoint>&& bulk_out_ep,
			std::exclusive_ptr<USBInEndpoint>&& bulk_in_ep);

	void addDevDepMsgOutHandler(out_msg_handler& handler)
	{
		dev_dep_msg_out_handler = &handler;
	}
	void addDevDepMsgInReqHandler(in_msg_handler& handler)
	{
		dev_dep_msg_in_req_handler = &handler;
	}

	// publicly visible records for e.g. USBTMCDevice to manage some things for us
	unsigned char bulkIn_bTag{};
private:
	out_msg_handler* dev_dep_msg_out_handler;
	in_msg_handler* dev_dep_msg_in_req_handler;
};

class USBTMCDevice : public USBDevice
{
	struct USBTMCSetupRequestHandler : USBSetupRequestHandler
	{
		virtual int get_status(USBDevice&, char*);
		virtual int usb_clear_feature(USBDevice&, char*);
		virtual int usb_set_feature(USBDevice&, char*);
		virtual int usb_set_address(USBDevice&, char*);
		virtual int usb_get_descriptor(USBDevice&, char*);
		virtual int usb_set_descriptor(USBDevice&, char*);
		virtual int usb_get_configuration(USBDevice&, char*);
		virtual int usb_set_configuration(USBDevice&, char*);
		virtual int usb_get_interface(USBDevice&, char*);
		virtual int usb_set_interface(USBDevice&, char*);
	} usbtmc_setup_handler;

	int usbtmc_class_request_handler(USBControlEndpoint* ep0, char* bytes);
public:
	USBTMCDevice(USBDevice&& dev)
		: USBDevice{ std::move(dev) }
	{
		addSetupRequestHandler(&usbtmc_setup_handler);
#if 0
		addClassRequestHandler({
				[this](USBControlEndpoint* ep0, char* bytes) {
					return usbtmc_class_request_handler(ep0, bytes);
				}
			});
#endif
	}
};

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
	int send_string_descriptor(USBDevice& device, const wchar_t* string_,
				   size_t string_length, size_t tx_length)
	{
		const char STRING = 3;
		// STRING desciptors contain a 2-byte header.
		if (tx_length < 2) {
			return -1;
		}
		else if (string_length > 253) {
			// descriptor strings can't be longer than 253 bytes.
			return -1;
		}
		tx_length -= 2;
		// Truncate descriptor if needed to fit in requested transmission length.
		size_t payload_length{ (string_length < tx_length) ? string_length : tx_length };
		char* bytes = new(std::nothrow) char[payload_length + 2];

		bytes[0] = string_length + 2; // Total descriptor length.
		bytes[1] = STRING;
		memcpy(&bytes[2], string_, payload_length);
		int status = device.ep0.sendData(bytes, payload_length + 2);
		delete[] bytes;
		return status;
		
	}
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
		switch (buf[2] & 0xff) {
		case 0:
			device.ep0.sendData(descriptor0, 4);
			break;
		case 4:
			send_string_descriptor(device, manufacturer_name_,
					       manufacturer_name_size, length);
			break;
		case 2:
			send_string_descriptor(device, product_name_,
					       product_name_size, length);
			break;
		case 3:
			send_string_descriptor(device, serial_number_,
					       serial_number_size, length);
			break;
		default:
			break;
		}
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

USBTMCDevice create_usbtmc_device(const wchar_t* manufacturer_name,
				  const wchar_t* product_name,
				  const wchar_t* serial_number,
				  const USBTMC_bInterfaceProtocol protocol,
				  USBTMCCapabilities* capabilities,
				  const USBDeviceFactory* cstr,
				  const Invokable<std::exclusive_ptr<USBOutEndpoint>()>* get_out_ep,
				  const Invokable<std::exclusive_ptr<USBInEndpoint>()>* get_in_ep,
				  USBTMCInterface::out_msg_handler* out_handler,
				  USBTMCInterface::in_msg_handler* in_handler);

#endif  // USBTMC_HH_
