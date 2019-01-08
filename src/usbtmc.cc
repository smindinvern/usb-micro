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
#include "usbtmc.hh"
#include "usbtmc488.hh"

int usbtmc_endpoint_request_handler(USBControlEndpoint* ep0, char* bytes)
{
	USBStandardDeviceRequest req{ bytes };
	const unsigned int& bRequest{ req.bRequest() };
	switch (bRequest) {
	default:
		return false;
	case INITIATE_ABORT_BULK_IN:
		// this covers the case where a command message was rejected by the command parser.
		// in this case, a response message will not be sent, so we just need to respond to
		// the host here to confirm.
		char response_bytes[2] = { STATUS_FAILED, req.wValue() };
		ep0->sendData(response_bytes, sizeof(response_bytes));
		return true;
	}
}

/**
 * USBTMCInterface implementation
 *
 * USBTMCInterface is a wrapper around USBInterface that
 * allows a user to register handlers for the following messages:
 *  -DEV_DEP_MSG_OUT
 *  -REQUEST_DEV_DEP_MSG_IN
 *  -VENDOR_SPECIFIC_OUT
 *  -REQUEST_VENDOR_SPECIFIC_IN
 *
 * Receipt and aggregation of message contents is taken care of
 * by the interface.
 */

USBTMCInterface::USBTMCInterface(USBTMC_bInterfaceProtocol bInterfaceProtocol,
				 USBTMCCapabilities* interface_capabilities, /* HACK */
				 std::exclusive_ptr<USBOutEndpoint>&& bulk_out_ep,
				 std::exclusive_ptr<USBInEndpoint>&& bulk_in_ep)
	: USBInterface{ 0xfe, 0x03, (unsigned char)bInterfaceProtocol },
	  capabilities{ interface_capabilities }
{
	bulk_out = bulk_out_ep.clear_ptr();
	bulk_in = new(std::nothrow) USBTMCInEndpoint(std::move(*bulk_in_ep.clear_ptr()));
	bulk_in->addClassRequestHandler(usbtmc_endpoint_request_handler);
	USBInterface::addOutEndpoint(bulk_out, {
			[&](char* d, unsigned int n) {
				return out_token_handler(d, n);
			}
		});

	USBInterface::addInEndpoint(bulk_in, {
			[&]() {
				return in_token_handler();
			}
		});
	USBInterface::addClassRequestHandler({
			[=](USBControlEndpoint *ep0, char* bytes) -> auto {
				return class_request_handler(ep0, bytes);
			}
		});
}

int USBTMCInterface::class_request_handler(USBControlEndpoint* ep0, char* bytes)
{
	USBStandardDeviceRequest req{ bytes };
	switch (req.bRequest()) {
	case GET_CAPABILITIES:
	{
		char* response_bytes;
		unsigned int response_size;
		if (bInterfaceProtocol == USBTMC_USB488_interface) {
			/* HACK */
			USBTMC488Capabilities* caps = reinterpret_cast<USBTMC488Capabilities*>(capabilities);
			USBTMC488GetCapabilitiesResponse response = {
				STATUS_SUCCESS,
				0x0100,
				caps->base_caps.acceptsIndicatorPulse,
				caps->base_caps.talk_only,
				caps->base_caps.listen_only,
				caps->base_caps.supportsTermChar,
				caps->isUSB488_2,
				caps->acceptsGoToLocal,
				caps->acceptsTrigMsg,
				caps->understandsMandatorySCPI,
				caps->sr1Capable,
				caps->rl1Capable,
				caps->dt1Capable
			};
			response_bytes = response.serialize();
			response_size = response.size();
		}
		else {
			USBTMCGetCapabilitiesResponse response = {
				STATUS_SUCCESS,
				0x0100,
				capabilities->acceptsIndicatorPulse,
				capabilities->talk_only,
				capabilities->listen_only,
				capabilities->supportsTermChar
			};
			response_bytes = response.serialize();
			response_size = response.size();
		}
		if (!response_bytes) {
			return -1;
		}
		size_t length = req.wLength() > response_size ? response_size : req.wLength();
		ep0->sendData(response_bytes, length);
		delete[] response_bytes;
		break;
	}
	default:
		return false;
	}
	return true;
}


int USBTMCInterface::handle_current_message()
{
	char& MsgID{ out_state.current_message.bytes[0] };
	int (USBTMCInterface::* handler)(char*, unsigned int);
	
	switch (MsgID) {
	case DEV_DEP_MSG_OUT:
		handler = &USBTMCInterface::handle_dev_dep_msg_out;
		break;
	case REQUEST_DEV_DEP_MSG_IN:
		handler = &USBTMCInterface::handle_dev_dep_msg_in;
		break;
	case VENDOR_SPECIFIC_OUT:
		handler = &USBTMCInterface::handle_vendor_specific_out;
		break;
	case REQUEST_VENDOR_SPECIFIC_IN:
		handler = &USBTMCInterface::handle_vendor_specific_in;
		break;
	default:
		return -1;
	}

	int status = (this->*handler)(out_state.current_message.bytes.get_ptr(),
								  out_state.current_message.total_bytes);
	out_state.current_message = {};
	out_state.current_state = USBTMCBulkOutState::IDLE;
	return status;
}

/**
 * append_transfer_to_message()
 *
 * Appends the contents of a USBTMC transfer to the contents of a USBTMC message.
 * The message may be empty.
 * After a successful call to this function, the contents of the transfer are cleared.
 */
bool USBTMCInterface::append_transfer_to_message(InProgressTransfer& t, InProgressTransfer& m)
{
	if (!m.bytes) {
		m = std::move(t);
	}
	else {
		// reallocate message buffer to accomodate new data
		char* temp = static_cast<char*>(realloc(m.bytes.get_ptr(), m.total_bytes + t.total_bytes));
		if (!temp) {
			return false;
		}
		m.bytes = temp;
		m.total_bytes += t.total_bytes;
		// join previously received message data with latest transfer data
		m.append(t.bytes.get_ptr(), t.total_bytes);
	}
	// clear contents of transfer
	t = {};
	return true;
}

/**
 * out_token_handler()
 *
 * This is a callback function which is invoked when a USB token with PID=OUT
 * is received on the bulk out endpoint.
 * This function is responsible for decoding bulk-out transfers from the host
 * and calling handle_current_message() once the USBTMC message transfer is complete.
 *
 * NOTE: `packet_data' is owned by the caller, and must not be freed by this function!
 */
int USBTMCInterface::out_token_handler(char* packet_data, unsigned int bytes)
{
	if (!bytes) {
		return -1;
	}

	if (out_state.current_state == USBTMCBulkOutState::IDLE) {
		out_state.current_state = USBTMCBulkOutState::PROCESSING_TRANSACTION;
		// this is a new transfer
		if (bytes < 12) { // 12 == size of USBTMC Bulk-OUT header
			// USBTMC: 3.2.2.3: Table 7: Index 1
			// incomplete Bulk-OUT header received.
			this->bulk_out->stall();
			out_state.current_state = USBTMCBulkOutState::ERROR;
			return false;
		}

		char& MsgID{ packet_data[0] };
		unsigned int& total_bytes = out_state.current_transfer.total_bytes;

		if (MsgID == DEV_DEP_MSG_OUT) {
			USBTMCDevDepMsgOut msg{ packet_data };
			total_bytes = msg.template get<1>() + USBTMCDevDepMsgOut::size();
		}
		else if (MsgID == REQUEST_DEV_DEP_MSG_IN) {
			// there will be no more bytes in the future
			total_bytes = bytes;
		}
		else {
			while (1);
		}
		// TODO: do some kind of check on transfer size...
		out_state.current_transfer = InProgressTransfer{ total_bytes };
		if (!out_state.current_transfer.bytes) {
			return -1;
		}
		// copy over already-received bytes
		out_state.current_transfer.append(packet_data, bytes);
		// record our progress so far
		out_state.current_state = (bytes < total_bytes) ?
			USBTMCBulkOutState::RXING_BULK_OUT_TRANSFER : USBTMCBulkOutState::TRANSFER_COMPLETED;
	}
	else if (out_state.current_state == USBTMCBulkOutState::RXING_BULK_OUT_TRANSFER) {
		// continue receiving Bulk-OUT message
		// current_transfer.bytes guaranteed to be non-null
		// delta = bytes left in this transfer
		out_state.current_state = USBTMCBulkOutState::PROCESSING_TRANSACTION;
		unsigned int delta = out_state.current_transfer.total_bytes
			- out_state.current_transfer.bytes_so_far;
		if (bytes > delta + 3) {
			// we can have at most 3 padding bytes that are not
			// included in `transferSize', and hence,
			// current_transfer.total_bytes
			return -1;
		}

		// if this transaction doesn't fill the buffer, copy the entire
		// received data.  otherwise, don't overfill the buffer by
		// copying any padding bytes
		out_state.current_transfer.append(packet_data, bytes);

		// take account of newly received bytes
		out_state.current_state = (out_state.current_transfer.bytes_so_far == out_state.current_transfer.total_bytes) ? USBTMCBulkOutState::TRANSFER_COMPLETED : USBTMCBulkOutState::RXING_BULK_OUT_TRANSFER;
	}
	
	// if the transfer is complete, and this is the last transfer, the message
	// has been completely received
	if (out_state.current_state == USBTMCBulkOutState::TRANSFER_COMPLETED) {
		// move contents of completed transfer to message buffer
		if (!append_transfer_to_message(out_state.current_transfer, out_state.current_message)) {
			out_state.current_message = {};
			out_state.current_transfer = {};
			return false;
		}

		// for DEV_DEP_MSG_OUT, bmTransferAttributes = 1 <=> message complete
		bool lastTransfer = (out_state.current_message.bytes[8] & 1) || out_state.current_message.bytes[0] == REQUEST_DEV_DEP_MSG_IN;
		if (lastTransfer) {
			out_state.current_state = USBTMCBulkOutState::PROCESSING_MESSAGE;
			return handle_current_message();
		}
	}

	return true;
}

int USBTMCInterface::handle_dev_dep_msg_out(char* packet_data, unsigned int bytes)
{
	if (bytes < USBTMCDevDepMsgOut::size()) {
		// msg is too short
		return -1;
	}

	// deserialize device-dependent message header
	USBTMCDevDepMsgOut msg{ packet_data };

	USBTMCBulkHeaderBase& hdr{ msg.template get<0>() };
	unsigned char& MsgID{ hdr.template get<0>() };
	unsigned char& bTag{ hdr.template get<1>() };
	unsigned int& transferSize{ msg.template get<1>() };
	char* msg_start = &packet_data[USBTMCDevDepMsgOut::size()];

	if (USBTMCDevDepMsgOut::size() + transferSize != bytes) {
		return -1;
	}
	if (dev_dep_msg_out_handler) {
		return (*dev_dep_msg_out_handler)(MsgID, bTag, transferSize, msg_start);
	}
	else {
		return -1;
	}
}

int USBTMCInterface::handle_dev_dep_msg_in(char* packet_data, unsigned int bytes)
{
	// host is requesting that we send a device-dependent response message
	if (bytes < USBTMCRequestDevDepMsgIn::size()) {
		// msg is too short
		return -1;
	}

	// deserialize header
	USBTMCRequestDevDepMsgIn msg{ packet_data };

	USBTMCBulkHeaderBase& hdr{ msg.template get<0>() };
	unsigned char& MsgID{ hdr.template get<0>() };
	unsigned char& bTag{ hdr.template get<1>() };
	unsigned int& transferSize{ msg.template get<1>() };
	unsigned char& bmTransferAttributes{ msg.template get<2>() };
	unsigned char& termChar{ msg.template get<3>() };

	// record bTag for this request in case of a future INITIATE_ABORT_BULK_IN
	bulk_in->last_bTag = bTag;
	
	if (dev_dep_msg_in_req_handler) {
		return (*dev_dep_msg_in_req_handler)(*bulk_in, MsgID, bTag, transferSize,
											 bmTransferAttributes | 0x02,
											 termChar);
	}
	else {
		return -1;
	}
}

int USBTMCInterface::handle_vendor_specific_out(char* packet_data, unsigned int bytes)
{
	return false;
}

int USBTMCInterface::handle_vendor_specific_in(char* packet_data, unsigned int bytes)
{
	// host is requesting that we send a vendor-specific response message
	return false;
}

int USBTMCInterface::in_token_handler()
{
	return false;
}

/**
 * USBTMCSetupRequestHandler implementation
 *
 * USBTMCSetupRequestHandler implements the USBSetupRequestHandler interface,
 * which is used by a USBDevice to dispatch incoming default control endpoint
 * SETUP messages to.
 *
 * This is used by USBTMCDevice to ensure that class-specific SETUP messages
 * are responded to appropriately.
 *
 * Most of the handlers are stubs, as the USBTMC class doesn't mandate special
 * behavior for most messages.
 */

int USBTMCDevice::USBTMCSetupRequestHandler::get_status(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_clear_feature(USBDevice& device, char*)
{
	// TODO
	device.ep0.sendZLP();
	return true;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_set_feature(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_set_address(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_get_descriptor(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_set_descriptor(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_get_configuration(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_set_configuration(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_get_interface(USBDevice&, char*)
{
	return false;
}

int USBTMCDevice::USBTMCSetupRequestHandler::usb_set_interface(USBDevice&, char*)
{
	return false;
}

USBTMCDevice create_usbtmc_device(const wchar_t* manufacturer_name,
				  const wchar_t* product_name,
				  const wchar_t* serial_number,
				  const USBTMC_bInterfaceProtocol protocol,
				  USBTMCCapabilities* capabilities,
				  const USBDeviceFactory& cstr,
				  const Invokable<std::exclusive_ptr<USBOutEndpoint>()>& get_out_ep,
				  const Invokable<std::exclusive_ptr<USBInEndpoint>()>& get_in_ep,
				  USBTMCInterface::out_msg_handler& out_handler,
				  USBTMCInterface::in_msg_handler& in_handler)
{
	USBConfigurationFactory configFactory =
		[&](unsigned char n) -> USBConfiguration*
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
			    new(std::nothrow) USBTMCInterface(protocol,
							      capabilities,
							      get_out_ep(),
							      get_in_ep());
			if (!iface) {
				delete new_config;
				return nullptr;
			}

			iface->addDevDepMsgOutHandler(out_handler);
			iface->addDevDepMsgInReqHandler(in_handler);

			new_config->interfaces.push_back(iface);
			return new_config;
		};
	USBTMCDevice tmc_dev{ cstr(std::move(configFactory)) };

	usbtmc_setup_request_handler* handler =
		new(std::nothrow) usbtmc_setup_request_handler{ manufacturer_name,
								product_name,
								serial_number };
	tmc_dev.addSetupRequestHandler(handler);
	return tmc_dev;
}
								  
