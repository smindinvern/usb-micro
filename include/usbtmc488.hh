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

#ifndef USBTMC_488_HH_
#define USBTMC_488_HH_

#include "usbtmc.hh"

/**
 * The TRIGGER command message provides a mechanism for the Host to trigger
 * device-dependent actions on a device synchronously with other Bulk-OUT
 * messages.  Support for this MsgID is optional.
 */
#define USBTMC_488_TRIG_MSG  128

class USBTMC488TrigMsg : public DeSerializable<USBTMCBulkHeaderBase,
					       unsigned int, /* Reserved */
					       unsigned int, /* Reserved */
					       unsigned int> /* Reserved */
{
	using DeSerializable::DeSerializable;
};

struct USBTMC488Capabilities
{
	struct USBTMCCapabilities base_caps;
	bool isUSB488_2;
	bool acceptsGoToLocal;
	bool acceptsTrigMsg;
	bool understandsMandatorySCPI;
	bool sr1Capable;
	bool rl1Capable;
	bool dt1Capable;
};

struct USBTMC488GetCapabilitiesResponse : Serializable<USBTMCGetCapabilitiesResponse,
						       unsigned short, /* bcdUSB488 */
						       unsigned char, /* USB488 interface capabilities */
						       unsigned char, /* USB488 device capabilities */
						       unsigned char> /* Reserved */
{
	USBTMC488GetCapabilitiesResponse() = default;
	USBTMC488GetCapabilitiesResponse(unsigned char status,
					 unsigned short bcdUSBTMC,
					 bool acceptsIndicatorPulse,
					 bool talk_only,
					 bool listen_only,
					 bool supportsTermChar,
					 bool isUSB488_2,
					 bool acceptsGoToLocal,
					 bool acceptsTrigMsg,
					 bool understandsMandatorySCPI,
					 bool sr1Capable,
					 bool rl1Capable,
					 bool dt1Capable)
		: Serializable{
		USBTMCGetCapabilitiesResponse{
			status,
			bcdUSBTMC,
			acceptsIndicatorPulse,
			talk_only,
			listen_only,
			supportsTermChar
		},
		0x0100,
		char((isUSB488_2 << 2) | (acceptsGoToLocal << 1) | acceptsTrigMsg),
		char((understandsMandatorySCPI << 3) | (sr1Capable << 2) | (rl1Capable << 1) | dt1Capable),
		0x00
	} {}
};

/**
 * USB488 subclass specific requests
 */
enum USB488_class_requests {
	READ_STATUS_BYTE = 128,
	REN_CONTROL = 160,
	GO_TO_LOCAL,
	LOCAL_LOCKOUT
};

enum USB488_status {
	STATUS_INTERRUPT_IN_BUSY = 32
};

class USB488ReadStatusByteResponse : public Serializable<unsigned char, /* status */
							 unsigned char, /* bTag */
							 unsigned short> /* status byte */
{
	USB488ReadStatusByteResponse(unsigned char usbtmc_status,
				     unsigned char bTag,
				     unsigned short status_byte)
		: Serializable{ usbtmc_status, bTag, status_byte } {}
};

USBTMCDevice create_usbtmc488_device(
    const wchar_t* manufacturer_name,
    const wchar_t* product_name,
    const wchar_t* serial_number,
    const USBDeviceFactory& cstr,
    const Invokable<std::exclusive_ptr<USBOutEndpoint>()>& get_out_ep,
    const Invokable<std::exclusive_ptr<USBInEndpoint>()>& get_in_ep);

#endif // USBTMC_488_HH_
