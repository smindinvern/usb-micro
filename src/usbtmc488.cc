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
									 USBDeviceFactory& cstr,
									 Invokable<std::exclusive_ptr<USBOutEndpoint>()>& get_out_ep,
									 Invokable<std::exclusive_ptr<USBInEndpoint>()>& get_in_ep)
{
	return create_usbtmc_device(manufacturer_name,
								product_name,
								serial_number,
								USBTMC_USB488_interface,
								&interface_capabilities.base_caps,
								cstr,
								get_out_ep,
								get_in_ep,
								usbtmc488_dev_dep_out_handler,
								usbtmc488_dev_dep_in_handler);
}
