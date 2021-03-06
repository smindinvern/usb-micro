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

USBTMCDevice create_usbtmc488_device(
    const wchar_t* manufacturer_name,
    const wchar_t* product_name,
    const wchar_t* serial_number,
    const USBDeviceFactory* cstr,
    const Invokable<std::exclusive_ptr<USBOutEndpoint>()>* get_out_ep,
    const Invokable<std::exclusive_ptr<USBInEndpoint>()>* get_in_ep,
    const USBTMCInterface::out_msg_handler* out_handler,
    const USBTMCInterface::in_msg_handler* in_handler)
{
    return create_usbtmc_device(
	manufacturer_name,
	product_name,
	serial_number,
	USBTMC_USB488_interface,
	reinterpret_cast<USBTMCCapabilities*>(&interface_capabilities),
	cstr,
	get_out_ep,
	get_in_ep,
	out_handler,
	in_handler);
}
    
