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

#ifndef SAM_USB_HH_
#define SAM_USB_HH_

#include "std.hh"
#include "usb.hh"
#include "sam.hh"

/**
 * struct AtmelSAMUSBEndpointImpl
 *
 * Provides the USBEndpoint implementation for Atmel SAM
 * hardware.
 */
struct AtmelUSBEndpointImplBase : public USBEndpointImpl {
    unsigned char ep_number;
    USBEndpoint::ep_type type;
    enum ep_dir {
	out_dir,
	in_dir,
	control_dir
    } dir;
    bool rx_ppb{};

    AtmelUSBEndpointImplBase(unsigned char epNumber, USBEndpoint::ep_type epType, ep_dir epDir)
	: ep_number{ epNumber },
	  type{ epType },
	  dir{ epDir } {}
};

struct AtmelSAMUSBEndpointImpl : public AtmelUSBEndpointImplBase {
    /* USBEndpointImpl interface implementation */
    virtual void reset();
    virtual void stall();
    virtual void unstall();
    virtual int send_data(char*, unsigned int, bool);
    virtual char* read_data(unsigned int&);
    virtual char* read_setup(unsigned int&);

    AtmelSAMUSBEndpointImpl(unsigned char epNumber, USBEndpoint::ep_type epType, ep_dir epDir);
};

struct AtmelSAMUSBInEndpoint : public USBInEndpoint {
    AtmelSAMUSBInEndpoint(unsigned char epNum, ep_type epType, unsigned short maxPackSize)
	: USBInEndpoint((unsigned char)(epNum | 0x80), epType, maxPackSize,
			new(std::nothrow) AtmelSAMUSBEndpointImpl(epNum, epType, AtmelSAMUSBEndpointImpl::in_dir)) {}
    AtmelSAMUSBInEndpoint(const AtmelSAMUSBInEndpoint&) = delete;
    AtmelSAMUSBInEndpoint(AtmelSAMUSBInEndpoint&&) = default;
};

struct AtmelSAMUSBOutEndpoint : public USBOutEndpoint {
    AtmelSAMUSBOutEndpoint(unsigned char epNum, ep_type epType, unsigned short maxPackSize)
	: USBOutEndpoint((unsigned char)(epNum & ~0x80), epType, maxPackSize,
			 new(std::nothrow) AtmelSAMUSBEndpointImpl(epNum, epType, AtmelSAMUSBEndpointImpl::out_dir)) {}
    AtmelSAMUSBOutEndpoint(const AtmelSAMUSBOutEndpoint&) = delete;
    AtmelSAMUSBOutEndpoint(AtmelSAMUSBOutEndpoint&&) = default;
};

struct AtmelSAMUSBControlEndpoint : public USBControlEndpoint {
    AtmelSAMUSBControlEndpoint(unsigned char epNum, unsigned short maxPackSize)
	: USBControlEndpoint((unsigned char)(epNum & ~0x80), maxPackSize,
			     new(std::nothrow) AtmelSAMUSBEndpointImpl(epNum, control_ep, AtmelSAMUSBEndpointImpl::control_dir)) {}
    AtmelSAMUSBControlEndpoint(const AtmelSAMUSBControlEndpoint&) = delete;
    AtmelSAMUSBControlEndpoint(AtmelSAMUSBControlEndpoint&&) = default;
};

inline USBDevice create_sam4s_usb_device(USBDeviceDescriptor descriptor,
										 Invokable<USBConfiguration*(unsigned char)>&& configFactory)
{
	USBDevice dev{ AtmelSAMUSBControlEndpoint{ 0, 64 },
			       descriptor, std::move(configFactory),
				   new(std::nothrow) USBDeviceGenericImpl() };
	// At this point all endpoints have been configured.  All that remains
	// to be done is to enable the USB function as a whole.
	sam_usb_enable();
	return dev;
}

#endif
