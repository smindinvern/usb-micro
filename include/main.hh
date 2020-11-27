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

#ifndef MAIN_HH_
#define MAIN_HH_

#include "stdbool.hh"

#ifdef __cplusplus
extern "C" {
#endif
	void wait(unsigned short ticks);
#ifdef __cplusplus
}
#endif

#include "Registers.hh"

#include "usb.hh"
#include "i2c.hh"

// Global information needed e.g. for servicing interrupts.
struct globals {
	struct usb_status_info usb_info;
	struct i2c_status_info i2c_info;
};

inline volatile struct globals* getGlobals()
{
	// For C compatibility: cast notation from integral to pointer type should
	// be equivalent to C++'s reinterpret_cast<>().
	return (volatile struct globals *)(0x20000000);
}

inline volatile struct i2c_status_info* getI2CStatusInfo()
{
	return &getGlobals()->i2c_info;
}

inline volatile struct usb_status_info* getUSBStatusInfo()
{
	return &getGlobals()->usb_info;
}

#ifdef __cplusplus
template<class T> T& getUSBEndpoint(unsigned char ep)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };
	return *static_cast<T*>(usb_status->usb_eps[ep]);
}
#endif


#endif  // MAIN_HH_
