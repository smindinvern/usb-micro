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

#ifndef SAMD21_USB_HH_
#define SAMD21_USB_HH_

#ifdef __cplusplus
extern "C" {
#endif
	void samd_usb_set_address(unsigned int address);
	void samd_usb_set_configured(bool);
	void samd_usb_init_usb(void);
	void samd_usb_ep_isr(unsigned int ep);
	void samd_usb_enter_default_state(void);
	void samd_usb_enable(void);
#ifdef __cplusplus
}
#endif

#include "main.hh"
#include "mm.hh"
#include "usb.hh"

#include "atsamd21/atsamd21.hh"

// USB common device registers
#define USB_CTRLA        (USB + 0x00UL)
#define USB_SYNCBUSY     (USB + 0x02UL)
#define USB_QOSCTRL      (USB + 0x03UL)
#define USB_FSMSTATUS    (USB + 0x0DUL)
#define USB_DESCADD      (USB + 0x24UL)
#define USB_PADCAL       (USB + 0x28UL)

// USB general device registers
#define USB_CTRLB        (USB + 0x08UL)
#define USB_DADD         (USB + 0x0AUL)
#define USB_STATUS       (USB + 0x0CUL)
#define USB_FNUM         (USB + 0x10UL)
#define USB_INTENCLR     (USB + 0x14UL)
#define USB_INTENSET     (USB + 0x18UL)
#define USB_INTFLAG      (USB + 0x1CUL)
#define USB_EPINTSMRY    (USB + 0x20UL)

// Device endpoint n registers
#define USB_EPCFG(n)       (USB + 0x100UL + ((unsigned int)n * 0x20UL))
#define USB_EPSTATUSCLR(n) (USB + 0x104UL + ((unsigned int)n * 0x20UL))
#define USB_EPSTATUSSET(n) (USB + 0x105UL + ((unsigned int)n * 0x20UL))
#define USB_EPSTATUS(n)    (USB + 0x106UL + ((unsigned int)n * 0x20UL))
#define USB_EPINTFLAG(n)   (USB + 0x107UL + ((unsigned int)n * 0x20UL))
#define USB_EPINTENCLR(n)  (USB + 0x108UL + ((unsigned int)n * 0x20UL))
#define USB_EPINTENSET(n)  (USB + 0x109UL + ((unsigned int)n * 0x20UL))

struct samd21_ep_bank_desc {
	unsigned int addr:32;     // Data buffer address in RAM
	/**
	 * Packet Size
	 *
	 * 31:         AUTO_ZLP
	 * 30..28:     SIZE
	 * 27..14:     MULTI_PACKET_SIZE
	 * 13..0:      BYTE_COUNT
	 */
	unsigned int pcksize:32;
	/**
	 * Extended Register
	 *
	 * 14..4:      VARIABLE
	 * 3..0:       SUBPID
	 */
	unsigned int extreg:32;
	/**
	 * Device Status Bank
	 *
	 * 1:          ERRORFLOW
	 * 0:          CRCERR
	 */
	unsigned char status_bk:8;
	unsigned char :8;  // Single padding byte
};

struct samd21_ep_desc {
	struct samd21_ep_bank_desc bank0;
	unsigned char :0;  // Prevent addition of padding bytes
	struct samd21_ep_bank_desc bank1;
	unsigned char :0;  // Prevent addition of padding bytes
};


/**
 * struct SAMDUSBEndpointImpl
 *
 * Provides the USBEndpoint implementation for Microchip SAMD hardware
 */

#ifdef __cplusplus
struct SAMDUSBEndpointImpl : public USBEndpointImpl
{
private:
	unsigned char ep_num;
	bool dir;  // false => in, true => out
	USBEndpoint::ep_type ep_type;
	samd21_ep_desc* desc;
	unsigned short max_pack_size;
public:
	virtual void reset()
	{
		// Free buffers (if they exist).
		delete[] reinterpret_cast<char*>(desc->bank0.addr);
		delete[] reinterpret_cast<char*>(desc->bank1.addr);
		// Allocate new buffers.
		configure_bank(dir, log2(max_pack_size >> 3));
		if (ep_type == USBEndpoint::ep_type::control_ep) {
			configure_bank(!dir, log2(max_pack_size >> 3));
		}
		// Configure the endpoint.
		Reg8 epcfg{ USB_EPCFG(ep_num) };
		if (ep_type == USBEndpoint::ep_type::control_ep) {
			epcfg = (1 << 4) | 1;
		}
		else {
			unsigned char k;
			switch (ep_type) {
			case USBEndpoint::ep_type::isochronous_ep:
				k = 0x2;
				break;
			case USBEndpoint::ep_type::bulk_ep:
				k = 0x3;
				break;
			case USBEndpoint::ep_type::interrupt_ep:
				k = 0x4;
				break;
			default:
				return;
			}
			const unsigned char shift = dir ? 0 : 4;
			epcfg = k << shift;
		}

		// Enable all interupts for this endpoint
		Reg8 epintenset{ USB_EPINTENSET(ep_num) };
		// EPINTENSET.STALL0/1 = 1
		// EPINTENSET.RXSTP = 1
		// EPINTENSET.TRFAIL0/1 = 1
		// EPINTENSET.TRCPT0/1 = 1
		if (ep_type == USBEndpoint::ep_type::control_ep) {
			epintenset = 0x7F;
		}
		else if (dir == false) { // IN endpoint
			epintenset = (1 << 3) | (1 << 1);
		}
		else { // OUT endpoint
			epintenset = (1 << 5) | (1 << 2) | 1;
		}

		// Clear data toggle bits, etc.
		Reg8 epstatusclr{ USB_EPSTATUSCLR(ep_num) };
		epstatusclr = 0xFF;
		if (dir == false) { // IN endpoint
			Reg8 epstatusset{ USB_EPSTATUSSET(ep_num) };
			epstatusset = 1 << 2;
		}
	}
	virtual void stall()
	{
		Reg8 epstatusset{ USB_EPSTATUSSET(ep_num) };
		unsigned char shift = dir ? 4 : 5;
		epstatusset = 1 << shift;
	}
	virtual void unstall()
	{
		Reg8 epstatusclr{ USB_EPSTATUSCLR(ep_num) };
		unsigned char shift = dir ? 4 : 5;
		epstatusclr = 1 << shift;
	}
	void set_tx_size(unsigned short size)
	{
		desc->bank1.pcksize &= ~0x3FFF;
		desc->bank1.pcksize |= (size & 0x3FFF);
	}
	unsigned short get_rx_size()
	{
		return desc->bank0.pcksize & 0x3FFF;
	}
	virtual int send_data(const char* buf, unsigned int size, bool wait)
	{
		unsigned int addr = desc->bank1.addr;
		desc->bank1.addr = reinterpret_cast<const unsigned int>(buf);
		set_tx_size(size);
		// Set EPSTATUS.BK1RDY to indicate data is ready to send
		do_dmb();
		Reg8 epstatusset{ USB_EPSTATUSSET(ep_num) };
		epstatusset = 1 << 7;
		// Wait for TRCPT1 interrupt flag to be set
		Reg8 epintflag{ USB_EPINTFLAG(ep_num) };
		while ((epintflag & (1 << 1)) == 0);
		desc->bank1.addr = addr;
		// Clear TRCPT1 interrupt flag
		epintflag = 1 << 1;
		return 0;
	}
	char* get_data(unsigned int& size)
	{
		size = get_rx_size();
		char* data{ new(std::nothrow) char[size] };
		memcpy(data, reinterpret_cast<char*>(desc->bank0.addr), size);
		// Clear BANKRDY0 bit to allow future incoming transactions.
		Reg8 epstatusclr{ USB_EPSTATUSCLR(ep_num) };
		epstatusclr = 1 << 6;
		return data;
	}
	virtual char* read_data(unsigned int& size)
	{
		char* data{ get_data(size) };
		// Clear TRCPT0 interrupt flag
		Reg8 epintflag{ USB_EPINTFLAG(ep_num) };
		epintflag = 1;
		return data;
	}
	virtual char* read_setup(unsigned int& size)
	{
		char* data{ get_data(size) };
		// Clear RXSTP and TRCPT0 interrupt flags
		Reg8 epintflag{ USB_EPINTFLAG(ep_num) };
		epintflag = (1 << 4) | 1;
		return data;
	}
	samd21_ep_bank_desc* bank(const bool dir_) const
	{
		return dir_ ? &desc->bank0 : &desc->bank1;
	}
	void configure_bank(const bool dir_, const unsigned char size_bits)
	{
		unsigned int size{ 1UL << (size_bits + 3) };
		bank(dir_)->addr = reinterpret_cast<unsigned int>(new(std::nothrow) char[size]);
		// PCKSIZE.SIZE = size_bits
		// PCKSIZE.AUTO_ZLP = 0
		// PCKSIZE.MULTI_PACKET_SIZE = 0
		// PCKSIZE.BYTE_COUNT = 0
		bank(dir_)->pcksize = size_bits << 28;
	}
	SAMDUSBEndpointImpl(unsigned char ep_num_,
						bool dir_,
						USBEndpoint::ep_type ep_type_,
						unsigned short size,
						samd21_ep_desc* descs_)
		: ep_num{ ep_num_ },
		  dir{ dir_ },
		  ep_type{ ep_type_ },
		  desc{ &descs_[ep_num_] },
		  max_pack_size{ size }
	{
		// When `reset()' is called, it frees these buffers, so we need
		// to make sure that when it is called after EORST the free
		// becomes a NOP.
		desc->bank0.addr = 0;
		desc->bank1.addr = 0;
		reset();
	}
	virtual ~SAMDUSBEndpointImpl()
	{
		// Disable endpoint
		Reg8 epcfg{ USB_EPCFG(ep_num) };
		epcfg = 0;
		// Free associated endpoint descriptors
		delete[] reinterpret_cast<char*>(desc->bank0.addr);
		desc->bank0.addr = 0;
		delete[] reinterpret_cast<char*>(desc->bank1.addr);
		desc->bank1.addr = 0;
	}
};

struct SAMDUSBInEndpoint : public USBInEndpoint {
	SAMDUSBInEndpoint(unsigned char epNum,
					  USBEndpoint::ep_type epType,
					  unsigned short maxPackSize,
					  unsigned char interval,
					  samd21_ep_desc* descs)
		: USBInEndpoint((unsigned char)(epNum | 0x80), epType, maxPackSize, interval,
						new(std::nothrow) SAMDUSBEndpointImpl(epNum, false, epType, maxPackSize, descs)) {}
	SAMDUSBInEndpoint(const SAMDUSBInEndpoint&) = delete;
	SAMDUSBInEndpoint(SAMDUSBInEndpoint&&) = default;
};

struct SAMDUSBOutEndpoint : public USBOutEndpoint {
	SAMDUSBOutEndpoint(unsigned char epNum,
					   USBEndpoint::ep_type epType,
					   unsigned short maxPackSize,
					   unsigned char interval,
					   samd21_ep_desc* descs)
		: USBOutEndpoint(epNum, epType, maxPackSize, interval,
						 new(std::nothrow) SAMDUSBEndpointImpl(epNum, true, epType, maxPackSize, descs)) {}
	SAMDUSBOutEndpoint(const SAMDUSBOutEndpoint&) = delete;
	SAMDUSBOutEndpoint(SAMDUSBOutEndpoint&&) = default;
};

struct SAMDUSBControlEndpoint : public USBControlEndpoint {
	SAMDUSBControlEndpoint(unsigned char epNum,
						   unsigned short maxPackSize,
						   samd21_ep_desc* descs)
		: USBControlEndpoint(epNum, maxPackSize,
							 new(std::nothrow) SAMDUSBEndpointImpl(epNum, true, USBEndpoint::ep_type::control_ep, maxPackSize, descs)) {}
	SAMDUSBControlEndpoint(const SAMDUSBControlEndpoint&) = delete;
	SAMDUSBControlEndpoint(SAMDUSBControlEndpoint&&) = default;
};

USBDevice create_samd21_usb_device(const unsigned char n_eps);
#endif

#endif  // SAMD21_USB_HH_
