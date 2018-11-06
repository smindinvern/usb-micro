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

#ifndef USB_PRIVATE_HH_
#define USB_PRIVATE_HH_

#include "mm.hh"
#include "Tuple.hh"

/* descriptor types */
enum {
	DEVICE = 1,
	CONFIGURATION,
	STRING,
	INTERFACE,
	ENDPOINT,
	DEVICE_QUALIFIER,
	OTHER_SPEED_CONFIGURATION,
	INTERFACE_POWER,
};

/* request types */
enum {
	GET_STATUS = 0,
	CLEAR_FEATURE,
	SET_FEATURE = 3,
	SET_ADDRESS = 5,
	GET_DESCRIPTOR,
	SET_DESCRIPTOR,
	GET_CONFIGURATION,
	SET_CONFIGURATION,
	GET_INTERFACE,
	SET_INTERFACE,
	SYNCH_FRAME
};

struct USBStandardDeviceRequest : public DeSerializable<unsigned char,  /* bmRequestType */
							unsigned char,  /* bRequest */
							unsigned short, /* wValue */
							unsigned short, /* wIndex */
							unsigned short  /* wLength */>
{
	using DeSerializable::DeSerializable;
	unsigned char& bmRequestType()
	{
		return this->template get<0>();
	}
	unsigned char& bRequest()
	{
		return this->template get<1>();
	}
	unsigned short& wValue()
	{
		return this->template get<2>();
	}
	unsigned short& wIndex()
	{
		return this->template get<3>();
	}
	unsigned short& wLength()
	{
		return this->template get<4>();
	}
};

struct USBGetDescriptorRequest : public USBStandardDeviceRequest
{
	using USBStandardDeviceRequest::USBStandardDeviceRequest;
	unsigned char getDescriptorType()
	{
		return wValue() >> 8;
	}
	unsigned char getDescriptorIndex()
	{
		return wValue() & 0xff;
	}
};

/**
 * USB*Descriptor - classes to aid in representing and serializing the various
 * descriptors that need to be sent back to the host
 */
template<unsigned char bDescriptorType, class T, class U, class... V> struct USBDescriptor
{
public:
	static constexpr unsigned int size()
	{
		return 2 + decltype(data)::size();
	}
	
	typedef Serializable<T, U, V...> type;
	type data;

	USBDescriptor(const Tuple<T, U, V...>& value)
		: data(value) {}
	USBDescriptor(const USBDescriptor&) = default;
	USBDescriptor(USBDescriptor&&) = default;

	template<int I> auto get()
	{
		return data.template get<I>();
	}
	
	int copyTo(char* array, unsigned int length) const
	{
		if (!array) {
			return -1;
		}
		(Serializable<unsigned char, unsigned char>{ size(), bDescriptorType }).copyTo(array, length);
		data.copyTo(array + 2, (length >= 2) ? length - 2 : 0);
		return 0;
	}	
};

using USBRawDeviceDescriptor = USBDescriptor<DEVICE,
					     unsigned short /* bcdUSB */,
					     unsigned char /* bDeviceClass */,
					     unsigned char /* bDeviceSubClass */,
					     unsigned char /* bDeviceProtocol */,
					     unsigned char /* bMaxPacketSize0 */,
					     unsigned short /* idVendor */,
					     unsigned short /* idProduct */,
					     unsigned short /* bcdDevice */,
					     unsigned char /* iManufacturer */,
					     unsigned char /* iProduct */,
					     unsigned char /* iSerialNumber */,
					     unsigned char /* bNumConfigurations */>;

struct USBDeviceDescriptor : public USBRawDeviceDescriptor
{
	enum {
		device_bLength = 0,
		device_bDescriptorType,
		device_bcdUSB,
		device_bDeviceClass = device_bcdUSB + 2,
		device_bDeviceSubClass,
		device_bDeviceProtocol,
		device_bMaxPacketSize0,
		device_idVendor,
		device_idProduct = device_idVendor + 2,
		device_bcdDevice = device_idProduct + 2,
		device_iManufacturer = device_bcdDevice + 2,
		device_iProduct,
		device_iSerialNumber,
		device_bNumConfigurations,
		device_size
	};

	USBDeviceDescriptor(unsigned char bDeviceClass,
			    unsigned char bDeviceSubClass,
			    unsigned char bDeviceProtocol,
			    unsigned char bMaxPacketSize0,
			    unsigned short idVendor,
			    unsigned short idProduct,
			    unsigned short bcdDevice,
			    unsigned char iManufacturer,
			    unsigned char iProduct,
			    unsigned char iSerialNumber,
			    unsigned char bNumConfigurations)
		: USBDescriptor({	0x110,
					bDeviceClass,
					bDeviceSubClass,
					bDeviceProtocol,
					bMaxPacketSize0,
					idVendor,
					idProduct,
					bcdDevice,
					iManufacturer,
					iProduct,
					iSerialNumber,
					bNumConfigurations }) {}

	USBDeviceDescriptor(const USBDeviceDescriptor&) = default;
	USBDeviceDescriptor(USBDeviceDescriptor&&) = default;
};

using USBRawConfigurationDescriptor = USBDescriptor<CONFIGURATION,
						    unsigned short /* wTotalLength */,
						    unsigned char /* bNumInterfaces */,
						    unsigned char /* bConfigurationValue */,
						    unsigned char /* iConfiguration */,
						    unsigned char /* bmAttributes */,
						    unsigned char /* bMaxPower */>;

struct USBConfigurationDescriptor : public USBRawConfigurationDescriptor
{
	unsigned short wTotalLength_;
	
	enum {
		configuration_bLength = 0,
		configuration_bDescriptorType,
		configuration_wTotalLength,
		configuration_bNumInterfaces = configuration_wTotalLength + 2,
		configuration_bConfigurationValue,
		configuration_iConfiguration,
		configuration_bmAttributes,
		configuration_bMaxPower,
		configuration_size
	};

	unsigned int total_length()
	{
		return wTotalLength_;
	}

	USBConfigurationDescriptor(unsigned short wTotalLength,
				   unsigned char bNumInterfaces,
				   unsigned char bConfigurationValue,
				   unsigned char iConfiguration,
				   unsigned char bmAttributes,
				   unsigned char bMaxPower)
		: USBRawConfigurationDescriptor({ wTotalLength,
					bNumInterfaces,
					bConfigurationValue,
					iConfiguration,
					bmAttributes,
					bMaxPower }),
		  wTotalLength_{ wTotalLength } {}
	USBConfigurationDescriptor(const USBConfigurationDescriptor&) = default;
	USBConfigurationDescriptor(USBConfigurationDescriptor&&) = default;
};

using USBRawInterfaceDescriptor = USBDescriptor<INTERFACE,
						unsigned char /* bInterfaceNumber */,
						unsigned char /* bAlternateSetting */,
						unsigned char /* bNumEndpoints */,
						unsigned char /* bInterfaceClass */,
						unsigned char /* bInterfaceSubClass */,
						unsigned char /* bInterfaceProtocol */,
						unsigned char /* iInterface */>;

struct USBInterfaceDescriptor : public USBRawInterfaceDescriptor
{
	enum {
		interface_bLength = 0,
		interface_bDescriptorType,
		interface_bInterfaceNumber,
		interface_bAlternateSetting,
		interface_bNumEndpoints,
		interface_bInterfaceClass,
		interface_bInterfaceSubClass,
		interface_bInterfaceProtocol,
		interface_iInterface,
		interface_size
	};

	USBInterfaceDescriptor(unsigned char bInterfaceNumber,
			       unsigned char bAlternateSetting,
			       unsigned char bNumEndpoints,
			       unsigned char bInterfaceClass,
			       unsigned char bInterfaceSubClass,
			       unsigned char bInterfaceProtocol,
			       unsigned char iInterface)
		: USBRawInterfaceDescriptor({ bInterfaceNumber,
					bAlternateSetting,
					bNumEndpoints,
					bInterfaceClass,
					bInterfaceSubClass,
					bInterfaceProtocol,
					iInterface }) {}
	USBInterfaceDescriptor(const USBInterfaceDescriptor&) = default;
	USBInterfaceDescriptor(USBInterfaceDescriptor&&) = default;
};

using USBRawEndpointDescriptor = USBDescriptor<ENDPOINT,
					       unsigned char /* bEndpointAddress */,
					       unsigned char /* bmAttributes */,
					       unsigned short /* wMaxPacketSize */,
					       unsigned char /* bInterval */>;

struct USBEndpointDescriptor : public USBRawEndpointDescriptor
{
	enum {
		endpoint_bLength = 0,
		endpoint_bDescriptorType,
		endpoint_bEndpointAddress,
		endpoint_bmAttributes,
		endpoint_wMaxPacketSize,
		endpoint_bInterval = endpoint_wMaxPacketSize + 2,
		endpoint_size
	};

	USBEndpointDescriptor(unsigned char bEndpointAddress,
			      unsigned char bmAttributes,
			      unsigned short wMaxPacketSize,
			      unsigned char bInterval)
		: USBRawEndpointDescriptor({ bEndpointAddress,
					bmAttributes,
					wMaxPacketSize,
					bInterval }) {}
	USBEndpointDescriptor(const USBEndpointDescriptor&) = default;
	USBEndpointDescriptor(USBEndpointDescriptor&&) = default;
};

#define usb_func(x) \
	samd_usb_##x

#define __usb_set_address usb_func(set_address)
#define __usb_set_configured usb_func(set_configured)
#define __usb_init_usb usb_func(init_usb)
#define __usb_ep_isr usb_func(ep_isr)
#define __usb_enter_default_state usb_func(enter_default_state)
#define __usb_enable usb_func(enable)

#ifdef __cplusplus
extern "C" {
#endif
	void __usb_set_address(const unsigned int);
	void __usb_set_configured(const bool);
	void __usb_init_usb();
	void __usb_ep_isr(unsigned int);
	void __usb_enter_default_state();
	void __usb_enable();
#ifdef __cplusplus
}
#endif

#endif  // USB_PRIVATE_HH_
