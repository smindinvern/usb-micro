#include "usb.hh"
#include "usb_private.hh"

// Class field values
#define USBHID_CLASS (0x3)
#define USBHID_NO_SUBCLASS (0)
#define USBHID_BOOT_INTERFACE_SUBCLASS (1)

// Subclass field values
#define USBHID_PROTOCOL_NONE (0)
#define USBHID_PROTOCOL_KEYBOARD (1)
#define USBHID_PROTOCOL_MOUSE (2)

// Descriptor type values
#define USBHID_CLASS_DESCRIPTOR (0x21)
#define USBHID_REPORT_DESCRIPTOR (0x22)
#define USBHID_PHYSICAL_DESCRIPTOR (0x23)

// Class specific request type values
#define USBHID_GET_REPORT   (0x01)
#define USBHID_GET_IDLE     (0x02)
#define USBHID_GET_PROTOCOL (0x03)
#define USBHID_SET_REPORT   (0x09)
#define USBHID_SET_IDLE     (0x0A)
#define USBHID_SET_PROTOCOL (0x0B)

// Country codes
#define USBHID_CC_ARABIC (1)
#define USBHID_CC_BELGIAN (2)
#define USBHID_CC_CANADIAN_BILINGUAL (3)
#define USBHID_CC_CANADIAN_FRENCH (4)
#define USBHID_CC_CZECH_REPUBLIC (5)
#define USBHID_CC_DANISH (6)
#define USBHID_CC_FINNISH (7)
#define USBHID_CC_FRENCH (8)
#define USBHID_CC_GERMAN (9)
#define USBHID_CC_GREEK (10)
#define USBHID_CC_HEBREW (11)
#define USBHID_CC_HUNGARY (12)
#define USBHID_CC_INTERNATIONAL_ISO (13)
#define USBHID_CC_ITALIAN (14)
#define USBHID_CC_JAPAN_KATAKANA (15)
#define USBHID_CC_KOREAN (16)
#define USBHID_CC_LATIN_AMERICAN (17)
#define USBHID_CC_NETHERLANDS_DUTCH (18)
#define USBHID_CC_NORWEGIAN (19)
#define USBHID_CC_PERSIAN_FARSI (20)
#define USBHID_CC_POLAND (21)
#define USBHID_CC_PORTUGUESE (22)
#define USBHID_CC_RUSSIA (23)
#define USBHID_CC_SLOVAKIA (24)
#define USBHID_CC_SPANISH (25)
#define USBHID_CC_SWEDISH (26)
#define USBHID_CC_SWISS_FRENCH (27)
#define USBHID_CC_SWISS_GERMAN (28)
#define USBHID_CC_SWITZERLAND (29)
#define USBHID_CC_TAIWAN (30)
#define USBHID_CC_TURKISH_Q (31)
#define USBHID_CC_UK (32)
#define USBHID_CC_US (33)
#define USBHID_CC_YUGOSLAVIA (34)
#define USBHID_CC_TURKISH_F (35)


const char kb_descriptor[63] = {
0x05, 0x01,
0x09, 0x06,
0xa1, 0x01,
0x05, 0x07,
0x19, 0xe0,
0x29, 0xe7,
0x15, 0x00,
0x25, 0x01,
0x75, 0x01,
0x95, 0x08,
0x81, 0x02,
0x95, 0x01,
0x75, 0x08,
0x81, 0x03,
0x95, 0x05,
0x75, 0x01,
0x05, 0x08,
0x19, 0x01,
0x29, 0x05,
0x91, 0x02,
0x95, 0x01,
0x75, 0x03,
0x91, 0x03,
0x95, 0x06,
0x75, 0x08,
0x15, 0x00,
0x25, 0x65,
0x05, 0x07,
0x19, 0x00,
0x29, 0x65,
0x81, 0x00,
0xc0
};


struct USBKeyboardReportDescriptor
{
	static constexpr unsigned int size()
	{
		return sizeof(kb_descriptor);
	}
};

using USBRawHIDDescriptor = USBDescriptor<USBHID_CLASS_DESCRIPTOR,
					  unsigned short /* bcdHID */,
					  unsigned char /* bCountryCode */,
					  unsigned char /* bNumDescriptors */,
					  unsigned char /* bDescriptorType2 */,
					  unsigned short /* wDescriptorLength */>;

struct USBHIDDescriptor : public USBRawHIDDescriptor
{
	enum {
		hid_bLength = 0,
		hid_bDescriptorType,
		hid_bcdHID,
		hid_bCountryCode = hid_bcdHID + 2,
		hid_bNumDescriptors,
		hid_bDescriptorType2,
		hid_wDescriptorLength,
		hid_size = hid_wDescriptorLength + 2
	};

	USBHIDDescriptor(unsigned short bcdHID,
			 unsigned char bCountryCode,
			 unsigned char bNumDescriptors,
			 unsigned char bDescriptorType2,
			 unsigned short wDescriptorLength)
		: USBRawHIDDescriptor({ bcdHID,
					bCountryCode,
					bNumDescriptors,
					bDescriptorType2,
					wDescriptorLength }) {}
	USBHIDDescriptor(const USBHIDDescriptor&) = default;
	USBHIDDescriptor(USBHIDDescriptor&&) = default;
};
