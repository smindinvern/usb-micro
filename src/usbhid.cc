#include "usbhid.hh"
#include "usb.hh"
#include "mm.hh"
#include "main.hh"

class USBHIDReportDescriptor
{
public:
	class InputTag;
	class OutputTag;
	class FeatureTag;
	class CollectionTag;
	class EndCollectionTag;

	// Short report items
	enum ItemType {
		MainItemType = 0,
		GlobalItemType = 1,
		LocalItemType = 2,
		ReservedItemType = 3
	};
	template<unsigned char bSize, // 2 bits
			 enum ItemType bType, // 2 bits
			 unsigned char bTag>  // 4 bits
	struct ShortItem;
	template<enum ItemType bType,
			 unsigned char bTag>
	struct ShortItem<0, bType, bTag>
	{
		unsigned char _bytes[1];
		ShortItem() : _bytes{ (bTag << 4) | ((unsigned int)bType << 2) } {}
	};
	template<unsigned char bSize,
			 enum ItemType bType,
			 unsigned char bTag>
	struct ShortItem
	{
		constexpr static unsigned char nBytes{ (bSize == 0 ? 0 : (1 << (bSize - 1))) };
		unsigned char _bytes[nBytes + 1];
		ShortItem(const unsigned char (&_data)[nBytes])
		{
			_bytes[0] = (bTag << 4) | ((unsigned int)bType << 2) | bSize;
			memcpy(&_bytes[1], _data, nBytes);
		}
	};
	template<enum ItemType bType,
			 unsigned char bTag>
	using ShortItem0 = ShortItem<0, bType, bTag>;
	template<enum ItemType bType,
			 unsigned char bTag>
	using ShortItem1 = ShortItem<1, bType, bTag>;
	template<enum ItemType bType,
			 unsigned char bTag>
	using ShortItem2 = ShortItem<2, bType, bTag>;
	template<enum ItemType bType,
			 unsigned char bTag>
	using ShortItem4 = ShortItem<4, bType, bTag>;

	template<enum ItemType bType,
			 unsigned char bTag>
	struct OneByteItem : ShortItem1<bType, bTag>
	{
		OneByteItem(unsigned char value)
			: ShortItem1<bType, bTag>{ { value } } {};
	};
	template<enum ItemType bType,
			 unsigned char bTag>
	struct TwoByteItem : ShortItem2<bType, bTag>
	{
		TwoByteItem(unsigned short value)
			: ShortItem2<bType, bTag>{
			{ (unsigned char)(value & 0xFF), (unsigned char)(value >> 8) }
		} {}
	};

	// Main Items
	template<unsigned char bTag>
	struct MainItem : public ShortItem2<MainItemType, bTag>
	{
		MainItem(bool isConstant,
				 bool isArray,
				 bool isRelative,
				 bool isWrapped,
				 bool isLinear,
				 bool hasPreferredState,
				 bool hasNullPosition,
				 bool isVolatile,
				 bool isBufferedBytes)
			: ShortItem2<MainItemType, bTag>{ {
				(unsigned char)((isVolatile << 7)
								| (hasNullPosition << 6)
								| (!hasPreferredState << 5)
								| (!isLinear << 4)
								| (isWrapped << 3)
								| (isRelative << 2)
								| (!isArray << 1)
								| isConstant),
					isBufferedBytes } } {}
	};
	struct InputItem : public MainItem<0x8>
	{
		InputItem(bool isConstant, bool isArray, bool isRelative, bool isWrapped,
				  bool isLinear, bool hasPreferredState, bool hasNullPosition,
				  bool isBufferedBytes)
			: MainItem(isConstant, isArray, isRelative, isWrapped, isLinear,
					   hasPreferredState, hasNullPosition, false, isBufferedBytes) {}
	};
	struct OutputItem : public MainItem<0x9>
	{
		using MainItem::MainItem;
	};
	struct FeatureItem : public MainItem<0xB>
	{
		using MainItem::MainItem;
	};

	enum CollectionTypes {
		PhysicalCollection = 0x00,
		ApplicationCollection = 0x01,
		LogicalCollection = 0x02,
		ReportCollection = 0x03,
		NamedArrayCollection = 0x04,
		UsageSwitchCollection = 0x05,
		UsageModifierCollection = 0x06
	};
	struct CollectionItem : public OneByteItem<MainItemType, 0xA>
	{
		CollectionItem(CollectionTypes cType)
			: OneByteItem<MainItemType, 0xA>{ static_cast<unsigned char>(cType) } {}
	};
	struct EndCollectionItem : public ShortItem0<MainItemType, 0xC>
	{
		EndCollectionItem() = default;
	};

	// Global Items
	typedef TwoByteItem<GlobalItemType, 0x0> UsagePageItem;
	typedef TwoByteItem<GlobalItemType, 0x1> LogicalMinimumItem;
	typedef TwoByteItem<GlobalItemType, 0x2> LogicalMaximumItem;
	typedef TwoByteItem<GlobalItemType, 0x3> PhysicalMinimumItem;
	typedef TwoByteItem<GlobalItemType, 0x4> PhysicalMaximumItem;
	typedef OneByteItem<GlobalItemType, 0x5> UnitExponentItem;
	typedef OneByteItem<GlobalItemType, 0x6> UnitItem;
	typedef OneByteItem<GlobalItemType, 0x7> ReportSizeItem;
	typedef OneByteItem<GlobalItemType, 0x8> ReportIDItem;
	typedef OneByteItem<GlobalItemType, 0x9> ReportCountItem;
	typedef ShortItem0<GlobalItemType, 0xA> PushItem;

	// Local Items
	typedef TwoByteItem<LocalItemType, 0x0> UsageItem;
	typedef TwoByteItem<LocalItemType, 0x1> UsageMinimumItem;
	typedef TwoByteItem<LocalItemType, 0x2> UsageMaximumItem;
#if 0
	static const OneByteItem<LocalItemType, 0xA> OpenDelimiterItem{ 1 };
	static const OneByteItem<LocalItemType, 0xA> CloseDelimiterItem{ 0 };
#endif
	unsigned char* keyboard_protocol()
	{
		UsagePageItem generic_desktop{ 0x01 };
		UsageItem keyboard{ 0x06 };
		CollectionItem application{ ApplicationCollection };
		UsagePageItem keycodes{ 0x07 };
		UsageMinimumItem modifier_min{ 224 };
		UsageMaximumItem modifer_max{ 231 };
		LogicalMinimumItem modifier_log_min{ 0 };
		LogicalMaximumItem modifier_log_max{ 1 };
		ReportSizeItem modifier_size{ 1 };
		ReportCountItem modifier_count{ 8 };
		InputItem modifier_byte{ false, false, false, false, true, true, false, false };
		ReportCountItem reserved_byte_count{ 1 };
		ReportSizeItem reserved_byte_size{ 8 };
		InputItem reserved_byte{ true, false, false, false, true, false, false, false };
		ReportCountItem leds_count{ 5 };
		ReportSizeItem leds_size{ 1 };
		UsagePageItem leds_page{ 0x08 };
		UsageMinimumItem leds_min{ 1 };
		UsageMaximumItem leds_max{ 5 };
		OutputItem leds_report{ false, false, false, false, false, false, false, false, false };
		ReportCountItem led_pad_count{ 1 };
		ReportSizeItem led_pad_size{ 3 };
		OutputItem led_pad{ true, false, false, false, false, false, false, false, false };
		ReportCountItem key_codes_count{ 6 };
		ReportSizeItem key_codes_size{ 8 };
		LogicalMinimumItem key_codes_log_min{ 0 };
		LogicalMaximumItem key_codes_log_max{ 101 };
		UsagePageItem key_codes_page{ 0x07 };
		UsageMinimumItem key_codes_min{ 0 };
		UsageMaximumItem key_codes_max{ 101 };
		InputItem key_array{ false, true, false, false, true, true, false, false };
		EndCollectionItem end_collection{ };
		return nullptr;
	}

};



/**
 * USB HID support functions
 */
int usb_get_report(USBControlEndpoint* ep0, char* buf)
{
	if (buf[2] != 0 || buf[3] != 1 || buf[4] != 0 || buf[5] != 0) {
		return -1;
	}

	char* report = new(std::nothrow) char[8];
	if (report == nullptr)
	{
	    return -1;
	}
	memset(report, 0, 8);

	unsigned int size{ buf[6] < 8 ? buf[6] : 8 };
	ep0->queue_data(report, size);
	return true;
}

int usb_get_idle(USBControlEndpoint* ep0, char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	if (buf[2] != 0 || buf[3] != 0) {
		return -1;
	}

	char* idle = new(std::nothrow) char[1];
	if (idle == nullptr)
	{
	    return -1;
	}
	*idle = usb_status->idle;

	ep0->queue_data(idle, 1);
	return true;
}

int usb_get_protocol(USBControlEndpoint* ep0, char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	if (buf[2] != 0 || buf[3] != 0 || buf[4] != 0 || buf[5] != 0 || buf[6] != 1 || buf[7] != 0) {
		return -1;
	}

	char* protocol = new(std::nothrow) char[1];
	if (protocol == nullptr)
	{
	    return -1;
	}
	*protocol = usb_status->protocol;

	ep0->queue_data(protocol, 1);
	return true;
}

int usb_set_report(USBControlEndpoint* ep0, char* buf)
{
	return true;
}

int usb_set_idle(USBControlEndpoint* ep0, char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	if (buf[2] != 0) {
		return -1;
	}

	usb_status->idle = (unsigned char)(buf[3]);
	return true;
}

int usb_set_protocol(USBControlEndpoint* ep0, char* buf)
{
	volatile struct usb_status_info* usb_status{ getUSBStatusInfo() };

	if (buf[4] != 0 || buf[5] != 0 || buf[6] != 0 || buf[7] != 0 || buf[3] != 0) {
		return -1;
	}

	usb_status->protocol = buf[2];
	return true;
}

int usb_hid_class_request_handler(USBControlEndpoint* ep0, char* buf)
{
	switch (buf[1]) {
	case USBHID_GET_REPORT:
		usb_get_report(ep0, buf);
		return true;
	case USBHID_GET_IDLE:
		usb_get_idle(ep0, buf);
		return true;
	case USBHID_GET_PROTOCOL:
		usb_get_protocol(ep0, buf);
		return true;
	case USBHID_SET_REPORT:
		usb_set_report(ep0, buf);
		return true;
	case USBHID_SET_IDLE:
		usb_set_idle(ep0, buf);
		return true;
	case USBHID_SET_PROTOCOL:
		usb_set_protocol(ep0, buf);
		return true;
	default:
		return false;
	}
}

