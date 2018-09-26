#ifndef _GPIO_HH
#define _GPIO_HH

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_FUNC(x) \
	xglue(CHIP_FAMILY, x)
	
	void GPIO_FUNC(_configure_pin_as_gpio)(unsigned char pin_no);
	void GPIO_FUNC(_configure_pin_direction)(unsigned char pin_no,
											 bool output);
	enum PinStates {
		PinHighState,
		PinLowState,
		PinTriState
	};
	void GPIO_FUNC(_set_pin_state)(unsigned char pin_no,
								   PinStates state);
	void GPIO_FUNC(_enable_pin_pullup)(unsigned char pin_no);
	void GPIO_FUNC(_disable_pin_pullup)(unsigned char pin_no);
	void GPIO_FUNC(_enable_pin_pulldown)(unsigned char pin_no);
	void GPIO_FUNC(_disable_pin_pulldown)(unsigned char pin_no);

#define configure_pin_as_gpio(pin_no) \
	GPIO_FUNC(_configure_pin_as_gpio)(pin_no)
#define configure_pin_direction(pin_no, output) \
	GPIO_FUNC(_configure_pin_direction)(pin_no, output)
#define set_pin_state(pin_no, state) \
	GPIO_FUNC(_set_pin_state)(pin_no, state)
#define enable_pin_pullup(pin_no) \
	GPIO_FUNC(_enable_pin_pullup)(pin_no)
#define disable_pin_pullup(pin_no) \
	GPIO_FUNC(_disable_pin_pullup)(pin_no)
#define enable_pin_pulldown(pin_no) \
	GPIO_FUNC(_enable_pin_pulldown)(pin_no)
#define disable_pin_pulldown(pin_no) \
	GPIO_FUNC(_disable_pin_pulldown)(pin_no)
	
#ifdef __cplusplus
}
#endif

#endif  // _GPIO_HH
