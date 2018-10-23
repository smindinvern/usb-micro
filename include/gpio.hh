#ifndef _GPIO_HH
#define _GPIO_HH

#include "macros.hh"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_FUNC(x) \
	xglue(CHIP_FAMILY, x)
	
	void GPIO_FUNC(_configure_pin_as_gpio)(unsigned char pin);
	void GPIO_FUNC(_configure_pin_direction)(unsigned char pin,
											 bool output);
	void GPIO_FUNC(_set_pin_value)(unsigned char pin,
								   bool value);
	bool GPIO_FUNC(_get_pin_state)(unsigned char pin);
	void GPIO_FUNC(_tristate_pin)(unsigned char pin);
	void GPIO_FUNC(_enable_pin_pullup)(unsigned char pin);
	void GPIO_FUNC(_enable_pin_pulldown)(unsigned char pin);
	void GPIO_FUNC(_disable_pin_pull)(unsigned char pin);

#define configure_pin_as_gpio(pin) \
	GPIO_FUNC(_configure_pin_as_gpio)(pin)
#define configure_pin_direction(pin, output) \
	GPIO_FUNC(_configure_pin_direction)(pin, output)
#define set_pin_value(pin, value) \
	GPIO_FUNC(_set_pin_value)(pin, value)
#define get_pin_state(pin) \
	GPIO_FUNC(_get_pin_state)(pin)
#define tristate_pin(pin) \
	GPIO_FUNC(_tristate_pin)(pin)
#define enable_pin_pullup(pin) \
	GPIO_FUNC(_enable_pin_pullup)(pin)
#define enable_pin_pulldown(pin) \
	GPIO_FUNC(_enable_pin_pulldown)(pin)
#define disable_pin_pull(pin) \
	GPIO_FUNC(_disable_pin_pullup)(pin)
	
#ifdef __cplusplus
}
#endif

#endif  // _GPIO_HH
