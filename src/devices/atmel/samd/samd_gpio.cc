#include "atsamd21.hh"
#include "gpio.hh"

extern "C" {
	void SAMD21_configure_pin_as_gpio(unsigned char pin)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg8 pincfg{ PORT_PINCFG(group, pin_no) };
		// PINCFG.PMUXEN = 0
		pincfg &= ~0x1;
	}
	void SAMD21_configure_pin_direction(unsigned char pin,
										bool output)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 dir{ output ? PORT_DIRSET(group) : PORT_DIRCLR(group) };
		dir = (1 << pin_no);
	}
	void SAMD21_set_pin_value(unsigned char pin,
							  bool value)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 val{ value ? PORT_OUTSET(group) : PORT_OUTCLR(group) };
		val = (1 << pin_no);
	}
	void set_pull_resistor_state(unsigned char pin,
								 bool enabled)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg8 pincfg{ PORT_PINCFG(group, pin_no) };
		pincfg = (pincfg & (1 << 2)) | (enabled << 2);
	}
	/**
	 * When the pin is configured as an input, PINCFG.PULLEN=1 connects the pin
	 * to an internal pull-up/down resistor.  The value of the OUT register
	 * determines the direction that the pin is pulled.  A 1 means it is
	 * connected to a pull-up resistor, a 0 means it is connected to a pull-down
	 * resistor.  Setting PINCFG.PULLEN=0 while the pin is configured as an
	 * input leaves the pin in Hi-Z.
	 */
	void SAMD21_tristate_pin(unsigned char pin)
	{
		// If the pin is configured as an output, this will be a NOP.
		set_pull_resistor_state(pin, false);
		// If the pin is already configured as an input, this will be a NOP.
		SAMD21_configure_pin_direction(pin, false);
	}
	void enable_pin_pull(unsigned char pin,
								bool dir)
	{
		// Let the pin float briefly as we connect the pull-up.
		SAMD21_tristate_pin(pin);
		SAMD21_set_pin_value(pin, dir);
		set_pull_resistor_state(pin, true);
	}
	void SAMD21_enable_pin_pullup(unsigned char pin)
	{
		enable_pin_pull(pin, true);
	}
	void SAMD21_enable_pin_pulldown(unsigned char pin)
	{
		enable_pin_pull(pin, false);
	}
	void SAMD21_disable_pin_pull(unsigned char pin)
	{
		set_pull_resistor_state(pin, false);
	}
}
