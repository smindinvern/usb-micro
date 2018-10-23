#include "gpio.hh"
#include "Registers.hh"
#include "sam4s.hh"

extern "C" {
	void SAM4S_configure_pin_as_gpio(unsigned char pin)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 pio_per{ PIO_PER(group) };
		pio_per = 1 << pin_no;
	}
	void SAM4S_configure_pin_direction(unsigned char pin,
									   bool output)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 pio_oxr{ output ? PIO_OER(group) : PIO_ODR(group) };
		pio_oxr = 1 << pin_no;
	}
	void SAM4S_set_pin_value(unsigned char pin,
							 bool value)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 pio_xodr{ value ? PIO_SODR(group) : PIO_CODR(group) };
		pio_xodr = 1 << pin_no;
	}
	bool SAM4S_get_pin_state(unsigned char pin)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 pio_pdsr{ PIO_PDSR(group) };
		return pio_pdsr & (1 << pin_no);
	}
	void SAM4S_tristate_pin(unsigned char pin)
	{
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 pio_pudr{ PIO_PUDR(group) };
		Reg32 pio_ppddr{ PIO_PPDDR(group) };
		pio_pudr = 1 << pin_no;
		pio_ppddr = 1 << pin_no;
		SAM4S_configure_pin_direction(pin, false);
	}
	void SAM4S_enable_pin_pullup(unsigned char pin)
	{
		SAM4S_tristate_pin(pin);
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 pio_puer{ PIO_PUER(group) };
		pio_puer = 1 << pin_no;
	}
	void SAM4S_enable_pin_pulldown(unsigned char pin)
	{
		SAM4S_tristate_pin(pin);
		unsigned char group{ PIN_GROUP(pin) };
		unsigned char pin_no{ PIN_N(pin) };
		Reg32 pio_ppder{ PIO_PPDER(group) };
		pio_ppder = 1 << pin_no;
	}
	
}
