#ifndef _REGISTERS_HH
#define _REGISTERS_HH

#ifdef __cplusplus
template<typename T> class Register
{
private:
	volatile T* reg;
public:
	Register(unsigned int address)
		: reg{ reinterpret_cast<volatile T*>(address) } {}
	Register(const Register&) = delete;
	Register(Register&&) = delete;

	Register& operator=(T value)
	{
		*reg = value;
		return *this;
	}
	Register& operator|=(T value)
	{
		*reg |= value;
		return *this;
	}
	Register& operator&=(T value)
	{
		*reg &= value;
		return *this;
	}

	operator T()
	{
		return *reg;
	}
};

typedef Register<unsigned char> Reg8;
typedef Register<unsigned short> Reg16;
typedef Register<unsigned int> Reg32;
#endif  // __cplusplus

#endif  // _REGISTERS_HH
