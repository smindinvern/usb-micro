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

#ifndef __cplusplus
typedef _Bool bool;
#endif

#ifdef __cplusplus
extern "C" {
#endif
	void wait(unsigned short ticks);
	void do_isb(void);
	void do_dmb(void);
#ifdef __cplusplus
}
#endif

// This is only relevant for C++.
#ifdef __cplusplus
namespace std {
	template<class> struct remove_reference;
	template<class T> struct remove_reference
	{
		typedef T type;
	};
	template<class T> struct remove_reference<T&>
	{
		typedef T type;
	};
	template<class T> struct remove_reference<T&&>
	{
		typedef T type;
	};

	template<class T> using remove_reference_t = typename remove_reference<T>::type;

	// decltype(t) is either T& or T&&, so it never invokes a constructor
	// T&& because T& can't bind to pr-values
	template<class T> constexpr remove_reference_t<T>&& move(T&& t) noexcept
	{
		return static_cast<remove_reference_t<T>&&>(t);
	}
}

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
#endif

#endif  // MAIN_HH_
