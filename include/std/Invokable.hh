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

#ifndef INVOKABLE_HH_
#define INVOKABLE_HH_

#include "main.hh"
#include "mm.hh"

template<class> struct Invokable;

template<class R, class... ArgTypes> struct InvokeHelper {
	virtual R INVOKE(ArgTypes...) = 0;
	virtual ~InvokeHelper() = default;
};

template<class F, class R, class... ArgTypes> struct InvokableWrapper : public InvokeHelper<R, ArgTypes...>
{
	std::remove_reference_t<F> callable_object;
	InvokableWrapper(const F& object)
		: callable_object{ object } {}

	virtual R INVOKE(ArgTypes... args)
	{
		return callable_object(static_cast<ArgTypes>(args)...);
	}
};

template<class R, class... ArgTypes> struct Invokable<R(ArgTypes...)>
{
	InvokeHelper<R, ArgTypes...>* invoker{};

	template<class F> Invokable(const F& func)
		: invoker{ new(std::nothrow) InvokableWrapper<F, R, ArgTypes...>{ func } } {}
	template<class F> Invokable(F& func)
		: Invokable(static_cast<const F&>(func)) {}
	Invokable(R (*func)(ArgTypes...))
	        : invoker{ new(std::nothrow) InvokableWrapper<R (*)(ArgTypes...), R, ArgTypes...>{ func } } {}
	
	Invokable() = default;
	Invokable(Invokable&& other)
		: invoker{ other.invoker }
	{
		other.invoker = nullptr;
	}
	Invokable& operator=(Invokable&& rhs)
	{
		invoker = rhs.invoker;
		rhs.invoker = nullptr;
		return *this;
	}

	operator bool()
	{
		return (invoker != nullptr);
	}
	R operator()(ArgTypes... args) const
	{
		// TODO: if (!*this) { /* ... */ }
		return invoker->INVOKE(static_cast<ArgTypes>(args)...);
	}

	~Invokable()
	{
		delete invoker;
	}
};

#endif  // INVOKABLE_HH_
