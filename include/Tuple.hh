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

#ifndef TUPLE_HH_
#define TUPLE_HH_

#include "mm.hh"
#include "main.hh"


template<class T, class U, class... V> struct Tuple;
template<class T, class U> struct Tuple<T, U>;

template<int, class T, class... V> struct TupleType;

template<class T, class... V> struct TupleType<0, T, V...>
{
	typedef T type;
};

template<int I, class T, class... V> struct TupleType
{
	typedef typename TupleType<I-1, V...>::type type;
};

template<class T, class U> struct Tuple<T, U>
{
	T first;
	U second;

	static constexpr size_t get_size(...) { return sizeof(T); }
	template<class T_, class... U_> static constexpr size_t get_size(Tuple<T_, U_...>*) { return Tuple<T_, U_...>::size(); }

	static constexpr size_t size()
	{
		return get_size(static_cast<T*>(nullptr)) + sizeof(U);
	}

	Tuple(T value1, U value2)
		: first{ static_cast<T&&>(value1) },
		  second{ static_cast<U&&>(value2) } {}
	Tuple() = default;
	Tuple(const Tuple&) = default;
	Tuple(Tuple&&) = default;
	Tuple<T, U>& operator=(const Tuple<T, U>&) = default;
	Tuple<T, U>& operator=(Tuple<T, U>&&) = default;
};

template<class T, class U, class... V> struct Tuple
{
	T first;
	Tuple<U, V...> second;

	template<int I, class T_, class... V_> struct getter;
	template<class T_, class U_, class... V_> struct getter<0, T_, U_, V_...>
	{
		static T_& get(Tuple<T_, U_, V_...>& tt)
		{
			return tt.first;
		}
	};
	template<class T_> struct getter<0, T_>
	{
		static T_& get(T_& t)
		{
			return t;
		}
	};
	template<int I, class T_, class... V_> struct getter
	{
		static auto get(Tuple<T_, V_...>& tt)
			-> typename TupleType<I, T_, V_...>::type&
		{
			return getter<I-1, V_...>::get(tt.second);
		}
	};
	
	template<int I> auto get()
		-> typename TupleType<I, T, U, V...>::type&
	{
		return getter<I, T, U, V...>::get(*this);
	}

	static constexpr size_t get_size(...) { return sizeof(T); }
	template<class T_, class... U_> static constexpr size_t get_size(Tuple<T_, U_...>*) { return Tuple<T_, U_...>::size(); }

	static constexpr size_t size()
	{
		return get_size(static_cast<T*>(nullptr)) + Tuple<U, V...>::size();
	}
	
	Tuple(T value1, U value2, V... value3)
		: first{ static_cast<T&&>(value1) },
		  second{ static_cast<U&&>(value2), static_cast<V&&>(value3)... } {}
	Tuple() = default;
	Tuple(const Tuple&) = default;
	Tuple(Tuple&&) = default;
	Tuple<T, U, V...>& operator=(const Tuple<T, U, V...>&) = default;
	Tuple<T, U, V...>& operator=(Tuple<T, U, V...>&&) = default;
};

template<class T, class U, class... V> class Serializable : public Tuple<T, U, V...>
{
	template<class TT, class TU , class TV, class... TW> void copyElement(const Tuple<TT, TU, TV, TW...>& element, char* array, size_t len) const
	{
		size_t copySize{ (len > elementSize(element.first)) ? elementSize(element.first) : len };
		if (!copySize) {
			return;
		}
		elementCopy(element.first, array, copySize);
		copyElement<TU, TV, TW...>(element.second, array + copySize, len - copySize);
	}

	template<class TT, class TU> void copyElement(const Tuple<TT, TU>& element, char* array, size_t len) const
	{
		size_t copySize{ (len > elementSize(element.first)) ? elementSize(element.first) : len };
		if (!copySize) {
			return;
		}
		elementCopy(element.first, array, copySize);
		len -= copySize;
		array += copySize;
		copySize = (len > elementSize(element.second)) ? elementSize(element.second) : len;
		if (!copySize) {
			return;
		}
		elementCopy(element.second, array, copySize);
	}

	template<typename TT> size_t elementSize(TT&) const
	{
		return sizeof(TT);
	}
	template<typename TT, typename...TU> size_t elementSize(Serializable<TT, TU...>& e) const
	{
		return e.size();
	}
	
	template<typename TT> void elementCopy(TT& e, char* buffer, size_t length) const
	{
		memcpy(buffer, &e, length);
	}
	template<typename TT, typename...TU> void elementCopy(Serializable<TT, TU...>& e, char* buffer, size_t length) const
	{
		e.copyTo(buffer, length);
	}

public:
	using Tuple<T, U, V...>::Tuple;
	Serializable() = default;
	Serializable(const Tuple<T, U, V...>& other)
		: Tuple<T, U, V...>(other) {}
	Serializable(Tuple<T, U, V...>&& other)
		: Tuple<T, U, V...>(std::move(other)) {}

	char* serialize() const
	{
		char* buffer{ new(std::nothrow) char[Tuple<T, U, V...>::size()] };
		if (!buffer) {
			return nullptr;
		}
		copyTo(buffer, Tuple<T, U, V...>::size());
		return buffer;
	}
	void copyTo(char* buffer, size_t length) const
	{
		if (!buffer) {
			return;
		}
		copyElement(*this, buffer, length);
	}
};


template<class T, class U, class... V> class DeSerializable : public Serializable<T, U, V...>
{
	template<class TT> TT deser(char*& buffer, const TT&)
	{
		TT var;
		memcpy(&var, buffer, sizeof(TT));
		buffer += sizeof(TT);
		return var;
	}
	template<class TX, class TT, class TU, class... TV> DeSerializable<TT, TU, TV...> deser(char*& buffer, const DeSerializable<TT, TU, TV...>&)
	{
		DeSerializable<TT, TU, TV...> obj{ buffer };
		buffer += obj.size();
		return obj;
	}

public:
	DeSerializable() = default;
	/* constructs by deserializing */
	// TODO: this isn't right...
	DeSerializable(char* buffer)
		: Serializable<T, U, V...>{ deser<T>(buffer, T{}), deser<U>(buffer, U{}), deser<V>(buffer, V{})... } {}
	/* constructs from values, pass-through for Serializable */
	using Serializable<T, U, V...>::Serializable;
};

#endif  // TUPLE_HH_
