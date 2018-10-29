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

#ifndef POINTERS_HH_
#define POINTERS_HH_

#include "main.hh"

namespace std {
	// NB: This is NOT technically correct for memory blocks allocated
	// with `new[]'.
	template<typename T> struct exclusive_ptr
	{
	private:
		typedef std::remove_reference_t<T> * T_ptr;
		T_ptr obj;
	public:
		T_ptr get_ptr() { return obj; }
		T_ptr clear_ptr()
		{
			T_ptr temp = obj;
			obj = nullptr;
			return temp;
		}

		exclusive_ptr(T_ptr ptr)
			: obj{ ptr } {}
		exclusive_ptr(const exclusive_ptr<T>&) = delete;
		exclusive_ptr(exclusive_ptr<T>&& other)
		{
			this->obj = other.obj;
			other.obj = nullptr;
		}
		~exclusive_ptr()
		{
			delete obj;
		}
	};

	template<typename T> struct shared_ptr
	{
		typedef std::remove_reference_t<T> T_;
		typedef T_* T_ptr;
	private:
		struct shared_ptr_T_inner
		{
			T_ptr obj {};
			size_t ref_count {};
		} *inner {};
	public:
		operator bool()
		{
			return bool(inner) && bool(inner->obj);
		}
		shared_ptr() = default;
		shared_ptr& operator=(const shared_ptr& rhs)
		{
			this->~shared_ptr();
			inner = rhs.inner;
			if (inner) {
				inner->ref_count++;
			}
			return *this;
		}
		shared_ptr(const shared_ptr& rhs)
		{
			*this = rhs;
		}
		shared_ptr& operator=(shared_ptr&& rhs)
		{
			this->~shared_ptr();
			inner = rhs.inner;
			rhs.inner = nullptr;
			return *this;
		}
		shared_ptr(shared_ptr&& rhs)
		{
			*this = std::move(rhs);
		}
		// shared_ptr& operator=(T_ptr ptr)
		// {
		// 	if (!inner) {
		// 		inner = new(std::nothrow) shared_ptr_T_inner{ ptr, 1 };
		// 	}
		// 	else {
		// 		delete inner->obj;
		// 		inner->obj = ptr;
		// 	}
		// 	return *this;
		// }
		shared_ptr(T_ptr ptr)
			: inner{ new(std::nothrow) shared_ptr_T_inner{ ptr, 1 } } {}
		~shared_ptr()
		{
			if (inner) {
				if (--inner->ref_count == 0) {
					delete inner->obj;
					delete inner;
				}
			}
		}

		T_ptr get_ptr() { return inner->obj; }
		T_& operator*()
		{
			// NB: This will result in a null-pointer dereference if *this
			// was created with the default constructor.
			// This can also result in a null-pointer dereference if *this
			// was initialized with a null pointer.
			return *get_ptr();
		}
		T_& operator[](size_t i)
		{
			// NB: same caveat applies here as for operator*().
			return get_ptr()[i];
		}
	};

	template<typename T>
	shared_ptr<T> make_shared_ptr(T&& obj)
	{
		typename shared_ptr<T>::T_ptr ptr{ new T(std::move(obj)) };
		return new shared_ptr<T>(ptr);
	}
}

#endif
