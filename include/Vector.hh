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

#ifndef VECTOR_HH_
#define VECTOR_HH_

#include "std.hh"
#include "mm.hh"

template<class T> class Vector
{
	T* array{};
	unsigned int n_elements{};
public:
	void push_back(T new_element)
	{
		T* new_array{ new(std::nothrow) T[n_elements+1] };
		if (!new_array) {
			return;
		}
		for (unsigned int i = 0; i < n_elements; i++) {
			new_array[i] = std::move(array[i]);
		}
		delete[] array;
		new_array[n_elements] = std::move(new_element);
		array = new_array;
		n_elements++;
	}
	T& operator[](unsigned int i)
	{
		return array[i];
	}
	unsigned int size()
	{
		return n_elements;
	}
	bool empty()
	{
		return !size();
	}

	Vector() = default;
	Vector(unsigned int elements)
		: array{ (elements>0) ? new(std::nothrow) T[elements] : nullptr },
		  n_elements{ elements }
	{
		if (!array) {
			n_elements = 0;
		}
	}
	Vector(const Vector<T>& other)
		: n_elements{ other.n_elements }
	{
		if (!n_elements) {
			return;
		}

		array = new(std::nothrow) T[n_elements];
		if (!array) {
			n_elements = 0;
			return;
		}

		for (unsigned int i = 0; i < n_elements; i++) {
			array[i] = other.array[i];
		}
	}
	Vector<T>& operator=(const Vector<T>& rhs)
	{
		delete[] array;
		n_elements = 0;
		// make a local copy
		Vector<T> temp{ rhs };
		// now move to *this
		*this = std::move(temp);
		return *this;
	}
	Vector(Vector<T>&& other)
		: array{ other.array },
		  n_elements{ other.n_elements }
	{
		other.array = nullptr;
		other.n_elements = 0;
	}
	Vector<T>& operator=(Vector<T>&& rhs)
	{
		delete[] array;
		array = rhs.array;
		n_elements = rhs.n_elements;
		rhs.array = nullptr;
		rhs.n_elements = 0;
		return *this;
	}
	~Vector()
	{
		delete[] array;
	}

};

#endif  // VECTOR_HH_
