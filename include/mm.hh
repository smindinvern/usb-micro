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

#ifndef MM_HH_
#define MM_HH_

#include "String.h"

constexpr int log2(const unsigned int n)
{
	if (n == 0) {
		return -1;
	}
	else {
		return log2(n / 2) + 1;
	}
}

constexpr unsigned int exp2(const unsigned int n)
{
	// undefined behavior if n == 0?
	return (1 << n);
}

static const size_t PAGE_SIZE = 64;
static const size_t NUM_PAGES = exp2(log2((size_t )((RAM_SIZE) / (PAGE_SIZE))));
static const size_t HEAP_LEVELS = (log2(NUM_PAGES) + 1);

extern "C" {
	void* memcpy(void* dest, const void* src, size_t count);
	void* memset(void* buf, int val, size_t count);
	void* realloc(void* orig, unsigned int nu_size);
	void do_isb();
	void do_dmb();
}

class MemoryManager
{
private:
	volatile char stack[STACK_SIZE];
	static constexpr unsigned int node_count{ 2*NUM_PAGES - 1 };
	char heap[node_count] {};
	unsigned int find_free_page(unsigned int);
	void mark_children(unsigned int, char allocated);
	void mark_page(unsigned int, char allocated);
	void* address_from_page(unsigned int);
	unsigned int page_from_address(void*);
public:
	MemoryManager()
	{
		// Erase RAM to ensure deterministic behavior
		// across boots.
		memset(heap, 0, RAM_SIZE - STACK_SIZE);
		/* mark the first few pages as allocated
		   to account for the memory we take up */
		allocate_memory(sizeof(*this));
	}

	void* allocate_memory(unsigned int size);
	void free_memory(void* address);
	void* resize(void* orig, unsigned int nu_size);
	
	void* operator new(size_t, void* address)
	{
		// just `place' ourselves wherever requested
		return address;
	}
};

MemoryManager* init_mm();

namespace std
{
	struct nothrow_t { constexpr nothrow_t() {} };
	const struct nothrow_t nothrow;
}

void* operator new(size_t, const std::nothrow_t&) noexcept;
void* operator new[](size_t, const std::nothrow_t&) noexcept;
void operator delete(void*) noexcept;
void operator delete(void* addr, size_t) noexcept;
void operator delete[](void*) noexcept;
void operator delete[](void*, size_t) noexcept;


#endif  // MM_HH_
