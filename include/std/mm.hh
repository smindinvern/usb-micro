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

inline int log2(unsigned int n)
{
	if (n == 0) {
		return -1;
	}

	for (int e = 0; ; e++) {
		n /= 2;
		if (n == 0) {
			return e;
		}
	}
}

#define PAGE_SIZE 64
#define NUM_PAGES 2048
#define HEAP_LEVELS 12

typedef unsigned int size_t;

extern "C" {
	void* memcpy(void* dest, const void* src, unsigned int length);
	void memset(char* buf, int val, unsigned int length);
	void* realloc(void* orig, unsigned int nu_size);
	void do_isb();
	void do_dmb();
}

class MemoryManager
{
private:
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
		//		base_address = (reinterpret_cast<unsigned int>(this) & ~0b11) + 4;
		/* mark the first (NUM_PAGES/PAGE_SIZE) page as allocated
		   to account for the memory we take up */
		allocate_memory(sizeof(heap));
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
