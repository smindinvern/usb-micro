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

#include "mm.hh"

unsigned int exp2(unsigned int n)
{
	// undefined behavior if n == 0?
	return (1 << n);
}

unsigned int page_level(unsigned int page)
{
	return log2(page) + 1;
}
unsigned int page_offset_on_level(unsigned int page)
{
	return page - exp2(page_level(page) - 1);
}
unsigned int allocation_size(unsigned int page)
{
	return (PAGE_SIZE * NUM_PAGES) / exp2(page_level(page)-1);
}

void* MemoryManager::address_from_page(unsigned int page)
{
#if 0
	// page is the 1-indexed number of the node in the tree
	// L, therefore, is the 0-indexed level of the tree from the top down
	int L{ log2(page) };
	if (L < 0) {
		return nullptr;
	}
	// `this_size' is the size of the allocation
	unsigned int this_size{ PAGE_SIZE * exp2(HEAP_LEVELS - L - 1) };
	unsigned int O{ page - exp2(L) };
	return reinterpret_cast<void*>(reinterpret_cast<char*>(this) + (O * this_size));
#endif
	char* base{ reinterpret_cast<char*>(this) };
	unsigned int this_level_offset{ page_offset_on_level(page) };

	return reinterpret_cast<void*>(base + (this_level_offset * allocation_size(page)));
}

unsigned int MemoryManager::page_from_address(void* address)
{
	unsigned int offset{ reinterpret_cast<unsigned int>(address) - reinterpret_cast<unsigned int>(this) };
	offset /= PAGE_SIZE;
	/*                        total nodes      -  offset from end of last level */
	for (unsigned int page = NUM_PAGES + offset; page > 0; page /= 2) {
		if (heap[page - 1] == 2) {
			/* this was the allocation */
			return page;
		}
	}
	return 0;
}

void* MemoryManager::allocate_memory(unsigned int size)
{
	if (!size) {
		return nullptr;
	}
	unsigned int num_pages_needed{ (size / PAGE_SIZE) };
	if ((size % PAGE_SIZE) != 0) {
		num_pages_needed += 1;
	}
	if (num_pages_needed > NUM_PAGES) {
		// not enough memory to begin with
		return nullptr;
	}

	// calculate which level of the heap corresponds to
	// this size of block
	int heap_level{ log2(num_pages_needed) };
	if (heap_level < 0) {
		return nullptr;
	}
	else if (num_pages_needed > exp2(heap_level)) {
		heap_level += 1;
	}
	if (heap_level >= HEAP_LEVELS) {
		return nullptr;
	}
	// `heap_level' is the number of levels up from the _bottom_ level
	// convert it to be zero-indexed at the top
	heap_level = HEAP_LEVELS - heap_level - 1;

	// heap_level is in the range [0, HEAP_LEVELS)
	unsigned int free_page{ find_free_page(heap_level) };
	if (!free_page) {
		return nullptr;
	}
	
	// otherwise allocate the page
	mark_page(free_page, 1);

	// translate page number to address
	void* address{ address_from_page(free_page) };
	if (!address) {
		// mark page as free and return
		mark_page(free_page, 0);
		return nullptr;
	}

	return address;
}

void MemoryManager::free_memory(void* address)
{
	unsigned int page{ page_from_address(address) };
	if (!page) {
		// error condition
		return;
	}
	mark_page(page, 0);
}

// level is zero-indexed
unsigned int MemoryManager::find_free_page(unsigned int level)
{
	// level must be in the range [0, HEAP_LEVELS)
	if (level >= HEAP_LEVELS) {
		return 0;
	}
	// find the start of the `level'th row of the tree
	// node is 1-indexed, level is 0-indexed
	unsigned int node{ exp2(level) };

	// scan the row for empty slots
	for (unsigned int i = node; i < 2 * node; i++) {
		if (heap[i - 1] == 0) {
			// this slot is free, so return it
			return i;
		}
	}

	return 0;
}

void MemoryManager::mark_children(unsigned int page, char allocated)
{
	if (page > (NUM_PAGES * 2)) {
		return;
	}
	
	heap[page - 1] = allocated;

	// mark all children
	mark_children(page * 2, allocated);
	mark_children(page * 2 + 1, allocated);
}

void MemoryManager::mark_page(unsigned int page, char allocated)
{
	mark_children(page, allocated);
	if (allocated) {
		heap[page - 1] = 2;
		page /= 2;
	}
	// now mark ancestors
	while (page > 0) {
		heap[page - 1] = allocated;
		if (!allocated) {
			// if marking nodes freed, make sure this node's sibling is also free
			// before moving onto the parent
			unsigned int sibling{ page ^ 1 };
			if (heap[sibling - 1]) {
				// sibling is still allocated, so leave all ancestors allocated
				break;
			}
		}
		page /= 2;
	}
}

void* MemoryManager::resize(void* orig, unsigned int nu_size)
{
	unsigned int orig_size = allocation_size(page_from_address(orig));
	void* nu = allocate_memory(nu_size);
	if (!nu) {
		return nullptr;
	}
	memcpy(nu, orig, orig_size);
	free_memory(orig);
	return nu;
}

MemoryManager* init_mm()
{
	MemoryManager* mm{ new(reinterpret_cast<void*>(0x20001000)) MemoryManager() };
	return mm;
}

MemoryManager* getMemoryManager()
{
	return reinterpret_cast<MemoryManager*>(0x20001000);
}

extern "C" {
	void* malloc(unsigned int size)
	{
		MemoryManager* mm{ getMemoryManager() };
		if (!mm) {
			return nullptr;
		}
		return mm->allocate_memory(size);
	}

	void free(void* address)
	{
		MemoryManager* mm{ getMemoryManager() };
		if (!mm) {
			return;
		}
		return mm->free_memory(address);
	}

	/* provide some usefule <cstring> memory functions */

	void* memcpy(void* dest, const void* src, unsigned int length)
	{
		for (unsigned int i = 0; i < length; i++) {
			((char*)dest)[i] = ((char*)src)[i];
		}
		return dest;
	}

	void memset(char* buf, int val, unsigned int length)
	{
		for (unsigned int i = 0; i < length; i++) {
			buf[i] = val;
		}
	}

	void* realloc(void* orig, unsigned int nu_size)
	{
		MemoryManager* mm{ getMemoryManager() };
		if (!mm) {
			return nullptr;
		}
		return mm->resize(orig, nu_size);
	}
}

void* operator new(size_t size, const std::nothrow_t&) noexcept
{
	return malloc(size);
}

void operator delete(void* address) noexcept
{
	free(address);
}

void operator delete(void* address, size_t) noexcept
{
	free(address);
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept
{
	return malloc(size);
}

void operator delete[](void* address) noexcept
{
	free(address);
}

void operator delete[](void* address, size_t) noexcept
{
	free(address);
}
