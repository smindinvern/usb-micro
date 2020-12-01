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
#include "primitives.hh"

// This is equivalent to floor(log2(val))
int log2(unsigned int val)
{
    return 31 - clz(val);
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

namespace MemoryMarks
{
    enum
    {
	Free = 0b00,
	PartiallyAllocated = 0b01,
	FullyAllocated = 0b10,
	DirectlyAllocated = 0b11
    };
    bool is_fully_allocated(char mark)
    {
	return (mark & 0b10) != 0;
    }
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
	    if (heap[page - 1] == MemoryMarks::DirectlyAllocated) {
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
	mark_page(free_page, true);

	// translate page number to address
	return address_from_page(free_page);
}

void MemoryManager::free_memory(void* address)
{
	unsigned int page{ page_from_address(address) };
	if (!page) {
		// error condition
		return;
	}
	mark_page(page, false);
}

unsigned int find_free_page(char* heap, unsigned int target_level, unsigned int current_level, unsigned int current_node)
{
    if (current_level > target_level)
    {
	return 0;
    }
    else if (heap[current_node - 1] == MemoryMarks::Free)
    {
	unsigned int remaining_levels{ target_level - current_level };
	if (remaining_levels > 0)
	{
	    current_node <<= remaining_levels;
	}
	return current_node;
    }
    else if (heap[current_node - 1] == MemoryMarks::PartiallyAllocated)
    {
	unsigned int next_node{ find_free_page(heap, target_level, current_level + 1, current_node * 2) };
	if (next_node == 0)
	{
	    next_node = find_free_page(heap, target_level, current_level + 1, (current_node * 2) + 1);
	}
	return next_node;
    }
    else
    {
	// Directly or fully allocated.
	return 0;
    }
}

// level is zero-indexed
unsigned int MemoryManager::find_free_page(unsigned int level)
{
	// level must be in the range [0, HEAP_LEVELS)
	if (level >= HEAP_LEVELS) {
		return 0;
	}
	// Start at root of tree and find a path to the `level'th level that is not fully allocated.
	return ::find_free_page(heap, level, 0, 1);
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

void MemoryManager::mark_page(unsigned int page, bool allocated)
{
    if (allocated) {
	mark_children(page, MemoryMarks::FullyAllocated);
	heap[page - 1] = MemoryMarks::DirectlyAllocated;
    }
    else {
	mark_children(page, MemoryMarks::Free);
	heap[page - 1] = MemoryMarks::Free;
    }
    // now mark ancestors
    while (page > 1) {
	char page_mark{ heap[page - 1] };
	char sibling_mark{ heap[(page ^ 1U) - 1] };
	page /= 2;
	if (MemoryMarks::is_fully_allocated(page_mark) && MemoryMarks::is_fully_allocated(sibling_mark))
	{
	    heap[page - 1] = MemoryMarks::FullyAllocated;
	}
	else if (page_mark == MemoryMarks::Free && sibling_mark == MemoryMarks::Free)
	{
	    heap[page - 1] = MemoryMarks::Free;
	}
	else
	{
	    heap[page - 1] = MemoryMarks::PartiallyAllocated;
	}
    }
}

void* MemoryManager::resize(void* orig, unsigned int nu_size)
{
	// If we're preempted our allocation could be stolen.
	unsigned int primask{ push_primask() };

	unsigned int page{ page_from_address(orig) };
	unsigned int orig_size = allocation_size(page);

	// Mark the page as free so that we have a chance to resize in-place.
	mark_page(page, false);
	void* nu = allocate_memory(nu_size);
	if (!nu) {
		// Restore original state.
		mark_page(page, true);
		pop_primask(primask);
		return nullptr;
	}
	// If the allocation was resized in-place then we don't need to copy anything.
	if (nu != orig) {
		unsigned int copy_size = (orig_size < nu_size) ? orig_size : nu_size;
		memcpy(nu, orig, copy_size);
	}
	pop_primask(primask);
	return nu;
}

MemoryManager* init_mm()
{
	MemoryManager* mm{ new(reinterpret_cast<void*>(0x20000000)) MemoryManager() };
	return mm;
}

MemoryManager* getMemoryManager()
{
	return reinterpret_cast<MemoryManager*>(0x20000000);
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

	void* memcpy(void* dest, const void* src, size_t count)
	{
		for (unsigned int i = 0; i < count; i++) {
			((char*)dest)[i] = ((char*)src)[i];
		}
		return dest;
	}

	void* memset(void* buf, int val, size_t count)
	{
	    char* buffer = reinterpret_cast<char*>(buf);
	    for (size_t i = 0; i < count; i++) {
		buffer[i] = val;
	    }
	    return buf;
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
