#include <cstdio>
#include <cstring>
#include <list>

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
	return (1 << n);
}

unsigned int
section_level(unsigned int section)
{
	return log2(section) + 1;
}

unsigned int
section_offset_on_level(unsigned int section)
{
	return section - exp2(section_level(section) - 1);
}

unsigned int allocation_size(unsigned int section,
							 unsigned int section_size,
							 unsigned int num_sections)
{
	return (section_size * num_sections) / exp2(section_level(section) - 1);
}

unsigned int address_from_section(unsigned int base,
								  unsigned int section,
								  unsigned int section_size,
								  unsigned int num_sections)
{
	unsigned int this_level_offset{ section_offset_on_level(section) };
	return base + (this_level_offset * allocation_size(section, section_size, num_sections));
}

std::list<unsigned int>
get_allocated_sections(const char* buffer, size_t size)
{
	std::list<unsigned int> allocations{};
	for (size_t i = 0; i < size; i++) {
		if (buffer[i] == 0x02) {
			allocations.push_back(i);
		}
	}
	return allocations;
}

int main(int argc, const char** argv)
{
	if (argc != 2) {
		printf("invocation error\n");
		return -1;
	}
	const char* f_name{ argv[1] };
	FILE* fp{ fopen(f_name, "r") };
	fseek(fp, 0, SEEK_END);
	long int len{ ftell(fp) };
	fseek(fp, 0, SEEK_SET);
	char* buf = new char[len];
	if (fread(buf, 1, len, fp) != (unsigned long int)len) {
		printf("error reading file\n");
		return -1;
	}
	// Allocations are marked with 0x02.  Used sections are denoted with 0x01.
	// A section is used if it is part of a larger allocation, or if it has
	// sub-sections that are allocated.

	unsigned int n_pages = len / 2;
	#define PAGE_SIZE 64
	#define BASE_ADDRESS 0x20001000
	std::list<unsigned int> allocs{ get_allocated_sections(buf, len) };
	for (const auto& i : allocs) {
		unsigned int address{ address_from_section(BASE_ADDRESS, i, PAGE_SIZE, n_pages) };
		unsigned int alloc_size{ allocation_size(i, PAGE_SIZE, n_pages) };
		printf("addr = 0x%.8x, size = 0x%.8x\n", address, alloc_size);
	}
	return 0;
}
