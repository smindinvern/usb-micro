#include "String.h"

size_t strlen(const char* s)
{
	size_t i = 0;
	while (*(s++) != '\0') {
		i++;
	}
	return i;
}

size_t wcslen(const wchar_t* s)
{
	size_t i = 0;
	while (*(s++) != 0x0000) {
		i++;
	}
	return i;
}

char* strcpy(char* restrict s1,
			 const char* restrict s2)
{
	char* ptr = s1;
	while (*s2 != '\0') {
		*(s1++) = *(s2++);
	}
	// Copy the terminating null byte.
	*s1 = *s2;
	return ptr;
}

wchar_t ctowc(const char c)
{
	// if char is a signed type and c < 0, then
	// ((wchar_t )c) & 0xff00 == 0xff00, which is not what we want.
	return ((wchar_t )c) & 0x00ff;
}

size_t stowcs(wchar_t* restrict pwcs,
			  const char* restrict s,
			  size_t n)
{
	size_t i = 0;
	while (i < n) {
		*(pwcs++) = ctowc(*s);
		if (*s == '\0') {
			// Return value does not include terminating null character.
			// i.e. stowcs(&wcs, &s, n) < n => stowcs(&wcs, &s, n) == wcslen(&wcs)
			return i;
		}
		else {
			s++;
			i++;
		}
	}
}
