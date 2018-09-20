#ifndef _STRINGS_HH
#define _STRINGS_HH

#ifdef __cplusplus
extern "C" {
#define _restrict 
#else
#define _restrict restrict
#endif
	typedef unsigned int size_t;
#ifndef __cplusplus
	typedef unsigned short wchar_t;
#endif

#define NULL ((void *)0)

	size_t strlen(const char* s);
	size_t wcslen(const wchar_t* s);
	char* strcpy(char* _restrict s1,
				 const char* _restrict s2);

	// Non-standard
	size_t stowcs(wchar_t* _restrict pwcs,
				  const char* _restrict s,
				  size_t n);
#ifdef __cplusplus
}
#endif


#endif
