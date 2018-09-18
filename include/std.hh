#ifndef _STD_HH
#define _STD_HH

// This is only relevant for C++.
#ifdef __cplusplus
namespace std {
	template<class> struct remove_reference;
	template<class T> struct remove_reference
	{
		typedef T type;
	};
	template<class T> struct remove_reference<T&>
	{
		typedef T type;
	};
	template<class T> struct remove_reference<T&&>
	{
		typedef T type;
	};

	template<class T> using remove_reference_t = typename remove_reference<T>::type;

	// decltype(t) is either T& or T&&, so it never invokes a constructor
	// T&& because T& can't bind to pr-values
	template<class T> constexpr remove_reference_t<T>&& move(T&& t) noexcept
	{
		return static_cast<remove_reference_t<T>&&>(t);
	}
}

#endif  // defined(__cplusplus)

#endif  // !defined(_STD_HH)
