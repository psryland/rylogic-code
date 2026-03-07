//******************************************
// Aligned Type
//  Copyright (C) Rylogic Ltd 2010
//******************************************

#pragma once

// C++11's alignas
#if _MSC_VER < 1900
#  ifndef alignas
#    define alignas(alignment) __declspec(align(alignment))
#  endif
#endif

namespace pr
{
	namespace meta
	{
		// Note: if you get an error saying: "'type' : is not a member of 'meta::aligned_type<Alignment>'"
		// Then it's probably because 'Alignment' is not one of the following specialisations
		template <std::size_t Alignment> struct aligned_type {};

		template <> struct aligned_type<1>  { struct type { char   a; }; };
		template <> struct aligned_type<2>  { struct type { short  a; }; };
		template <> struct aligned_type<4>  { struct type { int    a; }; };
		template <> struct aligned_type<8>  { struct type { double a; }; };
		template <> struct aligned_type<16> { struct alignas(16) type { char a[16]; }; };
	}
}