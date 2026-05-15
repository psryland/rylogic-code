//*********************************************
// Renderer
//  Copyright (c) Rylogic Ltd 2012
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr::compute
{
	// A generator for an unordered map with a custom allocator
	template <typename Key, typename Value>
	struct LookupGenerator
	{
		// Use:
		//   typedef Lookup<int, char>::type CharLookup;
		using pair   = std::pair<const Key, Value>;
		using hasher = std::hash<Key>;
		using keyeq  = std::equal_to<Key>;
		using alloc  = Allocator<pair>;

		using type = std::unordered_map<Key, Value, hasher, keyeq, alloc>;
	};

	// An unordered map with a custom allocator
	template <typename Key, typename Value>
	struct Lookup :LookupGenerator<Key,Value>::type
	{
		using generator = LookupGenerator<Key, Value>;
		using base = typename generator::type;
		using pair = typename generator::pair;

		Lookup()
			:base(8, generator::hasher(), generator::keyeq(), generator::alloc())
		{}
	};
}


