//******************************************
// Suffix Array
//  Copyright (c) Rylogic Ltd 2024
//******************************************
// Explanation:
//   https://www.youtube.com/watch?v=Oj1wKc3CRL0&list=PL2mpR0RYFQsDFNyRsTNcWkFTHTkxWREeb
// Based on:
//   Nong G, Zhang S, Chan WH "Two efficient algorithms for linear time suffix array construction"
//   https://www.researchgate.net/publication/224176324_Two_Efficient_Algorithms_for_Linear_Time_Suffix_Array_Construction
#pragma once
#include <span>
#include <string>
#include <vector>
#include <concepts>
#include <algorithm>
#include <stdexcept>
#include <cassert>

namespace pr::suffix_array
{
	namespace impl
	{
		struct SuffixType
		{
			std::vector<bool> m_stypes;

			template <std::integral Int>
			SuffixType(std::span<Int const> data)
				:m_stypes(data.size())
			{
				// Classify the type of each character into S or L types. Suffixes that aren't S-type are L-type.
				// There is an implicit sentinal at the end of the string that is less than all characters in the alphabet
				m_stypes[std::ssize(data) - 1] = false; // The last character is always larger than the sentinal
				for (auto i = std::ssize(data) - 2; i >= 0; --i)
				{
					m_stypes[i] =
						(data[i] <  data[i + 1]) ||
						(data[i] == data[i + 1] && m_stypes[i + 1]);
				}
			}

			bool IsSType(int i) const
			{
				return i >= 0 && m_stypes[i];
			}
			bool IsLType(int i) const
			{
				return i >= 0 && !m_stypes[i];
			}

			// True if index 'i' is an "LMS" (i.e. leftmost S-type) character
			bool IsLeftmostSType(int i) const
			{
				return i > 0 && IsSType(i) && IsLType(i - 1);
			}
		};
		struct BucketIndexRanges
		{
			std::vector<int> beg;
			std::vector<int> end;
		};
		struct MatchResult
		{
			// The length of the match
			size_t length = 0;

			// The index range of the match in 'sa'
			size_t sa_beg = 0;
			size_t sa_end = 0;
		};

		// Return the bucket indices for the start/end of the character buckets.
		template <std::integral Int>
		BucketIndexRanges GetBuckets(std::span<Int const> data, int alphabet_size)
		{
			// Get the character frequencies in 'data'
			auto freq = std::vector<int>(alphabet_size);
			for (auto d : data)
				freq[d]++;

			auto bkt = BucketIndexRanges{ freq, freq };

			// Calculate the start/end indices for each bucket
			int sum = 0;
			auto beg = bkt.beg.begin();
			auto end = bkt.end.begin();
			for (auto& f : freq)
			{
				*beg++ = sum;
				sum += f;
				*end++ = sum;
			}

			return bkt;
		}

		// Sort L-type suffix indices by induction from left to right. 'bkt_beg' is the start index for each bucket
		template <std::integral Int>
		void InduceSortLtoR(SuffixType const& sfx_type, std::span<int> SA, std::span<Int const> data, std::span<int> bkt_beg)
		{
			// Normally, there is a sentinel that is the first value in 'SA'. This implementation uses an implicit sentinel.
			// The character before the sentinel is always L-Type.
			{
				auto j = static_cast<int>(std::ssize(data) - 1);
				SA[bkt_beg[data[j]]++] = j;
			}

			// Left to Right pass
			for (auto i = 0; i != std::ssize(data); ++i)
			{
				// Look at each 'known' index in 'SA'
				if (SA[i] == -1)
					continue;

				// Read the type of the immediately prior character. If the type is 'L-type' then write
				// its index into the associated bucket in 'SA', filling from the left.
				// The 'SA' is assumed to contain '-1' for unknown indices.
				auto j = SA[i] - 1;
				if (j >= 0 && sfx_type.IsLType(j))
					SA[bkt_beg[data[j]]++] = j;
			}
		}

		// Sort R-type suffix indices by induction from right to left. 'bkt_end' is the end index for each bucket
		template <std::integral Int>
		void InduceSortRtoL(SuffixType const& sfx_type, std::span<int> SA, std::span<Int const> data, std::span<int> bkt_end)
		{
			// Right to Left pass
			for (auto i = std::ssize(data) - 1; i >= 0; --i)
			{
				// Look at each 'known' index in 'SA'
				if (SA[i] == -1)
					continue;

				auto j = SA[i] - 1;
				if (j >= 0 && sfx_type.IsSType(j))
					SA[--bkt_end[data[j]]] = j;
			}
		}

		// Find the range of suffixes in 'sa' that match 'sub'.
		// If 'LCPOnly' is true, then the search exits as soon as any match is found.
		template <std::integral Int, bool LCPOnly>
		MatchResult Find(std::span<Int const> sub, std::span<Int const> data, std::span<int const> sa)
		{
			// An empty index has no suffixes to search, so the result is always an empty match range.
			// This keeps public callers from indexing into 'sa[0]' when the index was built from zero bytes.
			if (sa.empty())
				return {};

			// Binary search until 'mid' lands in the range of matches for 'sub'
			size_t lcp0 = 0, lcp1 = 0; // track the longest common prefix for the lower/upper bounds
			for (size_t low = 0, high = sa.size(); ; )
			{
				auto mid = (low + high) / 2;

				// Compare suffixes starting at 'i'. Assumes a[0..i) == b[0..i)
				auto Compare = [](std::span<Int const> a, std::span<Int const> b, size_t& i) -> int
				{
					for (;;++i)
					{
						if (i == a.size()) return static_cast<int>(b.size() - a.size());
						if (i == b.size()) return static_cast<int>(a.size() - b.size());
						if (a[i] < b[i]) return -1;
						if (a[i] > b[i]) return +1;
					}
				};

				// Find the longest common prefix at 'mid'.
				// We know the prefix matches up to 'match_length' so far.
				auto prefix = data.subspan(sa[mid]);
				auto match_length = std::min(lcp0, lcp1);
				auto sign = Compare(sub, prefix, match_length);

				// If 'mid' is within the range of matches, binary search to find the high/low bounds.
				if (match_length == sub.size())
				{
					if constexpr (LCPOnly)
					{
						return { match_length, mid, mid + 1 };
					}
					else
					{
						size_t lcp;

						// Binary search to the lower bound
						for (size_t hi = mid; low < hi;)
						{
							auto m = (low + hi) / 2;
							auto p = data.subspan(sa[m]);
							Compare(sub, p, lcp = lcp0);
							if (lcp == sub.size()) { hi = m; }
							else { low = m + 1; lcp0 = lcp; }
						}

						// Binary search to the upper bound
						for (size_t lo = mid; lo < high;)
						{
							auto m = (lo + high) / 2;
							auto p = data.subspan(sa[m]);
							Compare(p, sub, lcp = lcp1);
							if (lcp == sub.size()) { lo = m + 1; }
							else { high = m; lcp1 = lcp; }
						}

						return { match_length, low, high };
					}
				}

				// Otherwise, keep searching for any match
				if (sign > 0)
				{
					low = mid + 1;
					lcp0 = match_length;
				}
				else
				{
					high = mid;
					lcp1 = match_length;
				}

				// If the search range reaches zero, return the longest common prefix
				if (low == high)
				{
					return { match_length, low, high };
				}
			}
		}
	}

	using MatchResult = impl::MatchResult;

	// Construction ***********************************************************

	// Construct the suffix array of 'data' where each element is in the range [0, alphabet_size).
	template <std::integral Int>
	void Build(std::span<Int const> data, std::span<int> SA, int alphabet_size)
	{
		// NOTE: This implementation does not require a sentinel at the end of 'data'.
		// 
		// Basic notation commonly used in the presentations of this algorithm.
		// Let S be a string of n characters stored in an array[0..n − 1], and Σ(S) be the alphabet of S.
		// For a substring, S[i]S[i + 1]...S[j] in S, we denote it as S[i..j].
		// For presentation simplicity, S is supposed to be terminated by a sentinel $, which is the unique
		// lexicographically smallest character in S (using a sentinel is widely adopted in the literature for SACAs [5]).
		// Let suf(S, i) be the sufﬁx in S starting at S[i] and running to the sentinel.
		// A sufﬁx suf (S, i) is said to be S-type or L-type if suf (S, i) < suf(S, i + 1) or suf(S, i) > suf(S, i + 1), respectively.
		// The last sufﬁx suf(S, n − 1) consisting of only the single character $ (the sentinel) is deﬁned as S-type.
		// Correspondingly, we can classify a character S[i] to be S-type or L-type if suf(S, i) is S-type or L-type, respectively.
		// To store the type of every character/sufﬁx, we introduce an n-bit boolean array t, where t[i] records the type of character
		// S[i] as well as sufﬁx suf(S, i): 1 for S-type and 0 for L-type.
		// From the S-type and L-type deﬁnitions, we observe the following properties:
		//   1) S[i] is S-type if (i.1) S[i] < S[i + 1] or (i.2)S[i] = S[i + 1] and suf(S, i + 1) is S-type.
		//   2) S[i] isL-type if (ii.1) S[i] > S[i + 1] or (ii.2) S[i] = S[i + 1] andsuf(S, i + 1) is L-type.
		// These properties suggest that by scanning S once from right to left, we can determine the type of each
		// character/sufﬁx in O(1) time and ﬁll out the type array in O(n) time.
		//
		// So:
		//  S-type means "this suffix is smaller than the next adjacent suffix".
		//  L-type means "this suffix is larger than the next adjacent suffix".
		//
		// Uses a working space (excluding 'data' and 'SA') of at most 2.25n+O(1) for a constant alphabet.
		//
		// OK, WTF does this do?
		// 1. use a bit array to mark the relationship between adjacent suffixes as either S or L.
		// 2. Find the 'LMS' suffixes. These partition the suffixes into blocks of guaranteed increasing order.
		// 3. The R-to-L pass adds all LMS indices to the ends of their buckets. These are not yet sorted.
		// 4. The L-to-R pass adds L-type suffixes.
		//     The first L-type suffix is correctly sorted w.r.t. to the LMS suffixes already in SA.
		//     Each subsequent L-type suffix is then correctly sorted w.r.t. to the previous L-type suffixes,
		//     because if it wasn't, there would have been a contradiction in the S/L classification.
		// 5. The next R-to-L pass adds the S-type suffixes correctly sorted.
		
		using namespace impl;

		// Handle degenerate cases
		if (data.empty())
		{
			return;
		}
		if (data.size() > SA.size())
		{
			throw std::runtime_error("The output suffix array size must be >= input data size");
		}
		if (data.size() == 1)
		{
			SA[0] = 0;
			return;
		}
		if (data.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
		{
			throw std::runtime_error("The input data size must be <= INT_MAX");
		}

		// Ensure all characters are within the alphabet
		assert(std::all_of(data.begin(), data.end(), [alphabet_size](auto c) { return c >= 0 && c < alphabet_size; }));

		// Classify the type of each character into S or L types
		SuffixType sfx_type(data);

		// Initialize the suffix array to 'unknowns'
		std::fill(SA.begin(), SA.end(), -1);

		// Determine bucket index ranges
		auto const [bkt_beg, bkt_end] = GetBuckets<Int>(data, alphabet_size);
		std::vector<int> bkt(alphabet_size);

		// Stage 1: reduce the problem by at least 1/2. Sort all the S-suffixes
		{
			// Record the index of each "LMS" suffix in the bucket corresponding to its first character.
			// Fill the buckets from the right (that's why bucket end indices are used)
			bkt = bkt_end;
			for (int i = 1; i != std::ssize(data); ++i)
			{
				if (sfx_type.IsLeftmostSType(i))
					SA[--bkt[data[i]]] = i;
			}

			InduceSortLtoR(sfx_type, SA, data, bkt = bkt_beg);
			InduceSortRtoL(sfx_type, SA, data, bkt = bkt_end);
		}

		// Compact all the sorted LMS suffixes into the first 'n1' items of SA.
		// '2*n1' is not larger than 'size' (proveable)
		ptrdiff_t n1 = 0;
		for (int i = 0; i != std::ssize(data); ++i)
			if (sfx_type.IsLeftmostSType(SA[i]))
				SA[n1++] = SA[i];

		// Reset the unused space in 'SA'
		std::fill(SA.begin() + n1, SA.end(), -1);

		// Assign lexicographic names to the LMS suffixes.
		int name = 0, prev = -1;
		for (auto i = 0; i != n1; ++i)
		{
			auto pos = SA[i];
			for (int d = 0; d != std::ssize(data) - std::max(pos, prev); ++d)
			{
				// If this substring is different from the previous one, then it gets a new name.
				if (prev == -1 || data[pos + d] != data[prev + d] || sfx_type.IsSType(pos + d) != sfx_type.IsSType(prev + d))
				{
					name++;
					prev = pos;
					break;
				}
				if (d > 0 && (sfx_type.IsLeftmostSType(pos + d) || sfx_type.IsLeftmostSType(prev + d)))
				{
					break;
				}
			}

			pos = (pos % 2 == 0) ? pos / 2 : (pos - 1) / 2;
			SA[n1 + pos] = name - 1;
		}

		// Move the names to the end of 'SA'
		for (auto i = std::ssize(data) - 1, j = i; i >= n1; --i)
		{
			if (SA[i] >= 0)
				SA[j--] = SA[i];
		}

		// Stage 2: solve the reduced problem.
		auto data1 = SA.subspan(data.size() - n1, n1);
		auto SA1 = SA.subspan(0, n1);

		// Recurse if names are not yet unique.
		if (name < n1)
		{
			Build<int>(data1, SA1, name);
		}
		else
		{
			// Generate the suffix array of 'data1' directly
			for (int i = 0; i != n1; ++i)
				SA1[data1[i]] = i;
		}

		// Stage 3: induce the result for the original problem.
		{
			// Put all the LMS characters into their buckets
			bkt = bkt_end;
			for (int i = 1, j = 0; i < std::ssize(data); ++i)
				if (sfx_type.IsLeftmostSType(i))
					data1[j++] = i; // get p1

			// Get index in src
			for (int i = 0; i < n1; i++)
				SA1[i] = data1[SA1[i]];

			std::fill(SA.begin() + n1, SA.end(), -1);

			// Init SA[n1..n-1]
			for (auto i = n1 - 1; i >= 0; i--)
			{
				auto j = SA[i];
				SA[i] = -1;
				SA[--bkt[data[j]]] = j;
			}

			InduceSortLtoR(sfx_type, SA, data, bkt = bkt_beg);
			InduceSortRtoL(sfx_type, SA, data, bkt = bkt_end);
		}
	}

	// Construct the suffix array of the string 'str'
	inline void Build(std::string_view str, std::span<int> SA)
	{
		auto data = std::span{reinterpret_cast<unsigned char const*>(str.data()), str.size()};
		Build(data, SA, 256);
	}

	// Suffix Array Operations ************************************************

	// See if substring 'sub' occurs in 'data'
	template <std::integral Int>
	bool Contains(std::span<Int const> sub, std::span<Int const> data, std::span<int const> sa)
	{
		auto mr = impl::Find<Int, true>(sub, data, sa);
		return mr.length == sub.size();
	}

	// Count the occurrences of 'sub' in 'data'
	template <std::integral Int>
	size_t Count(std::span<Int const> sub, std::span<Int const> data, std::span<int const> sa)
	{
		// Find the node in the SA that matches 'sub'.
		auto mr = impl::Find<Int, false>(sub, data, sa);
		return mr.sa_end - mr.sa_beg;
	}
	
	// Return the locations of the occurrences of 'sub' in 'data.
	// Locations in 'data' are given by 'sa[MatchResult.sa_beg..MatchResult.sa_end)'
	template <std::integral Int>
	MatchResult Find(std::span<Int const> sub, std::span<Int const> data, std::span<int const> sa)
	{
		return impl::Find<Int, false>(sub, data, sa);
	}
}

#if PR_UNITTESTS
#include <random>
#include <fstream>
#include <filesystem>
#include "pr/common/unittests.h"
namespace pr::container
{
	PRUnitTestFamily(SuffixArrayTests, Slow)
	{
		{// Empty data and empty index
			std::string data;
			std::vector<int> sa;

			suffix_array::Build(data, sa);
			PR_EXPECT(sa.empty());

			auto empty_query = std::span<char const>{};
			auto mr = suffix_array::Find<char>(empty_query, data, sa);

			PR_EXPECT(mr.length == 0);
			PR_EXPECT(mr.sa_beg == 0);
			PR_EXPECT(mr.sa_end == 0);
			PR_EXPECT(suffix_array::Contains<char>(empty_query, data, sa));
			PR_EXPECT(!suffix_array::Contains<char>({ "x", 1 }, data, sa));
			PR_EXPECT(suffix_array::Count<char>(empty_query, data, sa) == 0);
			PR_EXPECT(suffix_array::Count<char>({ "x", 1 }, data, sa) == 0);
		}
		{// String data
			//                  0123456789ab
			std::string data = "mmiisiisiissiippiiii";
			std::vector<int> sa(std::size(data));

			for (auto& c : data) c -= 'a';
			suffix_array::Build<char>(data, sa, 'z' - 'a');
			for (auto& c : data) c += 'a';

			// Check that each substring is less than the next
			std::string_view sdata(data);
			for (auto i = 0; i != isize(sa) - 1; ++i)
			{
				auto a = sdata.substr(sa[i + 0]);
				auto b = sdata.substr(sa[i + 1]);
				PR_EXPECT(a < b);
			}

			PR_EXPECT(suffix_array::Contains<char>({ "m", 1 }, data, sa));
			PR_EXPECT(suffix_array::Contains<char>({ "i", 1 }, data, sa));
			PR_EXPECT(suffix_array::Contains<char>({ "iis", 3 }, data, sa));
			PR_EXPECT(!suffix_array::Contains<char>({ "isp", 3 }, data, sa));
			PR_EXPECT(suffix_array::Contains<char>({ "mmiisiisiissiippiiii", 20 }, data, sa));
			PR_EXPECT(!suffix_array::Contains<char>({ "iiiii", 5 }, data, sa));

			// An empty query is a prefix of every suffix in the index.
			PR_EXPECT(suffix_array::Contains<char>(std::span<char const>{}, data, sa));
			PR_EXPECT(suffix_array::Count<char>(std::span<char const>{}, data, sa) == data.size());
			{
				auto mr = suffix_array::Find<char>(std::span<char const>{}, data, sa);
				PR_EXPECT(mr.length == 0);
				PR_EXPECT(mr.sa_beg == 0);
				PR_EXPECT(mr.sa_end == sa.size());
			}

			PR_EXPECT(suffix_array::Count<char>({ "i", 1 }, data, sa) == 12);
			PR_EXPECT(suffix_array::Count<char>({ "ii", 2 }, data, sa) == 7);
			PR_EXPECT(suffix_array::Count<char>({ "iii", 3 }, data, sa) == 2);
			PR_EXPECT(suffix_array::Count<char>({ "iiii", 4 }, data, sa) == 1);
			PR_EXPECT(suffix_array::Count<char>({ "iiiii", 5 }, data, sa) == 0);
			PR_EXPECT(suffix_array::Count<char>({ "m", 1 }, data, sa) == 2);
			PR_EXPECT(suffix_array::Count<char>({ "isis", 4 }, data, sa) == 0);
			{
				auto mr = suffix_array::Find<char>({ "ii", 2 }, data, sa);
				for (auto i = 0; i != std::ssize(sa); ++i)
				{
					auto s = data.substr(sa[i]);
					auto is_ii = s.substr(0, 2) == "ii";
					auto should_be_ii = size_t(i) >= mr.sa_beg && size_t(i) < mr.sa_end;
					PR_EXPECT(is_ii == should_be_ii);
				}
			}
			{
				auto mr = suffix_array::Find<char>({ "isi", 3 }, data, sa);
				for (auto i = 0; i != std::ssize(sa); ++i)
				{
					auto s = data.substr(sa[i]);
					auto is_isi = s.substr(0, 3) == "isi";
					auto should_be_isi = size_t(i) >= mr.sa_beg && size_t(i) < mr.sa_end;
					PR_EXPECT(is_isi == should_be_isi);
				}
			}
		}
		{// Large random data
			std::uniform_int_distribution<int> dist(0, 'z' - 'a');
			std::default_random_engine rng(0);

			std::string data(1024, '\0');
			std::vector<int> sa(data.size());

			for (auto& c : data) c = static_cast<char>(dist(rng));
			suffix_array::Build<char>(data, sa, dist.max() + 1);
			for (auto& c : data) c += 'a';

			// Check that each substring is less than the next
			std::string_view sdata(data);
			for (auto i = 0; i != isize(sa) - 1; ++i)
			{
				auto a = sdata.substr(sa[i + 0]);
				auto b = sdata.substr(sa[i + 1]);
				PR_EXPECT(a < b);
			}
		}
		{// Limited alphabet data

			std::vector<unsigned char> data = {0,1,2,3,2,1,0,1,2,0,3,0,1,3,1,2,2,3,1,1,1,3,0,0,1,0};
			std::vector<int> sa(data.size());

			suffix_array::Build<unsigned char>(data, sa, 4);
			for (auto& c : data) c += 'a';


			// Check that each substring is less than the next
			auto sdata = std::string_view(reinterpret_cast<char const*>(data.data()), data.size());
			for (auto i = 0; i != isize(sa) - 1; ++i)
			{
				auto a = sdata.substr(sa[i]);
				auto b = sdata.substr(sa[i + 1]);
				PR_EXPECT(a < b);
			}
		}
		{// Highly repeditious data
			std::string data = "aabbaabbaabbbbaabbaabbaabbaa";
			std::vector<int> sa(data.size());
			
			for (auto& c : data) c -= 'a';
			suffix_array::Build<char>(data, sa, 2);
			for (auto& c : data) c += 'a';

			// Check that each substring is less than the next
			for (auto i = 0; i != isize(sa) - 1; ++i)
			{
				auto a = std::string_view(data.data() + sa[i]);
				auto b = std::string_view(data.data() + sa[i + 1]);
				PR_EXPECT(a < b);
			}
		}
		{// int data
			std::uniform_int_distribution<int> dist(0, 65535);
			std::default_random_engine rng(0);

			std::vector<int> data(23);
			std::vector<int> sa(data.size());
			for (auto& c : data) c = dist(rng);

			suffix_array::Build<int>(data, sa, dist.max() + 1);

			// Check that each substring is less than the next
			std::u32string_view sdata(reinterpret_cast<char32_t const*>(data.data()), data.size());
			for (auto i = 0; i != isize(sa) - 1; ++i)
			{
				auto a = sdata.substr(sa[i]);
				auto b = sdata.substr(sa[i + 1]);
				PR_EXPECT(a < b);
			}
		}
		{// use this file
			std::string data(s_cast<size_t>(std::filesystem::file_size(__FILE__)), '\0');
			{
				std::ifstream ifile(__FILE__, std::ios::in | std::ios::binary);
				ifile.read(data.data(), data.size());
			}

			std::vector<int> sa(data.size());
			suffix_array::Build(data, sa);

			// Check that each substring is less than the next
			std::string_view sdata(data);
			for (auto i = 0; i != isize(sa) - 1; ++i)
			{
				auto a = sdata.substr(sa[i + 0]);
				auto b = sdata.substr(sa[i + 1]);
				PR_EXPECT(a < b);
			}

			PR_EXPECT(suffix_array::Contains<char>({ "Boobs", 5 }, data, sa));
		}
	}
}
#endif
