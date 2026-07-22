//*********************************************
// Bit Array
//  Copyright (c) Rylogic Ltd 2007
//*********************************************
#pragma once
#include <cstdint>
#include <bit>
#include <array>
#include <bitset>
#include <vector>
#include <string>
#include <string_view>
#include <concepts>
#include <stdexcept>
#include <algorithm>
#include <format>
#include <span>

namespace pr
{
	enum class EEndian
	{
		Little,
		Big,
	};

	// Use std::bitset for compile time sets
	template <std::size_t N>
	class bitset : std::bitset<N> {};

	// Dynamic bitset
	template <std::unsigned_integral WordType = unsigned char>
	class bitsetRT
	{
		// Note:
		//  Memory layout:
		//  Word |MSB           LSB
		//         +-------------+
		//   0     |1010101100111|
		//         +-------------+
		//   1     |    <-- 10011|
		//         +-------------+
		//   2     |             |
		//  In String Form:
		//    LSB   ->  MSB,LSB ->
		//    "111001101010111001..."
		// Shifts:
		//  Shifts are performed in the string representation, so "1110010" >> 3 = "0001110"
		//  Since LSB is first in string form, this is actually a << operation in memory
		
		// Assuming little endian architecture. Will need to extend this class if not true
		static_assert(std::endian::native == std::endian::little);

		inline static constexpr WordType ALL_ZEROS = static_cast<WordType>(0ULL);
		inline static constexpr WordType ALL_ONES = static_cast<WordType>(~0ULL);
		enum { BitsPerWord = sizeof(WordType) * 8 };
		using TBits = std::vector<WordType>;

		TBits m_bits; // The container of bits
		int m_unused; // The number of unused bits in the last word

	public:

		bitsetRT()
			: m_bits()
			, m_unused()
		{}
		bitsetRT(bitsetRT&&) = default;
		bitsetRT(bitsetRT const&) = default;
		bitsetRT& operator = (bitsetRT&&) = default;
		bitsetRT& operator = (bitsetRT const&) = default;

		explicit bitsetRT(std::string_view bit_string)
			: bitsetRT()
		{
			*this = bit_string;
		}
		explicit bitsetRT(size_t count, bool bit)
			: bitsetRT()
		{
			resize(count, bit);
		}

		// The number of bits in each word
		static constexpr int bits_per_word()
		{
			return BitsPerWord;
		}

		// Proxy for a single bit
		class reference
		{
			friend class bitsetRT;

			bitsetRT* m_bs;
			std::size_t m_idx;

			reference(bitsetRT& bs, std::size_t idx)
				:m_bs(&bs)
				,m_idx(idx)
			{}

		public:

			reference& operator = (bool val)
			{
				m_bs->set(m_idx, val);
				return *this;
			}
			reference& operator = (reference const& ref)
			{
				m_bs->set(m_idx, static_cast<bool>(ref));
				return *this;
			}
			reference& flip()
			{
				m_bs->set(m_idx, !m_bs->test(m_idx));
				return *this;
			}
			bool operator~() const
			{
				return !m_bs->test(m_idx);
			}
			operator bool() const
			{
				return m_bs->test(m_idx);
			}
		};

		// Access the n'th (zero based) bit
		bool operator[](std::size_t n) const
		{
			if (n >= size()) throw std::out_of_range(std::format("bitsetRT::operator[] index {} is outside range [0,{})", n, size()));
			return test(n);
		}
		reference operator[](std::size_t n)
		{
			if (n >= size()) throw std::out_of_range(std::format("bitsetRT::operator[] index {} is outside range [0,{})", n, size()));
			return reference(*this, n);
		}

		// Set the bits based on a string of 1s and 0s
		bitsetRT& operator = (std::string_view bit_string)
		{
			m_bits.resize(0);
			m_bits.reserve(word_count(bit_string.size()));

			auto eat_ws = [](auto& ptr, auto end)
			{
				for (; ptr != end && isspace(*ptr); ++ptr) {}
				if (ptr != end && *ptr != '0' && *ptr != '1')
					throw std::invalid_argument("bitsetRT::operator= invalid character in bit string");
			};

			auto len = 0;
			auto ptr = bit_string.data();
			auto end = ptr + bit_string.size();
			eat_ws(ptr, end);

			for (; ptr != end; )
			{
				auto w = WordType{};
				for (int j = 0; j != BitsPerWord && ptr != end; ++j, ++len, eat_ws(ptr, end))
					w |= int(*ptr++ == '1') << j;

				m_bits.push_back(w);
			}

			m_unused = static_cast<int>(ssize(m_bits) * BitsPerWord - len);
			return *this;
		}

		// True if there are no bits in the container
		bool empty() const
		{
			return m_bits.empty();
		}

		// The number of bits in the container
		std::size_t size() const
		{
			return size_in_words() * BitsPerWord - m_unused;
		}

		// The number of words used to contain the bits
		std::size_t size_in_words() const
		{
			return m_bits.size();
		}

		// Access the bit data buffer
		WordType const* data() const
		{
			return m_bits.data();
		}
		WordType* data()
		{
			return m_bits.data();
		}

		// Access the bit data buffer at an offset and reinterpret as type T.
		// Callers must request an address that is already suitably aligned for T and
		// that covers a whole number of T-sized words in the underlying storage.
		template <typename T> requires (std::is_standard_layout_v<T>)
		T const* ptr(std::size_t word_offset) const
		{
			static_assert(sizeof(T) % sizeof(WordType) == 0, "Size of T is not compatible with the word type");

			auto const t_size_in_words = sizeof(T) / sizeof(WordType);
			auto const size_words = size_in_words();
			if (t_size_in_words > size_words || word_offset > size_words - t_size_in_words)
				throw std::out_of_range("bitsetRT::ptr word offset is outside the buffer");

			auto ptr = data() + word_offset;
			if ((reinterpret_cast<std::uintptr_t>(ptr) % alignof(T)) != 0)
				throw std::invalid_argument("bitsetRT::ptr requested address is not aligned for T");

			return reinterpret_cast<T const*>(ptr);
		}
		template <typename T> requires (std::is_standard_layout_v<T>)
		T* ptr(std::size_t word_offset)
		{
			return const_cast<T*>(const_cast<bitsetRT const*>(this)->ptr<T>(word_offset));
		}

		// Reserve space for 'count' bits
		void reserve(std::size_t count)
		{
			m_bits.reserve(word_count(count));
		}

		// Resize to a new number of bits
		void resize(std::size_t count)
		{
			m_bits.resize(word_count(count));
			m_unused = static_cast<int>(ssize(m_bits) * BitsPerWord - count);
			MaskLast();
		}

		// Resize to a new number of bits filling with 'bit'
		void resize(std::size_t count, bool bit)
		{
			if (m_unused != 0 && bit)
				m_bits.back() |= ALL_ONES << (BitsPerWord - m_unused);
			
			m_bits.resize(word_count(count), bit ? ALL_ONES : ALL_ZEROS);
			m_unused = static_cast<int>(ssize(m_bits) * BitsPerWord - count);
			MaskLast();
		}

		// Append a bit to the end of the container (preventing implicit conversion)
		template <typename T> requires (std::is_same_v<T, bool> || std::is_same_v<T, int>)
		bitsetRT& append(T bit)
		{
			if constexpr (std::is_same_v<T, int>)
			{
				if (bit != 0 && bit != 1)
					throw std::invalid_argument("bitsetRT::append invalid bit value");
			}
			return AppendBits(int(bit), 1);
		}

		// Append bits from 'value'.
		template <typename T> requires (std::is_integral_v<T>)
		bitsetRT& append(T value, int bits)
		{
			using unsigned_t =
				std::conditional_t<sizeof(T) == 1, uint8_t,
				std::conditional_t<sizeof(T) == 2, uint16_t,
				std::conditional_t<sizeof(T) == 4, uint32_t,
				std::conditional_t<sizeof(T) == 8, uint64_t,
				void>>>>;

			static_assert(!std::is_same_v<unsigned_t, void>, "Unsupported integer type");

			if (bits < 0 || bits > sizeof(T) * 8)
				throw std::invalid_argument("bitsetRT::append invalid number of bits");

			auto data = std::bit_cast<unsigned_t>(value);
			return AppendBits(data, bits);
		}

		// Allow enums to be used like an integral type
		template <typename T> requires (std::is_enum_v<T>)
		bitsetRT& append(T value, int bits)
		{
			return AppendBits(static_cast<uint64_t>(value), bits);
		}

		// Append bits from 'value'.
		template <typename T> requires (std::is_floating_point_v<T>)
		bitsetRT& append(T value)
		{
			using unsigned_t =
				std::conditional_t<std::is_same_v<T, float>, uint32_t,
				std::conditional_t<std::is_same_v<T, double>, uint64_t,
				void>>;

			static_assert(!std::is_same_v<unsigned_t, void>, "Unsupported floating point type");

			auto data = std::bit_cast<unsigned_t>(value);
			return AppendBits(data, sizeof(unsigned_t) * 8);
		}

		// Append bits from another bitset
		template <size_t N>
		bitsetRT& append(std::bitset<N> const& rhs)
		{
			for (size_t i = 0; i != rhs.size(); ++i)
				AppendBits(rhs.test(i), 1);

			return *this;
		}

		// Append bits from another bitset
		bitsetRT& append(bitsetRT const& rhs)
		{
			for (auto i = 0ULL; i != rhs.size(); ++i)
				AppendBits(rhs.test(i), 1);

			return *this;
		}

		// Append a range of bytes to the end of the container. 8 bits are added for each byte.
		bitsetRT& append_bytes(std::span<uint8_t const> bytes)
		{
			for (auto byte : bytes)
				AppendBits(byte, 8);

			return *this;
		}
		bitsetRT& append_bytes(std::string_view bytes)
		{
			auto data = std::span<uint8_t const>{ reinterpret_cast<uint8_t const*>(bytes.data()), bytes.size() };
			return append_bytes(data);
		}

		// Append 'count' 0s or 1s to the container
		bitsetRT& append_fill(bool bit, int count)
		{
			auto const value = bit ? ALL_ONES : ALL_ZEROS;
			auto max = m_unused ? m_unused : BitsPerWord;
			for (; count != 0; )
			{
				auto num = std::min(count, max);
				AppendBits(value, num);
				max = BitsPerWord;
				count -= num;
			}
			return *this;
		}

		// Set all bits to zero
		bitsetRT& reset()
		{
			memset(m_bits.data(), 0x00, m_bits.size() * sizeof(WordType));
			MaskLast();
			return *this;
		}

		// Set all bits to one
		bitsetRT& set()
		{
			memset(m_bits.data(), 0xFF, m_bits.size() * sizeof(WordType));
			MaskLast();
			return *this;
		}

		// Flip all bits in this container
		bitsetRT& flip()
		{
			for (auto& w : m_bits)
				w = ~w;

			MaskLast();
			return *this;
		}

		// Test the value of the 'nth' bit
		bool test(std::size_t n) const
		{
			auto w = word(n);
			n %= BitsPerWord;
			return w & (1 << n);
		}

		// Set the value of the 'nth' bit
		bitsetRT& set(std::size_t n, bool val = true)
		{
			auto& w = word(n);
			n %= BitsPerWord;
			if (val) w |=  (1 << n);
			else     w &= ~(1 << n);
			return *this;
		}

		// True if any bit is set. Empty containers return false
		bool any() const
		{
			// Note: the last word should have zeros in the unused bit positions
			for (auto& w : m_bits) { if (w) return true; }
			return false;
		}

		// True if all bits are set. Empty containers return false
		bool all() const
		{
			if (empty())
				return false;

			auto whole_words = std::span<WordType const>{ m_bits.data(), m_bits.size() - int(m_unused != 0) };
			return
				std::all_of(whole_words.begin(), whole_words.end(), [](auto w) { return w == ALL_ONES; }) &&
				(m_unused == 0 || m_bits.back() == WordType(ALL_ONES >> m_unused));
		}

		// Test bitset for equality
		friend bool operator == (bitsetRT const& lhs, bitsetRT const& rhs)
		{
			return lhs.m_bits == rhs.m_bits && lhs.m_unused == rhs.m_unused;
		}
		friend bool operator != (bitsetRT const& lhs, bitsetRT const& rhs)
		{
			return !(lhs == rhs);
		}

		// Performs a bitwise combination of bitsets with the logical AND operation.
		friend bitsetRT& operator &= (bitsetRT& lhs, bitsetRT const& rhs)
		{
			if (lhs.size() != rhs.size())
				throw std::invalid_argument("bitsetRT::operator& bitset sizes do not match");

			for (auto count = lhs.m_bits.size(); count-- != 0;)
				lhs.m_bits[count] &= rhs.m_bits[count];

			lhs.MaskLast();
			return lhs;
		}
		friend bitsetRT operator & (bitsetRT const& lhs, bitsetRT const& rhs)
		{
			bitsetRT bs(lhs);
			return bs &= rhs;
		}

		// Performs a bitwise combination of bitsets with the inclusive OR operation.
		friend bitsetRT& operator |= (bitsetRT& lhs, bitsetRT const& rhs)
		{
			if (lhs.size() != rhs.size())
				throw std::invalid_argument("bitsetRT::operator| bitset sizes do not match");

			for (auto count = lhs.m_bits.size(); count-- != 0;)
				lhs.m_bits[count] |= rhs.m_bits[count];

			lhs.MaskLast();
			return lhs;
		}
		friend bitsetRT operator | (bitsetRT const& lhs, bitsetRT const& rhs)
		{
			bitsetRT bs(lhs);
			return bs |= rhs;
		}

		// Performs a bitwise combination of bitsets with the exclusive OR operation.
		friend bitsetRT& operator ^= (bitsetRT& lhs, bitsetRT const& rhs)
		{
			if (lhs.size() != rhs.size())
				throw std::invalid_argument("bitsetRT::operator^ bitset sizes do not match");

			for (auto count = lhs.m_bits.size(); count-- != 0;)
				lhs.m_bits[count] ^= rhs.m_bits[count];

			lhs.MaskLast();
			return lhs;
		}
		friend bitsetRT operator ^ (bitsetRT const& lhs, bitsetRT const& rhs)
		{
			bitsetRT bs(lhs);
			return bs ^= rhs;
		}

		// Shifts the bits in a bitset to the left a specified number of positions
		// NOTE: The shift is performed in the string representation, so "1110010" << 3 = "0010000"
		// which is actually a >> operation in memory since the LSB is first in the string form.
		friend bitsetRT& operator <<= (bitsetRT& lhs, std::size_t n)
		{
			if (n == 0)
				return lhs;
			if (n >= lhs.size())
				return lhs.reset();

			auto shift = n % BitsPerWord;
			auto count = n / BitsPerWord;

			if (count != 0)
			{
				auto i = 0ULL;
				for (; i != lhs.m_bits.size() - count; ++i)
					lhs.m_bits[i] = lhs.m_bits[i + count];
				for (; i != lhs.m_bits.size(); ++i)
					lhs.m_bits[i] = 0;
			}
			if (shift != 0)
			{
				for (auto i = 0; i != lhs.m_bits.size() - std::max(1ULL, count); ++i)
					lhs.m_bits[i] = (lhs.m_bits[i] >> shift) | (lhs.m_bits[i + 1] << (BitsPerWord - shift));
				lhs.m_bits.back() >>= shift;
			}

			lhs.MaskLast();
			return lhs;
		}
		friend bitsetRT operator << (bitsetRT const& lhs, std::size_t n)
		{
			bitsetRT bs(lhs);
			return bs <<= n;
		}

		// Shifts the bits to the right by a specified number of positions.
		// NOTE: The shift is performed in the string representation, so "1110010" >> 3 = "0001110"
		// which is actually a << operation in memory since the LSB is first in the string form.
		friend bitsetRT& operator >>= (bitsetRT& lhs, std::size_t n)
		{
			if (n == 0)
				return lhs;
			if (n >= lhs.size())
				return lhs.reset();

			auto shift = n % BitsPerWord;
			auto count = n / BitsPerWord;

			if (count != 0)
			{
				for (auto i = lhs.m_bits.size(); i-- != count; )
					lhs.m_bits[i] = lhs.m_bits[i - count];
				for (auto i = count; i-- != 0; )
					lhs.m_bits[i] = 0;
			}
			if (shift != 0)
			{
				for (auto i = lhs.m_bits.size(); i-- != std::max<size_t>(1, count); )
					lhs.m_bits[i] = (lhs.m_bits[i - 1] >> (BitsPerWord - shift)) | (lhs.m_bits[i] << shift);
				lhs.m_bits.front() <<= shift;
			}

			lhs.MaskLast();
			return lhs;
		}
		friend bitsetRT operator >> (bitsetRT const& lhs, std::size_t n)
		{
			bitsetRT bs(lhs);
			return bs >>= n;
		}

		// Returns a container with all bits flipped
		friend bitsetRT operator ~(bitsetRT const& lhs)
		{
			bitsetRT bs(lhs);
			for (auto& w : bs.m_bits) w = ~w;
			bs.MaskLast();
			return bs;
		}

		// Convert the bitset to a string of 1s and 0s.
		// Note: see comments above for the bit order. Left-most character is the LSB of Word 0
		std::string to_string(std::string_view delimiter = "") const
		{
			std::string str;
			if (!empty())
			{
				str.reserve(size() + size_in_words()*delimiter.size());

				auto whole_words = std::span<WordType const>{ m_bits.data(), m_bits.size() - int(m_unused != 0) };
				for (auto w : whole_words)
				{
					if (!str.empty())
						str.append(delimiter);

					for (int i = BitsPerWord; i-- != 0; w >>= 1)
						str.push_back('0' + (w & 1));
				}
				if (m_unused != 0)
				{
					if (!str.empty())
						str.append(delimiter);

					auto w = m_bits.back();
					for (int i = BitsPerWord; i-- != m_unused; w >>= 1)
						str.push_back('0' + (w & 1));
				}
			}
			return str;
		}

		// Implicit conversion to a span of words containing all bits
		operator std::span<WordType const>() const
		{
			return { data(), size_in_words() };
		}

	private:

		// Returns the number of words required to store the specified number of bits
		constexpr std::size_t word_count(std::size_t bit_count) const
		{
			return (bit_count + BitsPerWord - 1) / BitsPerWord;
		}

		// Access the word that contains the n'th bit
		WordType word(std::size_t n) const
		{
			if (n >= size()) throw std::out_of_range(std::format("bitsetRT::word() index {} is outside range [0,{})", n, size()));
			return m_bits[n / BitsPerWord];
		}
		WordType& word(std::size_t n)
		{
			if (n >= size()) throw std::out_of_range(std::format("bitsetRT::word() index {} is outside range [0,{})", n, size()));
			return m_bits[n / BitsPerWord];
		}

		// Append a bit to the end of the container (preventing implicit conversion)
		bitsetRT& AppendBits(uint64_t value, int bits)
		{
			for (; bits != 0; )
			{
				if (m_unused == 0)
				{
					m_bits.push_back(0);
					m_unused = BitsPerWord;
				}

				auto count = std::min(bits, m_unused);
				auto mask = (1ULL << count) - 1;

				m_bits.back() |= WordType(value & mask) << (BitsPerWord - m_unused);

				m_unused -= count;
				value >>= count;
				bits -= count;
			}
			return *this;
		}

		// Set the unused bits at the end of the container to zeros
		void MaskLast()
		{
			if (!m_unused) return;
			m_bits.back() &= WordType(ALL_ONES >> m_unused);
		}
	};
}

// unit tests
#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::container
{
	PRUnitTest(BitArrayTests)
	{
		{
			bitsetRT<unsigned char> bs1;
			PR_EXPECT(bs1.empty());
			PR_EXPECT(bs1.size() == 0);
			bs1.append(true);  PR_EXPECT(bs1.size() == 1);
			bs1.append(false); PR_EXPECT(bs1.size() == 2);
			bs1.append(true);  PR_EXPECT(bs1.size() == 3);
			bs1.append(false); PR_EXPECT(bs1.size() == 4);
			bs1.append(false); PR_EXPECT(bs1.size() == 5);
			bs1.append(true);  PR_EXPECT(bs1.size() == 6);
			PR_EXPECT(bs1[0] == true);
			PR_EXPECT(bs1[1] == false);
			PR_EXPECT(bs1[2] == true);
			PR_EXPECT(bs1[3] == false);
			PR_EXPECT(bs1[4] == false);
			PR_EXPECT(bs1[5] == true);
			PR_EXPECT(bs1.to_string() == "101001");
			PR_EXPECT(!bs1.empty());

			bs1.append(true);  PR_EXPECT(bs1.size() == 7);
			bs1.append(false); PR_EXPECT(bs1.size() == 8);
			bs1.append(true);  PR_EXPECT(bs1.size() == 9);
			bs1.append(false); PR_EXPECT(bs1.size() == 10);
			bs1.append(false); PR_EXPECT(bs1.size() == 11);
			PR_EXPECT(bs1[6] == true);
			PR_EXPECT(bs1[7] == false);
			PR_EXPECT(bs1[8] == true);
			PR_EXPECT(bs1[9] == false);
			PR_EXPECT(bs1[10] == false);
			PR_EXPECT(bs1.to_string() == "10100110100");
		}
		{
			auto bs1 = bitsetRT<unsigned char>{ "10100110100" };
			bs1.resize(8);
			PR_EXPECT(bs1.size() == 8);
			PR_EXPECT(bs1.to_string() == "10100110");
			bs1[7] = true;
			PR_EXPECT(bs1.to_string() == "10100111");
			bs1.flip();
			PR_EXPECT(bs1.to_string() == "01011000");

			bs1[6] = true;
			bs1[7] = true;
			PR_EXPECT(bs1.to_string() == "01011011");
			bs1.resize(6);
			PR_EXPECT(bs1.to_string() == "010110");
			bs1.resize(7);
			PR_EXPECT(bs1.to_string() == "0101100");
		}
		{
			bitsetRT<unsigned char> bs1{ "0101100" };
			PR_EXPECT(bs1.all() == false);
			PR_EXPECT(bs1.any() == true);
			bs1.set();
			PR_EXPECT(bs1.all() == true);
			PR_EXPECT(bs1.any() == true);
			bs1.reset();
			PR_EXPECT(bs1.all() == false);
			PR_EXPECT(bs1.any() == false);
		}
		{
			bitsetRT<unsigned char> bs1{"0110010"};
			PR_EXPECT(bs1.to_string() == "0110010");
			bs1 = "1011 0111 0010 1110 10";
			PR_EXPECT(bs1.to_string() == "101101110010111010");
			PR_EXPECT(bs1.data()[0] == 0b11101101);
			PR_EXPECT(bs1.data()[1] == 0b01110100);
			PR_EXPECT(bs1.data()[2] == 0b00000001);
		}
		{
			bitsetRT<unsigned char> bs1{ "101101110010111010" };
			bitsetRT<unsigned char> bs2("101101110010111010");
			PR_EXPECT(bs1 == bs2);
			bs1[1].flip();
			PR_EXPECT(bs1 != bs2);

			PR_EXPECT(bs1.to_string() == "111101110010111010");
			bs1 >>= 9;
			PR_EXPECT(bs1.to_string() == "000000000111101110");
			bs1 <<= 10;
			PR_EXPECT(bs1.to_string() == "111011100000000000");
			PR_EXPECT(bs1.to_string() == "111011100000000000");
			PR_EXPECT(bs2.to_string() == "101101110010111010");
			auto bs3 = bs1 & bs2;
			PR_EXPECT(bs3.to_string() == "101001100000000000");
			auto bs4 = bs1 | bs2;
			PR_EXPECT(bs4.to_string() == "111111110010111010");
			auto bs5 = bs1 ^ bs2;
			PR_EXPECT(bs5.to_string() == "010110010010111010");
			auto bs6 = ~bs2;
			PR_EXPECT(bs2.to_string() == "101101110010111010");
			PR_EXPECT(bs6.to_string() == "010010001101000101");
		}
		{
			bitsetRT<unsigned char> bs1{ "10000000" };
			PR_EXPECT(bs1.to_string() == "10000000");
			bs1 >>= 1;
			PR_EXPECT(bs1.to_string() == "01000000");
			bs1 >>= 6;
			PR_EXPECT(bs1.to_string() == "00000001");
			bs1 >>= 1;
			PR_EXPECT(bs1.to_string() == "00000000");
		}
		{
			bitsetRT<unsigned char> bs1{ "00000001" };
			PR_EXPECT(bs1.to_string() == "00000001");
			bs1 <<= 1;
			PR_EXPECT(bs1.to_string() == "00000010");
			bs1 <<= 6;
			PR_EXPECT(bs1.to_string() == "10000000");
			bs1 <<= 1;
			PR_EXPECT(bs1.to_string() == "00000000");
		}
		{
			bitsetRT<unsigned char> bs1;
			bs1.append(true);
			PR_EXPECT(bs1.size() == 1);
			bs1.append(0x01, 8);
			PR_EXPECT(bs1.size() == 9);
			bs1.append(6.28);
			PR_EXPECT(bs1.size() == 73);
		}
		{
			// Check the raw bytes directly so the test locks down the serialized format rather than
			// relying on floating-point comparisons that can hide bit-pattern regressions.
			auto expect_bytes = [](auto const& bs, std::span<uint8_t const> expected)
			{
				PR_EXPECT(bs.size() == expected.size() * 8);
				for (std::size_t i = 0; i != expected.size(); ++i)
					PR_EXPECT(bs.data()[i] == expected[i]);
			};

			{
				bitsetRT<uint8_t> bs;
				bs.append(1.0f);
				auto const expected = std::array<uint8_t, 4>{ 0x00, 0x00, 0x80, 0x3F };
				expect_bytes(bs, expected);
			}
			{
				bitsetRT<uint8_t> bs;
				bs.append(-0.0f);
				auto const expected = std::array<uint8_t, 4>{ 0x00, 0x00, 0x00, 0x80 };
				expect_bytes(bs, expected);
			}
			{
				bitsetRT<uint8_t> bs;
				bs.append(std::bit_cast<float>(0x7F800000u));
				auto const expected = std::array<uint8_t, 4>{ 0x00, 0x00, 0x80, 0x7F };
				expect_bytes(bs, expected);
			}
			{
				bitsetRT<uint8_t> bs;
				bs.append(std::bit_cast<float>(0x7FC12345u));
				auto const expected = std::array<uint8_t, 4>{ 0x45, 0x23, 0xC1, 0x7F };
				expect_bytes(bs, expected);
			}
			{
				bitsetRT<uint8_t> bs;
				bs.append(1.0);
				auto const expected = std::array<uint8_t, 8>{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F };
				expect_bytes(bs, expected);
			}
			{
				bitsetRT<uint8_t> bs;
				bs.append(-0.0);
				auto const expected = std::array<uint8_t, 8>{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80 };
				expect_bytes(bs, expected);
			}
			{
				bitsetRT<uint8_t> bs;
				bs.append(std::bit_cast<double>(0x7FF0000000000000ull));
				auto const expected = std::array<uint8_t, 8>{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x7F };
				expect_bytes(bs, expected);
			}
			{
				bitsetRT<uint8_t> bs;
				bs.append(std::bit_cast<double>(0x7FF8123456789ABCull));
				auto const expected = std::array<uint8_t, 8>{ 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0xF8, 0x7F };
				expect_bytes(bs, expected);
			}
		}
		{
			bitsetRT<unsigned char> bs1;
			PR_EXPECT(bs1.empty());
			PR_EXPECT(bs1.to_string() == "");
			bs1.append_bytes("\x01\x02\x03\x04");
			PR_EXPECT(bs1.size() == 4*8);
			PR_EXPECT(bs1.to_string(" ") == "10000000 01000000 11000000 00100000");
		}
		{
			bitsetRT<unsigned char> bs1;
			bs1.append(0x4321, 10);
			PR_EXPECT(bs1.data()[0] == 0x21);
			PR_EXPECT(bs1.data()[1] == 0x03);
			PR_EXPECT(bs1.to_string(" ") == "10000100 11");
		}
		{
			enum class EFlags { One = 1 << 0, Two = 1 << 1, Three = 1 << 2, Four = 1 << 3 };

			bitsetRT<unsigned char> bs1;
			bs1.append(EFlags::Three, 4);
			PR_EXPECT(bs1.to_string() == "0010");
		}
		{
			bitsetRT<unsigned char> bs1("0101");
			bs1.append_fill(false, 3);
			bs1.append_fill(true, 5);
			PR_EXPECT(bs1.to_string() == "010100011111");
		}
		{
			bitsetRT<unsigned char> bs1("1010");
			std::bitset<6> bs2(0b100111);

			bs1.append(bs2);
			PR_EXPECT(bs1.to_string() == "1010111001");
		}
		{
			bitsetRT<unsigned char> bs1("0101");
			bitsetRT<unsigned char> bs2("1010");

			bs1.append(bs2);
			PR_EXPECT(bs1.to_string() == "01011010");
		}
		{
			auto bs1 = bitsetRT<unsigned char>(5, true);
			bs1.resize(7, false);
			bs1.resize(10, true);
			PR_EXPECT(bs1.to_string() == "1111100111");
			bs1.resize(6);
			PR_EXPECT(bs1.to_string() == "111110");
		}
		{
			bitsetRT<unsigned char> bs1("10101010 01010101 10101010 01010101 10101010 01010101 10101010 01010101");
			auto const& cbs1 = bs1;

			PR_EXPECT(bs1.size_in_words() == 8);
			PR_EXPECT(*cbs1.ptr<uint32_t>(0) == 0xAA55AA55);
			PR_EXPECT(*cbs1.ptr<uint32_t>(4) == 0xAA55AA55);

			*bs1.ptr<uint16_t>(0) = 0xFFFF;
			*bs1.ptr<uint16_t>(4) = 0xFFFF;
			*bs1.ptr<uint8_t>(6) = 0xFF;

			PR_EXPECT(bs1.data()[0] == 0xFF);
			PR_EXPECT(bs1.data()[1] == 0xFF);
			PR_EXPECT(bs1.data()[2] == 0x55);
			PR_EXPECT(bs1.data()[3] == 0xAA);
			PR_EXPECT(bs1.data()[4] == 0xFF);
			PR_EXPECT(bs1.data()[5] == 0xFF);
			PR_EXPECT(bs1.data()[6] == 0xFF);
			PR_EXPECT(bs1.data()[7] == 0xAA);

			PR_THROWS(bs1.ptr<uint32_t>(1), std::invalid_argument);
			PR_THROWS(bs1.ptr<uint32_t>(6), std::out_of_range);
		}
	}
}
#endif
