//*********************************************
// Bit Reader
//  Copyright (c) Rylogic Ltd 2024
//*********************************************
// Read or Write bits to a data source or sink

#pragma once
#include <vector>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <cstdint>
#include <concepts>
#include <type_traits>
#include <limits>
#include <span>

namespace pr
{
	namespace bit_data
	{
		// A bit reader data source must have a ReadWord() method that returns a word type
		template <typename T, typename WordType>
		concept DataSource = requires(T a)
		{
			std::is_unsigned_v<WordType>;
			{ a.ReadWord() } -> std::convertible_to<WordType>;
			{ a.SizeInBits() } -> std::convertible_to<int64_t>;
		};

		// A bit writer data sink must have a WriteWord() method that takes a word type
		template <typename T, typename WordType>
		concept DataSink = requires(T a, WordType b)
		{
			std::is_unsigned_v<WordType>;
			{ a.WriteWord(b) };
		};

		// A data source/sink based on a contiguous block of memory
		template <typename WordType>
		struct ContiguousDataSource
		{
			std::span<WordType const> m_data;
			size_t m_bit_count;
			size_t m_offset;

			ContiguousDataSource(std::span<WordType const> data, size_t bit_count = std::dynamic_extent)
				: m_data(data)
				, m_bit_count(bit_count != std::dynamic_extent ? bit_count : data.size() * sizeof(WordType) * 8)
				, m_offset(0)
			{}
			WordType ReadWord()
			{
				if (m_offset >= m_data.size())
					throw std::out_of_range("End of data source");

				return m_data[m_offset++];
			}
			int64_t SizeInWords() const
			{
				return m_data.size();
			}
			int64_t SizeInBits() const
			{
				return m_bit_count;
			}
		};
		template <typename WordType>
		struct ContiguousDataSink
		{
			std::span<WordType> m_data;
			size_t m_offset;

			ContiguousDataSink(std::span<WordType> data)
				: m_data(data)
				, m_offset(0)
			{}
			void WriteWord(WordType w)
			{
				if (m_offset >= m_data.size())
					throw std::out_of_range("End of data sink");

				m_data[m_offset++] = w;
			}
		};

		// A data source based on a stream
		struct StreamDataSource
		{
			std::istream* m_stream;
			StreamDataSource(std::istream& stream)
				: m_stream(&stream)
			{}
			uint8_t ReadWord()
			{
				if (m_stream->eof())
					throw std::out_of_range("End of data source");

				// read one byte from the stream
				uint8_t word;
				if (!m_stream->read(reinterpret_cast<char*>(&word), 1).good())
					throw std::runtime_error("Failed to read from data source");

				return word;
			}
			int64_t SizeInBits() const
			{
				return m_stream->eof() ? 0 : std::numeric_limits<int64_t>::max();
			}
		};
		struct StreamDataSink
		{
			std::ostream* m_stream;
			StreamDataSink(std::ostream& stream)
				: m_stream(&stream)
			{}
			void WriteWord(uint8_t word)
			{
				if (m_stream->eof())
					throw std::out_of_range("End of data sink");

				// write one byte to the stream
				if (!m_stream->write(reinterpret_cast<char const*>(&word), 1).good())
					throw std::runtime_error("Failed to write to data sink");
			}
		};

		// Build a mask for the low 'bits' bits of a word without shifting by the full word size.
		template <std::unsigned_integral WordType>
		constexpr WordType BitsMask(int bits)
		{
			if (bits == 0)
				return {};

			if (bits == std::numeric_limits<WordType>::digits)
				return std::numeric_limits<WordType>::max();

			return (WordType{1} << bits) - 1;
		}

		// Read bits from a data source
		template <std::unsigned_integral WordType, DataSource<WordType> DataSrc>
		class Reader
		{
			static constexpr int WordSize = sizeof(WordType) * 8;
			static constexpr int64_t OfsMask = WordSize - 1;

			DataSrc m_src;   // The source of bits.
			int64_t m_pos;   // The position of the next bit (in [0, m_src.SizeInBits()))
			int64_t m_wpos;  // The word index corresponding to 'm_word'
			WordType m_word; // The last word read from the source

		public:

			Reader()
				: m_src()
				, m_pos()
				, m_wpos(-1)
				, m_word()
			{}
			Reader(DataSrc&& src)
				: m_src(std::move(src))
				, m_pos()
				, m_wpos(-1)
				, m_word()
			{}
			Reader(Reader&&) = default;
			Reader(Reader const&) = delete;
			Reader& operator=(Reader&&) = default;
			Reader& operator=(Reader const&) = delete;

			// Access the data source
			DataSrc const& Source() const
			{
				return m_src;
			}

			// Return the current read position
			int64_t Position() const
			{
				return m_pos;
			}

			// Return the number of bits remaining in the data source
			int64_t RemainingBits() const
			{
				return m_src.SizeInBits() - m_pos;
			}

			// Read a single bit
			bool ReadBit()
			{
				auto word = CurrentWord();
				auto ofs = m_pos++ & OfsMask;
				return (word >> ofs) & 1;
			}

			// Read a number of bits
			template <typename T> requires (std::is_integral_v<T> || std::is_enum_v<T>)
			T ReadBits(int count)
			{
				if (count > sizeof(T) * 8)
					throw std::invalid_argument("count bits is larger than the size of the output type");
				if (count > m_src.SizeInBits() - m_pos)
					throw std::out_of_range("End of data source");

				uint64_t result = {};
				for (int idx = 0; idx != count; )
				{
					// The current word from the data source
					auto word = CurrentWord();

					// The number of bits to read from the current word
					auto ofs = static_cast<int>(m_pos & OfsMask);
					auto bits = std::min<int>(count - idx, WordSize - ofs);
					auto mask = BitsMask<WordType>(bits);

					result |= static_cast<uint64_t>((word >> ofs) & mask) << idx;

					m_pos += bits;
					idx += bits;
				}

				return static_cast<T>(result);
			}

			// Read a 'T'
			template <typename T> requires (std::is_integral_v<T> || std::is_floating_point_v<T> || std::is_enum_v<T>)
			T Read()
			{
				constexpr int bit_size = sizeof(T) * 8;
				if constexpr (bit_size == 8)
				{
					auto value = ReadBits<uint8_t>(bit_size);
					return reinterpret_cast<T const&>(value);
				}
				else if constexpr (bit_size == 16)
				{
					auto value = ReadBits<uint16_t>(bit_size);
					return reinterpret_cast<T const&>(value);
				}
				else if constexpr (bit_size == 32)
				{
					auto value = ReadBits<uint32_t>(bit_size);
					return reinterpret_cast<T const&>(value);
				}
				else if constexpr (bit_size == 64)
				{
					auto value = ReadBits<uint64_t>(bit_size);
					return reinterpret_cast<T const&>(value);
				}
				else
				{
					static_assert(std::is_same_v<T, void>, "Unsupported type size");
				}
			}

			// Read an array of 'T'
			template <typename T> requires (std::is_integral_v<T> || std::is_floating_point_v<T> || std::is_enum_v<T>)
			std::vector<T> ReadArray(int count)
			{
				std::vector<T> result = {};
				result.reserve(count);
				for (int i = 0; i != count; ++i)
					result.push_back(Read<T>());
				
				return result;
			}

			// Read a string of bytes
			std::string ReadString(int count)
			{
				std::string result;
				result.reserve(count);
				for (int i = 0; i != count; ++i)
					result.push_back(Read<char>());
				
				return result;
			}

		private:

			// Read the current word from the data source, advancing if necessary
			WordType CurrentWord()
			{
				if (RemainingBits() <= 0)
					throw std::out_of_range("End of data source");

				// This should only read each word once from the data source
				for (; m_pos / WordSize > m_wpos; ++m_wpos)
					m_word = m_src.ReadWord();

				return m_word;
			}
		};

		// Write bits to a data sink
		template <std::unsigned_integral WordType, DataSink<WordType> DataDst>
		class Writer
		{
			static constexpr int WordSize = sizeof(WordType) * 8;
			static constexpr int64_t OfsMask = WordSize - 1;

			DataDst m_dst;   // The sink for bits.
			int64_t m_pos;   // The number of bits written
			int64_t m_wpos;  // The number of words written
			WordType m_word; // The current word being written

		public:

			Writer()
				: m_dst()
				, m_pos()
				, m_wpos()
				, m_word()
			{}
			explicit Writer(DataDst&& dst)
				: m_dst(std::move(dst))
				, m_pos()
				, m_wpos()
				, m_word()
			{}
			Writer(Writer&&) = default;
			Writer(Writer const&) = delete;
			Writer& operator=(Writer&&) = default;
			Writer& operator=(Writer const&) = delete;
			~Writer()
			{
				Flush();
			}

			// Access the data destination
			DataDst const& Destination() const
			{
				return m_dst;
			}

			// Write a single bit
			Writer& WriteBit(bool bit)
			{
				auto& word = CurrentWord();
				auto ofs = m_pos++ & OfsMask;
				word |= static_cast<WordType>(bit) << ofs;
				return *this;
			}

			// Write a number of bits
			template <std::integral T>
			Writer& WriteBits(T value, int count)
			{
				if (count > sizeof(T) * 8)
					throw std::invalid_argument("count bits is larger than the size of 'value'");

				for (int idx = 0; idx != count; )
				{
					// The current word being constructed
					auto& word = CurrentWord();

					// The number of bits to write to the current word
					auto ofs = static_cast<int>(m_pos & OfsMask);
					auto bits = std::min<int>(count - idx, WordSize - ofs);
					auto mask = BitsMask<WordType>(bits);

					word |= static_cast<WordType>((value >> idx) & mask) << ofs;

					m_pos += bits;
					idx += bits;
				}

				return *this;
			}

			// Write a 'T'
			template <typename T> requires (std::is_integral_v<T> || std::is_floating_point_v<T>)
			Writer& Write(T value)
			{
				constexpr int bit_size = sizeof(T) * 8;
				if constexpr (bit_size == 8)
				{
					auto data = reinterpret_cast<uint8_t const&>(value);
					WriteBits<uint8_t>(data, bit_size);
				}
				else if constexpr (bit_size == 16)
				{
					auto data = reinterpret_cast<uint16_t const&>(value);
					WriteBits<uint16_t>(data, bit_size);
				}
				else if constexpr (bit_size == 32)
				{
					auto data = reinterpret_cast<uint32_t const&>(value);
					WriteBits<uint32_t>(data, bit_size);
				}
				else if constexpr (bit_size == 64)
				{
					auto data = reinterpret_cast<uint64_t const&>(value);
					WriteBits<uint64_t>(data, bit_size);
				}
				else
				{
					static_assert(std::is_same_v<T, void>, "Unsupported type size");
				}
				return *this;
			}

			// Write an array of 'T'
			template <typename T> requires (std::is_integral_v<T> || std::is_floating_point_v<T>)
			Writer& WriteArray(std::span<T const> data)
			{
				for (auto const& value : data)
					Write(value);
			
				return *this;
			}

			// Send any bits in the last word to the sink
			void Flush()
			{
				// Round 'm_pos' up to the next word boundary
				m_pos += ~(m_pos - 1) & (WordSize - 1);
				CurrentWord();
			}

		private:

			// When the current word is full, write it to the data sink
			WordType& CurrentWord()
			{
				for (; m_pos / WordSize > m_wpos; ++m_wpos)
				{
					m_dst.WriteWord(m_word);
					m_word = 0;
				}

				return m_word;
			}
		};
	}

	// Construct a bit reader for contiguous data
	template <std::unsigned_integral T = uint8_t>
	inline bit_data::Reader<T, bit_data::ContiguousDataSource<T>> BitReader(std::span<T const> data, size_t bit_count = std::dynamic_extent)
	{
		using SrcType = bit_data::ContiguousDataSource<T>;
		return bit_data::Reader<T, SrcType>(SrcType(data, bit_count));
	}
	inline bit_data::Reader<uint8_t, bit_data::ContiguousDataSource<uint8_t>> BitReader(std::span<uint8_t const> data, size_t bit_count = std::dynamic_extent)
	{
		return BitReader<uint8_t>(data, bit_count);
	}

	// Construct a bit reader for a stream source
	inline bit_data::Reader<uint8_t, bit_data::StreamDataSource> BitReader(std::istream& stream)
	{
		using SrcType = bit_data::StreamDataSource;
		return bit_data::Reader<uint8_t, SrcType>(SrcType(stream));
	}

	// Construct a bit writer for contiguous data
	template <std::unsigned_integral T = uint8_t>
	inline bit_data::Writer<T, bit_data::ContiguousDataSink<T>> BitWriter(std::span<T> data)
	{
		using DstType = bit_data::ContiguousDataSink<T>;
		return bit_data::Writer<T, DstType>(DstType(data));
	}
	inline bit_data::Writer<uint8_t, bit_data::ContiguousDataSink<uint8_t>> BitWriter(std::span<uint8_t> data)
	{
		return BitWriter<uint8_t>(data);
	}

	// Construct a bit writer for a stream sink
	inline bit_data::Writer<uint8_t, bit_data::StreamDataSink> BitWriter(std::ostream& stream)
	{
		using DstType = bit_data::StreamDataSink;
		return bit_data::Writer<uint8_t, DstType>(DstType(stream));
	}
}


// unit tests
#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(BitReaderTests, Quick)
	{
		{
			uint8_t data[] = { 0x21, 0x43, 0x65, 0x87, 0xA9 };
			auto reader = BitReader(data);

			PR_EXPECT(reader.ReadBit() == 1);
			PR_EXPECT(reader.ReadBit() == 0);
			PR_EXPECT(reader.ReadBit() == 0);
			PR_EXPECT(reader.ReadBit() == 0);

			PR_EXPECT(reader.ReadBits<int>(4) == 2);
			PR_EXPECT(reader.ReadBits<int>(8) == 0x43);

			PR_EXPECT(reader.Read<uint8_t>() == 0x65);
			PR_EXPECT(reader.Read<uint16_t>() == 0xA987);
		}
		{
			uint8_t data[] = { 0x12, 0x34, 0x56 };
			auto reader = BitReader(data, 19);

			PR_EXPECT(reader.ReadBits<int>(12) == 0x412);
			PR_EXPECT(reader.RemainingBits() == 7);
			PR_EXPECT(reader.ReadBits<int>(7) == 0x63);
			PR_EXPECT(reader.RemainingBits() == 0);
			PR_THROWS(reader.ReadBit(), std::out_of_range);
		}
		{
			uint8_t data[] = { 0x21, 0x43, 0x65, 0x87, 0xA9, 0xCB, 0xED, 0x0F, 0x10, 0x32 };
			auto reader = BitReader(data);

			PR_EXPECT(reader.ReadBits<int>(4) == 0x01);
			PR_EXPECT(reader.Read<uint64_t>() == 0x00FEDCBA98765432ULL);
			PR_EXPECT(reader.ReadBits<int>(12) == 0x321);
		}
		{
			uint16_t data[] = { 0x4321, 0x8765, 0xCBA9, 0x0FED, 0x3210 };
			auto reader = BitReader<uint16_t>(data);

			PR_EXPECT(reader.Read<uint8_t>() == 0x21);
			PR_EXPECT(reader.Read<uint8_t>() == 0x43);

			auto arr0 = reader.ReadArray<uint16_t>(2);
			PR_EXPECT(arr0.size() == 2);
			PR_EXPECT(arr0[0] == 0x8765);
			PR_EXPECT(arr0[1] == 0xCBA9);

			auto arr1 = reader.ReadArray<uint8_t>(4);
			PR_EXPECT(arr1.size() == 4);
			PR_EXPECT(arr1[0] == 0xED);
			PR_EXPECT(arr1[1] == 0x0F);
			PR_EXPECT(arr1[2] == 0x10);
			PR_EXPECT(arr1[3] == 0x32);
		}
		{
			std::string str = "ABCDEFGH";
			std::stringstream ss(str);
			auto reader = BitReader(ss);
			PR_EXPECT(reader.Read<char>() == 'A');
			PR_EXPECT(reader.Read<char>() == 'B');
			PR_EXPECT(reader.Read<char>() == 'C');

			auto arr = reader.ReadArray<char>(5);
			PR_EXPECT(arr.size() == 5);
			PR_EXPECT(arr[0] == 'D');
			PR_EXPECT(arr[1] == 'E');
			PR_EXPECT(arr[2] == 'F');
			PR_EXPECT(arr[3] == 'G');
			PR_EXPECT(arr[4] == 'H');
		}
		{
			uint8_t data[] = { 'B', 'o', 'o', 'b', 's' };
			auto reader = BitReader(data);

			PR_EXPECT(reader.ReadString(5) == "Boobs");
		}
		{
			enum class E : uint8_t { A = 0x12, B = 0x34, C = 0x56, D = 0x78, E = 0x9A };
			enum class F : uint8_t { AA = 0x04, BB = 0x03 };
			uint8_t data[] = { 0x12, 0x34, 0x56, 0x78, 0x9A };
			auto reader = BitReader(data);

			PR_EXPECT(reader.Read<E>() == E::A);
			PR_EXPECT(reader.ReadBits<F>(4) == F::AA);
			PR_EXPECT(reader.ReadBits<F>(4) == F::BB);
			PR_EXPECT(reader.Read<uint8_t>() == 0x56);
		}
	}
	PRUnitTest(BitWriterTests, Quick)
	{
		// Zero bits should leave both the writer output and reader position unchanged.
		{
			uint32_t data[] = { 0xDEADBEEF };
			auto writer = BitWriter<uint32_t>({ &data[0], _countof(data) });
			writer.WriteBits<uint32_t>(0x12345678u, 0);
			writer.Flush();
			PR_EXPECT(data[0] == 0xDEADBEEFu);

			auto reader = BitReader<uint32_t>(data);

			PR_EXPECT(reader.ReadBits<uint32_t>(0) == 0u);
			PR_EXPECT(reader.Position() == 0);
		}

		// These chunks hit the 32-bit full-width mask path and the carry into the next word.
		{
			uint32_t data[2] = {};
			auto writer = BitWriter<uint32_t>({ &data[0], _countof(data) });

			writer.WriteBits<uint32_t>(0x1u, 1);
			writer.WriteBits<uint32_t>(0x7FFFFFFFu, 31);
			writer.WriteBits<uint32_t>(0xFFFFFFFFu, 32);
			writer.Flush();

			PR_EXPECT(data[0] == 0xFFFFFFFFu);
			PR_EXPECT(data[1] == 0xFFFFFFFFu);

			auto reader = BitReader<uint32_t>(data);
			PR_EXPECT(reader.ReadBits<uint32_t>(1) == 0x1u);
			PR_EXPECT(reader.ReadBits<uint32_t>(31) == 0x7FFFFFFFu);
			PR_EXPECT(reader.ReadBits<uint32_t>(32) == 0xFFFFFFFFu);
		}

		// These chunks hit the 64-bit full-width mask path.
		{
			uint64_t data[2] = {};
			auto writer = BitWriter<uint64_t>({ &data[0], _countof(data) });

			writer.WriteBits<uint64_t>(0x1ull, 1);
			writer.WriteBits<uint64_t>(0x7FFFFFFFFFFFFFFFull, 63);
			writer.WriteBits<uint64_t>(0xFFFFFFFFFFFFFFFFull, 64);
			writer.Flush();

			PR_EXPECT(data[0] == 0xFFFFFFFFFFFFFFFFull);
			PR_EXPECT(data[1] == 0xFFFFFFFFFFFFFFFFull);

			auto reader = BitReader<uint64_t>(data);
			PR_EXPECT(reader.ReadBits<uint64_t>(1) == 0x1ull);
			PR_EXPECT(reader.ReadBits<uint64_t>(63) == 0x7FFFFFFFFFFFFFFFull);
			PR_EXPECT(reader.ReadBits<uint64_t>(64) == 0xFFFFFFFFFFFFFFFFull);
		}
		{
			uint8_t data[16] = {};
			auto writer = BitWriter({ &data[0], _countof(data) });

			writer.WriteBit(1);
			writer.WriteBit(0);
			writer.WriteBit(1);
			writer.WriteBit(0);
			writer.Flush();

			PR_EXPECT(data[0] == 0b0101);

			writer.WriteBits(0xAAAA, 12);
			writer.Flush();

			PR_EXPECT(data[1] == 0xAA);
			PR_EXPECT(data[2] == 0x0A);

			writer.Write<int>(0x12345678);
			writer.Flush();

			PR_EXPECT(data[3] == 0x78);
			PR_EXPECT(data[4] == 0x56);
			PR_EXPECT(data[5] == 0x34);
			PR_EXPECT(data[6] == 0x12);

			uint16_t arr[] = { 0x0123, 0x4567, 0x89AB };
			writer.WriteArray<uint16_t>(arr);
			writer.Flush();

			PR_EXPECT(data[7] == 0x23);
			PR_EXPECT(data[8] == 0x01);
			PR_EXPECT(data[9] == 0x67);
			PR_EXPECT(data[10] == 0x45);
			PR_EXPECT(data[11] == 0xAB);
			PR_EXPECT(data[12] == 0x89);
		}
		{
			uint32_t data[16] = {};
			auto writer = BitWriter<uint32_t>({ &data[0], _countof(data) });

			writer.WriteBit(1);
			writer.Flush();

			PR_EXPECT(data[0] == 1);
		}
		{
			std::stringstream ss;
			auto writer = BitWriter(ss);

			writer.Write('B');
			writer.Write('o');
			writer.Write('o');
			writer.Write('b');
			writer.Write('s');
			writer.Flush();

			PR_EXPECT(ss.str().size() == 5);
			PR_EXPECT(ss.str() == "Boobs");

			// Flush should be idempotent
			writer.Flush();
			PR_EXPECT(ss.str().size() == 5);
			PR_EXPECT(ss.str() == "Boobs");
		}
	}
}
#endif
