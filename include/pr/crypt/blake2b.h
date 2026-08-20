//*************************************************************************************************
// Blake2b
//*************************************************************************************************
//  100% free public domain implementation of the SHA-512 algorithm.
//  Github: https://github.com/LoupVaillant/Monocypher
//
// Reduced to a single header and converted to C++s by me
#pragma once

#include <array>
#include <fstream>
#include <filesystem>
#include <cstdint>
#include <intrin.h>

namespace pr::hash
{
	template <int HashSize = 64>
	class Blake2b
	{
		uint64_t m_hash[8];
		uint64_t m_input_offset[2];
		uint64_t m_input[16];
		size_t   m_input_idx;

	public:

		using hash_t = typename std::array<uint8_t, HashSize>;

		Blake2b()
			:Blake2b(nullptr, 0ULL)
		{}
		Blake2b(uint8_t const* key, size_t key_size)
			:m_hash()
			,m_input_offset()
			,m_input()
			,m_input_idx()
		{
			// initial hash
			for (size_t i = 0; i != 8; ++i)
				m_hash[i] = iv[i];

			m_hash[0] ^= 0x01010000 ^ (key_size << 8) ^ HashSize;

			// if there is a key, the first block is that key (padded with zeroes)
			if (key_size > 0)
			{
				Update(key, key_size);
				Update(zero, 128 - key_size);
			}
		}
		~Blake2b()
		{
			// Reset memory for security reasons
			auto me = reinterpret_cast<uint8_t volatile*>(this);
			for (auto size = sizeof(*this); size-- != 0; ++me)
				*me = 0;
		}

		// Add data to the hash
		void Update(void const* data, size_t len)
		{
			auto bytes = static_cast<uint8_t const*>(data);

			// Align ourselves with block boundaries
			size_t align = std::min(align_to(m_input_idx, 128), len);
			Update(bytes, align);
			bytes += align;
			len -= align;

			// Process the bytes block by block
			for (size_t i = 0; i != len >> 7; ++i)
			{
				// number of blocks
				EndBlock();
				for (size_t j = 0; j != 16; ++j)
					m_input[j] = load64_le(bytes + j * 8);

				bytes += 128;
				m_input_idx = 128;
			}
			len &= 127;

			// Remaining bytes
			Update(bytes, len);
		}

		// Finalise  hash; call this after all data is added
		hash_t Final()
		{
			// Pad the end of the block with zeroes
			for (size_t i = m_input_idx; i != 128; ++i)
				SetInput(0, i);

			Increment();  // update the input offset
			Compress(true); // compress the last block

			return Hash();
		}

		// Get the hash value
		hash_t Hash() const
		{
			hash_t hash;

			// NOte: HashSize not necessarily 64 so 'num_bytes * 8' != HashSize
			size_t num_bytes = HashSize / 8;

			// Copy hash to output (big endian)
			for (size_t i = 0; i != num_bytes; ++i)
				store64_le(&hash[i * 8], m_hash[i]);
			for (size_t i = num_bytes * 8; i != HashSize; ++i)
				hash[i] = (m_hash[i >> 3] >> (8 * (i & 7))) & 0xff;

			return hash;
		}

	private:

		static constexpr size_t align_to(size_t x, size_t alignment)
		{
			return (~x + 1) & (alignment - 1);
		}
		static constexpr uint32_t load24_le(const uint8_t s[3])
		{
			return
				((uint32_t)s[0]) |
				((uint32_t)s[1] << 8) |
				((uint32_t)s[2] << 16);
		}
		static constexpr uint32_t load32_le(const uint8_t s[4])
		{
			return
				((uint32_t)s[0]) |
				((uint32_t)s[1] << 8) |
				((uint32_t)s[2] << 16) |
				((uint32_t)s[3] << 24);
		}
		static constexpr uint64_t load64_le(const uint8_t s[8])
		{
			return load32_le(s) | ((uint64_t)load32_le(s + 4) << 32);
		}
		static constexpr void store32_le(uint8_t* out, uint32_t in)
		{
			out[0] = in & 0xff;
			out[1] = (in >> 8) & 0xff;
			out[2] = (in >> 16) & 0xff;
			out[3] = (in >> 24) & 0xff;
		}
		static constexpr void store64_le(uint8_t* out, uint64_t in)
		{
			store32_le(out, (uint32_t)in);
			store32_le(out + 4, in >> 32);
		}
		static constexpr uint64_t rotr64(uint64_t x, uint64_t n)
		{
			return (x >> n) ^ (x << (64 - n));
		}
		static constexpr uint32_t rotl32(uint32_t x, uint32_t n)
		{
			return (x << n) ^ (x >> (32 - n));
		}

		void Update(uint8_t const* bytes, size_t len)
		{
			for (size_t i = 0; i != len; ++i)
			{
				EndBlock();
				SetInput(bytes[i], m_input_idx);
				m_input_idx++;
			}
		}
		void EndBlock()
		{
			// If buffer is full,
			if (m_input_idx == 128)
			{
				Increment(); // update the input offset
				Compress(false); // and compress the (not last) block
				m_input_idx = 0;
			}
		}
		void Compress(bool is_last_block)
		{
			static uint8_t const sigma[12][16] =
			{
				{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
			{ 14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3 },
			{ 11, 8, 12, 0, 5, 2, 15, 13, 10, 14, 3, 6, 7, 1, 9, 4 },
			{ 7, 9, 3, 1, 13, 12, 11, 14, 2, 6, 5, 10, 4, 0, 15, 8 },
			{ 9, 0, 5, 7, 2, 4, 10, 15, 14, 1, 11, 12, 6, 8, 3, 13 },
			{ 2, 12, 6, 10, 0, 11, 8, 3, 4, 13, 7, 5, 15, 14, 1, 9 },
			{ 12, 5, 1, 15, 14, 13, 4, 10, 0, 7, 6, 3, 9, 2, 8, 11 },
			{ 13, 11, 7, 14, 12, 1, 3, 9, 5, 0, 15, 4, 8, 6, 2, 10 },
			{ 6, 15, 14, 9, 11, 3, 0, 8, 12, 2, 13, 7, 1, 4, 10, 5 },
			{ 10, 2, 8, 4, 7, 6, 1, 5, 15, 11, 9, 14, 3, 12, 13, 0 },
			};

			// init work vector
			uint64_t v0 = m_hash[0];  uint64_t v8 = iv[0];
			uint64_t v1 = m_hash[1];  uint64_t v9 = iv[1];
			uint64_t v2 = m_hash[2];  uint64_t v10 = iv[2];
			uint64_t v3 = m_hash[3];  uint64_t v11 = iv[3];
			uint64_t v4 = m_hash[4];  uint64_t v12 = iv[4] ^ m_input_offset[0];
			uint64_t v5 = m_hash[5];  uint64_t v13 = iv[5] ^ m_input_offset[1];
			uint64_t v6 = m_hash[6];  uint64_t v14 = iv[6] ^ (is_last_block ? ~0ULL : 0ULL);
			uint64_t v7 = m_hash[7];  uint64_t v15 = iv[7];

			// mangle work vector
			uint64_t* input = m_input;
			#define BLAKE2_G(v, a, b, c, d, x, y)\
				v##a += v##b + x;  v##d = rotr64(v##d ^ v##a, 32); \
				v##c += v##d;      v##b = rotr64(v##b ^ v##c, 24); \
				v##a += v##b + y;  v##d = rotr64(v##d ^ v##a, 16); \
				v##c += v##d;      v##b = rotr64(v##b ^ v##c, 63);

			#define BLAKE2_ROUND(i)\
				BLAKE2_G(v, 0, 4,  8, 12, input[sigma[i][ 0]], input[sigma[i][ 1]]);\
				BLAKE2_G(v, 1, 5,  9, 13, input[sigma[i][ 2]], input[sigma[i][ 3]]);\
				BLAKE2_G(v, 2, 6, 10, 14, input[sigma[i][ 4]], input[sigma[i][ 5]]);\
				BLAKE2_G(v, 3, 7, 11, 15, input[sigma[i][ 6]], input[sigma[i][ 7]]);\
				BLAKE2_G(v, 0, 5, 10, 15, input[sigma[i][ 8]], input[sigma[i][ 9]]);\
				BLAKE2_G(v, 1, 6, 11, 12, input[sigma[i][10]], input[sigma[i][11]]);\
				BLAKE2_G(v, 2, 7,  8, 13, input[sigma[i][12]], input[sigma[i][13]]);\
				BLAKE2_G(v, 3, 4,  9, 14, input[sigma[i][14]], input[sigma[i][15]])

			BLAKE2_ROUND(0);  BLAKE2_ROUND(1);  BLAKE2_ROUND(2);  BLAKE2_ROUND(3);
			BLAKE2_ROUND(4);  BLAKE2_ROUND(5);  BLAKE2_ROUND(6);  BLAKE2_ROUND(7);
			BLAKE2_ROUND(8);  BLAKE2_ROUND(9);  BLAKE2_ROUND(0);  BLAKE2_ROUND(1);

			#undef BLAKE2_G
			#undef BLAKE2_ROUND

			// update hash
			m_hash[0] ^= v0 ^ v8;
			m_hash[1] ^= v1 ^ v9;
			m_hash[2] ^= v2 ^ v10;
			m_hash[3] ^= v3 ^ v11;
			m_hash[4] ^= v4 ^ v12;
			m_hash[5] ^= v5 ^ v13;
			m_hash[6] ^= v6 ^ v14;
			m_hash[7] ^= v7 ^ v15;
		}
		void SetInput(uint8_t input, size_t index)
		{
			if (index == 0)
				memset(&m_input[0], 0, sizeof(m_input));

			size_t word = index >> 3;
			size_t byte = index & 7;
			m_input[word] |= (uint64_t)input << (byte << 3);
		}
		void Increment()
		{
			// increment the input offset
			uint64_t* x = &m_input_offset[0];
			size_t y = m_input_idx;
			x[0] += y;
			if (x[0] < y)
				x[1]++;
		}

	private:
		inline static const uint8_t zero[128] =
		{
			0
		};
		inline static const uint64_t iv[8] =
		{
			0x6a09e667f3bcc908, 0xbb67ae8584caa73b,
			0x3c6ef372fe94f82b, 0xa54ff53a5f1d36f1,
			0x510e527fade682d1, 0x9b05688c2b3e6c1f,
			0x1f83d9abfb41bd6b, 0x5be0cd19137e2179,
		};
	};

	// Return the Blake2b hash of the given data
	template <int HashSize = 64>
	inline typename Blake2b<HashSize>::hash_t Blake2bHash(uint8_t const* key, size_t key_size, void const* data, size_t len)
	{
		Blake2b<HashSize> blake(key, key_size);
		blake.Update(data, len);
		return blake.Final();
	}

	// Return the Blake2b hash of the given data
	inline Blake2b<>::hash_t Blake2bHash(void const* data, size_t len)
	{
		return Blake2bHash<64>(0, 0, data, len);
	}

	// Hash file contents
	inline Blake2b<>::hash_t Blake2bHashFile(std::filesystem::path const& filepath)
	{
		Blake2b sha;
		std::array<char, 4096> buf;
		std::ifstream file(filepath, std::ios_base::binary);
		for (;file.good();)
		{
			auto read = file.read(buf.data(), buf.size()).gcount();
			sha.Update(buf.data(), int(read));
		}
		return sha.Final();
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::hash
{
	PRUnitTest(Blake2bTests, Quick)
	{
		using namespace pr::hash;

		char str0[] = "01234567890";
		char str1[] = "0123456789a";
		auto hash1 = Blake2bHash(str0, sizeof(str0));
		auto hash2 = Blake2bHash(str1, sizeof(str1));
		PR_EXPECT(hash1 != hash2);

		// Known-answer test spanning a full 128-byte block plus one extra byte. This exercises the
		// block-boundary clearing in 'SetInput', which must zero the whole 128-byte input buffer
		// when starting a new block; clearing only part of it leaves stale bytes from the previous
		// (full) block corrupting the following (short) block.
		{
			uint8_t data[129];
			for (int i = 0; i != 129; ++i)
				data[i] = (uint8_t)i;

			auto hash = Blake2bHash(data, sizeof(data));
			Blake2b<>::hash_t expected = {
				0xF5, 0x97, 0x11, 0xD4, 0x4A, 0x03, 0x1D, 0x5F, 0x97, 0xA9, 0x41, 0x3C, 0x06, 0x5D, 0x1E, 0x61,
				0x4C, 0x41, 0x7E, 0xDE, 0x99, 0x85, 0x90, 0x32, 0x5F, 0x49, 0xBA, 0xD2, 0xFD, 0x44, 0x4D, 0x3E,
				0x44, 0x18, 0xBE, 0x19, 0xAE, 0xC4, 0xE1, 0x14, 0x49, 0xAC, 0x1A, 0x57, 0x20, 0x78, 0x98, 0xBC,
				0x57, 0xD7, 0x6A, 0x1B, 0xCF, 0x35, 0x66, 0x29, 0x2C, 0x20, 0xC6, 0x83, 0xA5, 0xC4, 0x64, 0x8F,
			};
			PR_EXPECT(hash == expected);
		}

		// This hash doesn't seem to produce the same result as the windows context menu one
		//auto hash3 = Blake2bHashFile("P:\\pr\\include\\pr\\crypt\\rijndael.h");
		//auto hash4 = Blake2b<>::hash_t{0xD4,0x90,0xE7,0x7B,0x83,0x9D,0x61,0x2B,0x72,0x93,0x76,0x6E,0xFC,0x01,0x4C,0x0B,0x45,0x51,0x63,0x29,0x21,0xBA,0x3F,0xB4,0xB6,0x2A,0x5C,0xA5,0x40,0x0D,0x7E,0x75};
		//PR_EXPECT(hash3 == hash4);
	}
}
#endif