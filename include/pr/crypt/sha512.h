//*************************************************************************************************
// SHA512
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
	class SHA512
	{
		uint64_t m_workspace[80];
		uint64_t m_hash[8];
		uint64_t m_input[16];
		uint64_t m_input_size[2];
		size_t   m_input_idx;

	public:

		using hash_t = std::array<uint8_t, 64>;

		SHA512()
			:m_workspace()
			, m_hash()
			, m_input()
			, m_input_size()
			, m_input_idx()
		{
			m_hash[0] = 0x6a09e667f3bcc908;
			m_hash[1] = 0xbb67ae8584caa73b;
			m_hash[2] = 0x3c6ef372fe94f82b;
			m_hash[3] = 0xa54ff53a5f1d36f1;
			m_hash[4] = 0x510e527fade682d1;
			m_hash[5] = 0x9b05688c2b3e6c1f;
			m_hash[6] = 0x1f83d9abfb41bd6b;
			m_hash[7] = 0x5be0cd19137e2179;
		}
		~SHA512()
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
			auto align = std::min(align_to(m_input_idx, 128), len);
			Update(bytes, align);
			bytes += align;
			len -= align;

			// Process the message block by block
			for (size_t i = 0; i < len / 128; ++i)
			{
				// Number of blocks
				for (size_t j = 0; j < 16; j++)
					m_input[j] = load64_be(bytes + j * 8);

				bytes += 128;
				m_input_idx += 128;
				EndBlock();
			}
			len &= 127;

			// remaining bytes
			Update(bytes, len);
		}

		// Finalize hash; call this after all data is added
		hash_t Final()
		{
			Increment(m_input_size, m_input_idx * 8); // size is in bits
			SetInput(128); // padding

			// compress penultimate block (if any)
			if (m_input_idx > 111)
			{
				Compress();
				memset(&m_input[0], 0, 14 * sizeof(uint64_t));
			}

			// compress last block
			m_input[14] = m_input_size[0];
			m_input[15] = m_input_size[1];
			Compress();

			return Hash();
		}

		// Get the hash value
		hash_t Hash() const
		{
			hash_t hash;
			
			// Copy hash to output (big endian)
			for (size_t i = 0; i < 8; i++)
				store64_be(&hash[i * 8], m_hash[i]);

			return hash;
		}

	private:

		static constexpr size_t align_to(size_t x, size_t alignment)
		{
			return (~x + 1) & (alignment - 1);
		}
		static constexpr uint64_t rot(uint64_t x, int c)
		{
			return (x >> c) | (x << (64 - c));
		}
		static constexpr uint64_t ch(uint64_t x, uint64_t y, uint64_t z)
		{
			return (x & y) ^ (~x & z);
		}
		static constexpr uint64_t maj(uint64_t x, uint64_t y, uint64_t z)
		{
			return (x & y) ^ (x & z) ^ (y & z);
		}
		static constexpr uint64_t big_sigma0(uint64_t x)
		{
			return rot(x, 28) ^ rot(x, 34) ^ rot(x, 39);
		}
		static constexpr uint64_t big_sigma1(uint64_t x)
		{
			return rot(x, 14) ^ rot(x, 18) ^ rot(x, 41);
		}
		static constexpr uint64_t lit_sigma0(uint64_t x)
		{
			return rot(x, 1) ^ rot(x, 8) ^ (x >> 7);
		}
		static constexpr uint64_t lit_sigma1(uint64_t x)
		{
			return rot(x, 19) ^ rot(x, 61) ^ (x >> 6);
		}
		static constexpr uint64_t load64_be(const uint8_t s[8])
		{
			return
				((uint64_t)s[0] << 56) |
				((uint64_t)s[1] << 48) |
				((uint64_t)s[2] << 40) |
				((uint64_t)s[3] << 32) |
				((uint64_t)s[4] << 24) |
				((uint64_t)s[5] << 16) |
				((uint64_t)s[6] << 8) |
				((uint64_t)s[7]);
		}
		static constexpr void store64_be(uint8_t* out, uint64_t in)
		{
			out[0] = (in >> 56) & 0xff;
			out[1] = (in >> 48) & 0xff;
			out[2] = (in >> 40) & 0xff;
			out[3] = (in >> 32) & 0xff;
			out[4] = (in >> 24) & 0xff;
			out[5] = (in >> 16) & 0xff;
			out[6] = (in >> 8) & 0xff;
			out[7] = in & 0xff;
		}

		void Update(uint8_t const* bytes, size_t len)
		{
			for (size_t i = 0; i < len; i++)
			{
				SetInput(bytes[i]);
				m_input_idx++;
				EndBlock();
			}
		}
		void EndBlock()
		{
			if (m_input_idx == 128)
			{
				Increment(m_input_size, 1024); // size is in bits
				Compress();
				m_input_idx = 0;
			}
		}
		void Compress()
		{
			auto w = &m_workspace[0];
			for (size_t i = 0; i < 16; i++)
				w[i] = m_input[i];
			for (size_t i = 16; i < 80; i++)
				w[i] = (lit_sigma1(w[i - 2]) + w[i - 7] + lit_sigma0(w[i - 15]) + w[i - 16]);

			uint64_t a = m_hash[0], b = m_hash[1];
			uint64_t c = m_hash[2], d = m_hash[3];
			uint64_t e = m_hash[4], f = m_hash[5];
			uint64_t g = m_hash[6], h = m_hash[7];
			for (size_t i = 0; i < 80; i++)
			{
				uint64_t t1 = big_sigma1(e) + ch(e, f, g) + h + iv[i] + w[i];
				uint64_t t2 = big_sigma0(a) + maj(a, b, c);
				h = g;  g = f;  f = e;  e = d + t1;
				d = c;  c = b;  b = a;  a = t1 + t2;
			}
			m_hash[0] += a; m_hash[1] += b;
			m_hash[2] += c; m_hash[3] += d;
			m_hash[4] += e; m_hash[5] += f;
			m_hash[6] += g; m_hash[7] += h;
		}
		void SetInput(uint8_t input)
		{
			if (m_input_idx == 0)
				memset(&m_input, 0, sizeof(m_input));

			auto word = m_input_idx / 8;
			auto byte = m_input_idx % 8;
			m_input[word] |= (uint64_t)input << (8 * (7 - byte));
		}
		void Increment(uint64_t (&x)[2], uint64_t y)
		{
			// Increment a 128-bit "word".
			x[1] += y;
			if (x[1] < y)
				x[0]++;
		}

	private:

		inline static const uint64_t iv[80] =
		{
			0x428a2f98d728ae22, 0x7137449123ef65cd, 0xb5c0fbcfec4d3b2f, 0xe9b5dba58189dbbc,
			0x3956c25bf348b538, 0x59f111f1b605d019, 0x923f82a4af194f9b, 0xab1c5ed5da6d8118,
			0xd807aa98a3030242, 0x12835b0145706fbe, 0x243185be4ee4b28c, 0x550c7dc3d5ffb4e2,
			0x72be5d74f27b896f, 0x80deb1fe3b1696b1, 0x9bdc06a725c71235, 0xc19bf174cf692694,
			0xe49b69c19ef14ad2, 0xefbe4786384f25e3, 0x0fc19dc68b8cd5b5, 0x240ca1cc77ac9c65,
			0x2de92c6f592b0275, 0x4a7484aa6ea6e483, 0x5cb0a9dcbd41fbd4, 0x76f988da831153b5,
			0x983e5152ee66dfab, 0xa831c66d2db43210, 0xb00327c898fb213f, 0xbf597fc7beef0ee4,
			0xc6e00bf33da88fc2, 0xd5a79147930aa725, 0x06ca6351e003826f, 0x142929670a0e6e70,
			0x27b70a8546d22ffc, 0x2e1b21385c26c926, 0x4d2c6dfc5ac42aed, 0x53380d139d95b3df,
			0x650a73548baf63de, 0x766a0abb3c77b2a8, 0x81c2c92e47edaee6, 0x92722c851482353b,
			0xa2bfe8a14cf10364, 0xa81a664bbc423001, 0xc24b8b70d0f89791, 0xc76c51a30654be30,
			0xd192e819d6ef5218, 0xd69906245565a910, 0xf40e35855771202a, 0x106aa07032bbd1b8,
			0x19a4c116b8d2d0c8, 0x1e376c085141ab53, 0x2748774cdf8eeb99, 0x34b0bcb5e19b48a8,
			0x391c0cb3c5c95a63, 0x4ed8aa4ae3418acb, 0x5b9cca4f7763e373, 0x682e6ff3d6b2b8a3,
			0x748f82ee5defb2fc, 0x78a5636f43172f60, 0x84c87814a1f0ab72, 0x8cc702081a6439ec,
			0x90befffa23631e28, 0xa4506cebde82bde9, 0xbef9a3f7b2c67915, 0xc67178f2e372532b,
			0xca273eceea26619c, 0xd186b8c721c0c207, 0xeada7dd6cde0eb1e, 0xf57d4f7fee6ed178,
			0x06f067aa72176fba, 0x0a637dc5a2c898a6, 0x113f9804bef90dae, 0x1b710b35131c471b,
			0x28db77f523047d84, 0x32caab7b40c72493, 0x3c9ebe0a15c9bebc, 0x431d67c49c100d4c,
			0x4cc5d4becb3e42b6, 0x597f299cfc657e2a, 0x5fcb6fab3ad6faec, 0x6c44198c4a475817
		};
	};

	// Return the SHA512 hash of the given data
	inline SHA512::hash_t Sha512Hash(void const* data, size_t len)
	{
		SHA512 ctx;
		ctx.Update(data, len);
		return ctx.Final();
	}

	// Hash file contents
	inline SHA512::hash_t Sha512HashFile(std::filesystem::path const& filepath)
	{
		SHA512 sha;
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
	PRUnitTest(Sha512Tests)
	{
		using namespace pr::hash;

		char str0[] = "01234567890";
		char str1[] = "0123456789a";
		auto hash1 = Sha512Hash(str0, sizeof(str0));
		auto hash2 = Sha512Hash(str1, sizeof(str1));
		PR_EXPECT(hash1 != hash2);

		// This hash doesn't seem to produce the same result as the windows context menu one
		//auto hash3 = Sha512HashFile("P:\\pr\\include\\pr\\crypt\\rijndael.h");
		//auto hash4 = SHA512::hash_t{0x49,0x74,0x3D,0x87,0xDF,0xC8,0x63,0x83,0x18,0xBE,0x23,0x42,0xB2,0x47,0x06,0xE9,0x16,0x09,0xE9,0x85};
		//PR_EXPECT(hash3 == hash4);
	}

	// These lengths exercise the block-boundary clearing logic in SetInput()/Final(): a 129-byte
	// input leaves a 1-byte remainder block after the first full 128-byte block is compressed
	// directly (bypassing SetInput's per-block clear), and a 112-byte input fills the "penultimate
	// block" branch in Final() exactly, leaving the size-field words as the only ones not set by
	// message bytes. Expected hashes were computed independently (Python hashlib.sha512) for the
	// deterministic byte pattern `i % 251`.
	PRUnitTest(Sha512BlockBoundaryTests)
	{
		using namespace pr::hash;

		std::array<uint8_t, 129> data129;
		for (size_t i = 0; i != data129.size(); ++i)
			data129[i] = static_cast<uint8_t>(i % 251);

		std::array<uint8_t, 112> data112;
		for (size_t i = 0; i != data112.size(); ++i)
			data112[i] = static_cast<uint8_t>(i % 251);

		auto hash129 = Sha512Hash(data129.data(), data129.size());
		auto expected129 = SHA512::hash_t{
			0x1d, 0x9d, 0xa5, 0x7f, 0xbb, 0xda, 0xb0, 0x9a, 0xfb, 0x35, 0x06, 0xab, 0x2d, 0x22, 0x3d, 0x06,
			0x10, 0x9d, 0x65, 0xc1, 0xc8, 0xad, 0x19, 0x7f, 0x50, 0x13, 0x8f, 0x71, 0x4b, 0xc4, 0xc3, 0xf2,
			0xfe, 0x57, 0x87, 0x92, 0x26, 0x39, 0xc6, 0x80, 0xac, 0xad, 0x1c, 0x65, 0x1f, 0x95, 0x59, 0x90,
			0x42, 0x59, 0x54, 0xce, 0x2c, 0xba, 0x0c, 0x5c, 0xc8, 0x3f, 0x26, 0x67, 0xd8, 0x78, 0xeb, 0x0f };
		PR_EXPECT(hash129 == expected129);

		auto hash112 = Sha512Hash(data112.data(), data112.size());
		auto expected112 = SHA512::hash_t{
			0xc5, 0xfb, 0xd7, 0x31, 0xd1, 0x9d, 0x2a, 0xe1, 0x18, 0x0f, 0x00, 0x1b, 0xe7, 0x2c, 0x2c, 0x1a,
			0xab, 0xa1, 0xd7, 0xb0, 0x94, 0xb3, 0x74, 0x88, 0x80, 0xe2, 0x45, 0x93, 0xb8, 0xe1, 0x17, 0xa7,
			0x50, 0xe1, 0x1c, 0x1b, 0xd8, 0x67, 0xcc, 0x2f, 0x96, 0xda, 0xce, 0x8c, 0x8b, 0x74, 0xab, 0xd2,
			0xd5, 0xc4, 0xf2, 0x36, 0xbe, 0x44, 0x4e, 0x77, 0xd3, 0x0d, 0x19, 0x16, 0x17, 0x40, 0x70, 0xb9 };
		PR_EXPECT(hash112 == expected112);
	}
}
#endif
