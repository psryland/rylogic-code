//*************************************************************************************************
// MD5
//*************************************************************************************************
// RSA Data Security, Inc. MD5 Message Digest Algorithm      
//
// Reduced to a single header by me.
#pragma once

#include <array>
#include <fstream>
#include <intrin.h>

namespace pr
{
	namespace hash
	{
		// Class for computing the MD5 hash of blocks of data
		class MD5
		{
			using byte = unsigned char;
			using u32 = unsigned int;

			// Data structure for MD5 (Message Digest) computation
			u32  m_bits[2];    // number of _bits_ handled mod 2^64
			u32  m_state[4];     // scratch buffer
			byte m_in[64];     // input buffer
			byte m_digest[16]; // actual digest after MD5Final call

		public:

			MD5()
				:m_bits()
				,m_state()
				,m_in()
				,m_digest()
			{
				m_bits[0] = m_bits[1] = (u32)0;

				// Load magic initialization constants.
				m_state[0] = 0x67452301U;
				m_state[1] = 0xefcdab89U;
				m_state[2] = 0x98badcfeU;
				m_state[3] = 0x10325476U;
			}
			~MD5()
			{
				// Reset memory for security reasons
				memset(&m_state[0], 0, sizeof(m_state));
			}

			// Add data to the hash
			void Add(void const* data, int length)
			{
				auto inBuf = static_cast<byte const*>(data);
				auto inLen = u32(length);

				// Compute number of bytes mod 64
				auto mdi = int((m_bits[0] >> 3) & 0x3F);

				// Update number of bits
				if ((m_bits[0] + (inLen << 3)) < m_bits[0])
					m_bits[1]++; // overflow

				m_bits[0] += inLen << 3;
				m_bits[1] += inLen >> 29;

				for (;inLen-- != 0;)
				{
					// Add new character to buffer, increment 'mdi'
					m_in[mdi++] = *inBuf++;

					// Transform if necessary
					if (mdi == 0x40)
					{
						u32 in[16];
						for (u32 i = 0, ii = 0; i != 16; ++i, ii += 4)
						{
							in[i] = (u32(m_in[ii + 3]) << 24) |
									(u32(m_in[ii + 2]) << 16) |
									(u32(m_in[ii + 1]) <<  8) |
									(u32(m_in[ii + 0]) <<  0);
						}
						Transform(m_state, in);
						mdi = 0;
					}
				}
			}

			// Finalize hash; call this after all data is added
			byte const (&Final())[16]
			{
				static byte const PADDING[64] =
				{
					0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
				};

				u32 in[16];

				// Save number of bits
				in[14] = m_bits[0];
				in[15] = m_bits[1];

				// compute number of bytes mod 64
				auto mdi = int((m_bits[0] >> 3) & 0x3F);

				// Pad out to 56 mod 64
				auto padLen = (mdi < 56) ? (56 - mdi) : (120 - mdi);
				Add(PADDING, padLen);

				// Append length in bits and transform
				for (u32 i = 0, ii = 0; i != 14; ++i, ii += 4)
				{
					in[i] = (u32(m_in[ii + 3]) << 24) |
							(u32(m_in[ii + 2]) << 16) |
							(u32(m_in[ii + 1]) <<  8) |
							(u32(m_in[ii + 0]) <<  0);
				}
				Transform(m_state, in);

				// Store buffer in digest
				for (u32 i = 0, ii = 0; i != 4; ++i, ii += 4)
				{
					m_digest[ii + 0] = byte((m_state[i] >>  0) & 0xFF);
					m_digest[ii + 1] = byte((m_state[i] >>  8) & 0xFF);
					m_digest[ii + 2] = byte((m_state[i] >> 16) & 0xFF);
					m_digest[ii + 3] = byte((m_state[i] >> 24) & 0xFF);
				}

				return Hash();
			}

			// Get the hash value
			byte const (&Hash() const)[16]
			{
				return m_digest;
			}

		private:

			// Basic MD5 step. Transform 'buf' based on 'in'.
			static void Transform(u32* buf, u32 const* in)
			{
				#pragma region Helper Functions
				struct S
				{
					// F, G and H are basic MD5 functions: selection, majority, parity
					static u32 F(u32 x, u32 y, u32 z) { return (x & y) | (~x & z); }
					static u32 G(u32 x, u32 y, u32 z) { return (x & z) | (y & ~z); }
					static u32 H(u32 x, u32 y, u32 z) { return x ^ y ^ z; }
					static u32 I(u32 x, u32 y, u32 z) { return y ^ (x | ~z); }

					// RotateLeft rotates x left n bits
					static u32 RotateLeft(u32 x, u32 n)
					{
						#ifdef _MSC_VER
						return _rotl(x, n);
						#else
						return (x << n) | (x >> (32 - n));
						#endif
					}

					// FF, GG, HH, and II transformations for rounds 1, 2, 3, and 4
					// Rotation is separate from addition to prevent re-computation
					static void FF(u32& a, u32 b, u32 c, u32 d, u32 x, u32 s, u32 ac)
					{
						a += F(b, c, d) + x + ac;
						a = RotateLeft(a, s);
						a += b;
					}
					static void GG(u32& a, u32 b, u32 c, u32 d, u32 x, u32 s, u32 ac)
					{
						a += G(b, c, d) + x + ac;
						a = RotateLeft(a, s);
						a += b;
					}
					static void HH(u32& a, u32 b, u32 c, u32 d, u32 x, u32 s, u32 ac)
					{
						a += H(b, c, d) + x + ac;
						a = RotateLeft(a, s);
						a += b;
					}
					static void II(u32& a, u32 b, u32 c, u32 d, u32 x, u32 s, u32 ac)
					{
						a += I(b, c, d) + x + ac;
						a = RotateLeft(a, s);
						a += b;
					}
				};
				#pragma endregion

				u32 a = buf[0], b = buf[1], c = buf[2], d = buf[3];

				// Round 1
				u32 const S11 =  7;
				u32 const S12 = 12;
				u32 const S13 = 17;
				u32 const S14 = 22;
				S::FF(a, b, c, d, in[0 ], S11, 3614090360); // 1 
				S::FF(d, a, b, c, in[1 ], S12, 3905402710); // 2 
				S::FF(c, d, a, b, in[2 ], S13, 606105819 ); // 3 
				S::FF(b, c, d, a, in[3 ], S14, 3250441966); // 4 
				S::FF(a, b, c, d, in[4 ], S11, 4118548399); // 5 
				S::FF(d, a, b, c, in[5 ], S12, 1200080426); // 6 
				S::FF(c, d, a, b, in[6 ], S13, 2821735955); // 7 
				S::FF(b, c, d, a, in[7 ], S14, 4249261313); // 8 
				S::FF(a, b, c, d, in[8 ], S11, 1770035416); // 9 
				S::FF(d, a, b, c, in[9 ], S12, 2336552879); // 10
				S::FF(c, d, a, b, in[10], S13, 4294925233); // 11
				S::FF(b, c, d, a, in[11], S14, 2304563134); // 12
				S::FF(a, b, c, d, in[12], S11, 1804603682); // 13
				S::FF(d, a, b, c, in[13], S12, 4254626195); // 14
				S::FF(c, d, a, b, in[14], S13, 2792965006); // 15
				S::FF(b, c, d, a, in[15], S14, 1236535329); // 16

				// Round 2
				u32 const S21 =  5;
				u32 const S22 =  9;
				u32 const S23 = 14;
				u32 const S24 = 20;
				S::GG(a, b, c, d, in[1 ], S21, 4129170786); // 17
				S::GG(d, a, b, c, in[6 ], S22, 3225465664); // 18
				S::GG(c, d, a, b, in[11], S23, 643717713 ); // 19
				S::GG(b, c, d, a, in[0 ], S24, 3921069994); // 20
				S::GG(a, b, c, d, in[5 ], S21, 3593408605); // 21
				S::GG(d, a, b, c, in[10], S22, 38016083  ); // 22
				S::GG(c, d, a, b, in[15], S23, 3634488961); // 23
				S::GG(b, c, d, a, in[4 ], S24, 3889429448); // 24
				S::GG(a, b, c, d, in[9 ], S21, 568446438 ); // 25
				S::GG(d, a, b, c, in[14], S22, 3275163606); // 26
				S::GG(c, d, a, b, in[3 ], S23, 4107603335); // 27
				S::GG(b, c, d, a, in[8 ], S24, 1163531501); // 28
				S::GG(a, b, c, d, in[13], S21, 2850285829); // 29
				S::GG(d, a, b, c, in[2 ], S22, 4243563512); // 30
				S::GG(c, d, a, b, in[7 ], S23, 1735328473); // 31
				S::GG(b, c, d, a, in[12], S24, 2368359562); // 32

				// Round 3
				u32 const S31 =  4;
				u32 const S32 = 11;
				u32 const S33 = 16;
				u32 const S34 = 23;
				S::HH(a, b, c, d, in[5 ], S31, 4294588738); // 33
				S::HH(d, a, b, c, in[8 ], S32, 2272392833); // 34
				S::HH(c, d, a, b, in[11], S33, 1839030562); // 35
				S::HH(b, c, d, a, in[14], S34, 4259657740); // 36
				S::HH(a, b, c, d, in[1 ], S31, 2763975236); // 37
				S::HH(d, a, b, c, in[4 ], S32, 1272893353); // 38
				S::HH(c, d, a, b, in[7 ], S33, 4139469664); // 39
				S::HH(b, c, d, a, in[10], S34, 3200236656); // 40
				S::HH(a, b, c, d, in[13], S31, 681279174 ); // 41
				S::HH(d, a, b, c, in[0 ], S32, 3936430074); // 42
				S::HH(c, d, a, b, in[3 ], S33, 3572445317); // 43
				S::HH(b, c, d, a, in[6 ], S34, 76029189  ); // 44
				S::HH(a, b, c, d, in[9 ], S31, 3654602809); // 45
				S::HH(d, a, b, c, in[12], S32, 3873151461); // 46
				S::HH(c, d, a, b, in[15], S33, 530742520 ); // 47
				S::HH(b, c, d, a, in[2 ], S34, 3299628645); // 48

				// Round 4
				u32 const S41 =  6;
				u32 const S42 = 10;
				u32 const S43 = 15;
				u32 const S44 = 21;
				S::II(a, b, c, d, in[0 ], S41, 4096336452); // 49
				S::II(d, a, b, c, in[7 ], S42, 1126891415); // 50
				S::II(c, d, a, b, in[14], S43, 2878612391); // 51
				S::II(b, c, d, a, in[5 ], S44, 4237533241); // 52
				S::II(a, b, c, d, in[12], S41, 1700485571); // 53
				S::II(d, a, b, c, in[3 ], S42, 2399980690); // 54
				S::II(c, d, a, b, in[10], S43, 4293915773); // 55
				S::II(b, c, d, a, in[1 ], S44, 2240044497); // 56
				S::II(a, b, c, d, in[8 ], S41, 1873313359); // 57
				S::II(d, a, b, c, in[15], S42, 4264355552); // 58
				S::II(c, d, a, b, in[6 ], S43, 2734768916); // 59
				S::II(b, c, d, a, in[13], S44, 1309151649); // 60
				S::II(a, b, c, d, in[4 ], S41, 4149444226); // 61
				S::II(d, a, b, c, in[11], S42, 3174756917); // 62
				S::II(c, d, a, b, in[2 ], S43, 718787259 ); // 63
				S::II(b, c, d, a, in[9 ], S44, 3951481745); // 64

				buf[0] += a;
				buf[1] += b;
				buf[2] += c;
				buf[3] += d;
			}
		};

		// Return the MD5 hash of the given data
		inline std::array<unsigned char, 16> Md5Hash(void const* data, int length)
		{
			MD5 md5;
			md5.Add(data, length);
			auto hash = md5.Final();

			std::array<unsigned char, 16> ret;
			memcpy(ret.data(), &hash[0], ret.size());
			return ret;
		}

		// Hash file contents
		inline std::array<unsigned char, 16> Md5HashFile(wchar_t const* filepath)
		{
			if (filepath == nullptr)
				throw std::exception("filepath is null");

			MD5 md5;
			std::array<char, 4096> buf;
			std::ifstream file(filepath, std::ios_base::binary);
			for (int read = 1; read != 0;)
			{
				read = file.good() ? int(file.read(&buf[0], buf.size()).gcount()) : 0;
				md5.Add(buf.data(), int(read));
			}
			auto hash = md5.Final();

			std::array<unsigned char, 16> ret;
			memcpy(ret.data(), &hash[0], ret.size());
			return ret;
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::hash
{
	PRUnitTest(Md5Tests, Quick)
	{
		char str0[] = "01234567890";
		char str1[] = "0123456789a";
		auto hash1 = Md5Hash(str0, sizeof(str0));
		auto hash2 = Md5Hash(str1, sizeof(str1));
		PR_EXPECT(hash1 != hash2);
	}
}
#endif

/*
 **********************************************************************
 ** md5.h -- Header file for implementation of MD5                   **
 ** RSA Data Security, Inc. MD5 Message Digest Algorithm             **
 ** Created: 2/17/90 RLR                                             **
 ** Revised: 12/27/90 SRD,AJ,BSK,JT Reference C version              **
 ** Revised (for MD5): RLR 4/27/91                                   **
 **   -- G modified to have y&~z instead of y&z                      **
 **   -- FF, GG, HH modified to add in last register done            **
 **   -- Access pattern: round 2 works mod 5, round 3 works mod 3    **
 **   -- distinct additive constant for each step                    **
 **   -- round 4 added, working mod 7                                **
 **********************************************************************
 */

/*
 **********************************************************************
 ** Copyright (C) 1990, RSA Data Security, Inc. All rights reserved. **
 **                                                                  **
 ** License to copy and use this software is granted provided that   **
 ** it is identified as the "RSA Data Security, Inc. MD5 Message     **
 ** Digest Algorithm" in all material mentioning or referencing this **
 ** software or this function.                                       **
 **                                                                  **
 ** License is also granted to make and use derivative works         **
 ** provided that such works are identified as "derived from the RSA **
 ** Data Security, Inc. MD5 Message Digest Algorithm" in all         **
 ** material mentioning or referencing the derived work.             **
 **                                                                  **
 ** RSA Data Security, Inc. makes no representations concerning      **
 ** either the merchantability of this software or the suitability   **
 ** of this software for any particular purpose.  It is provided "as **
 ** is" without express or implied warranty of any kind.             **
 **                                                                  **
 ** These notices must be retained in any copies of any part of this **
 ** documentation and/or software.                                   **
 **********************************************************************
 */