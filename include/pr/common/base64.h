//******************************************
// Base64
//  Copyright (c) March 2008 Paul Ryland
//******************************************

#pragma once

// Implements the Base64 Content-Transfer-Encoding standard described in RFC1113 (http://www.faqs.org/rfcs/rfc1113.html).
//
// This is the coding scheme used by MIME to allow binary data to be transferred by SMTP mail.
// Groups of 3 bytes from a binary stream are coded as groups of 4 bytes in a text stream.
// The input stream is 'padded' with zeros to create an input that is an even multiple of 3.
// A special character ('=') is used to denote padding so that the stream can be decoded back to its exact size.
//
// Example encoding:
//  The stream 'ABCD' is 32 bits long.  It is mapped as follows:
//
// ABCD
//   A (65)     B (66)     C (67)     D (68)   (None) (None)
//  01000001   01000010   01000011   01000100
//  16 (Q)  20 (U)  9 (J)   3 (D)    17 (R) 0 (A)  NA (=) NA (=)
//  010000  010100  001001  000011   010001 000000 000000 000000
// QUJDRA==
//
// Decoding is the process in reverse.  A 'decode' lookup table has been created to avoid string scans.
//
// Note: If using atl, you can use the follow instead
//
//  #include <atlenc.h>
//  inline unsigned int EncodeSize(int src_length) { return Base64EncodeGetRequiredLength(src_length, ATL_BASE64_FLAG_NOPAD|ATL_BASE64_FLAG_NOCRLF); }
//  inline unsigned int DecodeSize(int src_length) { return Base64DecodeGetRequiredLength(src_length); }
//  inline bool Encode(void const* src, int src_length, void* dst, int& dst_length)
//  {
//     return Base64Encode(static_cast<BYTE const*>(src), src_length, static_cast<CHAR*>(dst), &dst_length, ATL_BASE64_FLAG_NOPAD|ATL_BASE64_FLAG_NOCRLF) == TRUE;
//  }
//  inline bool Decode(void const* src, int src_length, void* dst, int& dst_length)
//  {
//     return Base64Decode(static_cast<CHAR const*>(src), src_length, static_cast<BYTE*>(dst), &dst_length) == TRUE;
//  }

namespace pr
{
	namespace base64
	{
		// Note:
		//        size != DecodeSize(EncodeSize(size))
		//  because encoding pads the data to a multiple of 4 bytes

		// Returns the size in bytes required to store 'src_length' bytes of data after encoding
		inline unsigned int EncodeSize(unsigned int src_length)
		{
			return ((src_length + 2) / 3) * 4;
		}

		// Returns the size in bytes required to store 'src_length' bytes of data after decoding
		inline unsigned int DecodeSize(unsigned int src_length)
		{
			return ((src_length + 3) / 4) * 3;
		}

		// Encode 'src' to base64 into 'dst'
		// The length of 'dst' must be >= 'EncodeSize(src_length)'
		inline void Encode(void const* src, unsigned int src_length, void* dst, unsigned int& dst_length)
		{
			static const char enc[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"; // Translation Table as described in RFC1113
			unsigned char const* in  = static_cast<unsigned char const*>(src);
			char* out = static_cast<char*>(dst);

			for (; src_length >= 3; src_length -= 3, in += 3) // encoding works in blocks of 3
			{
				*out++ = enc[((in[0]       ) >> 2)];
				*out++ = enc[((in[0] & 0x03) << 4)|((in[1] & 0xf0) >> 4)];
				*out++ = enc[((in[1] & 0x0f) << 2)|((in[2] & 0xc0) >> 6)];
				*out++ = enc[((in[2] & 0x3f)     )];
			}
			if (src_length != 0)
			{
				unsigned char s0 = in[0], s1 = (src_length>1)?in[1]:0, s2 = (src_length>2)?in[2]:0;
				*out++ = enc[((s0       ) >> 2)];
				*out++ = enc[((s0 & 0x03) << 4)|((s1 & 0xf0) >> 4)];
				*out++ = src_length >= 2 ? enc[((s1 & 0x0f) << 2)|((s2 & 0xc0) >> 6)] : '=';
				*out++ = '=';
			}
			dst_length = static_cast<unsigned int>(out - static_cast<char*>(dst));
		}

		// Decode base64 data 'src' into 'dst'
		// The length of 'dst' must be >= 'Base64_DecodeSize(src_length)'
		inline void Decode(void const* src, unsigned int src_length, void* dst, unsigned int& dst_length)
		{
			static unsigned char const dec_data[] =
			{
				62U, 0U, 0U, 0U,63U,52U,53U,54U,55U,56U,57U,58U,59U,60U,61U, 0U, 0U, 0U, 0U, 0U,
				 0U, 0U, 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,10U,11U,12U,13U,14U,15U,16U,17U,
				18U,19U,20U,21U,22U,23U,24U,25U, 0U, 0U, 0U, 0U, 0U, 0U,26U,27U,28U,29U,30U,31U,
				32U,33U,34U,35U,36U,37U,38U,39U,40U,41U,42U,43U,44U,45U,46U,47U,48U,49U,50U,51U
			};
			static unsigned char const* dec = dec_data - 43; // All references to dec should be in the range [43,122]
			struct is { static bool base64(char c) { return (c != '=') && ((c == 43) || (c >= 47 && c < 58) || (c >= 65 && c < 91) || (c >= 97 && c < 123)); } }; // isalnum, '+', or '/'
			
			unsigned char const* in = static_cast<unsigned char const*>(src);
			unsigned char* out = static_cast<unsigned char*>(dst);

			for (; src_length >= 4; src_length -= 4, in += 4) // decoding works in blocks of 4
			{
				unsigned int i; for (i = 0; i != 4 && is::base64(in[i]); ++i) {}
				if (i != 4) { src_length = i; break; }
				*out++ = ((dec[in[0]]      ) << 2) + ((dec[in[1]] & 0x30) >> 4);
				*out++ = ((dec[in[1]] & 0xf) << 4) + ((dec[in[2]] & 0x3c) >> 2);
				*out++ = ((dec[in[2]] & 0x3) << 6) + ((dec[in[3]]       )     );
			}

			// Validate the trailing partial block before indexing the translated table.
			unsigned int tail_length;
			for (tail_length = 0; tail_length != src_length && is::base64(in[tail_length]); ++tail_length) {}
			src_length = tail_length;
			if (src_length > 1) *out++ = ((dec[in[0]]      ) << 2) + ((dec[in[1]] & 0x30) >> 4);
			if (src_length > 2) *out++ = ((dec[in[1]] & 0xf) << 4) + ((dec[in[2]] & 0x3c) >> 2);
			if (src_length > 3) *out++ = ((dec[in[2]] & 0x3) << 6) + ((dec[in[3]]       )     );
			dst_length = static_cast<unsigned int>(out - static_cast<unsigned char*>(dst));
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(Base64Tests)
	{
		using namespace pr::base64;
		unsigned char src[1024], dst[1024];
		unsigned int src_length, dst_length;
		unsigned int len, dlen;

		// zero length data
		len = EncodeSize(0);                          PR_EXPECT(len        == 0U);
		Encode("", 0, dst, dst_length);               PR_EXPECT(dst_length == 0U);
		Decode(dst, dst_length, src, src_length);     PR_EXPECT(src_length == 0U);

		// one input char
		len = EncodeSize(1);                          PR_EXPECT(len == 4U);
		Encode("A", 1, dst, dst_length);              PR_EXPECT(dst_length == 4 && dst[0]=='Q' && dst[1]=='Q' && dst[2]=='=' && dst[3]=='=');
		Decode(dst, dst_length, src, src_length);     PR_EXPECT(src_length == 1 && src[0]=='A');

		// two chars
		len = EncodeSize(2);                          PR_EXPECT(len == 4U);
		Encode("AB", 2, dst, dst_length);             PR_EXPECT(dst_length == 4 && dst[0]=='Q' && dst[1]=='U' && dst[2]=='I' && dst[3]=='=');
		Decode(dst, dst_length, src, src_length);     PR_EXPECT(src_length == 2 && src[0]=='A' && src[1]=='B');

		// three chars
		len = EncodeSize(3);                          PR_EXPECT(len == 4U);
		Encode("ABC", 3, dst, dst_length);            PR_EXPECT(dst_length == 4 && dst[0]=='Q' && dst[1]=='U' && dst[2]=='J' && dst[3]=='D');
		Decode(dst, dst_length, src, src_length);     PR_EXPECT(src_length == 3 && src[0]=='A' && src[1]=='B' && src[2]=='C');

		// four chars
		len = EncodeSize(4);                          PR_EXPECT(len == 8U);
		Encode("ABCD", 4, dst, dst_length);           PR_EXPECT(dst_length == 8 && dst[0]=='Q' && dst[1]=='U' && dst[2]=='J' && dst[3]=='D' && dst[4]=='R' && dst[5]=='A');
		Decode(dst, dst_length, src, src_length);     PR_EXPECT(src_length == 4 && src[0]=='A' && src[1]=='B' && src[2]=='C' && src[3]=='D');

		// Tail decoding
		{
			unsigned char const tail[] = { 'Q', 'Q', '=', '=' };
			Decode(tail, sizeof(tail), src, src_length); PR_EXPECT(src_length == 1 && src[0] == 'A');
		}
		{
			unsigned char const tail[] = { 'Q', 'U', 'J' };
			Decode(tail, sizeof(tail), src, src_length); PR_EXPECT(src_length == 2 && src[0] == 'A' && src[1] == 'B');
		}
		{
			unsigned char const tail[] = { 'Q' };
			Decode(tail, sizeof(tail), src, src_length); PR_EXPECT(src_length == 0U);
		}
		{
			unsigned char const tail[] = { 'Q', 0x01 };
			Decode(tail, sizeof(tail), src, src_length); PR_EXPECT(src_length == 0U);
		}
		{
			unsigned char const tail[] = { 'Q', '{' };
			Decode(tail, sizeof(tail), src, src_length); PR_EXPECT(src_length == 0U);
		}

		// All bytes from 0 to ff
		unsigned char sbuf[256]; for (int i = 0; i != 256; ++i) sbuf[i] = (unsigned char)i;
		unsigned char dbuf[] =	"AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIj"
								"JCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+P0BBQkNERUZH"
								"SElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiY2RlZmdoaWpr"
								"bG1ub3BxcnN0dXZ3eHl6e3x9fn+AgYKDhIWGh4iJiouMjY6P"
								"kJGSk5SVlpeYmZqbnJ2en6ChoqOkpaanqKmqq6ytrq+wsbKz"
								"tLW2t7i5uru8vb6/wMHCw8TFxsfIycrLzM3Oz9DR0tPU1dbX"
								"2Nna29zd3t/g4eLj5OXm5+jp6uvs7e7v8PHy8/T19vf4+fr7"
								"/P3+/w==";
		unsigned int ssize = sizeof(sbuf), dsize = sizeof(dbuf) - 1;

		len = EncodeSize(ssize);
		PR_EXPECT(len == dsize);
		
		Encode(sbuf, ssize, dst, dst_length);
		PR_EXPECT(dst_length == dsize);
		for (size_t i = 0; i != dsize; ++i)
			PR_EXPECT(dst[i] == dbuf[i]);
		
		dlen = DecodeSize(len);
		PR_EXPECT(dlen < sizeof(src));
		Decode(dst, dst_length, src, src_length);
		PR_EXPECT(src_length == ssize);
		for (size_t i = 0; i != ssize; ++i)
			PR_EXPECT(src[i] == sbuf[i]);

		// Random binary data
		for (size_t i = 0; i != ssize; ++i)
			sbuf[i] = (unsigned char)(::rand() & 0xFF);

		Encode(sbuf, ssize, dst, dst_length);
		Decode(dst, dst_length, src, src_length);
		PR_EXPECT(src_length == ssize);
		for (size_t i = 0; i != ssize; ++i)
			PR_EXPECT(src[i] == sbuf[i]);
	}
}
#endif
