//*****************************************************************************************
// Space Invaders
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************

#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <type_traits>
#include <cassert>

namespace pr::onebit
{
	// LSB mask. e.g. i=3 => 0b00000111
	template <typename TWord = uint8_t>
	constexpr static TWord MaskLo(int i)
	{
		assert(i < 8*sizeof(TWord));
		return static_cast<TWord>((1ULL << i) - 1);
	}

	// MSB mask. e.g. i=3 => 0b11111000
	template <typename TWord = uint8_t>
	constexpr static TWord MaskHi(int i)
	{
		assert(i < 8*sizeof(TWord));
		return static_cast<TWord>(~0ULL << i);
	}

	// Convert a Y coordinate to a block index
	template <typename TWord = uint8_t>
	constexpr static int BlockIndex(int y)
	{
		// Handle negative values correctly. e.g. [-WordSize,0) == block -1, [0, WordSize) = block 0
		constexpr int WordSize = 8*sizeof(TWord);
		return y >= 0 ? (y / WordSize) : (-~y / WordSize - 1);
	}

	// Viewport rectangle
	struct Viewport
	{
		int l, t, r, b;
	};

	// Return data for a clipped rectangle
	struct ClippedQuad
	{
		int x, y;         // The top/left corner in the quad that is on screen
		int w, h;         // The width/height of the quad that is on screen
		int scn0, scn1;   // The inclusive range of blocks spanned on the screen
		int quad0, quad1; // The inclusive range of blocks spanned in the quad
		int yofs;         // The relative offset between the quad and the screen (mod block height)
	};

	// Clip a rectangle to a viewport
	template <typename TWord = uint8_t>
	bool ClipQuadToViewport(int X, int Y, int dx, int dy, Viewport const& vp, ClippedQuad& clip)
	{
		// The region 'clip.x/y clip.w/h' represents the area within the quad that
		// is still visible (i.e. in quad space). The screen coordinates of the top/left
		// corner of the visible area are not returned, but can be inferred as the viewport
		// top/left if clip.x/y is not (0,0). Equivalently, as (X + clip.x, Y + clip.y)
		constexpr int WordSize = 8*sizeof(TWord);
		
		// Clip the quad to the the viewport bounds
		clip.x = 0;
		clip.y = 0;
		clip.w = dx;
		clip.h = dy;
		if (X + dx > vp.r) { clip.w  = vp.r - X; }
		if (Y + dy > vp.b) { clip.h  = vp.b - Y; }
		if (X      < vp.l) { clip.w -= vp.l - X; clip.x += vp.l - X; }
		if (Y      < vp.t) { clip.h -= vp.t - Y; clip.y += vp.t - Y; }
		if (clip.w <= 0 || clip.h <= 0)
			return false;

		// Offset from a row boundary to the top edge of the quad
		clip.yofs = Y - BlockIndex<TWord>(Y) * WordSize;

		// The y position of the visible area of the quad
		int y = Y + clip.y;

		// The (inclusive) block range on the screen spanned by the quad
		clip.scn0 = BlockIndex<TWord>(y);
		clip.scn1 = BlockIndex<TWord>(y + clip.h - 1);

		// The (inclusive) block range in the quad spanned by the screen
		clip.quad0 = BlockIndex<TWord>(clip.y);
		clip.quad1 = BlockIndex<TWord>(clip.y + clip.h - 1);

		return true;
	}

	// A 1-Bit readonly bitmap
	template <typename TWord = uint8_t>
	struct BitmapR
	{
		// Notes:
		//  - The required memory layout of the bitmap is:
		//    +--+--+--+--+--+
		//    |W1|W2|W3|W4|W5|
		//    +--+--+--+--+--+
		//    |W6|W7|W8|W9| ...
		//    +--+--+--+--+
		//    ...
		//    where a 'Word' represents a '1 x WordSize' column of pixels (called a 'Block')
		//  - The LSB of 'W1' is the top/left corner of the image.
		//  - A BitmapR is readonly, so does not need to contain its data. The 'R' stands for readonly, or reference,.. can't decide
		//  - A Bitmap contains a local buffer so it can be edited.
		//  - Drawing/Accessing out-of-bounds is silently ignored.
		//  - It's too complicated allowing mixed word size bitmaps. Only use bitmaps of the same word size together

		using Word = TWord;
		static constexpr int WordSize = sizeof(Word) * 8;

		Word const* m_data;
		int m_dimx, m_dimy;
		int m_stride;

		BitmapR(Word const* data, int dimx, int dimy, int stride = 0)
			: m_data(data)
			, m_dimx(dimx)
			, m_dimy(dimy)
			, m_stride(stride ? stride : dimx)
		{}

		// Access a block of pixel data
		Word Block(int b, int x) const
		{
			assert(b >= 0 && b < BlockIndex<Word>(m_dimy + WordSize - 1));
			assert(x >= 0 && x < m_dimx);
			return m_data[b * m_stride + x];
		}

		// Pixel state at (x,y)
		bool operator()(int x, int y) const
		{
			if (x < 0 || x >= m_dimx) return false;
			if (y < 0 || y >= m_dimy) return false;
			return Block(BlockIndex<Word>(y), x) & (1 << (y % WordSize));
		}

		// Write the bitmap to a file
		void DumpToFile(std::filesystem::path const& filepath) const
		{
			std::ofstream out(filepath);
			for (int y = 0; y != m_dimy; ++y)
			{
				for (int x = 0; x != m_dimx; ++x)
				{
					if ((*this)(x, y))
						out.write("#", 1);
					else
						out.write(".", 1);
				}
				out.write("\n", 1);
			}
		}
	};

	// A mutable 1-Bit bitmap
	template <int DimX, int DimY, typename TWord = uint8_t>
	struct Bitmap :BitmapR<TWord>
	{
		using BitmapR = BitmapR<TWord>;
		using Word = typename BitmapR::Word;
		static constexpr int WordSize = sizeof(Word) * 8;

		// The capacity of the bitmap
		constexpr static int DimX = DimX;
		constexpr static int DimY = DimY;
		static_assert(DimX > 0 && DimY > 0);

		// The height of the bitmap in blocks
		constexpr static int BlockHeight = (DimY + WordSize - 1) / WordSize;

		// The bitmap data
		Word m_buf[DimX * BlockHeight];

		Bitmap() noexcept
			:BitmapR(&m_buf[0], DimX, DimY, DimX)
			,m_buf()
		{}
		Bitmap(Word const* data, int dimx, int dimy, int stride = 0) noexcept
			:Bitmap()
		{
			Init(data, dimx, dimy, stride);
		}
		Bitmap(Bitmap const& rhs) noexcept
			:Bitmap(rhs.m_data, rhs.m_dimx, rhs.m_dimy, rhs.m_stride)
		{}
		Bitmap(BitmapR const& rhs) noexcept
			:Bitmap(rhs.m_data, rhs.m_dimx, rhs.m_dimy, rhs.m_stride)
		{}
		Bitmap& operator =(Bitmap const& rhs) noexcept
		{
			if (this == &rhs) return *this;
			this->m_data = &m_buf[0];
			this->m_dimx = rhs.m_dimx;
			this->m_dimy = rhs.m_dimy;
			this->m_stride = rhs.m_stride;
			memcpy(&m_buf[0], &rhs.m_buf[0], sizeof(m_buf));
			return *this;
		}

		// Populate this bitmap with the given data
		void Init(Word const* data, int dimx, int dimy, int stride = 0)
		{
			stride = stride ? stride : dimx;
			assert(dimx <= stride);
			assert(dimx >= 0 && dimx <= DimX);
			assert(dimy >= 0 && dimy <= DimY);

			if (stride == DimX)
			{
				memcpy(&m_buf[0], &data[0], sizeof(m_buf));
			}
			else
			{
				for (int b = 0; b != BlockHeight; ++b)
					memcpy(&m_buf[b * DimX], &data[b * stride], dimx * sizeof(Word));
			}
			this->m_dimx = dimx;
			this->m_dimy = dimy;
			this->m_stride = stride;
		}

		// Write access to a block of image data
		Word Block(int b, int x) const
		{
			return BitmapR::Block(b, x);
		}
		Word& Block(int b, int x)
		{
			assert(b >= 0 && b < BlockHeight);
			assert(x >= 0 && x < this->m_dimx);
			return m_buf[b * DimX + x];
		}

		// Clear the image
		void Clear(int value = 0)
		{
			memset(&m_buf[0], value, sizeof(m_buf));
		}
		void Clear(int X, int Y, int W, int H, Word value = 0)
		{
			// Clip the clear rectangle to the image
			ClippedQuad clip;
			if (!ClipQuadToViewport(X, Y, W, H, {0, 0, DimX, DimY}, clip))
				return;

			// Shift (X,Y) to the top/left of the visible area
			X += clip.x;
			Y += clip.y;

			// Fill the rectange with 'value'
			auto bbeg = BlockIndex<Word>(Y + 0);
			auto bend = BlockIndex<Word>(Y + H);
			for (int b = bbeg; b <= bend; ++b)
			{
				auto mask = value;
				if (b == bbeg) mask |= MaskLo<Word>((Y + 0) % WordSize);
				if (b == bend) mask |= MaskHi<Word>((Y + H) % WordSize);
				for (int x = X; x != X + W; ++x)
					Block(b, x) &= mask;
			}
		}

		// Draw an image into this image
		template <typename Image>
		void Draw(Image const& img, int X, int Y)
		{
			Combine(*this, img, X, Y, [](auto& lhs, auto const&, int b, int x, Word word, Word)
			{
				// Combine bits with OR to print 'img' into this bitmap
				lhs.Block(b, x) |= word;
				return false;
			});
		}
	};

	// Combine 'lhs' and 'rhs' at (X,Y) relative to 'lhs' using 'op'
	// Op should be callable with signature:
	//   bool op(ImageL& lhs, ImageR& rhs, int block_index, int column_index, Word bits, Word mask);
	//   'lhs' and 'rhs' are writable if Combine is called with non-const parameters.
	//   'block_index' and 'column_index' are the coordinates in 'lhs'
	//   'bits' is the bits from 'rhs' to write at the coordinates in 'lhs'.
	//   'mask' is a bit mask of the valid bits in 'bits'
	// Returns true if 'op' returns true
	template <typename BitmapL, typename BitmapR, typename Op>
	bool Combine(BitmapL&& lhs, BitmapR&& rhs, int X, int Y, Op op)
	{
		using BmpL = std::decay_t<BitmapL>;
		using BmpR = std::decay_t<BitmapR>;
		using Word = typename BmpL::Word;
		constexpr int WordSize = sizeof(Word) * 8;
		static_assert(std::is_same_v<typename BmpL::Word, typename BmpR::Word>, "Bitmaps must have the same Word size");

		// Clip 'rhs' to the bounds of 'lhs'
		ClippedQuad clip;
		if (!ClipQuadToViewport<Word>(X, Y, rhs.m_dimx, rhs.m_dimy, {0, 0, lhs.m_dimx, lhs.m_dimy}, clip))
			return false;

		// Shift (X,Y) to the top/left of the visible area
		X += clip.x;
		Y += clip.y;

		// Loop over the blocks that span 'rhs'
		for (auto b = clip.scn0; b <= clip.scn1; ++b)
		{
			// Get the minimum (virtual) block index in 'rhs' that overlaps blk 'b' (can be negative)
			auto B = BlockIndex<Word>(b * WordSize - (Y - clip.y));

			// Create a mask for the bits to write to in this block
			Word mask = static_cast<Word>(~0U);
			if (Y        > (b+0)*WordSize) mask &= MaskHi<Word>(Y        - b*8);
			if (Y+clip.h < (b+1)*WordSize) mask &= MaskLo<Word>(Y+clip.h - b*8);

			// Loop over the horizontal range in 'rhs'
			for (int x = clip.x, xend = clip.x + clip.w; x != xend; ++x)
			{
				Word word = 0;
				if (B >= clip.quad0)
				{
					auto bits = rhs.Block(B, x);
					if (clip.yofs != 0) bits >>= (WordSize - clip.yofs); // shift down so that 'yofs' bits remain
					word |= bits;
				}
				if (B < clip.quad1 && clip.yofs != 0)
				{
					auto bits = rhs.Block(B + 1, x);
					bits <<= clip.yofs; // shift up so that (WordSize-yofs) bits remain
					word |= bits;
				}

				// Apply the combine operation
				if (op(lhs, rhs, b, X + (x - clip.x), word, mask))
					return true;
			}
		}

		return false;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include <cstdint>
namespace pr::onebit
{
	PRUnitTest(OneBitTests, Quick)
	{
		uint8_t const cross_data[] =
		{
			0x01, 0x02, //        #      # 
			0x04, 0x08, //      #      #   
			0x10, 0x20, //    #      #     
			0x40, 0x80, //  #      #       
			0x40, 0x20, //  #        #     
			0x10, 0x08, //    #        #   
			0x04, 0x02, //      #        # 
			0x01, 0x40, //        # #      
			0x20, 0x10, //   #        #    
			0x08, 0x04, //     #        #  
			0x02, 0x01, //       #        #
			0x00, 0x01, //                #
			0x02, 0x04, //       #      #  
			0x08, 0x10, //     #      #    
			0x20, 0x40, //   #      #      
		};
		BitmapR<uint8_t> cross(&cross_data[0], 15, 15);
		uint8_t const small_spaceship_data[] =
		{
			0xF0,
			0x60,
			0x70,
			0xF8,
			0xFF,
			0xF8,
			0x70,
			0x60,
			0xF0,
		};
		BitmapR<uint8_t> small_spaceship(&small_spaceship_data[0], 9, 8);
		uint8_t const alien_data[] =
		{
			0x8C, // #   ##  
			0x5E, //  # #### 
			0xBB, // # ### ##
			0x5F, //  # #####
			0x5F, //  # #####
			0xBB, // # ### ##
			0x5E, //  # #### 
			0x8C, // #   ##  
		};
		BitmapR<uint8_t> alien(&alien_data[0], 8, 4);
		uint8_t const big_spaceship_data[] =
		{
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0xE0,
			0xFE, 0xFF, 0xFF, 0xFE,
			0xE0, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x80, 0xF0, 0x80, 0x00,
			0x00, 0x00, 0x80, 0xC0,
			0xE0, 0xF0, 0xF8, 0x7F,
			0xBF, 0xDF, 0xDF, 0xBF,
			0x7F, 0xF8, 0xF0, 0xE0,
			0xC0, 0x80, 0x00, 0x00,
			0x00, 0x80, 0xF0, 0x80,
			0xFF, 0xFF, 0xFF, 0xFC,
			0xFE, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFE,
			0xFC, 0xFF, 0xFF, 0xFF,
			0x07, 0x0F, 0x07, 0x01,
			0x01, 0x03, 0x03, 0x03,
			0x03, 0x03, 0x03, 0x03,
			0x01, 0x01, 0x01, 0x01,
			0x03, 0x03, 0x03, 0x03,
			0x03, 0x03, 0x03, 0x01,
			0x01, 0x07, 0x0F, 0x07,
		};
		BitmapR<uint8_t> big_spaceship(&big_spaceship_data[0], 28, 28);

		{// Clipping
			Bitmap<64, 64, uint8_t> screen;
			screen.Draw(cross, 1, +2);
			screen.Draw(cross, 17, -2);
			screen.Draw(cross, 33, -10);
			screen.Draw(cross, 1, 58);
			screen.Draw(cross, 17, 54);
			screen.Draw(cross, 33, 62);
			screen.Draw(small_spaceship, 50, 10);
			screen.Draw(big_spaceship, 30, 30);
			screen.Draw(alien, 20, 40);
			//screen.DumpToFile("P:\\dump\\sprite.txt");
		}
		{// Collision
			Bitmap<64, 64, uint8_t> screen;
			Bitmap<16, 16, uint8_t> s0(cross);
			Bitmap<16, 16, uint8_t> s1(cross);
			screen.Draw(s0, 10, 10);
			screen.Draw(s1, 20, 18);
			//screen.DumpToFile("P:\\dump\\sprite.txt");

			bool hit;

			hit = Combine(s0, s1, 10, 10, [](auto& lhs, auto&, int b, int x, auto word, auto mask)
			{
				return (lhs.Block(b, x) & word & mask) != 0;
			});
			PR_EXPECT(hit);

			hit = Combine(s0, s1, 10, 8, [](auto& lhs, auto&, int b, int x, auto word, auto mask)
			{
				return (lhs.Block(b, x) & word & mask) != 0;
			});
			PR_EXPECT(!hit);
		}
	}
}
#endif
