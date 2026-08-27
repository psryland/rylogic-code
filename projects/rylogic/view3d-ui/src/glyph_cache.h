//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// A bounded, deterministic CPU-side allocator for glyph bitmap placements within a fixed-size grid
// of fixed-size atlas pages (implementation-plan.md section 9.2, Config::max_glyph_cache_bytes and
// Config::max_glyph_cache_pages). This class only tracks placement bookkeeping - it never touches
// GPU resources or DirectWrite; the M3 renderer owns the actual atlas texture(s) and copies glyph
// coverage bitmaps into the pixel rectangles this class hands out. Keeping this logic GPU-free
// makes it directly unit-testable without a device.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

namespace pr::view3d::ui
{
	// The placement of one previously-rasterized glyph within the atlas: which page, and the pixel
	// rectangle on that page holding its coverage bitmap. uv0/uv1 are that rectangle expressed as
	// normalized page-relative texture coordinates, ready for the renderer's glyph shader.
	// origin_x_px/origin_y_px carry the bitmap's top-left corner offset from the glyph's baseline
	// origin (TextShaper::GlyphBitmap::origin_x_px/origin_y_px at the moment it was rasterized),
	// since that offset is only produced once, by Rasterize(), and must still be available to
	// position this glyph on screen on every later frame that only calls Find().
	struct GlyphSlot
	{
		std::uint32_t page;
		std::uint32_t pixel_x;
		std::uint32_t pixel_y;
		std::uint32_t pixel_w;
		std::uint32_t pixel_h;
		std::int32_t origin_x_px;
		std::int32_t origin_y_px;
		float uv0_x;
		float uv0_y;
		float uv1_x;
		float uv1_y;
	};

	// Identifies one rasterized glyph: a specific font face/size rendering of a specific glyph
	// index. Distinct font families/sizes never share cache entries even if a glyph index happens
	// to collide numerically, because the id is only unique within a single font face's own table.
	struct GlyphKey
	{
		std::uint64_t font_key; // a stable hash of (family, size_dip); see TextShaper::FontKey
		std::uint32_t glyph_index;

		friend bool operator==(GlyphKey const& lhs, GlyphKey const& rhs)
		{
			return lhs.font_key == rhs.font_key && lhs.glyph_index == rhs.glyph_index;
		}
	};
	struct GlyphKeyHash
	{
		std::size_t operator()(GlyphKey const& key) const
		{
			return std::hash<std::uint64_t>{}(key.font_key) ^ (std::hash<std::uint32_t>{}(key.glyph_index) << 1);
		}
	};

	// A fixed-size square atlas page and a simple left-to-right, top-to-bottom shelf packer over
	// it. Shelf packing is the right trade-off here: glyph bitmaps at a handful of UI font sizes
	// are all a similar height, so the wasted space of a naive shelf packer is small, and its
	// placement logic is trivial to reason about and unit-test compared to a general bin packer.
	class GlyphAtlasPage
	{
		std::uint32_t m_dim_px;
		std::uint32_t m_cursor_x = 0;
		std::uint32_t m_cursor_y = 0;
		std::uint32_t m_shelf_h = 0;

	public:
		explicit GlyphAtlasPage(std::uint32_t dim_px)
			: m_dim_px(dim_px)
		{
		}

		// Reserve a width_px x height_px rectangle on this page, advancing the shelf cursor.
		// Returns false (without mutating any state) if the glyph does not fit on a fresh shelf of
		// this page at all, so the caller can move on to allocating a new page.
		bool TryPlace(std::uint32_t width_px, std::uint32_t height_px, std::uint32_t& out_x, std::uint32_t& out_y);
	};

	// Bounded glyph cache: Acquire() either returns an already-cached slot, or reserves-and-packs a
	// new one, up to Config::max_glyph_cache_pages pages of Config-derived pixel dimensions; once
	// that limit is reached, Acquire() returns EStatus::ResourceLimit rather than growing the
	// atlas, matching the "never silently drop, never grow unbounded" ABI contract. This class only
	// hands out placement rectangles - the caller (the renderer) is responsible for rasterizing the
	// glyph's coverage bitmap and copying it into the returned rectangle on whichever page it maps
	// to a real GPU texture.
	class GlyphCache
	{
		std::uint32_t m_page_dim_px;
		std::uint32_t m_max_pages;
		std::uint32_t m_bytes_per_page;
		std::vector<GlyphAtlasPage> m_pages;
		std::unordered_map<GlyphKey, GlyphSlot, GlyphKeyHash> m_slots;

	public:
		// 'page_dim_px' is the fixed square page size in pixels (single-channel coverage, so
		// bytes-per-page is page_dim_px * page_dim_px); 'max_pages' and 'max_bytes' come directly
		// from Config and jointly bound this cache's memory (the tighter of the two limits wins).
		GlyphCache(std::uint32_t page_dim_px, std::uint32_t max_pages, std::uint64_t max_bytes);

		// Look up or newly place the slot for 'key'. 'width_px'/'height_px' are the glyph's coverage
		// bitmap dimensions and 'origin_x_px'/'origin_y_px' its baseline-relative offset (already
		// known to the caller from a prior Rasterize() call; this class never rasterizes anything
		// itself). Returns Success with 'out_slot' populated and 'out_new_page' set to whichever
		// page index was freshly allocated to satisfy this request (or unchanged if none was, i.e.
		// an existing page/slot already covered it), or ResourceLimit if no existing page has room
		// and the page-count/byte-budget limit has already been reached.
		EStatus Acquire(GlyphKey const& key, std::uint32_t width_px, std::uint32_t height_px, std::int32_t origin_x_px, std::int32_t origin_y_px, GlyphSlot& out_slot, bool& out_new_page);

		// Returns the slot already placed for 'key', or nullptr if it has never been placed (the
		// caller must then rasterize the glyph and call Acquire). Lets a caller skip re-rasterizing
		// an already-resident glyph purely to learn its dimensions, unlike Acquire which always
		// requires them up front.
		GlyphSlot const* Find(GlyphKey const& key) const;

		// Discards every cached slot and page, e.g. on device loss/renderer reset.
		void Reset();

		std::size_t PageCount() const
		{
			return m_pages.size();
		}
		std::uint32_t PageDim() const
		{
			return m_page_dim_px;
		}
		std::uint32_t MaxPages() const
		{
			return m_max_pages;
		}
		std::uint64_t BytesUsed() const
		{
			return static_cast<std::uint64_t>(m_pages.size()) * m_bytes_per_page;
		}
	};
}
