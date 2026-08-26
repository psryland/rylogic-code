//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "glyph_cache.h"

namespace pr::view3d::ui
{
	bool GlyphAtlasPage::TryPlace(std::uint32_t width_px, std::uint32_t height_px, std::uint32_t& out_x, std::uint32_t& out_y)
	{
		if (width_px > m_dim_px || height_px > m_dim_px)
			return false; // a single glyph larger than a whole page can never be packed

		// Try to continue the current shelf first; only start a new shelf below it when the
		// glyph does not fit in the remaining width, which keeps horizontal packing tight.
		if (m_cursor_x + width_px <= m_dim_px && m_cursor_y + height_px <= m_dim_px)
		{
			out_x = m_cursor_x;
			out_y = m_cursor_y;
			m_cursor_x += width_px;
			m_shelf_h = std::max(m_shelf_h, height_px);
			return true;
		}

		auto next_shelf_y = m_cursor_y + m_shelf_h;
		if (next_shelf_y + height_px > m_dim_px || width_px > m_dim_px)
			return false; // this page has no room left for a glyph of this height, regardless of x

		out_x = 0;
		out_y = next_shelf_y;
		m_cursor_x = width_px;
		m_cursor_y = next_shelf_y;
		m_shelf_h = height_px;
		return true;
	}

	GlyphCache::GlyphCache(std::uint32_t page_dim_px, std::uint32_t max_pages, std::uint64_t max_bytes)
		: m_page_dim_px(page_dim_px)
		, m_bytes_per_page(page_dim_px * page_dim_px)
	{
		// The effective page limit is the tighter of the caller's explicit page count and however
		// many pages fit within the byte budget, so a caller only needs to reason about one of the
		// two Config fields at a time without either one silently overriding the other.
		auto pages_by_bytes = m_bytes_per_page != 0 ? static_cast<std::uint32_t>(max_bytes / m_bytes_per_page) : 0u;
		m_max_pages = std::min(max_pages, pages_by_bytes);
	}

	EStatus GlyphCache::Acquire(GlyphKey const& key, std::uint32_t width_px, std::uint32_t height_px, std::int32_t origin_x_px, std::int32_t origin_y_px, GlyphSlot& out_slot, bool& out_new_page)
	{
		out_new_page = false;

		if (auto it = m_slots.find(key); it != m_slots.end())
		{
			out_slot = it->second;
			return EStatus::Success;
		}

		// Try every existing page in order before allocating a new one, so earlier pages fill up
		// completely rather than fragmenting placements arbitrarily across pages.
		for (auto page_index = std::size_t{}; page_index != m_pages.size(); ++page_index)
		{
			std::uint32_t x = 0, y = 0;
			if (!m_pages[page_index].TryPlace(width_px, height_px, x, y))
				continue;

			auto slot = GlyphSlot{
				.page = static_cast<std::uint32_t>(page_index),
				.pixel_x = x,
				.pixel_y = y,
				.pixel_w = width_px,
				.pixel_h = height_px,
				.origin_x_px = origin_x_px,
				.origin_y_px = origin_y_px,
				.uv0_x = static_cast<float>(x) / static_cast<float>(m_page_dim_px),
				.uv0_y = static_cast<float>(y) / static_cast<float>(m_page_dim_px),
				.uv1_x = static_cast<float>(x + width_px) / static_cast<float>(m_page_dim_px),
				.uv1_y = static_cast<float>(y + height_px) / static_cast<float>(m_page_dim_px),
			};
			m_slots.emplace(key, slot);
			out_slot = slot;
			return EStatus::Success;
		}

		if (m_pages.size() >= m_max_pages)
			return EStatus::ResourceLimit;

		// Reject a glyph that cannot fit even a fresh, entirely empty page before allocating one,
		// so a pathologically oversized glyph fails without silently consuming a page of budget.
		if (width_px > m_page_dim_px || height_px > m_page_dim_px)
			return EStatus::ResourceLimit;

		m_pages.emplace_back(m_page_dim_px);
		out_new_page = true;

		std::uint32_t x = 0, y = 0;
		if (!m_pages.back().TryPlace(width_px, height_px, x, y))
			return EStatus::ResourceLimit; // the glyph does not fit even on a fresh empty page

		auto slot = GlyphSlot{
			.page = static_cast<std::uint32_t>(m_pages.size() - 1),
			.pixel_x = x,
			.pixel_y = y,
			.pixel_w = width_px,
			.pixel_h = height_px,
			.origin_x_px = origin_x_px,
			.origin_y_px = origin_y_px,
			.uv0_x = static_cast<float>(x) / static_cast<float>(m_page_dim_px),
			.uv0_y = static_cast<float>(y) / static_cast<float>(m_page_dim_px),
			.uv1_x = static_cast<float>(x + width_px) / static_cast<float>(m_page_dim_px),
			.uv1_y = static_cast<float>(y + height_px) / static_cast<float>(m_page_dim_px),
		};
		m_slots.emplace(key, slot);
		out_slot = slot;
		return EStatus::Success;
	}

	void GlyphCache::Reset()
	{
		m_pages.clear();
		m_slots.clear();
	}

	GlyphSlot const* GlyphCache::Find(GlyphKey const& key) const
	{
		auto it = m_slots.find(key);
		return it != m_slots.end() ? &it->second : nullptr;
	}
}
