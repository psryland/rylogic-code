//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// DirectWrite-backed text layout, glyph shaping, hit testing and CPU coverage rasterization.
//
// Every text question this module can ask - what glyphs to draw and where, which character a
// pointer landed on, where the caret is, which rectangles cover a selection or an IME composition -
// is answered from one IDWriteTextLayout built from the same string, so rendering, caret placement,
// selection highlighting and the semantic text ranges can never disagree with each other. Using
// IDWriteTextLayout also means bidirectional (RTL/mixed) text, complex-script shaping, ligatures
// and per-character font fallback come from DirectWrite's own implementation rather than a
// home-grown approximation.
//
// Font resolution policy (see ResolveFont): a requested family name must exist in the system font
// collection. An unknown family is reported as EStatus::MissingAsset and is never silently replaced
// with an unrelated face. Only an *unspecified* family (the empty string, i.e. a control with no
// font resource) selects the built-in "Segoe UI" fallback, and only when that family is itself
// present. Layout and render results are therefore deterministic across machines that have the
// exact same font resource installed; DirectWrite's per-character fallback for code points the
// resolved face does not cover is a system-family substitution and is not claimed to be
// pixel-identical across machines.
//
// No D3D11-on-12 or Direct2D device/queue is created or touched anywhere in this class;
// IDWriteGlyphRunAnalysis produces its coverage bitmaps purely on the CPU.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

namespace pr::view3d::ui
{
	// One shaped glyph ready for layout: which glyph index to rasterize/draw, which resolved face
	// (at which size) it belongs to, and its layout-relative origin/advance, all in DIPs. Because
	// DirectWrite may resolve different runs of one string to different faces, 'font_key' - not the
	// requested family name - is what identifies the face a glyph index is valid for.
	// 'bidi_level' is the Unicode embedding level DirectWrite resolved for the run this glyph came
	// from; an odd level means the run reads right-to-left, which is what decides the direction the
	// pen advances within that run.
	struct ShapedGlyph
	{
		std::uint64_t font_key;
		std::uint32_t glyph_index;
		std::uint32_t bidi_level;
		float advance_x;
		float origin_x;
		float origin_y;
	};

	// A rasterized glyph's single-channel (alpha-only) coverage bitmap, in physical (device) pixels,
	// plus the offset of its top-left corner relative to the glyph's own origin - DirectWrite's
	// alpha-texture bounds are not guaranteed to start exactly at the origin, so this offset must be
	// applied when the renderer positions the bitmap.
	struct GlyphBitmap
	{
		std::uint32_t width_px;
		std::uint32_t height_px;
		std::int32_t origin_x_px;
		std::int32_t origin_y_px;
		std::vector<std::uint8_t> alpha; // width_px * height_px bytes, row-major, 0 = transparent
	};

	// Where a caret sits within a laid-out string, in DIPs relative to the layout origin. 'height'
	// is the line height at that position, so the caller can draw a caret without re-deriving font
	// metrics, and 'is_rtl' reports whether the text at that position runs right-to-left (which is
	// what makes a caret in bidirectional text land visually correctly).
	struct CaretGeometry
	{
		float x;
		float y;
		float height;
		std::int32_t is_rtl;
	};

	// The DIP y-coordinate of a text run's layout box top inside a control of height 'bounds_h',
	// given the resolved font's ascent and descent. Everything that positions text or text
	// decorations - glyph baselines, selection fills, composition underlines, the caret, and the
	// caret rectangle the ABI reports back to the application - must go through this one formula so
	// they cannot drift apart when the font or the control height changes.
	inline float TextOriginYDip(float bounds_y, float bounds_h, float ascent_dip, float descent_dip)
	{
		return bounds_y + (bounds_h - (ascent_dip + descent_dip)) * 0.5f;
	}

	class TextShaper
	{
		// One resolved font face plus the DirectWrite text format built from it, cached per
		// (requested family, size) so repeated layout of the same control costs no COM creation.
		struct FontEntry
		{
			Microsoft::WRL::ComPtr<IDWriteFontFace> face;
			Microsoft::WRL::ComPtr<IDWriteTextFormat> format;
			float size_dip;
			float ascent_dip;
			float descent_dip;
		};

		// One face actually used by a shaped run, registered under its own content-derived key so
		// Rasterize can find it again from nothing but a ShapedGlyph::font_key.
		struct RunFace
		{
			Microsoft::WRL::ComPtr<IDWriteFontFace> face;
			float size_dip;
		};

		// One cached IDWriteTextLayout together with the offset maps that translate between this
		// module's UTF-8 byte offsets and DirectWrite's UTF-16 code-unit positions.
		struct LayoutEntry
		{
			std::string text;
			std::string family;
			float size_dip;
			Microsoft::WRL::ComPtr<IDWriteTextLayout> layout;
			std::vector<std::uint32_t> utf16_to_utf8; // one entry per UTF-16 unit, plus a terminating total
			std::vector<std::uint32_t> utf8_to_utf16; // one entry per UTF-8 byte, plus a terminating total
		};

		Microsoft::WRL::ComPtr<IDWriteFactory> m_factory;
		std::unordered_map<std::uint64_t, FontEntry> m_fonts;
		std::vector<std::uint64_t> m_font_mru;      // m_fonts keys, most recently resolved first
		std::unordered_map<std::uint64_t, RunFace> m_run_faces;
		std::vector<std::uint64_t> m_run_face_mru;  // m_run_faces keys, most recently registered first
		std::vector<LayoutEntry> m_layouts;         // bounded most-recently-used cache

	public:
		// Hard upper bounds on the two content-keyed caches this class owns. Neither can grow with
		// the number of distinct fonts or fallback faces a document uses: the least recently used
		// entry is evicted once the bound is reached. Eviction is always safe because both keys are
		// derived purely from content, so a later Shape() call re-creates the identical entry.
		static constexpr std::size_t MaxCachedFonts = 16;
		static constexpr std::size_t MaxCachedRunFaces = 32;

		// Creates the process-local IDWriteFactory; throws pr::view3d::ui::EngineException(EStatus::
		// InternalError, ...) if DirectWrite is unavailable, which the caller (Renderer) surfaces
		// through its own construction failure path rather than partially constructing.
		TextShaper();

		TextShaper(TextShaper const&) = delete;
		TextShaper& operator=(TextShaper const&) = delete;
		TextShaper(TextShaper&&) = default;
		TextShaper& operator=(TextShaper&&) = default;

		// A stable hash of (family, size_dip) used to key this class's own resolved-font cache.
		// Glyph-atlas keys use ShapedGlyph::font_key instead, because one string can be shaped
		// across several faces.
		static std::uint64_t FontKey(std::string_view family, float size_dip);

		// Shapes 'utf8_text' at 'family'/'size_dip' into 'out_glyphs' in visual (left-to-right on
		// screen) order, replacing its prior contents; returns the total layout width in DIPs.
		// Throws EngineException(MissingAsset) when 'family' names a font that is not installed.
		float Shape(std::string_view family, float size_dip, float dpi_scale, std::string_view utf8_text, std::vector<ShapedGlyph>& out_glyphs);

		// Rasterizes a single glyph index previously produced by Shape() for the face identified by
		// 'font_key', returning its coverage bitmap in physical pixels. Throws
		// EngineException(InvalidArgument) for a key no Shape() call has registered.
		GlyphBitmap Rasterize(std::uint64_t font_key, float dpi_scale, std::uint32_t glyph_index);

		// Ascent/descent of 'family'/'size_dip' in DIPs, so the renderer can vertically center a
		// baseline-relative glyph run within a control's bounds without re-deriving font metrics.
		void Metrics(std::string_view family, float size_dip, float& out_ascent_dip, float& out_descent_dip);

		// Maps a layout-relative point to the UTF-8 byte offset of the nearest insertion position
		// in 'utf8_text'. DirectWrite's trailing-hit flag is honoured, then the result is snapped to
		// the *nearest* grapheme-cluster boundary, so clicking the right-hand half of a cluster
		// places the caret after it rather than collapsing back to its start.
		std::uint32_t OffsetFromPoint(std::string_view family, float size_dip, std::string_view utf8_text, float x_dip, float y_dip);

		// Caret position/height for the insertion point at UTF-8 byte offset 'utf8_offset'.
		CaretGeometry CaretAt(std::string_view family, float size_dip, std::string_view utf8_text, std::uint32_t utf8_offset);

		// Layout-relative rectangles covering the UTF-8 byte range [begin, end) - one per visually
		// contiguous run, so a range spanning a bidirectional boundary yields several rectangles.
		// 'max_rects' bounds the result. A range that needs more rectangles than the bound allows
		// is retried once at the bound and, if DirectWrite still refuses to fill the buffer, yields
		// no rectangles at all rather than uninitialised ones.
		std::vector<Rect> RangeRects(std::string_view family, float size_dip, std::string_view utf8_text, std::uint32_t begin, std::uint32_t end, std::uint32_t max_rects);

		// Current occupancy of the two bounded caches, so a caller (or a test) can observe that
		// neither grows past MaxCachedFonts/MaxCachedRunFaces.
		std::size_t CachedFontCount() const
		{
			return m_fonts.size();
		}
		std::size_t CachedRunFaceCount() const
		{
			return m_run_faces.size();
		}

	private:
		// Resolves (and caches) the face/format for 'family' at 'size_dip' under this class's
		// documented font policy. Throws EngineException(MissingAsset) when the requested family,
		// or the built-in fallback used for an unspecified family, is not installed.
		FontEntry& ResolveFont(std::string_view family, float size_dip);

		// Builds (or reuses) the IDWriteTextLayout for one (family, size, text) triple, along with
		// its UTF-8/UTF-16 offset maps. Throws EngineException(InvalidArgument) for text that is
		// not well-formed UTF-8.
		LayoutEntry& ResolveLayout(std::string_view family, float size_dip, std::string_view utf8_text);

		// Registers 'face' at 'size_dip' under a content-derived key and returns that key, so a
		// glyph shaped from a fallback face can be rasterized later without re-running layout.
		std::uint64_t RegisterRunFace(IDWriteFontFace* face, float size_dip);

		// Move 'key' to the front of 'mru' and, when that pushes the list past 'limit', erase the
		// least recently used key from both 'mru' and 'map'. Returns nothing; the caller inserts
		// its new entry afterwards so the entry it is about to return is never the one evicted.
		template <typename Map> static void TouchCacheKey(std::vector<std::uint64_t>& mru, Map& map, std::uint64_t key, std::size_t limit)
		{
			if (auto it = std::find(mru.begin(), mru.end(), key); it != mru.end())
			{
				std::rotate(mru.begin(), it, it + 1);
				return;
			}

			while (mru.size() >= limit && !mru.empty())
			{
				map.erase(mru.back());
				mru.pop_back();
			}
			mru.insert(mru.begin(), key);
		}
	};
}
