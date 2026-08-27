//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "text_shaper.h"
#include "text_unicode.h"
#include "pr/view3d-ui/engine.h" // for pr::view3d::ui::EngineException

// See view3d-12/src/main/renderer.cpp for the precedent of linking D3D/DirectWrite libraries via
// #pragma comment rather than per-project AdditionalDependencies entries.
#pragma comment(lib, "dwrite.lib")

namespace pr::view3d::ui
{
	namespace
	{
		// The one family this module may substitute for a control that names no font at all. Every
		// other unresolvable family is a MissingAsset; see the header's font policy.
		constexpr wchar_t const* BuiltinFallbackFamilyW = L"Segoe UI";

		// A fixed locale keeps shaping decisions that vary by language (e.g. Han glyph variants)
		// identical on every machine, which is what makes layout snapshots comparable at all.
		constexpr wchar_t const* LayoutLocaleW = L"en-us";

		// A single-line text field is laid out unwrapped, so the layout box only has to be wider
		// than any string this module will ever measure rather than tracking the control's width.
		constexpr float LayoutBoxDip = 1.0e6f;

		// Most-recently-used layout cache depth. Two entries would satisfy the renderer's
		// shape-then-draw pair for one item; eight keeps a small form's controls resident without
		// letting the cache grow with the document.
		constexpr std::size_t MaxCachedLayouts = 8;

		// Upper bound on the hit-test rectangles DirectWrite is asked to produce for one range, so
		// a pathological bidirectional string cannot drive an unbounded allocation.
		constexpr std::uint32_t MaxHitTestMetrics = 64;

		// FNV-1a over a byte range, used for every content-derived key in this file.
		std::uint64_t HashBytes(void const* data, std::size_t size, std::uint64_t seed)
		{
			auto const* bytes = static_cast<std::uint8_t const*>(data);
			auto hash = seed;
			for (auto i = std::size_t(0); i != size; ++i)
			{
				hash ^= bytes[i];
				hash *= 1099511628211ull;
			}
			return hash;
		}

		// Converts a UTF-8 byte sequence to UTF-16 for the DirectWrite APIs, rejecting malformed
		// input rather than substituting replacement characters, and records the UTF-8 offset each
		// UTF-16 unit came from (plus the reverse map) so caret/selection offsets can be translated
		// in both directions without re-scanning.
		void BuildOffsetMaps(std::string_view utf8, std::wstring& out_utf16, std::vector<std::uint32_t>& out_utf16_to_utf8, std::vector<std::uint32_t>& out_utf8_to_utf16)
		{
			out_utf16.clear();
			out_utf16_to_utf8.clear();
			out_utf8_to_utf16.assign(utf8.size() + 1, 0);

			for (auto offset = std::uint32_t(0); offset < utf8.size();)
			{
				char32_t cp = 0;
				std::uint32_t length = 0;
				if (!Utf8Decode(utf8, offset, cp, length))
					throw EngineException(EStatus::InvalidArgument, std::format("TextShaper: text is not valid UTF-8 at byte {}", offset));

				// Every byte of one UTF-8 sequence maps to the UTF-16 position of its first unit,
				// so an offset in the middle of a sequence still translates to a sane position.
				auto const utf16_pos = static_cast<std::uint32_t>(out_utf16.size());
				for (auto i = std::uint32_t(0); i != length; ++i)
					out_utf8_to_utf16[offset + i] = utf16_pos;

				if (cp < 0x10000)
				{
					out_utf16.push_back(static_cast<wchar_t>(cp));
					out_utf16_to_utf8.push_back(offset);
				}
				else
				{
					auto const adjusted = cp - 0x10000;
					out_utf16.push_back(static_cast<wchar_t>(0xD800 + (adjusted >> 10)));
					out_utf16.push_back(static_cast<wchar_t>(0xDC00 + (adjusted & 0x3FF)));
					out_utf16_to_utf8.push_back(offset);
					out_utf16_to_utf8.push_back(offset);
				}
				offset += length;
			}

			out_utf8_to_utf16[utf8.size()] = static_cast<std::uint32_t>(out_utf16.size());
			out_utf16_to_utf8.push_back(static_cast<std::uint32_t>(utf8.size()));
		}

		// Collects the glyph runs IDWriteTextLayout::Draw emits, in the visual order DirectWrite
		// produces them, so bidirectional reordering and per-run font fallback are handled by
		// DirectWrite rather than by this module. Only DrawGlyphRun does any work: underlines,
		// strikethroughs and inline objects are never applied to a layout built here.
		class GlyphRunCollector : public IDWriteTextRenderer
		{
			ULONG m_ref_count = 1;
			std::vector<IDWriteFontFace*> m_faces;
			std::vector<float> m_font_sizes;
			std::vector<std::vector<std::uint16_t>> m_indices;
			std::vector<std::vector<float>> m_advances;
			std::vector<std::vector<DWRITE_GLYPH_OFFSET>> m_offsets;
			std::vector<float> m_baseline_x;
			std::vector<float> m_baseline_y;
			std::vector<std::uint32_t> m_bidi_levels;

		public:
			GlyphRunCollector() = default;
			GlyphRunCollector(GlyphRunCollector const&) = delete;
			GlyphRunCollector& operator=(GlyphRunCollector const&) = delete;
			~GlyphRunCollector()
			{
				for (auto* face : m_faces)
					face->Release();
			}

			// Number of glyph runs collected so far, in visual order.
			std::size_t RunCount() const
			{
				return m_faces.size();
			}

			// The 'index'th collected run's face, glyph data and baseline origin.
			IDWriteFontFace* RunFace(std::size_t index) const
			{
				return m_faces[index];
			}
			float RunFontSize(std::size_t index) const
			{
				return m_font_sizes[index];
			}
			std::vector<std::uint16_t> const& RunIndices(std::size_t index) const
			{
				return m_indices[index];
			}
			std::vector<float> const& RunAdvances(std::size_t index) const
			{
				return m_advances[index];
			}
			std::vector<DWRITE_GLYPH_OFFSET> const& RunOffsets(std::size_t index) const
			{
				return m_offsets[index];
			}
			float RunBaselineX(std::size_t index) const
			{
				return m_baseline_x[index];
			}
			float RunBaselineY(std::size_t index) const
			{
				return m_baseline_y[index];
			}

			// The Unicode embedding level DirectWrite resolved for the run. An odd level means the
			// run's baseline origin is its *right* edge and its glyph advances accumulate leftwards.
			std::uint32_t RunBidiLevel(std::size_t index) const
			{
				return m_bidi_levels[index];
			}

			HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv) override
			{
				if (ppv == nullptr)
					return E_POINTER;

				if (riid == __uuidof(IUnknown) || riid == __uuidof(IDWritePixelSnapping) || riid == __uuidof(IDWriteTextRenderer))
				{
					*ppv = static_cast<IDWriteTextRenderer*>(this);
					AddRef();
					return S_OK;
				}

				*ppv = nullptr;
				return E_NOINTERFACE;
			}
			ULONG STDMETHODCALLTYPE AddRef() override
			{
				return ++m_ref_count;
			}
			ULONG STDMETHODCALLTYPE Release() override
			{
				// This object is always a stack local owned by its creator, so the reference count
				// is only tracked to satisfy COM rules; it is never deleted from here.
				return --m_ref_count;
			}

			HRESULT STDMETHODCALLTYPE IsPixelSnappingDisabled(void*, BOOL* disabled) override
			{
				*disabled = TRUE; // this module positions glyphs itself, in DIPs
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetCurrentTransform(void*, DWRITE_MATRIX* transform) override
			{
				*transform = DWRITE_MATRIX{ 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f };
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetPixelsPerDip(void*, FLOAT* pixels_per_dip) override
			{
				*pixels_per_dip = 1.0f; // layout is performed entirely in DIPs
				return S_OK;
			}

			HRESULT STDMETHODCALLTYPE DrawGlyphRun(void*, FLOAT baseline_x, FLOAT baseline_y, DWRITE_MEASURING_MODE, DWRITE_GLYPH_RUN const* run, DWRITE_GLYPH_RUN_DESCRIPTION const*, IUnknown*) override
			{
				if (run == nullptr || run->glyphCount == 0 || run->fontFace == nullptr)
					return S_OK;

				// DirectWrite owns the run arrays only for the duration of this callback, so each
				// run's glyph data is copied out before returning. glyphAdvances and glyphOffsets
				// are both documented as optional and are null for a run that needs neither, in
				// which case the run's own defaults - zero offsets, design advances - apply.
				m_indices.emplace_back(run->glyphIndices, run->glyphIndices + run->glyphCount);
				if (run->glyphAdvances != nullptr)
				{
					m_advances.emplace_back(run->glyphAdvances, run->glyphAdvances + run->glyphCount);
				}
				else
				{
					// Fall back to the face's own design advances so a run without explicit
					// advances still lays out at its natural width instead of collapsing to zero.
					m_advances.emplace_back(run->glyphCount, 0.0f);
					auto design = std::vector<DWRITE_GLYPH_METRICS>(run->glyphCount, DWRITE_GLYPH_METRICS{});
					DWRITE_FONT_METRICS face_metrics{};
					run->fontFace->GetMetrics(&face_metrics);
					if (face_metrics.designUnitsPerEm != 0 && SUCCEEDED(run->fontFace->GetDesignGlyphMetrics(run->glyphIndices, run->glyphCount, design.data(), run->isSideways)))
					{
						auto const to_dip = run->fontEmSize / static_cast<float>(face_metrics.designUnitsPerEm);
						for (auto i = std::size_t(0); i != design.size(); ++i)
							m_advances.back()[i] = static_cast<float>(design[i].advanceWidth) * to_dip;
					}
				}
				if (run->glyphOffsets != nullptr)
					m_offsets.emplace_back(run->glyphOffsets, run->glyphOffsets + run->glyphCount);
				else
					m_offsets.emplace_back(run->glyphCount, DWRITE_GLYPH_OFFSET{ 0.0f, 0.0f });

				// The face is retained because the copied run outlives this callback, and the
				// layout that produced it is free to drop its own reference in between.
				run->fontFace->AddRef();
				m_faces.push_back(run->fontFace);
				m_font_sizes.push_back(run->fontEmSize);
				m_baseline_x.push_back(baseline_x);
				m_baseline_y.push_back(baseline_y);
				m_bidi_levels.push_back(run->bidiLevel);
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE DrawUnderline(void*, FLOAT, FLOAT, DWRITE_UNDERLINE const*, IUnknown*) override
			{
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE DrawStrikethrough(void*, FLOAT, FLOAT, DWRITE_STRIKETHROUGH const*, IUnknown*) override
			{
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE DrawInlineObject(void*, FLOAT, FLOAT, IDWriteInlineObject*, BOOL, BOOL, IUnknown*) override
			{
				return S_OK;
			}
		};
	}

	TextShaper::TextShaper()
	{
		auto hr = ::DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory), reinterpret_cast<IUnknown**>(m_factory.GetAddressOf()));
		if (FAILED(hr))
			throw EngineException(EStatus::InternalError, "TextShaper: DWriteCreateFactory failed");
	}

	std::uint64_t TextShaper::FontKey(std::string_view family, float size_dip)
	{
		auto hash = HashBytes(family.data(), family.size(), 1469598103934665603ull);
		std::uint32_t size_bits = 0;
		std::memcpy(&size_bits, &size_dip, sizeof(size_bits));
		return HashBytes(&size_bits, sizeof(size_bits), hash);
	}

	TextShaper::FontEntry& TextShaper::ResolveFont(std::string_view family, float size_dip)
	{
		auto key = FontKey(family, size_dip);
		if (auto it = m_fonts.find(key); it != m_fonts.end())
		{
			TouchCacheKey(m_font_mru, m_fonts, key, MaxCachedFonts);
			return it->second;
		}

		Microsoft::WRL::ComPtr<IDWriteFontCollection> collection;
		if (FAILED(m_factory->GetSystemFontCollection(collection.GetAddressOf(), FALSE)))
			throw EngineException(EStatus::InternalError, "TextShaper: GetSystemFontCollection failed");

		// An unspecified family is the only case that may take the built-in fallback; a named
		// family that is not installed is reported as a missing asset instead of being replaced by
		// an unrelated face, so the application always learns that its font never loaded.
		std::wstring family_w;
		if (family.empty())
		{
			family_w = BuiltinFallbackFamilyW;
		}
		else if (!Utf8ToUtf16(family, family_w))
		{
			throw EngineException(EStatus::InvalidArgument, "TextShaper: font family name is not valid UTF-8");
		}

		std::uint32_t family_index = 0;
		BOOL exists = FALSE;
		if (FAILED(collection->FindFamilyName(family_w.c_str(), &family_index, &exists)))
			throw EngineException(EStatus::InternalError, "TextShaper: FindFamilyName failed");
		if (!exists)
			throw EngineException(EStatus::MissingAsset, std::format("TextShaper: font family '{}' is not installed", family.empty() ? std::string("Segoe UI") : std::string(family)));

		Microsoft::WRL::ComPtr<IDWriteFontFamily> font_family;
		if (FAILED(collection->GetFontFamily(family_index, font_family.GetAddressOf())))
			throw EngineException(EStatus::InternalError, "TextShaper: GetFontFamily failed");

		Microsoft::WRL::ComPtr<IDWriteFont> font;
		if (FAILED(font_family->GetFirstMatchingFont(DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STRETCH_NORMAL, DWRITE_FONT_STYLE_NORMAL, font.GetAddressOf())))
			throw EngineException(EStatus::InternalError, "TextShaper: GetFirstMatchingFont failed");

		Microsoft::WRL::ComPtr<IDWriteFontFace> face;
		if (FAILED(font->CreateFontFace(face.GetAddressOf())))
			throw EngineException(EStatus::InternalError, "TextShaper: CreateFontFace failed");

		Microsoft::WRL::ComPtr<IDWriteTextFormat> format;
		if (FAILED(m_factory->CreateTextFormat(family_w.c_str(), nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL, size_dip > 0.0f ? size_dip : 1.0f, LayoutLocaleW, format.GetAddressOf())))
			throw EngineException(EStatus::InternalError, "TextShaper: CreateTextFormat failed");

		// A single-line editable field never wraps; DirectWrite still reorders bidirectional runs
		// and applies complex-script shaping within that one line.
		format->SetWordWrapping(DWRITE_WORD_WRAPPING_NO_WRAP);

		DWRITE_FONT_METRICS metrics{};
		face->GetMetrics(&metrics);
		auto units_per_em = metrics.designUnitsPerEm != 0 ? static_cast<float>(metrics.designUnitsPerEm) : 1.0f;

		auto entry = FontEntry{
			.face = face,
			.format = format,
			.size_dip = size_dip,
			.ascent_dip = static_cast<float>(metrics.ascent) / units_per_em * size_dip,
			.descent_dip = static_cast<float>(metrics.descent) / units_per_em * size_dip,
		};

		// Evict before inserting so the entry about to be returned is never the one discarded.
		TouchCacheKey(m_font_mru, m_fonts, key, MaxCachedFonts);
		return m_fonts.insert_or_assign(key, std::move(entry)).first->second;
	}

	std::uint64_t TextShaper::RegisterRunFace(IDWriteFontFace* face, float size_dip)
	{
		// Key the face by the content of its font-file reference keys and its face index, so the
		// same physical face always produces the same key regardless of allocation addresses.
		auto hash = std::uint64_t(1469598103934665603ull);
		std::uint32_t file_count = 0;
		if (SUCCEEDED(face->GetFiles(&file_count, nullptr)) && file_count != 0)
		{
			std::vector<IDWriteFontFile*> files(file_count, nullptr);
			if (SUCCEEDED(face->GetFiles(&file_count, files.data())))
			{
				for (auto* file : files)
				{
					void const* reference_key = nullptr;
					std::uint32_t reference_size = 0;
					if (file != nullptr && SUCCEEDED(file->GetReferenceKey(&reference_key, &reference_size)) && reference_key != nullptr)
						hash = HashBytes(reference_key, reference_size, hash);

					if (file != nullptr)
						file->Release();
				}
			}
		}

		auto const face_index = face->GetIndex();
		auto const simulations = static_cast<std::uint32_t>(face->GetSimulations());
		hash = HashBytes(&face_index, sizeof(face_index), hash);
		hash = HashBytes(&simulations, sizeof(simulations), hash);
		std::uint32_t size_bits = 0;
		std::memcpy(&size_bits, &size_dip, sizeof(size_bits));
		hash = HashBytes(&size_bits, sizeof(size_bits), hash);

		// The face cache is bounded the same way as the font cache: because the key is derived
		// purely from the face's file references, index, simulations and size, an evicted entry is
		// re-created identically by the next Shape() call that uses that face.
		TouchCacheKey(m_run_face_mru, m_run_faces, hash, MaxCachedRunFaces);
		if (m_run_faces.find(hash) == m_run_faces.end())
			m_run_faces.emplace(hash, RunFace{ .face = face, .size_dip = size_dip });

		return hash;
	}

	TextShaper::LayoutEntry& TextShaper::ResolveLayout(std::string_view family, float size_dip, std::string_view utf8_text)
	{
		// A most-recently-used probe keeps the renderer's shape-then-draw pair for one item to a
		// single layout construction without letting the cache grow with the control count.
		for (auto i = std::size_t(0); i != m_layouts.size(); ++i)
		{
			if (m_layouts[i].size_dip != size_dip || m_layouts[i].family != family || m_layouts[i].text != utf8_text)
				continue;

			if (i != 0)
				std::rotate(m_layouts.begin(), m_layouts.begin() + i, m_layouts.begin() + i + 1);

			return m_layouts.front();
		}

		auto& font = ResolveFont(family, size_dip);

		LayoutEntry entry{};
		entry.text = std::string(utf8_text);
		entry.family = std::string(family);
		entry.size_dip = size_dip;

		std::wstring utf16;
		BuildOffsetMaps(utf8_text, utf16, entry.utf16_to_utf8, entry.utf8_to_utf16);

		if (FAILED(m_factory->CreateTextLayout(utf16.c_str(), static_cast<UINT32>(utf16.size()), font.format.Get(), LayoutBoxDip, LayoutBoxDip, entry.layout.GetAddressOf())))
			throw EngineException(EStatus::InternalError, "TextShaper: CreateTextLayout failed");

		if (m_layouts.size() >= MaxCachedLayouts)
			m_layouts.pop_back();

		m_layouts.insert(m_layouts.begin(), std::move(entry));
		return m_layouts.front();
	}

	float TextShaper::Shape(std::string_view family, float size_dip, float dpi_scale, std::string_view utf8_text, std::vector<ShapedGlyph>& out_glyphs)
	{
		(void)dpi_scale; // shaping happens in DIPs; dpi_scale only matters once Rasterize() maps to physical pixels

		out_glyphs.clear();
		if (utf8_text.empty() || size_dip <= 0.0f)
			return 0.0f;

		auto& entry = ResolveLayout(family, size_dip, utf8_text);

		GlyphRunCollector collector;
		if (FAILED(entry.layout->Draw(nullptr, &collector, 0.0f, 0.0f)))
			throw EngineException(EStatus::InternalError, "TextShaper: IDWriteTextLayout::Draw failed");

		// Runs arrive in visual order with absolute baseline origins. For a left-to-right run that
		// origin is the run's left edge and the pen moves right; for a right-to-left run (odd
		// embedding level) DirectWrite reports the run's *right* edge and the advances accumulate
		// leftwards, so each glyph's own left-edge origin is one advance further left than the pen.
		for (auto run = std::size_t(0); run != collector.RunCount(); ++run)
		{
			auto const font_key = RegisterRunFace(collector.RunFace(run), collector.RunFontSize(run));
			auto const& indices = collector.RunIndices(run);
			auto const& advances = collector.RunAdvances(run);
			auto const& offsets = collector.RunOffsets(run);
			auto const bidi_level = collector.RunBidiLevel(run);
			auto const rtl = (bidi_level & 1u) != 0u;

			auto pen_x = collector.RunBaselineX(run);
			auto const baseline_y = collector.RunBaselineY(run);
			for (auto i = std::size_t(0); i != indices.size(); ++i)
			{
				// A glyph offset is expressed along the run's advance direction, so it too flips
				// sign inside a right-to-left run.
				auto origin_x = 0.0f;
				if (rtl)
				{
					pen_x -= advances[i];
					origin_x = pen_x - offsets[i].advanceOffset;
				}
				else
				{
					origin_x = pen_x + offsets[i].advanceOffset;
					pen_x += advances[i];
				}

				out_glyphs.push_back(ShapedGlyph{
					.font_key = font_key,
					.glyph_index = indices[i],
					.bidi_level = bidi_level,
					.advance_x = advances[i],
					.origin_x = origin_x,
					.origin_y = baseline_y - offsets[i].ascenderOffset,
				});
			}
		}

		DWRITE_TEXT_METRICS metrics{};
		if (FAILED(entry.layout->GetMetrics(&metrics)))
			throw EngineException(EStatus::InternalError, "TextShaper: GetMetrics failed");

		return metrics.widthIncludingTrailingWhitespace;
	}

	GlyphBitmap TextShaper::Rasterize(std::uint64_t font_key, float dpi_scale, std::uint32_t glyph_index)
	{
		auto it = m_run_faces.find(font_key);
		if (it == m_run_faces.end())
			throw EngineException(EStatus::InvalidArgument, std::format("TextShaper: no shaped run has registered font key {}", font_key));

		// Rasterizing is what keeps a face genuinely in use, so it counts as a cache touch and a
		// face the renderer draws every frame is never the one evicted for a one-off fallback.
		TouchCacheKey(m_run_face_mru, m_run_faces, font_key, MaxCachedRunFaces);

		auto size_px = it->second.size_dip * dpi_scale;

		UINT16 glyph_id = static_cast<UINT16>(glyph_index);
		FLOAT advance = 0.0f;
		DWRITE_GLYPH_OFFSET offset{};
		DWRITE_GLYPH_RUN run{};
		run.fontFace = it->second.face.Get();
		run.fontEmSize = size_px;
		run.glyphCount = 1;
		run.glyphIndices = &glyph_id;
		run.glyphAdvances = &advance;
		run.glyphOffsets = &offset;
		run.isSideways = FALSE;
		run.bidiLevel = 0;

		Microsoft::WRL::ComPtr<IDWriteGlyphRunAnalysis> analysis;
		auto hr = m_factory->CreateGlyphRunAnalysis(&run, 1.0f, nullptr, DWRITE_RENDERING_MODE_ALIASED, DWRITE_MEASURING_MODE_NATURAL, 0.0f, 0.0f, analysis.GetAddressOf());
		if (FAILED(hr))
			throw EngineException(EStatus::InternalError, "TextShaper: CreateGlyphRunAnalysis failed");

		RECT bounds{};
		if (FAILED(analysis->GetAlphaTextureBounds(DWRITE_TEXTURE_ALIASED_1x1, &bounds)))
			throw EngineException(EStatus::InternalError, "TextShaper: GetAlphaTextureBounds failed");

		auto width = bounds.right - bounds.left;
		auto height = bounds.bottom - bounds.top;
		if (width <= 0 || height <= 0)
			return GlyphBitmap{ .width_px = 0, .height_px = 0, .origin_x_px = 0, .origin_y_px = 0, .alpha = {} }; // a whitespace glyph has no coverage pixels at all

		std::vector<std::uint8_t> alpha(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
		if (FAILED(analysis->CreateAlphaTexture(DWRITE_TEXTURE_ALIASED_1x1, &bounds, alpha.data(), static_cast<UINT32>(alpha.size()))))
			throw EngineException(EStatus::InternalError, "TextShaper: CreateAlphaTexture failed");

		return GlyphBitmap{
			.width_px = static_cast<std::uint32_t>(width),
			.height_px = static_cast<std::uint32_t>(height),
			.origin_x_px = bounds.left,
			.origin_y_px = bounds.top,
			.alpha = std::move(alpha),
		};
	}

	void TextShaper::Metrics(std::string_view family, float size_dip, float& out_ascent_dip, float& out_descent_dip)
	{
		auto& entry = ResolveFont(family, size_dip);
		out_ascent_dip = entry.ascent_dip;
		out_descent_dip = entry.descent_dip;
	}

	std::uint32_t TextShaper::OffsetFromPoint(std::string_view family, float size_dip, std::string_view utf8_text, float x_dip, float y_dip)
	{
		if (utf8_text.empty())
			return 0;

		auto& entry = ResolveLayout(family, size_dip, utf8_text);

		BOOL trailing = FALSE;
		BOOL inside = FALSE;
		DWRITE_HIT_TEST_METRICS metrics{};
		if (FAILED(entry.layout->HitTestPoint(x_dip, y_dip, &trailing, &inside, &metrics)))
			throw EngineException(EStatus::InternalError, "TextShaper: HitTestPoint failed");

		// The trailing flag says the point fell in the second half of the hit cluster, so honouring
		// it is what lets a click on the right of a character place the caret after it.
		auto utf16_pos = metrics.textPosition + (trailing != FALSE ? metrics.length : 0u);
		if (utf16_pos >= entry.utf16_to_utf8.size())
			return static_cast<std::uint32_t>(utf8_text.size());

		// DirectWrite reports shaping clusters, which are not always whole extended grapheme
		// clusters: an emoji ZWJ sequence or a base plus combining mark can hit-test to an interior
		// position. Snapping to the *nearest* boundary keeps the guarantee that no caret splits a
		// grapheme while still letting the right-hand half of a cluster select the position after
		// it, which rounding down alone would never reach.
		return NearestGraphemeBoundary(utf8_text, entry.utf16_to_utf8[utf16_pos]);
	}

	CaretGeometry TextShaper::CaretAt(std::string_view family, float size_dip, std::string_view utf8_text, std::uint32_t utf8_offset)
	{
		if (utf8_text.empty())
		{
			// An empty field still has a caret, so report the resolved font's line height rather
			// than a zero-height rectangle the renderer would have to special-case.
			float ascent = 0.0f, descent = 0.0f;
			Metrics(family, size_dip, ascent, descent);
			return CaretGeometry{ .x = 0.0f, .y = 0.0f, .height = ascent + descent, .is_rtl = 0 };
		}

		auto& entry = ResolveLayout(family, size_dip, utf8_text);

		// A caller may hand over an offset produced from a different string revision or from a
		// byte-oriented calculation, so snap it rather than draw a caret between the halves of a
		// surrogate pair or inside a combining sequence.
		auto const clamped = ClampToGraphemeBoundary(utf8_text, utf8_offset);
		auto const utf16_pos = entry.utf8_to_utf16[clamped];

		FLOAT point_x = 0.0f, point_y = 0.0f;
		DWRITE_HIT_TEST_METRICS metrics{};
		if (FAILED(entry.layout->HitTestTextPosition(utf16_pos, FALSE, &point_x, &point_y, &metrics)))
			throw EngineException(EStatus::InternalError, "TextShaper: HitTestTextPosition failed");

		return CaretGeometry{
			.x = point_x,
			.y = point_y,
			.height = metrics.height,
			.is_rtl = (metrics.bidiLevel % 2) != 0 ? 1 : 0,
		};
	}

	std::vector<Rect> TextShaper::RangeRects(std::string_view family, float size_dip, std::string_view utf8_text, std::uint32_t begin, std::uint32_t end, std::uint32_t max_rects)
	{
		std::vector<Rect> rects;
		if (utf8_text.empty() || begin >= end || max_rects == 0)
			return rects;

		auto& entry = ResolveLayout(family, size_dip, utf8_text);

		// Snapping both ends keeps a highlight from covering half a grapheme even when the caller
		// derived the range from something other than the caret machinery.
		auto const utf16_begin = entry.utf8_to_utf16[ClampToGraphemeBoundary(utf8_text, begin)];
		auto const utf16_end = entry.utf8_to_utf16[ClampToGraphemeBoundary(utf8_text, end)];
		if (utf16_end <= utf16_begin)
			return rects;

		auto const capacity = std::min(max_rects, MaxHitTestMetrics);
		std::vector<DWRITE_HIT_TEST_METRICS> metrics(capacity);
		UINT32 actual = 0;
		auto hr = entry.layout->HitTestTextRange(utf16_begin, utf16_end - utf16_begin, 0.0f, 0.0f, metrics.data(), capacity, &actual);

		// On E_NOT_SUFFICIENT_BUFFER DirectWrite reports the required count but leaves the buffer
		// untouched, so the array holds nothing usable. Retry once at the module's own bound when
		// that is larger than the caller asked for; if the range still needs more rectangles than
		// the bound allows, report none rather than drawing uninitialised geometry. Highlighting is
		// a visual aid, so degrading to "no highlight" is preferable to a wrong one.
		if (hr == E_NOT_SUFFICIENT_BUFFER && actual > capacity && actual <= MaxHitTestMetrics)
		{
			metrics.assign(actual, DWRITE_HIT_TEST_METRICS{});
			hr = entry.layout->HitTestTextRange(utf16_begin, utf16_end - utf16_begin, 0.0f, 0.0f, metrics.data(), actual, &actual);
		}
		if (hr == E_NOT_SUFFICIENT_BUFFER)
			return rects;
		if (FAILED(hr))
			throw EngineException(EStatus::InternalError, "TextShaper: HitTestTextRange failed");

		auto const count = std::min<std::uint32_t>(actual, std::min<std::uint32_t>(max_rects, static_cast<std::uint32_t>(metrics.size())));
		rects.reserve(count);
		for (auto i = std::uint32_t(0); i != count; ++i)
			rects.push_back(Rect{ metrics[i].left, metrics[i].top, metrics[i].width, metrics[i].height });

		return rects;
	}
}
