//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// White-box tests for the CPU-side text shaping/measuring/rasterization helper the renderer,
// caret placement and semantics all share. TextShaper needs no D3D12 device at all (DirectWrite is
// a software text layout API, independent of any GPU), so every test here runs unconditionally
// rather than skipping in a headless/no-GPU environment; it does still require the OS DirectWrite
// component and the Segoe UI system font, both of which are present on every supported Windows
// target for this project.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "text_shaper.h"

namespace pr::view3d::ui::tests
{
	PRUnitTest(TextShaperFontKeyIsStableAndDistinguishesFamilyAndSize, Quick)
	{
		auto const key_a = TextShaper::FontKey("Segoe UI", 14.0f);
		auto const key_b = TextShaper::FontKey("Segoe UI", 14.0f);
		PR_EXPECT(key_a == key_b);

		auto const key_diff_size = TextShaper::FontKey("Segoe UI", 18.0f);
		auto const key_diff_family = TextShaper::FontKey("Consolas", 14.0f);
		PR_EXPECT(key_a != key_diff_size);
		PR_EXPECT(key_a != key_diff_family);
	}

	PRUnitTest(TextShaperShapeProducesOneGlyphPerCodepointWithIncreasingAdvance, Quick)
	{
		auto shaper = TextShaper();
		auto glyphs = std::vector<ShapedGlyph>();
		auto const total_advance = shaper.Shape("Segoe UI", 16.0f, 1.0f, "Hi", glyphs);

		PR_EXPECT(glyphs.size() == 2);
		PR_EXPECT(total_advance > 0.0f);
		PR_EXPECT(glyphs[0].origin_x == 0.0f); // the first glyph's pen position is always the origin
		PR_EXPECT(glyphs[0].advance_x > 0.0f);
		PR_EXPECT(glyphs[1].origin_x == glyphs[0].advance_x); // the second glyph starts where the first ends
		PR_EXPECT(glyphs[0].glyph_index != glyphs[1].glyph_index); // 'H' and 'i' are visually and metrically distinct
	}

	PRUnitTest(TextShaperShapeOfEmptyTextProducesNoGlyphsAndZeroAdvance, Quick)
	{
		auto shaper = TextShaper();
		auto glyphs = std::vector<ShapedGlyph>();
		glyphs.push_back(ShapedGlyph{}); // pre-populate so the test also proves Shape clears its output
		auto const total_advance = shaper.Shape("Segoe UI", 16.0f, 1.0f, "", glyphs);

		PR_EXPECT(glyphs.empty());
		PR_EXPECT(total_advance == 0.0f);
	}

	PRUnitTest(TextShaperUnresolvableFamilyReportsMissingAssetRatherThanSubstituting, Quick)
	{
		// The M9 font policy: a control that explicitly names a font resource keeps that family
		// verbatim, so a family absent from this machine's collection is a diagnosable missing
		// asset rather than a silent substitution with an unrelated face. Only a control that
		// names no font at all is answered with the built-in fallback (covered below).
		auto shaper = TextShaper();
		auto glyphs = std::vector<ShapedGlyph>();
		auto status = EStatus::Success;
		try
		{
			shaper.Shape("Not A Real Font Family XYZ", 16.0f, 1.0f, "A", glyphs);
		}
		catch (EngineException const& ex)
		{
			status = ex.Status();
		}
		PR_EXPECT(status == EStatus::MissingAsset);
	}

	PRUnitTest(TextShaperEmptyFamilyUsesTheBuiltInFallbackFace, Quick)
	{
		// An empty family is the module's own "no font named" signal, and is the only case where
		// substitution is permitted at all.
		auto shaper = TextShaper();
		auto glyphs = std::vector<ShapedGlyph>();
		auto fallback_glyphs = std::vector<ShapedGlyph>();
		auto const empty_advance = shaper.Shape("", 16.0f, 1.0f, "A", glyphs);
		auto const named_advance = shaper.Shape("Segoe UI", 16.0f, 1.0f, "A", fallback_glyphs);

		PR_EXPECT(glyphs.size() == 1);
		PR_EXPECT(empty_advance == named_advance);
		PR_EXPECT(glyphs[0].glyph_index == fallback_glyphs[0].glyph_index);
	}

	PRUnitTest(TextShaperRasterizeOfVisibleGlyphProducesNonEmptyCoverageBitmap, Quick)
	{
		auto shaper = TextShaper();
		auto glyphs = std::vector<ShapedGlyph>();
		shaper.Shape("Segoe UI", 24.0f, 1.0f, "A", glyphs);
		PR_EXPECT(glyphs.size() == 1);

		auto const bitmap = shaper.Rasterize(glyphs[0].font_key, 1.0f, glyphs[0].glyph_index);
		PR_EXPECT(bitmap.width_px > 0);
		PR_EXPECT(bitmap.height_px > 0);
		PR_EXPECT(bitmap.alpha.size() == static_cast<std::size_t>(bitmap.width_px) * bitmap.height_px);
	}

	PRUnitTest(TextShaperRasterizeOfWhitespaceGlyphHasNoCoveragePixels, Quick)
	{
		// The renderer relies on this exact zero-size contract (renderer.cpp's PrepareText) to skip
		// placing whitespace glyphs into the atlas at all, rather than wasting a slot on them.
		auto shaper = TextShaper();
		auto glyphs = std::vector<ShapedGlyph>();
		shaper.Shape("Segoe UI", 16.0f, 1.0f, " ", glyphs);
		PR_EXPECT(glyphs.size() == 1);

		auto const bitmap = shaper.Rasterize(glyphs[0].font_key, 1.0f, glyphs[0].glyph_index);
		PR_EXPECT(bitmap.width_px == 0);
		PR_EXPECT(bitmap.height_px == 0);
		PR_EXPECT(bitmap.alpha.empty());
	}

	PRUnitTest(TextShaperRasterizeOfAnUnregisteredFontKeyIsRejected, Quick)
	{
		// A font key only becomes meaningful once Shape has associated it with a resolved face, so
		// a fabricated key must fail loudly rather than silently rasterizing from some other face.
		auto shaper = TextShaper();
		auto status = EStatus::Success;
		try
		{
			shaper.Rasterize(0xDEADBEEFULL, 1.0f, 1);
		}
		catch (EngineException const& ex)
		{
			status = ex.Status();
		}
		PR_EXPECT(status == EStatus::InvalidArgument);
	}

	PRUnitTest(TextShaperMetricsReturnsPositiveAscentAndDescentForANormalFontSize, Quick)
	{
		auto shaper = TextShaper();
		auto ascent_dip = 0.0f;
		auto descent_dip = 0.0f;
		shaper.Metrics("Segoe UI", 16.0f, ascent_dip, descent_dip);

		PR_EXPECT(ascent_dip > 0.0f);
		PR_EXPECT(descent_dip > 0.0f);

		// Metrics scale with the requested size: doubling the font size must not shrink either.
		auto ascent_dip_large = 0.0f;
		auto descent_dip_large = 0.0f;
		shaper.Metrics("Segoe UI", 32.0f, ascent_dip_large, descent_dip_large);
		PR_EXPECT(ascent_dip_large > ascent_dip);
		PR_EXPECT(descent_dip_large > descent_dip);
	}
}
