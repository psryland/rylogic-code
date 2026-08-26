//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M9 tests for the shared shaped-text layout: bidirectional caret/hit-test geometry, pointer caret
// placement and the font resolution/substitution policy. These use a real IDWriteFactory (no D3D
// device, no window) so the tests exercise the same DirectWrite layout that rendering, caret
// placement, selection ranges and semantic ranges all agree on.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h" // for pr::view3d::ui::EngineException
#include "test_support.h"
#include "text_shaper.h"
#include "text_unicode.h"
#include <string>

namespace pr::view3d::ui::tests
{
	namespace
	{
		auto const kFamily = std::string_view("Segoe UI");
		auto const kSize = 14.0f;

		// Total advance of 'text', i.e. the width the renderer lays it out at.
		float WidthOf(TextShaper& shaper, std::string_view text)
		{
			auto glyphs = std::vector<ShapedGlyph>{};
			return shaper.Shape(kFamily, kSize, 1.0f, text, glyphs);
		}
	}

	PRUnitTest(CaretGeometryAdvancesMonotonicallyThroughLeftToRightText, Quick)
	{
		auto shaper = TextShaper{};
		auto const text = std::string("Hello world");

		// Every cluster boundary must yield a caret strictly to the right of the previous one in
		// pure LTR text, and the last one must sit at the end of the run.
		auto previous = -1.0f;
		for (auto offset = std::uint32_t(0); offset <= text.size(); offset = NextGraphemeBoundary(text, offset))
		{
			auto const caret = shaper.CaretAt(kFamily, kSize, text, offset);
			PR_EXPECT(caret.height > 0.0f);
			PR_EXPECT(caret.is_rtl == 0);
			PR_EXPECT(caret.x > previous);
			previous = caret.x;

			if (offset == text.size())
				break;
		}

		auto const width = WidthOf(shaper, text);
		PR_EXPECT(std::abs(previous - width) < 1.0f);
	}

	PRUnitTest(CaretGeometryInRightToLeftTextReportsRtlAndRunsTheOtherWay, Quick)
	{
		auto shaper = TextShaper{};

		// Hebrew is strongly RTL, so DirectWrite's own bidi resolution must report the run as RTL
		// and place successive logical carets leftwards rather than rightwards.
		auto const rtl = std::string("\u05E9\u05DC\u05D5\u05DD");
		auto const at_start = shaper.CaretAt(kFamily, kSize, rtl, 0);
		auto const at_end = shaper.CaretAt(kFamily, kSize, rtl, static_cast<std::uint32_t>(rtl.size()));

		PR_EXPECT(at_start.is_rtl != 0);
		PR_EXPECT(at_start.x > at_end.x);
		PR_EXPECT(at_end.x >= 0.0f);
	}

	PRUnitTest(BidirectionalTextPlacesTheCaretUsingDirectWritesResolvedVisualOrder, Quick)
	{
		auto shaper = TextShaper{};

		// "abc" + Hebrew + "def": the logical middle is visually reordered, so a home-grown
		// left-to-right walk would put the caret in the wrong place. The caret at the boundary
		// between the LTR prefix and the RTL run must not be at either extreme of the line.
		auto const mixed = std::string("abc\u05E9\u05DC\u05D5def");
		auto const width = WidthOf(shaper, mixed);
		auto const at_boundary = shaper.CaretAt(kFamily, kSize, mixed, 3);

		PR_EXPECT(at_boundary.x > 0.0f);
		PR_EXPECT(at_boundary.x < width);

		// Every logical offset produces a caret inside the laid-out line, which is the invariant a
		// rendered caret depends on regardless of visual order.
		for (auto offset = std::uint32_t(0); offset <= mixed.size(); offset = NextGraphemeBoundary(mixed, offset))
		{
			auto const caret = shaper.CaretAt(kFamily, kSize, mixed, offset);
			PR_EXPECT(caret.x >= -1.0f && caret.x <= width + 1.0f);
			PR_EXPECT(caret.height > 0.0f);

			if (offset == mixed.size())
				break;
		}
	}

	PRUnitTest(PointerHitTestingRoundTripsThroughCaretGeometry, Quick)
	{
		auto shaper = TextShaper{};
		auto const text = std::string("Pointer placement");

		// Hit-testing the exact caret position of each cluster must return that cluster's offset,
		// which is what makes a click land where the caret is drawn.
		for (auto offset = std::uint32_t(0); offset < text.size(); offset = NextGraphemeBoundary(text, offset))
		{
			auto const caret = shaper.CaretAt(kFamily, kSize, text, offset);
			auto const hit = shaper.OffsetFromPoint(kFamily, kSize, text, caret.x + 0.5f, caret.y + caret.height * 0.5f);
			PR_EXPECT(hit == offset);
		}
	}

	PRUnitTest(PointerHitTestingClampsOutsideTheRunAndNeverSplitsACluster, Quick)
	{
		auto shaper = TextShaper{};

		// A string of astral emoji: a byte-based hit test would happily return an offset inside a
		// four-byte cluster.
		auto const text = std::string("\U0001F600\U0001F601\U0001F602");
		auto const width = WidthOf(shaper, text);

		PR_EXPECT(shaper.OffsetFromPoint(kFamily, kSize, text, -500.0f, 0.0f) == 0);
		PR_EXPECT(shaper.OffsetFromPoint(kFamily, kSize, text, width + 500.0f, 0.0f) == text.size());

		// Sweep the whole run; every result must be a legal caret position.
		for (auto x = 0.0f; x < width; x += 1.0f)
		{
			auto const hit = shaper.OffsetFromPoint(kFamily, kSize, text, x, 0.0f);
			PR_EXPECT(IsGraphemeBoundary(text, hit));
		}
	}

	PRUnitTest(SelectionRangeRectanglesCoverTheSelectedRunAndAreEmptyWhenCollapsed, Quick)
	{
		auto shaper = TextShaper{};
		auto const text = std::string("select me");

		// A collapsed range has no area to fill.
		PR_EXPECT(shaper.RangeRects(kFamily, kSize, text, 3, 3, 32).empty());

		// A real range produces at least one non-degenerate rectangle inside the laid-out line.
		auto const width = WidthOf(shaper, text);
		auto const rects = shaper.RangeRects(kFamily, kSize, text, 0, 6, 32);
		PR_EXPECT(!rects.empty());

		auto covered = 0.0f;
		for (auto const& r : rects)
		{
			PR_EXPECT(r.w > 0.0f && r.h > 0.0f);
			PR_EXPECT(r.x >= -1.0f && r.x + r.w <= width + 1.0f);
			covered += r.w;
		}
		PR_EXPECT(covered > 0.0f && covered <= width + 1.0f);
	}

	PRUnitTest(SelectionRangeRectanglesInBidirectionalTextStayWithinTheLine, Quick)
	{
		auto shaper = TextShaper{};

		// A logically contiguous range that straddles a direction change is visually discontiguous,
		// so DirectWrite may return several rectangles. All of them must lie inside the line.
		auto const mixed = std::string("abc\u05E9\u05DC\u05D5def");
		auto const width = WidthOf(shaper, mixed);
		auto const rects = shaper.RangeRects(kFamily, kSize, mixed, 1, 8, 32);

		PR_EXPECT(!rects.empty());
		for (auto const& r : rects)
		{
			PR_EXPECT(r.w > 0.0f && r.h > 0.0f);
			PR_EXPECT(r.x >= -1.0f && r.x + r.w <= width + 1.0f);
		}
	}

	PRUnitTest(RangeRectangleCountIsBoundedByTheCallersCapacity, Quick)
	{
		auto shaper = TextShaper{};

		// The renderer's decoration budget is fixed, so the shaper must never return more
		// rectangles than the caller can draw.
		auto const mixed = std::string("abc\u05E9\u05DC\u05D5def\u05E9\u05DCghi");
		auto const rects = shaper.RangeRects(kFamily, kSize, mixed, 0, static_cast<std::uint32_t>(mixed.size()), 2);
		PR_EXPECT(rects.size() <= 2);
	}

	PRUnitTest(ShapedRunsAndCaretGeometryAgreeOnTheSameLayout, Quick)
	{
		auto shaper = TextShaper{};
		auto const text = std::string("agreement");

		// The caret at the end of the string must coincide with the total advance the renderer
		// uses to position the run; a disagreement here is exactly the bug where the caret drifts
		// away from the drawn glyphs.
		auto glyphs = std::vector<ShapedGlyph>{};
		auto const total = shaper.Shape(kFamily, kSize, 1.0f, text, glyphs);
		auto const caret = shaper.CaretAt(kFamily, kSize, text, static_cast<std::uint32_t>(text.size()));

		PR_EXPECT(!glyphs.empty());
		PR_EXPECT(std::abs(caret.x - total) < 1.0f);

		// The selection rectangle for the whole string covers the same extent.
		auto const rects = shaper.RangeRects(kFamily, kSize, text, 0, static_cast<std::uint32_t>(text.size()), 32);
		PR_EXPECT(rects.size() == 1);
		PR_EXPECT(std::abs((rects[0].x + rects[0].w) - total) < 1.0f);
	}

	PRUnitTest(FontPolicyReportsMissingAssetForAnUnresolvableFamilyRatherThanSubstituting, Quick)
	{
		auto shaper = TextShaper{};
		auto glyphs = std::vector<ShapedGlyph>{};

		// Silently substituting an unrelated face would make a missing font resource invisible to
		// the application; every entry point that resolves a family must fail the same way.
		PR_THROWS(shaper.Shape("NoSuchFontFamilyEverInstalled", kSize, 1.0f, "abc", glyphs), EngineException);
		PR_THROWS(shaper.CaretAt("NoSuchFontFamilyEverInstalled", kSize, "abc", 0), EngineException);
		PR_THROWS(shaper.OffsetFromPoint("NoSuchFontFamilyEverInstalled", kSize, "abc", 0.0f, 0.0f), EngineException);
		PR_THROWS(shaper.RangeRects("NoSuchFontFamilyEverInstalled", kSize, "abc", 0, 3, 32), EngineException);

		auto ascent = 0.0f;
		auto descent = 0.0f;
		PR_THROWS(shaper.Metrics("NoSuchFontFamilyEverInstalled", kSize, ascent, descent), EngineException);
	}

	PRUnitTest(FontPolicyUsesTheBuiltInFallbackOnlyWhenNoFamilyWasRequested, Quick)
	{
		auto shaper = TextShaper{};

		// An empty family means "the control specified no font", which resolves to the documented
		// built-in fallback. It must produce the identical layout to naming that fallback outright.
		auto const text = std::string("fallback");
		auto const unnamed = WidthOf(shaper, text);

		auto explicit_glyphs = std::vector<ShapedGlyph>{};
		auto const named = shaper.Shape("", kSize, 1.0f, text, explicit_glyphs);
		PR_EXPECT(named > 0.0f);
		PR_EXPECT(std::abs(named - unnamed) < 0.01f);
	}

	PRUnitTest(LayoutMeasurementsAreStableAcrossRepeatedAndInterleavedQueries, Quick)
	{
		// The layout cache is bounded and evicts, so a cache miss must reproduce the same answer a
		// cache hit gave. This is what "deterministic when the same font resource is available"
		// means in practice.
		auto shaper = TextShaper{};
		auto const a = std::string("stability check");
		auto const b = std::string("\u05E9\u05DC\u05D5\u05DD");

		auto const a_first = shaper.CaretAt(kFamily, kSize, a, 5);
		auto const b_first = shaper.CaretAt(kFamily, kSize, b, 2);

		// Push several other layouts through to force eviction of the originals.
		for (auto i = 0; i != 12; ++i)
			WidthOf(shaper, "filler " + std::to_string(i));

		auto const a_again = shaper.CaretAt(kFamily, kSize, a, 5);
		auto const b_again = shaper.CaretAt(kFamily, kSize, b, 2);

		PR_EXPECT(a_again.x == a_first.x);
		PR_EXPECT(a_again.height == a_first.height);
		PR_EXPECT(b_again.x == b_first.x);
		PR_EXPECT(b_again.is_rtl == b_first.is_rtl);
	}

	PRUnitTest(ComplexScriptClustersAreShapedAsSingleUnitsByDirectWrite, Quick)
	{
		auto shaper = TextShaper{};

		// A base plus combining mark, an emoji ZWJ sequence and a flag pair each occupy one caret
		// step, and hit-testing anywhere inside them lands on their boundaries only.
		auto const text = std::string("e\u0301\U0001F468\u200D\U0001F469\U0001F1F3\U0001F1FF");
		auto const width = WidthOf(shaper, text);
		PR_EXPECT(width > 0.0f);

		for (auto x = 0.0f; x < width; x += 0.5f)
		{
			auto const hit = shaper.OffsetFromPoint(kFamily, kSize, text, x, 0.0f);
			PR_EXPECT(IsGraphemeBoundary(text, hit));
		}
	}

	PRUnitTest(CaretOffsetsInsideAClusterAreSnappedToItsBoundary, Quick)
	{
		auto shaper = TextShaper{};

		// A caller that hands over a mid-cluster offset must still get a drawable caret rather
		// than an exception or a caret between the halves of a surrogate pair.
		auto const text = std::string("a\U0001F600b");
		auto const at_boundary = shaper.CaretAt(kFamily, kSize, text, 1);
		auto const inside = shaper.CaretAt(kFamily, kSize, text, 3);
		PR_EXPECT(inside.x == at_boundary.x);
	}
}
