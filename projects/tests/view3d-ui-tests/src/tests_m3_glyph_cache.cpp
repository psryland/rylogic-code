//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M3 white-box tests (implementation-plan.md section 9.2/9.4): the CPU-only, deterministic glyph
// atlas allocator the M3 renderer uses to bound its glyph cache growth. Unlike every M0-M2 test
// file, this one includes an internal, non-ABI header (glyph_cache.h) directly rather than going
// through the public dynamic facade (pr/view3d-ui/view3d-ui.h), because GlyphCache's placement
// bookkeeping has no public ABI surface at all - it is a pure implementation detail the renderer
// alone consumes. This is deliberately white-box, in contrast to every other test file in this
// project; view3d-ui-tests.vcxproj links view3d-ui.vcxproj's static library directly to make this
// possible without disturbing the DLL's own dependency-minimal public surface.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "glyph_cache.h"

namespace pr::view3d::ui::tests
{
	PRUnitTest(GlyphCacheAcquireReturnsSameSlotForRepeatedKey, Quick)
	{
		auto cache = GlyphCache(64, 4, 64u * 64u * 4u);

		auto slot_a = GlyphSlot{};
		auto new_page_a = false;
		PR_EXPECT(cache.Acquire(GlyphKey{1, 1}, 8, 8, 0, 0, slot_a, new_page_a) == EStatus::Success);
		PR_EXPECT(new_page_a);
		PR_EXPECT(slot_a.page == 0);

		// A second Acquire for the identical key must return the same placement without touching
		// the packer again (an unrelated glyph would otherwise start a new shelf at a different
		// position), and must not report a fresh page allocation the second time.
		auto slot_b = GlyphSlot{};
		auto new_page_b = false;
		PR_EXPECT(cache.Acquire(GlyphKey{1, 1}, 8, 8, 0, 0, slot_b, new_page_b) == EStatus::Success);
		PR_EXPECT(!new_page_b);
		PR_EXPECT(slot_b.page == slot_a.page);
		PR_EXPECT(slot_b.pixel_x == slot_a.pixel_x);
		PR_EXPECT(slot_b.pixel_y == slot_a.pixel_y);
	}

	PRUnitTest(GlyphCacheFindReflectsOnlyAlreadyPlacedGlyphs, Quick)
	{
		auto cache = GlyphCache(64, 4, 64u * 64u * 4u);
		PR_EXPECT(cache.Find(GlyphKey{7, 3}) == nullptr);

		auto slot = GlyphSlot{};
		auto new_page = false;
		PR_EXPECT(cache.Acquire(GlyphKey{7, 3}, 4, 4, -1, -2, slot, new_page) == EStatus::Success);

		auto const* found = cache.Find(GlyphKey{7, 3});
		PR_EXPECT(found != nullptr);
		PR_EXPECT(found->page == slot.page);
		PR_EXPECT(found->origin_x_px == -1);
		PR_EXPECT(found->origin_y_px == -2);
	}

	PRUnitTest(GlyphCacheDistinctGlyphsShareOnePageUntilItFills, Quick)
	{
		// A 16x16 page can hold exactly four 8x8 glyphs shelf-packed 2x2; placing a fifth distinct
		// glyph must therefore allocate a second page rather than silently overwriting an existing
		// placement (implementation-plan.md's "never grow unbounded, never overwrite" contract).
		auto cache = GlyphCache(16, 4, 16u * 16u * 4u);
		for (auto i = std::uint32_t{}; i != 4; ++i)
		{
			auto slot = GlyphSlot{};
			auto new_page = false;
			PR_EXPECT(cache.Acquire(GlyphKey{0, i}, 8, 8, 0, 0, slot, new_page) == EStatus::Success);
			PR_EXPECT(slot.page == 0);
		}
		PR_EXPECT(cache.PageCount() == 1);

		auto slot = GlyphSlot{};
		auto new_page = false;
		PR_EXPECT(cache.Acquire(GlyphKey{0, 4}, 8, 8, 0, 0, slot, new_page) == EStatus::Success);
		PR_EXPECT(new_page);
		PR_EXPECT(slot.page == 1);
		PR_EXPECT(cache.PageCount() == 2);
	}

	PRUnitTest(GlyphCacheAcquireReturnsResourceLimitOnceMaxPagesIsReached, Quick)
	{
		// max_pages == 0 forces ResourceLimit on the very first never-before-seen glyph,
		// deterministically and without depending on any pixel-packing arithmetic - this is
		// exactly the bounded-degradation path Renderer::PrepareText relies on (renderer.cpp).
		auto cache = GlyphCache(64, 0, 64u * 64u * 4u);

		auto slot = GlyphSlot{};
		auto new_page = false;
		PR_EXPECT(cache.Acquire(GlyphKey{1, 1}, 8, 8, 0, 0, slot, new_page) == EStatus::ResourceLimit);
		PR_EXPECT(!new_page);
		PR_EXPECT(cache.PageCount() == 0);
		PR_EXPECT(cache.MaxPages() == 0);
	}

	PRUnitTest(GlyphCacheByteBudgetTightensAnExplicitlyLargerPageCount, Quick)
	{
		// Config's two bounds are jointly enforced (the tighter one wins): requesting up to 8
		// pages of a 32x32 page (1024 bytes/page) but only budgeting 2048 bytes must cap this
		// cache at exactly 2 pages, matching GlyphCache's constructor contract (glyph_cache.h).
		auto cache = GlyphCache(32, 8, 2048);
		PR_EXPECT(cache.MaxPages() == 2);

		for (auto i = std::uint32_t{}; i != 2; ++i)
		{
			auto slot = GlyphSlot{};
			auto new_page = false;
			// One 32x32 glyph fills an entire page on its own, forcing a fresh page each time.
			PR_EXPECT(cache.Acquire(GlyphKey{0, i}, 32, 32, 0, 0, slot, new_page) == EStatus::Success);
		}
		PR_EXPECT(cache.PageCount() == 2);
		PR_EXPECT(cache.BytesUsed() == 2048);

		auto slot = GlyphSlot{};
		auto new_page = false;
		PR_EXPECT(cache.Acquire(GlyphKey{0, 2}, 32, 32, 0, 0, slot, new_page) == EStatus::ResourceLimit);
	}

	PRUnitTest(GlyphCacheGlyphLargerThanPageIsAlwaysRejected, Quick)
	{
		// A glyph that cannot fit even a fresh, entirely empty page can never be placed regardless
		// of how many pages remain available; this must fail without consuming a page (a
		// pathologically huge font size should not silently exhaust the atlas budget).
		auto cache = GlyphCache(16, 4, 16u * 16u * 4u);

		auto slot = GlyphSlot{};
		auto new_page = false;
		PR_EXPECT(cache.Acquire(GlyphKey{0, 0}, 17, 8, 0, 0, slot, new_page) == EStatus::ResourceLimit);
		PR_EXPECT(cache.PageCount() == 0);
	}

	PRUnitTest(GlyphCacheResetDiscardsEveryPageAndSlot, Quick)
	{
		auto cache = GlyphCache(16, 4, 16u * 16u * 4u);
		auto slot = GlyphSlot{};
		auto new_page = false;
		PR_EXPECT(cache.Acquire(GlyphKey{0, 0}, 8, 8, 0, 0, slot, new_page) == EStatus::Success);
		PR_EXPECT(cache.PageCount() == 1);

		cache.Reset();
		PR_EXPECT(cache.PageCount() == 0);
		PR_EXPECT(cache.Find(GlyphKey{0, 0}) == nullptr);

		// A device-loss reset must still allow the cache to be reused from scratch afterwards.
		PR_EXPECT(cache.Acquire(GlyphKey{0, 0}, 8, 8, 0, 0, slot, new_page) == EStatus::Success);
		PR_EXPECT(new_page);
		PR_EXPECT(cache.PageCount() == 1);
	}
}
