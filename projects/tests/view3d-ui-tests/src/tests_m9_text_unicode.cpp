//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M9 - the pure Unicode text-unit seam (src/text_unicode.h). These tests need no engine, no
// device and no DirectWrite: they pin the grapheme-cluster, word-boundary and UTF-conversion
// contracts that every M9 editing operation is built on.
#include "pr/common/unittests.h"
#include "text_unicode.h"
#include <string>
#include <string_view>
#include <vector>

namespace pr::view3d::ui::tests
{
	namespace
	{
		// The grapheme cluster byte-lengths of 'text', in order. Expressing expectations this way
		// keeps each case readable as "these are the user-perceived characters".
		std::vector<std::uint32_t> ClusterLengths(std::string_view text)
		{
			auto boundaries = GraphemeBoundaries(text);
			auto lengths = std::vector<std::uint32_t>{};
			for (auto i = std::size_t(1); i < boundaries.size(); ++i)
				lengths.push_back(boundaries[i] - boundaries[i - 1]);

			return lengths;
		}
	}

	PRUnitTest(GraphemeClustersKeepSurrogatePairAstralCharactersWhole, Quick)
	{
		// U+1F600 GRINNING FACE is four UTF-8 bytes and one grapheme; scalar- or byte-based
		// editing would split it.
		auto const text = std::string("a\U0001F600b");
		PR_EXPECT(text.size() == 6);
		PR_EXPECT(GraphemeCount(text) == 3);
		PR_EXPECT(ClusterLengths(text) == std::vector<std::uint32_t>({ 1, 4, 1 }));

		// Moving right from the 'a' skips the whole emoji, and moving back returns to it exactly.
		PR_EXPECT(NextGraphemeBoundary(text, 1) == 5);
		PR_EXPECT(PrevGraphemeBoundary(text, 5) == 1);

		// An offset landing inside the emoji is never a valid caret position.
		PR_EXPECT(!IsGraphemeBoundary(text, 2));
		PR_EXPECT(!IsGraphemeBoundary(text, 3));
		PR_EXPECT(ClampToGraphemeBoundary(text, 3) == 1);
	}

	PRUnitTest(GraphemeClustersAbsorbCombiningMarksIntoTheirBaseCharacter, Quick)
	{
		// "e" + U+0301 COMBINING ACUTE ACCENT is one grapheme (GB9), as is a base with two marks.
		auto const text = std::string("e\u0301x\u0300\u0323");
		PR_EXPECT(GraphemeCount(text) == 2);
		PR_EXPECT(ClusterLengths(text) == std::vector<std::uint32_t>({ 3, 5 }));
		PR_EXPECT(NextGraphemeBoundary(text, 0) == 3);
		PR_EXPECT(PrevGraphemeBoundary(text, static_cast<std::uint32_t>(text.size())) == 3);
	}

	PRUnitTest(GraphemeClustersAbsorbVariationSelectorsAndKeycapSequences, Quick)
	{
		// U+FE0F VARIATION SELECTOR-16 is Extend, so the emoji presentation form stays one unit.
		auto const heart = std::string("\u2764\uFE0F");
		PR_EXPECT(GraphemeCount(heart) == 1);

		// A keycap sequence is digit + VS16 + U+20E3, all Extend after the base.
		auto const keycap = std::string("1\uFE0F\u20E3");
		PR_EXPECT(GraphemeCount(keycap) == 1);
		PR_EXPECT(NextGraphemeBoundary(keycap, 0) == keycap.size());
	}

	PRUnitTest(GraphemeClustersKeepEmojiZeroWidthJoinerSequencesWhole, Quick)
	{
		// Family: man + ZWJ + woman + ZWJ + girl. GB11 joins pictographic-ZWJ-pictographic runs.
		auto const family = std::string("\U0001F468\u200D\U0001F469\u200D\U0001F467");
		PR_EXPECT(family.size() == 18);
		PR_EXPECT(GraphemeCount(family) == 1);
		PR_EXPECT(NextGraphemeBoundary(family, 0) == family.size());
		PR_EXPECT(PrevGraphemeBoundary(family, static_cast<std::uint32_t>(family.size())) == 0);

		// A ZWJ between non-pictographics does not join, so GB11 is not over-applied.
		auto const letters = std::string("a\u200Db");
		PR_EXPECT(GraphemeCount(letters) == 2);
	}

	PRUnitTest(GraphemeClustersPairRegionalIndicatorsByParity, Quick)
	{
		// Two flags: each is exactly two regional indicators (GB12/GB13), so four RIs are two
		// graphemes rather than one run of four.
		auto const flags = std::string("\U0001F1F3\U0001F1FF\U0001F1E6\U0001F1FA");
		PR_EXPECT(GraphemeCount(flags) == 2);
		PR_EXPECT(ClusterLengths(flags) == std::vector<std::uint32_t>({ 8, 8 }));

		// An odd trailing indicator stands alone rather than joining the preceding pair.
		auto const odd = std::string("\U0001F1F3\U0001F1FF\U0001F1E6");
		PR_EXPECT(GraphemeCount(odd) == 2);
		PR_EXPECT(ClusterLengths(odd) == std::vector<std::uint32_t>({ 8, 4 }));
	}

	PRUnitTest(GraphemeClustersTreatCarriageReturnLineFeedAsOneLogicalBreak, Quick)
	{
		// GB3 keeps CRLF together so a single Backspace removes the whole line break.
		auto const text = std::string("a\r\nb");
		PR_EXPECT(GraphemeCount(text) == 3);
		PR_EXPECT(ClusterLengths(text) == std::vector<std::uint32_t>({ 1, 2, 1 }));
		PR_EXPECT(!IsGraphemeBoundary(text, 2));
		PR_EXPECT(PrevGraphemeBoundary(text, 3) == 1);

		// A lone CR or LF is still its own cluster (GB4/GB5).
		PR_EXPECT(GraphemeCount(std::string("a\nb")) == 3);
		PR_EXPECT(GraphemeCount(std::string("a\rb")) == 3);
	}

	PRUnitTest(GraphemeClustersKeepHangulSyllableJamoSequencesWhole, Quick)
	{
		// L + V + T composes one syllable under GB6-GB8.
		auto const jamo = std::string("\u1100\u1161\u11A8");
		PR_EXPECT(GraphemeCount(jamo) == 1);

		// A precomposed LVT syllable followed by a trailing jamo also stays whole.
		auto const lvt = std::string("\uAC01\u11A8");
		PR_EXPECT(GraphemeCount(lvt) == 1);
	}

	PRUnitTest(GraphemeOffsetAtAndCountAgreeAcrossTheWholeString, Quick)
	{
		auto const text = std::string("a\u0301\U0001F600\r\n\U0001F1F3\U0001F1FFz");
		auto const count = GraphemeCount(text);
		PR_EXPECT(count == 5);

		// Index-to-offset is total over [0, count] and always lands on a boundary.
		for (auto i = std::uint32_t(0); i <= count; ++i)
			PR_EXPECT(IsGraphemeBoundary(text, GraphemeOffsetAt(text, i)));

		PR_EXPECT(GraphemeOffsetAt(text, 0) == 0);
		PR_EXPECT(GraphemeOffsetAt(text, count) == text.size());

		// An out-of-range index saturates at the end rather than reading past it.
		PR_EXPECT(GraphemeOffsetAt(text, count + 10) == text.size());
	}

	PRUnitTest(WordBoundariesSkipRunsOfLikeCharactersForControlArrowMovement, Quick)
	{
		auto const text = std::string("hello, brave  new world");

		// Ctrl+Right stops at each change of character class, so the comma after "hello" is its own
		// stop before the caret reaches "brave".
		PR_EXPECT(NextWordBoundary(text, 0) == 5);
		PR_EXPECT(NextWordBoundary(text, 5) == 7);
		PR_EXPECT(NextWordBoundary(text, 7) == 14);
		PR_EXPECT(NextWordBoundary(text, 14) == 18);

		// Ctrl+Left moves to the start of the preceding word, not one character back, and steps
		// over a whole whitespace run in one move.
		auto const end = static_cast<std::uint32_t>(text.size());
		PR_EXPECT(PrevWordBoundary(text, end) == 18);
		PR_EXPECT(PrevWordBoundary(text, 18) == 14);
		PR_EXPECT(PrevWordBoundary(text, 14) == 7);

		// The extremes are fixed points so repeated movement terminates.
		PR_EXPECT(PrevWordBoundary(text, 0) == 0);
		PR_EXPECT(NextWordBoundary(text, end) == end);
	}

	PRUnitTest(WordBoundariesAlwaysLandOnGraphemeBoundaries, Quick)
	{
		auto const text = std::string("a\u0301b \U0001F600\U0001F600 cd");
		for (auto offset = std::uint32_t(0); offset <= text.size(); ++offset)
		{
			// Word movement is defined only over caret positions, so both directions must produce
			// offsets the caret can legally occupy even when started mid-cluster.
			PR_EXPECT(IsGraphemeBoundary(text, NextWordBoundary(text, offset)));
			PR_EXPECT(IsGraphemeBoundary(text, PrevWordBoundary(text, offset)));
		}
	}

	PRUnitTest(Utf8ValidationRejectsMalformedSequencesRatherThanSubstituting, Quick)
	{
		PR_EXPECT(Utf8Validate("plain ascii"));
		PR_EXPECT(Utf8Validate("\u00e9\u4e2d\U0001F600"));

		// Truncated, orphaned-continuation, overlong and surrogate encodings are all rejected.
		PR_EXPECT(!Utf8Validate(std::string_view("\xE4\xB8", 2)));
		PR_EXPECT(!Utf8Validate(std::string_view("\x80", 1)));
		PR_EXPECT(!Utf8Validate(std::string_view("\xC0\xAF", 2)));
		PR_EXPECT(!Utf8Validate(std::string_view("\xED\xA0\x80", 3)));
		PR_EXPECT(!Utf8Validate(std::string_view("\xF5\x80\x80\x80", 4)));
	}

	PRUnitTest(UtfConversionsRoundTripAndFailClosedOnMalformedInput, Quick)
	{
		auto const utf8 = std::string("a\u00e9\u4e2d\U0001F600");
		auto wide = std::wstring{};
		PR_EXPECT(Utf8ToUtf16(utf8, wide));
		PR_EXPECT(wide.size() == 5);

		auto back = std::string{};
		PR_EXPECT(Utf16ToUtf8(wide, back));
		PR_EXPECT(back == utf8);

		// A malformed input clears the output and reports failure; it never yields U+FFFD, because
		// silently substituting would let corrupt text reach the durable document.
		auto out = std::string("stale");
		PR_EXPECT(!Utf16ToUtf8(std::wstring_view(L"\xD83D", 1), out));
		PR_EXPECT(out.empty());

		auto out_wide = std::wstring(L"stale");
		PR_EXPECT(!Utf8ToUtf16(std::string_view("\xE4\xB8", 2), out_wide));
		PR_EXPECT(out_wide.empty());
	}

	PRUnitTest(Utf8AppendAndDecodeAreInverseOverEveryScalarRangeBoundary, Quick)
	{
		// One representative from each UTF-8 length class, plus the boundaries between them.
		auto const scalars = std::vector<char32_t>{ 0x00, 0x41, 0x7F, 0x80, 0x7FF, 0x800, 0xD7FF, 0xE000, 0xFFFF, 0x10000, 0x10FFFF };
		for (auto cp : scalars)
		{
			auto encoded = std::string{};
			Utf8Append(encoded, cp);
			PR_EXPECT(Utf8Validate(encoded));

			auto decoded = char32_t{};
			auto length = std::uint32_t{};
			PR_EXPECT(Utf8Decode(encoded, 0, decoded, length));
			PR_EXPECT(decoded == cp);
			PR_EXPECT(length == encoded.size());
		}
	}

	PRUnitTest(GraphemeBreakClassificationTablesAreSortedAndDisjoint, Quick)
	{
		// GraphemeBreakOf and CharClassOf binary-search sorted static range tables. A range that is
		// out of order silently becomes unreachable, so probe representatives spread across the
		// whole table rather than a few adjacent ones: a sort break past any probe would show up
		// here as an 'Other' classification.
		struct Probe { char32_t cp; EGraphemeBreak brk; };
		auto const probes = std::vector<Probe>
		{
			{ 0x0000, EGraphemeBreak::Control },
			{ 0x000D, EGraphemeBreak::CR },
			{ 0x000A, EGraphemeBreak::LF },
			{ 0x0041, EGraphemeBreak::Other },
			{ 0x00AD, EGraphemeBreak::Control },
			{ 0x0300, EGraphemeBreak::Extend },
			{ 0x0903, EGraphemeBreak::SpacingMark },
			{ 0x0600, EGraphemeBreak::Prepend },
			{ 0x1100, EGraphemeBreak::HangulL },
			{ 0x1161, EGraphemeBreak::HangulV },
			{ 0x11A8, EGraphemeBreak::HangulT },
			{ 0xAC00, EGraphemeBreak::HangulLV },
			{ 0xAC01, EGraphemeBreak::HangulLVT },
			{ 0x200D, EGraphemeBreak::ZWJ },
			{ 0x200B, EGraphemeBreak::Control },
			{ 0x20E3, EGraphemeBreak::Extend },
			{ 0x2764, EGraphemeBreak::ExtendedPictographic },
			{ 0xFE0F, EGraphemeBreak::Extend },
			{ 0xFF21, EGraphemeBreak::Other },
			{ 0x1F1E6, EGraphemeBreak::RegionalIndicator },
			{ 0x1F1FF, EGraphemeBreak::RegionalIndicator },
			{ 0x1F600, EGraphemeBreak::ExtendedPictographic },
			{ 0x1F9D1, EGraphemeBreak::ExtendedPictographic },
			{ 0xE0100, EGraphemeBreak::Extend },
			{ 0x10FFFF, EGraphemeBreak::Other },
		};
		for (auto const& probe : probes)
			PR_EXPECT(GraphemeBreakOf(probe.cp) == probe.brk);

		// The word-movement classes must likewise stay reachable across their whole range.
		PR_EXPECT(CharClassOf(U' ') == ECharClass::Whitespace);
		PR_EXPECT(CharClassOf(U'\t') == ECharClass::Whitespace);
		PR_EXPECT(CharClassOf(0x00A0) == ECharClass::Whitespace);
		PR_EXPECT(CharClassOf(0x3000) == ECharClass::Whitespace);
		PR_EXPECT(CharClassOf(U'a') == ECharClass::Word);
		PR_EXPECT(CharClassOf(U'7') == ECharClass::Word);
		PR_EXPECT(CharClassOf(0x4E2D) == ECharClass::Word);
		PR_EXPECT(CharClassOf(0x1F600) == ECharClass::Word);
		PR_EXPECT(CharClassOf(U',') == ECharClass::Punctuation);
		PR_EXPECT(CharClassOf(U'.') == ECharClass::Punctuation);

		// Classification must also be total: every code point yields exactly one of the declared
		// enumerators, and every classifier agrees on which offsets are cluster boundaries.
		for (auto cp = char32_t(0); cp != 0x3000; ++cp)
		{
			auto const brk = GraphemeBreakOf(cp);
			auto const known_break =
				brk == EGraphemeBreak::Other || brk == EGraphemeBreak::CR || brk == EGraphemeBreak::LF ||
				brk == EGraphemeBreak::Control || brk == EGraphemeBreak::Extend || brk == EGraphemeBreak::ZWJ ||
				brk == EGraphemeBreak::RegionalIndicator || brk == EGraphemeBreak::Prepend ||
				brk == EGraphemeBreak::SpacingMark || brk == EGraphemeBreak::HangulL || brk == EGraphemeBreak::HangulV ||
				brk == EGraphemeBreak::HangulT || brk == EGraphemeBreak::HangulLV || brk == EGraphemeBreak::HangulLVT ||
				brk == EGraphemeBreak::ExtendedPictographic;
			PR_EXPECT(known_break);

			// A repeated call must agree with itself; a table lookup that fell through a range
			// would otherwise be indistinguishable from a deliberate 'Other'.
			PR_EXPECT(GraphemeBreakOf(cp) == brk);

			auto const cls = CharClassOf(cp);
			PR_EXPECT(cls == ECharClass::Whitespace || cls == ECharClass::Word || cls == ECharClass::Punctuation);
			PR_EXPECT(CharClassOf(cp) == cls);
		}
	}

	PRUnitTest(IndicConjunctsFormOneClusterUnderRuleGb9c, Quick)
	{
		// GB9c keeps a consonant, a virama (linker) and the following consonant together, so a
		// Devanagari conjunct is one user-perceived character. Without the rule the virama would
		// merely be an Extend and the trailing consonant would start a second cluster, letting the
		// caret land inside the conjunct.
		PR_EXPECT(IndicConjunctBreakOf(0x0915) == EIndicConjunctBreak::Consonant); // KA
		PR_EXPECT(IndicConjunctBreakOf(0x094D) == EIndicConjunctBreak::Linker);    // VIRAMA
		PR_EXPECT(IndicConjunctBreakOf(0x0020) == EIndicConjunctBreak::None);

		// क + virama + ष = क्ष, one cluster of nine UTF-8 bytes.
		auto const kshha = std::string("\u0915\u094D\u0937");
		PR_EXPECT(kshha.size() == 9);
		PR_EXPECT(GraphemeCount(kshha) == 1);
		PR_EXPECT(ClusterLengths(kshha) == std::vector<std::uint32_t>({ 9 }));
		PR_EXPECT(NextGraphemeBoundary(kshha, 0) == 9);

		// Bengali behaves the same way: ক + virama + ষ.
		auto const bengali = std::string("\u0995\u09CD\u09B7");
		PR_EXPECT(GraphemeCount(bengali) == 1);

		// A linker with no following consonant does not glue the next character on; the rule needs
		// a consonant on both sides, so a trailing space still starts its own cluster.
		auto const dangling = std::string("\u0915\u094D ");
		PR_EXPECT(GraphemeCount(dangling) == 2);

		// Two independent consonants with no linker between them are two clusters.
		auto const separate = std::string("\u0915\u0937");
		PR_EXPECT(GraphemeCount(separate) == 2);

		// Intervening non-spacing marks between the linker and the consonant are permitted.
		auto const with_mark = std::string("\u0915\u094D\u200D\u0937");
		PR_EXPECT(GraphemeCount(with_mark) == 1);
	}

	PRUnitTest(WordClassificationUsesUnicodePropertiesNotAnAsciiOnlyGuess, Quick)
	{
		// Word movement asked "is this ASCII punctuation?" before; anything else counted as a word
		// character, so a CJK full stop or a typographic quote never ended a word. Classification
		// now follows the White_Space and general punctuation/symbol categories.
		PR_EXPECT(CharClassOf(0x3002) == ECharClass::Punctuation); // IDEOGRAPHIC FULL STOP
		PR_EXPECT(CharClassOf(0x201C) == ECharClass::Punctuation); // LEFT DOUBLE QUOTATION MARK
		PR_EXPECT(CharClassOf(0x2014) == ECharClass::Punctuation); // EM DASH
		PR_EXPECT(CharClassOf(0x00BF) == ECharClass::Punctuation); // INVERTED QUESTION MARK
		PR_EXPECT(CharClassOf(0x060C) == ECharClass::Punctuation); // ARABIC COMMA
		PR_EXPECT(CharClassOf(0x2009) == ECharClass::Whitespace);  // THIN SPACE
		PR_EXPECT(CharClassOf(0x2028) == ECharClass::Whitespace);  // LINE SEPARATOR

		// Letters, digits and marks in any script remain word characters, and an underscore stays
		// one deliberately because identifiers read as a single word.
		PR_EXPECT(CharClassOf(0x05D0) == ECharClass::Word); // HEBREW ALEF
		PR_EXPECT(CharClassOf(0x0627) == ECharClass::Word); // ARABIC ALEF
		PR_EXPECT(CharClassOf(0x0915) == ECharClass::Word); // DEVANAGARI KA
		PR_EXPECT(CharClassOf(U'_') == ECharClass::Word);

		// Ctrl+Right therefore stops at a CJK sentence break rather than running to the end: the
		// word run ends before the full stop, and the punctuation run is then its own step.
		auto const cjk = std::string("\u4E2D\u6587\u3002\u82F1\u6587");
		PR_EXPECT(cjk.size() == 15);
		PR_EXPECT(NextWordBoundary(cjk, 0) == 6);
		PR_EXPECT(NextWordBoundary(cjk, 6) == 9);
		PR_EXPECT(NextWordBoundary(cjk, 9) == 15);
		PR_EXPECT(PrevWordBoundary(cjk, 15) == 9);
	}

	PRUnitTest(NearestGraphemeBoundaryRoundsToTheClosestClusterEdge, Quick)
	{
		// Pointer hit-testing lands on arbitrary byte offsets; rounding down alone would make the
		// right half of a cluster place the caret before it, so the nearest edge is used instead.
		auto const text = std::string("a\U0001F600b"); // 1 + 4 + 1 bytes
		PR_EXPECT(NearestGraphemeBoundary(text, 0) == 0);
		PR_EXPECT(NearestGraphemeBoundary(text, 1) == 1);
		PR_EXPECT(NearestGraphemeBoundary(text, 2) == 1); // nearer the emoji's start
		PR_EXPECT(NearestGraphemeBoundary(text, 4) == 5); // nearer the emoji's end
		PR_EXPECT(NearestGraphemeBoundary(text, 5) == 5);
		PR_EXPECT(NearestGraphemeBoundary(text, 99) == text.size());

		// An exact midpoint resolves to the earlier boundary, so the choice is deterministic.
		auto const pair = std::string("\u00E9"); // two bytes, one cluster
		PR_EXPECT(NearestGraphemeBoundary(pair, 1) == 0);
	}

	PRUnitTest(GraphemeNavigationOfMalformedBytesStaysTotalAndTerminating, Quick)
	{
		// Editing must not become unbounded or infinite just because a caller handed over bytes
		// that are not valid UTF-8; each malformed byte forms its own cluster.
		auto const text = std::string("a\xE4\xB8z");
		PR_EXPECT(!Utf8Validate(text));

		auto offset = std::uint32_t(0);
		auto steps = 0;
		for (; offset != text.size() && steps != 100; ++steps)
		{
			auto const next = NextGraphemeBoundary(text, offset);
			PR_EXPECT(next > offset);
			offset = next;
		}
		PR_EXPECT(offset == text.size());

		for (; offset != 0 && steps != 200; ++steps)
		{
			auto const prev = PrevGraphemeBoundary(text, offset);
			PR_EXPECT(prev < offset);
			offset = prev;
		}
		PR_EXPECT(offset == 0);

		// An offset past the end clamps to the end rather than reading out of bounds.
		PR_EXPECT(ClampToGraphemeBoundary(text, 500) == text.size());
		PR_EXPECT(NextGraphemeBoundary(text, 500) == text.size());
	}

	PRUnitTest(EmptyTextHasExactlyOneCaretPositionAndNoClusters, Quick)
	{
		auto const empty = std::string_view{};
		PR_EXPECT(GraphemeCount(empty) == 0);
		PR_EXPECT(IsGraphemeBoundary(empty, 0));
		PR_EXPECT(NextGraphemeBoundary(empty, 0) == 0);
		PR_EXPECT(PrevGraphemeBoundary(empty, 0) == 0);
		PR_EXPECT(NextWordBoundary(empty, 0) == 0);
		PR_EXPECT(PrevWordBoundary(empty, 0) == 0);
		PR_EXPECT(GraphemeBoundaries(empty).size() <= 1);
	}
}
