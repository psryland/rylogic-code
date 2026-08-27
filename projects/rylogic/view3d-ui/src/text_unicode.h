//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Pure Unicode text-unit services for the TextBox edit model (implementation-plan.md section 7.3).
// Nothing here touches Win32, DirectWrite, COM or any engine state, so every rule below is directly
// unit-testable without a window, a device or an installed IME.
//
// The edit model addresses text by UTF-8 byte offset internally, but every user-visible operation
// (caret movement, Backspace/Delete, selection extent, max-length enforcement) is expressed in
// *extended grapheme clusters* so a caret can never land inside a surrogate pair, a combining
// sequence, a variation-selector sequence, an emoji ZWJ sequence, a regional-indicator flag pair,
// or a CRLF line break.
//
// Coverage and versioning: the property tables in text_unicode.cpp are hand-maintained against
// UnicodeDataVersion below. Every UAX #29 extended-grapheme-cluster *rule* (GB1-GB13, including
// GB9c Indic conjunct clusters) is implemented, but the property *tables* the rules consult are
// deliberately partial - they cover the scripts, marks and pictographic ranges this UI can shape,
// not the complete Unicode Character Database. A code point outside those tables classifies as
// Grapheme_Cluster_Break=Other, which degrades to one cluster per code point rather than to a
// broken caret. This module therefore implements UAX #29 segmentation over a documented subset of
// the UCD; it does not claim conformance over the whole repertoire.
#pragma once
#include "pr/view3d-ui/forward.h"

namespace pr::view3d::ui
{
	// The Unicode Character Database release the property tables in text_unicode.cpp were derived
	// from. Bump this together with the tables so a behavioural change is always attributable to a
	// specific UCD revision rather than to an undated edit.
	constexpr char const* UnicodeDataVersion = "15.1.0";

	// UAX #29 Grapheme_Cluster_Break property values, plus Extended_Pictographic (needed by rule
	// GB11). 'Other' covers every code point no table below assigns, which is the correct default
	// for ordinary letters, digits, punctuation and unassigned code points.
	enum class EGraphemeBreak
	{
		Other,
		CR,
		LF,
		Control,
		Extend,
		ZWJ,
		RegionalIndicator,
		Prepend,
		SpacingMark,
		HangulL,
		HangulV,
		HangulT,
		HangulLV,
		HangulLVT,
		ExtendedPictographic,
	};

	// Indic_Conjunct_Break property values, used only by rule GB9c so that a consonant joined to a
	// following consonant through a virama (and any intervening marks) stays one cluster. 'None' is
	// the default for every code point outside the Indic conjunct scripts.
	enum class EIndicConjunctBreak
	{
		None,
		Consonant,
		Linker,
		Extend,
	};

	// Coarse character class used by Ctrl+Left/Ctrl+Right word movement. This is deliberately the
	// simple whitespace/word/punctuation classification Windows edit controls use, not UAX #29
	// word segmentation, so caret movement matches what users expect from a native text field.
	// The mapping is by Unicode general category: White_Space code points are Whitespace, code
	// points in the punctuation (P*) and symbol (S*) categories are Punctuation, and everything
	// else - letters, marks, digits and, deliberately, Extended_Pictographic emoji - is Word.
	// Emoji are grouped with Word because they are content the user types, and classifying them as
	// delimiters would make Ctrl+arrow stop inside a flag or ZWJ sequence's neighbourhood.
	enum class ECharClass
	{
		Whitespace,
		Word,
		Punctuation,
	};

	// Returns the Grapheme_Cluster_Break property of 'cp'.
	EGraphemeBreak GraphemeBreakOf(char32_t cp);

	// Returns the Indic_Conjunct_Break property of 'cp'.
	EIndicConjunctBreak IndicConjunctBreakOf(char32_t cp);

	// Returns the word-movement class of 'cp'.
	ECharClass CharClassOf(char32_t cp);

	// Decodes the UTF-8 sequence starting at 'offset'. Returns false (leaving the outputs
	// untouched) when 'offset' is at or past the end, or the bytes there are not a well-formed,
	// shortest-form, non-surrogate, in-range UTF-8 sequence.
	bool Utf8Decode(std::string_view text, std::uint32_t offset, char32_t& out_cp, std::uint32_t& out_length);

	// True if every byte of 'text' participates in a well-formed UTF-8 sequence. Used to reject
	// text arriving over the ABI before it is ever stored in an edit buffer.
	bool Utf8Validate(std::string_view text);

	// Appends 'cp' to 'out' as UTF-8. Throws EngineException(InvalidArgument) for a surrogate or
	// out-of-range scalar value, so an invalid code point can never enter an edit buffer.
	void Utf8Append(std::string& out, char32_t cp);

	// Converts between UTF-8 and UTF-16 without ever substituting a replacement character: both
	// return false for malformed input (an unpaired surrogate, an over-long sequence, a truncated
	// sequence) and leave 'out' empty, so the caller can reject the payload explicitly.
	bool Utf16ToUtf8(std::wstring_view text, std::string& out);
	bool Utf8ToUtf16(std::string_view text, std::wstring& out);

	// Returns the byte offsets of every extended-grapheme-cluster boundary in 'text', always
	// starting with 0 and ending with text.size(). An empty string yields a single 0 entry.
	// Malformed bytes are treated as one single-byte cluster each so segmentation is total.
	std::vector<std::uint32_t> GraphemeBoundaries(std::string_view text);

	// True if 'offset' is exactly on a grapheme-cluster boundary of 'text' (0 and text.size() both
	// are).
	bool IsGraphemeBoundary(std::string_view text, std::uint32_t offset);

	// Moves one grapheme cluster forward/backward from 'offset', clamped to [0, text.size()]. An
	// 'offset' that is not already on a boundary is first snapped outward in the direction of
	// travel, so a caret can never be left inside a cluster.
	std::uint32_t NextGraphemeBoundary(std::string_view text, std::uint32_t offset);
	std::uint32_t PrevGraphemeBoundary(std::string_view text, std::uint32_t offset);

	// Snaps 'offset' back to the start of the grapheme cluster containing it (or to text.size()).
	std::uint32_t ClampToGraphemeBoundary(std::string_view text, std::uint32_t offset);

	// Snaps 'offset' to whichever grapheme-cluster boundary of 'text' is closest in bytes, resolving
	// an exact tie towards the start of the cluster. Used for pointer hit testing, where landing in
	// the second half of a cluster should place the caret after it rather than before it.
	std::uint32_t NearestGraphemeBoundary(std::string_view text, std::uint32_t offset);

	// Number of extended grapheme clusters in 'text'. This is the "Unicode text unit" that
	// ControlDesc::max_text_length bounds.
	std::uint32_t GraphemeCount(std::string_view text);

	// Byte offset of the 'index'th grapheme-cluster boundary, clamped to text.size().
	std::uint32_t GraphemeOffsetAt(std::string_view text, std::uint32_t index);

	// Word-movement targets for Ctrl+Right / Ctrl+Left, evaluated over grapheme clusters. Right
	// skips the run the caret is in and then any following whitespace, landing on the start of the
	// next word; Left skips any preceding whitespace and then the run before it, landing on the
	// start of that word. Both clamp to the ends of 'text'.
	std::uint32_t NextWordBoundary(std::string_view text, std::uint32_t offset);
	std::uint32_t PrevWordBoundary(std::string_view text, std::uint32_t offset);
}
