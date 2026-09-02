//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - a pull-based, UTF-8 validating byte cursor.
#pragma once
#include <vector>
#include <string_view>
#include <cstring>
#include <cctype>
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/script/reader2/input.h"

namespace pr::script::v2
{
	// A forward-only cursor over the UTF-8 bytes produced by an 'IInput'.
	//
	// 'Cursor' is the lowest level of the reader2 stack: it owns a sliding window of
	// buffered bytes (refilled in 'BlockSize' chunks from its 'IInput'), validates
	// the UTF-8 encoding as it advances, and maintains a 'Loc' with the same
	// line/column/position semantics as the legacy 'pr::script::Loc'.
	//
	// The public interface intentionally mirrors a plain forward pointer
	// ('operator*', 'operator++', 'operator+=', 'operator[]') so that 'Cursor'
	// instances can be passed directly to the char-generic 'pr::str::Extract*'
	// family (ExtractInt, ExtractReal, ExtractBool, ExtractIdentifier, ExtractString,
	// ExtractToken, ExtractNumber, ...), reusing their exact parsing rules rather
	// than re-implementing numeric/string literal parsing from scratch.
	//
	// Because those algorithms operate purely on bytes (they only ever compare
	// against ASCII values), 'Cursor' advances one BYTE at a time. Location tracking
	// still reflects decoded Unicode characters (not raw bytes): continuation bytes
	// of a multi-byte UTF-8 sequence do not advance 'Loc's position/line/column, only
	// the lead byte of a sequence does. For pure-ASCII scripts (byte count == char
	// count) this is identical to the legacy per-decoded-character semantics.
	class Cursor
	{
		// The input supplying bytes to refill the window. Owned, so that a 'Cursor'
		// created for a pushed include file keeps that file's input source alive for
		// exactly as long as the cursor itself needs it.
		std::unique_ptr<IInput> m_input;

		// A sliding window of buffered bytes. 'm_pos' is the index of the "current"
		// character; bytes before it have already been consumed and are candidates
		// for compaction, bytes at/after it are unread lookahead.
		std::vector<char> m_win;
		size_t m_pos;

		// True once 'm_input' has reported end-of-data and the window has no more
		// buffered bytes beyond 'm_pos'.
		bool m_input_exhausted;

		// The current location (line/column/character offset) of 'm_pos'.
		Loc m_loc;

		// Once the consumed prefix of the window exceeds this many bytes, it is
		// erased. This bounds the cursor's memory to roughly one unbroken token's
		// worth of lookahead rather than growing for the lifetime of the source,
		// matching the soft-bounded-memory requirement for reader2.
		static constexpr size_t CompactThreshold = 2 * BlockSize;

	public:

		// Construct a cursor over 'input', starting at 'loc' (typically identifying
		// just the source's file path; position/line/column default to the start).
		// Takes ownership of 'input' so the cursor controls its lifetime.
		explicit Cursor(std::unique_ptr<IInput> input, Loc const& loc = Loc())
			: m_input(std::move(input))
			, m_win()
			, m_pos(0)
			, m_input_exhausted(false)
			, m_loc(loc)
		{
			m_win.reserve(BlockSize);

			// A byte-order-mark, if present, is not part of the script content.
			SkipBom();
		}

		// Cursors are cheap to move (they own a growable buffer) but not to copy,
		// since copying the buffered window is rarely what's intended.
		Cursor(Cursor const&) = delete;
		Cursor& operator =(Cursor const&) = delete;
		Cursor(Cursor&&) = default;
		Cursor& operator =(Cursor&&) = default;

		// The current location within the underlying source.
		Loc const& Location() const
		{
			return m_loc;
		}

		// True once the source is fully consumed (i.e. '*cursor == 0').
		bool AtEnd()
		{
			return PeekByte(0) == 0;
		}

		// Look ahead 'i' bytes without consuming, refilling the window as needed.
		// Returns 0 (the same sentinel as an ordinary null-terminated string) once
		// past the end of the input.
		char PeekByte(size_t i)
		{
			while (m_pos + i >= m_win.size() && !m_input_exhausted)
				Refill();

			return (m_pos + i) < m_win.size() ? m_win[m_pos + i] : 0;
		}

		// Forward-pointer-style access, so 'Cursor' satisfies the 'Ptr' concept used
		// throughout 'pr::str::Extract*'.
		char operator *()
		{
			return PeekByte(0);
		}
		char operator [](size_t i)
		{
			return PeekByte(i);
		}
		Cursor& operator ++()
		{
			AdvanceOne();
			return *this;
		}
		Cursor& operator +=(size_t n)
		{
			for (; n-- != 0;)
				AdvanceOne();

			return *this;
		}

		// Case-sensitive/insensitive match of a literal ASCII string against the
		// upcoming bytes. On a match, consumes the matched bytes when 'consume' is
		// true. Mirrors the legacy 'Src::Match' helper used by the preprocessor to
		// recognise directive keywords such as "define" or "include".
		bool Match(std::string_view s, bool consume, bool case_sensitive = true)
		{
			for (size_t i = 0; i != s.size(); ++i)
			{
				auto have = PeekByte(i);
				auto want = s[i];
				auto eq = case_sensitive
					? have == want
					: std::tolower(static_cast<unsigned char>(have)) == std::tolower(static_cast<unsigned char>(want));

				if (!eq)
					return false;
			}
			if (consume)
				*this += s.size();

			return true;
		}

	private:

		// Discard a UTF-8 byte-order-mark at the very start of the input, if present.
		void SkipBom()
		{
			char probe[3];
			for (size_t i = 0; i != 3; ++i)
				probe[i] = PeekByte(i);

			if (Utf8BomLength(std::string_view(probe, 3)) == 3)
				*this += 3;
		}

		// Determine the byte-length of the UTF-8 sequence starting with lead byte
		// 'b'. Returns 0 for a byte that can never legally start a sequence.
		static int Utf8SeqLen(unsigned char b)
		{
			if (b < 0x80) return 1;
			if ((b & 0xE0) == 0xC0) return b >= 0xC2 ? 2 : 0; // reject overlong 0xC0/0xC1 lead bytes
			if ((b & 0xF0) == 0xE0) return 3;
			if ((b & 0xF8) == 0xF0) return b <= 0xF4 ? 4 : 0; // reject lead bytes beyond the Unicode range
			return 0;
		}

		// Refill the window with the next physical block of bytes from 'm_input'.
		void Refill()
		{
			char block[BlockSize];
			auto n = m_input->Read(block, BlockSize);
			if (n == 0)
			{
				m_input_exhausted = true;
				return;
			}
			m_win.insert(m_win.end(), block, block + n);
		}

		// Drop the already-consumed prefix of the window once it grows large enough
		// to be worth reclaiming, keeping the cursor's memory footprint bounded.
		void Compact()
		{
			if (m_pos < CompactThreshold)
				return;

			m_win.erase(m_win.begin(), m_win.begin() + static_cast<ptrdiff_t>(m_pos));
			m_pos = 0;
		}

		// Advance over exactly one byte, updating 'm_loc' once per decoded Unicode
		// character (i.e. once per UTF-8 sequence, not once per byte). Validates
		// that a multi-byte sequence's lead byte is followed by well-formed
		// continuation bytes, throwing 'EResult::WrongEncoding' otherwise.
		void AdvanceOne()
		{
			auto b = PeekByte(0);
			if (b == 0)
				return; // No-op at end of input, matching Loc::inc's treatment of '\0'.

			auto ub = static_cast<unsigned char>(b);
			auto continuation = (ub & 0xC0) == 0x80;
			if (!continuation)
			{
				// Validate that a multi-byte lead byte is followed by the expected
				// number of well-formed continuation bytes before accepting it.
				auto len = Utf8SeqLen(ub);
				if (len == 0)
					throw ScriptException(EResult::WrongEncoding, m_loc, "invalid UTF-8 lead byte");

				for (int k = 1; k != len; ++k)
				{
					auto c = static_cast<unsigned char>(PeekByte(static_cast<size_t>(k)));
					if ((c & 0xC0) != 0x80)
						throw ScriptException(EResult::WrongEncoding, m_loc, "invalid UTF-8 continuation byte");
				}

				// Only the lead byte of a sequence advances the decoded-character
				// location; ASCII bytes are their own (single-byte) sequence. The raw
				// byte value is safe to forward as-is: '\n' (0x0A) and '\t' (0x09) can
				// only ever appear as single-byte ASCII lead bytes, never as the lead
				// byte of a multi-byte sequence (those all start at 0xC2 or above).
				m_loc.inc(static_cast<char>(ub));
			}

			m_pos += 1;
			Compact();
		}
	};
}
