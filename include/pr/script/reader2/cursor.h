//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - a pull-based, UTF-8 validating byte cursor.
#pragma once
#include <vector>
#include <string_view>
#include <algorithm>
#include <cstring>
#include <cctype>
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/script/reader2/input.h"

namespace pr::script::v2
{
	// True when a location carries no caller-supplied source identity or position metadata.
	inline bool IsDefaultLocation(Loc const& loc)
	{
		return
			loc.Filepath().empty() &&
			loc.FileSize() == 0 &&
			loc.Pos() == 0 &&
			loc.LinePos() == 0 &&
			loc.Line() == 1 &&
			loc.Col() == 1;
	}

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

		// Memory inputs remain borrowed in place; streamed inputs use 'm_win'.
		std::string_view m_memory;
		bool m_is_memory;

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

		// Remaining bytes in the UTF-8 scalar whose lead byte was already validated.
		int m_continuations;
		size_t m_peak_retained_bytes;

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
			, m_memory()
			, m_is_memory(false)
			, m_win()
			, m_pos(0)
			, m_input_exhausted(false)
			, m_loc(IsDefaultLocation(loc) ? Loc(m_input->Filepath()) : loc)
			, m_continuations(0)
			, m_peak_retained_bytes()
		{
			// Preserve caller-owned memory as a zero-copy byte range.
			if (auto memory = dynamic_cast<MemoryInput const*>(m_input.get()))
			{
				m_memory = memory->m_data;
				m_is_memory = true;
				m_input_exhausted = true;
			}
			else
			{
				m_win.reserve(BlockSize);
				m_peak_retained_bytes = m_win.capacity() * sizeof(char);
			}

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

		// True when this cursor borrows one complete caller-owned memory range.
		bool IsMemory() const noexcept
		{
			return m_is_memory;
		}

		// Return bytes currently retained by this cursor's transport window.
		size_t RetainedBytes() const noexcept
		{
			return m_win.capacity() * sizeof(char);
		}

		// Return the largest transport-window capacity observed by this cursor.
		size_t PeakRetainedBytes() const noexcept
		{
			return m_peak_retained_bytes;
		}

		// Look ahead 'i' bytes without consuming, refilling the window as needed.
		// Returns 0 (the same sentinel as an ordinary null-terminated string) once
		// past the end of the input.
		char PeekByte(size_t i)
		{
			if (m_is_memory)
				return m_pos + i < m_memory.size() ? m_memory[m_pos + i] : 0;

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

		// Return the next 'count' bytes as one borrowed span. The view remains
		// valid only until this cursor is advanced or another refill is requested.
		std::string_view View(size_t count)
		{
			if (count == 0)
				return {};

			if (m_is_memory)
				return m_pos + count <= m_memory.size() ? m_memory.substr(m_pos, count) : std::string_view{};

			PeekByte(count - 1);
			if (m_pos + count > m_win.size())
				return {};

			return std::string_view(m_win.data() + m_pos, count);
		}

		// Return the currently contiguous unread bytes, refilling an empty stream window once.
		std::string_view RemainingView()
		{
			if (m_is_memory)
				return m_memory.substr(m_pos);

			if (m_pos == m_win.size() && !m_input_exhausted)
				Refill();
			if (m_pos == m_win.size())
				return {};

			return std::string_view(m_win.data() + m_pos, m_win.size() - m_pos);
		}

		// Consume a caller-validated ASCII run with one location update.
		void AdvanceAscii(size_t count)
		{
			assert(m_continuations == 0);
			auto text = View(count);
			assert(text.size() == count);
			assert(std::all_of(text.begin(), text.end(), [](char ch) { return static_cast<unsigned char>(ch) < 0x80; }));
			m_loc.IncAscii(text);
			m_pos += count;
			Compact();
		}

		// Consume and validate a contiguous UTF-8 run while advancing decoded
		// character locations in ASCII batches.
		void AdvanceUtf8(size_t count)
		{
			assert(m_continuations == 0);
			auto text = View(count);
			assert(text.size() == count);
			for (size_t i = 0; i != text.size();)
			{
				// Ordinary ASCII runs update line and column state in bulk.
				auto ascii_end = i;
				for (; ascii_end != text.size() && static_cast<unsigned char>(text[ascii_end]) < 0x80; ++ascii_end) {}
				if (ascii_end != i)
				{
					m_loc.IncAscii(text.substr(i, ascii_end - i));
					i = ascii_end;
					continue;
				}

				// Validate one non-ASCII scalar before advancing its decoded location.
				auto lead = static_cast<unsigned char>(text[i]);
				auto len = ValidateUtf8(lead, [&](size_t offset)
				{
					return i + offset < text.size() ? static_cast<unsigned char>(text[i + offset]) : 0;
				});
				m_loc.inc(static_cast<char>(lead));
				i += static_cast<size_t>(len);
			}
			m_pos += count;
			Compact();
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
				m_pos += 3;
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

		// Validate one complete UTF-8 scalar using caller-provided byte lookahead.
		template <typename Peek> int ValidateUtf8(unsigned char lead, Peek&& peek) const
		{
			auto len = Utf8SeqLen(lead);
			if (len == 0)
				throw ScriptException(EResult::WrongEncoding, m_loc, "invalid UTF-8 lead byte");
			for (int k = 1; k != len; ++k)
			{
				auto byte = peek(static_cast<size_t>(k));
				if ((byte & 0xC0) != 0x80)
					throw ScriptException(EResult::WrongEncoding, m_loc, "invalid UTF-8 continuation byte");
			}

			// Exclude overlong scalars, UTF-16 surrogates, and values beyond U+10FFFF.
			auto second = len != 1 ? peek(1) : 0;
			if ((lead == 0xE0 && second < 0xA0) ||
				(lead == 0xED && second > 0x9F) ||
				(lead == 0xF0 && second < 0x90) ||
				(lead == 0xF4 && second > 0x8F))
				throw ScriptException(EResult::WrongEncoding, m_loc, "invalid UTF-8 scalar value");

			return len;
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
			m_peak_retained_bytes = std::max(m_peak_retained_bytes, RetainedBytes());
		}

		// Drop the already-consumed prefix of the window once it grows large enough
		// to be worth reclaiming, keeping the cursor's memory footprint bounded.
		void Compact()
		{
			if (m_is_memory || m_pos < CompactThreshold)
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
			if (continuation)
			{
				if (m_continuations == 0)
					throw ScriptException(EResult::WrongEncoding, m_loc, "unexpected UTF-8 continuation byte");

				--m_continuations;
			}
			else
			{
				// Validate that a multi-byte lead byte is followed by the expected
				// number of well-formed continuation bytes before accepting it.
				auto len = ValidateUtf8(ub, [&](size_t offset)
				{
					return static_cast<unsigned char>(PeekByte(offset));
				});
				m_continuations = len - 1;

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
