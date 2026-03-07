//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/script/buf.h"

namespace pr::script
{
	// Base class for a stream of characters
	struct Src
	{
		// Notes:
		//  - The source is exhausted when 'Peek' returns '\0'
		//    *Careful*, only Read() returns EOF, not *src.
		//  - This class supports local buffering (through ReadAhead)
		//  - Don't make 'Read' public, that would by-pass the local buffering.
		//  - Windows strings (using wchar_t) are actually UTF-16 encoded.
		//  - The stream operates on single characters so they have to be a fixed width.
		//    Use wchar_t so that most characters are covered, any beyond that will have
		//    to be treated as encoding errors.
		//  - Src can also wrap another stream.

		static int const EOS = -1; // end of stream
		static int64_t const AllData = std::numeric_limits<int64_t>::max();

	protected:

		// A reference to a wrapped source (or this). This member should not generally be
		// used by 'Src' itself, its mainly a convenience for types that wrap a 'Src'.
		Src& m_src;

		// A local read ahead buffer
		// Note: the length of this buffer can be greater than the
		// number of characters available when 'Limit()' is used.
		string_t m_buffer;

		// Encoding of data returned from 'Read()'
		EEncoding m_enc;

		// Multi-byte read state
		std::mbstate_t m_mb;

		// Stream position (i.e. position of m_buffer[0])
		Loc m_loc;

		// Remaining characters to read.
		int64_t m_remaining;

		#if PR_DBG
		Buf<8, wchar_t> m_history;
		#endif

	protected:

		// Warning fix
		constexpr Src& This() { return *this; }

		// Constructor for character sources
		Src(EEncoding enc, Loc const& loc = Loc()) noexcept
			:m_src(This())
			,m_buffer()
			,m_enc(enc)
			,m_mb()
			,m_loc(loc)
			,m_remaining(AllData)
			#if PR_DBG
			,m_history()
			#endif
		{}

		// Constructor for source wrappers
		Src(Src& wrapped, EEncoding enc) noexcept
			:m_src(wrapped)
			,m_buffer()
			,m_enc(enc)
			,m_mb()
			,m_loc()
			,m_remaining(AllData)
			#if PR_DBG
			,m_history()
			#endif
		{}

		// Return the next byte or decoded character from the underlying stream.
		// The interpretation of what is returned from 'Read()' is based on 'm_enc'.
		// For all encodings except 'already_decode', the 'ReadAhead()' function will assume 'Read()'
		// returns bytes and convert encodings to utf-16. For these sources, returning EOS is needed
		// because 0 maybe a valid byte in the encoding. If 'm_enc' is 'already_decoded' the 'Read()'
		// is assumed to return already decoded utf-16 characters and so should never see an 'EOS'.
		virtual int Read() = 0;

	public:

		virtual ~Src() {}

		// Move construct only
		Src(Src&& rhs) noexcept
			:m_src(&rhs == &rhs.m_src ? This() : rhs.m_src)
			,m_buffer(std::move(rhs.m_buffer))
			,m_enc(rhs.m_enc)
			,m_mb(rhs.m_mb)
			,m_loc(rhs.m_loc)
			,m_remaining(rhs.m_remaining)
		{}
		Src(Src const& rhs) = delete;
		Src& operator = (Src&& rhs) = delete;
		Src& operator = (Src const& rhs) = delete;

		// The current position in the root underlying source
		virtual Loc Location() const noexcept
		{
			auto s = this;
			for (; s != &m_src; s = &s->m_src) { }
			return s->m_loc;
		}

		// Access the local cache of characters read from the source
		string_t const& Buffer() const noexcept
		{
			return m_buffer;
		}
		string_t& Buffer() noexcept
		{
			return m_buffer;
		}

		// Buffer up to (start + count) and return a string view within this range
		string_view_t Buffer(int64_t start, int64_t count, bool allow_partial = false)
		{
			auto len = ReadAhead(start + count);

			// If 'count' bytes are expected, throw if they're not available
			allow_partial |= count == AllData;
			if (!allow_partial && len < start + count)
				throw ScriptException(EResult::UnexpectedEndOfFile, Location(), Fmt("Could not buffer %d characters. End of stream reached", start + count));

			auto str = m_buffer.data();
			return
				len <= start ? string_view_t() :
				len <= start + count ? string_view_t(str + start, s_cast<size_t>(len - start)) :
				string_view_t(str + start, s_cast<size_t>(count));
		}

		// Get/Set the maximum number of characters to emit from this stream (can be less than the underlying source length)
		int64_t Limit() const
		{
			return m_remaining;
		}
		void Limit(int64_t remaining)
		{
			m_remaining = remaining >= 0 ? remaining : AllData;
		}

		// Peek
		char_t operator *()
		{
			return (*this)[0];
		}

		// Read ahead array access
		char_t operator[](int i)
		{
			auto len = ReadAhead(i + 1);
			return i < len ? m_buffer[i] : '\0';
		}

		// Advance by 1 character
		Src& operator ++()
		{
			Next();
			return *this;
		}

		// Advance by n characters
		Src& operator += (int64_t n)
		{
			Next(n);
			return *this;
		}

		// Increment to the next character
		void Next(int64_t n = 1)
		{
			if (n < 0)
				throw std::runtime_error("Cannot seek backwards");
			if (n > m_remaining)
				n = m_remaining;

			for (;;)
			{
				// Consume from the buffered characters first
				auto remove = s_cast<int>(std::min(n, s_cast<int64_t>(m_buffer.size())));
				for (int i = 0; i != remove; ++i)
				{
					#if PR_DBG
					m_history.shift(m_buffer[i]);
					#endif
					m_loc.inc(m_buffer[i]);
				}

				m_buffer.erase(0, remove);

				m_remaining -= remove;
				assert(m_remaining >= 0);

				n -= remove;
				if (n == 0)
					break;

				// Buffer and dump, since we need to read whole characters from the underlying source
				if (ReadAhead(std::min(n, 4096LL)) == 0)
					break;
			}
		}

		// Attempt to buffer 'n' characters locally. Less than 'n' characters can be buffered if EOF or Limit is hit.
		// Returns the number of characters available (a value in [0, n])
		// Do *not* use Buffer.Length as the number available, it can be greater than 'Limit'
		int64_t ReadAhead(int64_t n)
		{
			if (n < 0)
				throw std::runtime_error("Cannot read backwards");
			if (n > m_remaining)
				n = s_cast<int64_t>(m_remaining);

			for (; n > s_cast<int64_t>(m_buffer.size());)
			{
				// Ensure 'Buffer's length grows with each loop
				auto const count = m_buffer.size();

				// Read the next complete character from the underlying stream
				int ch = '\0';
				switch (m_enc)
				{
					case EEncoding::already_decoded:
					{
						// Already decoded streams should output characters and then 0s.
						// EOS occurs for invalid character encodings only.
						ch = Read();
						if (ch == EOS)
							throw std::runtime_error("Read() should not return EOS for 'already_decoded' streams.");
						break;
					}
					case EEncoding::ascii:
					case EEncoding::ascii_extended:
					{
						auto const b = Read();
						if (b == EOS) break;
						if (m_enc == EEncoding::ascii && b > 127)
							throw ScriptException(EResult::WrongEncoding, Location(), Fmt(L"Source is not an ASCII character stream. Invalid character with value %d found", b));
						else if (b > 255)
							throw ScriptException(EResult::WrongEncoding, Location(), Fmt(L"Source is not an extended ASCII character stream. Invalid character with value %d found", b));
						ch = b;
						break;
					}
					case EEncoding::utf8:
					{
						// Interpret a multi byte UTF8 character
						for (auto mb = std::mbstate_t{};;)
						{
							// Read a byte at a time until a complete multibyte character is found (or EOS)
							auto const b = Read();
							if (b == EOS) break;

							auto c = char(b);
							char16_t c16 = 0;
							auto const r = std::mbrtoc16(&c16, &c, 1, &mb);
							if (r == static_cast<size_t>(-1)) throw ScriptException(EResult::WrongEncoding, Location(), "UTF-8 encoding error in source character stream");
							if (r == static_cast<size_t>(-2)) continue;
							ch = c16;
							break;
						}
						break;
					}
					case EEncoding::utf16_le:
					{
						//// The stream source reads 16bit characters. (At least it does for 'mem_istream<Char>' in 'ldraw::SourceString')
						//int c = {};
						//if ((c = Read()) != EOS) ch = c; else break;
						//if (ch < 0 || ch > UnicodeMaxValue) throw ScriptException(EResult::WrongEncoding, Location(), Fmt("Unsupported UTF-16 encoding. Value %d is out of range", ch));
						
						// If the script is doing the decoding, then the source should be outputting bytes. If the source outputs characters, then the encoding should be 'already_decoded'
						int lo, hi;
						if ((lo = Read()) == EOS) break;
						if ((hi = Read()) == EOS) break;
						ch = (static_cast<uint8_t>(hi) << 8) | static_cast<uint8_t>(lo);
						if (ch < 0 || ch > UnicodeMaxValue) throw ScriptException(EResult::WrongEncoding, Location(), Fmt("Unsupported UTF-16 encoding. Value %d is out of range", ch));
						break;
					}
					case EEncoding::utf16_be:
					{
						// If the script is doing the decoding, then the source should be outputting bytes. If the source outputs characters, then the encoding should be 'already_decoded'
						int lo, hi;
						if ((hi = Read()) == EOS) break;
						if ((lo = Read()) == EOS) break;
						ch = (static_cast<uint8_t>(hi) << 8) | static_cast<uint8_t>(lo);
						if (ch < 0) throw ScriptException(EResult::WrongEncoding, Location(), Fmt("Unsupported UTF-16 encoding. Value %d is out of range", ch));
						break;
					}
					default:
					{
						throw std::runtime_error(FmtS("Unsupported character encoding: %d", m_enc));
					}
				}

				// Buffer the read character
				if (ch != '\0')
				{
					assert(ch != EOS);
					m_buffer.append(1, s_cast<char_t>(ch));
					continue;
				}
				
				// Stop reading if the buffer isn't growing
				if (m_buffer.size() <= count)
					break;
			}
			return std::min(n, s_cast<int64_t>(m_buffer.size()));
		}

		// String compare. Note: asymmetric, i.e. src="abcd", str="ab", src.Match(str) == true
		bool Match(string_view_t str)
		{
			return Match(str, 0);
		}
		bool Match(string_view_t str, int start)
		{
			return Match(str, start, int(str.size()));
		}
		bool Match(string_view_t str, int start, int count)
		{
			return Match(str, start, count, std::equal_to<wchar_t>{});
		}
		bool MatchI(string_view_t str)
		{
			return MatchI(str, 0);
		}
		bool MatchI(string_view_t str, int start)
		{
			return MatchI(str, start, int(str.size()));
		}
		bool MatchI(string_view_t str, int start, int count)
		{
			return Match(str, start, count, [](wchar_t a, wchar_t b) { return std::tolower(a) == std::tolower(b); });
		}
		template <typename Eql> bool Match(string_view_t str, int start, int count, Eql eql)
		{
			// 'start' is where to start looking in the buffer
			// 'count' is the number of characters to compare.
			if (count > s_cast<int>(str.size()))
				throw std::runtime_error(Fmt("Src.Match comparing % characters but match string length is only %d", count, s_cast<int>(str.size())));

			auto len = ReadAhead(start + count);
			if (len < start + count)
				return false;

			int i = start;
			int const iend = start + count;
			for (; i != iend && eql(str[size_t(i) - start], m_buffer[i]); ++i) { }
			return i == iend;
		}

		// String compare and consume if matched
		bool Match(string_view_t str, bool consume)
		{
			if (!Match(str)) return false;
			if (consume) Next(int(str.size()));
			return true;
		}
		bool MatchI(string_view_t str, bool consume)
		{
			if (!MatchI(str)) return false;
			if (consume) Next(int(str.size()));
			return true;
		}
		
		// Buffer and hash characters on the range [start, start + count)
		int Hash(int start, int count)
		{
			auto len = ReadAhead(start + count);
			if (len < start + count)
				throw ScriptException(EResult::UnexpectedEndOfFile, Location(), Fmt("Could not buffer %d characters. End of stream reached", start + count));

			auto str = m_buffer.c_str();
			return hash::Hash(str + start, str + start + count);
		}

		// Read 'count' characters from the string and return them as a string
		string_t ReadN(int count)
		{
			string_t sb; sb.reserve(count);
			int i = 0;

			// Copy from the buffer in one block
			auto buffered = std::min(count, s_cast<int>(m_buffer.size()));
			sb.assign(m_buffer, 0, buffered);
			Next(buffered);
			i += buffered;

			// Fill the remaining from the underlying source
			for (char_t ch; i != count && (ch = **this) != '\0'; ++i, Next()) sb.append(1, ch);
			if (i != count) throw ScriptException(EResult::UnexpectedEndOfFile, Location(), Fmt("Could not read %d characters. End of stream reached", count));
			return sb;
		}

		// Reads all characters from the src and returns them as one string.
		string_t ReadToEnd()
		{
			string_t sb; sb.reserve(4096);

			// Copy from the buffer in one block
			auto buffered = s_cast<int>(m_buffer.size());
			sb.assign(m_buffer, 0, buffered);
			Next(buffered);

			// Fill the remaining from the underlying source
			for (char_t ch; (ch = **this) != '\0'; Next()) sb.append(1, ch);
			return sb;
		}

		// Reads characters up to and including a new-line. A new-line is a carriage return ('\r'),
		// a line feed ('\n'), or a carriage return immediately followed by a line feed ('\r\n').
		// If 'include_newline' is true, the returned string will include the newline character(s).
		string_t ReadLine(bool include_newline)
		{
			string_t sb; sb.reserve(256);
			for (char_t ch; (ch = **this) != '\0'; Next())
			{
				if (ch != '\r' && ch != '\n')
				{
					sb.append(1, ch);
					continue;
				}

				if (include_newline) sb.append(1, ch);
				Next();

				if (ch == '\r' && (ch = **this) == '\n')
				{
					if (include_newline) sb.append(1, ch);
					Next();
				}

				break;
			}
			return sb;
		}
	};

	// An empty source
	struct NullSrc :Src
	{
	protected:

		// Return the next byte or decoded character from the underlying stream, or EOS for the end of the stream.
		int Read() noexcept override
		{
			return '\0';
		}

	public:

		NullSrc() noexcept
			:Src(EEncoding::already_decoded, m_loc)
		{}
	};

	// A string source
	struct StringSrc :Src
	{
		// Note:
		//  - StringSrc only returns bytes so should *NOT* use the 'already_decoded' encoding.
		//  - A useful technique is to default construct a StringSrc then push text into its buffer.
		//    When the buffer is empty, 'Read()' will return EOS. Technically, the same would work
		//    with 'NullSrc', but that's likely to be confusing.
		//  - StringSrc has a copy constructor, but it can't have 'start'/'count' parameters because
		//    UTF string encodings have variable character widths.

		enum class EFlags
		{
			None = 0,
			BufferLocally = 1 << 0,
			_flags_enum = 0,
		};

	protected:

		char const* m_beg;
		char const* m_end;
		char const* m_ptr;

		// Return the next byte or decoded character from the underlying stream.
		int Read() noexcept override
		{
			// We're returning bytes. Already decoded would mean we're returning utf-16 char_t's
			assert(m_enc != EEncoding::already_decoded);
			return m_ptr != m_end ? static_cast<uint8_t>(*m_ptr++) : EOS;
		}

		// Read all data into the local buffer
		template <typename Char>
		void BufferLocally(std::basic_string_view<Char> str)
		{
			// Special case for already decoded strings
			if constexpr (std::is_same_v<std::basic_string_view<Char>, string_view_t>)
			{
				m_ptr = nullptr;
				m_end = nullptr;
				m_buffer.assign(str);
			}
			else
			{
				// Set up the pointers to 'str' then fill the local buffer
				m_ptr = char_ptr(str.data());
				m_end = char_ptr(str.data() + str.size());
				ReadAhead(s_cast<int>(str.size()));
				m_ptr = nullptr;
				m_end = nullptr;
			}
		}

	public:

		StringSrc()
			:StringSrc(EEncoding::utf16_le)
		{}
		explicit StringSrc(EEncoding enc)
			:Src(enc, {})
			,m_beg(nullptr)
			,m_end(nullptr)
			,m_ptr(nullptr)
		{}
		explicit StringSrc(std::string_view str)
			:StringSrc(str, EFlags::None, EEncoding::utf8)
		{}
		explicit StringSrc(std::wstring_view str)
			:StringSrc(str, EFlags::None, EEncoding::utf16_le)
		{}
		explicit StringSrc(std::string_view str, EEncoding enc)
			:StringSrc(str, EFlags::None, enc)
		{}
		explicit StringSrc(std::wstring_view str, EEncoding enc)
			:StringSrc(str, EFlags::None, enc)
		{}
		explicit StringSrc(std::string_view str, EFlags flags)
			:StringSrc(str, flags, EEncoding::utf8)
		{}
		explicit StringSrc(std::wstring_view str, EFlags flags)
			:StringSrc(str, flags, EEncoding::utf16_le)
		{}
		explicit StringSrc(std::string_view str, EFlags flags, EEncoding enc)
			:Src(enc, Loc{s_cast<std::streamsize>(str.size())})
			,m_beg(str.data())
			,m_end(str.data() + str.size())
			,m_ptr(m_beg)
		{
			if (AllSet(flags, EFlags::BufferLocally))
				BufferLocally(str);
		}
		explicit StringSrc(std::wstring_view str, EFlags flags, EEncoding enc)
			:Src(enc, Loc{s_cast<std::streamsize>(str.size() * sizeof(wchar_t))})
			,m_beg(char_ptr(str.data()))
			,m_end(char_ptr(str.data() + str.size()))
			,m_ptr(m_beg)
		{
			// Make a local copy of 'str' in the buffer
			if (AllSet(flags, EFlags::BufferLocally))
				BufferLocally(str);
		}
		explicit StringSrc(StringSrc const& rhs, Loc const* loc = nullptr)
			:Src(rhs.m_enc, loc ? *loc : rhs.Location())
			,m_beg(rhs.m_beg)
			,m_end(rhs.m_end)
			,m_ptr(rhs.m_ptr)
		{}

		// The remaining length in bytes
		int size_in_bytes() const
		{
			return static_cast<int>(m_end - m_ptr);
		}

		// Return a utf-8 view of the string
		std::string_view str8() const
		{
			return str8(0, size_in_bytes());
		}
		std::string_view str8(int start, int count) const
		{
			auto size = size_in_bytes();
			if (start >= size || count < 0 || start + count > size)
				throw std::runtime_error("Access out-of-bounds");

			return std::string_view(m_ptr + start, count);
		}

		// Return a utf-16 view of the string
		std::wstring_view str16() const
		{
			return str16(0, s_cast<int>(size_in_bytes() / sizeof(wchar_t)));
		}
		std::wstring_view str16(int start, int count) const
		{
			auto size = s_cast<int>(size_in_bytes() / sizeof(wchar_t));
			if (start >= size || count < 0 || start + count > size)
				throw std::runtime_error("Access out-of-bounds");

			return std::wstring_view(reinterpret_cast<wchar_t const*>(m_ptr) + start, count);
		}
	};

	// A file source
	struct FileSrc :Src
	{
	protected:

		// The open file stream
		std::ifstream m_file;

		// The size of the potion of the file to read.
		uintmax_t m_filesize;

		// Return the next byte or decoded character from the underlying stream, or EOS for the end of the stream.
		int Read() override
		{
			auto ch = m_file.get();
			return m_file.good() ? ch : EOS;
		}

	public:

		FileSrc(std::filesystem::path const& filepath, EEncoding enc = EEncoding::auto_detect)
			:FileSrc(filepath, 0, enc)
		{}
		FileSrc(std::filesystem::path const& filepath, std::streamsize ofs, EEncoding enc = EEncoding::auto_detect)
			:FileSrc(filepath, ofs, -1, enc)
		{}
		FileSrc(std::filesystem::path const& filepath, std::streamsize ofs, int64_t limit, EEncoding enc = EEncoding::auto_detect)
			:Src(enc, Loc{})
			,m_file()
			,m_filesize()
		{
			if (!filepath.empty())
				Open(filepath, ofs, limit, enc);
		}

		// Open a file as a stream source
		void Open(std::filesystem::path const& filepath, std::streamsize ofs = 0, int64_t limit = -1, EEncoding enc = EEncoding::auto_detect)
		{
			Close();

			// Check the file exists and the offset is valid
			if (!std::filesystem::exists(filepath))
				throw ScriptException(EResult::FileNotFound, Loc(filepath), std::wstring(L"File '").append(filepath).append(L"' does not exist"));
			if (std::filesystem::is_directory(filepath))
				throw ScriptException(EResult::FileNotFound, Loc(filepath), std::wstring(L"File '").append(filepath).append(L"' is a directory"));

			// Clamp to the file size
			m_filesize = std::filesystem::file_size(filepath);
			ofs = std::min(ofs, s_cast<std::streamsize>(m_filesize));

			// Determine file encoding, look for the BOM in the first 3 bytes
			m_enc = enc;
			auto bom_size = 0;
			if (m_enc == EEncoding::auto_detect)
				m_enc = filesys::DetectFileEncoding(filepath, bom_size);

			// Open the input file stream
			m_file.open(filepath, std::ios::binary);
			if (!m_file.good())
				throw ScriptException(EResult::FileNotFound, Loc(filepath), std::wstring(L"Failed to open file ").append(filepath));

			// Seek to the offset position
			m_file.seekg(bom_size + ofs);

			// Update the location
			m_loc = Loc{ filepath, s_cast<std::streamsize>(m_filesize), bom_size + ofs, bom_size + ofs, 1, 1, ofs == 0 };

			// If a limit is given, apply it
			if (limit >= 0)
				Limit(limit);
		}

		// Close the file stream
		void Close()
		{
			m_file.close();
			m_enc = EEncoding::auto_detect;
			m_loc = Loc();
		}

		// Get/Set the read position in the file
		int64_t Position() const
		{
			std::streamoff ofs = const_cast<std::ifstream&>(m_file).tellg();
			return s_cast<int64_t>(ofs);
		}
		int64_t Position(int64_t pos)
		{
			m_file.seekg(s_cast<std::streamoff>(pos), std::ios::beg);
			return Position();
		}

		// Buffer the file contents and return a string source from the buffer
		StringSrc ToStringSrc(int64_t start = 0, int64_t count = AllData)
		{
			auto range = Buffer(start, count, true);
			return StringSrc(range, StringSrc::EFlags::BufferLocally, m_enc);
		}
	};

	// An 'istream' source
	template <typename Char>
	struct StreamSrc :Src
	{
	protected:

		// The open stream
		std::basic_istream<Char>& m_stream;

		// Return the next byte or decoded character from the underlying stream, or EOS for the end of the stream.
		int Read() override
		{
			auto ch = m_stream.get();
			return m_stream.good() ? ch : m_enc == EEncoding::already_decoded ? 0 : EOS;
		}

	public:

		StreamSrc(std::basic_istream<Char>& stream, EEncoding enc = EEncoding::utf8, Loc const& loc = {})
			:StreamSrc(stream, 0, enc, loc)
		{}
		StreamSrc(std::basic_istream<Char>& stream, std::streamsize ofs, EEncoding enc = EEncoding::utf8, Loc const& loc = {})
			:StreamSrc(stream, ofs, -1, enc, loc)
		{}
		StreamSrc(std::basic_istream<Char>& istream, std::streamsize ofs, int64_t limit, EEncoding enc = EEncoding::utf8, Loc const& loc = {})
			:Src(enc, loc)
			,m_stream(istream)
		{
			if (ofs != 0)
				m_stream.seekg(ofs);
			if (limit >= 0)
				Limit(limit);
		}

		// Get/Set the read position in the file
		int64_t Position() const
		{
			auto ofs = m_stream.tellg();
			return static_cast<int64_t>(ofs);
		}
		int64_t Position(int64_t pos)
		{
			m_stream.seekg(s_cast<std::streamoff>(pos), std::ios::beg);
			return Position();
		}
	};

	// Call '++src' until 'pred' returns false.
	// 'eat_initial' and 'eat_final' are the number of characters to consume before
	// applying the predicate 'pred' and the number to consume after 'pred' returns false.
	template <typename TSrc, typename Char> inline void EatDelimiters(TSrc& src, Char const* delim)
	{
		for (; *str::FindChar(delim, *src) != 0; ++src) {}
	}
	template <typename TSrc, typename Pred> void Eat(TSrc& src, int eat_initial, int eat_final, Pred pred)
	{
		for (src += eat_initial; *src && pred(src); ++src) {}
		src += eat_final;
	}
	template <typename TSrc> inline void EatLineSpace(TSrc& src, int eat_initial, int eat_final)
	{
		Eat(src, eat_initial, eat_final, [](TSrc& s) { return str::IsLineSpace(*s); });
	}
	template <typename TSrc> inline void EatWhiteSpace(TSrc& src, int eat_initial, int eat_final)
	{
		Eat(src, eat_initial, eat_final, [](TSrc& s) { return str::IsWhiteSpace(*s); });
	}
	template <typename TSrc> inline void EatLine(TSrc& src, int eat_initial, int eat_final, bool eat_newline)
	{
		src += eat_initial;
		Eat(src, 0, 0, [](TSrc& s){ return !(s[0] == '\n') && !(s[0] == '\r' && s[1] == '\n'); });
		if (eat_newline) src += (src[0] == '\r' && src[1] == '\n') ? 2 : (src[0] == '\n') ? 1 : 0;
		src += eat_final;
	}
	template <typename TSrc> inline void EatBlock(TSrc& src, string_view_t block_beg, string_view_t block_end)
	{
		if (block_beg.empty())
			throw std::runtime_error("The block start marker cannot have length = 0");
		if (block_end.empty())
			throw std::runtime_error("The block end marker cannot have length = 0");
		if (!src.Match(block_beg))
			throw std::runtime_error("Don't call 'EatBlock' unless 'src' is pointing at the block start");

		Eat(src, static_cast<int>(block_beg.size()), static_cast<int>(block_end.size()), [=](TSrc& s) { return !s.Match(block_end); });
	}
	template <typename TSrc> inline void EatLiteral(TSrc& src, Loc const& loc = Loc())
	{
		// Don't call this unless 'src' is pointing at a literal string
		auto quote = *src;
		if (quote != '\"' && quote != '\'')
			throw ScriptException(EResult::InvalidString, loc, Fmt(L"Expected the start of a string literal, but the next character is: %c", quote));

		for (bool esc = true; *src != '\0' && (esc || *src != quote); esc = !esc && *src == '\\', ++src) {}
		if (*src != quote)
			throw ScriptException(EResult::InvalidString, loc, "Incomplete literal string or character");

		++src;
	}
	template <typename TSrc> inline void EatSection(TSrc& src, Loc const& loc = Loc())
	{
		// Don't call this unless 'src' is pointing at a '{'
		if (*src != '{')
			throw ScriptException(EResult::TokenNotFound, loc, Fmt("Expected to the start of a section block, but the next character is %c", *src));

		for (int nest = 0; *src;)
		{
			if (*src == L'\"') { EatLiteral(src, loc); continue; }
			nest += (*src == L'{') ? 1 : 0;
			nest -= (*src == L'}') ? 1 : 0;
			if (nest == 0) break;
			++src;
		}

		if (*src != '}')
			throw ScriptException(EResult::TokenNotFound, loc, "Incomplete section block");

		++src;
	}
	template <typename TSrc> inline void EatLineComment(TSrc& src, string_view_t line_comment = L"//")
	{
		assert(*src == line_comment[0]); // don't call this unless 'src' is pointing at a line comment
		EatLine(src, static_cast<int>(line_comment.size()), 0, false);
	}
	template <typename TSrc> inline void EatBlockComment(TSrc& src, string_view_t block_beg = L"/*", string_view_t block_end = L"*/")
	{
		assert(*src == block_beg[0]); // don't call this unless 'src' is pointing at a block comment
		EatBlock(src, block_beg, block_end);
	}

	// Buffer an identifier in 'src'. Returns true if a valid identifier was buffered.
	// On return, 'len' contains the length of the buffer up to and including the end
	// of the identifier (i.e. start + strlen(identifier)).
	inline bool BufferIdentifier(Src& src, int start = 0, int* len = nullptr)
	{
		auto i = start;
		auto x = Scope<void>([]{}, [&] { if (len) *len = i; });
		if (!str::IsIdentifier(src[i], true)) return false;
		for (++i; str::IsIdentifier(src[i], false); ++i) {}
		return true;
	}

	// Buffer a literal string or character in 'src'. Returns true if a complete literal string or
	// character was buffered. On return, 'len' contains the length of the buffer up to, and
	// including, the literal. (i.e. start + strlen(literal))
	inline bool BufferLiteral(Src& src, int start = 0, int* len = nullptr)
	{
		auto i = start;
		auto x = Scope<void>([]{}, [&]{ if (len) *len = i; });

		// Don't call this unless 'src' is pointing at a literal string
		auto quote = src[i];
		if (quote != '\"' && quote != '\'')
			return false;

		// Find the end of the literal
		for (bool esc = true; src[i] != '\0' && (esc || src[i] != quote); esc = !esc && src[i] == '\\', ++i) {}
		if (src[i] == quote) ++i; else return false;
		return true;
	}

	// Buffer characters for a number (real or int) in 'src'.
	// Format: [delim][{+|-}][0[{x|X|b|B}]][digits][.digits][{d|D|e|E|p|P}[{+|-}]digits][U][L][L]
	// Returns true if valid number characters where buffered
	inline bool BufferNumber(Src& src, int& radix, int start = 0, int* len = nullptr, str::ENumType type = str::ENumType::Any)
	{
		// Notes:
		//  - This duplicates the BufferNumber function in pr::str :-/
		//    The pr::str version consumes characters from a stream where as this version
		//    simply buffers the characters in 'src'. I don't want pr::str to depend on pr::script
		//    and I don't want to change the behaviour of the pr::str version, so duplication is
		//    the only option.

		auto i = start;
		auto x = Scope<void>([]{}, [&] { if (len) *len = i; });

		// Convert a character to it's numerical value
		constexpr auto digit = [](int ch)
		{
			if (ch >= '0' && ch <= '9') return ch - '0';
			if (ch >= 'a' && ch <= 'z') return 10 + ch - 'a';
			if (ch >= 'A' && ch <= 'Z') return 10 + ch - 'A';
			return std::numeric_limits<int>::max();
		};

		auto digits_found = false;
		auto allow_fp = AllSet(type, str::ENumType::FP);
		auto fp = false;

		// Look for the optional sign character
		// Ideally we'd prefer not to advance 'src' passed the '+' or '-' if the next
		// character is not the start of a number. However doing so means 'src' can't
		// be a forward only input stream. Therefore, I'm pushing the responsibility
		// back to the caller, they need to check that if *src is a '+' or '-' then
		// the following char is a decimal digit.
		if (src[i] == '+' || src[i] == '-')
			++i;

		// Look for a radix prefix on the number, this overrides 'radix'.
		// If the first digit is zero, then the number may have a radix prefix.
		// '0x' or '0b' must have at least one digit following the prefix
		// Adding 'o' for octal, in addition to standard C literal syntax
		if (src[i] == '0')
		{
			++i;
			auto radix_prefix = false;
			if (false) { }
			else if (tolower(src[i]) == 'x') { radix = 16; ++i; radix_prefix = true; }
			else if (tolower(src[i]) == 'o') { radix = 8; ++i; radix_prefix = true; }
			else if (tolower(src[i]) == 'b') { radix = 2; ++i; radix_prefix = true; }
			else
			{
				// If no radix prefix is given, then assume octal zero (for conformance with C) 
				if (radix == 0) radix = str::IsDigit(src[i]) ? 8 : 10;
				digits_found = true;
			}

			// Check for the required integer
			if (radix_prefix && digit(src[i]) >= radix)
				return false;
		}
		else if (radix == 0)
		{
			radix = 10;
		}

		// Read digits up to a delimiter, decimal point, or digit >= radix.
		auto assumed_fp_len = 0; // the length of the number when we first assumed a FP number.
		for (; src[i] != 0; ++i)
		{
			// If the character is greater than the radix, then assume a FP number.
			// e.g. 09.1 could be an invalid octal number or a FP number. 019 is assumed to be FP.
			auto d = digit(src[i]);
			if (d < radix)
			{
				digits_found = true;
				continue;
			}

			if (radix == 8 && allow_fp && d < 10)
			{
				if (assumed_fp_len == 0) assumed_fp_len = i;
				continue;
			}

			break;
		}

		// If we're assuming this is a FP number but no decimal point is found,
		// then truncate the string at the last valid character given 'radix'.
		// If a decimal point is found, change the radix to base 10.
		if (assumed_fp_len != 0)
		{
			if (src[i] == '.') radix = 10;
			else i = assumed_fp_len;
		}

		// FP numbers can be in dec or hex, but not anything else...
		allow_fp &= radix == 10 || radix == 16;
		if (allow_fp)
		{
			// If floating point is allowed, read a decimal point followed by more digits, and an optional exponent
			if (src[i] == '.' && str::IsDecDigit(src[++i]))
			{
				fp = true;
				digits_found = true;

				// Read decimal digits up to a delimiter, sign, or exponent
				for (; str::IsDecDigit(src[i]); ++i) { }
			}

			// Read an optional exponent
			auto ch = tolower(src[i]);
			if (ch == 'e' || ch == 'd' || (ch == 'p' && radix == 16))
			{
				++i;

				// Read the optional exponent sign
				if (src[i] == '+' || src[i] == '-')
					++i;

				// Read decimal digits up to a delimiter, or suffix
				for (; str::IsDecDigit(src[i]); ++i) { }
			}
		}

		// Read the optional number suffixes
		if (allow_fp && tolower(src[i]) == 'f')
		{
			fp = true;
			++i;
		}
		if (!fp && tolower(src[i]) == 'u')
		{
			++i;
		}
		if (!fp && tolower(src[i]) == 'l')
		{
			++i;
			if (tolower(src[i]) == 'l')
				++i;
		}
		return digits_found;
	}

	// Buffer up to the next '\n' in 'src'. Returns true if a new line or at least one character is buffered.
	// if 'include_newline' is false, the new line is removed from the buffer once read.
	// On return, 'len' contains the length of the buffer up to, and including the new line character (even if the newline character is removed).
	inline bool BufferLine(Src& src, bool include_newline, int start = 0, int* len = nullptr)
	{
		auto i = start;
		auto x = Scope<void>([]{}, [&]{ if (len) *len = i; });

		if (src[i] == '\0') return false;
		for (; src[i] != '\0' && src[i] != '\n'; ++i) {}
		if (src[i] != '\0') { if (include_newline) i += 1; else src.Buffer().erase(i, 1); }
		return true;
	}

	// Buffer up to and including 'end'. If 'include_end' is false, 'end' is removed from the buffer once read.
	// On return, 'len' contains the length of the buffer up to, and including 'end' (even if end is removed).
	inline bool BufferTo(Src& src, string_view_t end, bool include_end, int start = 0, int* len = nullptr)
	{
		auto i = start;
		auto x = Scope<void>([]{}, [&]{ if (len) *len = i; });

		for (; src[i] != '\0' && !src.Match(end, i); ++i) {}
		if (src[i] != '\0') { if (include_end) i += s_cast<int>(end.size()); else src.Buffer().erase(i, end.size()); }
		return src[i] != '\0';
	}

	// Buffer until 'adv' returns 0. Signature for AdvFunc: int(Src&,int)
	// Returns true if buffering stopped due to 'adv' returning 0.
	// On return, 'len' contains the length of the buffer up to where 'adv' returned 0.
	template <typename AdvFunc, typename = std::enable_if_t<std::is_invocable_r_v<int, AdvFunc, Src&, int>>>
	inline bool BufferWhile(Src& src, AdvFunc adv, int start = 0, int* len = nullptr)
	{
		auto i = start;
		auto x = Scope<void>([]{}, [&]{ if (len) *len = i; });
		
		for (auto inc = 0; src[i] != '\0' && (inc = adv(src, i)) != 0; i += inc) {}
		if (src[i] == '\0') i = static_cast<int>(std::min<int64_t>(src.Limit(), static_cast<int64_t>(src.Buffer().size()))); // Occurs if 'start' > 'src.Limit' or EOS
		return src[i] != '\0';
	}

	// Generate the hash of a keyword
	inline int HashKeyword(wchar_t const* keyword, bool case_sensitive)
	{
		// This function can't have the same name as 'HashKeyword' because it causes a compiler error
		auto const kw = case_sensitive ? hash::Hash(keyword) : hash::HashI(keyword);
		return kw;
	}
}