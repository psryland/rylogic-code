//**********************************************************************************
// Resource
//  Copyright (c) Rylogic Ltd 2009
//**********************************************************************************
#pragma once
#include <streambuf>
#include <iostream>
#include <type_traits>
#include <cassert>

namespace pr
{
	// Notes:
	//  - Input streams like 'istringstream' make a copy of the data they're reading from.
	//    'mem_istream' uses 'view_streambuf' to read directly from the data avoiding the copy.
	//  - Output streams contain the buffer they write to and have to be copied out.
	//    'mem_ostream' avoids this by allowing the caller to provide the buffer.
	//  - See: http://www.cplusplus.com/reference/streambuf/basic_streambuf/

	// A stream buffer that wraps a constant buffer of bytes
	template <typename Elem>
	class view_streambuf :public std::basic_streambuf<Elem>
	{
		using traits_type = typename view_streambuf::traits_type;
		using char_type   = typename view_streambuf::char_type;
		using int_type    = typename view_streambuf::int_type;
		using pos_type    = typename view_streambuf::pos_type;
		using off_type    = typename view_streambuf::off_type;

		char_type const* m_beg;
		char_type const* m_end;
		char_type const* m_ptr;

	public:

		view_streambuf(void const* data, size_t size)
			:std::basic_streambuf<Elem>()
			,m_beg(static_cast<char_type const*>(data))
			,m_end(m_beg + size)
			,m_ptr(m_beg)
		{}

		// Keep the streambuf hooks protected so derived tests can exercise the virtuals without exposing them publicly.
	protected:

		// Get the current character in the controlled input sequence without changing the current position (or return eof()).
		int_type underflow() override
		{
			return m_ptr != m_end
				? traits_type::to_int_type(*m_ptr)
				: traits_type::eof();
		}

		// Get the current character in the controlled input sequence and then advance the position indicator to the next character.
		int_type uflow() override
		{
			return m_ptr != m_end
				? traits_type::to_int_type(*m_ptr++)
				: traits_type::eof();
		}

		// Puts a character back into the input sequence, possibly modifying the input sequence
		int_type pbackfail(int_type ch)
		{
			return m_ptr != m_beg && (ch == traits_type::eof() || ch == m_ptr[-1])
				? traits_type::to_int_type(*--m_ptr)
				: traits_type::eof();
		}

		// Read "s-how-many-c". Get an estimate on the number of characters available in the associated input sequence.
		std::streamsize showmanyc()
		{
			assert(std::less_equal<char_type const*>()(m_ptr, m_end));
			return m_end - m_ptr;
		}

		// Sets the position indicator of the input and/or output sequence to an absolute position.
		pos_type seekpos(pos_type pos, std::ios_base::openmode which = std::ios_base::in | std::ios_base::out) override
		{
			// change to specified position, according to mode
			if (which & std::ios_base::in)
			{
				m_ptr = m_beg + clamp(ptrdiff_t(pos), 0, m_end - m_beg);
				return pos_type(m_ptr - m_beg);
			}
			return std::basic_streambuf<Elem>::seekpos(pos, which);
		}

		// Sets the position indicator of the input and/or output sequence relative to some other position.
		pos_type seekoff(off_type ofs, std::ios_base::seekdir dir, std::ios_base::openmode which = std::ios_base::in | std::ios_base::out) override
		{
			// change position by offset, according to way and mode
			if (which & std::ios_base::in)
			{
				switch (dir)
				{
				case std::ios_base::beg: m_ptr = m_beg + clamp(ptrdiff_t(ofs), m_beg - m_beg, m_end - m_beg); break;
				case std::ios_base::cur: m_ptr = m_ptr + clamp(ptrdiff_t(ofs), m_beg - m_ptr, m_end - m_ptr); break;
				case std::ios_base::end: m_ptr = m_end + clamp(ptrdiff_t(ofs), m_beg - m_end, m_end - m_end); break;
				}
				return pos_type(m_ptr - m_beg);
			}
			return std::basic_streambuf<Elem>::seekoff(ofs, dir, which);
		}
	
		// Clamp to a range
		static ptrdiff_t clamp(ptrdiff_t x, ptrdiff_t mn, ptrdiff_t mx)
		{
			return x < mn ? mn : x > mx ? mx : x;
		}
	};

	// A stream buffer that wraps callbacks to handle reading/writing data
	template <typename Elem>
	class callback_streambuf :public std::basic_streambuf<Elem>
	{
		// Notes:
		//  - the callback functions shouldn't throw, but return 0 bytes read/written.
		//  - the end of the buffer is determined implicitly from the success of reading/writting.
		//  - read/write can be null if only writing/reading respectively.

	public:

		using traits_type = typename callback_streambuf::traits_type;
		using char_type   = typename callback_streambuf::char_type;
		using int_type    = typename callback_streambuf::int_type;
		using pos_type    = typename callback_streambuf::pos_type;
		using off_type    = typename callback_streambuf::off_type;
		static_assert(sizeof(char_type) == sizeof(uint8_t));

		// Callback signature. Returns the number of characters successfully read/written.
		// Note: 'ofs' is in units of 'char_type', *not* 'Elem'.
		using write_t = std::streamsize (*)(void*, off_type ofs, char_type const* bytes, std::streamsize count);
		using read_t  = std::streamsize (*)(void*, off_type ofs, char_type* bytes, std::streamsize count);

	private:

		off_type m_gpos; // The read position in the buffer
		off_type m_ppos; // The write position in the buffer
		off_type m_end;  // The maximum size of the buffer
		write_t m_write; // The write callback
		read_t m_read;   // The read callback
		void* m_ctx;     // User context pointer

	public:

		callback_streambuf(read_t read, write_t write, void* ctx, pos_type end = 0)
			:m_gpos()
			,m_ppos()
			,m_end(end)
			,m_write(write)
			,m_read(read)
			,m_ctx(ctx)
		{}

	protected:

		// Get the current character in the controlled input sequence without changing the current position (or return eof()).
		int_type underflow() override
		{
			// Note: 'm_gpos' not incremented
			char_type ch;
			return m_read(m_ctx, m_gpos, &ch, 1) == 1
				? traits_type::to_int_type(ch)
				: traits_type::eof();
		}

		// Overflow is called to grow the output (or return eof())
		int_type overflow(int_type c) override
		{
			// EOF is a flush-style no-op for this streambuf; do not turn it into a data byte.
			if (c == traits_type::eof())
				return traits_type::not_eof(c);

			auto ch = traits_type::to_char_type(c);
			if (m_write == nullptr || m_write(m_ctx, m_ppos, &ch, 1) != 1)
				return traits_type::eof();

			// Keep the write cursor and the logical end in step with single-character writes.
			m_ppos += 1;
			m_end = std::max(m_end, m_ppos);
			return traits_type::to_int_type(ch);
		}

		// Get the current character in the controlled input sequence and then advance the position indicator to the next character.
		int_type uflow() override
		{
			int_type ch = underflow();
			if (ch != traits_type::eof())
			{
				m_gpos += 1;
				m_end = std::max(m_end, m_gpos);
			}
			return ch;
		}

		// Puts a character back into the input sequence, possibly modifying the input sequence
		int_type pbackfail(int_type c) override
		{
			// If at the start of the input buffer, then fail
			if (m_gpos == 0)
				return traits_type::eof();

			// If 'c' is 'eof()' then we don't care what the previous char was
			if (c == traits_type::eof())
				return m_gpos -= 1, c;

			// If 'c' matches the previous char, then put back succeeds
			if (char_type ch; m_read(m_ctx, pos_type(m_gpos - 1), &ch, 1) == 1 && traits_type::to_int_type(ch) == c)
				return m_gpos -= 1, traits_type::to_int_type(ch);
			
			// If overwriting the input sequence with 'c' succeeds, then put back succeeds
			if (char_type ch = traits_type::to_char_type(c); m_write != nullptr && m_write(m_ctx, pos_type(m_gpos - 1), &ch, 1) == 1)
				return m_gpos -= 1, traits_type::to_int_type(ch); 
			
			return traits_type::eof();
		}

		// Retrieves characters from the controlled input sequence and stores them in the array pointed by s,
		// until either n characters have been extracted or the end of the sequence is reached.
		std::streamsize xsgetn(char_type* s, std::streamsize n) override
		{
			auto count = m_read(m_ctx, m_gpos, s, n);
			m_gpos += count;
			m_end = std::max(m_gpos, m_end);
			return count;
		}

		// Writes characters from the array pointed to by s into the controlled output sequence,
		// until either n characters have been written or the end of the output sequence is reached.
		std::streamsize xsputn(char_type const* s, std::streamsize n) override 
		{ 
			auto count = m_write(m_ctx, m_ppos, s, n);
			m_ppos += count;
			m_end = std::max(m_ppos, m_end);
			return count;
		};

		// Read "s-how-many-c". Get an estimate on the number of characters available in the associated input sequence.
		std::streamsize showmanyc() override
		{
			return m_end - m_gpos;
		}

		// Sets the position indicator of the input and/or output sequence to an absolute position.
		pos_type seekpos(pos_type pos, std::ios_base::openmode which = std::ios_base::in | std::ios_base::out) override
		{
			if (which & std::ios_base::in)
			{
				// Clamp to the known range of data
				m_gpos = clamp(pos, 0, m_end);
				return pos_type(m_gpos);
			}
			if (which & std::ios_base::out)
			{
				// Fill to the required size
				for (; m_end < pos && overflow(0) != traits_type::eof(); ) {}
				m_ppos = clamp(pos, 0, m_end);
				return pos_type(m_ppos);
			}
			return std::basic_streambuf<Elem>::seekpos(pos, which);
		}

		// Sets the position indicator of the input and/or output sequence relative to some other position.
		pos_type seekoff(off_type ofs, std::ios_base::seekdir dir, std::ios_base::openmode which = std::ios_base::in | std::ios_base::out) override
		{
			static auto const inout = std::ios_base::in | std::ios_base::out;
			switch (dir)
			{
			case std::ios_base::beg: return seekpos(0     + ofs, which);
			case std::ios_base::end: return seekpos(m_end + ofs, which);
			case std::ios_base::cur: return
				which == std::ios_base::in ? seekpos(m_gpos + ofs, which) :
				which == std::ios_base::out ? seekpos(m_ppos + ofs, which) :
				which == inout && m_gpos == m_ppos ? seekpos(m_gpos + ofs, which) :
				throw std::runtime_error("relative seek for both in and out position is invalid when the current positions are not equal");
			}
			return std::basic_streambuf<Elem>::seekoff(ofs, dir, which);
		}
		
		// Clamp to a range
		static off_type clamp(off_type x, off_type mn, off_type mx)
		{
			return x < mn ? mn : x > mx ? mx : x;
		}
	};

	// An 'istream' wrapper around an immutable buffer. Use 'Elem == char' for 'std::isteam'
	template<typename Elem = uint8_t>
	struct mem_istream :public std::basic_istream<Elem>
	{
		using base_t = typename std::basic_istream<Elem>;
		using char_type = typename base_t::char_type;

		view_streambuf<Elem> m_buf;

		mem_istream(void const* data, std::size_t size)
			:base_t(&m_buf)
			,m_buf(data, size)
		{}
		mem_istream(std::span<Elem const> view)
			:mem_istream(view.data(), view.size())
		{
		}
		mem_istream(std::basic_string_view<Elem> view, int = 0) // use to disambiguate from 'span'
			:mem_istream(view.data(), view.size())
		{
		}
		mem_istream& read(char_type* src, std::streamsize count)
		{
			// Don't convert to std::span, the api needs to match 'std::istream'
			auto& base = *static_cast<base_t*>(this);
			base.read(reinterpret_cast<char_type*>(src), count);
			return *this;
		}
	};

	// An 'ostream' wrapper around a user provided container.  Use 'Elem == char' for 'std::osteam'
	template <typename Elem = uint8_t, typename Container = std::vector<Elem>>
	struct mem_ostream :public std::basic_ostream<Elem>
	{
		using base_t = typename std::basic_ostream<Elem>;
		using value_type = typename Container::value_type;
		using char_type = typename base_t::char_type;

		callback_streambuf<char_type> m_buf;
		Container& m_data;

		mem_ostream(Container& data)
			: base_t(&m_buf)
			, m_buf(nullptr, write, This())
			, m_data(data)
		{}

		mem_ostream& write(char_type const* src, std::streamsize count)
		{
			// Don't convert to std::span, the api needs to match 'std::ostream'
			auto& base = *static_cast<base_t*>(this);
			base.write(reinterpret_cast<char_type const*>(src), count);
			return *this;
		}

	private:

		// Warning fix
		constexpr mem_ostream* This() { return this; }

		// Convert from size in bytes to count in 'value_type's
		constexpr static size_t size_to_count(size_t size_in_bytes)
		{
			return (size_in_bytes + sizeof(value_type) - 1) / sizeof(value_type);
		}

		// Callback function that adds data to 'data'
		static std::streamsize write(void* ctx, std::streambuf::off_type ofs, char_type const* bytes, std::streamsize count)
		{
			auto& me = *static_cast<mem_ostream*>(ctx);

			// Make space in 'm_data'
			auto required = size_to_count(static_cast<size_t>(ofs + count));
			auto new_count = std::max(me.m_data.size(), required);
			me.m_data.resize(new_count);

			std::memcpy(reinterpret_cast<char_type*>(me.m_data.data()) + ofs, bytes, static_cast<size_t>(count));
			return count;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(MemStreamTests)
	{
		{ //' mem_istream
			int data[] = { 1, 2, 3 };
			pr::mem_istream<char> strm(data, sizeof(data));
			PR_EXPECT(!!strm);

			int out[3];
			strm.read((char*)out, sizeof(int) * 3);
			PR_EXPECT(memcmp(data, out, sizeof(int) * 3) == 0);

			strm.seekg(0);
			PR_EXPECT(!!strm);

			strm.read((char*)out, sizeof(int) * 3);
			PR_EXPECT(memcmp(data, out, sizeof(int) * 3) == 0);

			size_t pos = (size_t)strm.tellg();
			PR_EXPECT(pos == sizeof(int) * 3);

			strm.seekg(4);
			strm.read((char*)out, sizeof(int) * 1);
			PR_EXPECT(memcmp(&data[1], out, sizeof(int) * 1) == 0);

			pos = (size_t)strm.tellg();
			PR_EXPECT(pos == sizeof(int) * 2);
		}
		{ //' mem_ostream
			uint8_t src[] = { 1,1,1,1, 2,2,2,2, 3,3,3,3, 4,4,4,4 };
			int expected[] = { 0x01010101, 0x02020202, 0x03030303, 0x04040404 };

			std::vector<int> data;
			pr::mem_ostream strm(data);
		
			strm.write(&src[0], 8);
			PR_EXPECT(data.size() == 2U);
			PR_EXPECT(memcmp(data.data(), &expected[0], sizeof(int) * 2) == 0);

			strm.write(&src[8], 4);
			PR_EXPECT(data.size() == 3U);
			PR_EXPECT(memcmp(data.data(), &expected[0], sizeof(int) * 3) == 0);

			strm.seekp(0);
			strm.write(&src[4], 12);
			PR_EXPECT(data.size() == 3U);
			PR_EXPECT(memcmp(data.data(), &expected[1], sizeof(int) * 3) == 0);

			strm.seekp(8);
			strm.write(&src[8], 8);
			strm.seekp(0);
			strm.write(&src[0], 8);
			PR_EXPECT(data.size() == 4U);
			PR_EXPECT(memcmp(data.data(), &expected[0], sizeof(int) * 4) == 0);

			strm.seekp(16);
			size_t pos = (size_t)strm.tellp();
			PR_EXPECT(pos == 16U);
		}

		{ //' callback_streambuf
			struct write_log
			{
				std::vector<char> m_data;
				std::vector<size_t> m_write_offsets;

				// Record the callback write offsets so the tests can assert the logical cursor movement directly.
				static std::streamsize write(void* ctx, std::streambuf::off_type ofs, char const* bytes, std::streamsize count)
				{
					auto& me = *static_cast<write_log*>(ctx);

					// Grow first so gap filling leaves zero-initialised bytes between writes instead of collapsing them.
					auto required = static_cast<size_t>(ofs) + static_cast<size_t>(count);
					if (me.m_data.size() < required)
						me.m_data.resize(required, '\0');

					me.m_write_offsets.push_back(static_cast<size_t>(ofs));
					std::copy_n(bytes, static_cast<size_t>(count), me.m_data.data() + static_cast<size_t>(ofs));
					return count;
				}
			};

			struct test_callback_streambuf :pr::callback_streambuf<char>
			{
				using base_t = pr::callback_streambuf<char>;
				using write_t = typename base_t::write_t;
				using pos_type = typename base_t::pos_type;
				using base_t::overflow;

				// Expose the protected overflow hook so the EOF path can be checked without relying on a higher-level wrapper.
				test_callback_streambuf(write_t write, void* ctx, pos_type end = 0)
					:base_t(nullptr, write, ctx, end)
				{}
			};

			{ // Consecutive single-character writes must advance the logical output position.
				write_log log;
				test_callback_streambuf buf(write_log::write, &log);
				std::ostream strm(&buf);
				auto expected_offsets = std::vector<size_t>{0, 1};
				auto expected_data = std::vector<char>{'a', 'b'};

				strm.put('a');
				strm.put('b');

				PR_EXPECT(log.m_write_offsets == expected_offsets);
				PR_EXPECT(log.m_data == expected_data);
				PR_EXPECT((size_t)strm.tellp() == 2U);
			}

			{ // Mixed single-character and bulk writes must share the same write cursor.
				write_log log;
				test_callback_streambuf buf(write_log::write, &log);
				std::ostream strm(&buf);
				auto expected_offsets = std::vector<size_t>{0, 1, 3};
				auto expected_data = std::vector<char>{'a', 'b', 'c', 'd'};

				strm.put('a');
				strm.write("bc", 2);
				strm.put('d');

				PR_EXPECT(log.m_write_offsets == expected_offsets);
				PR_EXPECT(log.m_data == expected_data);
				PR_EXPECT((size_t)strm.tellp() == 4U);
			}

			{ // Seeking past the end must fill each missing byte at a unique offset before the next write.
				write_log log;
				test_callback_streambuf buf(write_log::write, &log);
				std::ostream strm(&buf);
				auto expected_offsets = std::vector<size_t>{0, 1, 2, 3, 4};
				auto expected_data = std::vector<char>{'\0', '\0', '\0', '\0', 'x'};

				strm.seekp(4);
				strm.put('x');

				PR_EXPECT(log.m_write_offsets == expected_offsets);
				PR_EXPECT(log.m_data == expected_data);
				PR_EXPECT((size_t)strm.tellp() == 5U);
			}

			{ // overflow(eof) is a no-op and must not synthesise a byte.
				write_log log;
				test_callback_streambuf buf(write_log::write, &log);
				auto const eof = std::char_traits<char>::eof();
				auto expected_offsets = std::vector<size_t>{0};
				auto expected_data = std::vector<char>{'q'};

				PR_EXPECT(buf.overflow(eof) != eof);
				PR_EXPECT(log.m_write_offsets.empty());

				PR_EXPECT(buf.overflow('q') != eof);
				PR_EXPECT(log.m_write_offsets == expected_offsets);
				PR_EXPECT(log.m_data == expected_data);
			}
		}
	}
}
#endif
