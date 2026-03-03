//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once
#include "pr/script/forward.h"

namespace pr::script
{
	// A location within a script source
	struct Loc
	{
	private:

		// The path to the stream source
		std::filesystem::path m_filepath;

		// The script source file size in characters, or 0 if unknown
		std::streamsize m_filesize;

		// The character offset into the stream (0-based)
		std::streamoff m_pos;

		// The character offset into the stream at the start of the last seen line (0-based)
		std::streamoff m_line_pos;

		// Line number in the character stream (natural number, i.e. 1-based)
		int m_line;

		// Column number in the character stream (natural number, i.e. 1-based) (note: *not* character index on the line because of tabs)
		int m_col;

		// The number of columns that a tab character corresponds to
		int m_tab_size;

		// True if the line and column values are valid
		bool m_lc_valid;

		// By default, 1 tab = 4 columns.
		static int const DefTabSize = 4;

	public:

		Loc()
			: Loc(std::filesystem::path())
		{}
		explicit Loc(std::filesystem::path const& filepath)
			: Loc(filepath, std::filesystem::exists(filepath) ? std::filesystem::file_size(filepath) : 0, 0, 0, 1, 1, true)
		{}
		explicit Loc(std::streamsize filesize)
			: Loc({}, filesize, 0, 0, 1, 1, true)
		{}
		Loc(std::filesystem::path const& filepath, std::streamoff pos)
			: Loc(filepath, std::filesystem::exists(filepath) ? std::filesystem::file_size(filepath) : 0, pos, 0, 1, 1, true)
		{}
		Loc(std::filesystem::path const& filepath, std::streamsize size, std::streamoff pos, std::streamoff line_pos, int line, int col, bool lc_valid, int tab_size = DefTabSize)
			: m_filepath(filepath)
			, m_filesize(size)
			, m_pos(pos)
			, m_line_pos(line_pos)
			, m_line(std::max(line, 1))
			, m_col(std::max(col, 1))
			, m_tab_size(tab_size)
			, m_lc_valid(lc_valid)
		{
			assert("Line index should be natural number, 1-based" && line >= 1);
			assert("Column index should be natural number, 1-based" && col >= 1);
		}

		// Advance the location by interpreting 'ch'
		wchar_t inc(wchar_t ch) noexcept
		{
			// '0' means EOS
			if (ch != 0)
				++m_pos;

			if (ch == L'\n')
			{
				m_line_pos = m_pos;
				++m_line;
				m_col = 1;
			}
			else if (ch == L'\t')
			{
				m_col += m_tab_size;
			}
			else if (ch != 0)
			{
				++m_col;
			}

			return ch;
		}
		char inc(char ch) noexcept
		{
			inc(static_cast<wchar_t>(ch));
			return ch;
		}

		// Get/Set the source path (usually file name)
		std::filesystem::path Filepath() const noexcept
		{
			return m_filepath;
		}
		void Filepath(std::filesystem::path const& filepath) noexcept
		{
			m_filepath = filepath;
		}

		// Get the file size in bytes, or 0 if unknown
		std::streamsize FileSize() const noexcept
		{
			return m_filesize;
		}

		// Get/set the stream position
		std::streamoff Pos() const noexcept
		{
			return m_pos;
		}

		// Get the character offset to the start of the line
		std::streamoff LinePos() const noexcept
		{
			return m_line_pos;
		}

		// Get/Set the line number (as a natural number, 1-based)
		int Line() const noexcept
		{
			return m_line;
		}
		void Line(int line) noexcept
		{
			assert("Line index should be natural number, 1-based" && line >= 1);
			m_line = line;
		}

		// Get/Set the column number (as a natural number, 1-based)
		int Col() const noexcept
		{
			return m_col;
		}
		void Col(int col) noexcept
		{
			assert("Column index should be natural number, 1-based" && col >= 1);
			m_col = col;
		}

		// True if the line/column values are valid
		bool LCValid() const noexcept
		{
			return m_lc_valid;
		}

		// Return a copy of this location at the start of the current line
		Loc LineStartLoc() const
		{
			return Loc(Filepath(), LinePos(), LinePos(), Line(), 1, LCValid(), m_tab_size);
		}

		// Output the location as a string
		std::wstring ToString() const
		{
			std::wstring s = Filepath().wstring();
			if (LCValid()) s.append(Fmt(L"(%d:%d)", Line(), Col()));
			s.append(Fmt(L" (offset:%d)", Pos()));
			return s;
		}

		// The location within a stream of characters
		friend bool operator == (Loc const& lhs, Loc const& rhs)
		{
			return lhs.Pos() == rhs.Pos() && lhs.Filepath() == rhs.Filepath();
		}
		friend bool operator != (Loc const& lhs, Loc const& rhs)
		{
			return !(lhs == rhs);
		}
		friend bool operator < (Loc const& lhs, Loc const& rhs)
		{
			if (lhs.Filepath() != rhs.Filepath()) return lhs.Filepath() < rhs.Filepath();
			return lhs.Pos() <= rhs.Pos();
		}
	};
}
