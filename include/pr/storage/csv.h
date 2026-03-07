//*****************************************************************************************
// CSV
//  Copyright (c) Rylogic 2011
//*****************************************************************************************
// Usage:
//  pr::csv::Csv csv;
//  if (!pr::csv::Load("my_csv.csv", csv)) { printf("Failed to load csv\n"); return; }
//  csv.erase(csv.begin());                                                                 // Erase column header row
//  std::sort(csv.begin(), csv.end(), pr::csv::SortColumn<>(0));                            // Sort by column zero
//  csv.erase(std::unique(csv.begin(), csv.end(), pr::csv::UniqueColumn<>(0)), csv.end());  // Make unique in column zero
#pragma once

#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cassert>
#include <filesystem>

#ifndef PR_CSV_USE_PRSTRING
#define PR_CSV_USE_PRSTRING 1
#endif

#if PR_CSV_USE_PRSTRING
#include "pr/str/string.h"
#endif

namespace pr::csv
{
	#if PR_CSV_USE_PRSTRING
	using Str = pr::string<char, 64>;
	#else
	using Str = std::string;
	#endif

	struct Row :std::vector<Str>
	{
		Row() :std::vector<Str>() {}
		Row(size_t sz) :std::vector<Str>(sz) {}
	};
	struct Csv :std::vector<Row>
	{
		Csv() :std::vector<Row>() {}
		Csv(size_t sz) :std::vector<Row>(sz) {}
	};
	struct Loc
	{
		std::size_t row, col;
		Loc()
			:row(0)
			,col(0)
		{}
		void inc(int ch)
		{
			if (ch == ',')
			{
				++col;
			}
			else if (ch == '\n')
			{
				++row;
				col = 0;
			}
		}
	};

	// Implementation
	namespace impl
	{
		struct BufferAdapter
		{
			char* m_data;
			size_t m_size;
			size_t m_capacity;
			
			template <int Capacity>
			explicit BufferAdapter(char (&item)[Capacity])
				:m_data(&item[0])
				,m_size(0)
				,m_capacity(Capacity)
			{
				m_data[0] = 0;
			}
			void push_back(char ch)
			{
				if (m_size >= m_capacity - 1)
					throw std::overflow_error("Provided character buffer is too small");

				m_data[m_size++] = ch;
				m_data[m_size] = 0;
			}
		};
	}

	// Range checked lookup
	inline Str const& Item(Row const& row, std::size_t col)                  { static Str null_str; return (col < row.size()) ? row[col] : null_str; }
	inline Row const& Item(Csv const& csv, std::size_t row)                  { static Row null_row; return (row < csv.size()) ? csv[row] : null_row; }
	inline Str const& Item(Csv const& csv, std::size_t row, std::size_t col) { return Item(Item(csv, row), col); }

	// Range checked lookup that throws when out of range
	inline Str const& ItemT(Row const& row, std::size_t col)                  { if (col >= row.size()) throw std::runtime_error("column index out of range"); return row[col]; }
	inline Row const& ItemT(Csv const& csv, std::size_t row)                  { if (row >= csv.size()) throw std::runtime_error("row index out of range"   ); return csv[row]; }
	inline Str const& ItemT(Csv const& csv, std::size_t row, std::size_t col) { return ItemT(ItemT(csv, row), col); }

	// CSV items can optionally be in quotes to allow elements to contain
	// ',' and '\n' characters. Quotes are escaped using double quotes.
	inline Str EscapeString(Str str)
	{
		// Search for characters that need escaping
		if (!std::any_of(std::begin(str), std::end(str), [](char c){ return c == '"' || c == ',' || c == '\n'; }))
			return str;

		Str out; out.reserve(str.size() * 11 / 10);
		out.push_back('"');
		for (auto ch : str)
		{
			if (ch == '"') out.push_back('"');
			out.push_back(ch);
		}
		out.push_back('"');
		return out;
	}
	inline Str UnescapeString(Str str)
	{
		if (str.size() < 2 || str[0] != '"')
			return str;

		Str out; out.reserve(str.size());
		auto ptr = str.c_str();
		for (++ptr; *ptr != 0; ++ptr)
		{
			if (*ptr == '"' && *(++ptr) != '"') break;
			out.push_back(*ptr);
		}
		if (*ptr != 0) throw std::runtime_error("'csv' string incorrectly escaped");
		return out;
	}

	// Set an element in the CSV
	inline Str& Item(Csv& csv, std::size_t row, std::size_t col)
	{
		if (row >= csv     .size()) csv     .resize(row + 1);
		if (col >= csv[row].size()) csv[row].resize(col + 1);
		return csv[row][col];
	}

	// Set an element in the CSV, throwing if out of range
	inline Str& ItemT(Csv& csv, std::size_t row, std::size_t col)
	{
		if (row >= csv     .size()) throw std::runtime_error("row index out of range"   );
		if (col >= csv[row].size()) throw std::runtime_error("column index out of range");
		return csv[row][col];
	}

	// This allows a 'Loc' to be used in a stream to remove ',' and '\n'
	// characters while also maintaining the CSV row,col position
	template <typename Stream>
	inline Stream& operator >> (Stream& s, Loc& loc)
	{
		int ch = s.peek();
		if (s.eof()) return s;
		if (s.bad()) throw std::runtime_error("invalid stream");
		if (ch != ',' && ch != '\n') throw std::runtime_error("expected a csv item or row delimiter");
		loc.inc(ch);
		s.get();
		return s;
	}

	// Read one element from a stream.
	// Returns true if a whole element was read, false if the stream is eof.
	// Throws on partial element or bad stream.
	// 's' will point at the next item (or eof) after a successful read.
	// Use 'loc' to determine new rows vs new items
	template <typename Stream>
	bool Read(Stream& s, Str& item, Loc& loc)
	{
		// Assume 's' is pointing to the first character of the item
		int ch = s.peek();
		if (s.eof()) return false;
		if (s.bad()) throw std::runtime_error("invalid stream");

		// If the first character is a quote, then this is a quoted item
		if (ch == '"')
		{
			s.get(); // skip the first '"'

			// Read to the next single '"'
			bool esc = false;
			for (ch = s.peek(); s.good(); s.get(), ch = s.peek())
			{
				if (ch == '"')
				{
					if (esc) item.push_back(static_cast<Str::value_type>(ch));
					esc = !esc;
				}
				else
				{
					if (esc) break;
					item.push_back(static_cast<Str::value_type>(ch));
				}
			}

			// Expect the quoted string to be closed
			if (!esc) // i.e. we should've seen one '"' followed by not '"' or eof
				throw std::runtime_error("incomplete CSV item");
		}

		// Read to the next ',' or '\n'
		for (ch = s.get(); s.good() && ch != ',' && ch != '\n'; ch = s.get())
			item.push_back(static_cast<Str::value_type>(ch));

		if (!s.eof()) loc.inc(ch);
		return true;
	}
	template <typename Stream, int N>
	bool Read(Stream& s, char (&item)[N], Loc& loc)
	{
		impl::BufferAdapter adp(item);
		return Read(s, adp, loc);
	}
	template <typename Stream>
	inline bool Read(Stream& s, Str& item)
	{
		Loc loc;
		return Read(s, item, loc);
	}
	template <typename Stream, std::size_t N>
	inline bool Read(Stream& s, char (&item)[N])
	{
		Loc loc;
		return Read(s, item, loc);
	}

	// Read one row from the stream
	// Returns true if a whole row is read, false if nothing was read
	// Throws on bad stream or partial element read.
	// 's' will point at the next item (or eof) after a successful read.
	// Use 'loc' to determine new rows vs new items
	template <typename Stream>
	bool Read(Stream& s, Row& row, Loc& loc)
	{
		// Assume 's' is pointing to the first character in a row of items
		int ch = s.peek();
		if (s.eof()) return false;
		if (s.bad()) throw std::runtime_error("invalid stream");

		// Empty row
		if (ch == '\n')
		{
			loc.inc(ch);
			s.get();
			return true;
		}

		// Read to the end of the row
		for (Str str; Read(s, str, loc);)
		{
			row.push_back(str);
			str.resize(0);
			if (loc.col == 0) break; // start of a new line means end of the row
		}
		return true;
	}
	template <typename Stream>
	inline bool Read(Stream& s, Row& row)
	{
		Loc loc;
		return Read(s, row, loc);
	}

	// Read all CSV data from a stream
	// Returns true if CSV is read, false if nothing was read
	// Throws on bad stream or partial element read.
	// 's' will point at eof after a successful read.
	template <typename Stream>
	bool Read(Stream& s, Csv& csv, Loc& loc)
	{
		// Assume 's' is pointing to the first character in csv data
		s.peek();
		if (s.eof()) return false;
		if (s.bad()) throw std::runtime_error("invalid stream");

		// Read to the end of the data
		for (Row row; Read(s, row, loc);)
		{
			csv.push_back(row);
			row.resize(0);
			if (s.eof()) break; // end of data
		}
		return true;
	}
	template <typename Stream>
	inline bool Read(Stream& s, Csv& csv)
	{
		Loc loc;
		return Read(s, csv, loc);
	}

	// Write one item to a stream
	template <typename Stream>
	inline void Write(Stream& s, Str const& item)
	{
		s << EscapeString(item);
	}

	// Write one row to a stream
	template <typename Stream>
	inline void Write(Stream& s, Row const& row)
	{
		auto count = row.size();
		for (auto item : row)
		{
			Write(s, item);
			if (--count != 0) s << ",";
		}
	}

	// Write all CSV data to a stream
	template <typename Stream>
	inline void Write(Stream& s, Csv const& csv)
	{
		auto count = csv.size();
		for (auto row : csv)
		{
			Write(s, row);
			if (--count != 0) s << '\n';
		}
	}

	// Write a CSV object to a file
	inline void Save(std::filesystem::path const& csv_filename, Csv const& csv)
	{
		std::ofstream out(csv_filename);
		if (!out.is_open()) throw std::runtime_error("failed to open file for writing");
		Write(out, csv);
	}

	// Populate a CSV object from a file
	inline bool Load(std::filesystem::path const& csv_filename, Csv& csv, Loc& loc)
	{
		std::ifstream in(csv_filename);
		if (!in.is_open()) throw std::runtime_error("failed to open file for reading");
		return Read(in, csv, loc);
	}
	inline bool Load(std::filesystem::path const& csv_filename, Csv& csv)
	{
		Loc loc;
		return Load(csv_filename, csv, loc);
	}

	// Insert an item delimiter into a stream
	inline Csv& __stdcall endi(Csv& csv)
	{
		if (csv.empty()) csv.push_back(Row());
		csv.back().push_back(Str());
		return csv;
	}
	inline Row& __stdcall endi(Row& row)
	{
		row.push_back(Str());
		return row;
	}

	// Insert a row delimiter into a stream
	inline Csv& __stdcall endr(Csv& csv)
	{
		csv.push_back(Row());
		return csv;
	}

	// CSV streaming operators
	inline Csv& operator << (Csv& csv, Str str)
	{
		if (csv.empty()) csv.push_back(Row());
		if (csv.back().empty()) csv.back().push_back(Str());
		csv.back().back().append(str);
		return csv;
	}
	inline Row& operator << (Row& row, Str str)
	{
		if (row.empty()) row.push_back(Str());
		row.back().append(str);
		return row;
	}

	// Item,row delimiter manipulator
	inline Csv& operator << (Csv& csv, Csv& (__stdcall *func)(Csv&))
	{
		return (*func)(csv);
	}
	inline Row& operator << (Row& row, Row& (__stdcall *func)(Row&))
	{
		return (*func)(row);
	}

	// Generate stream to CSV
	template <typename T> inline Csv& operator << (Csv& csv, T item)
	{
		std::stringstream ss; ss << item;
		return csv << Str(ss.str());
	}
	template <typename T> inline Row& operator << (Row& row, T item)
	{
		std::stringstream ss; ss << item;
		return row << Str(ss.str());
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::storage
{
	PRUnitTestClass(CsvTests)
	{
		using Str = csv::Str;
		using Csv = csv::Csv;

		std::filesystem::path test_csv = temp_dir() / L"test_csv.csv";

		PRUnitTestMethod(EscapeRoundTrip)
		{
			using namespace pr::csv;

			Str str = "A \"string\" with \r\n quotes, commas, and 'new' lines";
			auto esc = EscapeString(str);
			PR_EXPECT(esc == "\"A \"\"string\"\" with \r\n quotes, commas, and 'new' lines\"");
			auto ori = UnescapeString(esc);
			PR_EXPECT(ori == str);
		}
		PRUnitTestMethod(BasicCSV)
		{
			using namespace pr::csv;

			Csv csv;
			Item(csv, 1, 1) = "Hello";
			Item(csv, 1, 2) = "World";
			PR_EXPECT(ItemT(csv, 0).size() == 0U);
			PR_EXPECT(ItemT(csv, 1,0).size() == 0U);
			PR_EXPECT(ItemT(csv, 1,1) == "Hello");
			PR_EXPECT(ItemT(csv, 1,2) == "World");

			Loc loc;
			Csv csv2;
			Save(test_csv, csv);
			Load(test_csv, csv2, loc);
			PR_EXPECT(std::filesystem::remove(test_csv));
			PR_EXPECT(loc.row == 1 && loc.col == 2);

			PR_EXPECT(csv.size() == 2U);
			PR_EXPECT(csv[1].size() == 3U);
			PR_EXPECT(ItemT(csv, 0).size() == 0U);
			PR_EXPECT(ItemT(csv, 1,0).size() == 0U);
			PR_EXPECT(ItemT(csv, 1,1) == "Hello");
			PR_EXPECT(ItemT(csv, 1,2) == "World");
		}
		PRUnitTestMethod(SaveCSVWithEscapedItems)
		{
			using namespace pr::csv;

			Csv csv1;
			csv1 << "One" << endi << "Two" << endi << "Three" << endi << "\"Four\"" << endi << "\"," << "\r\n\"" << endr;
			csv1 << "1,1" << endi << "2\r2" << endi << "3\n3" << endi << "4\r\n" << endr;
			csv1 << endr;
			csv1 << 1 << endi << 3.14 << endi << '3' << endi << 16;

			Csv csv2;
			Save(test_csv, csv1);
			Load(test_csv, csv2);
			PR_EXPECT(std::filesystem::remove(test_csv));

			PR_EXPECT(csv2.size() == 4U);
			PR_EXPECT(csv2[0].size() == 5U);
			PR_EXPECT(csv2[1].size() == 4U);
			PR_EXPECT(csv2[2].size() == 0U);
			PR_EXPECT(csv2[3].size() == 4U);

			PR_EXPECT(csv2[0][0] == "One"      );
			PR_EXPECT(csv2[0][1] == "Two"      );
			PR_EXPECT(csv2[0][2] == "Three"    );
			PR_EXPECT(csv2[0][3] == "\"Four\"" );
			PR_EXPECT(csv2[0][4] == "\",\r\n\"");

			PR_EXPECT(csv2[1][0] == "1,1"  );
			PR_EXPECT(csv2[1][1] == "2\r2" );
			PR_EXPECT(csv2[1][2] == "3\n3" );
			PR_EXPECT(csv2[1][3] == "4\r\n");

			PR_EXPECT(csv2[3][0] == "1"  );
			PR_EXPECT(csv2[3][1] == "3.14" );
			PR_EXPECT(csv2[3][2] == "3" );
			PR_EXPECT(csv2[3][3] == "16");
		}
		PRUnitTestMethod(StreamCSV)
		{
			using namespace pr::csv;

			/*
			std::ofstream outf("test_csv.csv");
			outf
				<< "Hello,World\n"
				<< "a,,b,c\n"
				<< ",skip\n"
				<< 1 << ',' << 2.0f << ',' << 3.0 << '\n';
			outf.close();

			std::ifstream inf("test_csv.csv");

			Loc loc;
			Str s0; char s1[10];
			PR_EXPECT(Read(inf, s0, loc));
			PR_EXPECT(Read(inf, s1, loc));
			PR_EXPECT(s0 == "Hello");
			PR_EXPECT(std::string(s1) == "World");
			PR_EXPECT(loc.row = 1 && loc.col == 0);

			char ch;
			inf >> ch >> loc; PR_EXPECT(ch == 'a');
			inf >> loc;       PR_EXPECT(loc.row == 1 && loc.col == 1);
			inf >> ch >> loc; PR_EXPECT(ch == 'b');
			inf >> ch >> loc; PR_EXPECT(ch == 'c');

			PR_EXPECT(loc.row == 2 && loc.col == 0);
			csv >> loc;
			PR_EXPECT(loc.row == 2 && loc.col == 1);
			csv >> loc;
			PR_EXPECT(loc.row == 3 && loc.col == 0);

			int i; float f; double d;
			csv >> i >> loc >> f >> loc >> d >> loc;
			PR_EXPECT(i == 1);
			PR_EXPECT(f == 2.0f);
			PR_EXPECT(d == 3.0);

			PR_EXPECT(loc.row == 4 && loc.col == 0);
			PR_EXPECT(filesys::EraseFile<std::string>("test_csv.csv"));
			*/
		}
	};
}
#endif