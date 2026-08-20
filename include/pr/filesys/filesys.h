//**********************************************
// File path/File system operations
//  Copyright (c) Rylogic Ltd 2009
//**********************************************
#pragma once
#if 0
#include <cstdint>
#include <string>
#include <algorithm>
#include <type_traits>
#include <filesystem>
#include <iterator>
#include <fstream>
#include <locale>
#include <codecvt>
#include <cassert>
#include "pr/str/char8.h"
#include "pr/str/string_core.h"

namespace pr::filesys
{
	// Compare two, possibly non-existent, paths.
	inline int Compare(std::filesystem::path const& lhs, std::filesystem::path const& rhs, bool ignore_case)
	{
		auto n0 = lhs.lexically_normal();
		auto n1 = rhs.lexically_normal();
		if (ignore_case)
		{
			n0 = str::LowerCaseC(n0.wstring());
			n1 = str::LowerCaseC(n1.wstring());
		}
		auto r = n0.compare(n1);
		return r;
	}

	// Compare two, possibly non-existent, paths for equality.
	inline bool Equal(std::filesystem::path const& lhs, std::filesystem::path const& rhs, bool ignore_case)
	{
		return Compare(lhs, rhs, ignore_case) == 0;
	}

	// Compare the contents of two files and return true if they are the same.
	// Returns true if both files doesn't exist, or false if only one file exists.
	template <typename = void>
	bool EqualContents(std::filesystem::path const& lhs, std::filesystem::path const& rhs)
	{
		using namespace std::filesystem;

		// Both must exist or not exist
		auto e0 = exists(lhs);
		auto e1 = exists(rhs);
		if (!e0 || !e1)
			return !e0 && !e1;
		
		// Both must be files
		if (is_directory(lhs))
			throw std::runtime_error("EqualContents: 'lhs' is a directory, file expected.");
		if (is_directory(rhs))
			throw std::runtime_error("EqualContents: 'lhs' is a directory, file expected.");

		// Comparing the same file
		if (equivalent(lhs, rhs))
			return true; 

		std::ifstream f0(lhs, std::ios::binary);
		std::ifstream f1(rhs, std::ios::binary);

		// Both must have the same lengh
		auto s0 = f0.seekg(0, std::ifstream::end).tellg();
		auto s1 = f1.seekg(0, std::ifstream::end).tellg();
		if (s0 != s1)
			return false;

		f0.seekg(0, std::ifstream::beg);
		f1.seekg(0, std::ifstream::beg);

		// Both must have the same content
		enum { BlockSize = 4096 };
		char buf0[BlockSize];
		char buf1[BlockSize];
		for (; f0 && f1;)
		{
			auto r0 = static_cast<size_t>(f0.read(buf0, sizeof(buf0)).gcount());
			auto r1 = static_cast<size_t>(f1.read(buf1, sizeof(buf1)).gcount());
			if (r0 != r1) return false;
			if (memcmp(buf0, buf1, r1) != 0) return false;
		}
		
		// Both must reach EOF at the same time
		return f0.eof() == f1.eof();
	}

	// Simple read/write a text file into memory
	inline std::u8string ReadAllText(std::filesystem::path const& filepath)
	{
		std::ifstream ifile(filepath);
		return std::u8string(std::istreambuf_iterator<char>(ifile), {});
	}
	inline void WriteAllText(std::u8string_view text, std::filesystem::path const& filepath)
	{
		std::ofstream ifile(filepath);
		ifile.write(char_ptr(text.data()), text.size());
	}

	// Examines 'filepath' to guess at the file data encoding (assumes 'filepath' is a text file)
	// On return 'bom_size' is the length of the byte order mask.
	// Returns 'UTF-8' if unknown, since UTF-8 recommends not using BOMs
	inline EEncoding DetectFileEncoding(std::filesystem::path const& filepath, int& bom_size)
	{
		std::ifstream file(filepath, std::ios::binary);

		unsigned char bom[3];
		auto read = file.good() ? file.read(reinterpret_cast<char*>(&bom[0]), sizeof(bom)).gcount() : 0;
		if (read >= 3 && bom[0] == 0xEF && bom[1] == 0xBB && bom[2] == 0xBF)
		{
			bom_size = 3;
			return EEncoding::utf8;
		}
		if (read >= 2 && bom[0] == 0xFE && bom[1] == 0xFF)
		{
			bom_size = 2;
			return EEncoding::utf16_be;
		}
		if (read >= 2 && bom[0] == 0xFF && bom[1] == 0xFE)
		{
			bom_size = 2;
			return EEncoding::utf16_le;
		}

		// Assume utf-8 unless we find some invalid utf-8 sequences
		bom_size = 0;
		auto char_ofs = 0;

		// Scan the file (or some of it) to look for invalid utf-8 sequences
		unsigned char buf[4096];
		constexpr int jend = 0x100000 / sizeof(buf);
		for (int j = 0, margin = 0; j != jend && file.good(); ++j)
		{
			// Read a chunk of the file.
			auto count = file.read(reinterpret_cast<char*>(&buf[margin]), sizeof(buf) - margin).gcount();

			// Scan up to sizeof(buf) - 8 to handle multi-byte characters than span the buffer boundary.
			std::streamsize i = 0, iend = std::min<std::streamsize>(count, sizeof(buf) - 8);
			for (;i < iend; ++i, ++char_ofs)
			{
				auto c =
					(buf[i] & 0b10000000) == 0          ? 0 : // ASCII character, 0 continuation bytes
					(buf[i] & 0b11100000) == 0b11000000 ? 1 : // 2-byte UTF-8 character, 1 continuation byte
					(buf[i] & 0b11110000) == 0b11100000 ? 2 : // 3-byte UTF-8 character, 2 continuation bytes
					(buf[i] & 0b11111000) == 0b11110000 ? 3 : // 4-byte UTF-8 character, 3 continuation bytes
					-1;                                       // Not a valid UTF-8 character

				if (c == 0)
					continue;

				// Continuation bytes should be 0b10xxxxxx for valid utf-8
				for (; c > 0 && (buf[++i] & 0b11000000) == 0b10000000; --c) {}

				// Not valid UTF-8, assume extended ascii
				if (c != 0)
					return EEncoding::ascii_extended;
			}

			// Reached the end of the file?
			if (count != s_cast<std::streamsize>(sizeof(buf) - margin))
				break;

			// Copy the unscanned bytes to the start of buf and go round again
			margin = s_cast<int>(sizeof(buf) - i);
			memmove(&buf[0], &buf[i], margin);
		}
		return EEncoding::utf8;
	}
	inline EEncoding DetectFileEncoding(std::filesystem::path const& filepath)
	{
		int bom_size;
		return DetectFileEncoding(filepath, bom_size);
	}

	// Read the contents of a file into 'buf'
	// 'buf_enc' is how to convert the file data. The encoding in the file is automatically detected and converted to 'buf_enc'
	// 'file_enc' is used to say what the file encoding is. If equal to Text then the encoding is automatically detected
	template <typename Buf>
	inline bool FileToBuffer(std::filesystem::path const& filepath, Buf& buf, EEncoding buf_enc = EEncoding::binary)
	{
		using Elem = std::decay_t<decltype(buf[0])>;

		// Ensure the file exists
		if (!std::filesystem::exists(filepath))
			return false;

		// Determine the file size in bytes
		auto size = static_cast<size_t>(std::filesystem::file_size(filepath));

		// If we need to do a text encoding conversion...
		if (buf_enc != EEncoding::binary)
		{
			// Detect the file encoding and BOM length.
			int bom_length = 0;
			auto file_enc = DetectFileEncoding(filepath, bom_length);

			// Open the file stream
			std::basic_ifstream<Elem> file(filepath, std::ios::binary);
			if (!file.good())
				return false;

			// Imbue the file if the file encoding doesn't match the buffer encoding
			switch (file_enc)
			{
				case EEncoding::ascii:
				case EEncoding::utf8:
				{
					if (false) {}
					else if (buf_enc == EEncoding::ascii || buf_enc == EEncoding::utf8)
					{
					}
					else if (buf_enc == EEncoding::utf16_le)
					{
						using cvt_t = std::codecvt_utf8_utf16<wchar_t>;
						file.imbue(std::locale(file.getloc(), new cvt_t));
					}
					else if (buf_enc == EEncoding::ucs2_le)
					{
						using cvt_t = std::codecvt_utf8<wchar_t>;
						file.imbue(std::locale(file.getloc(), new cvt_t));
					}
					else
					{
						throw std::runtime_error("todo");
					}
					break;
				}
				case EEncoding::ascii_extended:
				{
					if (false) {}
					else if (buf_enc == EEncoding::ascii || buf_enc == EEncoding::ascii_extended)
					{
					}
					else if (buf_enc == EEncoding::utf16_le || buf_enc == EEncoding::ucs2_le)
					{
						// treats the characters as ascii
					}
					else
					{
						// extended ascii involves code pages...
						throw std::runtime_error("todo");
					}
					break;
				}
				case EEncoding::utf16_le:
				case EEncoding::ucs2_le:
				{
					// Imbue the file if the file encoding doesn't match the buffer encoding
					if (false) {}
					else if (buf_enc == EEncoding::utf8 || buf_enc == EEncoding::ascii || buf_enc == EEncoding::ascii_extended)
					{
						// There doesn't seem to be a codecvt instance that handles utf16 -> utf8
						// Have to read utf16 into a string then convert it to utf8
						std::wifstream wfile(filepath, std::ios::binary);
						if (!wfile.good())
							return false;

						using cvt_t = std::codecvt_utf16<wchar_t, 0x10ffff, std::codecvt_mode::little_endian>;
						wfile.imbue(std::locale(wfile.getloc(), new cvt_t));

						// Skip the BOM
						wfile.seekg(bom_length, std::ios::beg);
						size -= bom_length;

						// Read with formatting
						if (size != 0)
						{
							std::wstring tmp(size, 0);
							auto read = wfile.read(tmp.data(), size).gcount();
							tmp.resize(static_cast<size_t>(read));

							std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>, wchar_t> converter;
							auto u8 = converter.to_bytes(tmp);
							auto ptr = reinterpret_cast<Elem const*>(u8.data());
							assert(sizeof(Elem) == sizeof(char));
							buf.insert(end(buf), ptr, ptr + u8.size());
						}
						return true;

						// Tried:
						//using cvt_t = std::codecvt_utf8_utf16<wchar_t>;
						//using cvt_t = std::codecvt_utf8_utf16<char>;
						//using cvt_t = std::codecvt<char16_t, Elem, std::mbstate_t>;
						//using cvt_t = std::codecvt_utf8<wchar_t>;
						//using cvt_t = std::codecvt_utf16<wchar_t, 0x10ffff, std::codecvt_mode::little_endian>;
						//using cvt_t = std::codecvt_utf16<char16_t, 0x10ffff, std::codecvt_mode::little_endian>;
						//file.imbue(std::locale(file.getloc(), new cvt_t));
					}
					else if (buf_enc == EEncoding::utf16_le || buf_enc == EEncoding::ucs2_le)
					{
						using cvt_t = std::codecvt_utf16<Elem, 0x10ffff, std::codecvt_mode::little_endian>;
						file.imbue(std::locale(file.getloc(), new cvt_t));
					}
					else if (buf_enc == EEncoding::utf16_be || buf_enc == EEncoding::ucs2_be)
					{
						using cvt_t = std::codecvt_utf16<Elem>;
						file.imbue(std::locale(file.getloc(), new cvt_t));
					}
					else
					{
						throw std::runtime_error("todo");
					}
					break;
				}
				default:
				{
					throw std::runtime_error("todo");
				}
			}

			// Skip the BOM
			file.seekg(bom_length, std::ios::beg);
			size -= bom_length;

			// Read with formatting (appending to 'buf')
			if (size != 0)
			{
				auto initial = buf.size();
				buf.resize(size_t(initial + size));
				auto read = file.read(reinterpret_cast<Elem*>(&buf[initial]), size).gcount();
				buf.resize(size_t(initial + read));

				//if (read != size)
				//	throw std::runtime_error("Partial file read in FileToBuffer (text)");
			}
		}
		else
		{
			// Open the file stream
			std::ifstream file(filepath, std::ios::binary);
			if (!file.good())
				return false;

			// Read unformatted (appending to 'buf')
			if (size != 0)
			{
				auto initial = buf.size();
				buf.resize(size_t(initial + (size + sizeof(Elem) - 1) / sizeof(Elem)));
				auto read = file.read(reinterpret_cast<char*>(&buf[initial]), size).gcount();
				buf.resize(size_t(initial + (read + sizeof(Elem) - 1) / sizeof(Elem)));

				//if (read != size)
				//	throw std::runtime_error("Partial file read in FileToBuffer (binary)");
			}
		}
		return true;
	}
	template <typename Buf>
	inline Buf FileToBuffer(std::filesystem::path const& filepath, EEncoding buf_enc = EEncoding::binary)
	{
		Buf buf;
		if (!FileToBuffer(filepath, buf, buf_enc)) throw std::runtime_error("Failed to read file");
		return std::move(buf);
	}

	// Write a buffer to a file.
	// 'buf' points to the contiguous block of data to write
	// 'size' is the length to write (in bytes)
	// 'filepath' is the name of the file to create
	// 'file_enc' describes the encoding to be written to the file.
	// 'buf_enc' describes the encoding used in 'buf'
	// 'append' is true if the file should be appended to
	// 'add_bom' is true if a byte order mask should be written to the file (applies to text encoding only, prefer not for UTF-8)
	template <typename = void>
	bool BufferToFile(void const* buf, size_t size, std::filesystem::path const& filepath, EEncoding file_enc = EEncoding::binary, EEncoding buf_enc = EEncoding::binary, bool append = false, bool add_bom = false)
	{
		// Open the output file stream
		std::ofstream file(filepath, std::ios::binary | (append ? std::ios::app : 0));
		if (!file.good())
			return false;

		if (file_enc != EEncoding::binary)
		{
			// Add the byte order mask
			if (add_bom && file_enc != EEncoding::auto_detect)
			{
				unsigned char bom[3] = {}; int bom_length = 0;
				if (false) {}
				else if (file_enc == EEncoding::utf8)     { bom[0] = 0xEF; bom[1] = 0xBB; bom[2] = 0xBF; bom_length = 3; }
				else if (file_enc == EEncoding::utf16_le) { bom[0] = 0xFF; bom[1] = 0xFE; bom_length = 2; }
				else if (file_enc == EEncoding::utf16_be) { bom[0] = 0xFE; bom[1] = 0xFF; bom_length = 2; }
				else throw std::runtime_error("Cannot write the byte order mask for an unknown text encoding");
				if (!file.write(reinterpret_cast<char const*>(&bom[0]), bom_length).good())
					throw std::runtime_error("Failed to write the byte order mask");
			}

			// Imbue the file stream if the buffer encoding doesn't match the file encoding
			if (file_enc != buf_enc)
			{
				switch (file_enc)
				{
					case EEncoding::utf8:
					{
						if (false) {}
						else if (buf_enc == EEncoding::utf16_le)
						{
							// 'std::codecvt<char16_t,char,mbstate_t>::codecvt': warning STL4020:
							// std::codecvt<char16_t, char, mbstate_t>,
							// std::codecvt<char32_t, char, mbstate_t>,
							// std::codecvt_byname<char16_t, char, mbstate_t>, and
							// std::codecvt_byname<char32_t, char, mbstate_t>
							// are deprecated in C++20 and replaced by specializations with a second
							// argument of type char8_t.
							// You can define _SILENCE_CXX20_CODECVT_FACETS_DEPRECATION_WARNING or _SILENCE_ALL_CXX20_DEPRECATION_WARNINGS to acknowledge that you have received this warning.

							using cvt_t = std::codecvt<char16_t, char8_t, std::mbstate_t>;
							file.imbue(std::locale(file.getloc(), new cvt_t));
						}
						else
						{
							throw std::runtime_error("todo");
						}
						break;
					}
					case EEncoding::utf16_le:
					{
						if (false) {}
						else if (buf_enc == EEncoding::utf8)
						{
							using cvt_t = std::codecvt_utf16<wchar_t, 0x10ffff, std::codecvt_mode::little_endian>;
							file.imbue(std::locale(file.getloc(), new cvt_t));
						}
						else
						{
							throw std::runtime_error("todo");
						}
						break;
					}
					default:
					{
						throw std::runtime_error("todo");
					}
				}
			}
		}

		// Write the data to the file
		file.write(static_cast<char const*>(buf), size);
		return true;
	}

	// Write a buffer to a file.
	// 'ofs' and 'count' are the sub-range to write (in units of 'Elem')
	template <typename Buf, typename Elem = Buf::value_type>
	inline bool BufferToFile(Buf const& buf, size_t ofs, size_t len, std::filesystem::path const& filepath, EEncoding file_enc = EEncoding::binary, EEncoding buf_enc = EEncoding::binary, bool append = false, bool add_bom = false)
	{
		return buf.empty()
			? BufferToFile((void const*)nullptr, 0U, filepath, file_enc, buf_enc, append, add_bom)
			: BufferToFile(&buf[ofs], len * sizeof(Elem), filepath, file_enc, buf_enc, append, add_bom);
	}
	template <typename Buf, typename Elem = Buf::value_type>
	inline bool BufferToFile(Buf const& buf, std::filesystem::path const& filepath, EEncoding file_enc = EEncoding::binary, EEncoding buf_enc = EEncoding::binary, bool append = false, bool add_bom = false)
	{
		return BufferToFile(buf, 0, buf.size(), filepath, file_enc, buf_enc, append, add_bom);
	}
	template <typename Elem>
	inline bool BufferToFile(Elem const* buf, int64_t ofs, int64_t count, std::filesystem::path const& filepath, EEncoding file_enc = EEncoding::binary, EEncoding buf_enc = EEncoding::binary, bool append = false, bool add_bom = false)
	{
		return BufferToFile(buf + ofs, static_cast<size_t>(count * sizeof(Elem)), filepath, file_enc, buf_enc, append, add_bom);
	}
	template <typename Elem>
	inline bool BufferToFile(std::initializer_list<Elem> items, std::filesystem::path const& filepath, EEncoding file_enc = EEncoding::binary, EEncoding buf_enc = EEncoding::binary, bool append = false, bool add_bom = false)
	{
		return BufferToFile(items.begin(), items.size() * sizeof(Elem), filepath, file_enc, buf_enc, append, add_bom);
	}

	// Write a string to a file
	inline bool BufferToFile(std::string_view buf, std::filesystem::path const& filepath, EEncoding file_enc = EEncoding::utf8, EEncoding buf_enc = EEncoding::utf8, bool append = false, bool add_bom = false)
	{
		return BufferToFile(buf.data(), buf.size(), filepath, file_enc, buf_enc, append, add_bom);
	}
	inline bool BufferToFile(std::wstring_view buf, std::filesystem::path const& filepath, EEncoding file_enc = EEncoding::utf8, EEncoding buf_enc = EEncoding::utf16_le, bool append = false, bool add_bom = false)
	{
		return BufferToFile(buf.data(), buf.size(), filepath, file_enc, buf_enc, append, add_bom);
	}

	// Attempt to resolve a partial filepath given a list of directories to search
	template <typename PathCont = std::vector<std::filesystem::path>>
	std::filesystem::path ResolvePath(std::filesystem::path const& partial_path, PathCont const& search_paths = PathCont(), std::filesystem::path const* current_dir = nullptr, bool check_working_dir = true, PathCont* searched_paths = nullptr)
	{
		using namespace std::filesystem;

		// If the partial path is actually a full path
		if (partial_path.is_absolute())
		{
			// Return an empty string for unresolved
			return exists(partial_path) ? partial_path : path{};
		}

		// If a current directory is provided
		if (current_dir != nullptr)
		{
			auto path = *current_dir / partial_path;
			if (exists(path))
				return path;

			if (searched_paths)
				searched_paths->push_back(path.parent_path());
		}

		// Check the working directory
		if (check_working_dir)
		{
			// Convert to an absolute path using the current working directory
			auto path = absolute(partial_path);
			if (exists(path))
				return path;

			if (searched_paths)
				searched_paths->push_back(path.parent_path());
		}

		// Search the search paths
		for (auto& dir : search_paths)
		{
			auto path = (dir / partial_path).lexically_normal();
			if (exists(path))
				return path;

			// If the search paths contain partial paths, resolve recursively
			if (!path.is_absolute())
			{
				auto paths = search_paths;
				paths.erase(std::remove_if(begin(paths), end(paths), [&](auto& p) { return p == dir; }), end(paths));
				path = ResolvePath(path, paths, current_dir, check_working_dir, searched_paths);
				if (exists(path))
					return path;
			}

			if (searched_paths)
				searched_paths->push_back(path.parent_path());
		}

		// Return an empty string for unresolved
		return path{};
	}
}

#if PR_UNITTESTS
#include <fstream>
#include "pr/common/unittests.h"
namespace pr::filesys
{
	PRUnitTest(FilesysTests, Quick)
	{
		using namespace std::filesystem;

		{// std::filesystem tests
			auto p0 = path{ L"C:\\dir\\file.txt" };
			auto p1 = path{ "C:/DIR/DIR2/../FiLE.TXT" };

			// Just turns the path into normal form
			auto n0 = p0.lexically_normal();
			auto n1 = p1.lexically_normal();

			// Weakly canonical resolves simlinks for the parts of the path that actually exist.
			auto c0 = weakly_canonical(p0);
			auto c1 = weakly_canonical(p1);

			// equivalent requires the file/directory to exist
			PR_THROWS([[maybe_unused]] auto _ = equivalent(p0, p1), filesystem_error);

			// canonical requires the file/directory to exist
			PR_THROWS([[maybe_unused]] auto _ = canonical(p0), filesystem_error);

			//
			try
			{
				//// Path compare is case insensitive (on windows)
				//auto r0 = weakly_canonical(p0).compare(weakly_canonical(p1));
				//PR_EXPECT(r0 == 0);
			
			}
			catch (filesystem_error const& ex)
			{
				PR_UNITTEST_OUT << ex.what();
				throw;
			}
		}
		{// Equal paths
			auto p0 = path{ L"C:\\dir\\file.txt" };
			auto p1 = path{ "C:/DIR/DIR2/../FiLE.TXT" };
			PR_EXPECT(Equal(p0, p1, true));
		}
		{// Equal contents
			auto file_content0 = temp_dir / L"file_content0.bin";
			auto file_content1 = temp_dir / L"file_content1.bin";
			auto file_content2 = temp_dir / L"file_content2.bin";

			char const content0[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
			char const content1[] = { 0, 1, 2, 3, 'A', 5, 6, 7, 8, 9 };
			{
				std::ofstream file(file_content0);
				PR_EXPECT(file.write(&content0[0], sizeof(content0)).good());
			}
			{
				std::ofstream file(file_content1);
				PR_EXPECT(file.write(&content1[0], sizeof(content1)).good());
			}
			{
				std::ofstream file(file_content2);
				PR_EXPECT(file.write(&content0[0], sizeof(content0)).good());
			}

			PR_EXPECT(EqualContents(file_content0, file_content0));
			PR_EXPECT(!EqualContents(file_content0, file_content1));
			PR_EXPECT(EqualContents(file_content0, file_content2));
		}
		{// Buffer to/from File
			auto filepath = temp_dir / L"file_test.txt";
			{// Simple read text file
				std::u8string text = char8_ptr(u8"你好，This is some test text");
				WriteAllText(text, filepath);
				auto read = ReadAllText(filepath);
				PR_CHECK(read, text);
			}
			{// Write binary - Read binary
				{// write bytes
					unsigned char const data[] = { '0', '1', '2', '3', '4', '5' };
					BufferToFile(data, 0, _countof(data), filepath, EEncoding::binary);

					{// Read binary data into 'std::vector<byte>', no conversion
						auto read = FileToBuffer<std::vector<unsigned char>>(filepath, EEncoding::binary);
						PR_EXPECT(read.size() == sizeof(data));
						PR_EXPECT(memcmp(&read[0], data, sizeof(data)) == 0);
					}
					{// Read binary data into 'std::wstring', no conversion
						auto read = FileToBuffer<std::wstring>(filepath, EEncoding::binary);
						PR_EXPECT(read.size() == (sizeof(data) + 1) / 2);
						PR_EXPECT(memcmp(&read[0], data, sizeof(data)) == 0);
					}
				}
				{// write !bytes
					unsigned short const data[] = { '0', '1', '2', '3', '4', '5' };
					BufferToFile(data, 0, _countof(data), filepath, EEncoding::binary);

					{// Read binary data into 'std::vector<byte>', no conversion
						auto read = FileToBuffer<std::vector<unsigned char>>(filepath, EEncoding::binary);
						PR_EXPECT(read.size() == sizeof(data));
						PR_EXPECT(memcmp(&read[0], data, sizeof(data)) == 0);
					}
					{// Read binary data into 'std::wstring', no conversion
						auto read = FileToBuffer<std::wstring>(filepath, EEncoding::binary);
						PR_EXPECT(read.size() == sizeof(data) / 2);
						PR_EXPECT(memcmp(&read[0], data, sizeof(data)) == 0);
					}
				}
			}
			{// Write UTF-8 text
				unsigned char const utf8[] = { 0xe4, 0xbd, 0xa0, 0xe5, 0xa5, 0xbd, '\n', 0xe4, 0xbd, 0xa0, 0xe5, 0xa5, 0xbd }; // 'ni hao'
				wchar_t const utf16[] = { 0x4f60, 0x597d, '\n', 0x4f60, 0x597d };

				BufferToFile(utf8, 0, _countof(utf8), filepath, EEncoding::utf8, EEncoding::utf8, false, true);
				PR_EXPECT(DetectFileEncoding(filepath) == EEncoding::utf8);

				{// Read UTF-8 - BOM automatically stripped
					auto read = FileToBuffer<std::string>(filepath, EEncoding::utf8);
					PR_EXPECT(read.size() == _countof(utf8));
					PR_EXPECT(memcmp(&read[0], utf8, sizeof(utf8)) == 0);
				}
				{// Read UTF-8 to UTF-16 - BOM automatically stripped
					auto read = FileToBuffer<std::wstring>(filepath, EEncoding::utf16_le);
					PR_EXPECT(read.size() == _countof(utf16));
					PR_EXPECT(memcmp(&read[0], utf16, sizeof(utf16)) == 0);
				}
				{// Read UTF-8 to UCS2 - BOM automatically stripped
					auto read = FileToBuffer<std::wstring>(filepath, EEncoding::ucs2_le);
					PR_EXPECT(read.size() == _countof(utf16));
					PR_EXPECT(memcmp(&read[0], utf16, sizeof(utf16)) == 0);
				}
			}
			{// Write UTF-16 text
				unsigned char const utf8[] = { 0xe4, 0xbd, 0xa0, 0xe5, 0xa5, 0xbd, '\n', 0xe4, 0xbd, 0xa0, 0xe5, 0xa5, 0xbd }; // 'ni hao'
				wchar_t const utf16[] = { 0x4f60, 0x597d, '\n', 0x4f60, 0x597d };
				wchar_t const utf16be[] = { 0x604f, 0x7d59, 0x0A00, 0x604f, 0x7d59 };

				BufferToFile(utf16, 0, _countof(utf16), filepath, EEncoding::utf16_le, EEncoding::utf16_le, false, true);
				PR_EXPECT(DetectFileEncoding(filepath) == EEncoding::utf16_le);

				{// Read UTF-16 to UTF-16 - BOM automatically stripped
					auto read = FileToBuffer<std::wstring>(filepath, EEncoding::utf16_le);
					PR_EXPECT(read.size() == _countof(utf16));
					PR_EXPECT(memcmp(&read[0], utf16, sizeof(utf16)) == 0);
				}
				{// Read UTF-16 to UTF-16be - BOM automatically stripped
					auto read = FileToBuffer<std::wstring>(filepath, EEncoding::utf16_be);
					PR_EXPECT(read.size() == _countof(utf16be));
					PR_EXPECT(memcmp(&read[0], utf16be, sizeof(utf16be)) == 0);
				}
				{// Read UTF-16 to UTF-8 - BOM automatically stripped
					auto read = FileToBuffer<std::string>(filepath, EEncoding::utf8);
					PR_EXPECT(read.size() == _countof(utf8));
					PR_EXPECT(memcmp(&read[0], utf8, sizeof(utf8)) == 0);
				}
			}
			{// todo...
			}
		}
		{// Enumerate filesystem
			create_directories(temp_dir / L"dir1" / L"dir2");
			BufferToFile({ 0, 1, 2, 3, 4 }, temp_dir / L"dir1" / L"bytes.bin");
			BufferToFile("0123456789", temp_dir / L"dir1" / L"digits.txt");
			BufferToFile("ABCDEFGHIJ", temp_dir / L"dir1" / L"dir2" / L"letters.txt");

			std::vector<path> files;
			std::vector<path> dirs;
			for (auto& dir_entry : recursive_directory_iterator(temp_dir / L"dir1"))
			{
				if (dir_entry.is_directory())
					dirs.push_back(dir_entry.path());
				else
					files.push_back(dir_entry.path());
			}
			PR_CHECK(files.size(), 3U);
			PR_EXPECT(Equal(files[0], temp_dir / L"dir1" / L"bytes.bin", true));
			PR_EXPECT(Equal(files[1], temp_dir / L"dir1" / L"digits.txt", true));
			PR_EXPECT(Equal(files[2], temp_dir / L"dir1" / L"dir2" / L"letters.txt", true));
			PR_CHECK(dirs.size(), 1U);
			PR_EXPECT(Equal(dirs[0], temp_dir / L"dir1" / L"dir2", true));
		}
	}
}
#endif








#define PR_FILESYS_NOT_DEPRECATED 0
#if PR_FILESYS_NOT_DEPRECATED
#pragma warning(disable:4996)
//#pragma warning(push, 1)
//#include <io.h>
//#include <direct.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#pragma warning(pop)

// Pathname  = full path e.g. Drive:/path/path/file.ext
// Drive     = the drive e.g. "P". No ':'
// Path      = the directory without the drive. No leading '/', no trailing '/'. e.g. Path/path
// Directory = the drive + path. no trailing '/'. e.g P:/Path/path
// Extension = the last string following a '.'
// Filename  = file name including extension
// FileTitle = file name not including extension
// A full pathname = drive + ":/" + path + "/" + file-title + "." + extension
//
#include "pr/str/string_core.h"

namespace pr::filesys
{
	enum class EAttrib
	{
		None = 0,
		Device = 1 << 0,
		File = 1 << 1,
		Directory = 1 << 2,
		Pipe = 1 << 3,
		WriteAccess = 1 << 4,
		ReadAccess = 1 << 5,
		ExecAccess = 1 << 6,
		_flags_enum = 0,
	};
	enum class Access
	{
		Exists = 0,
		Write = 2,
		Read = 4,
		ReadWrite = Write | Read
	};

	// File timestamp data for a file. Note: these timestamps are in UTC Unix time
	struct FileTime
	{
		time_t m_last_access;   // Note: time_t is 64bit
		time_t m_last_modified;
		time_t m_created;
	};

#if 0
	#pragma region Traits
	template <typename C> struct is_char :std::false_type {};
	template <>           struct is_char<char> :std::true_type {};
	template <>           struct is_char<wchar_t> :std::true_type {};
	template <typename Char> using enable_if_char = typename std::enable_if<is_char<Char>::value>::type;
	#pragma endregion

	#pragma region String helpers
	template <typename Char, typename = std::enable_if_t<is_char_v<Char>>> inline Char const* c_str(Char const* s)
	{
		return s;
	}

	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>> inline Char const* c_str(Str const& s)
	{
		return s.c_str();
	}

	// Returns a pointer in 's' of the first instance of 'x' or the null terminator
	template <typename Char, typename = std::enable_if_t<is_char_v<Char>>> inline Char const* find(Char const* s, Char x)
	{
		for (; *s != 0 && *s != x; ++s) {}
		return s;
	}

	// Returns a pointer into 's' of the first instance of any character in 'x'
	template <typename Char, typename = std::enable_if_t<is_char_v<Char>>> inline Char const* find_first(Char const* s, Char const* x)
	{
		for (; *s != 0 && *find(x,*s) == 0; ++s) {}
		return s;
	}
	#pragma endregion

	#pragma region wchar_t/ char handling
	namespace impl
	{
		inline int strlen(char    const* s) { return int(::strlen(s)); }
		inline int strlen(wchar_t const* s) { return int(::wcslen(s)); }

		inline int access(char    const* filename, Access access_mode) { return ::_access (filename, int(access_mode)); }
		inline int access(wchar_t const* filename, Access access_mode) { return ::_waccess(filename, int(access_mode)); }

		inline int rename(char    const* oldname, char    const* newname) { return ::rename  (oldname, newname); }
		inline int rename(wchar_t const* oldname, wchar_t const* newname) { return ::_wrename(oldname, newname); }

		inline int remove(char    const* path) { return ::remove  (path); }
		inline int remove(wchar_t const* path) { return ::_wremove(path); }

		inline int mkdir(char    const* dirname) { return ::_mkdir (dirname); }
		inline int mkdir(wchar_t const* dirname) { return ::_wmkdir(dirname); }

		inline int rmdir(char    const* dirname) { return ::_rmdir (dirname); }
		inline int rmdir(wchar_t const* dirname) { return ::_wrmdir(dirname); }

		inline char*    getdcwd(int drive, char*    buf, size_t buf_size)    { return ::_getdcwd(drive, buf, int(buf_size)); }
		inline wchar_t* getdcwd(int drive, wchar_t* buf, size_t buf_size) { return ::_wgetdcwd(drive, buf, int(buf_size)); }

		inline char*    fullpath(char*    abs_path, char    const* rel_path, size_t max_length) { return ::_fullpath (abs_path, rel_path, max_length); }
		inline wchar_t* fullpath(wchar_t* abs_path, wchar_t const* rel_path, size_t max_length) { return ::_wfullpath(abs_path, rel_path, max_length); }

		inline int stat64(char    const* filepath, struct __stat64* info) { return _stat64(filepath, info); }
		inline int stat64(wchar_t const* filepath, struct __stat64* info) { return _wstat64(filepath, info); }

		inline int chmod(char    const* filepath, int mode) { return _chmod(filepath, mode); }
		inline int chmod(wchar_t const* filepath, int mode) { return _wchmod(filepath, mode); }

		inline errno_t mktemp(char*    templ, int length) { return _mktemp_s(templ, length); }
		inline errno_t mktemp(wchar_t* templ, int length) { return _wmktemp_s(templ, length); }
	}
	#pragma endregion
#endif

	// Return true if 'ch' is a directory marker
	[[deprecated]] constexpr bool DirMark(int ch)
	{
		return ch == '\\' || ch == '/';
	}

	// Return true if two characters are the same as far as a path is concerned
	[[deprecated]] inline bool EqualPathChar(int lhs, int rhs)
	{
		return tolower(lhs) == tolower(rhs) || (DirMark(lhs) && DirMark(rhs));
	}

	// Return true if 'path' is an absolute path. (i.e. contains a drive or is a UNC path)
	[[deprecated]] inline bool IsFullPath(std::filesystem::path const& path)
	{
		if (path.is_absolute())
			return true;
		
		// legacy...
		auto path_str = path.native();
		return path_str.size() >= 2 && (
			(str::IsAlpha(path_str[0]) && path_str[1] == ':') || // Rooted path
			(path_str[0] == '\\' && path_str[1] == '\\')); // UNC path
	}

	// Add quotes to the str if it doesn't already have them
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str AddQuotes(Str str)
	{
		if (str.size() > 1 && str[0] == '"' && str[str.size()-1] == '"') return str;
		str.insert(str.begin(), '"');
		str.insert(str.end  (), '"');
		return str;
	}

	// Remove quotes from 'str' if it has them
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str RemoveQuotes(Str str)
	{
		if (str.size() < 2 || str[0] != '"' || str[str.size()-1] != '"') return str;
		str = str.substr(1, str.size()-2);
		return str;
	}

	// Remove the leading back slash from 'str' if it exists
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str RemoveLeadingBackSlash(Str str)
	{
		if (!str.empty() && (str[0] == '\\' || str[0] == '/')) str.erase(0,1);
		return str;
	}

	// Remove the last back slash from 'str' if it exists
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str RemoveLastBackSlash(Str str)
	{
		if (!str.empty() && (str[str.size()-1] == '\\' || str[str.size()-1] == '/')) str.erase(str.size()-1, 1);
		return str;
	}

	// Convert 'C:\path0\.\path1\../path2\file.ext' into 'C:\path0\path2\file.ext'
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str Canonicalise(Str str)
	{
		Char const dir_marks[] = {Char('\\'),Char('/'),0};

		size_t d0 = 0, d1, d2, d3;
		for (;;)
		{
			d1 = str.find_first_of(dir_marks, d0);
			if (d1 == Str::npos)                                    { return str; }
			if (str[d1] == Char('/'))                                  { str[d1] = Char('\\'); }
			if (d1 == 1 && str[0] == Char('.'))                        { str.erase(0, 2); d0 = 0; continue; }

			d2 = str.find_first_of(dir_marks, d1+1);
			if (d2 == Str::npos)                                    { return str; }
			if (str[d2] == Char('/'))                                  { str[d2] = Char('\\'); }
			if (d2-d1 == 2 && str[d1+1] == Char('.'))                  { str.erase(d1+1, 2); d0 = 0; continue; }

			d3 = str.find_first_of(dir_marks, d2+1);
			if (d3 == Str::npos)                                    { return str; }
			if (str[d3] == Char('/'))                                  { str[d3] = Char('\\'); }
			if (d3-d2 == 2 && str[d2+1] == Char('.'))                  { str.erase(d2+1, 2); d0 = 0; continue; }
			if (d3-d2 == 3 && str[d2+1] == Char('.') && str[d2+2] == Char('.') &&
				(d2-d1 != 3 || str[d1+1] != Char('.') || str[d1+2] != Char('.'))) { str.erase(d1+1, d3-d1); d0 = 0; continue; }

			d0 = d1 + 1;
		}
	}

	// Convert a path name into a standard format of "c:\dir\dir\filename.ext" i.e. back slashes, and lower case
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str Standardise(Str str)
	{
		str = RemoveQuotes(str);
		str = RemoveLastBackSlash(str);
		str = Canonicalise(str);
		for (auto& i : str) i = static_cast<Char>(tolower(i));
		std::replace(str.begin(), str.end(), Char('/'), Char('\\'));
		return str;
	}

	// Get the drive letter from a full path description.
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str GetDrive(Str const& str)
	{
		Char const colon[] = {Char(':'),0};

		auto pos = str.find(colon);
		if (pos == Str::npos) return Str();
		return str.substr(0, pos);
	}

	// Get the path from a full path description
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str GetPath(Str const& str)
	{
		Char const colon[] = {Char(':'),0}, dir_marks[] = {Char('\\'),Char('/'),0};
		size_t p, first = 0, last = str.size();

		// Find the start of the path
		p = str.find(colon);
		if (p != Str::npos) first = p + 1;
		if (first != last && (str[first] == Char('\\') || str[first] == Char('/'))) ++first;

		// Find the end of the path
		p = str.find_last_of(dir_marks);
		if (p == Str::npos || p <= first) return Str();
		last = p;

		return str.substr(first, last - first);
	}

	// Get the directory including drive letter from a full path description
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str GetDirectory(Str const& str)
	{
		Char const dir_marks[] = {Char('\\'),Char('/'),0};

		// Find the end of the path
		auto p = str.find_last_of(dir_marks);
		if (p == Str::npos) return Str();
		return str.substr(0, p);
	}

	// Get the extension from a full path description (does not include the '.'). Note: std::filesystem::path.extension() *DOES* include the dot
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str GetExtension(Str const& str)
	{
		Char const dot[] = {Char('.'),0};

		auto p = str.find_last_of(dot);
		if (p == Str::npos) return Str();
		return str.substr(p+1);
	}

	// Returns a pointer to the extension part of a filepath (does not include the '.')
	template <typename Char, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Char const* GetExtensionInPlace(Char const* str)
	{
		Char const* extn = 0, *p;
		for (p = str; *p; ++p)
			if (*p == '.') extn = p + 1;

		return extn ? extn : p;
	}

	// Get the filename including extension from a full path description
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str GetFilename(Str const& str)
	{
		Char const dir_marks[] = {Char('\\'),Char('/'),0};

		// Find the end of the path
		auto p = str.find_last_of(dir_marks);
		if (p == Str::npos) return str;
		return str.substr(p+1);
	}

	// Get the file title from a full path description
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str GetFiletitle(Str const& str)
	{
		Char const dot[] = {Char('.'),0}, dir_marks[] = {Char('\\'),Char('/'),0};

		size_t p, first = 0, last = str.size();

		// Find the start of the file title
		p = str.find_last_of(dir_marks);
		if (p != Str::npos) first = p + 1;

		// Find the end of the file title
		p = str.find_last_of(dot);
		if (p != Str::npos && p > first) last = p;

		return str.substr(first, last - first);
	}

	// Remove the drive from 'str'
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str& RmvDrive(Str& str)
	{
		Char const colon[] = {Char(':'),0}, dir_marks[] = {Char('\\'),Char('/'),0};

		auto p = str.find(colon);
		if (p == Str::npos) return str;
		p = str.find_first_not_of(dir_marks, p+1);
		return str.erase(0, p);
	}

	// Remove the path from 'str'
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str& RmvPath(Str& str)
	{
		Char const colon[] = {Char(':'),0}, dir_marks[] = {Char('\\'),Char('/'),0};

		size_t p, first = 0, last = str.size();

		// Find the start of the path
		p = str.find(colon);
		if (p != Str::npos) first = p + 1;
		if (first != last && (str[first] == Char('\\') || str[first] == Char('/'))) ++first;

		// Find the end of the path
		p = str.find_last_of(dir_marks);
		if (p == Str::npos || p <= first) return str;
		last = p + 1;
		str.erase(first, last - first);
		return str;
	}

	// Remove the directory from 'str'
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str& RmvDirectory(Str& str)
	{
		Char const dir_marks[] = {Char('\\'),Char('/'),0};

		// Find the end of the path
		size_t p = str.find_last_of(dir_marks);
		if (p == Str::npos) return str;
		str.erase(0, p + 1);
		return str;
	}

	// Remove the extension from 'str'
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str& RmvExtension(Str& str)
	{
		Char const dot[] = {Char('.'),0};

		size_t p = str.find_last_of(dot);
		if (p == Str::npos) return str;
		str.erase(p);
		return str;
	}

	// Remove the filename from 'str'
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str& RmvFilename(Str& str)
	{
		Char const dir_marks[] = {Char('\\'),Char('/'),0};

		// Find the end of the path
		size_t p = str.find_last_of(dir_marks);
		if (p == Str::npos) return str;
		str.erase(p);
		return str;
	}

	// Remove the file title from 'str'
	template <typename Str, typename Char = Str::value_type, typename = std::enable_if_t<is_char_v<Char>>>
	[[deprecated]] inline Str& RmvFiletitle(Str& str)
	{
		Char const dot[] = {Char('.'),0}, dir_marks[] = {Char('\\'),Char('/'),0};

		size_t p, first = 0, last = str.size();

		// Find the start of the file title
		p = str.find_last_of(dir_marks);
		if (p != Str::npos) first = p + 1;

		// Find the end of the file title
		p = str.find_last_of(dot);
		if (p != Str::npos && p > first) last = p;

		str.erase(first, last - first);
		return str;
	}

	// Delete a file
	template <typename Str>
	[[deprecated]] inline bool EraseFile(Str const& filepath)
	{
		return impl::remove(c_str(filepath)) == 0;
	}

	// Delete an empty directory
	template <typename Str>
	[[deprecated]] inline bool EraseDir(Str const& path)
	{
		return impl::rmdir(c_str(path)) == 0;
	}

	// Delete a file or empty directory
	template <typename Str>
	[[deprecated]] inline bool Erase(Str const& path)
	{
		return EraseFile(path) || EraseDir(path);
	}

	// Rename a file
	template <typename Str1, typename Str2>
	[[deprecated]] inline bool RenameFile(Str1 const& old_filepath, Str2 const& new_filepath)
	{
		return impl::rename(c_str(old_filepath), c_str(new_filepath)) == 0;
	}

	// Copy a file
	template <typename Str1, typename Str2>
	[[deprecated]] inline bool CpyFile(Str1 const& src_filepath, Str2 const& dst_filepath)
	{
		std::ifstream src (c_str(src_filepath), std::ios::binary);
		std::ofstream dest(c_str(dst_filepath), std::ios::binary);
		if (!src.is_open() || !dest.is_open()) return false;
		dest << src.rdbuf();
		return true;
	}

	// Move 'src' to 'dst' replacing 'dst' if it already exists
	template <typename Str>
	[[deprecated]] inline bool RepFile(Str const& src, Str const& dst)
	{
		if (FileExists(dst)) EraseFile(dst);
		return RenameFile(src, dst);
	}

	// Return the length of a file, without opening it
	template <typename Str>
	[[deprecated]] inline int64_t FileLength(Str const& filepath)
	{
		struct __stat64 info;
		if (impl::stat64(c_str(filepath), &info) != 0) return 0;
		return info.st_size;
	}

	// Return the amount of free disk space. 'drive' = 'A', 'B', 'C', etc
	[[deprecated]] inline uint64_t GetDiskFree(char drive)
	{
		_diskfree_t drive_info;
		drive = char(toupper(drive));
		if (_getdiskfree(drive - 'A' + 1, &drive_info) != 0) return 0;
		uint64_t size = 1;
		size *= drive_info.bytes_per_sector;
		size *= drive_info.sectors_per_cluster;
		size *= drive_info.avail_clusters;
		return size;
	}

	// Return the size of a disk. 'drive' = 'A', 'B', 'C', etc
	[[deprecated]] inline uint64_t GetDiskSize(char drive)
	{
		_diskfree_t drive_info;
		drive = char(toupper(drive));
		if (_getdiskfree(drive - 'A' + 1, &drive_info) != 0) return 0;
		uint64_t size = 1;
		size *= drive_info.bytes_per_sector;
		size *= drive_info.sectors_per_cluster;
		size *= drive_info.total_clusters;
		return size;
	}

	// Return a bitwise combination of Attributes for 'str'
	[[deprecated]] inline EAttrib GetAttribs(std::filesystem::path const& str)
	{
		struct __stat64 info;
		if (_wstat64(str.c_str(), &info) != 0)
			return EAttrib::None;

		// Interpret the stats
		auto attribs = EAttrib::None;
		if ((info.st_mode & _S_IFREG ) != 0 && (info.st_mode & _S_IFCHR) != 0 ) attribs |= EAttrib::Device;
		if ((info.st_mode & _S_IFREG ) != 0 && (info.st_mode & _S_IFCHR) == 0 ) attribs |= EAttrib::File;
		if ( info.st_mode & _S_IFDIR ) attribs |= EAttrib::Directory;
		if ( info.st_mode & _S_IFIFO ) attribs |= EAttrib::Pipe;
		if ( info.st_mode & _S_IREAD ) attribs |= EAttrib::ReadAccess;
		if ( info.st_mode & _S_IWRITE) attribs |= EAttrib::WriteAccess;
		if ( info.st_mode & _S_IEXEC ) attribs |= EAttrib::ExecAccess;
		return attribs;
	}

	// Return the creation, last modified, and last access time of a file
	// Note: these timestamps are in UTC Unix time
	[[deprecated]] inline FileTime FileTimeStats(std::filesystem::path const& str)
	{
		FileTime file_time = {0, 0, 0};

		struct __stat64 info;
		if (_wstat64(str.c_str(), &info) != 0)
			return file_time;

		file_time.m_created       = info.st_ctime;
		file_time.m_last_modified = info.st_mtime;
		file_time.m_last_access   = info.st_atime;
		return file_time;
	}

	// Return true if 'filepath' is a file that exists
	[[deprecated]] inline bool FileExists(std::filesystem::path const& filepath)
	{
		return std::filesystem::exists(filepath);
	}

	// Return true if 'directory' exists
	[[deprecated]] inline bool DirectoryExists(std::filesystem::path const& directory)
	{
		return std::filesystem::exists(directory);
	}

	// Recursively create 'directory'
	template <typename Str, typename Char = Str::value_type>
	[[deprecated]] inline bool CreateDir(Str const& directory)
	{
		Char const dir_marks[] = {Char('\\'),Char('/'),0};

		auto dir = Canonicalise(directory);
		for (size_t n = dir.find_first_of(dir_marks); n != Str::npos; n = dir.find_first_of(dir_marks, n+1))
		{
			if (n < 1 || dir[n-1] == Char(':')) continue;
			if (impl::mkdir(dir.substr(0, n).c_str()) < 0 && errno != EEXIST)
				return false;
		}
		return impl::mkdir(c_str(dir)) == 0 || errno == EEXIST;
	}

#if 0
	// Check the access on a file
	template <typename Str>
	inline Access GetAccess(Str const& str)
	{
		int acc = 0;
		if (impl::access(c_str(str), Read ) == 0) acc |= Read;
		if (impl::access(c_str(str), Write) == 0) acc |= Write;
		return static_cast<Access>(acc);
	}

	// Set the attributes on a file
	template <typename Str> inline bool SetAccess(Str const& str, Access state)
	{
		int mode = 0;
		if (state & Read ) mode |= _S_IREAD;
		if (state & Write) mode |= _S_IWRITE;
		return impl::chmod(c_str(str), mode) == 0;
	}
#endif

	// Make a unique filename. Template should have the form: "FilenameXXXXXX". X's are replaced. Note, no extension
	template <typename Str>
	[[deprecated]] inline Str MakeUniqueFilename(Str const& tmplate)
	{
		auto str = tmplate;
		return impl::mktemp(&str[0], impl::strlen(&str[0]) + 1) == 0 ? str : Str();
	}

	// Return the current directory
	[[deprecated]] inline std::filesystem::path CurrentDirectory()
	{
		return std::filesystem::current_path();
	}

	// Replaces the extension of 'path' with 'new_extn'
	template <typename Str, typename Char = Str::value_type>
	[[deprecated]] inline Str ChangeExtn(Str const& path, decltype(path) new_extn)
	{
		auto s = path;
		RmvExtension(s);
		s += Char('.');
		s += new_extn;
		return s;
	}

	// Insert 'prefix' before, and 'postfix' after the file title in 'path', without modifying the extension
	template <typename Str, typename Char = Str::value_type>
	[[deprecated]] inline Str ChangeFilename(Str const& path, decltype(path) prefix, decltype(path) postfix)
	{
		auto s = path;
		RmvFilename(s);
		s += Char('\\');
		s += prefix;
		s += GetFiletitle(path);
		s += postfix;
		s += Char('.');
		s += GetExtension(path);
		return s;
	}

	// Combine two path fragments into a combined path
	template <typename Str>
	[[deprecated]] inline Str CombinePath(Str const& lhs, decltype(lhs) rhs)
	{
		if (IsFullPath(rhs)) return rhs;
		return Canonicalise(RemoveLastBackSlash(lhs).append(1,'\\').append(RemoveLeadingBackSlash(rhs)));
	}
	template <typename Str, typename... Parts>
	[[deprecated]] inline Str CombinePath(Str const& lhs, decltype(lhs) rhs, Parts&&... rest)
	{
		return CombinePath(CombinePath(lhs, rhs), rest...);
	}

	// Convert a relative path into a full path
	template <typename Str, typename Char = Str::value_type>
	[[deprecated]] inline Str GetFullPath(Str const& str)
	{
		Char buf[_MAX_PATH];
		Str path(const_cast<Char const*>(impl::fullpath(buf, c_str(str), _MAX_PATH)));
		return Standardise<Str>(path);
	}

	// Make 'full_path' relative to 'relative_to'.  e.g.  C:/path1/path2/file relative to C:/path1/path3/ = ../path2/file
	template <typename Str, typename Char = Str::value_type>
	[[deprecated]] inline Str GetRelativePath(Str const& full_path, Str const& relative_to)
	{
		Char const prev_dir[]  = {'.','.','/',0};
		Char const dir_marks[] = {'\\','/',0};

		// Find where the paths differ, recording the last common directory marker
		int i, d = -1;
		auto fpath = c_str(full_path);
		auto rpath = c_str(relative_to);
		for (i = 0; EqualPathChar(fpath[i], rpath[i]); ++i)
		{
			if (DirMark(full_path[i]))
				d = i;
		}

		// If the paths match for all of 'relative_to' just return the remainder of 'full_path'
		if (DirMark(fpath[i]) && rpath[i] == 0)
			return full_path.substr(i + 1);

		// If 'd==-1' then none of the paths matched.
		// If either path contains a drive then return 'full_path'
		if (d == -1 && (*find(fpath, Char(':')) != 0 || *find(rpath, Char(':')) != 0))
			return full_path;

		// Otherwise, the part of the path up to and including 'd' matches, so it's not part of the relative path
		Str path(fpath + d + 1);
		for (Char const *end = rpath + d + 1; *end; end += *end != 0)
		{
			auto ptr = end;
			end = find_first(ptr, dir_marks);
			path.insert(0, prev_dir);
		}
		return path;
	}
}


#if PR_UNITTESTS
#include <fstream>
#include "pr/common/unittests.h"
#include "pr/common/flags_enum.h"
namespace pr::filesys
{
	PRUnitTest(OldFilesysTests, Quick)
	{
		using namespace pr::filesys;

		{//Quotes
			std::string no_quotes = "path\\path\\file.extn";
			std::string has_quotes = "\"path\\path\\file.extn\"";
			std::string p = no_quotes;
			p = RemoveQuotes(p); PR_CHECK(no_quotes , p);
			p = AddQuotes(p);    PR_CHECK(has_quotes, p);
			p = AddQuotes(p);    PR_CHECK(has_quotes, p);
		}
		{//Slashes
			std::string has_slashes1 = "\\path\\path\\";
			std::string has_slashes2 = "/path/path/";
			std::string no_slashes1 = "path\\path";
			std::string no_slashes2 = "path/path";

			has_slashes1 = RemoveLeadingBackSlash(has_slashes1);
			has_slashes1 = RemoveLastBackSlash(has_slashes1);
			PR_CHECK(no_slashes1, has_slashes1);

			has_slashes2 = RemoveLeadingBackSlash(has_slashes2);
			has_slashes2 = RemoveLastBackSlash(has_slashes2);
			PR_CHECK(no_slashes2, has_slashes2);
		}
		{//Canonicalise
			std::string p0 = "C:\\path/.././path\\path\\path\\../../../file.ext";
			std::string P0 = "C:\\file.ext";
			p0 = Canonicalise(p0);
			PR_CHECK(P0, p0);

			std::string p1 = ".././path\\path\\path\\../../../file.ext";
			std::string P1 = "..\\file.ext";
			p1 = Canonicalise(p1);
			PR_CHECK(P1, p1);
		}
		{//Standardise
			std::string p0 = "c:\\path/.././Path\\PATH\\path\\../../../PaTH\\File.EXT";
			std::string P0 = "c:\\path\\file.ext";
			p0 = Standardise(p0);
			PR_CHECK(P0, p0);
		}
		{//GetDrive
			std::string p0 = GetDrive<std::string>("drive:/path");
			std::string P0 = "drive";
			PR_CHECK(P0, p0);
		}
		{//GetPath
			std::string p0 = GetPath<std::string>("drive:/path0/path1/file.ext");
			std::string P0 = "path0/path1";
			PR_CHECK(P0, p0);
		}
		{//GetDirectory
			std::string p0 = GetDirectory<std::string>("drive:/path0/path1/file.ext");
			std::string P0 = "drive:/path0/path1";
			PR_CHECK(P0, p0);
		}
		{//GetExtension
			std::string p0 = GetExtension<std::string>("drive:/pa.th0/path1/file.stuff.extn");
			std::string P0 = "extn";
			PR_CHECK(P0, p0);
		}
		{//GetFilename
			std::string p0 = GetFilename<std::string>("drive:/pa.th0/path1/file.stuff.extn");
			std::string P0 = "file.stuff.extn";
			PR_CHECK(P0, p0);
		}
		{//GetFiletitle
			std::string p0 = GetFiletitle<std::string>("drive:/pa.th0/path1/file.stuff.extn");
			std::string P0 = "file.stuff";
			PR_CHECK(P0, p0);
		}
		{//RmvDrive
			std::string p0 = "drive:/pa.th0/path1/file.stuff.extn";
			std::string P0 = "pa.th0/path1/file.stuff.extn";
			p0 = RmvDrive(p0);
			PR_CHECK(P0, p0);
		}
		{//RmvPath
			std::string p0 = "drive:/pa.th0/path1/file.stuff.extn";
			std::string P0 = "drive:/file.stuff.extn";
			p0 = RmvPath(p0);
			PR_CHECK(P0, p0);
		}
		{//RmvDirectory
			std::string p0 = "drive:/pa.th0/path1/file.stuff.extn";
			std::string P0 = "file.stuff.extn";
			p0 = RmvDirectory(p0);
			PR_CHECK(P0, p0);
		}
		{//RmvExtension
			std::string p0 = "drive:/pa.th0/path1/file.stuff.extn";
			std::string P0 = "drive:/pa.th0/path1/file.stuff";
			p0 = RmvExtension(p0);
			PR_CHECK(P0, p0);
		}
		{//RmvFilename
			std::string p0 = "drive:/pa.th0/path1/file.stuff.extn";
			std::string P0 = "drive:/pa.th0/path1";
			p0 = RmvFilename(p0);
			PR_CHECK(P0, p0);
		}
		{//RmvFiletitle
			std::string p0 = "drive:/pa.th0/path1/file.stuff.extn";
			std::string P0 = "drive:/pa.th0/path1/.extn";
			p0 = RmvFiletitle(p0);
			PR_CHECK(P0, p0);
		}
		{//Files
			auto dir = CurrentDirectory();
			PR_EXPECT(DirectoryExists(dir));

			auto fn = MakeUniqueFilename<std::string>("test_fileXXXXXX");
			PR_EXPECT(!FileExists(fn));

			auto path = dir / fn;

			std::ofstream f(path);
			f << "Hello World";
			f.close();

			PR_EXPECT(FileExists(path));
			std::string fn2 = MakeUniqueFilename<std::string>("test_fileXXXXXX");
			std::string path2 = GetFullPath(fn2);

			RenameFile(path, path2);
			PR_EXPECT(FileExists(path2));
			PR_EXPECT(!FileExists(path));

			CpyFile(path2, path);
			PR_EXPECT(FileExists(path2));
			PR_EXPECT(FileExists(path));

			EraseFile(path2);
			PR_EXPECT(!FileExists(path2));
			PR_EXPECT(FileExists(path));

			int64_t size = FileLength(path);
			PR_CHECK(size, 11);

			auto attr = GetAttribs(path);
			auto flags = EAttrib::File|EAttrib::WriteAccess|EAttrib::ReadAccess;
			PR_EXPECT(attr == flags);

			attr = GetAttribs(dir);
			flags = EAttrib::Directory|EAttrib::WriteAccess|EAttrib::ReadAccess|EAttrib::ExecAccess;
			PR_EXPECT(attr == flags);

			std::string drive = GetDrive(path);
			uint64_t disk_free = GetDiskFree(drive[0]);
			uint64_t disk_size = GetDiskSize(drive[0]);
			PR_EXPECT(disk_size > disk_free);

			EraseFile(path);
			PR_EXPECT(!FileExists(path));
		}
		{//DirectoryOps
			{
				std::string p0 = "C:/path0/../";
				std::string p1 = "./path4/path5";
				std::string P  = "C:\\path4\\path5";
				std::string R  = CombinePath(p0, p1);
				PR_CHECK(P, R);
			}
			{
				std::string p0 = "C:/path0/path1/path2/path3/file.extn";
				std::string p1 = "C:/path0/path4/path5";
				std::string P  = "../../path1/path2/path3/file.extn";
				std::string R  = GetRelativePath(p0, p1);
				PR_CHECK(P, R);
			}
			{
				std::string p0 = "/path1/path2/file.extn";
				std::string p1 = "/path1/path3/path4";
				std::string P  = "../../path2/file.extn";
				std::string R  = GetRelativePath(p0, p1);
				PR_CHECK(P, R);
			}
			{
				std::string p0 = "/path1/file.extn";
				std::string p1 = "/path1";
				std::string P  = "file.extn";
				std::string R  = GetRelativePath(p0, p1);
				PR_CHECK(P, R);
			}
			{
				std::string p0 = "path1/file.extn";
				std::string p1 = "path2";
				std::string P  = "../path1/file.extn";
				std::string R  = GetRelativePath(p0, p1);
				PR_CHECK(P, R);
			}
			{
				std::string p0 = "c:/path1/file.extn";
				std::string p1 = "d:/path2";
				std::string P  = "c:/path1/file.extn";
				std::string R  = GetRelativePath(p0, p1);
				PR_CHECK(P, R);
			}
		}
	}
}
#endif

#pragma warning(default:4996)
#endif
#endif
