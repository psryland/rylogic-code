//**********************************************
// File path/File system operations
//  Copyright (c) Rylogic Ltd 2009
//**********************************************
#pragma once
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <span>
#include <string>
#include "pr/str/encoding.h"
#include "pr/str/convert_utf.h"

namespace pr::filesys
{
	// Examines file data to guess at the encoding (assumes the data is text).
	// On return 'bom_size' is the length of the byte order mask.
	// Returns 'UTF-8' if unknown, since UTF-8 recommends not using BOMs.
	inline EEncoding DetectFileEncoding(std::span<char const> data, int& bom_size)
	{
		auto bytes = reinterpret_cast<unsigned char const*>(data.data());
		auto const size = data.size();
		if (size >= 3 && bytes[0] == 0xEF && bytes[1] == 0xBB && bytes[2] == 0xBF)
		{
			bom_size = 3;
			return EEncoding::utf8;
		}
		if (size >= 2 && bytes[0] == 0xFE && bytes[1] == 0xFF)
		{
			bom_size = 2;
			return EEncoding::utf16_be;
		}
		if (size >= 2 && bytes[0] == 0xFF && bytes[1] == 0xFE)
		{
			bom_size = 2;
			return EEncoding::utf16_le;
		}

		// Assume UTF-8 unless the data contains malformed UTF-8. A sequence cut off by the end of the scanned data is not an error.
		using converter_t = str::convert_utf<char, char32_t>;
		bom_size = 0;
		converter_t cvt;
		auto ignore = [](char32_t const*, char32_t const*) {};
		auto const scan_size = std::min<size_t>(size, 0x100000);
		for (auto i = size_t{}; i != scan_size; ++i)
		{
			if (cvt(data[i], ignore) == converter_t::error)
				return EEncoding::ascii_extended;
		}
		return EEncoding::utf8;
	}
	inline EEncoding DetectFileEncoding(std::span<char const> data)
	{
		int bom_size;
		return DetectFileEncoding(data, bom_size);
	}

	// Examines 'filepath' to guess at the file data encoding (assumes 'filepath' is a text file)
	// On return 'bom_size' is the length of the byte order mask.
	// Returns 'UTF-8' if unknown, since UTF-8 recommends not using BOMs
	inline EEncoding DetectFileEncoding(std::filesystem::path const& filepath, int& bom_size)
	{
		std::ifstream file(filepath, std::ios::binary);
		std::string data(size_t{ 0x100000 }, '\0');
		if (file.good())
		{
			file.read(data.data(), static_cast<std::streamsize>(data.size()));
			data.resize(static_cast<size_t>(file.gcount()));
		}
		else
		{
			data.resize(0);
		}

		return DetectFileEncoding(std::span<char const>(data.data(), data.size()), bom_size);
	}
	inline EEncoding DetectFileEncoding(std::filesystem::path const& filepath)
	{
		int bom_size;
		return DetectFileEncoding(filepath, bom_size);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::filesys
{
	PRUnitTest(DetectFileEncodingTests, Quick)
	{
		using namespace std::string_view_literals;
		auto Detect = [](std::string_view s)
		{
			return DetectFileEncoding(std::span<char const>(s.data(), s.size()));
		};

		// Well-formed UTF-8, including a sequence cut off by the end of the data
		PR_EXPECT(Detect("abc \xE6\xB0\xB4 \xF0\x9F\x8D\x8C"sv) == EEncoding::utf8);
		PR_EXPECT(Detect("abc \xE6\xB0"sv) == EEncoding::utf8);

		// Malformed UTF-8: overlong forms, encoded surrogates, values above U+10FFFF, and stray continuation bytes
		PR_EXPECT(Detect("a\xC0\x80"sv) == EEncoding::ascii_extended);
		PR_EXPECT(Detect("a\xE0\x80\x80"sv) == EEncoding::ascii_extended);
		PR_EXPECT(Detect("a\xED\xA0\x80"sv) == EEncoding::ascii_extended);
		PR_EXPECT(Detect("a\xF4\x90\x80\x80"sv) == EEncoding::ascii_extended);
		PR_EXPECT(Detect("a\x80"sv) == EEncoding::ascii_extended);
		PR_EXPECT(Detect("caf\xE9!"sv) == EEncoding::ascii_extended);
	}
}
#endif
