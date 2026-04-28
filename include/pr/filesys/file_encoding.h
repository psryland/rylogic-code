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

		// Assume UTF-8 unless we find invalid UTF-8 sequences.
		bom_size = 0;
		auto const scan_size = std::min<size_t>(size, 0x100000);
		for (auto i = size_t{}; i != scan_size; ++i)
		{
			auto c =
				(bytes[i] & 0b10000000) == 0          ? 0 : // ASCII character, 0 continuation bytes
				(bytes[i] & 0b11100000) == 0b11000000 ? 1 : // 2-byte UTF-8 character, 1 continuation byte
				(bytes[i] & 0b11110000) == 0b11100000 ? 2 : // 3-byte UTF-8 character, 2 continuation bytes
				(bytes[i] & 0b11111000) == 0b11110000 ? 3 : // 4-byte UTF-8 character, 3 continuation bytes
				-1;                                      // Not a valid UTF-8 character

			if (c == 0)
				continue;
			if (c < 0)
				return EEncoding::ascii_extended;
			if (i + static_cast<size_t>(c) >= scan_size)
				break;

			for (; c != 0 && (bytes[++i] & 0b11000000) == 0b10000000; --c) {}
			if (c != 0)
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
