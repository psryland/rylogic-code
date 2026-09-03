//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - UTF-8 native byte input sources.
//
// Style Guidance:
//  - This header, and the rest of 'pr/script/reader2/*', implement the redesigned,
//    UTF-8 native, header-only sibling to 'pr::script::Reader'. The original wchar_t
//    based reader (pr/script/reader.h and friends) is left completely unmodified.
//  - Reader2 favours direct byte buffers and pull-based refill over the old virtual
//    'Src' hierarchy. New code in this module should follow that pattern rather than
//    reintroducing per-character virtual dispatch.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <istream>
#include <filesystem>
#include <memory>
#include <vector>

namespace pr::script::v2
{
	// The size of the physical read block used to pull bytes out of an 'IInput'.
	// This is the "fixed transport buffer" referred to by the reader2 design: bulk
	// reads are always requested in chunks of this size (or the remaining tail).
	// Large streamed/file numeric and boundary workloads selected 32 KiB as the
	// best representative geometric mean without a material boundary regression.
	#if !defined(PR_SCRIPT_READER2_BLOCK_SIZE)
	#define PR_SCRIPT_READER2_BLOCK_SIZE 32768
	#endif
	constexpr size_t BlockSize = PR_SCRIPT_READER2_BLOCK_SIZE;

	// Abstract supplier of raw UTF-8 bytes. Implementations are pull-based: 'Read'
	// is called repeatedly by a 'Cursor' to top up its working window, in blocks of
	// up to 'BlockSize' bytes, rather than the caller pushing data in one shot.
	struct IInput
	{
		virtual ~IInput() = default;

		// Copy up to 'count' bytes into 'buf'. Returns the number of bytes copied,
		// or 0 to indicate the input is exhausted. May return fewer than 'count'
		// bytes even when more data remains (implementations are free to under-fill).
		virtual size_t Read(char* buf, size_t count) = 0;

		// The path associated with this input, used for 'Loc::Filepath()'. Empty for
		// anonymous in-memory sources.
		virtual std::filesystem::path const& Filepath() const = 0;

		// The source offset represented by the first returned byte, allowing caller-side encoding conversion to preserve a stripped byte-order-mark.
		virtual std::streamoff InitialOffset() const
		{
			return 0;
		}
	};

	// An 'IInput' over a caller-owned, contiguous block of UTF-8 memory. The memory
	// must outlive the 'MemoryInput' instance; no copy of the data is made.
	struct MemoryInput :IInput
	{
		std::string_view m_data;
		size_t m_pos;
		std::filesystem::path m_filepath;

		explicit MemoryInput(std::string_view data, std::filesystem::path filepath = {})
			: m_data(data)
			, m_pos(0)
			, m_filepath(std::move(filepath))
		{}

		// Copy the next available bytes out of the in-memory buffer.
		size_t Read(char* buf, size_t count) override
		{
			auto avail = m_data.size() - m_pos;
			auto n = avail < count ? avail : count;
			if (n != 0)
			{
				std::memcpy(buf, m_data.data() + m_pos, n);
				m_pos += n;
			}
			return n;
		}
		std::filesystem::path const& Filepath() const override
		{
			return m_filepath;
		}
	};

	// An 'IInput' that pulls bulk blocks from a 'std::istream'. The stream must
	// outlive the 'StreamInput' instance and must be opened in binary mode so that
	// no platform newline translation corrupts the byte-for-byte UTF-8 content.
	struct StreamInput :IInput
	{
		std::istream* m_stream;
		std::filesystem::path m_filepath;

		explicit StreamInput(std::istream& stream, std::filesystem::path filepath = {})
			: m_stream(&stream)
			, m_filepath(std::move(filepath))
		{}

		// Pull the next block of bytes from the stream.
		size_t Read(char* buf, size_t count) override
		{
			m_stream->read(buf, static_cast<std::streamsize>(count));
			auto n = static_cast<size_t>(m_stream->gcount());

			// EOF commonly sets failbit after returning a partial final block, but
			// badbit represents a hard transport failure and must not look like clean EOF.
			if (m_stream->bad())
				throw std::ios_base::failure("failed to read script input stream");

			// A stream that has hit EOF still reports the bytes it managed to read on
			// this call, so only treat 'n == 0' as exhaustion; clear a benign EOF flag
			// so a subsequent (empty) read behaves consistently instead of throwing.
			if (n == 0 && m_stream->eof())
				m_stream->clear(m_stream->rdstate() & ~std::ios::failbit);

			return n;
		}
		std::filesystem::path const& Filepath() const override
		{
			return m_filepath;
		}
	};

	// Detect and return the length (in bytes) of a UTF-8 byte-order-mark at the very
	// start of 'data', or 0 if none is present. Callers should skip this many bytes
	// before treating the remainder as script content.
	inline size_t Utf8BomLength(std::string_view data)
	{
		constexpr unsigned char bom[3] = { 0xEF, 0xBB, 0xBF };
		if (data.size() >= 3 &&
			static_cast<unsigned char>(data[0]) == bom[0] &&
			static_cast<unsigned char>(data[1]) == bom[1] &&
			static_cast<unsigned char>(data[2]) == bom[2])
			return 3;

		return 0;
	}
}
