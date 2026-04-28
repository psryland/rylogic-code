//**********************************************
// File path/File system operations
//  Copyright (c) Rylogic Ltd 2025
//**********************************************
#pragma once
#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include "pr/common/memstream.h"
#include "pr/win32/win32.h"

namespace pr::filesys
{
	struct FileSnapshot
	{
		struct Options
		{
			int m_max_attempts = 10;
			int m_max_block_time_ms = 5000;
			DWORD m_share_mode = FILE_SHARE_READ | FILE_SHARE_DELETE;
		};

		std::filesystem::path m_filepath;
		std::string m_data;

		FileSnapshot(std::filesystem::path const& filepath, Options opts = {})
			:m_filepath(filepath)
			,m_data()
		{
			auto file = Open(filepath, opts);
			m_data = ReadAll(filepath, file);
		}

		std::span<char const> bytes() const
		{
			return std::span<char const>(m_data.data(), m_data.size());
		}
		std::string_view str() const
		{
			return std::string_view(m_data.data(), m_data.size());
		}

	private:

		static bool IsTransient(DWORD err)
		{
			return err == ERROR_SHARING_VIOLATION || err == ERROR_LOCK_VIOLATION;
		}
		static std::runtime_error Error(std::filesystem::path const& filepath, std::string_view action, DWORD err)
		{
			auto const message = std::system_category().message(static_cast<int>(err));
			return std::runtime_error(std::format("{} '{}': {} ({})", action, filepath.string(), message, err));
		}
		static win32::Handle Open(std::filesystem::path const& filepath, Options const& opts)
		{
			auto const max_attempts = std::max(1, opts.m_max_attempts);
			auto const max_attempts_d = static_cast<double>(max_attempts);
			auto const back_off = 2.0 * opts.m_max_block_time_ms / (max_attempts_d * (1.0 + max_attempts_d));
			auto last_error = DWORD{};

			for (auto attempt = 0; attempt != max_attempts; ++attempt)
			{
				auto handle = win32::FileOpen(
					filepath,
					GENERIC_READ,
					opts.m_share_mode,
					OPEN_EXISTING,
					FILE_ATTRIBUTE_NORMAL,
					FILE_FLAG_SEQUENTIAL_SCAN);

				if (handle)
					return handle;

				last_error = ::GetLastError();
				if (!IsTransient(last_error) || attempt + 1 == max_attempts)
					break;

				::Sleep(static_cast<DWORD>((attempt + 1) * back_off));
			}

			if (IsTransient(last_error))
				throw Error(filepath, "File is busy", last_error);

			throw Error(filepath, "Failed to open file", last_error);
		}
		static std::string ReadAll(std::filesystem::path const& filepath, HANDLE file)
		{
			LARGE_INTEGER file_size = {};
			if (!::GetFileSizeEx(file, &file_size))
				throw Error(filepath, "Failed to query file size", ::GetLastError());
			if (file_size.QuadPart < 0)
				throw std::runtime_error(std::format("Invalid file size for '{}'", filepath.string()));

			auto data = std::string{};
			if (static_cast<unsigned long long>(file_size.QuadPart) > static_cast<unsigned long long>(data.max_size()))
				throw std::runtime_error(std::format("File '{}' is too large to snapshot", filepath.string()));

			data.resize(static_cast<size_t>(file_size.QuadPart));
			for (auto ofs = size_t{}; ofs != data.size(); )
			{
				auto const remaining = data.size() - ofs;
				auto const to_read = static_cast<DWORD>(std::min<size_t>(remaining, std::numeric_limits<DWORD>::max()));
				auto bytes_read = DWORD{};
				if (!::ReadFile(file, data.data() + ofs, to_read, &bytes_read, nullptr))
					throw Error(filepath, "Failed to read file", ::GetLastError());
				if (bytes_read == 0)
					throw std::runtime_error(std::format("Failed to read file '{}': unexpected end of file", filepath.string()));

				ofs += bytes_read;
			}
			return data;
		}
	};

	struct FileSnapshotStream
		: private std::string
		, public mem_istream<char>
	{
		using storage_t = std::string;

		FileSnapshotStream(std::string data)
			:storage_t(std::move(data))
			,mem_istream<char>(std::string_view(static_cast<storage_t const&>(*this)), 0)
		{}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::filesys
{
	PRUnitTest(FileSnapshotTests)
	{
		auto const filepath = temp_dir() / std::format("pr_file_snapshot_{}_{}.tmp", ::GetCurrentProcessId(), ::GetTickCount64());

		{
			std::ofstream file(filepath, std::ios::binary);
			file << "snapshot data";
		}

		auto snapshot = FileSnapshot(filepath);
		PR_EXPECT(snapshot.str() == "snapshot data");

		auto busy = win32::FileOpen(filepath, GENERIC_WRITE, FILE_SHARE_READ, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL);
		PR_EXPECT(!!busy);
		PR_THROWS(FileSnapshot(filepath, FileSnapshot::Options{ .m_max_attempts = 1, .m_max_block_time_ms = 0 }), std::exception);
	}
}
#endif
