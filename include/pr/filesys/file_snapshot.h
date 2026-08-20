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
#include <utility>
#include "pr/common/memstream.h"
#include "pr/win32/win32.h"

namespace pr::filesys
{
	struct FileSnapshot
	{
		struct Options
		{
			int m_max_attempts = 3;
			int m_max_block_time_ms = 100;
			DWORD m_share_mode = FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE;
		};

		std::filesystem::path m_filepath;
		std::string m_data;
		bool m_stable;

		FileSnapshot(std::filesystem::path const& filepath, Options opts = {})
			:m_filepath(filepath)
			,m_data()
			,m_stable()
		{
			auto const max_attempts = std::max(1, opts.m_max_attempts);
			auto const max_attempts_d = static_cast<double>(max_attempts);
			auto const back_off = 2.0 * opts.m_max_block_time_ms / (max_attempts_d * (1.0 + max_attempts_d));
			auto last_error = DWORD{};

			for (auto attempt = 0; attempt != max_attempts; ++attempt)
			{
				auto file = Open(filepath, opts.m_share_mode, last_error);
				if (!file)
				{
					if (!IsTransient(last_error) || attempt + 1 == max_attempts)
						break;

					::Sleep(static_cast<DWORD>((attempt + 1) * back_off));
					continue;
				}

				auto const before = FileState::Read(filepath, file);
				auto complete = false;
				m_data = ReadAll(filepath, file, before.m_size, complete);
				auto const after = FileState::Read(filepath, file);
				m_stable = complete && before.SameContentVersion(after);
				if (m_stable || attempt + 1 == max_attempts)
					return;

				::Sleep(static_cast<DWORD>((attempt + 1) * back_off));
			}

			if (!m_data.empty())
				return;
			if (IsTransient(last_error))
				throw Error(filepath, "File is busy", last_error);

			throw Error(filepath, "Failed to open file", last_error);
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

		struct FileState
		{
			uint64_t m_size;
			FILETIME m_last_write;

			static FileState Read(std::filesystem::path const& filepath, HANDLE file)
			{
				auto info = BY_HANDLE_FILE_INFORMATION{};
				if (!::GetFileInformationByHandle(file, &info))
					throw Error(filepath, "Failed to query file information", ::GetLastError());

				auto size = ULARGE_INTEGER{};
				size.LowPart = info.nFileSizeLow;
				size.HighPart = info.nFileSizeHigh;
				return FileState{ size.QuadPart, info.ftLastWriteTime };
			}
			bool SameContentVersion(FileState const& rhs) const
			{
				return m_size == rhs.m_size && ::CompareFileTime(&m_last_write, &rhs.m_last_write) == 0;
			}
		};

		static bool IsTransient(DWORD err)
		{
			return err == ERROR_SHARING_VIOLATION || err == ERROR_LOCK_VIOLATION;
		}
		static std::runtime_error Error(std::filesystem::path const& filepath, std::string_view action, DWORD err)
		{
			auto const message = std::system_category().message(static_cast<int>(err));
			return std::runtime_error(std::format("{} '{}': {} ({})", action, filepath.string(), message, err));
		}
		static win32::Handle Open(std::filesystem::path const& filepath, DWORD share_mode, DWORD& error)
		{
			auto handle = win32::FileOpen(
				filepath,
				GENERIC_READ,
				share_mode,
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL,
				FILE_FLAG_SEQUENTIAL_SCAN);

			error = handle ? ERROR_SUCCESS : ::GetLastError();
			return handle;
		}
		static std::string ReadAll(std::filesystem::path const& filepath, HANDLE file, uint64_t file_size, bool& complete)
		{
			auto data = std::string{};
			if (file_size > static_cast<uint64_t>(data.max_size()))
				throw std::runtime_error(std::format("File '{}' is too large to snapshot", filepath.string()));

			data.resize(static_cast<size_t>(file_size));
			complete = true;
			for (auto ofs = size_t{}; ofs != data.size(); )
			{
				auto const remaining = data.size() - ofs;
				auto const to_read = static_cast<DWORD>(std::min<size_t>(remaining, std::numeric_limits<DWORD>::max()));
				auto bytes_read = DWORD{};
				if (!::ReadFile(file, data.data() + ofs, to_read, &bytes_read, nullptr))
					throw Error(filepath, "Failed to read file", ::GetLastError());
				if (bytes_read == 0)
				{
					data.resize(ofs);
					complete = false;
					return data;
				}

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

		std::filesystem::path m_filepath;
		std::streamoff m_file_offset;

		FileSnapshotStream(std::filesystem::path filepath, std::string data, size_t offset = 0)
			: storage_t(std::move(data))
			, mem_istream<char>(DataView(static_cast<storage_t const&>(*this), offset), 0)
			, m_filepath(std::move(filepath))
			, m_file_offset(static_cast<std::streamoff>(std::min(offset, static_cast<storage_t const&>(*this).size())))
		{}
		FileSnapshotStream(std::string data)
			: FileSnapshotStream(std::filesystem::path{}, std::move(data))
		{}
		FileSnapshotStream(FileSnapshot snapshot, size_t offset = 0)
			: FileSnapshotStream(std::move(snapshot.m_filepath), std::move(snapshot.m_data), offset)
		{}

		std::filesystem::path const& filepath() const noexcept
		{
			return m_filepath;
		}
		std::streamsize file_size() const noexcept
		{
			return static_cast<std::streamsize>(static_cast<storage_t const&>(*this).size());
		}
		std::streamoff file_offset() const noexcept
		{
			return m_file_offset;
		}

	private:

		static std::string_view DataView(storage_t const& data, size_t offset)
		{
			offset = std::min(offset, data.size());
			return std::string_view(data.data() + offset, data.size() - offset);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::filesys
{
	PRUnitTest(FileSnapshotTests, Stress)
	{
		auto const filepath = temp_dir() / std::format("pr_file_snapshot_{}_{}.tmp", ::GetCurrentProcessId(), ::GetTickCount64());

		{
			std::ofstream file(filepath, std::ios::binary);
			file << "snapshot data";
		}

		auto snapshot = FileSnapshot(filepath);
		PR_EXPECT(snapshot.str() == "snapshot data");
		PR_EXPECT(snapshot.m_stable);

		auto stream = FileSnapshotStream(std::move(snapshot));
		PR_EXPECT(stream.filepath() == filepath);
		PR_EXPECT(stream.file_size() == 13);
		PR_EXPECT(stream.file_offset() == 0);

		auto busy = win32::FileOpen(filepath, GENERIC_WRITE, FILE_SHARE_READ, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL);
		PR_EXPECT(!!busy);
		snapshot = FileSnapshot(filepath, FileSnapshot::Options{ .m_max_attempts = 1, .m_max_block_time_ms = 0 });
		PR_EXPECT(snapshot.str() == "snapshot data");
	}
}
#endif
