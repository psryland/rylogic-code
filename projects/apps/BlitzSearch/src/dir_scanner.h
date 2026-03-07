//***********************************************************************
// BlitzSearch
//  Copyright (c) Rylogic Ltd 2024
//***********************************************************************
#include <vector>
#include <filesystem>
#include <functional>
#include <concurrent_vector.h>
#include "pr/threads/thread_pool.h"

namespace pr::filesys
{
	struct DirScanner
	{
		// Builds a collection of files that match the filter
		// Scans using multiple threads.
		using filepath_t = std::filesystem::path;

		threads::ThreadPool m_thread_pool;
		concurrency::concurrent_vector<filepath_t> m_files;

		DirScanner(std::span<filepath_t const> paths, std::function<bool(filepath_t)> filter, int thread_count = std::thread::hardware_concurrency())
			: m_thread_pool(thread_count)
			, m_files()
		{
			Scan(paths, filter);
		}

		// Scan the paths for files that match the filter
		void Scan(std::span<filepath_t const> paths, std::function<bool(filepath_t)> filter)
		{
			for (auto const& path : paths)
				ScanDir(path, filter);
		}

		// Wait till the scan is complete
		void Wait()
		{
			m_thread_pool.WaitAll();
		}

		// Return the collection of files found
		std::vector<filepath_t> GetFiles() const
		{
			const_cast<DirScanner*>(this)->Wait();
			return std::vector<filepath_t>(m_files.begin(), m_files.end());
		}

	private:

		// Recursively scan directory for files that match the filter
		void ScanDir(filepath_t const& dir, std::function<bool(filepath_t)> filter)
		{
			m_thread_pool.QueueTask([this, dir, filter]
			{
				for (auto& dir_entry : std::filesystem::directory_iterator(dir))
				{
					if (dir_entry.is_directory())
					{
						ScanDir(dir_entry.path(), filter);
						continue;
					}

					auto filepath = dir_entry.path();
					if (!filter(filepath))
						continue;

					m_files.push_back(std::move(filepath));
				}
			});
		}
	};

}