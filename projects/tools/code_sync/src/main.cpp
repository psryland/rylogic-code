//************************************
// CodeSync
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/code_sync.h"

namespace code_sync
{
	namespace fs = std::filesystem;

	void ShowHelp()
	{
		std::cout <<
			"CodeSync - Synchronise code blocks across files\n"
			"\n"
			"Usage: code_sync <dir1> [dir2] [...] [options]\n"
			"\n"
			"Options:\n"
			"  --ignore-dir <pattern> Ignore directory names at any depth below roots (repeatable)\n"
			"                         Case-insensitive; '*' matches any characters, '?' matches one\n"
			"  --tab-size N            Tab width in spaces (default: 4)\n"
			"  --stamp <path>          Stamp file to prevent re-runs within the same build\n"
			"  --stamp-max-age <secs>  Max age of stamp file in seconds (default: 30)\n"
			"  --verbose, -v           Print diagnostic output\n"
			"  --help, -h              Show this help\n"
			"\n"
			"Blocks tagged with PR_CODE" "_SYNC_BEGIN(name, source_of_truth) are the reference.\n"
			"Blocks tagged with PR_CODE" "_SYNC_BEGIN(name) are replaced with the reference content.\n"
			"All blocks end with PR_CODE" "_SYNC_END().\n"
			"\n";
	}
}

int main(int argc, char* argv[])
{
	using namespace code_sync;

	try
	{
		std::vector<fs::path> directories;
		std::vector<std::wstring> ignore_dirs;
		std::string stamp_path;
		int tab_size = 4;
		int stamp_max_age_sec = 30;
		bool verbose = false;

		for (int i = 1; i < argc; ++i)
		{
			auto arg = std::string_view(argv[i]);
			if (arg == "--ignore-dir")
			{
				if (i + 1 == argc || std::string_view(argv[i + 1]).starts_with("--"))
					throw std::invalid_argument("--ignore-dir requires a directory-name pattern.");

				auto pattern = std::string_view(argv[++i]);
				if (pattern.empty() || pattern.find_first_of("\\/:") != std::string_view::npos)
					throw std::invalid_argument("--ignore-dir requires a nonempty directory-name pattern, not a path.");

				ignore_dirs.push_back(fs::path(pattern).wstring());
			}
			else if (arg == "--tab-size" && i + 1 < argc)
				tab_size = std::stoi(argv[++i]);
			else if (arg == "--stamp" && i + 1 < argc)
				stamp_path = argv[++i];
			else if (arg == "--stamp-max-age" && i + 1 < argc)
				stamp_max_age_sec = std::stoi(argv[++i]);
			else if (arg == "--verbose" || arg == "-v")
				verbose = true;
			else if (arg == "--help" || arg == "-h")
				return ShowHelp(), 0;
			else if (arg.starts_with("--"))
				throw std::invalid_argument("Unknown or incomplete option: " + std::string(arg));
			else
				directories.emplace_back(fs::path(arg));
		}

		if (directories.empty())
		{
			ShowHelp();
			return 1;
		}

		// Normalise paths
		for (auto& dir : directories)
			dir = fs::canonical(dir);

		// Use a named mutex to serialize parallel invocations (e.g. from MSBuild parallel builds)
		auto mutex = CreateMutexW(nullptr, FALSE, L"Global\\RylogicCodeSync");
		if (mutex == nullptr)
			throw std::runtime_error("Failed to create mutex");

		WaitForSingleObject(mutex, INFINITE);
		try
		{
			// If a stamp file exists and is recent, skip the run
			if (!stamp_path.empty())
			{
				auto stamp = fs::path(stamp_path);
				if (fs::exists(stamp))
				{
					auto age = fs::file_time_type::clock::now() - fs::last_write_time(stamp);
					if (age < std::chrono::seconds(stamp_max_age_sec))
					{
						if (verbose)
							std::cout << "CodeSync: Skipped (recent stamp exists)." << std::endl;

						ReleaseMutex(mutex);
						CloseHandle(mutex);
						return 0;
					}
				}
			}

			CodeSync sync(tab_size, verbose, std::move(ignore_dirs));
			sync.Run(directories);

			// Write the stamp file after a successful run
			if (!stamp_path.empty())
			{
				auto stamp = fs::path(stamp_path);
				auto stamp_dir = stamp.parent_path();
				if (!stamp_dir.empty() && !fs::exists(stamp_dir))
					fs::create_directories(stamp_dir);

				std::ofstream f(stamp);
				f << "ok";
			}

			ReleaseMutex(mutex);
			CloseHandle(mutex);
			return 0;
		}
		catch (...)
		{
			ReleaseMutex(mutex);
			CloseHandle(mutex);
			throw;
		}
	}
	catch (std::exception const& ex)
	{
		std::cerr << "CodeSync error: " << ex.what() << std::endl;
		return 1;
	}
}
