#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// Maximum number of entries in the Recent Files submenu
	static constexpr int MaxRecentFiles = 10;

	// Persistent recent-files list stored in %APPDATA%.
	// Keeps an MRU (most-recently-used) list of scene file paths,
	// saved to a simple line-delimited text file between sessions.
	struct RecentFiles
	{
		std::vector<std::filesystem::path> m_paths;

		// Load/Save the recent files list from/to disk.
		void Load();
		void Save() const;

		// Add a path to the front (MRU order), removing duplicates and capping at max
		void Add(std::filesystem::path const& filepath);
	};
}