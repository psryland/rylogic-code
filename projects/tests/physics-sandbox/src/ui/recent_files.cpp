#include "src/ui/recent_files.h"

namespace physics_sandbox
{
	// Load/Save the recent files list from/to disk.
	void RecentFiles::Load()
	{
		m_paths.clear();

		auto path = AppDataPath() / "recent_files.txt";
		if (!std::filesystem::exists(path))
			return;

		if (std::ifstream ifile(path); ifile)
		{
			for (std::string line; std::getline(ifile, line);)
			{
				if (!line.empty())
					m_paths.emplace_back(line);
			}
		}
	}

	// Load/Save the recent files list from/to disk.
	void RecentFiles::Save() const
	{
		auto path = AppDataPath() / "recent_files.txt";
		if (path.empty())
			return;

		if (std::ofstream ofile(path); ofile)
		{
			for (auto const& p : m_paths)
				ofile << p.string() << "\n";
		}
	}

	// Add a path to the front (MRU order), removing duplicates and capping at max
	void RecentFiles::Add(std::filesystem::path const& filepath)
	{
		// Remove any existing entry for this path
		auto canonical = std::filesystem::weakly_canonical(filepath);
		std::erase_if(m_paths, [&](auto const& p)
		{
			return std::filesystem::weakly_canonical(p) == canonical;
		});

		// Insert at front (most recent first)
		m_paths.insert(m_paths.begin(), canonical);

		// Cap at maximum
		if (static_cast<int>(m_paths.size()) > MaxRecentFiles)
			m_paths.resize(MaxRecentFiles);

		Save();
	}
}
