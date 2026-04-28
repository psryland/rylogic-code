//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once
#include <vector>
#include <memory>
#include <istream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <unordered_map>
#include <format>
#include "pr/common/resource.h"
#include "pr/common/memstream.h"
#include "pr/common/event_handler.h"
#include "pr/common/flags_enum.h"
#include "pr/common/bit_fields.h"
#include "pr/filesys/file_snapshot.h"

namespace pr::filesys
{
	namespace resolver
	{
		// Info about the data being included
		enum class EFlags
		{
			None = 0,

			// True if the included data is binary data
			Binary = 1 << 0,

			// True for #include "file", false for #include <file>
			IncludeLocalDir = 1 << 1,

			// True if resolve failures do not throw errors
			IgnoreMissing = 1 << 2,

			_flags_enum = 0,
		};

		// Source locations for resolving paths
		enum class ESources
		{
			None = 0,
			Files = 1 << 0,
			Resources = 1 << 1,
			Strings = 1 << 2,
			All = ~None,
			_flags_enum = 0,
		};
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

	// A base class/interface for an implementation that can resolve paths to data
	struct IPathResolver
	{
		using EFlags = resolver::EFlags;
		using ESources = resolver::ESources;

		virtual ~IPathResolver() = default;

		// Add a path to the search paths. 'path' is the root path to search, 'index' controls the search order
		virtual void AddSearchPath(std::filesystem::path const& path, size_t index = ~size_t())
		{
			(void)path, index;
		}

		// Resolve an include into a full path.
		virtual std::filesystem::path ResolvePath(std::filesystem::path const& include, EFlags flags) const = 0;

		// Open 'path' as a binary stream
		virtual std::unique_ptr<std::istream> OpenStream(std::filesystem::path const& path, EFlags flags) const = 0;
	};

	// A path resolver that doesn't handle any paths.
	struct NoIncludes :IPathResolver
	{
		// Const default instance
		static NoIncludes const& Instance()
		{
			static NoIncludes s_instance;
			return s_instance;
		}

		// Resolve an include into a full path
		std::filesystem::path ResolvePath(std::filesystem::path const&, EFlags flags) const override
		{
			// Ignore if missing includes flagged
			if (AllSet(flags, EFlags::IgnoreMissing))
				return {};

			throw std::runtime_error("#include is not supported");
		}

		// Open 'include' as an ASCII stream
		std::unique_ptr<std::istream> OpenStream(std::filesystem::path const&, EFlags flags) const override
		{
			// Ignore if missing includes flagged
			if (AllSet(flags, EFlags::IgnoreMissing))
				return nullptr;

			throw std::runtime_error("#include is not supported");
		}
	};

	// A path resolver that tries to open data from resources, search paths, or a string table
	struct PathResolver :IPathResolver
	{
		// Notes:
		//  - Opening a file often means the LocalDir is set to the directory of the file.
		//    Callers should do this via a non-const reference to this, rather than all the resolve
		//    methods being non-const. Changing 'LocalDir' could cause race conditions, so the caller
		//    needs to manage changing it.
		using path_t = std::filesystem::path;
		using paths_t = std::vector<path_t>;
		using modules_t = std::vector<HMODULE>;
		using strtable_t = std::unordered_map<std::string, std::string>;

		// Const default instance
		static PathResolver const& Instance()
		{
			static PathResolver s_instance;
			return s_instance;
		}

	private:

		// Source that paths can resolve from
		ESources m_sources;

		// The search paths to resolve include files from
		paths_t m_paths;

		// The binary modules containing resources
		modules_t m_modules;

		// A map of include names to UTF-8 strings.
		strtable_t m_strtab;

		// The current 'local' directory
		path_t m_local_dir;

	public:

		PathResolver()
			: PathResolver(ESources::All)
		{}
		explicit PathResolver(ESources sources)
			: m_sources(sources)
			, m_paths()
			, m_modules()
			, m_strtab()
			, m_local_dir()
			, FileOpened()
		{}
		explicit PathResolver(std::string_view search_paths, ESources sources = ESources::All)
			:PathResolver(sources)
		{
			SearchPathList(search_paths);
		}
		explicit PathResolver(std::initializer_list<HMODULE> modules, ESources sources = ESources::All)
			:PathResolver(sources)
		{
			ResourceModules(modules);
		}
		explicit PathResolver(std::string_view search_paths, std::initializer_list<HMODULE> modules, ESources sources = ESources::All)
			:PathResolver(sources)
		{
			SearchPathList(search_paths);
			ResourceModules(modules);
		}

		PathResolver(PathResolver&&) = default;
		PathResolver(PathResolver const&) = default;
		PathResolver& operator=(PathResolver&&) = default;
		PathResolver& operator=(PathResolver const&) = default;

		// Raised whenever a file is opened
		EventHandler<PathResolver const&, std::filesystem::path const&, true> FileOpened;

		// Get/Set the locations to look for includes
		ESources Sources() const
		{
			return m_sources;
		}
		void Sources(ESources sources)
		{
			m_sources = sources;
		}

		// Get/Set the current "local" directory
		std::filesystem::path const& LocalDir() const
		{
			return m_local_dir;
		}
		void LocalDir(std::filesystem::path const& local_dir)
		{
			m_local_dir = local_dir;
		}

		// Get/Set the search directories for include files
		paths_t const& SearchPaths() const
		{
			return m_paths;
		}
		void SearchPaths(std::initializer_list<std::filesystem::path> paths)
		{
			m_paths.assign(paths);
		}

		// Get/Set the modules to check for resources
		modules_t const& ResourceModules() const
		{
			return m_modules;
		}
		void ResourceModules(std::initializer_list<HMODULE> modules)
		{
			m_modules.assign(modules);
		}

		// Get/Set the string table
		strtable_t const& StringTable() const
		{
			return m_strtab;
		}
		void StringTable(strtable_t const& strtab)
		{
			m_strtab = strtab;
		}

		// Get/Set the search paths as a delimited list
		std::string SearchPathList() const
		{
			std::string paths;
			for (auto& path : m_paths)
			{
				if (!paths.empty()) paths.append(1, ',');
				paths.append(path.string());
			}
			return paths;
		}
		void SearchPathList(std::string_view paths)
		{
			m_paths.resize(0);
			str::Split<std::string_view>(paths, ",;\n", [&](auto sub, int)
			{
				m_paths.push_back(sub);
			});
		}

		// Add a path to the include search paths. Ensures uniqueness of paths
		void AddSearchPath(std::filesystem::path const& path, size_t index = ~size_t()) override
		{
			auto p = path.lexically_normal();

			// Remove 'path' if already in the 'm_paths' collection
			auto end = std::remove_if(std::begin(m_paths), std::end(m_paths), [=](auto const& s) { return s == p; });
			m_paths.erase(end, std::end(m_paths));
			m_paths.insert(std::begin(m_paths) + std::min(m_paths.size(), index), p);
		}

		// Add a module handle to the modules collection. Ensures uniqueness
		void AddResourceModule(HMODULE module, size_t index = ~size_t())
		{
			// Remove 'module' if already in the 'm_modules' collection
			auto end = std::remove_if(std::begin(m_modules), std::end(m_modules), [&](auto m) { return m == module; });
			m_modules.erase(end, std::end(m_modules));
			m_modules.insert(std::begin(m_modules) + std::min(m_modules.size(), index), module);
		}

		// Add a string to the string include table
		void AddString(std::string_view key, std::string_view value)
		{
			m_strtab[std::string(key)] = value;
		}

		// Resolve an include into a full path. Use 'EFlags::IncludeLocalDir' for (#include "file" vs. #include <file>).
		std::filesystem::path ResolvePath(std::filesystem::path const& include, EFlags flags) const override
		{
			// Search files regardless of 'm_types' since this function is specifically for resolving filepaths
			std::filesystem::path fullpath;
			std::vector<std::filesystem::path> searched_paths;
			auto local_dir = AllSet(flags, EFlags::IncludeLocalDir) ? &m_local_dir : nullptr;
			if (ResolveFileInclude(include, local_dir, fullpath, searched_paths))
				return fullpath;

			// Ignore if missing includes flagged
			if (AllSet(flags, EFlags::IgnoreMissing))
				return {};

			// Raise an include missing error
			auto msg = std::format("Failed to resolve include '{}'", include.string());
			if (!searched_paths.empty()) msg.append("\n\nNot found in these search paths:");
			for (auto& path : searched_paths) msg.append("\n").append(path.string());
			throw std::runtime_error(msg);
		}

		// Open 'path' as a binary stream
		std::unique_ptr<std::istream> OpenStream(std::filesystem::path const& include, EFlags flags) const override
		{
			// Try file includes
			std::filesystem::path fullpath;
			std::vector<std::filesystem::path> searched_paths;
			auto local_dir = AllSet(flags, EFlags::IncludeLocalDir) ? &m_local_dir : nullptr;
			if (AllSet(m_sources, ESources::Files) && ResolveFileInclude(include, local_dir, fullpath, searched_paths))
			{
				auto snapshot = FileSnapshot(fullpath);
				FileOpened(*this, fullpath);
				return std::make_unique<FileSnapshotStream>(std::move(snapshot.m_data));
			}

			// Try resources
			HMODULE module;
			auto id = ResId(include);
			if (AllSet(m_sources, ESources::Resources) && ResolveResourceInclude(id, AllSet(flags, EFlags::Binary), module))
			{
				auto res = resource::Read<char>(id, AllSet(flags, EFlags::Binary) ? L"BINARY" : L"TEXT", module);
				return std::unique_ptr<std::basic_istream<char>>(new mem_istream<char>(res.m_data, res.size()));
			}

			// Try the string table
			strtable_t const* strtab;
			auto tag = include.string();
			if (AllSet(m_sources, ESources::Strings) && ResolveStringInclude(tag, strtab))
			{
				auto const& str = (*strtab).at(tag);
				return std::unique_ptr<std::istringstream>(new std::istringstream(str));
			}

			// If ignoring missing includes, return an empty source
			if (AllSet(flags, EFlags::IgnoreMissing))
				return std::unique_ptr<std::istringstream>(new std::istringstream(""));

			// Raise an include missing error
			auto msg = std::format("Failed to open include stream '{}'", include.string());
			if (!searched_paths.empty()) msg.append("\n\nNot found in these search paths:");
			for (auto& path : searched_paths) msg.append("\n").append(path.string());
			throw std::runtime_error(msg);
		}

	private:

		// Resolve an include into a full path
		bool ResolveFileInclude(std::filesystem::path const& include, std::filesystem::path const* local_dir, std::filesystem::path& result, paths_t& searched_paths) const 
		{
			result.clear();

			// Resolve the filepath
			auto filepath = filesys::ResolvePath(include, m_paths, local_dir, false, &searched_paths);
			if (filepath.empty())
				return false;

			// Return the resolved include
			result = filepath;
			return true;
		}

		// Resolve an include from the available modules
		bool ResolveResourceInclude(std::wstring_view id, bool binary, HMODULE& module) const
		{
			for (auto m : m_modules)
			{
				if (!resource::Find(id, binary ? L"BINARY" : L"TEXT", m)) continue;
				module = m;
				return true;
			}
			return false;
		}

		// Resolve an include into a string that is in the string table
		bool ResolveStringInclude(std::string const& tag, strtable_t const*& strtab) const
		{
			// Future version may have multiple string tables
			strtab = m_strtab.find(tag) != std::end(m_strtab) ? &m_strtab : nullptr;
			return strtab != nullptr;
		}

		// Convert 'name' into a resource string id
		static std::wstring ResId(std::filesystem::path const& name)
		{
			auto id = name.wstring();
			for (auto& ch : id) { ch = ch == '.' ? '_' : static_cast<wchar_t>(toupper(ch)); }
			return id;
		}
	};
}

