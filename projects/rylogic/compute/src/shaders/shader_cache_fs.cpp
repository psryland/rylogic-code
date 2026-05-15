//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/compute/shaders/shader_cache_fs.h"
#include "pr/compute/shaders/source_resolver.h"
namespace pr::compute::shader_cache
{
	namespace
	{
		static constexpr uint32_t CacheFormatVersion = 1;
		static constexpr char ManifestMagic[] = "RDR12SHC";

		struct CacheEntry
		{
			std::string m_key_text;
			CompileOutput m_output;
			std::vector<Dependency> m_dependencies;
		};

		// Return a filesystem-safe directory name for a cache namespace.
		std::filesystem::path CacheNamespace(std::string_view cache_namespace)
		{
			auto name = std::string(cache_namespace);
			if (name.empty())
				name = "default";

			for (auto& ch : name)
			{
				switch (ch)
				{
					case '<':
					case '>':
					case ':':
					case '"':
					case '/':
					case '\\':
					case '|':
					case '?':
					case '*':
					{
						ch = '_';
						break;
					}
					default:
					{
						break;
					}
				}
			}
			return std::filesystem::path(Widen(name));
		}

		// Write all bytes in 'data' to 'path'.
		void WriteBytes(std::filesystem::path const& path, std::span<uint8_t const> data)
		{
			auto file = std::ofstream(path, std::ios::binary);
			if (!file)
				throw std::runtime_error(std::format("Failed to write shader cache file '{}'", path.string()));

			if (!data.empty())
				file.write(reinterpret_cast<char const*>(data.data()), static_cast<std::streamsize>(data.size()));
		}

		// Write a plain-old-data value to a stream.
		template <typename Type> requires (std::is_trivially_copyable_v<Type>)
		void WritePod(std::ostream& os, Type const& value)
		{
			os.write(reinterpret_cast<char const*>(&value), sizeof(value));
		}

		// Read a plain-old-data value from a stream.
		template <typename Type> requires (std::is_trivially_copyable_v<Type>)
		Type ReadPod(std::istream& is)
		{
			Type value;
			is.read(reinterpret_cast<char*>(&value), sizeof(value));
			if (!is)
				throw std::runtime_error("Shader cache manifest is truncated");

			return value;
		}

		// Write a UTF-8 string to a stream.
		void WriteString(std::ostream& os, std::string_view value)
		{
			auto size = static_cast<uint64_t>(value.size());
			WritePod(os, size);
			if (size != 0)
				os.write(value.data(), static_cast<std::streamsize>(size));
		}

		// Read a UTF-8 string from a stream.
		std::string ReadString(std::istream& is)
		{
			auto size = ReadPod<uint64_t>(is);
			if (size > static_cast<uint64_t>((std::numeric_limits<std::string::size_type>::max)()))
				throw std::runtime_error("Shader cache manifest string is too large");

			std::string value(static_cast<size_t>(size), '\0');
			if (size != 0)
				is.read(value.data(), static_cast<std::streamsize>(size));
			if (!is)
				throw std::runtime_error("Shader cache manifest string is truncated");

			return value;
		}

		// Write a UTF-16 string as UTF-8 to a stream.
		void WriteWString(std::ostream& os, std::wstring_view value)
		{
			WriteString(os, Narrow(value));
		}

		// Read a UTF-16 string stored as UTF-8 from a stream.
		std::wstring ReadWString(std::istream& is)
		{
			return Widen(ReadString(is));
		}

		// Write one dependency to a stream.
		void WriteDependency(std::ostream& os, Dependency const& dep)
		{
			WritePod(os, static_cast<uint32_t>(dep.m_kind));
			WriteString(os, dep.m_identity);
			WriteWString(os, dep.m_path.wstring());
			WriteWString(os, dep.m_type);
			WritePod(os, dep.m_size);
			WritePod(os, dep.m_last_write);
			WritePod(os, dep.m_hash);
		}

		// Read one dependency from a stream.
		Dependency ReadDependency(std::istream& is)
		{
			auto kind = static_cast<Dependency::EKind>(ReadPod<uint32_t>(is));
			auto identity = ReadString(is);
			auto path = std::filesystem::path(ReadWString(is));
			auto type = ReadWString(is);
			auto size = ReadPod<uint64_t>(is);
			auto last_write = ReadPod<uint64_t>(is);
			auto hash = ReadPod<uint64_t>(is);

			return Dependency{
				.m_kind = kind,
				.m_identity = std::move(identity),
				.m_path = std::move(path),
				.m_type = std::move(type),
				.m_size = size,
				.m_last_write = last_write,
				.m_hash = hash,
			};
		}

		// Write a cache manifest.
		void WriteManifest(std::filesystem::path const& path, CacheEntry const& entry)
		{
			auto file = std::ofstream(path, std::ios::binary);
			if (!file)
				throw std::runtime_error(std::format("Failed to write shader cache manifest '{}'", path.string()));

			file.write(ManifestMagic, sizeof(ManifestMagic));
			WritePod(file, CacheFormatVersion);
			WriteString(file, entry.m_key_text);
			WriteWString(file, entry.m_output.m_pdb_name);

			auto dep_count = static_cast<uint64_t>(entry.m_dependencies.size());
			WritePod(file, dep_count);
			for (auto const& dep : entry.m_dependencies)
				WriteDependency(file, dep);
		}

		// Read a cache manifest.
		CacheEntry ReadManifest(std::filesystem::path const& path)
		{
			auto file = std::ifstream(path, std::ios::binary);
			if (!file)
				throw std::runtime_error(std::format("Failed to read shader cache manifest '{}'", path.string()));

			char magic[sizeof(ManifestMagic)] = {};
			file.read(magic, sizeof(magic));
			if (memcmp(magic, ManifestMagic, sizeof(ManifestMagic)) != 0)
				throw std::runtime_error("Shader cache manifest has an invalid header");

			auto version = ReadPod<uint32_t>(file);
			if (version != CacheFormatVersion)
				throw std::runtime_error("Shader cache manifest has an unsupported version");

			auto entry = CacheEntry{};
			entry.m_key_text = ReadString(file);
			entry.m_output.m_pdb_name = ReadWString(file);

			auto dep_count = ReadPod<uint64_t>(file);
			if (dep_count > 4096)
				throw std::runtime_error("Shader cache manifest has too many dependencies");

			entry.m_dependencies.reserve(static_cast<size_t>(dep_count));
			for (auto i = uint64_t{}; i != dep_count; ++i)
				entry.m_dependencies.push_back(ReadDependency(file));

			return entry;
		}

		// Append a length-prefixed string to cache key material.
		void AppendKey(std::string& key_text, std::string_view label, std::string_view value)
		{
			key_text.append(label);
			key_text.push_back('=');
			key_text.append(std::to_string(value.size()));
			key_text.push_back(':');
			key_text.append(value);
			key_text.push_back('\n');
		}

		// Append a length-prefixed wide string to cache key material.
		void AppendKey(std::string& key_text, std::string_view label, std::wstring_view value)
		{
			AppendKey(key_text, label, Narrow(value));
		}

		// Append dependency metadata to cache key material.
		void AppendDependencyKey(std::string& key_text, Dependency const& dep)
		{
			AppendKey(key_text, "dep.kind", std::to_string(static_cast<int>(dep.m_kind)));
			AppendKey(key_text, "dep.identity", dep.m_identity);
			AppendKey(key_text, "dep.path", dep.m_path.wstring());
			AppendKey(key_text, "dep.type", dep.m_type);
			AppendKey(key_text, "dep.size", std::to_string(dep.m_size));
			AppendKey(key_text, "dep.last_write", std::to_string(dep.m_last_write));
			AppendKey(key_text, "dep.hash", std::to_string(dep.m_hash));
		}

		// Return the canonical text used to identify a cache entry before include validation.
		std::string KeyText(ShaderCacheFS::Config const& config, CompileRequest const& request)
		{
			auto key_text = std::string{};
			AppendKey(key_text, "format", std::to_string(CacheFormatVersion));
			AppendKey(key_text, "namespace", config.m_namespace);
			AppendKey(key_text, "source", request.m_source_identity);
			AppendDependencyKey(key_text, request.m_source_dependency);
			AppendKey(key_text, "resolver", request.m_resolver_key);
			AppendKey(key_text, "compiler", request.m_compiler_key);
			AppendKey(key_text, "pdb_name_hint", request.m_pdb_name_hint);
			for (auto const& arg : request.m_args)
				AppendKey(key_text, "arg", arg);

			return key_text;
		}

		// Return a hex cache key for 'key_text'.
		std::string EntryKey(std::string_view key_text)
		{
			auto hash_value = key_text.empty()
				? uint64_t{}
				: static_cast<uint64_t>(hash::HashBytes64(key_text.data(), key_text.data() + key_text.size()));

			return std::format("{:016X}", hash_value);
		}

		// Return the directory that stores an entry for 'key'.
		std::filesystem::path EntryDir(ShaderCacheFS::Config const& config, std::string_view key)
		{
			return config.m_cache_dir / CacheNamespace(config.m_namespace) / Widen(key.substr(0, 2)) / Widen(key);
		}

		// Return a lock file path for an entry key.
		std::filesystem::path LockPath(ShaderCacheFS::Config const& config, std::string_view key)
		{
			return config.m_cache_dir / CacheNamespace(config.m_namespace) / Widen(key.substr(0, 2)) / (Widen(key) + L".lock");
		}

		// Return true if every stored dependency still matches the current dependency metadata.
		bool DependenciesValid(std::vector<Dependency> const& dependencies)
		{
			for (auto const& dep : dependencies)
			{
				auto current = CurrentDependency(dep);
				if (!current || !dep.SameContentVersion(*current))
					return false;
			}
			return true;
		}

		// Return a cache entry if the files exist, parse, and validate.
		std::optional<CompileOutput> TryReadEntry(std::filesystem::path const& entry_dir, std::string_view key_text)
		{
			if (!std::filesystem::exists(entry_dir / L"manifest.bin"))
				return std::nullopt;

			auto entry = ReadManifest(entry_dir / L"manifest.bin");
			if (entry.m_key_text != key_text)
				throw std::runtime_error("Shader cache key collision");
			if (!DependenciesValid(entry.m_dependencies))
				return std::nullopt;

			entry.m_output.m_byte_code = ReadBytes(entry_dir / L"bytecode.bin");
			auto pdb_path = entry_dir / L"pdb.bin";
			if (!entry.m_output.m_pdb_name.empty() && std::filesystem::exists(pdb_path))
				entry.m_output.m_pdb = ReadBytes(pdb_path);

			return entry.m_output;
		}

		// Materialise cached side outputs requested by the caller.
		bool MaterialisePDB(CompileRequest const& request, CompileOutput const& output)
		{
			if (request.m_pdb_output_dir.empty() || output.m_pdb.empty() || output.m_pdb_name.empty())
				return false;

			std::filesystem::create_directories(request.m_pdb_output_dir);
			WriteBytes(request.m_pdb_output_dir / output.m_pdb_name, output.m_pdb);
			return true;
		}

		// Return a kernel-owned per-key lock, or null if the wait timed out.
		win32::Handle AcquireLock(std::filesystem::path const& lock_path, std::chrono::milliseconds timeout, std::chrono::milliseconds poll_interval)
		{
			std::filesystem::create_directories(lock_path.parent_path());

			auto const start = std::chrono::steady_clock::now();
			for (;;)
			{
				auto lock = win32::FileOpen(
					lock_path,
					GENERIC_READ | GENERIC_WRITE | DELETE,
					0,
					CREATE_ALWAYS,
					FILE_ATTRIBUTE_TEMPORARY,
					FILE_FLAG_DELETE_ON_CLOSE);
				if (lock)
					return lock;

				auto const err = ::GetLastError();
				if (err != ERROR_SHARING_VIOLATION && err != ERROR_ACCESS_DENIED)
					throw std::runtime_error(std::format("Failed to acquire shader cache lock '{}': {}", lock_path.string(), err));

				if (std::chrono::steady_clock::now() - start >= timeout)
					return {};

				std::this_thread::sleep_for(poll_interval);
			}
		}

		// Return a dependency list sorted and deduplicated by stable identity.
		std::vector<Dependency> NormaliseDependencies(std::vector<Dependency> dependencies)
		{
			std::ranges::sort(dependencies, [](Dependency const& lhs, Dependency const& rhs)
			{
				auto lkey = std::tie(lhs.m_kind, lhs.m_identity, lhs.m_path, lhs.m_type);
				auto rkey = std::tie(rhs.m_kind, rhs.m_identity, rhs.m_path, rhs.m_type);
				return lkey < rkey;
			});

			auto same_dep = [](Dependency const& lhs, Dependency const& rhs)
			{
				return lhs.m_kind == rhs.m_kind &&
					lhs.m_identity == rhs.m_identity &&
					lhs.m_path == rhs.m_path &&
					lhs.m_type == rhs.m_type;
			};
			dependencies.erase(std::unique(dependencies.begin(), dependencies.end(), same_dep), dependencies.end());
			return dependencies;
		}

		// Publish a freshly compiled entry to the cache directory.
		void WriteEntry(std::filesystem::path const& entry_dir, std::string key_text, CompileResult result)
		{
			auto parent = entry_dir.parent_path();
			std::filesystem::create_directories(parent);

			auto tmp_name = std::format(L"{}.tmp.{}.{}", entry_dir.filename().wstring(), ::GetCurrentProcessId(), ::GetCurrentThreadId());
			auto tmp_dir = parent / tmp_name;
			std::error_code ec;
			std::filesystem::remove_all(tmp_dir, ec);
			std::filesystem::create_directories(tmp_dir);

			auto cleanup = Scope<void>([&]
			{
				std::error_code ignored;
				std::filesystem::remove_all(tmp_dir, ignored);
			});

			auto entry = CacheEntry{
				.m_key_text = std::move(key_text),
				.m_output = result.m_output,
				.m_dependencies = NormaliseDependencies(std::move(result.m_dependencies)),
			};
			WriteBytes(tmp_dir / L"bytecode.bin", entry.m_output.m_byte_code);
			if (!entry.m_output.m_pdb.empty())
				WriteBytes(tmp_dir / L"pdb.bin", entry.m_output.m_pdb);
			WriteManifest(tmp_dir / L"manifest.bin", entry);

			std::filesystem::remove_all(entry_dir, ec);
			if (!::MoveFileExW(tmp_dir.c_str(), entry_dir.c_str(), MOVEFILE_WRITE_THROUGH))
				throw std::runtime_error(std::format("Failed to publish shader cache entry '{}': {}", entry_dir.string(), ::GetLastError()));

			cleanup.m_dont = true;
		}
	}

	ShaderCacheFS::ShaderCacheFS(std::filesystem::path cache_dir, std::string cache_namespace)
		: ShaderCacheFS(Config{
			.m_cache_dir = std::move(cache_dir),
			.m_namespace = std::move(cache_namespace),
		})
	{
	}
	ShaderCacheFS::ShaderCacheFS(Config config)
		: m_config(std::move(config))
		, m_mutex()
		, m_stats()
	{
	}

	// Return cached output or compile and publish a new cache entry.
	CompileOutput ShaderCacheFS::GetOrCompile(CompileRequest const& request, IShaderCache::CompileFunc compile)
	{
		// If the request is missing any cache identity data, treat the cache as disabled for this compile.
		if (!request.IsCacheable())
		{
			auto result = compile();
			return std::move(result.m_output);
		}

		// Build the stable cache key and the corresponding filesystem locations once so all paths agree.
		auto const key_text = KeyText(m_config, request);
		auto const key = EntryKey(key_text);
		auto const entry_dir = EntryDir(m_config, key);
		auto const lock_path = LockPath(m_config, key);

		try
		{
			// Optimistic read before locking. Most calls should be hits, and valid entries are immutable once published.
			if (auto output = TryReadEntry(entry_dir, key_text))
			{
				if (MaterialisePDB(request, *output))
					Count(&CacheStats::m_hit_pdb_materialised);

				Count(&CacheStats::m_hit_validated);
				return *output;
			}
		}
		catch (...)
		{
			// Treat corrupt or partially written entries as cache misses. The compiler path below is still the source of truth.
			Count(&CacheStats::m_io_error_uncached);
			std::error_code ec;
			std::filesystem::remove_all(entry_dir, ec);
		}

		auto lock = win32::Handle{};
		try
		{
			// Serialise writers for this key across processes. Readers can still use already-published immutable entries.
			lock = AcquireLock(lock_path, m_config.m_lock_timeout, m_config.m_lock_poll_interval);
			if (!lock)
			{
				// Shader compilation must not fail just because the cache lock is held too long; compile without publishing.
				Count(&CacheStats::m_lock_timeout_uncached);
				auto result = compile();
				return std::move(result.m_output);
			}

			// Another process/thread may have populated the entry while we waited for the lock.
			if (auto output = TryReadEntry(entry_dir, key_text))
			{
				if (MaterialisePDB(request, *output))
					Count(&CacheStats::m_hit_pdb_materialised);

				Count(&CacheStats::m_hit_validated);
				return *output;
			}
		}
		catch (...)
		{
			// Cache filesystem/locking failures should not prevent shader creation. Fall back to an uncached compile.
			Count(&CacheStats::m_io_error_uncached);
			auto result = compile();
			return std::move(result.m_output);
		}

		auto result = CompileResult{};
		try
		{
			// We hold the cache lock and no valid entry exists, so this compile will become the candidate cache payload.
			result = compile();
		}
		catch (...)
		{
			Count(&CacheStats::m_miss_compile_failed);
			throw;
		}

		try
		{
			// Publishing is best-effort. The compile result is returned even if writing the cache entry fails.
			WriteEntry(entry_dir, key_text, result);
			Count(&CacheStats::m_miss_compiled);
		}
		catch (...)
		{
			Count(&CacheStats::m_io_error_uncached);
		}

		return std::move(result.m_output);
	}

	// Delete entries owned by this cache namespace.
	void ShaderCacheFS::Clear()
	{
		std::error_code ec;
		std::filesystem::remove_all(m_config.m_cache_dir / CacheNamespace(m_config.m_namespace), ec);
		if (ec)
			throw std::runtime_error(std::format("Failed to clear shader cache '{}': {}", m_config.m_cache_dir.string(), ec.message()));
	}

	// Return diagnostic counters for this cache instance.
	CacheStats ShaderCacheFS::Stats() const
	{
		auto lock = std::scoped_lock(m_mutex);
		return m_stats;
	}

	// Add 'value' to a named diagnostic counter.
	void ShaderCacheFS::Count(uint64_t CacheStats::*field, uint64_t value)
	{
		auto lock = std::scoped_lock(m_mutex);
		m_stats.*field += value;
	}
}

