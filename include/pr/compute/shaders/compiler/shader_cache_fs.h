//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/shaders/compiler/shader_cache.h"

namespace pr::compute::shader_cache
{
	// Filesystem-backed compiled shader cache.
	struct ShaderCacheFS : IShaderCache
	{
		struct Config
		{
			std::filesystem::path m_cache_dir;
			std::string m_namespace;
			std::chrono::milliseconds m_lock_timeout = std::chrono::seconds(30);
			std::chrono::milliseconds m_lock_poll_interval = std::chrono::milliseconds(10);
		};

		Config m_config;
		mutable std::mutex m_mutex;
		CacheStats m_stats;

		explicit ShaderCacheFS(std::filesystem::path cache_dir, std::string cache_namespace = {});
		explicit ShaderCacheFS(Config config);

		// Return cached output or compile and publish a new cache entry.
		CompileOutput GetOrCompile(CompileRequest const& request, IShaderCache::CompileFunc compile) override;

		// Delete entries owned by this cache namespace.
		void Clear();

		// Return diagnostic counters for this cache instance.
		CacheStats Stats() const override;

	private:

		// Add 'value' to a named diagnostic counter.
		void Count(uint64_t CacheStats::*field, uint64_t value = 1);
	};
}

