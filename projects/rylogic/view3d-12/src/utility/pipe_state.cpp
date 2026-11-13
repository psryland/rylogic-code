//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"

namespace pr::rdr12
{
	namespace
	{
		// Create the pipeline state object described by 'desc'
		D3DPtr<ID3D12PipelineState> CreatePSO(ID3D12Device4* device, PipeStateDesc const& desc)
		{
			// Create the pipeline from the final description
			auto const& d3d_desc = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
			D3DPtr<ID3D12PipelineState> pso;
			Check(device->CreateGraphicsPipelineState(&d3d_desc, __uuidof(ID3D12PipelineState), (void**)pso.address_of()));
			return pso;
		}

		// Milliseconds since 'start'
		double ElapsedMs(std::chrono::steady_clock::time_point start)
		{
			return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
		}
	}

	// Return a hash of the final state, for lookup only
	uint64_t PipeStateDesc::Hash() const
	{
		// Mix the description one word at a time. It is hashed on every pipeline change while drawing, so this must be cheap.
		static_assert(sizeof(m_desc) % sizeof(uint64_t) == 0);
		auto hash = 0x9E3779B97F4A7C15ULL;
		auto const* words = reinterpret_cast<uint64_t const*>(&m_desc);
		for (size_t i = 0, iend = sizeof(m_desc) / sizeof(uint64_t); i != iend; ++i)
		{
			// Each step spreads the word's bits over the whole hash
			hash = (hash ^ words[i]) * 0xFF51AFD7ED558CCDULL;
			hash ^= hash >> 32;
		}
		return hash;
	}

	PipeStateCache::PipeStateCache(Renderer& rdr)
		:m_rdr(&rdr)
		,m_entries()
		,m_requests()
		,m_in_flight()
		,m_stats()
		,m_mutex()
		,m_cv()
		,m_create_lock()
		,m_workers()
	{}

	// Find the entry for 'desc'
	PipeStateCache::Entry* PipeStateCache::Find(PipeStateDesc const& desc, uint64_t hash)
	{
		// The hash only narrows the search. Equal entries have an identical final state.
		for (auto [beg, end] = m_entries.equal_range(hash); beg != end; ++beg)
		{
			if (beg->second.m_desc == desc)
				return &beg->second;
		}
		return nullptr;
	}

	// Return the pipeline state object for 'desc', creating it if needed
	ID3D12PipelineState* PipeStateCache::Get(PipeStateDesc const& desc)
	{
		// Find an existing pipeline with an identical final state. Using a prewarmed pipeline ends its protection from trimming.
		auto const hash = desc.Hash();
		auto const now = clock_t::now();
		std::lock_guard<std::mutex> lock(m_mutex);
		if (auto* entry = Find(desc, hash); entry != nullptr)
		{
			entry->m_last_used = now;
			entry->m_pinned = false;
			return entry->m_pso.get();
		}

		// Release idle pipelines before adding another, so the cache does not grow without bound
		if (m_entries.size() >= TrimCount)
			Trim(now);

		// Create the pipeline state instance. This is the cost that prewarming moves off the drawing thread, so it is measured.
		auto pso = CreatePSO(m_rdr->D3DDevice(), desc);
		auto const ms = ElapsedMs(now);
		m_stats.m_created += 1;
		m_stats.m_created_ms += ms;
		m_stats.m_created_max_ms = std::max(m_stats.m_created_max_ms, ms);

		// Hold the root signature so that its address keeps identifying this entry
		auto const& d3d_desc = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
		auto& entry = m_entries.emplace(hash, Entry{
			.m_desc = desc,
			.m_signature = D3DPtr<ID3D12RootSignature>(d3d_desc.pRootSignature, true),
			.m_pso = pso,
			.m_last_used = now,
			.m_pinned = false,
		})->second;
		return entry.m_pso.get();
	}

	// Create pipelines for 'descs' on worker threads
	int PipeStateCache::Prewarm(std::span<PipeStateDesc const> descs)
	{
		// Queue the states that are not cached or already requested. The request holds the root signature until the pipeline is created.
		std::lock_guard<std::mutex> lock(m_mutex);
		auto added = 0;
		for (auto const& desc : descs)
		{
			auto const hash = desc.Hash();
			if (Find(desc, hash) != nullptr)
				continue;
			if (std::ranges::any_of(m_requests, [&](Request const& r) { return r.m_hash == hash && r.m_desc == desc; }))
				continue;

			auto const& d3d_desc = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
			m_requests.push_back(Request{ .m_desc = desc, .m_signature = D3DPtr<ID3D12RootSignature>(d3d_desc.pRootSignature, true), .m_hash = hash });
			++added;
		}

		// Start the workers on first use, so renderers that never prewarm have no threads. Pipeline creation is mostly CPU work in the driver,
		// so a few threads shorten a large batch without taking every core from the application.
		if (added != 0 && m_workers.empty())
		{
			auto const count = std::clamp(s_cast<int>(std::thread::hardware_concurrency()) / 2, 1, 4);
			for (int i = 0; i != count; ++i)
				m_workers.emplace_back([this](std::stop_token stop) { Worker(stop); });
		}

		m_cv.notify_all();
		return added;
	}

	// Create prewarm requests until stopped
	void PipeStateCache::Worker(std::stop_token stop)
	{
		for (;;)
		{
			// Wait for a request without holding the create lock, so that 'Forget' is not blocked by idle workers
			{
				std::unique_lock<std::mutex> lock(m_mutex);
				if (!m_cv.wait(lock, stop, [this] { return !m_requests.empty(); }))
					return;
			}

			// Take a request. The shared create lock keeps 'Forget' from releasing its bytecode or signature until the creation ends.
			std::shared_lock<std::shared_mutex> create(m_create_lock);
			Request request;
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				if (m_requests.empty())
					continue;

				request = std::move(m_requests.front());
				m_requests.pop_front();
				++m_in_flight;
			}

			// Create the pipeline outside 'm_mutex', so drawing threads are not blocked. Failures are counted, and 'Get' reports them if the state is drawn.
			auto const start = clock_t::now();
			D3DPtr<ID3D12PipelineState> pso;
			try
			{
				pso = CreatePSO(m_rdr->D3DDevice(), request.m_desc);
			}
			catch (std::exception const&)
			{
				pso = nullptr;
			}
			auto const ms = ElapsedMs(start);

			// Add the entry, unless 'Get' created the same state meanwhile. The duplicate pipeline is then released with 'pso'.
			std::lock_guard<std::mutex> lock(m_mutex);
			--m_in_flight;
			if (pso == nullptr)
			{
				m_stats.m_failed += 1;
				continue;
			}

			m_stats.m_prewarmed += 1;
			m_stats.m_prewarmed_ms += ms;
			if (Find(request.m_desc, request.m_hash) != nullptr)
				continue;

			m_entries.emplace(request.m_hash, Entry{
				.m_desc = request.m_desc,
				.m_signature = std::move(request.m_signature),
				.m_pso = pso,
				.m_last_used = clock_t::now(),
				.m_pinned = true,
			});
		}
	}

	// Release every pipeline that uses 'signature' or any shader stage starting at an address in 'bytecode'
	void PipeStateCache::Forget(ID3D12RootSignature const* signature, std::span<void const* const> bytecode)
	{
		// True if 'code' is one of the stages being forgotten
		auto forgotten = [=](D3D12_SHADER_BYTECODE const& code)
		{
			return code.pShaderBytecode != nullptr && std::ranges::find(bytecode, code.pShaderBytecode) != bytecode.end();
		};

		// Remove the entries and requests that refer to the forgotten objects. The exclusive create lock waits for creations in progress,
		// because a worker may be reading the forgotten bytecode or signature.
		auto doomed = [&](PipeStateDesc const& desc)
		{
			auto const& d = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
			return
				(signature != nullptr && d.pRootSignature == signature) ||
				forgotten(d.VS) || forgotten(d.PS) || forgotten(d.DS) || forgotten(d.HS) || forgotten(d.GS);
		};
		std::unique_lock<std::shared_mutex> create(m_create_lock);
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto iter = m_entries.begin(); iter != m_entries.end();)
		{
			auto& entry = iter->second;
			if (!doomed(entry.m_desc))
			{
				++iter;
				continue;
			}

			Release(entry);
			iter = m_entries.erase(iter);
		}
		std::erase_if(m_requests, [&](Request const& r) { return doomed(r.m_desc); });
	}

	// The number of cached pipeline state objects
	size_t PipeStateCache::Count() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_entries.size();
	}

	// The creation counters, and the number of prewarm requests not yet complete
	PipeStateCache::Stats PipeStateCache::GetStats() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto stats = m_stats;
		stats.m_pending = s_cast<int64_t>(m_requests.size()) + m_in_flight;
		stats.m_count = s_cast<int64_t>(m_entries.size());
		return stats;
	}

	// Release entries that have been idle for too long
	void PipeStateCache::Trim(clock_t::time_point now)
	{
		// Recently used entries are kept even if the cache stays large, because their states are still being drawn.
		// Prewarmed entries are kept until first used, because they were created for states that are about to be drawn.
		for (auto iter = m_entries.begin(); iter != m_entries.end();)
		{
			auto& entry = iter->second;
			if (entry.m_pinned || now - entry.m_last_used <= MaxIdleTime)
			{
				++iter;
				continue;
			}

			Release(entry);
			iter = m_entries.erase(iter);
		}
	}

	// Release one entry's pipeline once the GPU is no longer using it
	void PipeStateCache::Release(Entry& entry)
	{
		// Command lists that are recorded but not yet finished may still reference the pipeline and its root signature
		m_rdr->DeferRelease(entry.m_pso);
		m_rdr->DeferRelease(entry.m_signature);
	}
}