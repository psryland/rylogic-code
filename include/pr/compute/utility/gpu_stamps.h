//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Diagnostic GPU timestamp profiler for named command-list regions.
// Set 'PR_GPU_STAMPS=<file path>' before startup to enable it. Each module that records regions writes '<file path>.<module name>.txt',
// which lists the GPU duration of every region name (inclusive of nested regions), refreshed about every two seconds.
// Regions are the same begin/end pairs that are sent to PIX, see 'pix.h'. When disabled, the cost is one cached flag test per region.
#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <windows.h>
#include <d3d12.h>

namespace pr::compute::gpu_stamps
{
	// The profiler state for one module. Timestamp values are read some time after recording, because the profiler cannot see queue fences.
	struct Profiler
	{
		// The ring must be much larger than the read lag, and the read lag must cover all work the GPU could still be executing.
		static constexpr uint64_t Slots = 1 << 17;
		static constexpr uint64_t Lag = 1 << 14;

		// One timestamp slot. A begin slot records the absolute index of its end slot once the region closes.
		struct Slot
		{
			std::string m_name;
			uint64_t m_end;
			bool m_is_begin;
		};

		// Accumulated region durations.
		struct Total
		{
			double m_ticks;
			uint64_t m_count;
		};

		std::mutex m_mutex;
		std::string m_path;
		ID3D12QueryHeap* m_heap;
		ID3D12Resource* m_readback;
		uint64_t const* m_values;
		double m_freq;
		std::vector<Slot> m_slots;
		std::unordered_map<void*, std::vector<uint64_t>> m_open; // Open begin slots per command list
		std::unordered_map<std::string, Total> m_totals;
		uint64_t m_cursor;
		uint64_t m_harvested;
		std::chrono::steady_clock::time_point m_last_write;
		std::chrono::steady_clock::time_point m_start;

		Profiler()
			: m_mutex()
			, m_path()
			, m_heap()
			, m_readback()
			, m_values()
			, m_freq()
			, m_slots(Slots)
			, m_open()
			, m_totals()
			, m_cursor()
			, m_harvested()
			, m_last_write(std::chrono::steady_clock::now())
			, m_start(std::chrono::steady_clock::now())
		{
			// Read the output path, and suffix it with this module's name because each module has its own profiler instance.
			char path[MAX_PATH] = {};
			auto len = GetEnvironmentVariableA("PR_GPU_STAMPS", path, sizeof(path));
			if (len == 0 || len >= sizeof(path))
				return;

			HMODULE module = nullptr;
			char module_path[MAX_PATH] = {};
			GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<char const*>(&Instance), &module);
			GetModuleFileNameA(module, module_path, sizeof(module_path));
			auto const* module_name = strrchr(module_path, '\\');
			m_path = std::string(path) + "." + (module_name ? module_name + 1 : "module") + ".txt";
		}

		// The profiler for this module.
		static Profiler& Instance()
		{
			static Profiler instance;
			return instance;
		}

		// True when the profiler was requested at startup.
		bool Enabled() const
		{
			return !m_path.empty();
		}

		// Create the query heap and readback buffer on first use, using the device that owns the command list.
		bool Initialise(ID3D12GraphicsCommandList* list)
		{
			// Device objects are created lazily because the profiler has no other access to the device.
			if (m_heap != nullptr)
				return true;

			ID3D12Device* device = nullptr;
			if (FAILED(list->GetDevice(__uuidof(ID3D12Device), reinterpret_cast<void**>(&device))))
				return false;

			// Timestamp frequency is a queue property. Direct and compute queues share the GPU clock on supported hardware.
			{
				D3D12_COMMAND_QUEUE_DESC qdesc = {.Type = D3D12_COMMAND_LIST_TYPE_DIRECT};
				ID3D12CommandQueue* queue = nullptr;
				if (SUCCEEDED(device->CreateCommandQueue(&qdesc, __uuidof(ID3D12CommandQueue), reinterpret_cast<void**>(&queue))))
				{
					UINT64 freq = 0;
					queue->GetTimestampFrequency(&freq);
					m_freq = static_cast<double>(freq);
					queue->Release();
				}
			}

			// The query heap holds the ring of timestamps; each one is resolved into the persistently mapped readback buffer at the same slot.
			D3D12_QUERY_HEAP_DESC hdesc = {.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP, .Count = static_cast<UINT>(Slots)};
			auto ok = SUCCEEDED(device->CreateQueryHeap(&hdesc, __uuidof(ID3D12QueryHeap), reinterpret_cast<void**>(&m_heap)));

			D3D12_HEAP_PROPERTIES heap_props = {.Type = D3D12_HEAP_TYPE_READBACK};
			D3D12_RESOURCE_DESC rdesc = {
				.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
				.Width = Slots * sizeof(uint64_t),
				.Height = 1,
				.DepthOrArraySize = 1,
				.MipLevels = 1,
				.SampleDesc = {.Count = 1},
				.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
			};
			ok = ok && SUCCEEDED(device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &rdesc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, __uuidof(ID3D12Resource), reinterpret_cast<void**>(&m_readback)));

			void* mapped = nullptr;
			ok = ok && SUCCEEDED(m_readback->Map(0, nullptr, &mapped));
			m_values = static_cast<uint64_t const*>(mapped);
			device->Release();
			if (!ok)
				m_path.clear();

			return ok;
		}

		// Write a timestamp into the next ring slot and return its absolute index.
		uint64_t Stamp(ID3D12GraphicsCommandList* list, std::string name, bool is_begin)
		{
			// Resolve each stamp immediately so the profiler needs no knowledge of when command lists are closed or executed.
			auto idx = m_cursor++;
			auto slot = static_cast<UINT>(idx % Slots);
			list->EndQuery(m_heap, D3D12_QUERY_TYPE_TIMESTAMP, slot);
			list->ResolveQueryData(m_heap, D3D12_QUERY_TYPE_TIMESTAMP, slot, 1, m_readback, slot * sizeof(uint64_t));
			m_slots[slot] = Slot{.m_name = std::move(name), .m_end = ~0ULL, .m_is_begin = is_begin};
			return idx;
		}

		// Open a region on 'list'.
		void Begin(ID3D12GraphicsCommandList* list, std::string name)
		{
			// Copy lists cannot be assumed to support timestamps.
			if (list->GetType() == D3D12_COMMAND_LIST_TYPE_COPY)
				return;

			std::lock_guard<std::mutex> lock(m_mutex);
			if (!Initialise(list))
				return;

			m_open[list].push_back(Stamp(list, std::move(name), true));
		}

		// Close the most recent region on 'list'.
		void End(ID3D12GraphicsCommandList* list)
		{
			// Ignore ends whose begin was not stamped, such as regions on copy lists.
			std::lock_guard<std::mutex> lock(m_mutex);
			auto it = m_open.find(list);
			if (it == m_open.end() || it->second.empty())
				return;

			auto begin = it->second.back();
			it->second.pop_back();
			auto end = Stamp(list, {}, false);
			m_slots[begin % Slots].m_end = end;
			Harvest();
		}

		// Accumulate regions whose timestamps are old enough to have been written by the GPU, then periodically rewrite the report.
		void Harvest()
		{
			// Stop at the first region that is still open or too recent, so regions are consumed in order.
			for (; m_harvested + Lag < m_cursor; ++m_harvested)
			{
				auto& slot = m_slots[m_harvested % Slots];
				if (!slot.m_is_begin)
					continue;

				// A region left open for most of the ring's length is abandoned, so it cannot block the reader.
				if (slot.m_end == ~0ULL)
				{
					if (m_harvested + Slots / 2 < m_cursor)
						continue;

					break;
				}
				if (slot.m_end + Lag >= m_cursor)
					break;

				auto t0 = m_values[m_harvested % Slots];
				auto t1 = m_values[slot.m_end % Slots];
				auto& total = m_totals[slot.m_name];
				total.m_ticks += t1 >= t0 ? static_cast<double>(t1 - t0) : 0.0;
				total.m_count += 1;
			}

			auto now = std::chrono::steady_clock::now();
			if (now - m_last_write > std::chrono::seconds(2))
			{
				m_last_write = now;
				Write(std::chrono::duration<double>(now - m_start).count());
			}
		}

		// Rewrite the report, sorted by total GPU time.
		void Write(double elapsed_s) const
		{
			// Durations are inclusive of nested regions; 'per_s' divides by wall time since the profiler started.
			std::vector<std::pair<std::string, Total>> rows(m_totals.begin(), m_totals.end());
			std::sort(rows.begin(), rows.end(), [](auto const& a, auto const& b) { return a.second.m_ticks > b.second.m_ticks; });

			FILE* file = nullptr;
			if (fopen_s(&file, m_path.c_str(), "w") != 0 || file == nullptr)
				return;

			fprintf(file, "elapsed_s=%.3f freq=%.0f\n%-48s %10s %12s %12s %10s\n", elapsed_s, m_freq, "name", "count", "total_ms", "mean_us", "ms_per_s");
			for (auto const& [name, total] : rows)
			{
				auto ms = m_freq > 0 ? total.m_ticks * 1000.0 / m_freq : 0.0;
				fprintf(file, "%-48s %10llu %12.3f %12.3f %10.3f\n", name.c_str(), static_cast<unsigned long long>(total.m_count), ms, 1000.0 * ms / static_cast<double>(total.m_count), ms / elapsed_s);
			}
			fclose(file);
		}
	};

	// True when 'PR_GPU_STAMPS' was set at startup.
	inline bool Enabled()
	{
		static bool const enabled = Profiler::Instance().Enabled();
		return enabled;
	}

	// Open a named region on a graphics or compute command list. Other contexts are ignored.
	template <typename CONTEXT, typename... ARGS>
	inline void Begin(CONTEXT* context, char const* format_string, ARGS... args)
	{
		// Region names are formatted only when profiling is enabled.
		if constexpr (std::is_convertible_v<CONTEXT*, ID3D12GraphicsCommandList*>)
		{
			if (!Enabled() || context == nullptr)
				return;

			std::string name;
			if constexpr (sizeof...(ARGS) == 0)
			{
				name = format_string;
			}
			else
			{
				char buf[256];
				snprintf(buf, sizeof(buf), format_string, args...);
				name = buf;
			}
			Profiler::Instance().Begin(context, std::move(name));
		}
		else
		{
			(void)context, (void)format_string, ((void)args, ...);
		}
	}

	// Close the most recent region on 'context'.
	template <typename CONTEXT>
	inline void End(CONTEXT* context)
	{
		// Only graphics and compute command lists are stamped.
		if constexpr (std::is_convertible_v<CONTEXT*, ID3D12GraphicsCommandList*>)
		{
			if (Enabled() && context != nullptr)
				Profiler::Instance().End(context);
		}
		else
		{
			(void)context;
		}
	}
}
