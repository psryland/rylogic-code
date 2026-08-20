#pragma once
#include <concepts>
#include <type_traits>
#include <span>
#include <limits>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cassert>

namespace pr::algorithm::peak_detection
{
	// Configuration for peak detection
	template <std::floating_point T>
	struct Config
	{
		// Peaks with persistence less than 'threshold * max_persistence' are discarded.
		// Range: [0, 1]. Default: 0.05 (ignore peaks less than 5% of the most prominent peak).
		T threshold = T(0.05);
	};

	// Detect peaks in 'data' using persistent homology.
	// Peaks are returned as indices into 'data' via 'out', sorted by persistence (most prominent first).
	//
	// How it works (persistent homology for 1D signals):
	//
	//   Imagine the signal as a landscape viewed from above, and a "water level" that starts above the
	//   highest point and gradually lowers. As the water drops:
	//
	//   - When the water level passes a local maximum, a new "island" appears. This is the peak's BIRTH.
	//     The birth value is the height of the local maximum.
	//
	//   - When the water level drops further and two islands merge (at a local minimum between them),
	//     one island is absorbed into the other. The smaller island DIES. The death value is the height
	//     of the saddle point (local minimum) where they merge.
	//
	//   - A peak's PERSISTENCE = birth - death. This measures how prominent the peak is: a tall, isolated
	//     peak has high persistence, while a tiny noise bump has low persistence.
	//
	//   - The global maximum never dies (infinite persistence) and is always the most prominent peak.
	//
	//   Implementation: We sort all sample indices by descending value. Processing them in this order
	//   simulates lowering the water level. A union-find (disjoint set) structure tracks which islands
	//   are connected. When an activated sample has an already-activated neighbour in a different component,
	//   the component whose representative has the lower birth value dies, and its persistence is recorded.
	//
	template <std::floating_point T, std::invocable<int> Out>
	void DetectPeaks(std::span<T const> data, Config<T> const& config, Out out)
	{
		auto const n = static_cast<int>(data.size());
		if (n == 0)
			return;

		// Union-Find (disjoint set) with path compression and union by rank.
		// Each component tracks the index of the sample that "birthed" it (i.e., its local maximum).
		struct UnionFind
		{
			std::vector<int> m_parent;
			std::vector<int> m_rank;
			std::vector<int> m_birth; // Index of the local maximum that represents this component

			UnionFind(int n)
				: m_parent(n)
				, m_rank(n, 0)
				, m_birth(n)
			{
				std::iota(m_parent.begin(), m_parent.end(), 0);
				std::iota(m_birth.begin(), m_birth.end(), 0);
			}

			int Find(int x)
			{
				// Path compression: make every node point directly to the root
				while (m_parent[x] != x)
					x = m_parent[x] = m_parent[m_parent[x]];
				return x;
			}

			// Union two components. Returns the root of the component that dies (the one with the lower birth value),
			// or -1 if they are already in the same component.
			int Union(int a, int b)
			{
				a = Find(a);
				b = Find(b);
				if (a == b)
					return -1;

				// The component whose birth index has the higher data value survives.
				// We need the caller to tell us which birth is "higher", but since birth indices
				// are set at creation and we always keep the higher-birth as the surviving root,
				// we swap so that 'a' is the survivor (higher birth value).
				// Note: the caller ensures this by passing (survivor, dying) but we also
				// handle it via union by rank after selecting the survivor.

				// Union by rank for tree balance
				if (m_rank[a] < m_rank[b])
					std::swap(a, b);

				m_parent[b] = a;
				if (m_rank[a] == m_rank[b])
					++m_rank[a];

				// The surviving component keeps the birth index with the higher data value
				// (this is maintained by the caller selecting which root survives)
				return b;
			}
		};

		// Step 1: Sort indices by descending data value. This simulates lowering the water level
		// from the highest point down to the lowest.
		auto order = std::vector<int>(n);
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](int a, int b) { return data[a] > data[b]; });

		// Step 2: Process indices in descending value order, tracking component merges
		UnionFind uf(n);
		auto activated = std::vector<bool>(n, false);

		struct Peak
		{
			int index;        // Index into data[] where the peak is
			T persistence;    // How prominent this peak is (birth_value - death_value)
		};
		auto peaks = std::vector<Peak>();

		for (auto i : order)
		{
			// "Activate" this sample — an island appears or grows
			activated[i] = true;

			// Check left and right neighbours. If a neighbour is already activated and belongs
			// to a different component, the two islands are merging at this value (the saddle point).
			for (auto j : { i - 1, i + 1 })
			{
				if (j < 0 || j >= n || !activated[j])
					continue;

				auto root_i = uf.Find(i);
				auto root_j = uf.Find(j);
				if (root_i == root_j)
					continue;

				// Two distinct components are merging. The component with the lower birth value dies.
				// The current data[i] value is the saddle point where they meet (since we process
				// in descending order, data[i] is the highest unprocessed value = the merge level).
				auto birth_i = uf.m_birth[root_i];
				auto birth_j = uf.m_birth[root_j];

				// Determine which component dies: the one born at the lower value
				int dying_root, surviving_root;
				if (data[birth_i] >= data[birth_j])
				{
					surviving_root = root_i;
					dying_root = root_j;
				}
				else
				{
					surviving_root = root_j;
					dying_root = root_i;
				}

				// Record the dying peak's persistence
				auto dying_birth_idx = uf.m_birth[dying_root];
				auto persistence = data[dying_birth_idx] - data[i];
				peaks.push_back({ dying_birth_idx, persistence });

				// Merge: attach the dying component to the surviving one
				uf.m_parent[uf.Find(dying_root)] = uf.Find(surviving_root);

				// Ensure the surviving root retains the birth index of the component with the higher birth value
				auto new_root = uf.Find(surviving_root);
				uf.m_birth[new_root] = uf.m_birth[surviving_root];
			}
		}

		// The last surviving component is the global maximum — it has "infinite" persistence.
		// Find which birth index it has and add it as the most prominent peak.
		auto global_root = uf.Find(order[0]);
		peaks.push_back({ uf.m_birth[global_root], std::numeric_limits<T>::max() });

		// Step 3: Filter by threshold. The threshold is a fraction of the maximum finite persistence.
		auto const infinite_persistence = std::numeric_limits<T>::max();
		auto max_persistence = T(0);
		for (auto& p : peaks)
		{
			if (p.persistence < infinite_persistence)
				max_persistence = std::max(max_persistence, p.persistence);
		}
		auto cutoff = config.threshold * max_persistence;

		// Remove thresholded peaks. Zero-persistence merges are not peaks; they are samples that are
		// absorbed at their birth height, which happens along monotone edges and equal-height plateaus.
		std::erase_if(peaks, [cutoff, infinite_persistence](auto const& p)
		{
			if (p.persistence == infinite_persistence)
				return false;

			if (p.persistence <= T(0))
				return true;

			return p.persistence < cutoff;
		});

		// Step 4: Sort remaining peaks by persistence (most prominent first)
		std::sort(peaks.begin(), peaks.end(), [](auto const& a, auto const& b) { return a.persistence > b.persistence; });

		// Step 5: Output peak indices
		for (auto& p : peaks)
			out(p.index);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/common/ldraw.h"
namespace pr::algorithm
{
	PRUnitTestClass(PeakDetectionTests)
	{
		// Generate a signal with Gaussian bumps at specified centres and amplitudes on a flat baseline
		struct Bump { int center; float amplitude; };
		static std::vector<float> GenerateBumpSignal(int count, std::span<Bump const> bumps, float sigma = 15.0f)
		{
			auto data = std::vector<float>(count, 0.0f);
			auto sigma_sq2 = 2.0f * sigma * sigma;
			for (auto& bump : bumps)
				for (int i = 0; i != count; ++i)
					data[i] += bump.amplitude * std::exp(-float((i - bump.center) * (i - bump.center)) / sigma_sq2);
			return data;
		}

		PRUnitTestMethod(DetectPeaks_Empty)
		{
			// Empty data should produce no peaks
			std::vector<float> empty;
			std::vector<int> peaks;
			peak_detection::DetectPeaks<float>(empty, {}, [&](int idx) { peaks.push_back(idx); });
			PR_EXPECT(peaks.empty());
		}

		PRUnitTestMethod(DetectPeaks_SingleValue)
		{
			// A single value is the global max, so it should be returned
			std::vector<float> single = { 42.0f };
			std::vector<int> peaks;
			peak_detection::DetectPeaks<float>(single, {}, [&](int idx) { peaks.push_back(idx); });
			PR_EXPECT(peaks.size() == 1);
			PR_EXPECT(peaks[0] == 0);
		}

		PRUnitTestMethod(DetectPeaks_ZeroPersistenceSequences)
		{
			// Sequences without finite-prominence peaks should return only the surviving global maximum.
			auto expect_one_peak_with_value = [](std::vector<float> const& data, float peak_value)
			{
				auto peaks = std::vector<int>();
				peak_detection::DetectPeaks<float>(data, {}, [&](int idx) { peaks.push_back(idx); });

				PR_EXPECT(peaks.size() == 1);
				PR_EXPECT(data[peaks[0]] == peak_value);
			};

			expect_one_peak_with_value({ 0.0f, 0.0f, 0.0f, 0.0f }, 0.0f);
			expect_one_peak_with_value({ 0.0f, 1.0f, 2.0f, 3.0f }, 3.0f);
			expect_one_peak_with_value({ 3.0f, 2.0f, 1.0f, 0.0f }, 3.0f);
			expect_one_peak_with_value({ 0.0f, 1.0f, 1.0f, 0.0f }, 1.0f);
			expect_one_peak_with_value({ 0.0f, 1.0f, 0.0f }, 1.0f);
		}

		PRUnitTestMethod(DetectPeaks_KnownPeaks)
		{
			// Hand-crafted signal: flat baseline at 0, with Gaussian bumps at known locations.
			// Bump amplitudes: idx=100 -> 10, idx=300 -> 5, idx=500 -> 8, idx=700 -> 3
			// Expected persistence order: 100 (amp 10), 500 (amp 8), 300 (amp 5), 700 (amp 3)
			Bump bumps[] = { {100, 10.0f}, {300, 5.0f}, {500, 8.0f}, {700, 3.0f} };
			auto data = GenerateBumpSignal(1000, bumps);

			// Detect with a low threshold to find all 4 bumps
			peak_detection::Config<float> config;
			config.threshold = 0.01f;

			auto peaks = std::vector<int>();
			peak_detection::DetectPeaks<float>(data, config, [&](int idx) { peaks.push_back(idx); });

			// Should find at least 4 peaks (there may also be the global max and minor noise peaks)
			PR_EXPECT(peaks.size() >= 4);

			// The first 4 peaks should be near our known bump centres, in persistence order.
			// Allow ±2 samples of tolerance for the peak location.
			PR_EXPECT(std::abs(peaks[0] - 100) <= 2); // Amplitude 10 — most prominent
			PR_EXPECT(std::abs(peaks[1] - 500) <= 2); // Amplitude 8
			PR_EXPECT(std::abs(peaks[2] - 300) <= 2); // Amplitude 5
			PR_EXPECT(std::abs(peaks[3] - 700) <= 2); // Amplitude 3
		}

		PRUnitTestMethod(DetectPeaks_SineWave)
		{
			// A simple sine wave with 5 full periods should have 5 peaks
			constexpr int N = 1000;
			constexpr int periods = 5;
			auto data = std::vector<float>(N);
			for (int i = 0; i != N; ++i)
				data[i] = std::sin(2.0f * 3.14159265f * periods * i / N);

			peak_detection::Config<float> config;
			config.threshold = 0.1f;

			auto peaks = std::vector<int>();
			peak_detection::DetectPeaks<float>(data, config, [&](int idx) { peaks.push_back(idx); });

			// Should find 5 positive peaks and 5 negative peaks = 10 total, plus noise is filtered.
			// At minimum, we should find the 5 positive peaks (highest persistence).
			PR_EXPECT(peaks.size() >= 5);
		}

		PRUnitTestMethod(DetectPeaks_ThresholdFiltering)
		{
			// Higher threshold should produce fewer peaks
			Bump bumps[] = { {200, 10.0f}, {500, 3.0f}, {800, 1.0f} };
			auto data = GenerateBumpSignal(1000, bumps);

			// Low threshold: should find all bumps
			auto peaks_low = std::vector<int>();
			peak_detection::Config<float> cfg_low;
			cfg_low.threshold = 0.01f;
			peak_detection::DetectPeaks<float>(data, cfg_low, [&](int idx) { peaks_low.push_back(idx); });

			// High threshold: should find fewer bumps
			auto peaks_high = std::vector<int>();
			peak_detection::Config<float> cfg_high;
			cfg_high.threshold = 0.5f;
			peak_detection::DetectPeaks<float>(data, cfg_high, [&](int idx) { peaks_high.push_back(idx); });

			PR_EXPECT(peaks_high.size() < peaks_low.size());
		}

		PRUnitTestMethodFamily(DetectPeaks_Visualise, Slow)
		{
			// Generate a multi-frequency signal for visualisation
			constexpr int N = 1000;
			auto data = std::vector<float>(N);
			for (int i = 0; i != N; ++i)
			{
				auto t = static_cast<float>(i);
				data[i] = 10.0f * std::cos(0.01f * t) + 3.0f * std::sin(0.07f * t) + 1.5f * std::cos(0.2f * t);
			}

			peak_detection::Config<float> config;
			config.threshold = 0.05f;

			auto peaks = std::vector<int>();
			peak_detection::DetectPeaks<float>(data, config, [&](int idx) { peaks.push_back(idx); });

			// Build an ldraw visualisation of the signal and detected peaks
			ldraw::Builder builder;

			// Graph the signal as a line strip
			auto& signal_line = builder.Line("signal", 0xFF00FF00);
			for (int i = 0; i != N; ++i)
				signal_line.line_to({ static_cast<float>(i), data[i], 0.0f });

			// Overlay detected peaks as red points
			auto& peak_pts = builder.Point("peaks", 0xFFFF0000).size(10.0f);
			for (auto idx : peaks)
				peak_pts.pt(static_cast<float>(idx), data[idx], 0.0f);

			builder.Save(temp_dir() / "PeakDetection.ldr");
		}
	};
}
#endif
