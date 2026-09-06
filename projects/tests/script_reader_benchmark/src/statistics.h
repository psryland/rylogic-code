//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Shared robust statistics for the standalone Reader and LDraw adapter benchmark executables.
#pragma once
#include "forward.h"

namespace pr::script_bench
{
	// Robust summary statistics over a set of elapsed-time samples.
	struct Stats
	{
		double m_median;
		double m_p10;
		double m_p90;
		double m_mean;
		double m_stddev;
		double m_cv;
	};

	// Compute interpolated percentiles and distribution statistics for a non-empty sample set.
	inline Stats ComputeStats(std::vector<double> values)
	{
		std::sort(values.begin(), values.end());
		auto count = values.size();

		// Interpolate between the nearest order statistics for non-integral percentile positions.
		auto percentile = [&](double fraction)
		{
			auto index = fraction * double(count - 1);
			auto lower = size_t(std::floor(index));
			auto upper = size_t(std::ceil(index));
			auto weight = index - double(lower);
			return values[lower] + (values[upper] - values[lower]) * weight;
		};

		// Derive population variance because the samples are the complete measured run.
		auto mean = std::accumulate(values.begin(), values.end(), 0.0) / double(count);
		auto variance = 0.0;
		for (auto value : values)
			variance += (value - mean) * (value - mean);
		variance /= double(count);

		auto stddev = std::sqrt(variance);
		return Stats
		{
			.m_median = percentile(0.5),
			.m_p10 = percentile(0.10),
			.m_p90 = percentile(0.90),
			.m_mean = mean,
			.m_stddev = stddev,
			.m_cv = mean != 0.0 ? stddev / mean : 0.0,
		};
	}
}
