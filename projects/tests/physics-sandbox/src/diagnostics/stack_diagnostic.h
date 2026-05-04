#pragma once
#include "src/forward.h"

namespace physics_sandbox::diag
{
	struct StackDiagnosticOptions
	{
		std::filesystem::path m_scene_filepath;
		int m_steps = 600;
		double m_dt = 1.0 / 60.0;
		int m_report_interval = 60;
		int m_trace_body = -1;
		int m_trace_start = 0;
		int m_trace_end = 0x7fffffff;
		float m_trace_ke_jump = 1000.0f;
		bool m_scan_bodies = false;
	};

	struct StackDiagnosticResult
	{
		float m_max_depth = 0.0f;
		int m_max_depth_step = 0;
		int m_body_a = -1;
		int m_body_b = -1;
	};

	StackDiagnosticResult RunStackDiagnostic(StackDiagnosticOptions const& options);
}
