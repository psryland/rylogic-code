#pragma once
#include "src/forward.h"

namespace physics_sandbox::diag
{
	struct SceneDiagnosticOptions
	{
		std::filesystem::path m_scene_filepath;
		int m_steps = 600;
		double m_dt = 1.0 / 60.0;
		int m_report_interval = 60;
		int m_trace_body = -1;
		int m_trace_start = 0;
		int m_trace_end = 0x7fffffff;
		float m_trace_ke_jump = 1000.0f;
		std::optional<int> m_physics_substeps = {};
		std::optional<int> m_physics_solver_iterations = {};
		std::optional<int> m_physics_position_iterations = {};
		std::optional<float> m_physics_broadphase_aabb_margin = {};
		std::optional<float> m_physics_contact_sort_propagation_scale = {};
		std::optional<int> m_physics_contact_sort_shock_iterations = {};
		std::optional<float> m_physics_contact_slop_scale = {};
		std::optional<float> m_physics_support_contact_slop_scale = {};
		std::optional<float> m_physics_warm_start_scale = {};
		std::optional<int> m_physics_selective_refresh_passes = {};
		std::optional<int> m_physics_selective_refresh_max_pairs = {};
		std::optional<int> m_physics_selective_refresh_solver_iterations = {};
		std::optional<int> m_physics_selective_refresh_position_iterations = {};
		std::optional<float> m_physics_selective_refresh_bias_scale = {};
		std::optional<float> m_physics_selective_refresh_restitution_scale = {};
		std::optional<int> m_physics_selective_refresh_adaptive_body_limit = {};
		std::optional<int> m_physics_selective_refresh_adaptive_solver_iterations = {};
		std::optional<int> m_physics_selective_refresh_support_only = {};
		std::optional<int> m_physics_selective_refresh_resolve_support_only = {};
		bool m_scan_bodies = false;
		bool m_scan_non_spheres = false;
		bool m_column_metric = false;
		bool m_pyramid_metric = false;
		bool m_cradle_metric = false;
		bool m_engine_profile = false;
	};

	struct SceneDiagnosticResult
	{
		float m_max_depth = 0.0f;
		int m_max_depth_step = 0;
		int m_body_a = -1;
		int m_body_b = -1;
	};

	SceneDiagnosticResult RunSceneDiagnostic(SceneDiagnosticOptions const& options);
}
