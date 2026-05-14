//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once

namespace pr::rdr12::shader_cache
{
	struct IShaderCache;
}

namespace pr::physics
{
	struct EngineConfig
	{
		// Optional runtime shader cache used by GPU compute shader compilation.
		rdr12::shader_cache::IShaderCache* shader_cache = nullptr;

		// Maximum number of collision pairs that the engine can handle per frame.
		int max_collision_pairs = 65536;
		
		// Sleep thresholds: if a body has linear velocity below 'sleep_velocity_threshold_lin' and
		// angular velocity below 'sleep_velocity_threshold_ang' for some time, it can be put to sleep.
		// These thresholds are in world units (e.g., m/s for linear, rad/s for angular).
		bool sleeping_enabled = true;
		float sleep_velocity_threshold_lin = 0.25f;
		float sleep_velocity_threshold_ang = 1.50f;
		float sleep_delay_s = 1.0f;

		// Number of coloured Gauss-Seidel contact solver iterations.
		int solver_iterations = 8;

		// Number of iterations that push-out is distributed over. Each coloured group pushes
		// apart a small amount each iteration so that push out is spread out more evenly.
		int push_out_iterations = 4;

		// Bias contact ordering within near-equal collision-time buckets so impulses propagate
		// through contact dependency chains before downstream/free contacts are solved.
		float broadphase_aabb_margin = 0.0001f;

		// Contact sorting parameters. Contacts are sorted by time of impact (TOI) into discrete buckets,
		// then solved in sorted order. This allows impulses to propagate through long chains of contacts
		float contact_sort_propagation_scale = 1e-3f;
		int contact_sort_shock_iterations = 4;
		float contact_sort_shock_alignment = 1e-5f;
		float contact_sort_shock_min_strength = 1e-5f;
		float contact_sort_shock_decay = 0.98f;

		// Velocity-level Baumgarte bias for shallow penetrations.
		float penetration_slop = 0.005f;
		float velocity_baumgarte = 0.2f;

		// Position-level split correction. This moves bodies only; momenta are unchanged.
		float position_slop = 0.005f;
		float position_baumgarte = 0.2f;
		float contact_slop_scale = 0.01f;

		// Support contacts use a lower proportional slop so thin stacks do not accumulate
		// visible compression, while non-support impacts keep the more forgiving default.
		float support_contact_slop_scale = 0.005f;
		float warm_start_scale = 0.90f;

		// Existing deep-penetration positional correction parameters.
		float deep_penetration_threshold = 0.3f;
		float deep_penetration_range = 0.4f;
		float deep_penetration_baumgarte_min = 0.2f;
		float deep_penetration_baumgarte_max = 0.8f;

		// Selective contact refresh runs extra narrowphase/resolve passes over a compacted
		// subset of problematic pairs after the full contact graph has been resolved.
		int selective_refresh_passes = 1;
		int selective_refresh_max_pairs = 512;
		int selective_refresh_body_limit = 256;
		int selective_refresh_contact_limit = 512;
		int selective_refresh_solver_iterations = 12;
		int selective_refresh_position_iterations = 1;
		float selective_refresh_bias_scale = 1.0f;
		float selective_refresh_restitution_scale = 0.0f;

		// Small/medium scenes can afford a stiffer residual solve without making large stress
		// scenes pay the same cost for every selected support contact.
		int selective_refresh_adaptive_body_limit = 256;
		int selective_refresh_adaptive_solver_iterations = 48;

		// Seed selective refresh from support contacts by default, but resolve the refreshed local
		// contact neighbourhood so lateral/friction constraints can still converge with the stack.
		bool selective_refresh_support_only = true;
		bool selective_refresh_resolve_support_only = false;
		float selective_refresh_depth_slop = 0.015f;
		float selective_refresh_support_depth_slop = 0.002f;
		float selective_refresh_closing_speed_slop = 0.02f;
		float selective_refresh_support_alignment = 0.65f;
		float selective_refresh_aabb_margin = 0.03f;
	};
}
