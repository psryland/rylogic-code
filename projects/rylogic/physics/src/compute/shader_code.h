//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::compute
{
	struct ByteCode;
}

namespace pr::physics::shader_code
{
	using ByteCode = ::pr::compute::ByteCode;

	// Integration
	extern ByteCode const integrate;

	// Sleep/wake state management
	extern ByteCode const disturb_islands;
	extern ByteCode const init_sleep_state;
	extern ByteCode const union_sleep_contacts;
	extern ByteCode const canonicalise_sleep_roots;
	extern ByteCode const reduce_sleep_stats;
	extern ByteCode const apply_sleep_state;

	// Broadphase sort-and-sweep
	extern ByteCode const sweep;
	extern ByteCode const calc_cd_dispatch;

	// Narrowphase collision detection
	extern ByteCode const clear_collision_bins;
	extern ByteCode const bin_collision_pairs;
	extern ByteCode const build_collision_bin_dispatch;
	extern ByteCode const collide_sphere_vs_sphere;
	extern ByteCode const collide_box_vs_sphere;
	extern ByteCode const collide_box_vs_box;
	extern ByteCode const collide_line_vs_sphere;
	extern ByteCode const collide_line_vs_box;
	extern ByteCode const collide_line_vs_line;
	extern ByteCode const collide_triangle_vs_sphere;
	extern ByteCode const collide_triangle_vs_box;
	extern ByteCode const collide_triangle_vs_line;
	extern ByteCode const collide_triangle_vs_triangle;
	extern ByteCode const collide_polytope_vs_sphere;
	extern ByteCode const collide_polytope_vs_box;
	extern ByteCode const collide_polytope_vs_line;
	extern ByteCode const collide_polytope_vs_triangle;
	extern ByteCode const collide_polytope_vs_polytope;
	extern ByteCode const calc_resolve_dispatch;

	// Collision resolution
	extern ByteCode const compute_collision_times;
	extern ByteCode const clear_shock_lists;
	extern ByteCode const seed_shock_priority;
	extern ByteCode const propagate_shock_priority;
	extern ByteCode const commit_shock_priority;
	extern ByteCode const finalize_shock_priority;
	extern ByteCode const assign_colours;
	extern ByteCode const warm_start_clear;
	extern ByteCode const apply_warm_start;
	extern ByteCode const store_warm_start;
	extern ByteCode const position_solve;
	extern ByteCode const resolve;

	// Selective refresh
	extern ByteCode const prepare_selective_refresh;
	extern ByteCode const score_selective_contacts;
	extern ByteCode const compact_selective_pairs;
	extern ByteCode const build_selective_dispatch;
}
