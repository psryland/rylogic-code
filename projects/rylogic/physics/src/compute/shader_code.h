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
	extern ByteCode const seed_working_forces;
	extern ByteCode const integrate;

	// Gathered per-frame output
	extern ByteCode const prepare_substep_output;
	extern ByteCode const append_collision_events;
	extern ByteCode const gather_frame_bodies;
	extern ByteCode const gather_frame_articulations;

	// Sleep/wake state management
	extern ByteCode const disturb_islands;
	extern ByteCode const init_sleep_state;
	extern ByteCode const union_sleep_contacts;
	extern ByteCode const canonicalise_sleep_roots;
	extern ByteCode const reduce_sleep_stats;
	extern ByteCode const apply_sleep_state;

	// Broadphase sort-and-sweep
	extern ByteCode const sweep;
	extern ByteCode const sweep_filtered;
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

	// Persistent rigid constraints
	extern ByteCode const compile_constraints;
	extern ByteCode const assign_constraint_colours;
	extern ByteCode const apply_constraint_warm_start;
	extern ByteCode const clear_constraint_pseudo_velocity;
	extern ByteCode const solve_constraint_position;
	extern ByteCode const apply_constraint_position;
	extern ByteCode const solve_constraint_velocity;
	extern ByteCode const prepare_coupled_constraints;
	extern ByteCode const begin_coupled_velocity;
	extern ByteCode const build_coupled_velocity_candidates;
	extern ByteCode const gather_coupled_velocity_targets;
	extern ByteCode const select_coupled_velocity_trees;
	extern ByteCode const validate_coupled_velocity_trees;
	extern ByteCode const accept_coupled_velocity_islands;
	extern ByteCode const commit_coupled_velocity;
	extern ByteCode const finalize_coupled_velocity_islands;

	// Pure-tree articulation force ABA
	extern ByteCode const articulation_prepare;
	extern ByteCode const articulation_inward_dynamics;
	extern ByteCode const articulation_root_dynamics;
	extern ByteCode const articulation_outward_dynamics;
	extern ByteCode const articulation_midpoint;
	extern ByteCode const articulation_prepare_mobility;
	extern ByteCode const articulation_apply_impulses;
	extern ByteCode const articulation_evaluate_impulses;
	extern ByteCode const articulation_commit_impulses;
	extern ByteCode const articulation_gather_proxy_forces;
	extern ByteCode const articulation_refresh_proxies;

	// Selective refresh
	extern ByteCode const prepare_selective_refresh;
	extern ByteCode const score_selective_contacts;
	extern ByteCode const compact_selective_pairs;
	extern ByteCode const build_selective_dispatch;
}
