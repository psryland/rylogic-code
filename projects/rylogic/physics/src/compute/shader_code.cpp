//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/shader_code.h"
#include "pr/compute/utility/wrappers.h"

#ifdef NDEBUG
#define PR_PHYSICS_SHADER_COMPILED_DIR(file) PR_STRINGISE(physics/src/compute/compiled/release/##file)
#else
#define PR_PHYSICS_SHADER_COMPILED_DIR(file) PR_STRINGISE(physics/src/compute/compiled/debug/##file)
#endif

namespace pr::physics::shader_code
{
	// Integration
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(seed_working_forces_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(integrate_cs.h)
	}
	ByteCode const seed_working_forces(compiled::seed_working_forces_cs);
	ByteCode const integrate(compiled::integrate_cs);

	// Gathered per-frame output
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(prepare_substep_output_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(append_collision_events_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(gather_frame_bodies_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(gather_frame_articulations_cs.h)
	}
	ByteCode const prepare_substep_output(compiled::prepare_substep_output_cs);
	ByteCode const append_collision_events(compiled::append_collision_events_cs);
	ByteCode const gather_frame_bodies(compiled::gather_frame_bodies_cs);
	ByteCode const gather_frame_articulations(compiled::gather_frame_articulations_cs);

	// Sleep/wake state management
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(disturb_islands_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(init_sleep_state_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(union_sleep_contacts_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(canonicalise_sleep_roots_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(reduce_sleep_stats_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(apply_sleep_state_cs.h)
	}
	ByteCode const disturb_islands(compiled::disturb_islands_cs);
	ByteCode const init_sleep_state(compiled::init_sleep_state_cs);
	ByteCode const union_sleep_contacts(compiled::union_sleep_contacts_cs);
	ByteCode const canonicalise_sleep_roots(compiled::canonicalise_sleep_roots_cs);
	ByteCode const reduce_sleep_stats(compiled::reduce_sleep_stats_cs);
	ByteCode const apply_sleep_state(compiled::apply_sleep_state_cs);

	// Broadphase sort-and-sweep
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(sweep_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(sweep_filtered_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(calc_cd_dispatch_cs.h)
	}
	ByteCode const sweep(compiled::sweep_cs);
	ByteCode const sweep_filtered(compiled::sweep_filtered_cs);
	ByteCode const calc_cd_dispatch(compiled::calc_cd_dispatch_cs);

	// Narrowphase collision detection
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(clear_collision_bins_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(bin_collision_pairs_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(build_collision_bin_dispatch_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_sphere_vs_sphere_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_box_vs_sphere_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_box_vs_box_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_line_vs_sphere_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_line_vs_box_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_line_vs_line_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_triangle_vs_sphere_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_triangle_vs_box_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_triangle_vs_line_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_triangle_vs_triangle_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_polytope_vs_sphere_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_polytope_vs_box_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_polytope_vs_line_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_polytope_vs_triangle_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(collide_polytope_vs_polytope_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(calc_resolve_dispatch_cs.h)
	}
	ByteCode const clear_collision_bins(compiled::clear_collision_bins_cs);
	ByteCode const bin_collision_pairs(compiled::bin_collision_pairs_cs);
	ByteCode const build_collision_bin_dispatch(compiled::build_collision_bin_dispatch_cs);
	ByteCode const collide_sphere_vs_sphere(compiled::collide_sphere_vs_sphere_cs);
	ByteCode const collide_box_vs_sphere(compiled::collide_box_vs_sphere_cs);
	ByteCode const collide_box_vs_box(compiled::collide_box_vs_box_cs);
	ByteCode const collide_line_vs_sphere(compiled::collide_line_vs_sphere_cs);
	ByteCode const collide_line_vs_box(compiled::collide_line_vs_box_cs);
	ByteCode const collide_line_vs_line(compiled::collide_line_vs_line_cs);
	ByteCode const collide_triangle_vs_sphere(compiled::collide_triangle_vs_sphere_cs);
	ByteCode const collide_triangle_vs_box(compiled::collide_triangle_vs_box_cs);
	ByteCode const collide_triangle_vs_line(compiled::collide_triangle_vs_line_cs);
	ByteCode const collide_triangle_vs_triangle(compiled::collide_triangle_vs_triangle_cs);
	ByteCode const collide_polytope_vs_sphere(compiled::collide_polytope_vs_sphere_cs);
	ByteCode const collide_polytope_vs_box(compiled::collide_polytope_vs_box_cs);
	ByteCode const collide_polytope_vs_line(compiled::collide_polytope_vs_line_cs);
	ByteCode const collide_polytope_vs_triangle(compiled::collide_polytope_vs_triangle_cs);
	ByteCode const collide_polytope_vs_polytope(compiled::collide_polytope_vs_polytope_cs);
	ByteCode const calc_resolve_dispatch(compiled::calc_resolve_dispatch_cs);

	// Collision resolution
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(compute_collision_times_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(clear_shock_lists_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(seed_shock_priority_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(propagate_shock_priority_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(commit_shock_priority_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(finalize_shock_priority_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(assign_colours_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(warm_start_clear_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(apply_warm_start_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(store_warm_start_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(position_solve_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(resolve_cs.h)
	}
	ByteCode const compute_collision_times(compiled::compute_collision_times_cs);
	ByteCode const clear_shock_lists(compiled::clear_shock_lists_cs);
	ByteCode const seed_shock_priority(compiled::seed_shock_priority_cs);
	ByteCode const propagate_shock_priority(compiled::propagate_shock_priority_cs);
	ByteCode const commit_shock_priority(compiled::commit_shock_priority_cs);
	ByteCode const finalize_shock_priority(compiled::finalize_shock_priority_cs);
	ByteCode const assign_colours(compiled::assign_colours_cs);
	ByteCode const warm_start_clear(compiled::warm_start_clear_cs);
	ByteCode const apply_warm_start(compiled::apply_warm_start_cs);
	ByteCode const store_warm_start(compiled::store_warm_start_cs);
	ByteCode const position_solve(compiled::position_solve_cs);
	ByteCode const resolve(compiled::resolve_cs);

	// Persistent rigid constraints
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(compile_constraints_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(assign_constraint_colours_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(apply_constraint_warm_start_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(clear_constraint_pseudo_velocity_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(solve_constraint_position_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(apply_constraint_position_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(solve_constraint_velocity_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(prepare_coupled_constraints_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(begin_coupled_velocity_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(build_coupled_velocity_candidates_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(gather_coupled_velocity_targets_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(select_coupled_velocity_trees_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(validate_coupled_velocity_trees_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(evaluate_coupled_velocity_merit_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(commit_coupled_velocity_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(finalize_coupled_velocity_islands_cs.h)
	}
	ByteCode const compile_constraints(compiled::compile_constraints_cs);
	ByteCode const assign_constraint_colours(compiled::assign_constraint_colours_cs);
	ByteCode const apply_constraint_warm_start(compiled::apply_constraint_warm_start_cs);
	ByteCode const clear_constraint_pseudo_velocity(compiled::clear_constraint_pseudo_velocity_cs);
	ByteCode const solve_constraint_position(compiled::solve_constraint_position_cs);
	ByteCode const apply_constraint_position(compiled::apply_constraint_position_cs);
	ByteCode const solve_constraint_velocity(compiled::solve_constraint_velocity_cs);
	ByteCode const prepare_coupled_constraints(compiled::prepare_coupled_constraints_cs);
	ByteCode const begin_coupled_velocity(compiled::begin_coupled_velocity_cs);
	ByteCode const build_coupled_velocity_candidates(compiled::build_coupled_velocity_candidates_cs);
	ByteCode const gather_coupled_velocity_targets(compiled::gather_coupled_velocity_targets_cs);
	ByteCode const select_coupled_velocity_trees(compiled::select_coupled_velocity_trees_cs);
	ByteCode const validate_coupled_velocity_trees(compiled::validate_coupled_velocity_trees_cs);
	ByteCode const evaluate_coupled_velocity_merit(compiled::evaluate_coupled_velocity_merit_cs);
	ByteCode const commit_coupled_velocity(compiled::commit_coupled_velocity_cs);
	ByteCode const finalize_coupled_velocity_islands(compiled::finalize_coupled_velocity_islands_cs);

	// Pure-tree articulation force ABA
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_prepare_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_inward_dynamics_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_root_dynamics_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_outward_dynamics_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_midpoint_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_prepare_mobility_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_apply_impulses_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_evaluate_impulses_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_commit_impulses_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_gather_proxy_forces_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(articulation_refresh_proxies_cs.h)
	}
	ByteCode const articulation_prepare(compiled::articulation_prepare_cs);
	ByteCode const articulation_inward_dynamics(compiled::articulation_inward_dynamics_cs);
	ByteCode const articulation_root_dynamics(compiled::articulation_root_dynamics_cs);
	ByteCode const articulation_outward_dynamics(compiled::articulation_outward_dynamics_cs);
	ByteCode const articulation_midpoint(compiled::articulation_midpoint_cs);
	ByteCode const articulation_prepare_mobility(compiled::articulation_prepare_mobility_cs);
	ByteCode const articulation_apply_impulses(compiled::articulation_apply_impulses_cs);
	ByteCode const articulation_evaluate_impulses(compiled::articulation_evaluate_impulses_cs);
	ByteCode const articulation_commit_impulses(compiled::articulation_commit_impulses_cs);
	ByteCode const articulation_gather_proxy_forces(compiled::articulation_gather_proxy_forces_cs);
	ByteCode const articulation_refresh_proxies(compiled::articulation_refresh_proxies_cs);

	// Selective refresh
	namespace compiled
	{
		#include PR_PHYSICS_SHADER_COMPILED_DIR(prepare_selective_refresh_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(score_selective_contacts_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(compact_selective_pairs_cs.h)
		#include PR_PHYSICS_SHADER_COMPILED_DIR(build_selective_dispatch_cs.h)
	}
	ByteCode const prepare_selective_refresh(compiled::prepare_selective_refresh_cs);
	ByteCode const score_selective_contacts(compiled::score_selective_contacts_cs);
	ByteCode const compact_selective_pairs(compiled::compact_selective_pairs_cs);
	ByteCode const build_selective_dispatch(compiled::build_selective_dispatch_cs);
}
