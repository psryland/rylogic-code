//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/atmosphere/air.h"
#include "pr/physics/buoyancy/buoyancy_primitives.h"
#include "pr/physics/terrain/water/water_field.h"

namespace pr::physics
{
	// A water environment that applies buoyancy, drag, added mass, and wave damping to every dynamic shaped rigid body and articulation link in the engine.
	// The water surface is a height field over world XY with +Z up. A point is wet when it is below the sampled water surface.
	// Each body's collision shape is covered by surface cells (see m_surface_spacing). The water pressure, drag, and inertia forces are integrated over the
	// wet cells, and the dry part of each cell feels air drag. The result is applied with a linearly implicit update, so light bodies settle onto the water
	// instead of bouncing off it. Water is not a collision contact, so material restitution has no effect on it.
	//
	// Limitations:
	//  - The local wave elements, including wind-sea, shoaling, and breaking corrections, are found once per frame at the centre of each body's bounding box
	//    and used over the whole body. This is exact in deep water without a wind sea. Bodies much larger than the scale over which the depth or wind sea
	//    changes see the conditions at their centre only. A possible improvement is to represent the depth under each body as a plane rather than a value.
	//  - Bodies sleep normally. Sleeping bodies whose bounds overlap the band that waves can reach are woken each frame, so floating bodies on a moving
	//    surface stay awake while bodies resting below the band, such as dense objects on the sea floor, can sleep.
	struct WaterConfig
	{
		// The sampled water surface.
		terrain::water::WaterField m_field;

		// Water density (kg/m^3). Buoyancy is the integral of the water pressure, |gravity| * density * depth, over the wet surface cells,
		// which equals |gravity| * density per unit submerged volume.
		float m_density = 1000.0f;

		// Linear viscous drag time-constant (seconds). Each wet surface cell stands for the vertical water column between it and the water surface,
		// with signed volume dV (the columns sum to the submerged volume). Each column contributes
		//   dF = -(density / tau_linear) * dV * (v_linear - v_water)
		// Tau is the e-folding time for a fully submerged body whose density matches the water; lighter bodies decay faster, heavier bodies slower.
		// Set to <= 0 to disable linear drag.
		float m_linear_drag_time_constant_s = 5.0f;

		// Angular viscous drag time-constant (seconds). Each wet water column (see above) contributes
		//   dF = -(density / tau_angular) * dV * cross(omega, column_centroid - centre_of_mass)
		// Integrating the resulting torque over the submerged geometry damps rotation according to its actual wet shape and lever arms.
		// Set to <= 0 to disable angular volume drag.
		float m_angular_drag_time_constant_s = 0.75f;

		// Quadratic normal form-drag coefficient (dimensionless). Each wet surface cell contributes
		//   dF_n = -0.5 * density * Cd * dA * max(0, v_n)^2 * n_ws
		// where v_n is the outward-normal relative velocity. Typical values are 1.05 for a cube (default), 0.47 for a sphere, and 1.28 for a flat plate.
		// The dry part of each cell feels the same drag relative to the air (see m_air). Set to zero to disable form drag.
		float m_quadratic_drag_coefficient = 1.05f;

		// Tangential quadratic drag coefficient (dimensionless). This models unresolved surface shear independently of normal form drag:
		//   dF_t = -0.5 * density * Ct * dA * |v_t| * v_t
		// The default is a typical skin-friction value for a smooth hull. Together with the angular drag, it damps the spin of round bodies, whose
		// surface moves tangentially through the water. Set to zero to disable tangential drag.
		float m_tangential_drag_coefficient = 0.01f;

		// Added-mass coefficient (dimensionless). A body cannot accelerate relative to the water without also accelerating the water around it.
		// Each wet water column adds Ca * density * dV of translational inertia at the centre of mass, and the wave's lateral pressure gradient also
		// drives that water, giving the Morison inertia force (1 + Ca) * density * dV * a_fluid. For light bodies the added mass is many times the body
		// mass, which is why they follow the water instead of bouncing off it. 0.5 is the value for a sphere. Set to zero to disable added mass.
		float m_added_mass_coefficient = 0.5f;

		// Scale of the wave-radiation damping (dimensionless; 1 is the physical estimate, 0 disables it).
		// A floating body that moves up and down relative to the water makes surface waves, and the energy they carry away damps the motion.
		// For light bodies this damping is very strong, which is why they settle onto the water instead of bouncing on it.
		float m_radiation_damping = 1.0f;

		// Scale of the steady wave-making resistance (dimensionless; 1 is the physical estimate, 0 disables it).
		// A floating body moving horizontally relative to the surface water leaves a wake of waves behind it, and the energy they carry away resists
		// the motion. The resistance is the linear-theory value for a pressure patch with the body's displaced weight and waterplane shape
		// (see pr/physics/buoyancy/wave_resistance.hlsli). It is negligible for slow bodies, peaks when the body moves at about the speed of waves as
		// long as its waterline, and falls at higher speeds. Blunt hulls make less wave resistance than linear theory predicts, so values below 1 can
		// be used for them.
		float m_wave_resistance = 1.0f;

		// Maximum surface-cell diameter in shape-local metres. Buoyancy and drag are both integrated over these cells. Small hulls are sampled
		// more finely so each gets at least buoyancy::MinSurfaceSamplesPerHull visible samples (see buoyancy::BuildHullSurfacePlans). Smaller
		// values cost more GPU work per body.
		float m_surface_spacing = buoyancy::DefaultSurfaceSpacing;

		// The air above the water. Only bodies that touch the water feel air drag from this model; bodies flying clear of the water feel none.
		Air m_air;

		// Read back per-body force, torque, volume, and centre-of-buoyancy diagnostics (see Engine::WaterDiagnosticsFor).
		// Disabled by default so production stepping does not pay for validation data.
		bool m_enable_diagnostics = false;

		// Throw invalid_argument unless every parameter is finite, the densities, drag coefficients, and scales are non-negative, and the spacing is valid.
		void Validate() const;
	};

	// The water forces applied to one body or articulation link in the final internal substep of the most recent frame.
	struct WaterDiagnostics
	{
		// Submerged volume (m^3).
		float m_volume_m3 = 0.0f;

		// Total water and air force on the body (N, world space).
		v4 m_force_ws = v4::Zero();

		// Centre of the submerged volume (world space).
		v4 m_centre_buoyancy_ws = v4::Origin();

		// Total water and air torque about the centre of mass (N m, world space).
		v4 m_torque_ws = v4::Zero();
	};
}
