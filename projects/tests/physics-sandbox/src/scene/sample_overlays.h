#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// Geometry-only diagnostic samples, in collision-shape root space; no wetness or force filtering.
	struct SampleOverlayGeometry
	{
		std::vector<physics::surface::SurfaceSample> m_surface;
		std::vector<v4> m_volume;
	};

	// Additive geometry overlays, independent of base scene visualisation and physics registration.
	// Refresh/Reset require cleared renderer draw lists and completed GPU work before releasing instances.
	struct SampleOverlays
	{
		static constexpr float SurfaceSpacing = physics::surface::DefaultSpacing;
		static constexpr int VolumeSampleCount = 8192;
		static constexpr float NormalLength = 0.1f;

		// Immutable models shared across bodies with identical collision shape pointers.
		struct Model
		{
			rdr12::ldraw::LdrObjectPtr m_surface;
			rdr12::ldraw::LdrObjectPtr m_volume;
			std::string m_error;
		};

		// Separate persistent renderer instances keep simultaneous body transforms independent.
		struct Instance
		{
			collision::Shape const* m_shape = nullptr;
			rdr12::ldraw::LdrObjectPtr m_surface;
			rdr12::ldraw::LdrObjectPtr m_volume;
		};

		bool m_surface_enabled = false;
		bool m_volume_enabled = false;
		bool m_refresh_pending = false;
		std::unordered_map<collision::Shape const*, Model> m_models;
		std::unordered_map<void const*, Instance> m_instances;
		int m_failed_targets = 0;
		std::string m_first_error;

		// Build geometry using the existing surface/volume emitters and compound sibling ownership rules.
		static SampleOverlayGeometry BuildGeometry(collision::Shape const& shape, bool surface, bool volume);

		// Include all shaped non-static bodies, without inspecting or changing their sleeping state.
		static bool Eligible(physics::RigidBody const& body);

		// Include links with a floating root or a movable joint on their ancestor path.
		static bool Eligible(physics::Articulation const& articulation, physics::LinkHandle link);

		// Return whether either additive overlay is enabled.
		bool Enabled() const;

		// Change an independent overlay option; defer resource retirement until the renderer-safe refresh point.
		void Surface(bool enabled);
		void Volume(bool enabled);

		// Mark cached shape data stale without releasing anything referenced by existing draw lists.
		void Invalidate();

		// Release cached geometry and instances while preserving the user's checked options.
		void Reset();

		// Start a frame after any pending refresh, retaining previously submitted instance objects.
		void BeginFrame();

		// Add a target using cached root-space geometry and the target's current root-to-world transform.
		void Add(rdr12::Scene& scene, rdr12::Renderer& renderer, void const* target, collision::Shape const& shape, m4x4 const& root_to_world);
	};
}
