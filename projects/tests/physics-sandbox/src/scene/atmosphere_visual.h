//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox
{
	// GPU atmosphere solver and LDraw diagnostics for a scene-loaded atmosphere block.
	struct AtmosphereVisual
	{
		physics::Gpu m_gpu;
		physics::atmosphere::AtmosphereSolver m_solver;
		std::unique_ptr<physics::atmosphere::AtmosphereTracers> m_tracers;
		scene_loader::AtmosphereDesc m_desc;
		std::vector<physics::atmosphere::AtmosphereTracerParticle> m_particles;
		rdr12::Renderer& m_rdr;
		rdr12::ldraw::LdrObjectPtr m_gfx;
		bool m_gfx_stale; // True when 'm_gfx' must be rebuilt at the next AddToScene
		bool m_show_grid;
		bool m_show_particles;
		bool m_show_pressure_nodes;

		// Create the solver, tracer set, and initial diagnostic geometry on 'device', using a separate command queue so atmosphere work does not
		// interleave with physics steps that are still in flight. 'shader_cache' must outlive this object.
		AtmosphereVisual(ID3D12Device4* device, rdr12::Renderer& rdr, ::pr::compute::shader_cache::IShaderCache& shader_cache, scene_loader::AtmosphereDesc desc);

		// Advance the atmosphere solver and refresh CPU-visible tracer state.
		void Step(float dt);

		// Add current atmosphere diagnostics to the render scene. Call only after the scene's drawlists have been cleared.
		void AddToScene(rdr12::Scene& scene);

		// Toggle the floor/lid/domain grid overlay.
		void ShowGrid(bool show);

		// Toggle tracer particle rendering.
		void ShowParticles(bool show);

		// Toggle pressure-node rendering.
		void ShowPressureNodes(bool show);

		// Return true when the grid overlay is enabled.
		bool ShowGrid() const;

		// Return true when tracer particle rendering is enabled.
		bool ShowParticles() const;

		// Return true when pressure-node rendering is enabled.
		bool ShowPressureNodes() const;

	private:
		// Rebuild the LDraw diagnostic overlay from the latest CPU-visible state.
		void RebuildGfx();
	};
}
