//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox
{
	// CPU-prepared terrain mesh plus its renderer binding.
	struct TerrainVisual
	{
		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4           , m_i2w  , rdr12::EInstComp::I2WTransform)\
			x(rdr12::ModelPtr, m_model, rdr12::EInstComp::ModelPtr)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		// Timings and bounds captured while preparing one terrain mesh.
		struct Metrics
		{
			double m_point_sampling_ms = 0.0;
			double m_batch_sampling_ms = 0.0;
			double m_batch_scalar_ms = 0.0;
			double m_mesh_generation_ms = 0.0;
			int m_point_sample_count = 0;
			int m_batch_count = 0;
			int m_batch_width = 4;
			size_t m_vertex_count = 0;
			size_t m_triangle_count = 0;
			size_t m_retained_cpu_bytes = 0;
			size_t m_peak_scratch_bytes = 0;
		};

		// Renderer-independent terrain mesh prepared entirely on the CPU.
		struct PreparedMesh
		{
			std::vector<rdr12::Vert> m_vertices;
			std::vector<uint32_t> m_indices;
			BBox m_local_bounds = BBox::Reset();
			math::BoundingBox<double> m_world_bounds = math::BoundingBox<double>::Reset();
			pr::physics::terrain::v4d m_origin_ws = pr::physics::terrain::v4d{0.0, 0.0, 0.0, 1.0};
			scene_loader::TerrainDesc m_desc = {};
			Metrics m_metrics = {};
		};

		Instance m_inst;
		PreparedMesh m_mesh;

		// Prepare the CPU mesh in a renderer-independent form.
		static PreparedMesh PrepareMesh(pr::physics::terrain::landscape::BaselineSurface const& surface, scene_loader::TerrainDesc const& terrain);

		// Create the renderer resources for a previously prepared CPU mesh.
		TerrainVisual(rdr12::Renderer& rdr, PreparedMesh prepared_mesh);

		// Add the visual to the scene when renderer resources are available.
		void AddToScene(rdr12::Scene& scene);
	};
}
