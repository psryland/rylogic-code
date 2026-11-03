//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_water.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"
#include "pr/physics/surface/surface_sampling.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	// Sampled water forces for the engine's dynamic shaped bodies and articulation links. See WaterConfig for the force model.
	// Shape geometry is uploaded once, when a shape is first seen, and stays resident on the GPU.
	// Once per frame the host selects the hulls that may touch the water and stages their headers and local water elements. Each internal
	// substep then evaluates every selected hull's surface cells on the GPU from the hull's current pose, so the forces follow the body between
	// substeps. Frames without wet hulls record no GPU work.
	struct GpuWater
	{
		// Identifies a body or articulation link for diagnostics: the rigid body with link -1, or the articulation and link index.
		struct HullKey
		{
			void const* m_owner;
			int m_link;

			friend bool operator ==(HullKey const&, HullKey const&) = default;
		};
		struct HullKeyHash
		{
			size_t operator()(HullKey const& key) const noexcept
			{
				return std::hash<void const*>{}(key.m_owner) ^ (std::hash<int>{}(key.m_link) * 0x9e3779b97f4a7c15ULL);
			}
		};

		// The state of one body or articulation link needed to select and stage it.
		struct Hull
		{
			// The collision shape that bounds the displaced water. Must outlive the step.
			collision::Shape const* m_shape;

			// Transform from the shape's root frame to world space.
			m4x4 m_o2w;

			// World-space bounds of the shape, centre of mass, and linear speed of the centre of mass.
			BBox m_bbox_ws;
			v4 m_com_ws;
			float m_speed;

			// Magnitude of the gravity acting on the body (m/s^2). It only bounds how far the body can move towards the water in a frame;
			// the water forces use the water's own gravity (see WaterConfig::m_gravity).
			float m_fall_acceleration;

			// Index of the hull's body in the GPU body buffer.
			int m_step_index;

			HullKey m_key;
		};

		// Per-primitive surface counts, compact patch range, half of the largest cell height (for the dry early-out), and the first word of the
		// primitive's sample visibility mask in the resident mask array (see buoyancy::HullPrimitivePlan). Mirrors HLSL BuoySurfPrimRecord.
		struct PrimRecord
		{
			int m_count;
			int m_patch_begin;
			int m_patch_count;
			float m_half_cell_height;
			int m_visible_base;
		};
		static_assert(sizeof(PrimRecord) == 20);

		// A grow-only GPU copy of a host array. Entries from 'm_uploaded' onwards are uploaded at the next sync, so an entry changed in place
		// lowers 'm_uploaded' to its index.
		template <typename T>
		struct ResidentArray
		{
			std::vector<T> m_cpu;
			D3DPtr<ID3D12Resource> m_res;
			int m_capacity = 0;
			int m_uploaded = 0;
		};

		// Per-shape values used to select and stage a hull, shared by every hull that uses the same collision shape. The shape's geometry and surface
		// sample plans live in the resident arrays (see GetOrCreateShape).
		struct CompositeShape
		{
			// The shape's block of primitives and records in the resident arrays, and the total surface samples over those primitives.
			int m_prim_base = 0;
			int m_prim_count = 0;
			int m_total_surface_samples = 0;

			// Half of the largest surface cell height over the whole hull. A surface cell is partly wet while its sample is less than this far above
			// the surface, so the hull is only certainly dry when its lowest point is this far above the water.
			float m_max_half_cell_height = 0.0f;

			// Scale-relative slack for the host-side dry cull.
			float m_eps = 0.0f;

			// Shape-root-space bounds enclosing every primitive, for the host-side dry cull.
			BBox m_bbox_os = BBox::Reset();
		};

		// A hull selected for this frame's GPU work.
		struct ActiveHull
		{
			int m_step_index;
			CompositeShape const* m_shape;
			int m_element_base;
			int m_element_count;
			float m_water_max_height;
			int m_group_base;
			int m_group_count;
			HullKey m_key;
		};

		Gpu& m_gpu;
		WaterConfig m_config;
		ComputeStep m_surface_step;
		ComputeStep m_reduce_step;

		// Shape-derived data keyed by the caller-owned shape. Entries are removed before their shape is destroyed (see Engine::ForgetShape), and all are
		// cleared whenever caller shapes may have changed or been destroyed (see Engine::ResetCaches).
		std::unordered_map<collision::Shape const*, std::unique_ptr<CompositeShape>> m_shapes;

		// The geometry of every cached shape, resident on the GPU. Each shape is appended once when it is first built, with its patch and
		// visibility-mask offsets already rebased into these arrays, so frames only upload the per-hull headers and water elements. The arrays
		// are emptied with 'm_shapes', and like it they grow with the number of distinct shapes seen until then.
		ResidentArray<uint32_t> m_visible;
		ResidentArray<buoyancy::GpuPrimitive> m_prims;
		ResidentArray<PrimRecord> m_records;
		ResidentArray<surface::SurfacePatch> m_patches;

		// This frame's selected hulls and their local water elements.
		std::vector<ActiveHull> m_active;
		std::vector<terrain::water::WaterFieldElement> m_elements;

		// The total number of sample groups over all active hulls, and the width of the 2D grid they are dispatched as.
		int m_group_count;
		int m_dispatch_width;

		// Upload addresses of the frame's per-hull inputs and of the hull index of each sample group. Upload memory stays valid for the whole
		// recorded frame, so every substep reuses them.
		GpuAddress m_elements_addr;
		GpuAddress m_headers_addr;
		GpuAddress m_group_hulls_addr;

		// Grow-only outputs of the sample and reduce passes.
		D3DPtr<ID3D12Resource> m_r_partials;
		D3DPtr<ID3D12Resource> m_r_diagnostics;
		int m_partial_capacity;
		int m_diagnostic_capacity;

		// Diagnostics recorded in the final substep, and the values published by the last completed step.
		ReadbackAlloc m_pending_readback;
		std::vector<HullKey> m_pending_keys;
		std::unordered_map<HullKey, WaterDiagnostics, HullKeyHash> m_diagnostics;

		// GPU timestamps around each substep's water dispatches, and the GPU time they measured in the last completed step (ms).
		// This is queue time, so it excludes host recording and waiting.
		D3DPtr<ID3D12QueryHeap> m_queries;
		D3DPtr<ID3D12Resource> m_query_readback;
		int m_query_capacity;
		int m_query_count;
		uint64_t m_frequency;
		double m_gpu_ms;

		// Compile the water pipelines. 'shader_cache' is optional; when given, compiled kernels are reused across runs.
		GpuWater(Gpu& gpu, WaterConfig config, IShaderCache* shader_cache);

		// Replace the water parameters. Cached shape data is rebuilt when the surface spacing changes.
		void Config(WaterConfig config);

		// Forget all cached shape data and diagnostics. Call when caller-owned shapes may have changed or been destroyed.
		void Reset();

		// Update the cached data of a leaf shape after its 'm_s2r' changed in place. Shapes not in the cache need nothing.
		void RefreshTransform(collision::Shape const& shape);

		// Forget the cached data of 'shape' before its caller destroys it. Shapes not in the cache need nothing.
		void ForgetShape(collision::Shape const& shape);

		// Select the hulls that may be wet during a frame of 'elapsed_s' seconds and stage their inputs for each of its 'substeps' substeps.
		void BeginFrame(GpuJob& job, std::span<Hull const> hulls, float elapsed_s, int substeps);

		// Record one substep's force evaluation into the body force accumulators. Must run after every other force has been accumulated,
		// because the implicit response treats them as part of the body's load. Records nothing when the frame has no wet hulls.
		void Apply(GpuJob& job, ID3D12Resource* bodies, float dt, double time_s, bool final_substep);

		// Publish the diagnostics and GPU time read back by the completed step.
		void CompleteStep();

		// Return the GPU time of the water dispatches in the last completed step (ms). Zero for dry frames.
		double GpuTimeMs() const
		{
			return m_gpu_ms;
		}

		// Return the diagnostics of a hull from the last completed step, or null when the hull was not evaluated (dry, asleep, or diagnostics off).
		WaterDiagnostics const* Diagnostics(HullKey key) const;

		// Wake sleeping bodies and articulations whose bounds overlap the band of heights that the moving surface can reach under them.
		// Does nothing for a flat field.
		void WakeInSurfaceBand(std::span<RigidBody* const> bodies, std::span<Articulation* const> articulations) const;

		// Make the water hull of a rigid body. Returns false when the body cannot receive water forces (static, asleep, or shapeless).
		static bool MakeHull(RigidBody const& body, int step_index, Hull& hull);

		// Make the water hull of an articulation link. Returns false when the link has no shape.
		static bool MakeHull(Articulation const& articulation, LinkHandle link, int step_index, Hull& hull);

		// Return true when any point of 'hull' might be below the surface of 'field' during the next 'elapsed_s' seconds.
		static bool MayBeWet(Hull const& hull, terrain::water::WaterField const& field, float elapsed_s);

	private:

		// Return the cached shape data for 'shape', building it and appending its geometry to the resident arrays on first use.
		CompositeShape const& GetOrCreateShape(collision::Shape const& shape);

		// Forget every cached shape and its resident geometry. The GPU buffers are kept for reuse.
		void ClearShapes();

		// Upload the entries of 'arr' added since its last sync, replacing its GPU buffer when it is too small.
		template <typename T>
		void SyncResident(GpuJob& job, ResidentArray<T>& arr, std::string_view name);

		// Ensure the partials and diagnostics buffers can hold this frame's output.
		void EnsureGpuCapacity(GpuJob& job, int partial_capacity, int diagnostic_capacity);
	};
}
