//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/buoyancy/gpu_water.h"
#include "pr/physics/articulation/articulation.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/terrain/water/water_field.hlsli"
#include "src/utility/shader_recipes.h"

namespace pr::physics
{
	using namespace pr::compute;
	namespace
	{
		// Threads per group in the sample and reduce kernels. Must match gpu_water.hlsl. The reducer sums one partial per sample group on a
		// single thread group, so at most the reduce thread count of groups serve each hull; larger hulls stream extra samples through the same threads.
		constexpr int SampleThreadCount = 256;
		constexpr int ReduceThreadCount = 128;

		// Root constants for both kernels. Must match 'CBufGpuWater' in gpu_water.hlsl.
		struct CBufWater
		{
			int m_hull_count;
			int m_group_count;
			int m_dispatch_width;
			float m_time_s;
			float m_water_level;
			float m_fluid_density;
			float m_linear_drag_coefficient;
			float m_angular_drag_coefficient;
			float m_quadratic_drag_coefficient;
			float m_tangential_drag_coefficient;
			float m_added_mass_coefficient;
			float m_radiation_damping;
			float m_time_step_s;
			int m_enable_diagnostics;
			float m_wave_resistance;
			float m_air_density;
			float m_gravity;
		};
		static_assert(sizeof(CBufWater) % sizeof(uint32_t) == 0);

		// Mirrors HLSL BuoyancySums; see gpu_water.hlsl for the meaning of each sum. The host only sizes the buffer.
		struct GpuWaterPartial
		{
			float m_sums[55];
		};
		static_assert(sizeof(GpuWaterPartial) == 220);

		// Mirrors HLSL BuoyancyDiagnostic.
		struct GpuWaterDiagnostic
		{
			v4 m_force_ws;
			v4 m_torque_ws;
			v4 m_centre_buoyancy_ws;
			float m_volume_m3;
			int m_valid;
			float m_pad0;
			float m_pad1;
		};
		static_assert(sizeof(GpuWaterDiagnostic) == 64);

		// Per-hull header. Mirrors HLSL BuoySurfHeader. 'm_prim_base' and 'm_prim_count' index the resident primitive and record arrays,
		// 'm_element_base' and 'm_element_count' index the frame's concatenated local water elements, and 'm_group_base' and 'm_group_count'
		// are the hull's range of sample groups and reduction partials.
		struct GpuWaterHeader
		{
			int m_body_index;
			int m_prim_base;
			int m_prim_count;
			int m_total_surface_samples;
			int m_element_base;
			int m_element_count;
			float m_water_max_height;
			int m_group_base;
			int m_group_count;
			int m_pad0;
			int m_pad1;
			int m_pad2;
			v4 m_bbox_centre_os;
			v4 m_bbox_radius_os;
		};
		static_assert(sizeof(GpuWaterHeader) == 80);

		// Compile one water kernel. The constructor and precompilation share this so their cache keys match.
		std::vector<uint8_t> CompileWaterKernel(wchar_t const* entry_point, IShaderCache* shader_cache)
		{
			// The optional cache skips DXC when the source is unchanged.
			auto resolver = shader_cache::ResourceSourceResolver{};
			return ShaderCompiler{}
				.Source("src/buoyancy/gpu_water.hlsl", resolver)
				.Cache(shader_cache)
				.HlslVersion(EHlslVersion::Hlsl2021)
				.Define(L"SHADER_BUILD")
				.Optimise(true)
				.ShaderModel(L"cs_6_6")
				.EntryPoint(entry_point)
				.Compile();
		}
		std::vector<uint8_t> CompileSampleKernel(IShaderCache* shader_cache)
		{
			return CompileWaterKernel(L"CSBuoyancySurfaceSamples", shader_cache);
		}
		std::vector<uint8_t> CompileReduceKernel(IShaderCache* shader_cache)
		{
			return CompileWaterKernel(L"CSBuoyancySurfaceReduce", shader_cache);
		}

		// Create a default-heap UAV buffer of 'count' elements.
		template <typename T>
		D3DPtr<ID3D12Resource> CreateUavBuffer(ID3D12Device* device, int count, std::string_view name)
		{
			// Buffers start in COMMON but are tracked with a UAV default state, relying on buffer promotion for first use.
			auto const desc = D3D12_RESOURCE_DESC{
				.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
				.Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
				.Width = static_cast<UINT64>(count) * sizeof(T),
				.Height = 1,
				.DepthOrArraySize = 1,
				.MipLevels = 1,
				.Format = DXGI_FORMAT_UNKNOWN,
				.SampleDesc = DXGI_SAMPLE_DESC{ .Count = 1, .Quality = 0 },
				.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
				.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
			};
			auto res = D3DPtr<ID3D12Resource>{};
			auto const& heap_props = HeapProps::Default();
			Check(device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr, __uuidof(ID3D12Resource), reinterpret_cast<void**>(res.address_of())));
			DefaultResState(res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			DebugName(res, name);
			return res;
		}

		// Throw unless 'value' is finite and non-negative.
		void RequireNonNegative(float value, char const* what)
		{
			if (!std::isfinite(value) || value < 0.0f)
				throw std::invalid_argument(std::format("Water {} must be finite and non-negative", what));
		}

		// Return the lowest point of a shape-space box along 'up' after transforming it by 'o2w'.
		float LowestAlong(m4x4 const& o2w, BBox const& bbox_os, v4 up)
		{
			// Project the centre, then subtract the support distance contributed by each rotated axis scaled by its half extent.
			auto const centre_ws = o2w * bbox_os.m_centre;
			auto const r = bbox_os.m_radius;
			auto const extent = Abs(Dot3(o2w.x, up)) * r.x + Abs(Dot3(o2w.y, up)) * r.y + Abs(Dot3(o2w.z, up)) * r.z;
			return Dot3(centre_ws, up) - extent;
		}
	}

	// Validate the water parameters. The field validates itself on construction.
	void WaterConfig::Validate() const
	{
		// Density scales every water force, so it must be positive; clear the water to remove it.
		surface::ValidateSpacing(m_surface_spacing);
		if (!std::isfinite(m_density) || m_density <= 0.0f)
			throw std::invalid_argument("Water density must be finite and positive");

		// Gravity gives the water its pressure and its wave speeds, so it must be positive.
		if (!std::isfinite(m_gravity) || m_gravity <= 0.0f)
			throw std::invalid_argument("Water gravity must be finite and positive");

		// The remaining coefficients scale forces, so they must not be negative.
		RequireNonNegative(m_quadratic_drag_coefficient, "quadratic drag coefficient");
		RequireNonNegative(m_tangential_drag_coefficient, "tangential drag coefficient");
		RequireNonNegative(m_added_mass_coefficient, "added-mass coefficient");
		RequireNonNegative(m_radiation_damping, "radiation damping");
		RequireNonNegative(m_wave_resistance, "wave resistance");
		RequireNonNegative(m_air.m_density, "air density");

		// Time-constants may be any finite value because non-positive values disable their term.
		if (!std::isfinite(m_linear_drag_time_constant_s))
			throw std::invalid_argument("Water linear drag time-constant must be finite");
		if (!std::isfinite(m_angular_drag_time_constant_s))
			throw std::invalid_argument("Water angular drag time-constant must be finite");
	}

	// Add the recipes for the water kernels.
	void AppendWaterShaderRecipes(std::vector<ShaderRecipe>& recipes)
	{
		// The kernels do not depend on the water configuration.
		recipes.push_back({ .m_name = "Water.CSBuoyancySurfaceSamples", .m_compile = CompileSampleKernel });
		recipes.push_back({ .m_name = "Water.CSBuoyancySurfaceReduce", .m_compile = CompileReduceKernel });
	}

	// Compile the pipelines once; changing the water configuration later does not need new pipelines.
	GpuWater::GpuWater(Gpu& gpu, WaterConfig config, IShaderCache* shader_cache)
		: m_gpu(gpu)
		, m_config(std::move(config))
		, m_surface_step()
		, m_reduce_step()
		, m_shapes()
		, m_visible()
		, m_prims()
		, m_records()
		, m_patches()
		, m_active()
		, m_elements()
		, m_group_count()
		, m_dispatch_width()
		, m_elements_addr(nullptr)
		, m_headers_addr(nullptr)
		, m_group_hulls_addr(nullptr)
		, m_r_partials()
		, m_r_diagnostics()
		, m_partial_capacity()
		, m_diagnostic_capacity()
		, m_pending_readback()
		, m_pending_keys()
		, m_diagnostics()
		, m_queries()
		, m_query_readback()
		, m_query_capacity()
		, m_query_count()
		, m_frequency()
		, m_gpu_ms()
	{
		// The sample pass reads the bodies, water elements, primitives, sample visibility masks, headers, patches, primitive records,
		// and the hull of each group, and writes one partial sum per thread group. Root parameter order must match the binding order in Apply.
		m_config.Validate();
		m_surface_step.m_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<CBufWater>(hlsl::ECBufReg::b0)
			.UAV(hlsl::EUAVReg::u0)
			.SRV(hlsl::ESRVReg::t1)
			.SRV(hlsl::ESRVReg::t3)
			.SRV(hlsl::ESRVReg::t6)
			.SRV(hlsl::ESRVReg::t8)
			.SRV(hlsl::ESRVReg::t9)
			.SRV(hlsl::ESRVReg::t11)
			.SRV(hlsl::ESRVReg::t12)
			.UAV(hlsl::EUAVReg::u1)
			.Create(gpu, "Physics.Water.Samples.RootSig");
		m_surface_step.m_pso = ComputePSO(m_surface_step.m_sig.get(), CompileSampleKernel(shader_cache)).Cache(shader_cache).Create(gpu, "Physics.Water.Samples.PSO");

		// The reduce pass sums each hull's partials, applies the implicit response to the body, and optionally writes a diagnostic.
		m_reduce_step.m_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<CBufWater>(hlsl::ECBufReg::b0)
			.UAV(hlsl::EUAVReg::u0)
			.SRV(hlsl::ESRVReg::t8)
			.UAV(hlsl::EUAVReg::u1)
			.UAV(hlsl::EUAVReg::u2)
			.Create(gpu, "Physics.Water.Reduce.RootSig");
		m_reduce_step.m_pso = ComputePSO(m_reduce_step.m_sig.get(), CompileReduceKernel(shader_cache)).Cache(shader_cache).Create(gpu, "Physics.Water.Reduce.PSO");
	}

	// Replace the water parameters.
	void GpuWater::Config(WaterConfig config)
	{
		// Shape data depends on the surface spacing only.
		config.Validate();
		if (config.m_surface_spacing != m_config.m_surface_spacing)
			ClearShapes();

		m_config = std::move(config);
	}

	// Forget cached shape data and diagnostics.
	void GpuWater::Reset()
	{
		// Pointers to caller-owned shapes may now be dangling or reused.
		ClearShapes();
		m_diagnostics.clear();
	}

	// Update the cached data of a leaf shape after its transform changed.
	void GpuWater::RefreshTransform(collision::Shape const& shape)
	{
		// A leaf shape is one primitive whose surface plan is in the primitive's own frame, and a single primitive has no siblings to cull
		// against. So only the primitive's transform and the root-space bounds depend on 'm_s2r'.
		assert(shape.m_type != collision::EShape::Array && "Compound shapes keep absolute child transforms; refresh is for leaf shapes");
		auto iter = m_shapes.find(&shape);
		if (iter == m_shapes.end())
			return;

		auto& data = *iter->second;
		m_prims.m_cpu[data.m_prim_base].m_s2r = shape.m_s2r;
		m_prims.m_uploaded = std::min(m_prims.m_uploaded, data.m_prim_base);
		data.m_bbox_os = collision::CalcBBox(shape);
		data.m_eps = buoyancy::HullInsideSlack(shape);
	}

	// Forget the cached data of a shape that is about to be destroyed.
	void GpuWater::ForgetShape(collision::Shape const& shape)
	{
		// Only shapes of wet dynamic bodies are cached, so this is rare. Their geometry is appended to shared resident arrays that cannot drop one
		// shape's range, so the whole cache is rebuilt from the shapes still in use.
		if (m_shapes.contains(&shape))
			ClearShapes();
	}

	// Forget every cached shape and its resident geometry.
	void GpuWater::ClearShapes()
	{
		// Emptying the host arrays makes the next sync upload from the start, so the buffers can be reused.
		m_shapes.clear();
		auto clear = [](auto& arr)
		{
			arr.m_cpu.clear();
			arr.m_uploaded = 0;
		};
		clear(m_visible);
		clear(m_prims);
		clear(m_records);
		clear(m_patches);
	}

	// Select and stage the hulls that may be wet this frame.
	void GpuWater::BeginFrame(GpuJob& job, std::span<Hull const> hulls, float elapsed_s, int substeps)
	{
		// Start an empty frame. Diagnostics and timestamps are only recorded for frames that dispatch.
		m_active.clear();
		m_elements.clear();
		m_pending_keys.clear();
		m_pending_readback = {};
		m_query_count = 0;

		// Select the wet hulls and find the local water elements of each.
		auto const& field = m_config.m_field;
		auto const level = field.Level();
		auto buffer = std::array<terrain::water::WaterFieldElement, terrain::water::MaxElementCount>{};
		for (auto const& hull : hulls)
		{
			// A cheap conservative test first, so hulls far from the water cost nothing more.
			if (!MayBeWet(hull, field, elapsed_s))
				continue;

			// The local elements found at the centre of the hull's bounds are used over the whole hull for the frame (see WaterConfig).
			auto const centre = terrain::v2d{hull.m_bbox_ws.m_centre.x, hull.m_bbox_ws.m_centre.y};
			auto const elements = field.LocalElements(centre, buffer);

			// The highest surface these elements can make, inflated so float rounding on the GPU can never put a wet sample above it.
			auto bound = level;
			for (auto const& element : elements)
				bound += terrain::water::shared::WaterFieldElementAmplitudeBound(element);

			bound += 1e-6 * (std::abs(bound) + static_cast<double>(elements.size()) * (bound - level));
			auto const water_max_height = std::nextafter(static_cast<float>(bound), std::numeric_limits<float>::infinity());

			// Skip hulls whose whole shape is above that surface. The shader would find every sample dry, so this does not change the result.
			// Heights are measured along world +Z, the water's up.
			auto const& shape = GetOrCreateShape(*hull.m_shape);
			auto const lowest = LowestAlong(hull.m_o2w, shape.m_bbox_os, v4::ZAxis());
			if (lowest > water_max_height + std::max(shape.m_eps, 1e-4f) + shape.m_max_half_cell_height)
				continue;

			m_active.push_back(ActiveHull{
				.m_step_index = hull.m_step_index,
				.m_shape = &shape,
				.m_element_base = isize(m_elements),
				.m_element_count = isize(elements),
				.m_water_max_height = water_max_height,
				.m_group_base = 0,
				.m_group_count = 0,
				.m_key = hull.m_key,
			});
			m_elements.insert(m_elements.end(), elements.begin(), elements.end());
		}
		if (m_active.empty())
			return;

		// Give each hull only the sample groups its own surface needs, so small hulls do not pay for the largest one. Each group's partial
		// is reduced by one thread of the hull's reduce group, which bounds the groups per hull; larger hulls stream extra samples through them.
		auto const hull_count = isize(m_active);
		if (hull_count > 65535)
			throw std::runtime_error("Water hull count exceeds the dispatch dimension");

		m_group_count = 0;
		for (auto& active : m_active)
		{
			active.m_group_base = m_group_count;
			active.m_group_count = std::clamp((active.m_shape->m_total_surface_samples + SampleThreadCount - 1) / SampleThreadCount, 1, ReduceThreadCount);
			m_group_count += active.m_group_count;
		}
		m_dispatch_width = std::min(m_group_count, 65535);
		EnsureGpuCapacity(job, m_group_count, m_config.m_enable_diagnostics ? hull_count : 1);

		// Grow the timestamp storage to two queries per substep before any substep is recorded.
		if (substeps < 1)
			throw std::runtime_error("Water substep count must be positive");

		if (m_query_capacity < 2 * substeps)
		{
			auto const query_count = 2 * substeps;
			auto desc = D3D12_QUERY_HEAP_DESC{.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP, .Count = s_cast<UINT>(query_count)};
			auto queries = D3DPtr<ID3D12QueryHeap>{};
			Check(m_gpu->CreateQueryHeap(&desc, __uuidof(ID3D12QueryHeap), (void**)queries.address_of()));
			auto readback = ResDesc::Buf<uint64_t>(query_count, {});
			readback.HeapProps = HeapProps(D3D12_HEAP_TYPE_READBACK);
			readback.DefaultState = D3D12_RESOURCE_STATE_COPY_DEST;
			m_query_readback = m_gpu.CreateResource(readback, job, "Physics.Water.Timestamps");
			Check(job.m_queue->GetTimestampFrequency(&m_frequency));
			m_queries = std::move(queries);
			m_query_capacity = query_count;
		}

		// Upload the geometry of shapes built since the last frame. Steady-state frames upload nothing here.
		SyncResident(job, m_visible, "Physics.Water.Visible");
		SyncResident(job, m_prims, "Physics.Water.Prims");
		SyncResident(job, m_records, "Physics.Water.PrimRecords");
		SyncResident(job, m_patches, "Physics.Water.Patches");

		// Emit one header per hull referencing its shape's resident geometry, its local elements, and its sample groups, and map each group to its hull.
		auto upload_headers = job.m_upload.Alloc<GpuWaterHeader>(hull_count);
		auto upload_elements = job.m_upload.Alloc<terrain::water::WaterFieldElement>(std::max(isize(m_elements), 1));
		auto upload_group_hulls = job.m_upload.Alloc<int>(m_group_count);
		auto headers = upload_headers.ptr<GpuWaterHeader>();
		auto group_hulls = upload_group_hulls.ptr<int>();
		for (int i = 0; i != hull_count; ++i)
		{
			auto const& active = m_active[i];
			auto const& shape = *active.m_shape;
			std::fill_n(group_hulls + active.m_group_base, active.m_group_count, i);
			headers[i] = GpuWaterHeader{
				.m_body_index = active.m_step_index,
				.m_prim_base = shape.m_prim_base,
				.m_prim_count = shape.m_prim_count,
				.m_total_surface_samples = shape.m_total_surface_samples,
				.m_element_base = active.m_element_base,
				.m_element_count = active.m_element_count,
				.m_water_max_height = active.m_water_max_height,
				.m_group_base = active.m_group_base,
				.m_group_count = active.m_group_count,
				.m_pad0 = 0,
				.m_pad1 = 0,
				.m_bbox_centre_os = shape.m_bbox_os.m_centre,
				.m_bbox_radius_os = shape.m_bbox_os.m_radius,
			};
		}

		// Stage the local elements. A flat field has none, so the single bound entry is zeroed.
		memset(upload_elements.ptr<std::byte>(), 0, sizeof(terrain::water::WaterFieldElement));
		std::copy(m_elements.begin(), m_elements.end(), upload_elements.ptr<terrain::water::WaterFieldElement>());

		m_headers_addr = upload_headers;
		m_elements_addr = upload_elements;
		m_group_hulls_addr = upload_group_hulls;
	}

	// Record one substep's sampled water forces.
	void GpuWater::Apply(GpuJob& job, ID3D12Resource* bodies, float dt, double time_s, bool final_substep)
	{
		// Dry frames and zero-length substeps record nothing.
		if (m_active.empty() || !(dt > 0.0f))
			return;

		// Volume drag acts on the wet water columns with rho/tau coefficients; a non-positive time-constant disables its term.
		auto const& config = m_config;
		auto const cb = CBufWater{
			.m_hull_count = isize(m_active),
			.m_group_count = m_group_count,
			.m_dispatch_width = m_dispatch_width,
			.m_time_s = config.m_field.LocalTime(time_s),
			.m_water_level = static_cast<float>(config.m_field.Level()),
			.m_fluid_density = config.m_density,
			.m_linear_drag_coefficient = config.m_linear_drag_time_constant_s > 0.0f ? config.m_density / config.m_linear_drag_time_constant_s : 0.0f,
			.m_angular_drag_coefficient = config.m_angular_drag_time_constant_s > 0.0f ? config.m_density / config.m_angular_drag_time_constant_s : 0.0f,
			.m_quadratic_drag_coefficient = config.m_quadratic_drag_coefficient,
			.m_tangential_drag_coefficient = config.m_tangential_drag_coefficient,
			.m_added_mass_coefficient = config.m_added_mass_coefficient,
			.m_radiation_damping = config.m_radiation_damping,
			.m_time_step_s = dt,
			.m_enable_diagnostics = config.m_enable_diagnostics ? 1 : 0,
			.m_wave_resistance = config.m_wave_resistance,
			.m_air_density = config.m_air.m_density,
			.m_gravity = config.m_gravity,
		};

		// Bracket this substep's water work with timestamps so other GPU stages are not counted.
		if (m_query_count + 2 > m_query_capacity)
			throw std::runtime_error("Water timestamp capacity exceeded");

		auto& cmd_list = job.m_cmd_list;
		cmd_list.get()->EndQuery(m_queries.get(), D3D12_QUERY_TYPE_TIMESTAMP, m_query_count++);

		// Evaluate the surface samples into one partial record per thread group. The groups are tiled into a 2D grid because one
		// dispatch dimension is limited to 65535 groups.
		cmd_list.SetPipelineState(m_surface_step.m_pso.get());
		cmd_list.SetComputeRootSignature(m_surface_step.m_sig.get());
		cmd_list.AddComputeRoot32BitConstants(cb);
		cmd_list.AddComputeRootUnorderedAccessView(bodies);
		cmd_list.AddComputeRootShaderResourceView(m_elements_addr);
		cmd_list.AddComputeRootShaderResourceView(m_prims.m_res.get());
		cmd_list.AddComputeRootShaderResourceView(m_visible.m_res.get());
		cmd_list.AddComputeRootShaderResourceView(m_headers_addr);
		cmd_list.AddComputeRootShaderResourceView(m_patches.m_res.get());
		cmd_list.AddComputeRootShaderResourceView(m_records.m_res.get());
		cmd_list.AddComputeRootShaderResourceView(m_group_hulls_addr);
		cmd_list.AddComputeRootUnorderedAccessView(m_r_partials.get());
		cmd_list.Dispatch(m_dispatch_width, (m_group_count + m_dispatch_width - 1) / m_dispatch_width, 1);
		job.m_barriers.UAV(m_r_partials.get()).Commit();

		// Reduce the partials into each body's force accumulators. The diagnostics buffer stays bound even when unused, so there is one root layout.
		cmd_list.SetPipelineState(m_reduce_step.m_pso.get());
		cmd_list.SetComputeRootSignature(m_reduce_step.m_sig.get());
		cmd_list.AddComputeRoot32BitConstants(cb);
		cmd_list.AddComputeRootUnorderedAccessView(bodies);
		cmd_list.AddComputeRootShaderResourceView(m_headers_addr);
		cmd_list.AddComputeRootUnorderedAccessView(m_r_partials.get());
		cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics.get());
		cmd_list.Dispatch(cb.m_hull_count, 1, 1);
		job.m_barriers.UAV(bodies);
		if (config.m_enable_diagnostics)
			job.m_barriers.UAV(m_r_diagnostics.get());

		job.m_barriers.Commit();
		cmd_list.get()->EndQuery(m_queries.get(), D3D12_QUERY_TYPE_TIMESTAMP, m_query_count++);

		// The final substep copies the frame's timestamps out for CompleteStep.
		if (final_substep)
			cmd_list.get()->ResolveQueryData(m_queries.get(), D3D12_QUERY_TYPE_TIMESTAMP, 0, m_query_count, m_query_readback.get(), 0);

		// Only the final substep's diagnostics are observable after the frame, so only it is read back.
		if (config.m_enable_diagnostics && final_substep)
		{
			m_pending_readback = job.m_readback.Alloc<GpuWaterDiagnostic>(cb.m_hull_count);
			job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			cmd_list.CopyBufferRegion(m_pending_readback, m_r_diagnostics.get(), 0);
			job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();

			m_pending_keys.clear();
			for (auto const& active : m_active)
				m_pending_keys.push_back(active.m_key);
		}
	}

	// Publish the diagnostics of the completed step.
	void GpuWater::CompleteStep()
	{
		// Sum the paired substep intervals, so GPU work between the water dispatches is not counted. Dry frames record no queries.
		m_gpu_ms = 0;
		if (m_query_count != 0)
		{
			void* mapped = nullptr;
			auto range = D3D12_RANGE{0, sizeof(uint64_t) * m_query_count};
			Check(m_query_readback->Map(0, &range, &mapped));
			auto const* ticks = static_cast<uint64_t const*>(mapped);
			auto elapsed = uint64_t{};
			for (int i = 0; i + 1 < m_query_count; i += 2)
				elapsed += ticks[i + 1] - ticks[i];

			auto written = D3D12_RANGE{};
			m_query_readback->Unmap(0, &written);
			m_gpu_ms = 1000.0 * static_cast<double>(elapsed) / m_frequency;
		}

		// Hulls that were not evaluated in this step have no diagnostics.
		m_diagnostics.clear();
		if (m_pending_keys.empty())
			return;

		auto const diagnostics = std::span{ m_pending_readback.ptr<GpuWaterDiagnostic>(), m_pending_keys.size() };
		for (size_t i = 0; i != m_pending_keys.size(); ++i)
		{
			auto const& diag = diagnostics[i];
			if (diag.m_valid == 0)
				continue;

			m_diagnostics[m_pending_keys[i]] = WaterDiagnostics{
				.m_volume_m3 = diag.m_volume_m3,
				.m_force_ws = diag.m_force_ws,
				.m_centre_buoyancy_ws = diag.m_centre_buoyancy_ws,
				.m_torque_ws = diag.m_torque_ws,
			};
		}
		m_pending_keys.clear();
		m_pending_readback = {};
	}

	// Return the published diagnostics of a hull.
	WaterDiagnostics const* GpuWater::Diagnostics(HullKey key) const
	{
		auto iter = m_diagnostics.find(key);
		return iter != m_diagnostics.end() ? &iter->second : nullptr;
	}

	// Wake sleeping objects that the moving surface may reach.
	void GpuWater::WakeInSurfaceBand(std::span<RigidBody* const> bodies, std::span<Articulation* const> articulations) const
	{
		// A flat surface never moves, so resting objects keep their sleep state.
		auto const& field = m_config.m_field;
		if (field.IsFlat())
			return;

		// True when the vertical extent of 'bbox' overlaps the heights the surface can reach over its footprint.
		auto in_band = [&field](BBox const& bbox)
		{
			auto const lo = terrain::v2d{bbox.m_centre.x - bbox.m_radius.x, bbox.m_centre.y - bbox.m_radius.y};
			auto const hi = terrain::v2d{bbox.m_centre.x + bbox.m_radius.x, bbox.m_centre.y + bbox.m_radius.y};
			auto const bottom = static_cast<double>(bbox.m_centre.z - bbox.m_radius.z);
			auto const top = static_cast<double>(bbox.m_centre.z + bbox.m_radius.z);
			return bottom < field.MaxHeight(lo, hi) && top > field.MinHeight(lo, hi);
		};

		// Only sleeping dynamic bodies need testing; awake bodies are handled by hull selection.
		for (auto body : bodies)
		{
			if (!body->Sleeping() || body->InvMass() == 0.0f || !body->HasShape())
				continue;
			if (in_band(body->BBoxWS()))
				body->Wake();
		}

		// A sleeping articulation wakes when any of its shaped links is in the band.
		for (auto articulation : articulations)
		{
			if (!articulation->Sleeping())
				continue;

			for (int i = 0, iend = articulation->LinkCount(); i != iend; ++i)
			{
				auto const link = articulation->LinkAt(i);
				auto const& desc = articulation->LinkDescription(link);
				if (desc.m_shape == nullptr)
					continue;

				auto const o2w = articulation->LinkToWorld(link) * desc.m_shape_to_link;
				if (!in_band(o2w * collision::CalcBBox(*desc.m_shape)))
					continue;

				articulation->Wake();
				break;
			}
		}
	}

	// Make the water hull of a rigid body.
	bool GpuWater::MakeHull(RigidBody const& body, int step_index, Hull& hull)
	{
		// Static bodies do not move, and sleeping bodies are woken by WakeInSurfaceBand when the water can reach them.
		if (body.InvMass() == 0.0f || body.Sleeping() || !body.HasShape())
			return false;

		// The centre of mass moves at the linear momentum over the mass, which avoids forming the world-space inverse inertia.
		hull = Hull{
			.m_shape = &body.Shape(),
			.m_o2w = body.O2W(),
			.m_bbox_ws = body.BBoxWS(),
			.m_com_ws = body.CentreOfMassPositionWS(),
			.m_speed = Length(body.MomentumWS().lin) * body.InvMass(),
			.m_fall_acceleration = Length(body.GravityWS().w0()),
			.m_step_index = step_index,
			.m_key = HullKey{ &body, -1 },
		};
		return true;
	}

	// Make the water hull of an articulation link.
	bool GpuWater::MakeHull(Articulation const& articulation, LinkHandle link, int step_index, Hull& hull)
	{
		// Links without a collision shape displace no water.
		auto const& desc = articulation.LinkDescription(link);
		if (desc.m_shape == nullptr)
			return false;

		// The link velocity is in the link frame; the speed of the centre of mass does not depend on the frame it is measured in.
		auto const l2w = articulation.LinkToWorld(link);
		auto const o2w = l2w * desc.m_shape_to_link;
		auto const com_ls = desc.m_inertia.CoM();
		auto const velocity = articulation.LinkVelocity(link);
		hull = Hull{
			.m_shape = desc.m_shape,
			.m_o2w = o2w,
			.m_bbox_ws = o2w * collision::CalcBBox(*desc.m_shape),
			.m_com_ws = l2w * com_ls.w1(),
			.m_speed = Length(velocity.lin + Cross(velocity.ang, com_ls.w0())),
			.m_fall_acceleration = Length(articulation.GravityWS(link).w0()),
			.m_step_index = step_index,
			.m_key = HullKey{ &articulation, static_cast<int>(link.m_index) },
		};
		return true;
	}

	// Conservatively test whether the hull can reach the surface of 'field' before the frame ends.
	bool GpuWater::MayBeWet(Hull const& hull, terrain::water::WaterField const& field, float elapsed_s)
	{
		// Rotation about the centre of mass cannot move any point further than this sphere, whatever the frame's spin.
		auto const reach = Length(hull.m_bbox_ws.m_centre - hull.m_com_ws) + Length(hull.m_bbox_ws.m_radius.w0());

		// Allow for the current velocity plus twice the body's own gravity displacement over the frame, to cover other bounded forces.
		auto const drop = hull.m_speed * elapsed_s + hull.m_fall_acceleration * elapsed_s * elapsed_s;

		// The highest surface over the area the hull can cover this frame. Over dry land this is the still-water level, so land bodies cost no GPU work.
		auto const range = static_cast<double>(reach + drop);
		auto const lo = terrain::v2d{hull.m_com_ws.x - range, hull.m_com_ws.y - range};
		auto const hi = terrain::v2d{hull.m_com_ws.x + range, hull.m_com_ws.y + range};
		return static_cast<double>(hull.m_com_ws.z) - range < field.MaxHeight(lo, hi);
	}

	// Return the cached shape data, building it on first use.
	GpuWater::CompositeShape const& GpuWater::GetOrCreateShape(collision::Shape const& shape)
	{
		// Collision shapes are immutable while cached, so pointer identity is a stable key (see Reset).
		if (auto iter = m_shapes.find(&shape); iter != m_shapes.end())
			return *iter->second;

		// Flatten the shape into the primitive layout used by the shader.
		auto const hull = buoyancy::FlattenShape(shape);
		if (hull.Empty())
			throw std::runtime_error("Water hull contains no primitives");

		// Build the cull-classified surface plans in the same primitive order as the flattened descriptors. The largest cell of each primitive
		// bounds how far above the water a wet sample can be.
		auto data = std::make_unique<CompositeShape>();
		auto const plans = buoyancy::BuildHullSurfacePlans(shape, m_config.m_surface_spacing);
		auto half_cell_heights = std::vector<float>(plans.size(), 0.0f);
		auto patch_count = size_t{0};
		for (size_t k = 0; k != plans.size(); ++k)
		{
			auto const& plan = plans[k].m_plan;
			if (plan.m_count > surface::MaxSampleCount - static_cast<uint32_t>(data->m_total_surface_samples))
				throw std::runtime_error("Water hull exceeds the representable surface sample count");

			for (auto const& patch : plan.m_patches)
				half_cell_heights[k] = std::max(half_cell_heights[k], 0.5f * std::sqrt(surface::MaxSampleArea(patch)));

			data->m_total_surface_samples += static_cast<int>(plan.m_count);
			patch_count += plan.m_patches.size();
		}
		if (m_patches.m_cpu.size() + patch_count > static_cast<size_t>(std::numeric_limits<int>::max()) / sizeof(surface::SurfacePatch))
			throw std::runtime_error("Water surface patches exceed the addressable buffer size");

		// Bounds and a scale-relative slack for the dry cull.
		auto const bbox = collision::CalcBBox(shape);
		data->m_bbox_os = bbox;
		data->m_eps = buoyancy::HullInsideSlack(shape);
		data->m_max_half_cell_height = std::ranges::max(half_cell_heights);

		// Append the geometry to the resident arrays. The shader reads only the primitive descriptors, patches, and visibility masks, so only
		// the patch and mask offsets need rebasing into the shared arrays.
		data->m_prim_base = isize(m_prims.m_cpu);
		data->m_prim_count = isize(hull.m_primitives);
		for (int k = 0; k != data->m_prim_count; ++k)
		{
			m_prims.m_cpu.push_back(hull.m_primitives[k]);

			auto const& plan = plans[k].m_plan;
			m_records.m_cpu.push_back(PrimRecord{
				.m_count = static_cast<int>(plan.m_count),
				.m_patch_begin = isize(m_patches.m_cpu),
				.m_patch_count = isize(plan.m_patches),
				.m_half_cell_height = half_cell_heights[k],
				.m_visible_base = isize(m_visible.m_cpu),
			});
			m_patches.m_cpu.insert(m_patches.m_cpu.end(), plan.m_patches.begin(), plan.m_patches.end());
			m_visible.m_cpu.insert(m_visible.m_cpu.end(), plans[k].m_visible.begin(), plans[k].m_visible.end());
		}

		auto& slot = m_shapes[&shape];
		slot = std::move(data);
		return *slot;
	}

	// Upload the newly appended entries of a resident array.
	template <typename T>
	void GpuWater::SyncResident(GpuJob& job, ResidentArray<T>& arr, std::string_view name)
	{
		// A replaced buffer is empty, so everything is uploaded into it. Capacity doubles so that a stream of new shapes costs amortised
		// linear uploads. An empty array still gets one entry because the shader binding needs a valid address.
		auto const count = isize(arr.m_cpu);
		if (arr.m_res == nullptr || count > arr.m_capacity)
		{
			arr.m_capacity = std::max({ count, 2 * arr.m_capacity, 1 });
			arr.m_res = m_gpu.CreateResource(ResDesc::Buf<T>(arr.m_capacity, {}), job, name);
			arr.m_uploaded = 0;
		}
		if (arr.m_uploaded == count)
			return;

		// Copy only the tail. Earlier entries are unchanged, so work already recorded that reads them is unaffected. Entries changed in place
		// lower 'm_uploaded', which only happens between steps.
		auto const added = count - arr.m_uploaded;
		auto upload = job.m_upload.Alloc<T>(added);
		memcpy(upload.ptr<T>(), arr.m_cpu.data() + arr.m_uploaded, added * sizeof(T));
		job.m_barriers.Transition(arr.m_res.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(arr.m_res.get(), static_cast<UINT64>(arr.m_uploaded) * sizeof(T), upload);
		job.m_barriers.Transition(arr.m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
		arr.m_uploaded = count;
	}

	// Grow the output buffers as needed.
	void GpuWater::EnsureGpuCapacity(GpuJob& job, int partial_capacity, int diagnostic_capacity)
	{
		// Buffers only grow, so steady-state frames allocate nothing.
		if (partial_capacity > m_partial_capacity)
		{
			m_r_partials = CreateUavBuffer<GpuWaterPartial>(m_gpu, partial_capacity, "Physics.Water.Partials");
			m_partial_capacity = partial_capacity;
		}
		if (diagnostic_capacity > m_diagnostic_capacity)
		{
			m_r_diagnostics = CreateUavBuffer<GpuWaterDiagnostic>(m_gpu, diagnostic_capacity, "Physics.Water.Diagnostics");
			m_diagnostic_capacity = diagnostic_capacity;
		}

		// The unused diagnostics buffer is never accessed, so it only needs a transition when diagnostics are enabled.
		job.m_barriers.Transition(m_r_partials.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		if (m_config.m_enable_diagnostics)
			job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

		job.m_barriers.Commit();
	}
}
