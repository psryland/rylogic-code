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
			int m_groups_per_hull;
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

		// Per-hull header. Mirrors HLSL BuoySurfHeader. 'm_prim_base' and 'm_prim_count' index the frame's concatenated primitive array, and
		// 'm_element_base' and 'm_element_count' index the frame's concatenated local water elements.
		struct GpuWaterHeader
		{
			int m_body_index;
			int m_prim_base;
			int m_prim_count;
			int m_total_surface_samples;
			float m_eps;
			int m_element_base;
			int m_element_count;
			float m_water_max_height;
			v4 m_bbox_centre_os;
			v4 m_bbox_radius_os;
		};
		static_assert(sizeof(GpuWaterHeader) == 64);

		// Per-primitive surface counts, compact patch range, and half of the largest cell height (for the dry early-out). Mirrors HLSL BuoySurfPrimRecord.
		struct GpuWaterPrimRecord
		{
			int m_count;
			int m_patch_begin;
			int m_patch_count;
			float m_half_cell_height;
		};
		static_assert(sizeof(GpuWaterPrimRecord) == 16);

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
		, m_active()
		, m_active_shapes()
		, m_active_shape_index()
		, m_elements()
		, m_groups_per_hull()
		, m_elements_addr(nullptr)
		, m_face_planes_addr(nullptr)
		, m_headers_addr(nullptr)
		, m_prims_addr(nullptr)
		, m_records_addr(nullptr)
		, m_patches_addr(nullptr)
		, m_r_partials()
		, m_r_diagnostics()
		, m_partial_capacity()
		, m_diagnostic_capacity()
		, m_pending_readback()
		, m_pending_keys()
		, m_diagnostics()
	{
		// The sample pass reads the bodies, water elements, primitives, face planes (for the sibling cull), headers, patches, and primitive records,
		// and writes one partial sum per thread group. Root parameter order must match the binding order in Apply.
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
			m_shapes.clear();

		m_config = std::move(config);
	}

	// Forget cached shape data and diagnostics.
	void GpuWater::Reset()
	{
		// Pointers to caller-owned shapes may now be dangling or reused.
		m_shapes.clear();
		m_diagnostics.clear();
	}

	// Select and stage the hulls that may be wet this frame.
	void GpuWater::BeginFrame(GpuJob& job, std::span<Hull const> hulls, float elapsed_s)
	{
		// Start an empty frame. Diagnostics are only recorded for frames that dispatch.
		m_active.clear();
		m_active_shapes.clear();
		m_active_shape_index.clear();
		m_elements.clear();
		m_pending_keys.clear();
		m_pending_readback = {};

		// Select the wet hulls and find the local water elements of each.
		auto const& field = m_config.m_field;
		auto const level = field.Level();
		auto max_samples = 0;
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
			auto const& shape = GetOrCreateShape(*hull.m_shape);
			auto const g = hull.m_gravity_ws.w0();
			if (LengthSq(g) > 1e-12f)
			{
				auto const lowest = LowestAlong(hull.m_o2w, shape.m_bbox_os, -Normalise(g));
				if (lowest > water_max_height + std::max(shape.m_eps, 1e-4f) + shape.m_max_half_cell_height)
					continue;
			}

			// Bodies sharing a shape share one block of uploaded geometry.
			auto [iter, added] = m_active_shape_index.try_emplace(&shape, isize(m_active_shapes));
			if (added)
				m_active_shapes.push_back(&shape);

			m_active.push_back(ActiveHull{
				.m_step_index = hull.m_step_index,
				.m_shape_index = iter->second,
				.m_element_base = isize(m_elements),
				.m_element_count = isize(elements),
				.m_water_max_height = water_max_height,
				.m_key = hull.m_key,
			});
			m_elements.insert(m_elements.end(), elements.begin(), elements.end());
			max_samples = std::max(max_samples, shape.m_total_surface_samples);
		}
		if (m_active.empty())
			return;

		// Size the dispatch. Groups are laid out per hull with a uniform count, sized for the busiest hull.
		auto const hull_count = isize(m_active);
		if (hull_count > 65535)
			throw std::runtime_error("Water hull count exceeds the dispatch dimension");

		m_groups_per_hull = std::clamp((max_samples + SampleThreadCount - 1) / SampleThreadCount, 1, ReduceThreadCount);
		EnsureGpuCapacity(job, hull_count * m_groups_per_hull, m_config.m_enable_diagnostics ? hull_count : 1);

		// Count the geometry of the unique shapes. Each upload has at least one entry because empty arrays still need a valid address.
		auto total_prims = 0;
		auto total_face_planes = 0;
		auto total_patches = size_t{0};
		for (auto shape : m_active_shapes)
		{
			total_prims += isize(shape->m_hull.m_primitives);
			total_face_planes += isize(shape->m_hull.m_face_planes);
			for (auto const& plan : shape->m_surface_plans)
				total_patches += plan.m_patches.size();
		}
		if (total_patches > static_cast<size_t>(std::numeric_limits<int>::max()) / sizeof(surface::SurfacePatch))
			throw std::runtime_error("Water surface patch upload exceeds the addressable buffer size");

		auto upload_face_planes = job.m_upload.Alloc<v4>(std::max(total_face_planes, 1));
		auto upload_prims = job.m_upload.Alloc<buoyancy::GpuPrimitive>(std::max(total_prims, 1));
		auto upload_records = job.m_upload.Alloc<GpuWaterPrimRecord>(std::max(total_prims, 1));
		auto upload_patches = job.m_upload.Alloc<surface::SurfacePatch>(std::max(static_cast<int>(total_patches), 1));
		auto upload_headers = job.m_upload.Alloc<GpuWaterHeader>(hull_count);
		auto upload_elements = job.m_upload.Alloc<terrain::water::WaterFieldElement>(std::max(isize(m_elements), 1));

		// Concatenate each unique shape's geometry and sample records. Face-vertex indices stay relative to each primitive's vertex block,
		// and the surface-vertex array is not bound, so only the face offset needs shifting into the shared buffer.
		auto face_planes = upload_face_planes.ptr<v4>();
		auto prims = upload_prims.ptr<buoyancy::GpuPrimitive>();
		auto records = upload_records.ptr<GpuWaterPrimRecord>();
		auto patches = upload_patches.ptr<surface::SurfacePatch>();
		auto prim_bases = std::vector<int>(m_active_shapes.size());
		auto prim_base = 0;
		auto face_base = 0;
		auto patch_base = 0;
		for (int s = 0, send = isize(m_active_shapes); s != send; ++s)
		{
			auto const& shape = *m_active_shapes[s];
			auto const& hull = shape.m_hull;
			std::copy(hull.m_face_planes.begin(), hull.m_face_planes.end(), face_planes + face_base);
			for (int k = 0, kend = isize(hull.m_primitives); k != kend; ++k)
			{
				auto prim = hull.m_primitives[k];
				prim.m_face_ofs += face_base;
				prims[prim_base + k] = prim;

				auto const& plan = shape.m_surface_plans[k];
				records[prim_base + k] = GpuWaterPrimRecord{
					.m_count = static_cast<int>(plan.m_count),
					.m_patch_begin = patch_base,
					.m_patch_count = isize(plan.m_patches),
					.m_half_cell_height = shape.m_half_cell_heights[k],
				};
				for (auto const& patch : plan.m_patches)
					patches[patch_base++] = patch;
			}

			prim_bases[s] = prim_base;
			prim_base += isize(hull.m_primitives);
			face_base += isize(hull.m_face_planes);
		}

		// Emit one header per hull referencing its shape block and local elements.
		auto headers = upload_headers.ptr<GpuWaterHeader>();
		for (int i = 0; i != hull_count; ++i)
		{
			auto const& active = m_active[i];
			auto const& shape = *m_active_shapes[active.m_shape_index];
			headers[i] = GpuWaterHeader{
				.m_body_index = active.m_step_index,
				.m_prim_base = prim_bases[active.m_shape_index],
				.m_prim_count = isize(shape.m_hull.m_primitives),
				.m_total_surface_samples = shape.m_total_surface_samples,
				.m_eps = shape.m_eps,
				.m_element_base = active.m_element_base,
				.m_element_count = active.m_element_count,
				.m_water_max_height = active.m_water_max_height,
				.m_bbox_centre_os = shape.m_bbox_os.m_centre,
				.m_bbox_radius_os = shape.m_bbox_os.m_radius,
			};
		}

		// Stage the local elements. A flat field has none, so the single bound entry is zeroed.
		memset(upload_elements.ptr<std::byte>(), 0, sizeof(terrain::water::WaterFieldElement));
		std::copy(m_elements.begin(), m_elements.end(), upload_elements.ptr<terrain::water::WaterFieldElement>());

		m_face_planes_addr = upload_face_planes;
		m_prims_addr = upload_prims;
		m_records_addr = upload_records;
		m_patches_addr = upload_patches;
		m_headers_addr = upload_headers;
		m_elements_addr = upload_elements;
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
			.m_groups_per_hull = m_groups_per_hull,
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
		};

		// Evaluate the surface samples into one partial record per hull and thread group.
		auto& cmd_list = job.m_cmd_list;
		cmd_list.SetPipelineState(m_surface_step.m_pso.get());
		cmd_list.SetComputeRootSignature(m_surface_step.m_sig.get());
		cmd_list.AddComputeRoot32BitConstants(cb);
		cmd_list.AddComputeRootUnorderedAccessView(bodies);
		cmd_list.AddComputeRootShaderResourceView(m_elements_addr);
		cmd_list.AddComputeRootShaderResourceView(m_prims_addr);
		cmd_list.AddComputeRootShaderResourceView(m_face_planes_addr);
		cmd_list.AddComputeRootShaderResourceView(m_headers_addr);
		cmd_list.AddComputeRootShaderResourceView(m_patches_addr);
		cmd_list.AddComputeRootShaderResourceView(m_records_addr);
		cmd_list.AddComputeRootUnorderedAccessView(m_r_partials.get());
		cmd_list.Dispatch(m_groups_per_hull, cb.m_hull_count, 1);
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

		hull = Hull{
			.m_shape = &body.Shape(),
			.m_o2w = body.O2W(),
			.m_bbox_ws = body.BBoxWS(),
			.m_com_ws = body.CentreOfMassPositionWS(),
			.m_speed = Length(body.VelocityWS().lin),
			.m_gravity_ws = body.GravityWS(),
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
			.m_gravity_ws = articulation.GravityWS(link),
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

		// Allow for the current velocity plus twice gravity's displacement over the frame, to cover other bounded forces.
		auto const drop = hull.m_speed * elapsed_s + Length(hull.m_gravity_ws.w0()) * elapsed_s * elapsed_s;

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
		auto& slot = m_shapes[&shape];
		if (slot != nullptr)
			return *slot;

		// Flatten the shape into the primitive layout used by the shader.
		auto data = std::make_unique<CompositeShape>();
		data->m_hull = buoyancy::FlattenShape(shape);
		if (data->m_hull.Empty())
		{
			m_shapes.erase(&shape);
			throw std::runtime_error("Water hull contains no primitives");
		}

		// Build the surface plans in the same primitive order as the flattened descriptors. The largest cell of each primitive bounds how far above
		// the water a wet sample can be.
		auto const prims = buoyancy::CollectPrimitives(shape);
		data->m_surface_plans.resize(prims.size());
		data->m_half_cell_heights.assign(prims.size(), 0.0f);
		for (size_t k = 0; k != prims.size(); ++k)
		{
			auto& plan = data->m_surface_plans[k];
			plan = buoyancy::BuildSurfacePlan(*prims[k], m_config.m_surface_spacing);
			if (plan.m_count > surface::MaxSampleCount - static_cast<uint32_t>(data->m_total_surface_samples))
			{
				m_shapes.erase(&shape);
				throw std::runtime_error("Water hull exceeds the representable surface sample count");
			}

			for (uint32_t i = 0; i != plan.m_count; ++i)
				data->m_half_cell_heights[k] = std::max(data->m_half_cell_heights[k], 0.5f * std::sqrt(surface::EmitSurfaceSample(plan, i).m_darea));

			data->m_total_surface_samples += static_cast<int>(plan.m_count);
		}

		// Bounds for the dry cull and a scale-relative slack for the sibling inside test.
		auto const bbox = collision::CalcBBox(shape);
		data->m_bbox_os = bbox;
		data->m_eps = std::max(1e-6f, MaxElement(bbox.m_radius.w0()) * 1e-5f);
		data->m_max_half_cell_height = std::ranges::max(data->m_half_cell_heights);

		slot = std::move(data);
		return *slot;
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
