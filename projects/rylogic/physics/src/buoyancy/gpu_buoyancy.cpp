//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/physics/buoyancy/gpu_buoyancy.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"
#include "pr/compute/compute_pso.h"
#include "pr/compute/compute_step.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/utility/root_signature.h"

#include "src/utility/gpu.h"

namespace pr::physics
{
	namespace
	{
		// Sampled-composite volume-pass tunables. The total number of volume samples per hull is split
		// across the hull's primitives proportional to their volume; positions are
		// hash-derived per frame, but counts are fixed at registration. The per-hull group count is
		// ceil(total/thread_count) and must not exceed the reduce thread count (the reducer sums one
		// partial per group on a single thread group).
		static constexpr int BuoyancyVolumeSampleCount = 8192;
		static constexpr int BuoyancyVolumeThreadCount = 256;
		static constexpr int BuoyancyVolumeReduceThreadCount = 128;

		// Number of surface samples distributed across a composite hull's primitives for the drag
		// pass (proportional to per-primitive area). 8192 / 256 = 32 groups < 128 reduce limit.
		static constexpr int BuoyancySurfaceSampleCount = 8192;

		struct CBufGpuBuoyancy
		{
			int m_hull_count;
			int m_groups_per_hull;
			int m_water_field_count;
			float m_time_s;
			float m_water_level;
			float m_fluid_density;
			float m_linear_drag_coefficient;
			float m_angular_drag_coefficient;
			float m_quadratic_drag_coefficient;
			float m_tangential_drag_coefficient;
			float m_time_step_s;
			int m_enable_diagnostics;
		};
		static_assert(sizeof(CBufGpuBuoyancy) % sizeof(uint32_t) == 0);

		struct GpuBuoyancyWave
		{
			v4 m_direction_wavelength_phase_speed;
			v4 m_amplitude;
		};
		static_assert(sizeof(GpuBuoyancyWave) == 32);

		struct GpuBuoyancyPartial
		{
			v4 m_force_ws;
			v4 m_torque_ws;
			v4 m_moment_ws_volume;
		};
		static_assert(sizeof(GpuBuoyancyPartial) == 48);

		struct GpuBuoyancyDiagnostic
		{
			v4 m_force_ws;
			v4 m_torque_ws;
			v4 m_centre_buoyancy_ws;
			float m_volume_m3;
			int m_valid;
			float m_pad0;
			float m_pad1;
		};
		static_assert(sizeof(GpuBuoyancyDiagnostic) == 64);

		// Per-hull header for the sampled-composite volume pass. Mirrors HLSL BuoyVolHeader. The body
		// index is the body's STEP index (into the engine body list) resolved at dispatch time; the
		// hull id is the stable registration index used to seed the sample hash so positions do not
		// jitter when bodies are reordered. m_prim_base/m_prim_count index the concatenated primitive
		// array uploaded for this dispatch.
		struct GpuBuoyVolHeader
		{
			int m_body_index;
			int m_prim_base;
			int m_prim_count;
			int m_total_volume_samples;
			uint32_t m_hull_id;
			float m_eps;
			int m_pad0;
			int m_pad1;
		};
		static_assert(sizeof(GpuBuoyVolHeader) == 32);

		// Per-primitive sample bookkeeping for the volume pass. m_count is the number of volume
		// samples assigned to this primitive and m_dvol is the per-sample volume weight (primitive
		// volume / count). Mirrors HLSL BuoyVolPrimRecord.
		struct GpuBuoyVolPrimRecord
		{
			int m_count;
			float m_dvol;
		};
		static_assert(sizeof(GpuBuoyVolPrimRecord) == 8);

		// Per-hull header for the sampled-composite surface (drag) pass. Mirrors HLSL BuoySurfHeader.
		// Layout-identical to GpuBuoyVolHeader, but m_total_surface_samples counts surface samples and
		// the primitive block is indexed into the surface record array.
		struct GpuBuoySurfHeader
		{
			int m_body_index;
			int m_prim_base;
			int m_prim_count;
			int m_total_surface_samples;
			uint32_t m_hull_id;
			float m_eps;
			int m_pad0;
			int m_pad1;
		};
		static_assert(sizeof(GpuBuoySurfHeader) == 32);

		// Per-primitive sample bookkeeping for the surface pass. m_count is the number of surface
		// samples assigned to this primitive and m_darea is the per-sample area weight (primitive
		// area / count). Mirrors HLSL BuoySurfPrimRecord.
		struct GpuBuoySurfPrimRecord
		{
			int m_count;
			float m_darea;
		};
		static_assert(sizeof(GpuBuoySurfPrimRecord) == 8);


		// Throw if a floating-point scene value cannot be safely used by the GPU buoyancy pass.
		void ValidateFinite(float value, char const* name)
		{
			if (!std::isfinite(value))
			{
				throw std::runtime_error(pr::FmtS("GPU buoyancy water surface '%s' must be finite", name));
			}
		}

		// Throw if a water wave cannot be evaluated deterministically on CPU and GPU.
		void ValidateSineWave(GpuBuoyancy::SineWave const& wave)
		{
			ValidateFinite(wave.m_direction.x, "direction.x");
			ValidateFinite(wave.m_direction.y, "direction.y");
			ValidateFinite(wave.m_wavelength, "wavelength");
			ValidateFinite(wave.m_amplitude, "amplitude");
			ValidateFinite(wave.m_phase_speed, "phase_speed");
			if (LengthSq(wave.m_direction) <= tiny<float>)
			{
				throw std::runtime_error("GPU buoyancy water wave direction must be non-zero");
			}
			if (wave.m_wavelength <= 0.0f)
			{
				throw std::runtime_error("GPU buoyancy water wave wavelength must be positive");
			}
		}

		// Throw if a water surface exceeds the current GPU dispatch limits or contains invalid wave data.
		void ValidateWaterSurface(GpuBuoyancy::WaterSurface const& water_surface)
		{
			ValidateFinite(water_surface.m_level, "level");
			if (std::ssize(water_surface.m_waves) > GpuBuoyancy::MaxWaterWaveCount)
			{
				throw std::runtime_error(pr::FmtS("GPU buoyancy supports at most %d water waves", GpuBuoyancy::MaxWaterWaveCount));
			}

			for (auto const& wave : water_surface.m_waves)
			{
				ValidateSineWave(wave);
			}
		}

		// Create a default-heap UAV buffer used by the diagnostic compute passes.
		template <typename T>
		D3DPtr<ID3D12Resource> CreateDefaultUavBuffer(ID3D12Device* device, int count, std::string_view name)
		{
			if (count <= 0)
			{
				throw std::runtime_error("GPU buoyancy buffer capacity must be positive");
			}

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
			auto const& heap_props = ::pr::compute::HeapProps::Default();
			Check(device->CreateCommittedResource(
				&heap_props,
				D3D12_HEAP_FLAG_NONE,
				&desc,
				D3D12_RESOURCE_STATE_COMMON,
				nullptr,
				__uuidof(ID3D12Resource),
				reinterpret_cast<void**>(res.address_of())));

			::pr::compute::DefaultResState(res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			::pr::compute::DebugName(res, name);
			return res;
		}

		// Validate a custom water-field shader contract before it is used to compile any pipelines.
		GpuBuoyancy::WaterFieldExtension ValidateWaterFieldExtension(GpuBuoyancy::WaterFieldExtension extension)
		{
			if (!extension.Enabled())
			{
				if (!extension.m_shader_include.empty() || extension.m_element_stride != 0)
					throw std::runtime_error("GpuBuoyancy water-field extension requires both a shader include and an element stride");

				return extension;
			}
			if (extension.m_element_stride <= 0 || extension.m_element_stride % 16 != 0)
				throw std::runtime_error("GpuBuoyancy water-field element stride must be a positive multiple of 16 bytes");

			return extension;
		}

		// Compile a runtime compute shader entry point from the physics buoyancy shader source.
		std::vector<uint8_t> CompileBuoyancyShader(wchar_t const* entry_point, GpuBuoyancy::WaterFieldExtension const& water_field_extension)
		{
			auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};
			auto compiler = ::pr::compute::ShaderCompiler{}
				.Source("src/buoyancy/gpu_buoyancy.hlsl", resolver)
				.HlslVersion(::pr::compute::EHlslVersion::Hlsl2021)
				.Define(L"SHADER_BUILD")
				.Optimise(true)
				.ShaderModel(L"cs_6_6")
				.EntryPoint(entry_point);

			// A quoted macro expands directly in '#include GPU_BUOYANCY_WATER_FIELD_INCLUDE', allowing
			// the existing resource resolver to locate application-owned HLSL without physics knowing its path.
			if (water_field_extension.Enabled())
			{
				auto const include = std::format(L"\"{}\"", Widen(water_field_extension.m_shader_include));
				compiler.Define(L"GPU_BUOYANCY_WATER_FIELD_INCLUDE", include);
			}
			return compiler.Compile();
		}

		// Create the sampled-composite volume-sample compute step. The root signature mirrors the
		// resource access order of CSBuoyancyVolumeSamples: constants (b0), the body accumulator (u0),
		// the water-field SRV (t1), the volume-pass SRVs (t2..t7), the tet CDF (t12), then the partials UAV
		// (u1). The legacy box-hull SRV (t0) and diagnostics UAV (u2) are unused by this kernel.
		::pr::compute::ComputeStep CreateVolumeStep(ID3D12Device* device, GpuBuoyancy::WaterFieldExtension const& water_field_extension)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t1)
				.SRV(hlsl::ESRVReg::t2)
				.SRV(hlsl::ESRVReg::t3)
				.SRV(hlsl::ESRVReg::t4)
				.SRV(hlsl::ESRVReg::t5)
				.SRV(hlsl::ESRVReg::t6)
				.SRV(hlsl::ESRVReg::t7)
				.SRV(hlsl::ESRVReg::t12)
				.UAV(hlsl::EUAVReg::u1)
				.Create(device, "Physics.GpuBuoyancy.Volume.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyVolumeSamples", water_field_extension)).Create(device, "Physics.GpuBuoyancy.Volume.PSO");
			return step;
		}

		// Create the sampled-composite volume-reduce compute step. The root signature mirrors the
		// resource access order of CSBuoyancyVolumeReduce: constants (b0), the body accumulator (u0),
		// the per-hull headers SRV (t2), the partials UAV (u1), then the diagnostics UAV (u2).
		::pr::compute::ComputeStep CreateVolumeReduceStep(ID3D12Device* device, GpuBuoyancy::WaterFieldExtension const& water_field_extension)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t2)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.Create(device, "Physics.GpuBuoyancy.VolumeReduce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyVolumeReduce", water_field_extension)).Create(device, "Physics.GpuBuoyancy.VolumeReduce.PSO");
			return step;
		}

		// Create the sampled-composite surface-sample (drag) compute step. The root signature mirrors
		// the resource access order of CSBuoyancyDragSurfaceSamples: constants (b0), the body
		// accumulator (u0), the water-field SRV (t1, for water height/velocity), the primitives SRV (t3) and
		// face planes SRV (t6, for the sibling cull), the four surface-pass SRVs (t8..t11), then the
		// partials UAV (u1). Root descriptors need not be contiguous.
		::pr::compute::ComputeStep CreateSurfaceStep(ID3D12Device* device, GpuBuoyancy::WaterFieldExtension const& water_field_extension)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t1)
				.SRV(hlsl::ESRVReg::t3)
				.SRV(hlsl::ESRVReg::t6)
				.SRV(hlsl::ESRVReg::t8)
				.SRV(hlsl::ESRVReg::t9)
				.SRV(hlsl::ESRVReg::t10)
				.SRV(hlsl::ESRVReg::t11)
				.UAV(hlsl::EUAVReg::u1)
				.Create(device, "Physics.GpuBuoyancy.Surface.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyDragSurfaceSamples", water_field_extension)).Create(device, "Physics.GpuBuoyancy.Surface.PSO");
			return step;
		}

		// Create the sampled-composite surface-reduce compute step. The root signature mirrors the
		// resource access order of CSBuoyancyDragSurfaceReduce: constants (b0), the body accumulator
		// (u0), water field (t1), the per-hull surface headers SRV (t8), the partials UAV (u1), then the
		// diagnostics UAV (u2). The surface reduce ADDS drag force/torque to the body and diagnostic.
		::pr::compute::ComputeStep CreateSurfaceReduceStep(ID3D12Device* device, GpuBuoyancy::WaterFieldExtension const& water_field_extension)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t1)
				.SRV(hlsl::ESRVReg::t8)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.Create(device, "Physics.GpuBuoyancy.SurfaceReduce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyDragSurfaceReduce", water_field_extension)).Create(device, "Physics.GpuBuoyancy.SurfaceReduce.PSO");
			return step;
		}
	}

	struct GpuBuoyancy::Impl
	{
		// Immutable geometry and sample plans shared by every body that references the same collision
		// shape. Collision shapes are immutable value blobs, so pointer identity is a stable cache key
		// while at least one body retains the shape.
		struct CompositeShape
		{
			buoyancy::CompositeHull m_hull;

			// Volume-pass sample plan, computed once at registration. m_vol_counts[k] is the number of
			// volume samples assigned to primitive k (proportional to its volume) and m_vol_dvol[k] is
			// the matching per-sample volume weight (primitive volume / count). m_total_volume_samples
			// is the sum of m_vol_counts. m_eps is the scale-relative inside-test slack for sibling culling.
			std::vector<int> m_vol_counts;
			std::vector<float> m_vol_dvol;
			int m_total_volume_samples = 0;
			float m_eps = 0.0f;

			// Registration-time body-space AABB enclosing every primitive in the hull. Used by the
			// host-side flat-water dry broadphase cull to conservatively test whether the whole body
			// is above the water line along its local up (-gravity) axis.
			BBox m_obb_os = BBox::Reset();

			// Surface-pass sample plan, computed once at registration. m_surf_counts[k] is the number
			// of surface samples assigned to primitive k (proportional to its surface area) and
			// m_surf_darea[k] is the matching per-sample area weight (primitive area / count).
			// m_total_surface_samples is the sum of m_surf_counts.
			std::vector<int> m_surf_counts;
			std::vector<float> m_surf_darea;
			int m_total_surface_samples = 0;
		};
		struct ShapeCacheKey
		{
			collision::Shape const* m_shape;
			int m_polytope_tessellation;

			friend bool operator ==(ShapeCacheKey const&, ShapeCacheKey const&) = default;
		};
		struct ShapeCacheHash
		{
			std::size_t operator()(ShapeCacheKey const& key) const noexcept
			{
				auto hash = std::hash<collision::Shape const*>{}(key.m_shape);
				return hash ^ (std::hash<int>{}(key.m_polytope_tessellation) + 0x9e3779b9U + (hash << 6) + (hash >> 2));
			}
		};
		struct CompositeSlot
		{
			// Per-body registration state. Expensive shape-derived data is shared; only identity,
			// lifetime, deterministic sample seeding, and wake-state bookkeeping remain per body.
			int m_generation = -1;
			bool m_active = false;
			RigidBody* m_body = nullptr;
			bool m_prev_never_sleep = false;
			uint32_t m_hull_id = 0;
			multicast::AutoSub m_shape_change_sub;
			std::shared_ptr<CompositeShape const> m_shape_data;
		};

		// A compact reference to a registered hull participating in the current dispatch. Instances live in
		// reusable CPU scratch storage, so collecting active hulls does not allocate once registration is complete.
		struct ActiveHull
		{
			int m_body_index;
			int m_body_generation;
			int m_body_step_index;
			uint32_t m_hull_id;
			int m_shape_index;
		};
		struct ActiveShape
		{
			std::shared_ptr<CompositeShape const> m_shape_data;
			int m_prim_base;
			int m_prim_count;
		};
		struct ActiveShapeBucket
		{
			CompositeShape const* m_shape_data;
			int m_shape_index;
		};
		struct ActiveHullStats
		{
			int m_max_volume_samples;
			int m_max_surface_samples;
		};

		// Metadata retained until the matching diagnostic readback completes.
		struct PendingDiagnostic
		{
			int m_body_index;
			int m_body_generation;
		};

		// GPU virtual addresses for the transient upload blocks consumed by the sampled-composite passes.
		struct DispatchAddresses
		{
			D3D12_GPU_VIRTUAL_ADDRESS m_water_field;
			D3D12_GPU_VIRTUAL_ADDRESS m_volume_headers;
			D3D12_GPU_VIRTUAL_ADDRESS m_volume_primitives;
			D3D12_GPU_VIRTUAL_ADDRESS m_volume_records;
			D3D12_GPU_VIRTUAL_ADDRESS m_volume_verts;
			D3D12_GPU_VIRTUAL_ADDRESS m_tets;
			D3D12_GPU_VIRTUAL_ADDRESS m_tet_cdf;
			D3D12_GPU_VIRTUAL_ADDRESS m_face_planes;
			D3D12_GPU_VIRTUAL_ADDRESS m_surface_headers;
			D3D12_GPU_VIRTUAL_ADDRESS m_surface_primitives;
			D3D12_GPU_VIRTUAL_ADDRESS m_surface_records;
			D3D12_GPU_VIRTUAL_ADDRESS m_surface_verts;
			D3D12_GPU_VIRTUAL_ADDRESS m_surface_face_verts;
		};

		// Device services and callbacks remain valid for the lifetime of this implementation.
		ID3D12Device* m_device;
		StepIndexResolver m_step_index_resolver;
		BodyStateResolver m_body_state_resolver;
		WaterFieldExtension m_water_field_extension;

		// Immutable compute pipelines and the engine subscription used to append buoyancy work to each GPU step.
		::pr::compute::ComputeStep m_volume_step;
		::pr::compute::ComputeStep m_volume_reduce_step;
		::pr::compute::ComputeStep m_surface_step;
		::pr::compute::ComputeStep m_surface_reduce_step;
		multicast::AutoSub m_external_force_sub;

		// Runtime settings and registration-owned hull data. Hull slots are indexed by stable body index.
		WaterSurface m_water_surface;
		std::vector<std::byte> m_water_field;
		float m_water_field_level;
		Config m_config;
		std::unordered_map<ShapeCacheKey, std::weak_ptr<CompositeShape const>, ShapeCacheHash> m_shape_cache;
		std::vector<CompositeSlot> m_composite_hulls;

		// Reusable host-side dispatch state. Capacity tracks m_composite_hulls and is established during
		// registration so the per-step Apply/DispatchComposite path only clears and repopulates storage.
		std::vector<ActiveHull> m_active_hulls;
		std::vector<ActiveShape> m_active_shapes;
		std::vector<ActiveShapeBucket> m_active_shape_lookup;
		std::vector<PendingDiagnostic> m_pending_diagnostics;
		::pr::compute::GpuReadbackBuffer::Allocation m_pending_readback;

		// Grow-only GPU outputs shared by the volume and surface passes. They are recreated only when a
		// dispatch exceeds the largest hull/group population seen previously.
		D3DPtr<ID3D12Resource> m_r_partials;
		D3DPtr<ID3D12Resource> m_r_diagnostics;
		int m_partial_capacity;
		int m_diagnostic_capacity;

		// Latest completed diagnostic values published for callers. Readback completion writes under the same
		// mutex used by LatestDiagnostics because rendering and physics scheduling may run independently.
		mutable std::mutex m_diagnostics_mutex;
		std::vector<Diagnostics> m_diagnostics;

		// Construct and subscribe the buoyancy compute pass.
		Impl(ID3D12Device* device, Engine& engine, Config const& config, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver, WaterFieldExtension water_field_extension)
			:m_device(device)
			,m_step_index_resolver(std::move(step_index_resolver))
			,m_body_state_resolver(std::move(body_state_resolver))
			,m_water_field_extension(ValidateWaterFieldExtension(std::move(water_field_extension)))
			,m_volume_step(CreateVolumeStep(device, m_water_field_extension))
			,m_volume_reduce_step(CreateVolumeReduceStep(device, m_water_field_extension))
			,m_surface_step(CreateSurfaceStep(device, m_water_field_extension))
			,m_surface_reduce_step(CreateSurfaceReduceStep(device, m_water_field_extension))
			,m_external_force_sub(engine.ExternalForces += [this](Engine& sender, Engine::ExternalForceArgs const& args)
			{
				Apply(sender, args);
			})
			,m_water_surface()
			,m_water_field()
			,m_water_field_level()
			,m_config()
			,m_shape_cache()
			,m_composite_hulls()
			,m_active_hulls()
			,m_active_shapes()
			,m_active_shape_lookup()
			,m_pending_diagnostics()
			,m_pending_readback()
			,m_r_partials()
			,m_r_diagnostics()
			,m_partial_capacity()
			,m_diagnostic_capacity()
			,m_diagnostics_mutex()
			,m_diagnostics()
		{
			if (m_device == nullptr)
			{
				throw std::runtime_error("GpuBuoyancy requires a D3D12 device");
			}
			if (!m_step_index_resolver)
			{
				throw std::runtime_error("GpuBuoyancy requires a body step-index resolver");
			}
			if (!m_body_state_resolver)
			{
				throw std::runtime_error("GpuBuoyancy requires a body-state resolver");
			}

			// Validate and apply the initial config through the same path used by SetConfig so all
			// invariants (finite values, non-negative density) are checked once at construction.
			SetConfig(config);
		}

		// Destroy the buoyancy pass after all physics GPU work has completed.
		~Impl() = default;

		// Grow registration storage and its per-step scratch capacity outside the dispatch hot path.
		void EnsureHostCapacity(int slot_count)
		{
			if (slot_count > static_cast<int>(m_composite_hulls.size()))
			{
				m_composite_hulls.resize(static_cast<std::size_t>(slot_count));
			}

			// Active and pending populations cannot exceed the number of registration slots. Reserving to the
			// slot vector's geometric capacity avoids both per-step allocation and repeated exact-size growth.
			auto const slot_capacity = m_composite_hulls.capacity();
			m_active_hulls.reserve(slot_capacity);
			m_active_shapes.reserve(slot_capacity);
			m_pending_diagnostics.reserve(slot_capacity);

			// Keep the open-addressed table at least half empty so shared-shape lookup remains
			// effectively constant time without allocating hash-map nodes during each dispatch.
			auto bucket_count = std::size_t{1};
			while (bucket_count < 2 * slot_capacity)
				bucket_count *= 2;

			if (bucket_count > m_active_shape_lookup.size())
				m_active_shape_lookup.resize(bucket_count);
		}

		// Return shared derived data for a collision shape, creating it once per shape pointer and
		// tessellation setting. Weak cache entries do not extend the source shape's lifetime.
		std::shared_ptr<CompositeShape const> GetOrCreateCompositeShape(collision::Shape const& shape)
		{
			auto const key = ShapeCacheKey{ &shape, m_config.m_polytope_tessellation };
			if (auto iter = m_shape_cache.find(key); iter != m_shape_cache.end())
			{
				if (auto existing = iter->second.lock())
					return existing;
			}

			auto flattened = buoyancy::FlattenShape(shape, m_config.m_polytope_tessellation);
			if (flattened.Empty())
				throw std::runtime_error("Composite buoyancy hull contains no primitives");

			auto const prims = buoyancy::CollectPrimitives(shape);
			auto volumes = std::vector<float>(prims.size(), 0.0f);
			auto areas = std::vector<float>(prims.size(), 0.0f);
			for (std::size_t k = 0; k != prims.size(); ++k)
			{
				volumes[k] = buoyancy::PrimitiveVolume(*prims[k]);
				areas[k] = buoyancy::PrimitiveArea(*prims[k]);
			}

			// Build the volume plan in the same primitive order as the flattened GPU descriptors.
			auto vol_counts = buoyancy::DistributeCounts(volumes, BuoyancyVolumeSampleCount);
			auto vol_dvol = std::vector<float>(prims.size(), 0.0f);
			auto total_volume_samples = 0;
			for (std::size_t k = 0; k != prims.size(); ++k)
			{
				auto const count = vol_counts[k];
				vol_dvol[k] = count > 0 ? volumes[k] / static_cast<float>(count) : 0.0f;
				total_volume_samples += count;
			}

			// Build the matching surface plan used by the drag pass.
			auto surf_counts = buoyancy::DistributeCounts(areas, BuoyancySurfaceSampleCount);
			auto surf_darea = std::vector<float>(prims.size(), 0.0f);
			auto total_surface_samples = 0;
			for (std::size_t k = 0; k != prims.size(); ++k)
			{
				auto const count = surf_counts[k];
				surf_darea[k] = count > 0 ? areas[k] / static_cast<float>(count) : 0.0f;
				total_surface_samples += count;
			}

			auto const bbox = collision::CalcBBox(shape);
			auto const extent = MaxElement(bbox.m_radius.w0());

			auto data = std::make_shared<CompositeShape>();
			data->m_hull = std::move(flattened);
			data->m_vol_counts = std::move(vol_counts);
			data->m_vol_dvol = std::move(vol_dvol);
			data->m_total_volume_samples = total_volume_samples;
			data->m_eps = std::max(1e-6f, extent * 1e-5f);
			data->m_obb_os = bbox;
			data->m_surf_counts = std::move(surf_counts);
			data->m_surf_darea = std::move(surf_darea);
			data->m_total_surface_samples = total_surface_samples;
			m_shape_cache[key] = data;
			return data;
		}

		// Refresh a live registration after its rigid body adopts a new collision shape.
		void HandleShapeChange(int body_index, int body_generation, ChangeEventArgs<collision::Shape const*> const& args)
		{
			if (args.m_before)
				return;

			auto& slot = m_composite_hulls[body_index];
			if (!slot.m_active || slot.m_generation != body_generation)
				return;

			try
			{
				if (args.m_value == nullptr)
					throw std::runtime_error("A buoyancy-registered rigid body must have a collision shape");

				slot.m_shape_data = GetOrCreateCompositeShape(*args.m_value);
			}
			catch (...)
			{
				slot.m_shape_data.reset();
				throw;
			}

			auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
			m_diagnostics[body_index] = Diagnostics{};
			m_diagnostics[body_index].m_body_index = body_index;
			m_diagnostics[body_index].m_body_generation = body_generation;
		}

		// Register a rigid body's collision shape as its buoyancy hull.
		void RegisterCompositeHull(RigidBody& body, int body_index, int body_generation)
		{
			if (body_index < 0 || body_generation < 0)
				throw std::runtime_error("Invalid body handle for buoyancy hull registration");
			if (!body.HasShape())
				throw std::runtime_error("A buoyancy-registered rigid body must have a collision shape");

			// Composite slots are indexed by stable body index. Establish matching scratch capacity here so
			// collecting those slots during a physics step cannot grow a vector.
			EnsureHostCapacity(body_index + 1);

			auto& slot = m_composite_hulls[body_index];
			if (slot.m_active)
				throw std::runtime_error("A buoyancy hull is already registered for this body");

			auto shape_data = GetOrCreateCompositeShape(body.Shape());
			auto shape_change_sub = multicast::AutoSub(body.ShapeChange += [this, body_index, body_generation](RigidBody&, ChangeEventArgs<collision::Shape const*> const& args)
			{
				HandleShapeChange(body_index, body_generation, args);
			});

			{
				auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
				if (body_index >= static_cast<int>(m_diagnostics.size()))
					m_diagnostics.resize(static_cast<std::size_t>(body_index + 1));

				auto& diagnostic = m_diagnostics[body_index];
				diagnostic = Diagnostics{};
				diagnostic.m_body_index = body_index;
				diagnostic.m_body_generation = body_generation;
			}

			slot.m_generation = body_generation;
			slot.m_active = true;
			slot.m_hull_id = static_cast<uint32_t>(body_index);
			slot.m_shape_data = std::move(shape_data);
			slot.m_shape_change_sub = std::move(shape_change_sub);

			// Keep the body awake for the lifetime of the registration. Buoyancy is a continuous
			// environmental force: the engine bails out of the GPU pipeline (and therefore
			// ExternalForces) when every dynamic body is asleep, which would freeze a floater in
			// place even as waves pass under it. Remember the prior flag so unregister can restore it.
			slot.m_body = &body;
			slot.m_prev_never_sleep = body.NeverSleep();
			body.NeverSleep(true);
			body.Wake();
		}

		// Remove the buoyancy hull for a stable physics body index.
		void UnregisterHull(int body_index, int body_generation) noexcept
		{
			if (body_index < 0)
			{
				return;
			}

			// Composite hull slot.
			if (body_index < static_cast<int>(m_composite_hulls.size()))
			{
				auto& slot = m_composite_hulls[body_index];
				if (slot.m_active && slot.m_generation == body_generation)
				{
					if (slot.m_body != nullptr)
					{
						slot.m_body->NeverSleep(slot.m_prev_never_sleep);
					}

					slot = CompositeSlot{};

					auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
					if (body_index < static_cast<int>(m_diagnostics.size()))
					{
						m_diagnostics[body_index] = Diagnostics{};
					}
				}
			}
		}

		// Return the latest diagnostic record for a registered hull.
		Diagnostics LatestDiagnostics(int body_index, int body_generation) const
		{
			if (!m_config.m_enable_diagnostics)
				return Diagnostics{};

			auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
			if (body_index < 0 || body_index >= static_cast<int>(m_diagnostics.size()))
			{
				return Diagnostics{};
			}

			auto diagnostic = m_diagnostics[body_index];
			if (diagnostic.m_body_generation != body_generation)
			{
				return Diagnostics{};
			}

			return diagnostic;
		}

		// Set the water surface used by subsequent buoyancy force dispatches.
		void SetWaterSurface(WaterSurface const& water_surface)
		{
			if (m_water_field_extension.Enabled())
				throw std::runtime_error("GpuBuoyancy custom water-field mode does not accept WaterSurface data");

			m_water_surface = water_surface.Normalised();
		}

		// Return the current water surface used by buoyancy force dispatches.
		WaterSurface const& GetWaterSurface() const
		{
			return m_water_surface;
		}

		// Copy a custom water-field snapshot after validating its configured fixed stride.
		void SetWaterField(std::span<std::byte const> elements, int element_count, float water_level)
		{
			if (!m_water_field_extension.Enabled())
				throw std::runtime_error("GpuBuoyancy requires a WaterFieldExtension before custom field data can be set");
			if (element_count < 0)
				throw std::runtime_error("GpuBuoyancy water-field element count cannot be negative");
			if (!std::isfinite(water_level))
				throw std::runtime_error("GpuBuoyancy water level must be finite");

			auto const expected_size = static_cast<std::size_t>(element_count) * static_cast<std::size_t>(m_water_field_extension.m_element_stride);
			if (elements.size() != expected_size)
				throw std::runtime_error("GpuBuoyancy water-field byte count does not match its element count and configured stride");

			m_water_field.assign(elements.begin(), elements.end());
			m_water_field_level = water_level;
		}

		// Return the active field element count for the selected default or custom evaluator.
		int WaterFieldCount() const
		{
			return m_water_field_extension.Enabled()
				? static_cast<int>(m_water_field.size() / static_cast<std::size_t>(m_water_field_extension.m_element_stride))
				: static_cast<int>(m_water_surface.m_waves.size());
		}

		// Return the still-water level paired with the active field representation.
		float WaterLevel() const
		{
			return m_water_field_extension.Enabled()
				? m_water_field_level
				: m_water_surface.m_level;
		}

		// Set the tunable buoyancy parameters used by subsequent dispatches.
		void SetConfig(Config const& config)
		{
			if (!std::isfinite(config.m_fluid_density) || config.m_fluid_density < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy fluid density must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_linear_drag_time_constant_s))
			{
				throw std::runtime_error("GpuBuoyancy linear drag time-constant must be finite");
			}
			if (!std::isfinite(config.m_angular_drag_time_constant_s))
			{
				throw std::runtime_error("GpuBuoyancy angular drag time-constant must be finite");
			}
			if (!std::isfinite(config.m_quadratic_drag_coefficient) || config.m_quadratic_drag_coefficient < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy quadratic drag coefficient must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_tangential_drag_coefficient) || config.m_tangential_drag_coefficient < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy tangential drag coefficient must be a finite, non-negative value");
			}
			if (config.m_polytope_tessellation == 0)
			{
				throw std::runtime_error("GpuBuoyancy polytope tessellation must select the face fan (negative) or a positive grid resolution");
			}

			m_config = config;
		}

		// Return the tunable buoyancy parameters currently in effect.
		Config const& GetConfig() const
		{
			return m_config;
		}

		// Consume diagnostic readback data after the physics engine has completed its GPU step.
		void CompleteStep()
		{
			if (m_pending_diagnostics.empty())
			{
				m_pending_readback = {};
				return;
			}

			auto const diagnostic_count = static_cast<int>(m_pending_diagnostics.size());
			auto const diagnostics = std::span{ m_pending_readback.ptr<GpuBuoyancyDiagnostic>(), m_pending_diagnostics.size() };
			{
				auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
				for (int index = 0; index != diagnostic_count; ++index)
				{
					auto const& pending = m_pending_diagnostics[index];
					auto const body_index = pending.m_body_index;
					auto const body_generation = pending.m_body_generation;
					if (body_index < 0 || body_index >= static_cast<int>(m_diagnostics.size()))
					{
						throw std::runtime_error("GPU buoyancy diagnostic readback referred to an unknown body slot");
					}
					if (m_diagnostics[body_index].m_body_generation != body_generation)
					{
						throw std::runtime_error("GPU buoyancy diagnostic readback generation mismatch");
					}

					auto const& gpu_diag = diagnostics[index];
					auto& diag = m_diagnostics[body_index];
					diag.m_volume_m3 = gpu_diag.m_volume_m3;
					diag.m_force_ws = gpu_diag.m_force_ws;
					diag.m_torque_ws = gpu_diag.m_torque_ws;
					diag.m_centre_buoyancy_ws = gpu_diag.m_centre_buoyancy_ws;
					diag.m_valid = gpu_diag.m_valid != 0;
				}
			}

			m_pending_diagnostics.clear();
			m_pending_readback = {};
		}

		// Record the buoyancy force and diagnostic compute work into the active physics GPU job.
		void Apply(Engine&, Engine::ExternalForceArgs const& args)
		{
			m_pending_diagnostics.clear();
			m_pending_readback = {};

			// All buoyancy now flows through the sampled-composite volume/surface dispatch.
			DispatchComposite(args);
		}

		// Host-side flat-water dry broadphase cull. Returns true when, with no waves, the body's
		// registration-time AABB lies entirely above the water surface measured along the body's local
		// up (-gravity) axis. The AABB encloses every primitive, so every volume and surface sample is
		// guaranteed dry and the body contributes exactly zero buoyancy and drag - bit-identical to the
		// GPU readback (which fast-paths dry boxes/spheres and produces all-dry samples for polytopes
		// and triangles). Only valid for flat water: under waves the surface height varies across the
		// footprint, so a single conservative support point is unsafe.
		bool IsFlatWaterFullyDry(CompositeShape const& shape_data, BodyState const& bs) const
		{
			// Only safe for flat water; a wavy surface can rise above a conservative support point.
			if (WaterFieldCount() != 0)
				return false;

			// Need a valid body pose and a usable gravity direction to define "up".
			if (!bs.m_valid)
				return false;

			auto const g = bs.m_ws_gravity.w0();
			if (!IsFinite(g) || LengthSq(g) < 1e-12f)
				return false;

			auto const up = -Normalise(g);

			// Lowest extent of the world-space AABB along up: project the centre, then subtract the
			// support distance contributed by each rotated body axis scaled by its half-extent.
			auto const centre_ws = bs.m_o2w * shape_data.m_obb_os.m_centre;
			auto const r = shape_data.m_obb_os.m_radius;
			auto const extent_up =
				Abs(Dot3(bs.m_o2w.x, up)) * r.x +
				Abs(Dot3(bs.m_o2w.y, up)) * r.y +
				Abs(Dot3(bs.m_o2w.z, up)) * r.z;
			auto const lowest = Dot3(centre_ws, up) - extent_up;

			if (!IsFinite(lowest) || !IsFinite(WaterLevel()))
				return false;

			// Strict margin: the band [level, level+margin] still produces zero on the GPU (its
			// per-primitive dry fast-path / all-dry samples), so a positive margin is always safe and
			// avoids host/GPU float divergence right at the waterline.
			auto const margin = std::max(shape_data.m_eps, 1e-4f);
			return lowest > WaterLevel() + margin;
		}

		// Resolve registered hulls to the current engine body order and collect the compact dispatch population.
		ActiveHullStats CollectActiveHulls(Engine::ExternalForceArgs const& args)
		{
			m_active_hulls.clear();
			m_active_shapes.clear();
			for (auto& bucket : m_active_shape_lookup)
				bucket.m_shape_data = nullptr;

			assert(m_active_hulls.capacity() >= m_composite_hulls.size());
			assert(m_active_shapes.capacity() >= m_composite_hulls.size());

			auto stats = ActiveHullStats{};
			for (auto body_index = 0; body_index != static_cast<int>(m_composite_hulls.size()); ++body_index)
			{
				auto const& slot = m_composite_hulls[body_index];
				if (!slot.m_active || slot.m_shape_data == nullptr)
				{
					continue;
				}

				auto const body_step_index = m_step_index_resolver(body_index);
				if (body_step_index < 0)
				{
					continue;
				}
				if (body_step_index >= args.m_body_count)
				{
					throw std::runtime_error("Buoyancy composite hull resolved to an invalid physics step body index");
				}

				// Flat-water dry broadphase cull: if the whole body sits above the water line, skip the
				// GPU dispatch entirely and publish a zero diagnostic directly. This is result-preserving
				// (see IsFlatWaterFullyDry) so the readback is identical to dispatching the body. Skipped
				// when waves are present (the resolver call is then pure overhead).
				if (WaterFieldCount() == 0)
				{
					auto const bs = m_body_state_resolver(body_index);
					if (IsFlatWaterFullyDry(*slot.m_shape_data, bs))
					{
						auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
						if (body_index < static_cast<int>(m_diagnostics.size()))
						{
							auto diag = Diagnostics{};
							diag.m_body_index = body_index;
							diag.m_body_generation = slot.m_generation;
							diag.m_valid = true;
							m_diagnostics[body_index] = diag;
						}
						continue;
					}
				}

				// Multiple bodies can share a collision shape, while representative workloads can also
				// contain thousands of unique shapes. Use allocation-free open addressing so both cases
				// resolve in expected constant time instead of scanning all shapes collected so far.
				auto const* shape_data = slot.m_shape_data.get();
				auto const bucket_mask = m_active_shape_lookup.size() - 1;
				auto bucket_index = (reinterpret_cast<std::uintptr_t>(shape_data) >> 4) & bucket_mask;
				while (m_active_shape_lookup[bucket_index].m_shape_data != nullptr &&
					m_active_shape_lookup[bucket_index].m_shape_data != shape_data)
				{
					bucket_index = (bucket_index + 1) & bucket_mask;
				}

				auto& bucket = m_active_shape_lookup[bucket_index];
				if (bucket.m_shape_data == nullptr)
				{
					bucket.m_shape_data = shape_data;
					bucket.m_shape_index = static_cast<int>(m_active_shapes.size());
					m_active_shapes.push_back(ActiveShape{
						.m_shape_data = slot.m_shape_data,
						.m_prim_base = 0,
						.m_prim_count = 0,
					});
				}
				auto const shape_index = bucket.m_shape_index;

				m_active_hulls.push_back(ActiveHull{
					.m_body_index = body_index,
					.m_body_generation = slot.m_generation,
					.m_body_step_index = body_step_index,
					.m_hull_id = slot.m_hull_id,
					.m_shape_index = shape_index,
				});
				stats.m_max_volume_samples = std::max(stats.m_max_volume_samples, slot.m_shape_data->m_total_volume_samples);
				stats.m_max_surface_samples = std::max(stats.m_max_surface_samples, slot.m_shape_data->m_total_surface_samples);
			}
			return stats;
		}

		// Record volume sampling followed by reduction into the ordinary body force accumulators.
		void RecordVolumePass(Engine::ExternalForceArgs const& args, CBufGpuBuoyancy const& cb, DispatchAddresses const& addresses)
		{
			// Evaluate all volume samples into one partial record per hull/threadgroup. Root parameter
			// order must match CreateVolumeStep: b0, u0(bodies), t1(water field), t2(headers), t3(prims),
			// t4(volume_verts), t5(tets), t6(face_planes), t7(records), t12(tet_cdf), u1(partials).
			args.m_job.m_cmd_list.SetPipelineState(m_volume_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_volume_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_water_field);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_volume_headers);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_volume_primitives);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_volume_verts);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_tets);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_face_planes);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_volume_records);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_tet_cdf);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(cb.m_hull_count * cb.m_groups_per_hull, 1, 1);
			args.m_job.m_barriers.UAV(m_r_partials.get()).Commit();

			// Reduce per-threadgroup partials to one body force/torque accumulation. Diagnostic output
			// is optional, but its dummy UAV remains bound to keep one root-signature layout.
			args.m_job.m_cmd_list.SetPipelineState(m_volume_reduce_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_volume_reduce_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_volume_headers);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(cb.m_hull_count, 1, 1);
			args.m_job.m_barriers.UAV(args.m_bodies);
			if (cb.m_enable_diagnostics != 0)
				args.m_job.m_barriers.UAV(m_r_diagnostics.get());

			args.m_job.m_barriers.Commit();
		}

		// Record surface sampling followed by reduction that adds drag to the volume-pass result.
		void RecordSurfacePass(Engine::ExternalForceArgs const& args, CBufGpuBuoyancy const& cb, DispatchAddresses const& addresses)
		{
			// Evaluate all surface samples into one partial record per hull/threadgroup. Root parameter
			// order must match CreateSurfaceStep: b0, u0(bodies), t1(water field), t3(surf_prims),
			// t6(face_planes), t8(surf_headers), t9(verts), t10(face_verts), t11(surf_records),
			// u1(partials).
			args.m_job.m_cmd_list.SetPipelineState(m_surface_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_surface_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_water_field);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_primitives);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_face_planes);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_headers);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_verts);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_face_verts);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_records);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(cb.m_hull_count * cb.m_groups_per_hull, 1, 1);
			args.m_job.m_barriers.UAV(m_r_partials.get()).Commit();

			// Reduce the surface partials and add drag to the body and existing diagnostic. Root parameter
			// order must match CreateSurfaceReduceStep: b0, u0(bodies), t1(water field), t8(surf_headers),
			// u1(partials), u2(diagnostics).
			args.m_job.m_cmd_list.SetPipelineState(m_surface_reduce_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_surface_reduce_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_water_field);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_headers);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(cb.m_hull_count, 1, 1);
			args.m_job.m_barriers.UAV(args.m_bodies);
			if (cb.m_enable_diagnostics != 0)
				args.m_job.m_barriers.UAV(m_r_diagnostics.get());

			args.m_job.m_barriers.Commit();
		}

		// Queue the final per-hull diagnostics for consumption after the physics GPU job completes.
		void RecordDiagnosticReadback(Engine::ExternalForceArgs const& args, int hull_count)
		{
			m_pending_readback = args.m_job.m_readback.template Alloc<GpuBuoyancyDiagnostic>(hull_count);
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			args.m_job.m_cmd_list.CopyBufferRegion(m_pending_readback, m_r_diagnostics.get(), 0);
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		}

		// Prepare transient inputs and record the volume, optional drag, and diagnostic readback phases.
		void DispatchComposite(Engine::ExternalForceArgs const& args)
		{
			if (args.m_body_count == 0)
			{
				return;
			}

			// Phase 1: compact registrations into body-step order and establish the dispatch dimensions.
			auto const active_stats = CollectActiveHulls(args);
			if (m_active_hulls.empty())
			{
				return;
			}

			auto const hull_count = static_cast<int>(m_active_hulls.size());

			// Groups are laid out [hull0 groups][hull1 groups]..., with a UNIFORM group count per hull
			// (the kernel derives the hull from global_group_index / groups_per_hull). Size it for the
			// busiest hull. The reducer sums one partial per group on a single thread group, so the
			// group count must not exceed the reduce thread count.
			auto const groups_per_hull = std::max(1, (active_stats.m_max_volume_samples + BuoyancyVolumeThreadCount - 1) / BuoyancyVolumeThreadCount);
			if (groups_per_hull > BuoyancyVolumeReduceThreadCount)
			{
				throw std::runtime_error("Buoyancy composite hull exceeds the maximum supported volume sample count");
			}

			// Surface (drag) groups are sized independently from the volume groups because each hull's
			// surface-sample count differs from its volume-sample count. The reduce pass sums one
			// partial per group on a single thread group, so the group count must not exceed the reduce
			// thread count. Both passes share the per-threadgroup partials buffer, so it must be sized
			// for whichever pass needs the most groups.
			auto const surf_groups_per_hull = std::max(1, (active_stats.m_max_surface_samples + BuoyancyVolumeThreadCount - 1) / BuoyancyVolumeThreadCount);
			if (surf_groups_per_hull > BuoyancyVolumeReduceThreadCount)
			{
				throw std::runtime_error("Buoyancy composite hull exceeds the maximum supported surface sample count");
			}

			EnsureGpuCapacity(
				args.m_job,
				hull_count * std::max(groups_per_hull, surf_groups_per_hull),
				m_config.m_enable_diagnostics ? hull_count : 1);

			// Phase 2: sum geometry sizes across unique active collision shapes. Bodies sharing a shape
			// point at the same primitive block, so immutable data is uploaded only once per dispatch.
			// Each upload uses at least one element because empty arrays still need a valid address.
			auto total_prims = 0;
			auto total_volume_verts = 0;
			auto total_tets = 0;
			auto total_face_planes = 0;
			auto total_verts = 0;
			auto total_face_verts = 0;
			for (auto const& active_shape : m_active_shapes)
			{
				total_prims += static_cast<int>(active_shape.m_shape_data->m_hull.m_primitives.size());
				total_volume_verts += static_cast<int>(active_shape.m_shape_data->m_hull.m_volume_verts.size());
				total_tets += static_cast<int>(active_shape.m_shape_data->m_hull.m_tets.size());
				total_face_planes += static_cast<int>(active_shape.m_shape_data->m_hull.m_face_planes.size());
				total_verts += static_cast<int>(active_shape.m_shape_data->m_hull.m_verts.size());
				total_face_verts += static_cast<int>(active_shape.m_shape_data->m_hull.m_face_verts.size());
			}

			auto upload_headers = args.m_job.m_upload.template Alloc<GpuBuoyVolHeader>(hull_count);
			auto upload_prims = args.m_job.m_upload.template Alloc<buoyancy::GpuPrimitive>(std::max(total_prims, 1));
			auto upload_records = args.m_job.m_upload.template Alloc<GpuBuoyVolPrimRecord>(std::max(total_prims, 1));
			auto upload_volume_verts = args.m_job.m_upload.template Alloc<v4>(std::max(total_volume_verts, 1));
			auto upload_tets = args.m_job.m_upload.template Alloc<iv4>(std::max(total_tets, 1));
			auto upload_tet_cdf = args.m_job.m_upload.template Alloc<float>(std::max(total_tets, 1));
			auto upload_face_planes = args.m_job.m_upload.template Alloc<v4>(std::max(total_face_planes, 1));

			// Surface-pass upload buffers. The surface pass reads the same primitive descriptors but
			// needs them with surface-vertex / face-plane offsets shifted into the surface buffers (the
			// volume prims shift the volume-vertex / tet offsets instead), so it uses its own prims copy.
			auto upload_surf_headers = args.m_job.m_upload.template Alloc<GpuBuoySurfHeader>(hull_count);
			auto upload_surf_prims = args.m_job.m_upload.template Alloc<buoyancy::GpuPrimitive>(std::max(total_prims, 1));
			auto upload_surf_records = args.m_job.m_upload.template Alloc<GpuBuoySurfPrimRecord>(std::max(total_prims, 1));
			auto upload_verts = args.m_job.m_upload.template Alloc<v4>(std::max(total_verts, 1));
			auto upload_face_verts = args.m_job.m_upload.template Alloc<iv4>(std::max(total_face_verts, 1));

			auto headers = upload_headers.ptr<GpuBuoyVolHeader>();
			auto prims = upload_prims.ptr<buoyancy::GpuPrimitive>();
			auto records = upload_records.ptr<GpuBuoyVolPrimRecord>();
			auto volume_verts = upload_volume_verts.ptr<v4>();
			auto tets = upload_tets.ptr<iv4>();
			auto tet_cdf = upload_tet_cdf.ptr<float>();
			auto face_planes = upload_face_planes.ptr<v4>();

			auto surf_headers = upload_surf_headers.ptr<GpuBuoySurfHeader>();
			auto surf_prims = upload_surf_prims.ptr<buoyancy::GpuPrimitive>();
			auto surf_records = upload_surf_records.ptr<GpuBuoySurfPrimRecord>();
			auto verts = upload_verts.ptr<v4>();
			auto face_verts = upload_face_verts.ptr<iv4>();

			// Phase 3: concatenate each unique shape's geometry and sample records into shared buffers.
			// Tet-corner and face-vertex indices remain relative to each primitive's vertex block.
			auto prim_base = 0;
			auto vvert_base = 0;
			auto tet_base = 0;
			auto face_base = 0;
			auto svert_base = 0;
			auto sfvert_base = 0;
			for (auto& active_shape : m_active_shapes)
			{
				auto const& shape_data = *active_shape.m_shape_data;
				auto const& hull = shape_data.m_hull;
				auto const prim_count = static_cast<int>(hull.m_primitives.size());
				active_shape.m_prim_base = prim_base;
				active_shape.m_prim_count = prim_count;

				for (auto i = 0; i != static_cast<int>(hull.m_volume_verts.size()); ++i)
					volume_verts[vvert_base + i] = hull.m_volume_verts[i];
				for (auto i = 0; i != static_cast<int>(hull.m_tets.size()); ++i)
				{
					tets[tet_base + i] = hull.m_tets[i];
					tet_cdf[tet_base + i] = hull.m_tet_cdf[i];
				}
				for (auto i = 0; i != static_cast<int>(hull.m_face_planes.size()); ++i)
					face_planes[face_base + i] = hull.m_face_planes[i];

				// Surface geometry: the boundary verts and per-face vertex-index lists. Face-vertex
				// indices stay RELATIVE to the primitive's own vertex block (the kernel re-adds the
				// offset), so the index blocks are copied verbatim.
				for (auto i = 0; i != static_cast<int>(hull.m_verts.size()); ++i)
					verts[svert_base + i] = hull.m_verts[i];
				for (auto i = 0; i != static_cast<int>(hull.m_face_verts.size()); ++i)
					face_verts[sfvert_base + i] = hull.m_face_verts[i];

				for (auto k = 0; k != prim_count; ++k)
				{
					// Copy the primitive descriptor and shift its absolute array offsets into the shared
					// buffers. m_vert_ofs is left unchanged: the surface-vertex array is not bound for the
					// volume pass, so its offset is unused here.
					auto p = hull.m_primitives[k];
					p.m_volume_vert_ofs += vvert_base;
					p.m_tet_ofs += tet_base;
					p.m_face_ofs += face_base;
					prims[prim_base + k] = p;

					records[prim_base + k] = GpuBuoyVolPrimRecord{
						.m_count = shape_data.m_vol_counts[k],
						.m_dvol = shape_data.m_vol_dvol[k],
					};

					// Surface prims need the surface-vertex offset and face offset shifted instead. The
					// face_base is shared with the volume pass (the same face-plane buffer is bound to the
					// surface step), so the sibling cull reads identical planes in both passes.
					auto sp = hull.m_primitives[k];
					sp.m_vert_ofs += svert_base;
					sp.m_face_ofs += face_base;
					surf_prims[prim_base + k] = sp;

					surf_records[prim_base + k] = GpuBuoySurfPrimRecord{
						.m_count = shape_data.m_surf_counts[k],
						.m_darea = shape_data.m_surf_darea[k],
					};
				}

				prim_base += prim_count;
				vvert_base += static_cast<int>(hull.m_volume_verts.size());
				tet_base += static_cast<int>(hull.m_tets.size());
				face_base += static_cast<int>(hull.m_face_planes.size());
				svert_base += static_cast<int>(hull.m_verts.size());
				sfvert_base += static_cast<int>(hull.m_face_verts.size());
			}

			// Phase 4: emit per-body headers that reference the shared shape blocks. Body index,
			// generation, and deterministic hull seed remain distinct for each registration.
			assert(!m_config.m_enable_diagnostics || m_pending_diagnostics.capacity() >= m_composite_hulls.size());
			for (auto index = 0; index != hull_count; ++index)
			{
				auto const& active_hull = m_active_hulls[index];
				auto const& active_shape = m_active_shapes[active_hull.m_shape_index];
				auto const& shape_data = *active_shape.m_shape_data;

				headers[index] = GpuBuoyVolHeader{
					.m_body_index = active_hull.m_body_step_index,
					.m_prim_base = active_shape.m_prim_base,
					.m_prim_count = active_shape.m_prim_count,
					.m_total_volume_samples = shape_data.m_total_volume_samples,
					.m_hull_id = active_hull.m_hull_id,
					.m_eps = shape_data.m_eps,
					.m_pad0 = 0,
					.m_pad1 = 0,
				};
				surf_headers[index] = GpuBuoySurfHeader{
					.m_body_index = active_hull.m_body_step_index,
					.m_prim_base = active_shape.m_prim_base,
					.m_prim_count = active_shape.m_prim_count,
					.m_total_surface_samples = shape_data.m_total_surface_samples,
					.m_hull_id = active_hull.m_hull_id,
					.m_eps = shape_data.m_eps,
					.m_pad0 = 0,
					.m_pad1 = 0,
				};
				if (m_config.m_enable_diagnostics)
				{
					m_pending_diagnostics.push_back(PendingDiagnostic{
						.m_body_index = active_hull.m_body_index,
						.m_body_generation = active_hull.m_body_generation,
					});
				}
			}

			// Phase 5: finish the transient inputs with the active water field. The volume kernel needs
			// t1 bound even for a flat field, so always allocate and zero at least one element.
			auto const water_field_count = WaterFieldCount();
			auto const water_field_stride = m_water_field_extension.Enabled()
				? m_water_field_extension.m_element_stride
				: static_cast<int>(sizeof(GpuBuoyancyWave));
			auto upload_water_field = args.m_job.m_upload.Alloc(std::max(water_field_count, 1) * water_field_stride, 16);
			memset(upload_water_field.ptr<std::byte>(), 0, static_cast<std::size_t>(upload_water_field.m_size));
			if (m_water_field_extension.Enabled())
			{
				memcpy(upload_water_field.ptr<std::byte>(), m_water_field.data(), m_water_field.size());
			}
			else
			{
				auto waves = upload_water_field.ptr<GpuBuoyancyWave>();
				for (auto index = 0; index != water_field_count; ++index)
				{
					auto const& wave = m_water_surface.m_waves[index];
					waves[index] = GpuBuoyancyWave{
						.m_direction_wavelength_phase_speed = v4(wave.m_direction.x, wave.m_direction.y, wave.m_wavelength, wave.m_phase_speed),
						.m_amplitude = v4(wave.m_amplitude, 0.0f, 0.0f, 0.0f),
					};
				}
			}

			auto const gpu_va = [](auto const& alloc)
			{
				return alloc.m_res->GetGPUVirtualAddress() + alloc.m_ofs;
			};
			auto const addresses = DispatchAddresses{
				.m_water_field = gpu_va(upload_water_field),
				.m_volume_headers = gpu_va(upload_headers),
				.m_volume_primitives = gpu_va(upload_prims),
				.m_volume_records = gpu_va(upload_records),
				.m_volume_verts = gpu_va(upload_volume_verts),
				.m_tets = gpu_va(upload_tets),
				.m_tet_cdf = gpu_va(upload_tet_cdf),
				.m_face_planes = gpu_va(upload_face_planes),
				.m_surface_headers = gpu_va(upload_surf_headers),
				.m_surface_primitives = gpu_va(upload_surf_prims),
				.m_surface_records = gpu_va(upload_surf_records),
				.m_surface_verts = gpu_va(upload_verts),
				.m_surface_face_verts = gpu_va(upload_face_verts),
			};

			// Translational and rotational damping share the wet-volume samples but have independent
			// rho/tau coefficients. Quadratic drag remains disabled in this pass.
			auto const linear_drag_coefficient = m_config.m_linear_drag_time_constant_s > 0.0f
				? m_config.m_fluid_density / m_config.m_linear_drag_time_constant_s
				: 0.0f;
			auto const angular_drag_coefficient = m_config.m_angular_drag_time_constant_s > 0.0f
				? m_config.m_fluid_density / m_config.m_angular_drag_time_constant_s
				: 0.0f;
			auto const cb = CBufGpuBuoyancy{
				.m_hull_count = hull_count,
				.m_groups_per_hull = groups_per_hull,
				.m_water_field_count = water_field_count,
				.m_time_s = static_cast<float>(args.m_time_s),
				.m_water_level = WaterLevel(),
				.m_fluid_density = m_config.m_fluid_density,
				.m_linear_drag_coefficient = linear_drag_coefficient,
				.m_angular_drag_coefficient = angular_drag_coefficient,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.0f,
				.m_time_step_s = args.m_dt,
				.m_enable_diagnostics = m_config.m_enable_diagnostics ? 1 : 0,
			};

			// Phase 6: sample displaced volume and reduce it directly into each body's force/torque accumulators.
			RecordVolumePass(args, cb, addresses);

			// Phase 7: optionally sample surface drag. This reuses the per-threadgroup partials buffer (sized above for the
			// larger of the volume / surface group counts) and runs AFTER the volume reduce so it can
			// ADD drag force/torque onto the body accumulator and the existing diagnostic record. The
			// cbuffer carries the surface group count and both surface-drag coefficients. Skip the
			// surface pass when both are disabled, even if linear damping is active.
			auto const surf_quad_coefficient = std::max(0.0f, m_config.m_quadratic_drag_coefficient);
			auto const surf_tangent_coefficient = std::max(0.0f, m_config.m_tangential_drag_coefficient);
			auto const have_drag = surf_quad_coefficient > 0.0f || surf_tangent_coefficient > 0.0f;
			if (have_drag && active_stats.m_max_surface_samples > 0)
			{
				auto const cb_surf = CBufGpuBuoyancy{
					.m_hull_count = hull_count,
					.m_groups_per_hull = surf_groups_per_hull,
					.m_water_field_count = water_field_count,
					.m_time_s = static_cast<float>(args.m_time_s),
					.m_water_level = WaterLevel(),
					.m_fluid_density = m_config.m_fluid_density,
					.m_linear_drag_coefficient = 0.0f,
					.m_angular_drag_coefficient = 0.0f,
					.m_quadratic_drag_coefficient = surf_quad_coefficient,
					.m_tangential_drag_coefficient = surf_tangent_coefficient,
					.m_time_step_s = args.m_dt,
					.m_enable_diagnostics = m_config.m_enable_diagnostics ? 1 : 0,
				};

				RecordSurfacePass(args, cb_surf, addresses);
			}

			// Phase 8: diagnostic publication is opt-in because it adds GPU copy/readback work that
			// production force application does not require.
			if (m_config.m_enable_diagnostics)
				RecordDiagnosticReadback(args, hull_count);
		}

		// Resize GPU buffers used by the diagnostic dispatches. 'partial_capacity' is the number of
		// per-threadgroup partial records the volume pass will write and 'diagnostic_capacity'
		// is the number of per-hull diagnostic records the reduce pass will write. The volume and
		// surface dispatches both route through here so the buffers grow to the larger demand.
		void EnsureGpuCapacity(GpuJob& job, int partial_capacity, int diagnostic_capacity)
		{
			if (partial_capacity > m_partial_capacity)
			{
				m_r_partials = CreateDefaultUavBuffer<GpuBuoyancyPartial>(m_device, partial_capacity, "Physics.GpuBuoyancy.Partials");
				m_partial_capacity = partial_capacity;
			}
			if (diagnostic_capacity > m_diagnostic_capacity)
			{
				m_r_diagnostics = CreateDefaultUavBuffer<GpuBuoyancyDiagnostic>(m_device, diagnostic_capacity, "Physics.GpuBuoyancy.Diagnostics");
				m_diagnostic_capacity = diagnostic_capacity;
			}

			// New resources start in COMMON but are tracked with a UAV default state, relying on normal
			// D3D12 buffer promotion for first use. The dummy diagnostic UAV is not accessed in production.
			job.m_barriers.Transition(m_r_partials.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			if (m_config.m_enable_diagnostics)
				job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

			job.m_barriers.Commit();
		}
	};

	// Return a copy with the wave direction normalised.
	GpuBuoyancy::SineWave GpuBuoyancy::SineWave::Normalised() const
	{
		ValidateSineWave(*this);

		auto wave = *this;
		wave.m_direction /= Length(wave.m_direction);
		return wave;
	}

	// Return a copy with validated and normalised waves.
	GpuBuoyancy::WaterSurface GpuBuoyancy::WaterSurface::Normalised() const
	{
		ValidateWaterSurface(*this);

		auto water_surface = *this;
		for (auto& wave : water_surface.m_waves)
		{
			wave = wave.Normalised();
		}
		return water_surface;
	}

	// Return true when both parts of the custom shader/data contract are present.
	bool GpuBuoyancy::WaterFieldExtension::Enabled() const
	{
		return !m_shader_include.empty() && m_element_stride != 0;
	}

	// Return true when the water height is spatially constant.
	bool GpuBuoyancy::WaterSurface::IsFlat() const
	{
		return std::ranges::all_of(m_waves, [](SineWave const& wave)
		{
			return wave.m_amplitude == 0.0f;
		});
	}

	// Evaluate the water height above the world-space XY position at a simulation time.
	float GpuBuoyancy::WaterSurface::EvaluateHeight(v2 xy_ws, float time_s) const
	{
		auto height = m_level;
		for (auto const& wave : m_waves)
		{
			auto const phase = Dot(wave.m_direction, xy_ws) * constants<float>::tau / wave.m_wavelength + wave.m_phase_speed * time_s;
			height += wave.m_amplitude * std::sin(phase);
		}
		return height;
	}

	// Evaluate the XY surface gradient (dh/dx, dh/dy) of the water height at a simulation time.
	v2 GpuBuoyancy::WaterSurface::EvaluateGradient(v2 xy_ws, float time_s) const
	{
		auto gradient = v2::Zero();
		for (auto const& wave : m_waves)
		{
			// h_i(x,y,t) = A * sin(k * (d . xy) + omega * t), where k = 2*pi/wavelength.
			// dh_i/dx = A * k * d.x * cos(k * (d . xy) + omega * t); dh_i/dy uses d.y.
			auto const k = constants<float>::tau / wave.m_wavelength;
			auto const phase = Dot(wave.m_direction, xy_ws) * k + wave.m_phase_speed * time_s;
			auto const coeff = wave.m_amplitude * k * std::cos(phase);
			gradient += wave.m_direction * coeff;
		}
		return gradient;
	}

	// Evaluate the lateral pressure gradient implied by each configured wave's orbital acceleration.
	v2 GpuBuoyancy::WaterSurface::EvaluatePressureGradient(v2 xy_ws, float time_s, float gravity) const
	{
		if (!(gravity > tiny<float>))
			return v2::Zero();

		auto gradient = v2::Zero();
		for (auto const& wave : m_waves)
		{
			auto const k = constants<float>::tau / wave.m_wavelength;
			auto const omega = wave.m_phase_speed;
			auto const phase = Dot(wave.m_direction, xy_ws) * k + omega * time_s;
			auto const coeff = wave.m_amplitude * omega * omega * std::cos(phase) / gravity;
			gradient += wave.m_direction * coeff;
		}
		return gradient;
	}

	// Evaluate the world-space water particle velocity (orbital flow) at a world-space position.
	v4 GpuBuoyancy::WaterSurface::EvaluateVelocity(v4 pos_ws, float time_s) const
	{
		// Deep-water (Airy) linear wave theory. For a single component with surface elevation
		//   h_i = A * sin(phi),  phi = k*(d . xy) + omega*t,  k = tau/wavelength,  omega = phase_speed,
		// the irrotational velocity field below the mean surface is
		//   u_along_d = -A*omega * e^(k*z) * sin(phi)   (horizontal, along the wave direction d)
		//   w_up      =  A*omega * e^(k*z) * cos(phi)   (vertical)
		// where z is the signed height relative to the still-water level (z <= 0 in the fluid).
		// At z = 0 the vertical component reduces to dh/dt (the linear kinematic free-surface
		// condition), so the field stays consistent with EvaluateHeight. The depth is clamped at
		// the still-water level so samples at or above the surface use the unattenuated velocity.
		auto const xy = v2{pos_ws.x, pos_ws.y};
		auto depth = pos_ws.z - m_level;
		if (depth > 0.0f)
			depth = 0.0f;

		auto velocity = v4::Zero();
		for (auto const& wave : m_waves)
		{
			auto const k = constants<float>::tau / wave.m_wavelength;
			auto const omega = wave.m_phase_speed;
			auto const phase = Dot(wave.m_direction, xy) * k + omega * time_s;
			auto const speed = wave.m_amplitude * omega * std::exp(k * depth);

			auto const u_along_d = -speed * std::sin(phase);
			auto const w_up = speed * std::cos(phase);

			velocity.x += u_along_d * wave.m_direction.x;
			velocity.y += u_along_d * wave.m_direction.y;
			velocity.z += w_up;
		}
		return velocity;
	}

	// Construct an invalid buoyancy diagnostic record.
	GpuBuoyancy::Diagnostics::Diagnostics()
		:m_body_index(-1)
		,m_body_generation(-1)
		,m_volume_m3()
		,m_force_ws(v4::Zero())
		,m_centre_buoyancy_ws(v4::Zero())
		,m_torque_ws(v4::Zero())
		,m_valid()
	{
	}

	// Construct an empty registration handle.
	GpuBuoyancy::Registration::Registration()
		:m_owner()
		,m_body_index(-1)
		,m_body_generation(-1)
	{
	}

	// Construct a registration handle for an already registered hull.
	GpuBuoyancy::Registration::Registration(GpuBuoyancy& owner, int body_index, int body_generation)
		:m_owner(&owner)
		,m_body_index(body_index)
		,m_body_generation(body_generation)
	{
	}

	// Move ownership of a buoyancy hull registration.
	GpuBuoyancy::Registration::Registration(Registration&& rhs) noexcept
		:Registration()
	{
		std::swap(m_owner, rhs.m_owner);
		std::swap(m_body_index, rhs.m_body_index);
		std::swap(m_body_generation, rhs.m_body_generation);
	}

	// Replace this handle with another registration, unregistering the current hull first.
	GpuBuoyancy::Registration& GpuBuoyancy::Registration::operator=(Registration&& rhs) noexcept
	{
		if (this == &rhs)
		{
			return *this;
		}

		Reset();
		std::swap(m_owner, rhs.m_owner);
		std::swap(m_body_index, rhs.m_body_index);
		std::swap(m_body_generation, rhs.m_body_generation);
		return *this;
	}

	// Unregister the hull owned by this handle.
	GpuBuoyancy::Registration::~Registration()
	{
		Reset();
	}

	// Release the current hull registration, if any.
	void GpuBuoyancy::Registration::Reset() noexcept
	{
		if (m_owner == nullptr)
		{
			return;
		}

		m_owner->UnregisterHull(m_body_index, m_body_generation);
		m_owner = nullptr;
		m_body_index = -1;
		m_body_generation = -1;
	}

	// Return true when this handle owns a registered hull.
	GpuBuoyancy::Registration::operator bool() const
	{
		return m_owner != nullptr;
	}

	// Construct and subscribe the diagnostic buoyancy pass to a physics engine.
	GpuBuoyancy::GpuBuoyancy(ID3D12Device* device, Engine& engine, Config const& config, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver, WaterFieldExtension water_field_extension)
		:m_impl(std::make_unique<Impl>(device, engine, config, std::move(step_index_resolver), std::move(body_state_resolver), std::move(water_field_extension)))
	{
	}

	// Destroy the buoyancy module after all owned registrations have been released.
	GpuBuoyancy::~GpuBuoyancy()
	{
		#if PR_DBG
		auto const active_composite = std::ranges::any_of(m_impl->m_composite_hulls, [](Impl::CompositeSlot const& slot)
		{
			return slot.m_active;
		});
		PR_ASSERT(PR_DBG, !active_composite, "GpuBuoyancy registrations must be destroyed before the GpuBuoyancy module");
		#endif
	}

	// Return the latest diagnostic record for a registered hull.
	GpuBuoyancy::Diagnostics GpuBuoyancy::LatestDiagnostics(int body_index, int body_generation) const
	{
		return m_impl->LatestDiagnostics(body_index, body_generation);
	}

	// Consume diagnostic readback data after the physics engine has completed its GPU step.
	void GpuBuoyancy::CompleteStep()
	{
		m_impl->CompleteStep();
	}

	// Set the water surface used by subsequent buoyancy force dispatches.
	void GpuBuoyancy::SetWaterSurface(WaterSurface const& water_surface)
	{
		m_impl->SetWaterSurface(water_surface);
	}

	// Return the current water surface used by buoyancy force dispatches.
	GpuBuoyancy::WaterSurface const& GpuBuoyancy::GetWaterSurface() const
	{
		return m_impl->GetWaterSurface();
	}

	// Copy a custom water-field snapshot for subsequent buoyancy force dispatches.
	void GpuBuoyancy::SetWaterField(std::span<std::byte const> elements, int element_count, float water_level)
	{
		m_impl->SetWaterField(elements, element_count, water_level);
	}

	// Set the tunable buoyancy parameters used by subsequent dispatches.
	void GpuBuoyancy::SetConfig(Config const& config)
	{
		m_impl->SetConfig(config);
	}

	// Return the tunable buoyancy parameters currently in effect.
	GpuBuoyancy::Config const& GpuBuoyancy::GetConfig() const
	{
		return m_impl->GetConfig();
	}

	// Register a rigid body's collision shape as its buoyancy hull.
	GpuBuoyancy::Registration GpuBuoyancy::RegisterCompositeHull(RigidBody& body, int body_index, int body_generation)
	{
		m_impl->RegisterCompositeHull(body, body_index, body_generation);
		return Registration{*this, body_index, body_generation};
	}

	// Remove the buoyancy hull for a stable physics body index.
	void GpuBuoyancy::UnregisterHull(int body_index, int body_generation) noexcept
	{
		if (m_impl == nullptr)
		{
			return;
		}

		m_impl->UnregisterHull(body_index, body_generation);
	}
}
