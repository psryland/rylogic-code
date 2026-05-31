//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/physics/buoyancy/gpu_buoyancy.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/compute/compute_pso.h"
#include "pr/compute/compute_step.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/utility/root_signature.h"
#include "src/buoyancy/buoyancy_analytical.h"
#include "src/utility/gpu.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"

namespace pr::physics
{
	namespace
	{
		// Sampled-composite (phase 10) volume-pass tunables. The total number of volume samples per
		// hull is split across the hull's primitives proportional to their volume; positions are
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
			int m_wave_count;
			float m_time_s;
			float m_water_level;
			float m_fluid_density;
			float m_drag_coefficient;
			float m_quadratic_drag_coefficient;
			float m_pad0;
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
			v4 m_moment_ws_volume;
			int m_body_index;
			int m_valid;
			float m_volume_m3;
			float m_pad0;
		};
		static_assert(sizeof(GpuBuoyancyDiagnostic) == 80);

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

		// Compile a runtime compute shader entry point from the physics buoyancy shader source.
		std::vector<uint8_t> CompileBuoyancyShader(wchar_t const* entry_point)
		{
			auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};
			return ::pr::compute::ShaderCompiler{}
				.Source("src/buoyancy/gpu_buoyancy.hlsl", resolver)
				.HlslVersion(::pr::compute::EHlslVersion::Hlsl2021)
				.Define(L"SHADER_BUILD")
				.Optimise(true)
				.ShaderModel(L"cs_6_6")
				.EntryPoint(entry_point)
				.Compile();
		}

		// Create the sampled-composite volume-sample compute step. The root signature mirrors the
		// resource access order of CSBuoyancyVolumeSamples: constants (b0), the body accumulator (u0),
		// the wave SRV (t1, for the water height/gradient), the six volume-pass SRVs (t2..t7), then the
		// partials UAV (u1). The legacy box-hull SRV (t0) and the diagnostics UAV (u2) are unused by
		// this kernel and so are omitted; root descriptors need not be contiguous.
		::pr::compute::ComputeStep CreateVolumeStep(ID3D12Device* device)
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
				.UAV(hlsl::EUAVReg::u1)
				.Create(device, "Physics.GpuBuoyancy.Volume.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyVolumeSamples")).Create(device, "Physics.GpuBuoyancy.Volume.PSO");
			return step;
		}

		// Create the sampled-composite volume-reduce compute step. The root signature mirrors the
		// resource access order of CSBuoyancyVolumeReduce: constants (b0), the body accumulator (u0),
		// the per-hull headers SRV (t2), the partials UAV (u1), then the diagnostics UAV (u2).
		::pr::compute::ComputeStep CreateVolumeReduceStep(ID3D12Device* device)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t2)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.Create(device, "Physics.GpuBuoyancy.VolumeReduce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyVolumeReduce")).Create(device, "Physics.GpuBuoyancy.VolumeReduce.PSO");
			return step;
		}

		// Create the sampled-composite surface-sample (drag) compute step. The root signature mirrors
		// the resource access order of CSBuoyancyDragSurfaceSamples: constants (b0), the body
		// accumulator (u0), the wave SRV (t1, for water height/velocity), the primitives SRV (t3) and
		// face planes SRV (t6, for the sibling cull), the four surface-pass SRVs (t8..t11), then the
		// partials UAV (u1). Root descriptors need not be contiguous.
		::pr::compute::ComputeStep CreateSurfaceStep(ID3D12Device* device)
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

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyDragSurfaceSamples")).Create(device, "Physics.GpuBuoyancy.Surface.PSO");
			return step;
		}

		// Create the sampled-composite surface-reduce compute step. The root signature mirrors the
		// resource access order of CSBuoyancyDragSurfaceReduce: constants (b0), the body accumulator
		// (u0), the per-hull surface headers SRV (t8), the partials UAV (u1), then the diagnostics UAV
		// (u2). The surface reduce ADDS drag force/torque to the body and the existing diagnostic.
		::pr::compute::ComputeStep CreateSurfaceReduceStep(ID3D12Device* device)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t8)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.Create(device, "Physics.GpuBuoyancy.SurfaceReduce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancyDragSurfaceReduce")).Create(device, "Physics.GpuBuoyancy.SurfaceReduce.PSO");
			return step;
		}
	}

	struct GpuBuoyancy::Impl
	{
		struct CompositeSlot
		{
			// Bookkeeping for a buoyancy hull. Saves/restores the body's NeverSleep flag (a floating
			// body must stay awake so the environmental force keeps being applied) and owns a
			// flattened, immutable composite descriptor of convex primitives.
			int m_generation = -1;
			bool m_active = false;
			RigidBody* m_body = nullptr;
			bool m_prev_never_sleep = false;
			buoyancy::CompositeHull m_hull;

			// Volume-pass sample plan, computed once at registration. m_vol_counts[k] is the number of
			// volume samples assigned to primitive k (proportional to its volume) and m_vol_dvol[k] is
			// the matching per-sample volume weight (primitive volume / count). m_total_volume_samples
			// is the sum of m_vol_counts. m_hull_id seeds the deterministic sample hash (stable across
			// body reorders) and m_eps is the scale-relative inside-test slack for the sibling cull.
			std::vector<int> m_vol_counts;
			std::vector<float> m_vol_dvol;
			int m_total_volume_samples = 0;
			uint32_t m_hull_id = 0;
			float m_eps = 0.0f;

			// Surface-pass sample plan, computed once at registration. m_surf_counts[k] is the number
			// of surface samples assigned to primitive k (proportional to its surface area) and
			// m_surf_darea[k] is the matching per-sample area weight (primitive area / count).
			// m_total_surface_samples is the sum of m_surf_counts.
			std::vector<int> m_surf_counts;
			std::vector<float> m_surf_darea;
			int m_total_surface_samples = 0;
		};
		struct AnalyticResult
		{
			float m_volume_m3;
			v4 m_force_ws;
			v4 m_centre_buoyancy_ws;
			v4 m_torque_ws;
			bool m_valid;

			// Construct an invalid analytic buoyancy result.
			AnalyticResult()
				:m_volume_m3()
				,m_force_ws(v4::Zero())
				,m_centre_buoyancy_ws(v4::Zero())
				,m_torque_ws(v4::Zero())
				,m_valid()
			{
			}
		};

		ID3D12Device* m_device;
		StepIndexResolver m_step_index_resolver;
		BodyStateResolver m_body_state_resolver;
		::pr::compute::ComputeStep m_volume_step;
		::pr::compute::ComputeStep m_volume_reduce_step;
		::pr::compute::ComputeStep m_surface_step;
		::pr::compute::ComputeStep m_surface_reduce_step;
		multicast::AutoSub m_external_force_sub;

		WaterSurface m_water_surface;
		Config m_config;
		std::vector<CompositeSlot> m_composite_hulls;
		std::vector<int> m_pending_body_indices;
		std::vector<int> m_pending_body_generations;
		std::vector<AnalyticResult> m_pending_analytic_results;
		::pr::compute::GpuReadbackBuffer::Allocation m_pending_readback;
		int m_pending_diagnostic_count;

		D3DPtr<ID3D12Resource> m_r_partials;
		D3DPtr<ID3D12Resource> m_r_diagnostics;
		int m_partial_capacity;
		int m_diagnostic_capacity;

		mutable std::mutex m_diagnostics_mutex;
		std::vector<Diagnostics> m_diagnostics;

		// Construct and subscribe the buoyancy compute pass.
		Impl(ID3D12Device* device, Engine& engine, Config const& config, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver)
			:m_device(device)
			,m_step_index_resolver(std::move(step_index_resolver))
			,m_body_state_resolver(std::move(body_state_resolver))
			,m_volume_step(CreateVolumeStep(device))
			,m_volume_reduce_step(CreateVolumeReduceStep(device))
			,m_surface_step(CreateSurfaceStep(device))
			,m_surface_reduce_step(CreateSurfaceReduceStep(device))
			,m_external_force_sub(engine.ExternalForces += [this](Engine& sender, Engine::ExternalForceArgs const& args)
			{
				Apply(sender, args);
			})
			,m_water_surface()
			,m_config()
			,m_composite_hulls()
			,m_pending_body_indices()
			,m_pending_body_generations()
			,m_pending_analytic_results()
			,m_pending_readback()
			,m_pending_diagnostic_count()
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

		// Register a composite convex-primitive buoyancy hull against a stable physics body index.
		void RegisterCompositeHull(RigidBody& body, int body_index, int body_generation, collision::Shape const& shape)
		{
			if (body_index < 0 || body_generation < 0)
			{
				throw std::runtime_error("Invalid body handle for buoyancy hull registration");
			}

			// Flatten/copy the shape up front so the caller's shape may be modified or destroyed after
			// registration, and so a malformed shape (unsupported primitive / un-tessellated polytope)
			// fails loudly here rather than during force evaluation. Throws on bad input.
			auto flattened = buoyancy::FlattenShape(shape);
			if (flattened.Empty())
			{
				throw std::runtime_error("Composite buoyancy hull contains no primitives");
			}

			// Composite slots are indexed by the stable body index, mirroring the legacy hull slots.
			if (body_index >= static_cast<int>(m_composite_hulls.size()))
			{
				m_composite_hulls.resize(static_cast<std::size_t>(body_index + 1));
			}

			auto& slot = m_composite_hulls[body_index];
			if (slot.m_active)
			{
				throw std::runtime_error("A buoyancy hull is already registered for this body");
			}

			auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
			if (body_index >= static_cast<int>(m_diagnostics.size()))
			{
				m_diagnostics.resize(static_cast<std::size_t>(body_index + 1));
			}

			auto& diagnostic = m_diagnostics[body_index];
			diagnostic = Diagnostics{};
			diagnostic.m_body_index = body_index;
			diagnostic.m_body_generation = body_generation;

			slot.m_generation = body_generation;
			slot.m_active = true;
			slot.m_hull = std::move(flattened);

			// Compute the volume-pass sample plan once at registration. CollectPrimitives returns the
			// primitives in the same child order as FlattenShape, so m_vol_counts[k] aligns with
			// slot.m_hull.m_primitives[k]. Counts are distributed proportional to each primitive's
			// volume (so denser sampling tracks larger volumes) and the matching per-sample weight is
			// primitive_volume / count. This mirrors the CPU oracle's plan exactly, which the phase-11
			// parity gate depends on.
			{
				auto const prims = buoyancy::CollectPrimitives(shape);
				auto volumes = std::vector<float>(prims.size(), 0.0f);
				for (std::size_t k = 0; k != prims.size(); ++k)
					volumes[k] = buoyancy::PrimitiveVolume(*prims[k]);

				slot.m_vol_counts = buoyancy::DistributeCounts(volumes, BuoyancyVolumeSampleCount);
				slot.m_vol_dvol.assign(prims.size(), 0.0f);
				slot.m_total_volume_samples = 0;
				for (std::size_t k = 0; k != prims.size(); ++k)
				{
					auto const count = slot.m_vol_counts[k];
					slot.m_vol_dvol[k] = count > 0 ? volumes[k] / static_cast<float>(count) : 0.0f;
					slot.m_total_volume_samples += count;
				}

				// Stable seed for the deterministic sample hash and a scale-relative inside-test slack,
				// computed exactly as the CPU oracle does (CalcBBox extent * 1e-5, floored at 1e-6).
				slot.m_hull_id = static_cast<uint32_t>(body_index);
				auto const bbox = collision::CalcBBox(shape);
				auto const extent = MaxElement(bbox.m_radius.w0());
				slot.m_eps = std::max(1e-6f, extent * 1e-5f);
			}

			// Compute the surface-pass (drag) sample plan once at registration, parallel to the volume
			// plan above but distributed proportional to each primitive's surface area. m_surf_darea[k]
			// is the matching per-sample area weight (primitive area / count). Mirrors the CPU oracle.
			{
				auto const prims = buoyancy::CollectPrimitives(shape);
				auto areas = std::vector<float>(prims.size(), 0.0f);
				for (std::size_t k = 0; k != prims.size(); ++k)
					areas[k] = buoyancy::PrimitiveArea(*prims[k]);

				slot.m_surf_counts = buoyancy::DistributeCounts(areas, BuoyancySurfaceSampleCount);
				slot.m_surf_darea.assign(prims.size(), 0.0f);
				slot.m_total_surface_samples = 0;
				for (std::size_t k = 0; k != prims.size(); ++k)
				{
					auto const count = slot.m_surf_counts[k];
					slot.m_surf_darea[k] = count > 0 ? areas[k] / static_cast<float>(count) : 0.0f;
					slot.m_total_surface_samples += count;
				}
			}

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
			m_water_surface = water_surface.Normalised();
		}

		// Return the current water surface used by buoyancy force dispatches.
		WaterSurface const& GetWaterSurface() const
		{
			return m_water_surface;
		}

		// Set the tunable buoyancy parameters used by subsequent dispatches.
		void SetConfig(Config const& config)
		{
			if (!std::isfinite(config.m_fluid_density) || config.m_fluid_density < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy fluid density must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_drag_time_constant_s))
			{
				throw std::runtime_error("GpuBuoyancy drag time-constant must be finite");
			}
			if (!std::isfinite(config.m_quadratic_drag_coefficient) || config.m_quadratic_drag_coefficient < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy quadratic drag coefficient must be a finite, non-negative value");
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
			if (m_pending_diagnostic_count == 0)
			{
				m_pending_body_indices.clear();
				m_pending_body_generations.clear();
				m_pending_analytic_results.clear();
				m_pending_readback = {};
				return;
			}

			auto const diagnostics = std::span{ m_pending_readback.ptr<GpuBuoyancyDiagnostic>(), static_cast<std::size_t>(m_pending_diagnostic_count) };
			{
				auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
				for (int index = 0; index != m_pending_diagnostic_count; ++index)
				{
					auto const body_index = m_pending_body_indices[index];
					auto const body_generation = m_pending_body_generations[index];
					auto const& analytic = m_pending_analytic_results[index];
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
					diag.m_analytic_valid = analytic.m_valid;
					diag.m_analytic_volume_m3 = analytic.m_volume_m3;
					diag.m_analytic_force_ws = analytic.m_force_ws;
					diag.m_analytic_centre_buoyancy_ws = analytic.m_centre_buoyancy_ws;
					diag.m_analytic_torque_ws = analytic.m_torque_ws;
					diag.m_volume_error_m3 = analytic.m_valid ? diag.m_volume_m3 - analytic.m_volume_m3 : 0.0f;
					diag.m_force_error_ws = analytic.m_valid ? diag.m_force_ws - analytic.m_force_ws : v4::Zero();
					diag.m_centre_buoyancy_error_ws = analytic.m_valid ? diag.m_centre_buoyancy_ws - analytic.m_centre_buoyancy_ws : v4::Zero();
					diag.m_torque_error_ws = analytic.m_valid ? diag.m_torque_ws - analytic.m_torque_ws : v4::Zero();
				}
			}

			m_pending_body_indices.clear();
			m_pending_body_generations.clear();
			m_pending_analytic_results.clear();
			m_pending_readback = {};
			m_pending_diagnostic_count = 0;
		}

		// Record the buoyancy force and diagnostic compute work into the active physics GPU job.
		void Apply(Engine&, Engine::ExternalForceArgs const& args)
		{
			m_pending_body_indices.clear();
			m_pending_body_generations.clear();
			m_pending_analytic_results.clear();
			m_pending_readback = {};
			m_pending_diagnostic_count = 0;

			// All buoyancy now flows through the sampled-composite volume/surface dispatch.
			DispatchComposite(args);
		}

		// Record the sampled-composite buoyancy volume pass into the active physics GPU job. It uploads
		// the flattened composite primitives + interior tet/face geometry for every active hull, runs the
		// per-sample Froude-Krylov volume kernel into per-threadgroup partials, then reduces them to a
		// per-body force/torque accumulation plus one diagnostic record per hull. The diagnostic analytic
		// comparison is left invalid because there is no closed-form composite-union result in v1
		// (CompleteStep tolerates m_valid == false).
		void DispatchComposite(Engine::ExternalForceArgs const& args)
		{
			if (args.m_body_count == 0)
			{
				return;
			}

			// Build the compact active-hull list, resolving each stable registration index to a body
			// step index in this BeginStep() range. Inactive or out-of-range hulls are skipped.
			struct ActiveHull
			{
				int m_body_index;        // stable registration index (diagnostics + hash seed)
				int m_body_generation;   // generation for readback validation
				int m_body_step_index;   // index into the engine body list (g_bodies)
				CompositeSlot const* m_slot;
			};
			auto active = std::vector<ActiveHull>{};
			active.reserve(m_composite_hulls.size());
			auto max_total_volume_samples = 0;
			auto max_total_surface_samples = 0;
			for (auto body_index = 0; body_index != static_cast<int>(m_composite_hulls.size()); ++body_index)
			{
				auto const& slot = m_composite_hulls[body_index];
				if (!slot.m_active)
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

				active.push_back(ActiveHull{
					.m_body_index = body_index,
					.m_body_generation = slot.m_generation,
					.m_body_step_index = body_step_index,
					.m_slot = &slot,
				});
				max_total_volume_samples = std::max(max_total_volume_samples, slot.m_total_volume_samples);
				max_total_surface_samples = std::max(max_total_surface_samples, slot.m_total_surface_samples);
			}
			if (active.empty())
			{
				return;
			}

			auto const hull_count = static_cast<int>(active.size());

			// Groups are laid out [hull0 groups][hull1 groups]..., with a UNIFORM group count per hull
			// (the kernel derives the hull from global_group_index / groups_per_hull). Size it for the
			// busiest hull. The reducer sums one partial per group on a single thread group, so the
			// group count must not exceed the reduce thread count.
			auto const groups_per_hull = std::max(1, (max_total_volume_samples + BuoyancyVolumeThreadCount - 1) / BuoyancyVolumeThreadCount);
			if (groups_per_hull > BuoyancyVolumeReduceThreadCount)
			{
				throw std::runtime_error("Buoyancy composite hull exceeds the maximum supported volume sample count");
			}

			// Surface (drag) groups are sized independently from the volume groups because each hull's
			// surface-sample count differs from its volume-sample count. The reduce pass sums one
			// partial per group on a single thread group, so the group count must not exceed the reduce
			// thread count. Both passes share the per-threadgroup partials buffer, so it must be sized
			// for whichever pass needs the most groups.
			auto const surf_groups_per_hull = std::max(1, (max_total_surface_samples + BuoyancyVolumeThreadCount - 1) / BuoyancyVolumeThreadCount);
			if (surf_groups_per_hull > BuoyancyVolumeReduceThreadCount)
			{
				throw std::runtime_error("Buoyancy composite hull exceeds the maximum supported surface sample count");
			}

			EnsureGpuCapacity(args.m_job, hull_count * std::max(groups_per_hull, surf_groups_per_hull), hull_count);

			// Sum the concatenated geometry sizes across all active hulls so the upload buffers can be
			// allocated once. Each upload uses at least one element because an empty geometry array still
			// needs a valid GPU virtual address to bind.
			auto total_prims = 0;
			auto total_volume_verts = 0;
			auto total_tets = 0;
			auto total_face_planes = 0;
			auto total_verts = 0;
			auto total_face_verts = 0;
			for (auto const& a : active)
			{
				total_prims += static_cast<int>(a.m_slot->m_hull.m_primitives.size());
				total_volume_verts += static_cast<int>(a.m_slot->m_hull.m_volume_verts.size());
				total_tets += static_cast<int>(a.m_slot->m_hull.m_tets.size());
				total_face_planes += static_cast<int>(a.m_slot->m_hull.m_face_planes.size());
				total_verts += static_cast<int>(a.m_slot->m_hull.m_verts.size());
				total_face_verts += static_cast<int>(a.m_slot->m_hull.m_face_verts.size());
			}

			auto upload_headers = args.m_job.m_upload.template Alloc<GpuBuoyVolHeader>(hull_count);
			auto upload_prims = args.m_job.m_upload.template Alloc<buoyancy::GpuPrimitive>(std::max(total_prims, 1));
			auto upload_records = args.m_job.m_upload.template Alloc<GpuBuoyVolPrimRecord>(std::max(total_prims, 1));
			auto upload_volume_verts = args.m_job.m_upload.template Alloc<v4>(std::max(total_volume_verts, 1));
			auto upload_tets = args.m_job.m_upload.template Alloc<iv4>(std::max(total_tets, 1));
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
			auto face_planes = upload_face_planes.ptr<v4>();

			auto surf_headers = upload_surf_headers.ptr<GpuBuoySurfHeader>();
			auto surf_prims = upload_surf_prims.ptr<buoyancy::GpuPrimitive>();
			auto surf_records = upload_surf_records.ptr<GpuBuoySurfPrimRecord>();
			auto verts = upload_verts.ptr<v4>();
			auto face_verts = upload_face_verts.ptr<iv4>();

			// Walk the active hulls, concatenating each hull's geometry into the shared buffers and
			// shifting every primitive's array offsets by the running bases. Tet-corner and face-vertex
			// indices stay RELATIVE to each primitive's own vertex block (the kernel re-adds the offset),
			// so the geometry blocks are copied verbatim.
			auto prim_base = 0;
			auto vvert_base = 0;
			auto tet_base = 0;
			auto face_base = 0;
			auto svert_base = 0;
			auto sfvert_base = 0;
			for (auto index = 0; index != hull_count; ++index)
			{
				auto const& a = active[index];
				auto const& hull = a.m_slot->m_hull;
				auto const prim_count = static_cast<int>(hull.m_primitives.size());

				headers[index] = GpuBuoyVolHeader{
					.m_body_index = a.m_body_step_index,
					.m_prim_base = prim_base,
					.m_prim_count = prim_count,
					.m_total_volume_samples = a.m_slot->m_total_volume_samples,
					.m_hull_id = a.m_slot->m_hull_id,
					.m_eps = a.m_slot->m_eps,
					.m_pad0 = 0,
					.m_pad1 = 0,
				};

				// Surface header mirrors the volume header but carries the surface-sample total. It
				// shares the primitive base (the surf prims buffer is concatenated in the same order).
				surf_headers[index] = GpuBuoySurfHeader{
					.m_body_index = a.m_body_step_index,
					.m_prim_base = prim_base,
					.m_prim_count = prim_count,
					.m_total_surface_samples = a.m_slot->m_total_surface_samples,
					.m_hull_id = a.m_slot->m_hull_id,
					.m_eps = a.m_slot->m_eps,
					.m_pad0 = 0,
					.m_pad1 = 0,
				};

				for (auto i = 0; i != static_cast<int>(hull.m_volume_verts.size()); ++i)
					volume_verts[vvert_base + i] = hull.m_volume_verts[i];
				for (auto i = 0; i != static_cast<int>(hull.m_tets.size()); ++i)
					tets[tet_base + i] = hull.m_tets[i];
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
						.m_count = a.m_slot->m_vol_counts[k],
						.m_dvol = a.m_slot->m_vol_dvol[k],
					};

					// Surface prims need the surface-vertex offset and face offset shifted instead. The
					// face_base is shared with the volume pass (the same face-plane buffer is bound to the
					// surface step), so the sibling cull reads identical planes in both passes.
					auto sp = hull.m_primitives[k];
					sp.m_vert_ofs += svert_base;
					sp.m_face_ofs += face_base;
					surf_prims[prim_base + k] = sp;

					surf_records[prim_base + k] = GpuBuoySurfPrimRecord{
						.m_count = a.m_slot->m_surf_counts[k],
						.m_darea = a.m_slot->m_surf_darea[k],
					};
				}

				// Composite hulls have no closed-form union analytic in v1; record an invalid analytic
				// result so CompleteStep stores the GPU diagnostic without an error comparison.
				m_pending_body_indices.push_back(a.m_body_index);
				m_pending_body_generations.push_back(a.m_body_generation);
				m_pending_analytic_results.push_back(AnalyticResult{});

				prim_base += prim_count;
				vvert_base += static_cast<int>(hull.m_volume_verts.size());
				tet_base += static_cast<int>(hull.m_tets.size());
				face_base += static_cast<int>(hull.m_face_planes.size());
				svert_base += static_cast<int>(hull.m_verts.size());
				sfvert_base += static_cast<int>(hull.m_face_verts.size());
			}

			// Upload wave parameters (same packing as the legacy dispatch). The volume kernel needs the
			// wave SRV bound even for flat water, so always allocate at least one element.
			auto const wave_count = static_cast<int>(m_water_surface.m_waves.size());
			auto upload_waves = args.m_job.m_upload.template Alloc<GpuBuoyancyWave>(std::max(wave_count, 1));
			auto waves = upload_waves.ptr<GpuBuoyancyWave>();
			waves[0] = GpuBuoyancyWave{};
			for (auto index = 0; index != wave_count; ++index)
			{
				auto const& wave = m_water_surface.m_waves[index];
				waves[index] = GpuBuoyancyWave{
					.m_direction_wavelength_phase_speed = v4(wave.m_direction.x, wave.m_direction.y, wave.m_wavelength, wave.m_phase_speed),
					.m_amplitude = v4(wave.m_amplitude, 0.0f, 0.0f, 0.0f),
				};
			}

			auto const gpu_va = [](auto const& alloc)
			{
				return alloc.m_res->GetGPUVirtualAddress() + alloc.m_ofs;
			};
			auto const headers_va = gpu_va(upload_headers);
			auto const prims_va = gpu_va(upload_prims);
			auto const records_va = gpu_va(upload_records);
			auto const volume_verts_va = gpu_va(upload_volume_verts);
			auto const tets_va = gpu_va(upload_tets);
			auto const face_planes_va = gpu_va(upload_face_planes);
			auto const waves_va = gpu_va(upload_waves);
			auto const surf_headers_va = gpu_va(upload_surf_headers);
			auto const surf_prims_va = gpu_va(upload_surf_prims);
			auto const surf_records_va = gpu_va(upload_surf_records);
			auto const verts_va = gpu_va(upload_verts);
			auto const face_verts_va = gpu_va(upload_face_verts);

			// Volume pass uses only hull_count, groups_per_hull, wave_count, time_s, water_level and
			// fluid_density; the drag fields are left zero.
			auto const cb = CBufGpuBuoyancy{
				.m_hull_count = hull_count,
				.m_groups_per_hull = groups_per_hull,
				.m_wave_count = wave_count,
				.m_time_s = static_cast<float>(args.m_time_s),
				.m_water_level = m_water_surface.m_level,
				.m_fluid_density = m_config.m_fluid_density,
				.m_drag_coefficient = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_pad0 = 0.0f,
			};

			// Evaluate all volume samples into one partial record per hull/threadgroup. Root parameter
			// order must match CreateVolumeStep: b0, u0(bodies), t1(waves), t2(headers), t3(prims),
			// t4(volume_verts), t5(tets), t6(face_planes), t7(records), u1(partials).
			args.m_job.m_cmd_list.SetPipelineState(m_volume_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_volume_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(waves_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(headers_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(prims_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(volume_verts_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(tets_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(face_planes_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(records_va);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(hull_count * groups_per_hull, 1, 1);
			args.m_job.m_barriers.UAV(m_r_partials.get()).Commit();

			// Reduce per-threadgroup partials to one body force/torque accumulation and one diagnostic
			// record per hull. Root parameter order must match CreateVolumeReduceStep: b0, u0(bodies),
			// t2(headers), u1(partials), u2(diagnostics).
			args.m_job.m_cmd_list.SetPipelineState(m_volume_reduce_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_volume_reduce_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(headers_va);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(hull_count, 1, 1);
			args.m_job.m_barriers.UAV(args.m_bodies).UAV(m_r_diagnostics.get()).Commit();

			// Surface (drag) pass. Reuses the per-threadgroup partials buffer (sized above for the
			// larger of the volume / surface group counts) and runs AFTER the volume reduce so it can
			// ADD drag force/torque onto the body accumulator and the existing diagnostic record. The
			// cbuffer carries the surface group count and the real drag coefficients; the volume pass
			// left these zero. Skip entirely when there is no surface work or drag is disabled.
			auto const surf_drag_coefficient = m_config.m_drag_time_constant_s > 0.0f
				? m_config.m_fluid_density / m_config.m_drag_time_constant_s
				: 0.0f;
			auto const surf_quad_coefficient = std::max(0.0f, m_config.m_quadratic_drag_coefficient);
			auto const have_drag = surf_drag_coefficient > 0.0f || surf_quad_coefficient > 0.0f;
			if (have_drag && max_total_surface_samples > 0)
			{
				auto const cb_surf = CBufGpuBuoyancy{
					.m_hull_count = hull_count,
					.m_groups_per_hull = surf_groups_per_hull,
					.m_wave_count = wave_count,
					.m_time_s = static_cast<float>(args.m_time_s),
					.m_water_level = m_water_surface.m_level,
					.m_fluid_density = m_config.m_fluid_density,
					.m_drag_coefficient = surf_drag_coefficient,
					.m_quadratic_drag_coefficient = surf_quad_coefficient,
					.m_pad0 = 0.0f,
				};

				// Evaluate all surface samples into one partial record per hull/threadgroup. Root
				// parameter order must match CreateSurfaceStep: b0, u0(bodies), t1(waves), t3(surf_prims),
				// t6(face_planes), t8(surf_headers), t9(verts), t10(face_verts), t11(surf_records),
				// u1(partials).
				args.m_job.m_cmd_list.SetPipelineState(m_surface_step.m_pso.get());
				args.m_job.m_cmd_list.SetComputeRootSignature(m_surface_step.m_sig.get());
				args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb_surf);
				args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(waves_va);
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(surf_prims_va);
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(face_planes_va);
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(surf_headers_va);
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(verts_va);
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(face_verts_va);
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(surf_records_va);
				args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
				args.m_job.m_cmd_list.Dispatch(hull_count * surf_groups_per_hull, 1, 1);
				args.m_job.m_barriers.UAV(m_r_partials.get()).Commit();

				// Reduce the surface partials and ADD drag onto the body + diagnostic. Root parameter
				// order must match CreateSurfaceReduceStep: b0, u0(bodies), t8(surf_headers),
				// u1(partials), u2(diagnostics).
				args.m_job.m_cmd_list.SetPipelineState(m_surface_reduce_step.m_pso.get());
				args.m_job.m_cmd_list.SetComputeRootSignature(m_surface_reduce_step.m_sig.get());
				args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb_surf);
				args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
				args.m_job.m_cmd_list.AddComputeRootShaderResourceView(surf_headers_va);
				args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
				args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics->GetGPUVirtualAddress());
				args.m_job.m_cmd_list.Dispatch(hull_count, 1, 1);
				args.m_job.m_barriers.UAV(args.m_bodies).UAV(m_r_diagnostics.get()).Commit();
			}

			// Copy diagnostics to a readback allocation after the GPU has produced the final UAV result
			// (volume buoyancy plus, when enabled, the surface drag added above).
			m_pending_readback = args.m_job.m_readback.template Alloc<GpuBuoyancyDiagnostic>(hull_count);
			m_pending_diagnostic_count = hull_count;
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			args.m_job.m_cmd_list.CopyBufferRegion(m_pending_readback, m_r_diagnostics.get(), 0);
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
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

			// New resources start in COMMON but are tracked with a UAV default state, relying on normal D3D12 buffer promotion for first use.
			job.m_barriers.Transition(m_r_partials.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
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
	// This is the analytic counterpart to 'EvaluateHeight' and is shared between visualisation
	// (water mesh shading normals) and the GPU buoyancy pass (lateral hydrostatic force).
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
		,m_analytic_volume_m3()
		,m_analytic_force_ws(v4::Zero())
		,m_analytic_centre_buoyancy_ws(v4::Zero())
		,m_analytic_torque_ws(v4::Zero())
		,m_volume_error_m3()
		,m_force_error_ws(v4::Zero())
		,m_centre_buoyancy_error_ws(v4::Zero())
		,m_torque_error_ws(v4::Zero())
		,m_valid()
		,m_analytic_valid()
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
	GpuBuoyancy::GpuBuoyancy(ID3D12Device* device, Engine& engine, Config const& config, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver)
		:m_impl(std::make_unique<Impl>(device, engine, config, std::move(step_index_resolver), std::move(body_state_resolver)))
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

	// Register a composite convex-primitive buoyancy hull against a stable physics body index.
	GpuBuoyancy::Registration GpuBuoyancy::RegisterCompositeHull(RigidBody& body, int body_index, int body_generation, collision::Shape const& shape)
	{
		m_impl->RegisterCompositeHull(body, body_index, body_generation, shape);
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
