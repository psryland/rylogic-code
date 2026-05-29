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

namespace pr::physics
{
	namespace
	{
		static constexpr int BuoyancyGridDim = 128;
		static constexpr int BuoyancyColumnCount = BuoyancyGridDim * BuoyancyGridDim;
		static constexpr int BuoyancyColumnThreadCount = 256;
		static constexpr int BuoyancyReduceThreadCount = 128;
		static constexpr int BuoyancyGroupsPerHull = (BuoyancyColumnCount + BuoyancyColumnThreadCount - 1) / BuoyancyColumnThreadCount;
		static_assert(BuoyancyGroupsPerHull <= BuoyancyReduceThreadCount);

		struct CBufGpuBuoyancy
		{
			int m_hull_count;
			int m_groups_per_hull;
			int m_total_columns;
			int m_grid_x;
			int m_grid_y;
			int m_wave_count;
			float m_time_s;
			float m_water_level;
			float m_fluid_density;
			float m_drag_coefficient;
			float m_quadratic_drag_coefficient;
			float m_pad0;
		};
		static_assert(sizeof(CBufGpuBuoyancy) % sizeof(uint32_t) == 0);

		struct GpuBuoyancyHull
		{
			int m_body_index;
			float m_half_extents[3];
		};
		static_assert(sizeof(GpuBuoyancyHull) == 16);

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

		// Throw if a buoyancy hull size cannot describe a closed generated box.
		void ValidateBoxHullSize(v4 size)
		{
			if (size.x <= 0.0f || size.y <= 0.0f || size.z <= 0.0f || size.w != 0.0f)
			{
				throw std::runtime_error("Box buoyancy hulls require positive xyz dimensions and w = 0");
			}
		}

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

		// Create the column-evaluation compute step.
		::pr::compute::ComputeStep CreateColumnStep(ID3D12Device* device)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t0)
				.SRV(hlsl::ESRVReg::t1)
				.UAV(hlsl::EUAVReg::u1)
				.Create(device, "Physics.GpuBuoyancy.Columns.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSGpuBuoyancyColumns")).Create(device, "Physics.GpuBuoyancy.Columns.PSO");
			return step;
		}

		// Create the per-hull reduction compute step.
		::pr::compute::ComputeStep CreateReduceStep(ID3D12Device* device)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t0)
				.SRV(hlsl::ESRVReg::t1)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.Create(device, "Physics.GpuBuoyancy.Reduce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSGpuBuoyancyReduce")).Create(device, "Physics.GpuBuoyancy.Reduce.PSO");
			return step;
		}
	}

	struct GpuBuoyancy::Impl
	{
		struct HullSlot
		{
			int m_generation;
			v4 m_half_extents;
			bool m_active;

			// While a hull is registered we mark the owning body NeverSleep so the engine keeps
			// calling Engine::ExternalForces every step (it skips the entire post-pack pipeline
			// when no dynamic bodies are awake). We remember the body and the original NeverSleep
			// flag so unregister can restore the body to its pre-registration sleep behaviour.
			RigidBody* m_body;
			bool m_prev_never_sleep;
		};
		struct DispatchHull
		{
			int m_body_index;
			int m_body_generation;
			int m_body_step_index;
			v4 m_half_extents;
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
		::pr::compute::ComputeStep m_column_step;
		::pr::compute::ComputeStep m_reduce_step;
		multicast::AutoSub m_external_force_sub;

		WaterSurface m_water_surface;
		Config m_config;
		std::vector<HullSlot> m_hulls;
		std::vector<DispatchHull> m_dispatch_hulls;
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
			,m_column_step(CreateColumnStep(device))
			,m_reduce_step(CreateReduceStep(device))
			,m_external_force_sub(engine.ExternalForces += [this](Engine& sender, Engine::ExternalForceArgs const& args)
			{
				Apply(sender, args);
			})
			,m_water_surface()
			,m_config()
			,m_hulls()
			,m_dispatch_hulls()
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

		// Register a generated box buoyancy hull against a stable physics body index.
		void RegisterBoxHull(RigidBody& body, int body_index, int body_generation, v4 size)
		{
			if (body_index < 0 || body_generation < 0)
			{
				throw std::runtime_error("Invalid body handle for buoyancy hull registration");
			}
			ValidateBoxHullSize(size);

			// Hull slots are indexed by the stable body index so the per-frame callback only needs to resolve the compact engine step index.
			if (body_index >= static_cast<int>(m_hulls.size()))
			{
				m_hulls.resize(static_cast<std::size_t>(body_index + 1), HullSlot{ -1, v4::Zero(), false, nullptr, false });
			}

			auto& hull = m_hulls[body_index];
			if (hull.m_active)
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

			hull.m_generation = body_generation;
			hull.m_half_extents = size * 0.5f;
			hull.m_active = true;

			// Remember the body and its prior sleep policy, then force the body to stay awake.
			// Buoyancy is a continuous environmental force: the engine bails out of the GPU pipeline
			// (and therefore ExternalForces) when every dynamic body is asleep, which would freeze
			// a floater in place even as waves pass under it.
			hull.m_body = &body;
			hull.m_prev_never_sleep = body.NeverSleep();
			body.NeverSleep(true);
			body.Wake();
		}

		// Remove the buoyancy hull for a stable physics body index.
		void UnregisterHull(int body_index, int body_generation) noexcept
		{
			if (body_index < 0 || body_index >= static_cast<int>(m_hulls.size()))
			{
				return;
			}

			auto& hull = m_hulls[body_index];
			if (!hull.m_active)
			{
				return;
			}
			if (hull.m_generation != body_generation)
			{
				return;
			}

			// Restore the body's original NeverSleep flag so callers that destroy a buoyancy
			// registration end up with the body's sleep behaviour matching its pre-registration state.
			if (hull.m_body != nullptr)
			{
				hull.m_body->NeverSleep(hull.m_prev_never_sleep);
			}

			hull = HullSlot{ -1, v4::Zero(), false, nullptr, false };

			auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
			if (body_index < static_cast<int>(m_diagnostics.size()))
			{
				m_diagnostics[body_index] = Diagnostics{};
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
			if (args.m_body_count == 0)
			{
				return;
			}

			// Build the compact dispatch table from active hulls whose bodies are present in this Engine::BeginStep() range.
			m_dispatch_hulls.clear();
			for (auto body_index = 0; body_index != static_cast<int>(m_hulls.size()); ++body_index)
			{
				auto const& hull = m_hulls[body_index];
				if (!hull.m_active)
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
					throw std::runtime_error("Buoyancy hull resolved to an invalid physics step body index");
				}

				m_dispatch_hulls.push_back(DispatchHull{
					.m_body_index = body_index,
					.m_body_generation = hull.m_generation,
					.m_body_step_index = body_step_index,
					.m_half_extents = hull.m_half_extents,
				});
			}
			if (m_dispatch_hulls.empty())
			{
				return;
			}

			auto const hull_count = static_cast<int>(m_dispatch_hulls.size());
			EnsureGpuCapacity(args.m_job, hull_count);

			// Upload the body-index/hull table into the physics job upload buffer; the allocation stays alive until the submitted job completes.
			auto upload = args.m_job.m_upload.template Alloc<GpuBuoyancyHull>(hull_count);
			auto upload_hulls = upload.ptr<GpuBuoyancyHull>();
			auto const flat_water = m_water_surface.IsFlat();
			for (auto index = 0; index != static_cast<int>(m_dispatch_hulls.size()); ++index)
			{
				auto const& hull = m_dispatch_hulls[index];
				upload_hulls[index] = GpuBuoyancyHull{
					.m_body_index = hull.m_body_step_index,
					.m_half_extents = { hull.m_half_extents.x, hull.m_half_extents.y, hull.m_half_extents.z },
				};

				m_pending_body_indices.push_back(hull.m_body_index);
				m_pending_body_generations.push_back(hull.m_body_generation);
				m_pending_analytic_results.push_back(flat_water
					? CalculateAnalyticBoxBuoyancy(m_body_state_resolver(hull.m_body_index), hull.m_half_extents, m_water_surface.m_level, m_config.m_fluid_density)
					: AnalyticResult{});
			}

			// Upload wave parameters separately from the root constants so the root signature stays small as the scene adds waves.
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

			auto const hulls_gpu_va = upload.m_res->GetGPUVirtualAddress() + upload.m_ofs;
			auto const waves_gpu_va = upload_waves.m_res->GetGPUVirtualAddress() + upload_waves.m_ofs;

			// c_drag = density / tau_damp gives a per-volume linear damping that decays a body of
			// the same density as the fluid in 'tau' seconds. A non-positive tau disables drag.
			auto const drag_coefficient = m_config.m_drag_time_constant_s > 0.0f
				? m_config.m_fluid_density / m_config.m_drag_time_constant_s
				: 0.0f;

			auto const cb = CBufGpuBuoyancy{
				.m_hull_count = hull_count,
				.m_groups_per_hull = BuoyancyGroupsPerHull,
				.m_total_columns = BuoyancyColumnCount,
				.m_grid_x = BuoyancyGridDim,
				.m_grid_y = BuoyancyGridDim,
				.m_wave_count = wave_count,
				.m_time_s = static_cast<float>(args.m_time_s),
				.m_water_level = m_water_surface.m_level,
				.m_fluid_density = m_config.m_fluid_density,
				.m_drag_coefficient = drag_coefficient,
				.m_quadratic_drag_coefficient = std::max(0.0f, m_config.m_quadratic_drag_coefficient),
				.m_pad0 = 0.0f,
			};

			// Evaluate all column samples into one partial record per hull/threadgroup.
			args.m_job.m_cmd_list.SetPipelineState(m_column_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_column_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(hulls_gpu_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(waves_gpu_va);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(hull_count * BuoyancyGroupsPerHull, 1, 1);
			args.m_job.m_barriers.UAV(m_r_partials.get()).Commit();

			// Reduce per-threadgroup partials to one body force accumulator contribution and one diagnostic record per hull.
			// The reduce kernel also evaluates per-face quadratic form drag on the host body and needs the wave SRV (t1)
			// because each face sub-sample queries the water height at its world-space XY to test submergence.
			args.m_job.m_cmd_list.SetPipelineState(m_reduce_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_reduce_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(hulls_gpu_va);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(waves_gpu_va);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(hull_count, 1, 1);
			args.m_job.m_barriers.UAV(args.m_bodies).UAV(m_r_diagnostics.get()).Commit();

			// Copy diagnostics to a separate readback allocation after the GPU has produced the default-heap UAV result.
			m_pending_readback = args.m_job.m_readback.template Alloc<GpuBuoyancyDiagnostic>(hull_count);
			m_pending_diagnostic_count = hull_count;
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			args.m_job.m_cmd_list.CopyBufferRegion(m_pending_readback, m_r_diagnostics.get(), 0);
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		}

		// Calculate the exact flat-water analytic buoyancy result for a generated box hull.
		// 'fluid_density' must match the density used by the GPU pass so the reported diagnostic
		// error reflects only numerical/quantisation differences and not a density mismatch.
		// The body's own world-space gravity vector (carried in 'body_state') drives both the
		// magnitude and direction of the analytic Archimedes force, mirroring the per-body
		// gravity sampling the HLSL pass performs.
		static AnalyticResult CalculateAnalyticBoxBuoyancy(BodyState const& body_state, v4 half_extents, float water_level, float fluid_density)
		{
			auto result = AnalyticResult{};
			if (!body_state.m_valid)
				return result;

			result.m_valid = true;
			auto const volume_centroid = SubmergedBoxVolumeCentroid(body_state.m_o2w, half_extents, water_level);
			if (!volume_centroid.m_valid)
				return result;

			auto const centre_of_mass_ws = body_state.m_o2w * body_state.m_centre_of_mass_os.w1();
			result.m_volume_m3 = volume_centroid.m_volume_m3;
			result.m_centre_buoyancy_ws = volume_centroid.m_centroid_ws;
			result.m_force_ws = -body_state.m_ws_gravity * (fluid_density * volume_centroid.m_volume_m3);
			result.m_torque_ws = Cross(volume_centroid.m_centroid_ws - centre_of_mass_ws, result.m_force_ws);
			return result;
		}

		// Resize GPU buffers used by the diagnostic dispatches.
		void EnsureGpuCapacity(GpuJob& job, int hull_count)
		{
			auto const partial_capacity = hull_count * BuoyancyGroupsPerHull;
			if (partial_capacity > m_partial_capacity)
			{
				m_r_partials = CreateDefaultUavBuffer<GpuBuoyancyPartial>(m_device, partial_capacity, "Physics.GpuBuoyancy.Partials");
				m_partial_capacity = partial_capacity;
			}
			if (hull_count > m_diagnostic_capacity)
			{
				m_r_diagnostics = CreateDefaultUavBuffer<GpuBuoyancyDiagnostic>(m_device, hull_count, "Physics.GpuBuoyancy.Diagnostics");
				m_diagnostic_capacity = hull_count;
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
		auto const active_registration = std::ranges::any_of(m_impl->m_hulls, [](Impl::HullSlot const& hull)
		{
			return hull.m_active;
		});
		PR_ASSERT(PR_DBG, !active_registration, "GpuBuoyancy registrations must be destroyed before the GpuBuoyancy module");
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

	// Register a generated box buoyancy hull against a stable physics body index.
	GpuBuoyancy::Registration GpuBuoyancy::RegisterBoxHull(RigidBody& body, int body_index, int body_generation, v4 size)
	{
		m_impl->RegisterBoxHull(body, body_index, body_generation, size);
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
