//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/forward.h"
#include "src/core/physics/gpu_buoyancy.h"

namespace las
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
			float m_water_level;
			float m_fluid_density;
			float m_pad0;
			v4 m_gravity_ws;
		};
		static_assert(sizeof(CBufGpuBuoyancy) % sizeof(uint32_t) == 0);

		struct GpuBuoyancyHull
		{
			int m_body_index;
			float m_half_extents[3];
		};
		static_assert(sizeof(GpuBuoyancyHull) == 16);

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

		// Compile a runtime compute shader entry point from the LAS buoyancy shader source.
		std::vector<uint8_t> CompileBuoyancyShader(wchar_t const* entry_point)
		{
			auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};
			return ShaderCompiler{}
				.Source("src/core/physics/gpu_buoyancy.hlsl", resolver)
				.HlslVersion(EHlslVersion::Hlsl2021)
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
				.UAV(hlsl::EUAVReg::u1)
				.Create(device, "LAS.GpuBuoyancy.Columns.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSGpuBuoyancyColumns")).Create(device, "LAS.GpuBuoyancy.Columns.PSO");
			return step;
		}

		// Create the per-hull reduction compute step.
		::pr::compute::ComputeStep CreateReduceStep(ID3D12Device* device)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.SRV(hlsl::ESRVReg::t0)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.Create(device, "LAS.GpuBuoyancy.Reduce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSGpuBuoyancyReduce")).Create(device, "LAS.GpuBuoyancy.Reduce.PSO");
			return step;
		}

		struct VolumeCentroid
		{
			float m_volume_m3;
			v4 m_centroid_ws;
			bool m_valid;
		};

		constexpr float AnalyticWaterLevel = 0.0f;
		constexpr float AnalyticFluidDensity = 1000.0f;
		constexpr v4 AnalyticGravityWS = v4{0.0f, 0.0f, -9.81f, 0.0f};

		// Return true if 'point' is on the submerged side of the current flat water plane.
		bool IsSubmerged(v4 point, float water_level)
		{
			return point.z <= water_level;
		}

		// Return the point where an edge crosses the flat water plane.
		v4 IntersectWaterPlane(v4 a, v4 b, float water_level)
		{
			auto const t = (water_level - a.z) / (b.z - a.z);
			return a + (b - a) * t;
		}

		// Append 'point' if it is not already present within the analytic clipping tolerance.
		void AddUniquePoint(std::vector<v4>& points, v4 point)
		{
			for (auto const& existing : points)
			{
				if (Length(point - existing) < 1e-5f)
					return;
			}

			points.push_back(point);
		}

		// Clip a convex polygon against the submerged half-space of the flat water plane.
		std::vector<v4> ClipPolygonToWater(std::span<v4 const> polygon, float water_level)
		{
			auto clipped = std::vector<v4>{};
			if (polygon.empty())
				return clipped;

			auto prev = polygon.back();
			auto prev_inside = IsSubmerged(prev, water_level);
			for (auto const curr : polygon)
			{
				auto const curr_inside = IsSubmerged(curr, water_level);
				if (curr_inside != prev_inside)
					clipped.push_back(IntersectWaterPlane(prev, curr, water_level));

				if (curr_inside)
					clipped.push_back(curr);

				prev = curr;
				prev_inside = curr_inside;
			}

			return clipped;
		}

		// Sort the waterline cap vertices around the vertical water-plane normal.
		void SortWaterCap(std::vector<v4>& cap)
		{
			auto centre = v4::Zero();
			for (auto const& point : cap)
				centre += point;

			centre /= static_cast<float>(cap.size());
			std::sort(std::begin(cap), std::end(cap), [centre](v4 lhs, v4 rhs)
			{
				auto const lhs_angle = std::atan2(lhs.y - centre.y, lhs.x - centre.x);
				auto const rhs_angle = std::atan2(rhs.y - centre.y, rhs.x - centre.x);
				return lhs_angle < rhs_angle;
			});
		}

		// Return the volume and centroid of the submerged half-space clipped generated box.
		VolumeCentroid SubmergedBoxVolumeCentroid(m4x4 const& o2w, v4 half_extents, float water_level)
		{
			auto const hx = half_extents.x;
			auto const hy = half_extents.y;
			auto const hz = half_extents.z;
			auto const corners = std::vector<v4>
			{
				o2w * v4{-hx, -hy, -hz, 1.0f},
				o2w * v4{+hx, -hy, -hz, 1.0f},
				o2w * v4{+hx, +hy, -hz, 1.0f},
				o2w * v4{-hx, +hy, -hz, 1.0f},
				o2w * v4{-hx, -hy, +hz, 1.0f},
				o2w * v4{+hx, -hy, +hz, 1.0f},
				o2w * v4{+hx, +hy, +hz, 1.0f},
				o2w * v4{-hx, +hy, +hz, 1.0f},
			};

			auto submerged_count = 0;
			for (auto const& corner : corners)
			{
				if (IsSubmerged(corner, water_level))
					++submerged_count;
			}

			if (submerged_count == 0)
				return VolumeCentroid{};

			auto const full_volume = 8.0f * hx * hy * hz;
			if (submerged_count == isize(corners))
				return VolumeCentroid{full_volume, o2w * v4::Origin(), true};

			static constexpr int BoxFaces[6][4] =
			{
				{0, 3, 2, 1},
				{4, 5, 6, 7},
				{0, 1, 5, 4},
				{1, 2, 6, 5},
				{2, 3, 7, 6},
				{3, 0, 4, 7},
			};
			static constexpr int BoxEdges[12][2] =
			{
				{0, 1}, {1, 2}, {2, 3}, {3, 0},
				{4, 5}, {5, 6}, {6, 7}, {7, 4},
				{0, 4}, {1, 5}, {2, 6}, {3, 7},
			};

			auto faces = std::vector<std::vector<v4>>{};
			faces.reserve(7);
			for (auto const& face_indices : BoxFaces)
			{
				auto face = std::vector<v4>
				{
					corners[face_indices[0]],
					corners[face_indices[1]],
					corners[face_indices[2]],
					corners[face_indices[3]],
				};
				auto clipped = ClipPolygonToWater(face, water_level);
				if (isize(clipped) >= 3)
					faces.push_back(std::move(clipped));
			}

			auto cap = std::vector<v4>{};
			for (auto const& edge_indices : BoxEdges)
			{
				auto const a = corners[edge_indices[0]];
				auto const b = corners[edge_indices[1]];
				auto const a_inside = IsSubmerged(a, water_level);
				auto const b_inside = IsSubmerged(b, water_level);
				if (a_inside != b_inside)
					AddUniquePoint(cap, IntersectWaterPlane(a, b, water_level));
			}
			if (isize(cap) >= 3)
			{
				SortWaterCap(cap);
				faces.push_back(std::move(cap));
			}

			auto vertices = std::vector<v4>{};
			for (auto const& face : faces)
			{
				for (auto const& point : face)
					AddUniquePoint(vertices, point);
			}
			if (vertices.empty())
				return VolumeCentroid{};

			auto reference = v4::Zero();
			for (auto const& point : vertices)
				reference += point;

			reference /= static_cast<float>(vertices.size());

			auto volume = 0.0f;
			auto centroid_sum = v4::Zero();
			for (auto const& face : faces)
			{
				auto const& a = face[0];
				for (int i = 1, iend = isize(face) - 1; i != iend; ++i)
				{
					auto const b = face[i];
					auto const c = face[i + 1];
					auto tetra_volume = std::abs(Dot3(a - reference, Cross(b - reference, c - reference))) / 6.0f;
					if (tetra_volume == 0.0f)
						continue;

					auto const tetra_centroid = (reference + a + b + c) * 0.25f;
					volume += tetra_volume;
					centroid_sum += tetra_centroid * tetra_volume;
				}
			}

			if (volume <= 1e-6f)
				return VolumeCentroid{};

			return VolumeCentroid{volume, centroid_sum / volume, true};
		}
	}

	// Construct an invalid analytic body-state snapshot.
	GpuBuoyancy::BodyState::BodyState()
		:m_o2w(m4x4::Identity())
		,m_centre_of_mass_os(v4::Zero())
		,m_valid(false)
	{
	}

	// Construct an invalid buoyancy diagnostic record.
	GpuBuoyancy::Diagnostics::Diagnostics()
		:m_body_slot_index(-1)
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

	// Construct an invalid analytic buoyancy result.
	GpuBuoyancy::AnalyticResult::AnalyticResult()
		:m_volume_m3()
		,m_force_ws(v4::Zero())
		,m_centre_buoyancy_ws(v4::Zero())
		,m_torque_ws(v4::Zero())
		,m_valid()
	{
	}

	// Construct and subscribe the diagnostic-only buoyancy compute pass.
	GpuBuoyancy::GpuBuoyancy(ID3D12Device* device, physics::Engine& engine, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver)
		:m_device(device)
		,m_step_index_resolver(std::move(step_index_resolver))
		,m_body_state_resolver(std::move(body_state_resolver))
		,m_column_step(CreateColumnStep(device))
		,m_reduce_step(CreateReduceStep(device))
		,m_external_force_sub(engine.ExternalForces += [this](physics::Engine& sender, physics::Engine::ExternalForceArgs const& args)
		{
			Apply(sender, args);
		})
		,m_hulls()
		,m_dispatch_hulls()
		,m_pending_body_slots()
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
	}

	// Destroy the buoyancy pass after all physics GPU work has completed.
	GpuBuoyancy::~GpuBuoyancy() = default;

	// Calculate the exact flat-water analytic buoyancy result for a generated box hull.
	GpuBuoyancy::AnalyticResult GpuBuoyancy::CalculateAnalyticBoxBuoyancy(BodyState const& body_state, v4 half_extents)
	{
		auto result = AnalyticResult{};
		if (!body_state.m_valid)
			return result;

		result.m_valid = true;
		auto const volume_centroid = SubmergedBoxVolumeCentroid(body_state.m_o2w, half_extents, AnalyticWaterLevel);
		if (!volume_centroid.m_valid)
			return result;

		auto const centre_of_mass_ws = body_state.m_o2w * body_state.m_centre_of_mass_os.w1();
		result.m_volume_m3 = volume_centroid.m_volume_m3;
		result.m_centre_buoyancy_ws = volume_centroid.m_centroid_ws;
		result.m_force_ws = -AnalyticGravityWS * (AnalyticFluidDensity * volume_centroid.m_volume_m3);
		result.m_torque_ws = Cross(volume_centroid.m_centroid_ws - centre_of_mass_ws, result.m_force_ws);
		return result;
	}

	// Register a generated box buoyancy hull against a LAS physics body slot.
	void GpuBuoyancy::RegisterBoxHull(int body_slot_index, int body_generation, v4 size)
	{
		if (body_slot_index < 0 || body_generation < 0)
		{
			throw std::runtime_error("Invalid body handle for buoyancy hull registration");
		}
		ValidateBoxHullSize(size);

		// Hull slots are indexed by the stable LAS body slot so the per-frame callback only needs to resolve the compact engine step index.
		if (body_slot_index >= static_cast<int>(m_hulls.size()))
		{
			m_hulls.resize(static_cast<std::size_t>(body_slot_index + 1), HullSlot{ -1, v4::Zero(), false });
		}

		auto& hull = m_hulls[body_slot_index];
		if (hull.m_active)
		{
			throw std::runtime_error("A buoyancy hull is already registered for this body");
		}

		auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
		if (body_slot_index >= static_cast<int>(m_diagnostics.size()))
		{
			m_diagnostics.resize(static_cast<std::size_t>(body_slot_index + 1));
		}

		auto& diagnostic = m_diagnostics[body_slot_index];
		diagnostic = Diagnostics{};
		diagnostic.m_body_slot_index = body_slot_index;
		diagnostic.m_body_generation = body_generation;

		hull.m_generation = body_generation;
		hull.m_half_extents = size * 0.5f;
		hull.m_active = true;
	}

	// Remove the buoyancy hull for a LAS physics body slot.
	void GpuBuoyancy::UnregisterHull(int body_slot_index, int body_generation) noexcept
	{
		if (body_slot_index < 0 || body_slot_index >= static_cast<int>(m_hulls.size()))
		{
			return;
		}

		auto& hull = m_hulls[body_slot_index];
		if (!hull.m_active)
		{
			return;
		}
		if (hull.m_generation != body_generation)
		{
			return;
		}

		hull = HullSlot{ -1, v4::Zero(), false };

		auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
		if (body_slot_index < static_cast<int>(m_diagnostics.size()))
		{
			m_diagnostics[body_slot_index] = Diagnostics{};
		}
	}

	// Return the latest diagnostic record for a registered hull.
	GpuBuoyancy::Diagnostics GpuBuoyancy::LatestDiagnostics(int body_slot_index, int body_generation) const
	{
		auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
		if (body_slot_index < 0 || body_slot_index >= static_cast<int>(m_diagnostics.size()))
		{
			return Diagnostics{};
		}

		auto diagnostic = m_diagnostics[body_slot_index];
		if (diagnostic.m_body_generation != body_generation)
		{
			return Diagnostics{};
		}

		return diagnostic;
	}

	// Consume diagnostic readback data after the physics engine has completed its GPU step.
	void GpuBuoyancy::CompleteStep()
	{
		if (m_pending_diagnostic_count == 0)
		{
			m_pending_body_slots.clear();
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
				auto const body_slot_index = m_pending_body_slots[index];
				auto const body_generation = m_pending_body_generations[index];
				auto const& analytic = m_pending_analytic_results[index];
				if (body_slot_index < 0 || body_slot_index >= static_cast<int>(m_diagnostics.size()))
				{
					throw std::runtime_error("GPU buoyancy diagnostic readback referred to an unknown body slot");
				}
				if (m_diagnostics[body_slot_index].m_body_generation != body_generation)
				{
					throw std::runtime_error("GPU buoyancy diagnostic readback generation mismatch");
				}

				auto const& gpu_diag = diagnostics[index];
				auto& diag = m_diagnostics[body_slot_index];
				diag.m_volume_m3 = gpu_diag.m_volume_m3;
				diag.m_force_ws = gpu_diag.m_force_ws;
				diag.m_torque_ws = gpu_diag.m_torque_ws;
				diag.m_centre_buoyancy_ws = gpu_diag.m_centre_buoyancy_ws;
				diag.m_valid = gpu_diag.m_valid != 0;
				diag.m_analytic_volume_m3 = analytic.m_volume_m3;
				diag.m_analytic_force_ws = analytic.m_force_ws;
				diag.m_analytic_centre_buoyancy_ws = analytic.m_centre_buoyancy_ws;
				diag.m_analytic_torque_ws = analytic.m_torque_ws;
				diag.m_volume_error_m3 = diag.m_volume_m3 - analytic.m_volume_m3;
				diag.m_force_error_ws = diag.m_force_ws - analytic.m_force_ws;
				diag.m_centre_buoyancy_error_ws = diag.m_centre_buoyancy_ws - analytic.m_centre_buoyancy_ws;
				diag.m_torque_error_ws = diag.m_torque_ws - analytic.m_torque_ws;
				diag.m_analytic_valid = analytic.m_valid;
			}
		}

		m_pending_body_slots.clear();
		m_pending_body_generations.clear();
		m_pending_analytic_results.clear();
		m_pending_readback = {};
		m_pending_diagnostic_count = 0;
	}

	// Record the diagnostic-only buoyancy compute work into the active physics GPU job.
	void GpuBuoyancy::Apply(physics::Engine&, physics::Engine::ExternalForceArgs const& args)
	{
		m_pending_body_slots.clear();
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
		for (auto body_slot_index = 0; body_slot_index != static_cast<int>(m_hulls.size()); ++body_slot_index)
		{
			auto const& hull = m_hulls[body_slot_index];
			if (!hull.m_active)
			{
				continue;
			}

			auto const body_step_index = m_step_index_resolver(body_slot_index);
			if (body_step_index < 0)
			{
				continue;
			}
			if (body_step_index >= args.m_body_count)
			{
				throw std::runtime_error("Buoyancy hull resolved to an invalid physics step body index");
			}

			m_dispatch_hulls.push_back(DispatchHull{
				.m_body_slot_index = body_slot_index,
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
		for (auto index = 0; index != static_cast<int>(m_dispatch_hulls.size()); ++index)
		{
			auto const& hull = m_dispatch_hulls[index];
			upload_hulls[index] = GpuBuoyancyHull{
				.m_body_index = hull.m_body_step_index,
				.m_half_extents = { hull.m_half_extents.x, hull.m_half_extents.y, hull.m_half_extents.z },
			};

			m_pending_body_slots.push_back(hull.m_body_slot_index);
			m_pending_body_generations.push_back(hull.m_body_generation);
			m_pending_analytic_results.push_back(CalculateAnalyticBoxBuoyancy(m_body_state_resolver(hull.m_body_slot_index), hull.m_half_extents));
		}

		auto const hulls_gpu_va = upload.m_res->GetGPUVirtualAddress() + upload.m_ofs;
		auto const cb = CBufGpuBuoyancy{
			.m_hull_count = hull_count,
			.m_groups_per_hull = BuoyancyGroupsPerHull,
			.m_total_columns = BuoyancyColumnCount,
			.m_grid_x = BuoyancyGridDim,
			.m_grid_y = BuoyancyGridDim,
			.m_water_level = AnalyticWaterLevel,
			.m_fluid_density = AnalyticFluidDensity,
			.m_pad0 = 0.0f,
			.m_gravity_ws = AnalyticGravityWS,
		};

		// Evaluate all column samples into one partial record per hull/threadgroup.
		args.m_job.m_cmd_list.SetPipelineState(m_column_step.m_pso.get());
		args.m_job.m_cmd_list.SetComputeRootSignature(m_column_step.m_sig.get());
		args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
		args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
		args.m_job.m_cmd_list.AddComputeRootShaderResourceView(hulls_gpu_va);
		args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
		args.m_job.m_cmd_list.Dispatch(hull_count * BuoyancyGroupsPerHull, 1, 1);
		args.m_job.m_barriers.UAV(m_r_partials.get()).Commit();

		// Reduce per-threadgroup partials to one diagnostic record per hull. This pass does not write body force accumulators yet.
		args.m_job.m_cmd_list.SetPipelineState(m_reduce_step.m_pso.get());
		args.m_job.m_cmd_list.SetComputeRootSignature(m_reduce_step.m_sig.get());
		args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
		args.m_job.m_cmd_list.AddComputeRootShaderResourceView(hulls_gpu_va);
		args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials->GetGPUVirtualAddress());
		args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics->GetGPUVirtualAddress());
		args.m_job.m_cmd_list.Dispatch(hull_count, 1, 1);
		args.m_job.m_barriers.UAV(m_r_diagnostics.get()).Commit();

		// Copy diagnostics to a separate readback allocation after the GPU has produced the default-heap UAV result.
		m_pending_readback = args.m_job.m_readback.template Alloc<GpuBuoyancyDiagnostic>(hull_count);
		m_pending_diagnostic_count = hull_count;
		args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
		args.m_job.m_cmd_list.CopyBufferRegion(m_pending_readback, m_r_diagnostics.get(), 0);
		args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
	}

	// Resize GPU buffers used by the diagnostic dispatches.
	void GpuBuoyancy::EnsureGpuCapacity(physics::GpuJob& job, int hull_count)
	{
		auto const partial_capacity = hull_count * BuoyancyGroupsPerHull;
		if (partial_capacity > m_partial_capacity)
		{
			m_r_partials = CreateDefaultUavBuffer<GpuBuoyancyPartial>(m_device, partial_capacity, "LAS.GpuBuoyancy.Partials");
			m_partial_capacity = partial_capacity;
		}
		if (hull_count > m_diagnostic_capacity)
		{
			m_r_diagnostics = CreateDefaultUavBuffer<GpuBuoyancyDiagnostic>(m_device, hull_count, "LAS.GpuBuoyancy.Diagnostics");
			m_diagnostic_capacity = hull_count;
		}

		// New resources start in COMMON but are tracked with a UAV default state, relying on normal D3D12 buffer promotion for first use.
		job.m_barriers.Transition(m_r_partials.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}
}
