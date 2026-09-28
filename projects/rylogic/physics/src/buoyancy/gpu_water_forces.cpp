//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/buoyancy/gpu_water_forces.h"
#include "pr/physics/shape/shape_mass.h"
#include "pr/compute/compute_pso.h"
#include "pr/compute/compute_step.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/utility/root_signature.h"

namespace pr::physics
{
	using namespace pr::compute;
	namespace
	{
		// Threads per group in CSWaterForces.
		constexpr int WaterForcesThreadCount = 64;

		// Root constants for CSWaterForces. Must match 'CBufWaterForces' in gpu_water_forces.hlsl.
		struct CBufWaterForces
		{
			int m_candidate_count;
			int m_element_count;
			float m_time_s;
			float m_dt;
			float m_water_level;
			float m_density;
			float m_linear_drag_rate;
			float m_quadratic_drag_coefficient;
			float m_angular_drag_rate;
		};
		static_assert(sizeof(CBufWaterForces) % sizeof(uint32_t) == 0);
		static_assert(sizeof(GpuWaterForces::Candidate) == 96);
	}

	// Validate the water parameters. The field validates itself on construction.
	void WaterConfig::Validate() const
	{
		// Density scales every force, so it must be positive; drag values may be zero to disable a term.
		if (!std::isfinite(m_density) || m_density <= 0.0f)
			throw std::invalid_argument("Water density must be finite and positive");
		if (!std::isfinite(m_linear_drag_rate) || m_linear_drag_rate < 0.0f)
			throw std::invalid_argument("Water linear drag rate must be finite and non-negative");
		if (!std::isfinite(m_quadratic_drag_coefficient) || m_quadratic_drag_coefficient < 0.0f)
			throw std::invalid_argument("Water quadratic drag coefficient must be finite and non-negative");
		if (!std::isfinite(m_angular_drag_rate) || m_angular_drag_rate < 0.0f)
			throw std::invalid_argument("Water angular drag rate must be finite and non-negative");
	}

	// Compile the pipeline once; changing the water configuration later does not need a new pipeline.
	GpuWaterForces::GpuWaterForces(Gpu& gpu, WaterConfig config)
		: m_gpu(gpu)
		, m_config(std::move(config))
		, m_step()
		, m_candidates()
		, m_candidates_va()
		, m_elements_va()
	{
		// Root parameter order must match the binding order used in Apply.
		m_config.Validate();
		auto resolver = shader_cache::ResourceSourceResolver{};
		auto code = ShaderCompiler{}
			.Source("src/buoyancy/gpu_water_forces.hlsl", resolver)
			.HlslVersion(EHlslVersion::Hlsl2021)
			.Define(L"SHADER_BUILD")
			.Optimise(true)
			.ShaderModel(L"cs_6_6")
			.EntryPoint(L"CSWaterForces")
			.Compile();
		m_step.m_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<CBufWaterForces>(hlsl::ECBufReg::b0)
			.UAV(hlsl::EUAVReg::u0)
			.SRV(hlsl::ESRVReg::t0)
			.SRV(hlsl::ESRVReg::t1)
			.Create(gpu, "Physics.WaterForces.RootSig");
		m_step.m_pso = ComputePSO(m_step.m_sig.get(), code).Create(gpu, "Physics.WaterForces.PSO");
	}

	// Select candidates on the host so that dry bodies cost one bounds test per frame and no GPU work.
	void GpuWaterForces::BeginFrame(GpuJob& job, std::span<RigidBody* const> bodies, float elapsed_s)
	{
		// Collect the bodies that may reach the highest possible water surface during this frame.
		m_candidates.clear();
		auto const max_height = m_config.m_field.MaxHeight();
		for (int i = 0, iend = isize(bodies); i != iend; ++i)
		{
			// Static bodies do not move, and sleeping bodies wake through the engine's environment-change handling.
			auto const& body = *bodies[i];
			if (body.InvMass() == 0.0f || body.Sleeping() || !body.HasShape())
				continue;
			if (!MayBeWet(body, max_height, elapsed_s))
				continue;

			auto candidate = Candidate{};
			if (MakeCandidate(body, i, candidate))
				m_candidates.push_back(candidate);
		}
		if (m_candidates.empty())
			return;

		// Stage the candidates and the water elements in upload memory that stays valid for the whole recorded frame.
		// The element buffer always has one entry so it can be bound even when the field is flat.
		auto upload_candidates = job.m_upload.Alloc<Candidate>(isize(m_candidates));
		memcpy(upload_candidates.ptr<Candidate>(), m_candidates.data(), m_candidates.size() * sizeof(Candidate));

		auto const elements = m_config.m_field.Elements();
		auto upload_elements = job.m_upload.Alloc<terrain::water::WaterFieldElement>(std::max(isize(elements), 1));
		memset(upload_elements.ptr<std::byte>(), 0, sizeof(terrain::water::WaterFieldElement));
		if (!elements.empty())
			memcpy(upload_elements.ptr<std::byte>(), elements.data(), elements.size_bytes());

		m_candidates_va = upload_candidates.m_res->GetGPUVirtualAddress() + upload_candidates.m_ofs;
		m_elements_va = upload_elements.m_res->GetGPUVirtualAddress() + upload_elements.m_ofs;
	}

	// Add buoyancy and drag to the candidate bodies' force accumulators for one substep.
	void GpuWaterForces::Apply(GpuJob& job, ID3D12Resource* bodies, float dt, double time_s)
	{
		// Dry frames and zero-length substeps record nothing.
		if (m_candidates.empty() || !(dt > 0.0f))
			return;

		// Each thread owns one candidate, and each candidate is a distinct body, so the accumulation needs no atomics.
		auto const cb = CBufWaterForces{
			.m_candidate_count = isize(m_candidates),
			.m_element_count = isize(m_config.m_field.Elements()),
			.m_time_s = static_cast<float>(time_s),
			.m_dt = dt,
			.m_water_level = static_cast<float>(m_config.m_field.Level()),
			.m_density = m_config.m_density,
			.m_linear_drag_rate = m_config.m_linear_drag_rate,
			.m_quadratic_drag_coefficient = m_config.m_quadratic_drag_coefficient,
			.m_angular_drag_rate = m_config.m_angular_drag_rate,
		};
		job.m_cmd_list.SetPipelineState(m_step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(cb);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_candidates_va);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_elements_va);
		job.m_cmd_list.Dispatch((cb.m_candidate_count + WaterForcesThreadCount - 1) / WaterForcesThreadCount, 1, 1);
		job.m_barriers.UAV(bodies).Commit();
	}

	// Build the analytic volume proxy for one body.
	bool GpuWaterForces::MakeCandidate(RigidBody const& body, int body_index, Candidate& candidate)
	{
		// Every proxy displaces the true shape volume when fully submerged.
		auto const& shape = body.Shape();
		auto const volume = CalcMassProperties(shape, 1.0f).m_mass;
		if (!(volume > 0.0f))
			return false;

		// Spheres are exact. Boxes use their own frame. Other shapes use their shape-space bounding box.
		auto const& s2r = shape.m_s2r;
		switch (shape.m_type)
		{
			case EShape::Sphere:
			{
				auto const& sphere = shape_cast<ShapeSphere>(shape);
				candidate = Candidate{
					.m_body_index = body_index,
					.m_proxy = ProxySphere,
					.m_volume = volume,
					.m_pad = 0.0f,
					.m_centre_os = s2r.pos,
					.m_extent_os = v4{sphere.m_radius, sphere.m_radius, sphere.m_radius, 0.0f},
					.m_axis_x_os = s2r.x,
					.m_axis_y_os = s2r.y,
					.m_axis_z_os = s2r.z,
				};
				return true;
			}
			case EShape::Box:
			{
				auto const& box = shape_cast<ShapeBox>(shape);
				candidate = Candidate{
					.m_body_index = body_index,
					.m_proxy = ProxyBox,
					.m_volume = volume,
					.m_pad = 0.0f,
					.m_centre_os = s2r.pos,
					.m_extent_os = box.m_radius.w0(),
					.m_axis_x_os = s2r.x,
					.m_axis_y_os = s2r.y,
					.m_axis_z_os = s2r.z,
				};
				return true;
			}
			default:
			{
				// A degenerate bounding box has no volume to scale.
				auto const& bbox = shape.m_bbox;
				if (!(bbox.m_radius.x > 0.0f && bbox.m_radius.y > 0.0f && bbox.m_radius.z > 0.0f))
					return false;

				candidate = Candidate{
					.m_body_index = body_index,
					.m_proxy = ProxyBox,
					.m_volume = volume,
					.m_pad = 0.0f,
					.m_centre_os = s2r * bbox.m_centre.w1(),
					.m_extent_os = bbox.m_radius.w0(),
					.m_axis_x_os = s2r.x,
					.m_axis_y_os = s2r.y,
					.m_axis_z_os = s2r.z,
				};
				return true;
			}
		}
	}

	// Conservatively test whether the body can reach 'max_height' before the frame ends.
	bool GpuWaterForces::MayBeWet(RigidBody const& body, double max_height, float elapsed_s)
	{
		// Rotation about the centre of mass cannot move any point further than this sphere, whatever the frame's spin.
		auto const bbox = body.BBoxWS();
		auto const com = body.CentreOfMassPositionWS();
		auto const reach = Length(bbox.m_centre - com) + Length(bbox.m_radius.w0());

		// Allow for the current velocity plus twice gravity's displacement over the frame, to cover other bounded forces.
		auto const speed = Length(body.VelocityWS().lin);
		auto const gravity = Length(body.GravityWS());
		auto const drop = speed * elapsed_s + gravity * elapsed_s * elapsed_s;
		return static_cast<double>(com.z) - reach - drop < max_height;
	}
}
