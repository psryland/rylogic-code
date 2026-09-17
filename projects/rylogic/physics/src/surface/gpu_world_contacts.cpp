//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/surface/gpu_world_contacts.h"
#include "src/collision/shape_cache.h"
#include "src/compute/shader_code.h"
#include "pr/physics/materials/material.h"

namespace pr::physics
{
	using namespace pr::compute;
	namespace
	{
		// Root constants shared with the sampled world-contact shader; modes select terrain or cylinder.
		struct Constants
		{
			uint32_t m_endpoint, m_max_contacts, m_plan_offset;
			float m_height_upper;
			int m_sleeping_enabled, m_island_count;
			uint32_t m_mode, m_material;
			double m_centre_x, m_centre_y, m_radius;
			float m_spacing, m_pad;
		};

		// Upload a complete bounded stream; callers retain buffers until the recorded job completes.
		// Empty input reserves one addressable element without defining its contents.
		template <typename T> void UploadStream(Gpu& gpu, GpuJob& job, D3DPtr<ID3D12Resource>& resource, std::span<T const> data, char const* name)
		{
			if (data.size_bytes() > INT_MAX)
				throw std::runtime_error("World-contact stream exceeds addressable resource size");

			// Keep even an empty stream bindable, growing storage only when the existing allocation is too small.
			auto const count = std::max(size_t{1}, data.size());
			auto const bytes = count * sizeof(T);
			if (!resource || resource->GetDesc().Width < bytes)
				resource = gpu.CreateResource(ResDesc::Buf<T>(count, {}), job.m_cmd_list, name);

			// Stage the source bytes and make the recorded copy visible to subsequent shader reads.
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();
			auto upload = job.m_upload.Alloc<T>(s_cast<int>(count));
			if (!data.empty())
				memcpy(upload.template ptr<T>(), data.data(), data.size_bytes());

			// Copy the reserved range; shaders must not read beyond the supplied element count.
			job.m_cmd_list.CopyBufferRegion(resource.get(), 0, upload);
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}
	}

	// Own validated world sources and their shared sampled contact pipeline.
	GpuWorldContacts::GpuWorldContacts(Gpu& gpu, std::optional<terrain::landscape::BaselineSurface> surface, float spacing, std::optional<CylindricalBoundaryConfig> boundary)
		: m_gpu(gpu), m_surface(std::move(surface)), m_boundary(boundary), m_terrain_spacing(spacing)
		, m_spacing(m_surface ? spacing : boundary ? boundary->m_surface_spacing : spacing), m_height_upper(), m_endpoint()
	{
		surface::ValidateSpacing(spacing);
		if (m_boundary)
			m_boundary->Validate();
		if (m_surface && (m_surface->Config().m_material_id < 0 || m_surface->Config().m_material_id >= Material::MaxMaterialId))
			throw std::runtime_error("Terrain material ID is outside the engine material table");

		// The shapeless world endpoint participates in solving but not in physical shape sampling.
		m_endpoint.SetMassProperties(Inertia::Infinite());

		// Every corner gradient component is in [-1,2], and interpolation is convex. Eight bounds one noise octave.
		auto const recipe = m_surface ? m_surface->Recipe() : terrain::landscape::BaselineSurface{}.Recipe();

		// Bound one field independently of position by summing absolute octave amplitudes.
		auto bound = [&](int index)
		{
			auto const& band = recipe.m_fields[index];
			auto amplitude = std::abs(band.m_amplitude);
			auto sum = 0.0;
			for (int i = 0; i != band.m_octave_count; ++i)
			{
				sum += amplitude;
				amplitude *= std::abs(band.m_persistence);
			}
			return 8 * sum;
		};

		// Include every terrain-family offset and a margin for the normalized blend and rounding.
		auto const local_bound = bound(0) + std::abs(recipe.m_uplift_height_m) + std::abs(recipe.m_mountain_base_height_m) + 35 + std::max({bound(3), bound(4), bound(5)});
		auto const upper = recipe.m_sea_level_bias_m + 2 * local_bound + 1;
		if (!std::isfinite(upper) || std::abs(upper) > std::numeric_limits<float>::max() / 2)
			throw std::runtime_error("Terrain height bound is not representable by the physics engine");

		// Round the rejection height outward so float conversion cannot lower the bound.
		m_height_upper = std::nextafter(static_cast<float>(upper), std::numeric_limits<float>::infinity());

		// The shared pipeline includes the canonical terrain evaluator and requires double arithmetic and 64-bit integer shader operations.
		auto options = D3D12_FEATURE_DATA_D3D12_OPTIONS{};
		auto options1 = D3D12_FEATURE_DATA_D3D12_OPTIONS1{};
		Check(gpu->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &options, sizeof(options)));
		Check(gpu->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS1, &options1, sizeof(options1)));
		if (!options.DoublePrecisionFloatShaderOps || !options1.Int64ShaderOps)
			throw std::runtime_error("Sampled world collision requires FP64 and Int64ShaderOps");

		// Compile the sampled surface stage and the shared dispatch update against their respective root layouts.
		auto resolver = shader_cache::ResourceSourceResolver{};
		auto code = ShaderCompiler().Source("src/surface/gpu_world_contacts.hlsl", resolver).EntryPoint(L"CSWorldContacts").ShaderModel(L"cs_6_0").HlslVersion(EHlslVersion::Hlsl2021).Arg(L"-Gis").Optimise(true).Compile();
		m_step.m_sig = RootSig(ERootSigFlags::ComputeOnly).U32<Constants>(ECBufReg::b0)
			.SRV(ESRVReg::t0).SRV(ESRVReg::t1).SRV(ESRVReg::t2).SRV(ESRVReg::t3).SRV(ESRVReg::t4).SRV(ESRVReg::t5).SRV(ESRVReg::t6)
			.UAV(EUAVReg::u0).UAV(EUAVReg::u1).UAV(EUAVReg::u2).Create(gpu, "WorldContacts:Signature");
		m_step.m_pso = ComputePSO(m_step.m_sig.get(), code).Create(gpu, "WorldContacts:Contacts");
		m_dispatch.m_sig = RootSig(ERootSigFlags::ComputeOnly).U32<Constants>(ECBufReg::b0).UAV(EUAVReg::u0).UAV(EUAVReg::u2).Create(gpu, "WorldContacts:DispatchSignature");
		m_dispatch.m_pso = ComputePSO(m_dispatch.m_sig.get(), shader_code::calc_resolve_dispatch).Create(gpu, "WorldContacts:Dispatch");
	}

	// Invalidate shape-indexed plans after their submitted work has retired.
	void GpuWorldContacts::Reset()
	{
		m_shape_count = 0;
	}

	// Prepare cached primitive plans, this frame's instances, and timing capacity for the requested substeps.
	void GpuWorldContacts::Upload(GpuJob& job, ShapeCache const& shapes, std::span<GpuRigidBody const> bodies, int substeps)
	{
		// Query each substep separately so intervening solver/other physics commands do not inflate terrain GPU time.
		if (substeps < 1 || uint64_t(substeps) * 2 * sizeof(uint64_t) > INT_MAX)
			throw std::runtime_error("World-contact timestamp substep range is not representable");

		// Grow timestamp storage before recording any of the frame's world-contact work.
		auto const query_count = m_surface ? 2u * s_cast<uint32_t>(substeps) : 0u;
		if (m_query_capacity < query_count)
		{
			auto desc = D3D12_QUERY_HEAP_DESC{.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP, .Count = query_count};
			auto queries = D3DPtr<ID3D12QueryHeap>{};
			Check(m_gpu->CreateQueryHeap(&desc, __uuidof(ID3D12QueryHeap), (void**)queries.address_of()));
			auto buffer = ResDesc::Buf<uint64_t>(query_count, {});
			buffer.HeapProps = HeapProps(D3D12_HEAP_TYPE_READBACK);
			buffer.DefaultState = D3D12_RESOURCE_STATE_COPY_DEST;
			m_query_readback = m_gpu.CreateResource(buffer, job.m_cmd_list, "WorldContacts:Timestamps");
			Check(job.m_queue->GetTimestampFrequency(&m_frequency));
			m_queries = std::move(queries);
			m_query_capacity = query_count;
		}
		m_query_count = 0;

		// ShapeCache invalidates packed indices on reset/eviction; immutable plans follow that same lifetime.
		auto upload_plans = shapes.m_changed || m_shape_count != shapes.m_shapes.size();
		if (upload_plans)
		{
			// Different source densities retain independent ranges in the same cached patch stream.
			m_boundary_plan_offset = m_surface && m_boundary && m_spacing != m_boundary->m_surface_spacing ? s_cast<uint32_t>(shapes.m_shapes.size()) : 0;
			if (shapes.m_shapes.size() + m_boundary_plan_offset > INT_MAX / sizeof(Range))
				throw std::runtime_error("World-contact plan ranges exceed addressable resource size");

			// Rebuild ranges only when their packed shape identities have changed.
			m_patches.clear();
			m_ranges.assign(shapes.m_shapes.size() + m_boundary_plan_offset, {});
			m_sources.assign(shapes.m_shapes.size(), nullptr);
			m_planned.assign(shapes.m_shapes.size(), false);
			for (auto const& [shape, entry] : shapes.m_entries)
				m_sources[entry.gpu_index] = shape;

			// Preserve packed-index correspondence until the next shape-cache change.
			m_shape_count = shapes.m_shapes.size();
		}

		// Append primitive plans in packed leaf order, descending through compound arrays.
		auto append = [&](auto&& self, Shape const& shape, int& index, float spacing) -> void
		{
			switch (shape.m_type)
			{
				case collision::EShape::Array:
				{
					auto const& array = shape_cast<collision::ShapeArray>(shape);
					for (auto child = array.begin(); child != array.end(); child = collision::next(child))
						self(self, *child, index, spacing);
					return;
				}
				case collision::EShape::NoShape: { ++index; return; }
				default: { break; }
			}

			// Append each physical leaf in the same order as the packed shape cache.
			auto const plan = surface::BuildPlan(shape, spacing);
			if ((m_patches.size() + plan.m_patches.size()) * sizeof(surface::SurfacePatch) > INT_MAX)
				throw std::runtime_error("World-contact patch stream exceeds addressable resource size");

			// Publish the range only after its additional patch storage has passed the resource bound.
			m_ranges[index++] = Range{s_cast<uint32_t>(m_patches.size()), s_cast<uint32_t>(plan.m_patches.size()), plan.m_count, 0};
			m_patches.insert(m_patches.end(), plan.m_patches.begin(), plan.m_patches.end());
		};

		// The immutable recipe needs only one upload for this terrain source.
		if (!m_recipe)
		{
			auto const recipe = m_surface ? m_surface->Recipe() : terrain::landscape::BaselineSurface{}.Recipe();
			UploadStream(m_gpu, job, m_recipe, std::span(&recipe, 1), "WorldContacts:Recipe");
		}

		// Skip infinite-mass bodies and generate missing plans only when a shape first becomes dynamic.
		m_instances.clear();
		for (uint32_t i = 0; i != bodies.size(); ++i)
		{
			auto const& body = bodies[i];
			if (body.os_com_and_invmass.w == 0 || body.shape_id < 0)
				continue;

			// Compound leaves retain their packed indices and child identities in the per-frame stream.
			auto const& root = shapes.m_shapes[body.shape_id];
			auto const count = root.child_count != 0 ? root.child_count : 1;
			auto const begin = root.child_count != 0 ? root.child_offset : body.shape_id;
			if (!m_planned[body.shape_id])
			{
				auto index = begin;
				append(append, *m_sources[body.shape_id], index, m_spacing);
				if (m_boundary_plan_offset != 0)
				{
					index = begin + s_cast<int>(m_boundary_plan_offset);
					append(append, *m_sources[body.shape_id], index, m_boundary->m_surface_spacing);
				}
				m_planned[body.shape_id] = true;
				upload_plans = true;
			}
			for (int child = 0; child != count; ++child)
			{
				if (m_ranges[begin + child].m_sample_count != 0)
				{
					if (m_instances.size() == 65535)
						throw std::runtime_error("World contacts exceed 65535 convex instance groups");

					// One dispatch group owns each sampled leaf, within the API's X-dimension limit.
					m_instances.push_back(Instance{i, s_cast<uint32_t>(begin + child), s_cast<uint32_t>(child), 0});
				}
			}
		}

		// Refresh immutable shader streams only when packed indices or prepared plans have changed.
		if (upload_plans)
		{
			UploadStream(m_gpu, job, m_plans, std::span<Range const>(m_ranges), "WorldContacts:Plans");
			UploadStream(m_gpu, job, m_patch_buffer, std::span<surface::SurfacePatch const>(m_patches), "WorldContacts:Patches");
		}

		// Upload the current instance list and reserve a status word shared by all internal substeps.
		UploadStream(m_gpu, job, m_instance_buffer, std::span<Instance const>(m_instances), "WorldContacts:Instances");
		if (!m_status)
			m_status = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(StatusWordCount, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "WorldContacts:Status");

		// Clear status before the first world-contact dispatch can report a failure.
		job.m_barriers.Transition(m_status.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();
		auto zero = job.m_upload.Alloc<uint32_t>(StatusWordCount);
		std::fill_n(zero.ptr<uint32_t>(), StatusWordCount, 0u);
		job.m_cmd_list.CopyBufferRegion(m_status.get(), 0, zero);
	}

	// Append active world contacts and refresh the shared solver dispatch for one substep.
	void GpuWorldContacts::Collide(GpuJob& job, int endpoint, int max_contacts, bool sleeping_enabled, int island_count, ID3D12Resource* sleep_islands,
		ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters, ID3D12Resource* dispatch)
	{
		if (m_surface)
			Dispatch(job, 0, endpoint, max_contacts, sleeping_enabled, island_count, sleep_islands, bodies, shapes, contacts, counters, dispatch);
		if (m_boundary)
			Dispatch(job, 1, endpoint, max_contacts, sleeping_enabled, island_count, sleep_islands, bodies, shapes, contacts, counters, dispatch);
	}

	// Share surface sampling, reduction, endpoint lifetime and solver append infrastructure between world surfaces.
	void GpuWorldContacts::Dispatch(GpuJob& job, int mode, int endpoint, int max_contacts, bool sleeping_enabled, int island_count, ID3D12Resource* sleep_islands,
		ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters, ID3D12Resource* dispatch)
	{
		if (m_instances.empty())
			return;

		// Reserve both timestamps before recording any work for this substep.
		if (mode == 0 && m_query_count + 2 > m_query_capacity)
			throw std::runtime_error("World-contact timestamp capacity exceeded");

		// Make packed input and the shared contact stream accessible to the sampled surface shader.
		if (mode == 0)
			job.m_cmd_list.get()->EndQuery(m_queries.get(), D3D12_QUERY_TYPE_TIMESTAMP, m_query_count++);

		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(shapes, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(sleep_islands, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(contacts, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(counters, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_status.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
		job.m_cmd_list.SetPipelineState(m_step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_step.m_sig.get());
		auto const boundary = m_boundary.value_or(CylindricalBoundaryConfig{});
		job.m_cmd_list.AddComputeRoot32BitConstants(Constants{
			.m_endpoint = s_cast<uint32_t>(endpoint), .m_max_contacts = s_cast<uint32_t>(max_contacts), .m_plan_offset = mode == 0 ? 0 : m_boundary_plan_offset,
			.m_height_upper = m_height_upper, .m_sleeping_enabled = sleeping_enabled ? 1 : 0, .m_island_count = island_count,
			.m_mode = s_cast<uint32_t>(mode), .m_material = s_cast<uint32_t>(boundary.m_material_id),
			.m_centre_x = boundary.m_centre_x, .m_centre_y = boundary.m_centre_y, .m_radius = boundary.m_radius,
			.m_spacing = boundary.m_surface_spacing, .m_pad = 0});
		for (auto resource : {bodies, shapes, m_recipe.get(), m_plans.get(), m_patch_buffer.get(), m_instance_buffer.get(), sleep_islands})
			job.m_cmd_list.AddComputeRootShaderResourceView(resource->GetGPUVirtualAddress());

		// Bind append destinations after the read-only streams in root-signature order.
		for (auto resource : {counters, contacts, m_status.get()})
			job.m_cmd_list.AddComputeRootUnorderedAccessView(resource->GetGPUVirtualAddress());

		// Finish world-contact writes before deriving the solver's indirect dispatch from the combined count.
		job.m_cmd_list.Dispatch(s_cast<int>(m_instances.size()), 1, 1);
		job.m_barriers.UAV(counters);
		job.m_barriers.UAV(contacts);
		job.m_barriers.UAV(m_status.get());
		job.m_barriers.Transition(dispatch, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
		job.m_cmd_list.SetPipelineState(m_dispatch.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_dispatch.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(Constants{s_cast<uint32_t>(max_contacts), 0, 0, 0});
		job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(dispatch->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(1, 1, 1);
		job.m_barriers.UAV(dispatch);
		job.m_barriers.Commit();
		if (mode == 0)
			job.m_cmd_list.get()->EndQuery(m_queries.get(), D3D12_QUERY_TYPE_TIMESTAMP, m_query_count++);
	}

	// Record timing and failure-status copies without submitting or waiting for the job.
	ReadbackAlloc GpuWorldContacts::Readback(GpuJob& job)
	{
		if (m_query_count != 0)
			job.m_cmd_list.get()->ResolveQueryData(m_queries.get(), D3D12_QUERY_TYPE_TIMESTAMP, 0, m_query_count, m_query_readback.get(), 0);

		// Preserve frame failure status for the host's pre-publication validation.
		job.m_barriers.Transition(m_status.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();
		auto readback = job.m_readback.Alloc<uint32_t>(StatusWordCount);
		job.m_cmd_list.CopyBufferRegion(readback, m_status.get(), 0);
		return readback;
	}

	// Read only after the owning frame completes. This is queue time, not host recording or waiting time.
	double GpuWorldContacts::GpuTimeMs() const
	{
		if (m_query_count == 0)
			return 0;

		// Sum paired terrain intervals without including intervening GPU stages.
		void* mapped = nullptr;
		auto range = D3D12_RANGE{0, sizeof(uint64_t) * m_query_count};
		Check(m_query_readback->Map(0, &range, &mapped));
		auto const* ticks = static_cast<uint64_t const*>(mapped);
		auto elapsed = uint64_t{};
		for (uint32_t i = 0; i != m_query_count; i += 2)
			elapsed += ticks[i + 1] - ticks[i];

		// Release the read-only mapping and convert queue ticks to milliseconds.
		auto written = D3D12_RANGE{};
		m_query_readback->Unmap(0, &written);
		return 1000.0 * static_cast<double>(elapsed) / m_frequency;
	}
}
