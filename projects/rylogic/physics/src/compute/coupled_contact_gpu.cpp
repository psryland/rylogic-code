//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/cast.h"
#include "src/compute/coupled_contact_gpu.h"
#include "src/compute/gpu_capacity.h"
#include "src/compute/shader_code.h"
#include "src/compute/collide_gpu.h"
#include "src/unittests/shared_gpu.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Emit compiler-derived offsets for the temporary recorder's retained native structs.
		void WriteContactObservationAbi(std::ostream& output);

		// Copy only the reported valid prefix after completion; the capacity tail is never exported as records.
		size_t WriteContactObservationPrefix(std::ostream& stream, ReadbackAlloc const& allocation, int stride, int capacity, int reported)
		{
			if (stride <= 0 || capacity < 0 || reported < 0 || int64_t(stride) * capacity != allocation.m_size)
				throw std::runtime_error("Invalid passive contact observation prefix");

			auto const bytes = size_t(stride) * std::min(capacity, reported);
			if (bytes != 0)
				stream.write(reinterpret_cast<char const*>(allocation.ptr<uint8_t>()), static_cast<std::streamsize>(bytes));

			return bytes;
		}

		// Root-register assignments shared with coupled_contact.hlsl.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Counters = ESRVReg::t0;
			inline static constexpr auto Materials = ESRVReg::t1;
			inline static constexpr auto Links = ESRVReg::t2;
			inline static constexpr auto LinkToWorld = ESRVReg::t3;
			inline static constexpr auto Mobilities = ESRVReg::t4;
			inline static constexpr auto AbaScratch = ESRVReg::t5;
			inline static constexpr auto Ranges = ESRVReg::t6;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto Contacts = EUAVReg::u1;
			inline static constexpr auto Blocks = EUAVReg::u2;
			inline static constexpr auto Scratch = EUAVReg::u3;
			inline static constexpr auto Contributions = EUAVReg::u4;
			inline static constexpr auto EndpointKeys = EUAVReg::u5;
			inline static constexpr auto EndpointOrder = EUAVReg::u6;
			inline static constexpr auto TargetImpulses = EUAVReg::u7;
			inline static constexpr auto ParticipantDegrees = EUAVReg::u8;
			inline static constexpr auto LinkImpulses = EUAVReg::u9;
			inline static constexpr auto TreeSelection = EUAVReg::u10;
			inline static constexpr auto TreeResults = EUAVReg::u11;
			inline static constexpr auto State = EUAVReg::u12;
			inline static constexpr auto RigidPseudo = EUAVReg::u13;
			inline static constexpr auto LinkPseudo = EUAVReg::u14;
			inline static constexpr auto GeneralizedPseudo = EUAVReg::u15;
			inline static constexpr auto ArticulationWork = EUAVReg::u16;
			inline static constexpr auto VelocityDeltas = EUAVReg::u17;
			inline static constexpr auto Articulations = EUAVReg::u18;
			inline static constexpr auto Positions = EUAVReg::u19;
		};

		enum class ECoupledContactPhase
		{
			WarmStart = 0,
			Velocity = 1,
			Position = 2,
		};

		// Create or retain one typed UAV resource using geometric high-water growth.
		template <typename Type>
		void EnsureCoupledContactBuffer(Gpu& gpu, GpuJob& job, D3DPtr<ID3D12Resource>& resource, int required_count, int& capacity, char const* name)
		{
			if (required_count <= capacity)
				return;

			capacity = GrowCapacity(required_count, capacity);
			resource = gpu.CreateResource(ResDesc::Buf<Type>(capacity, {}).usage(EUsage::UnorderedAccess), job, name);
		}

		// Cover a bounded work range without exceeding a dispatch dimension or overflowing the ceiling division.
		iv3 CoupledContactDispatchGroups(int item_count)
		{
			auto const groups = item_count / ConstraintThreadCount + (item_count % ConstraintThreadCount != 0);
			return {std::min(groups, 65535), std::max(1, groups / 65535 + (groups % 65535 != 0)), 1};
		}
	}

	// Temporary contact-stage recorder. Allocations remain pinned until Engine's existing completion fence.
	struct GpuCoupledContactSolver::Observation
	{
		// GPU-dependent prefixes are decoded only after the owning frame completes.
		enum class ECount { Fixed, Contacts, Pairs };

		// One exact tracked copy; source-state restoration is part of the retained plan.
		struct Slice
		{
			int m_step;
			std::string m_stage;
			std::string m_name;
			std::string m_type;
			int m_stride;
			int m_capacity;
			ECount m_count;
			uint32_t m_source_state;
			ReadbackAlloc m_bytes;
		};

		// Each substep keeps its own counter allocation rather than reading the reused final counters.
		struct Step
		{
			int m_index;
			float m_dt;
			int m_body_count;
			int m_rigid_count;
			int m_sort_axis;
			int m_counters = -1;
			int m_material_count = 0;
			int m_velocity_updates = 0;
		};

		std::filesystem::path m_directory;
		std::ofstream m_plan;
		std::ofstream m_records;
		std::ofstream m_frames;
		std::vector<Slice> m_slices;
		std::vector<Step> m_steps;
		int m_frame = 0;

		// Opt in through a process-local diagnostic destination, requiring a fresh directory.
		static std::unique_ptr<Observation> Create()
		{
			auto const length = GetEnvironmentVariableW(L"PR_PHYSICS_CONTACT_CAPTURE", nullptr, 0);
			if (length == 0)
			{
				if (GetLastError() != ERROR_ENVVAR_NOT_FOUND)
					throw std::runtime_error("Cannot read PR_PHYSICS_CONTACT_CAPTURE");

				return {};
			}
			auto text = std::wstring(length, L'\0');
			if (GetEnvironmentVariableW(L"PR_PHYSICS_CONTACT_CAPTURE", text.data(), length) != length - 1)
				throw std::runtime_error("PR_PHYSICS_CONTACT_CAPTURE changed during initialization");

			text.resize(length - 1);
			auto result = std::make_unique<Observation>();
			result->m_directory = text;
			if (!std::filesystem::create_directory(result->m_directory))
				throw std::runtime_error("Passive contact observation requires a new destination");

			result->m_plan.exceptions(std::ios::badbit | std::ios::failbit);
			result->m_records.exceptions(std::ios::badbit | std::ios::failbit);
			result->m_frames.exceptions(std::ios::badbit | std::ios::failbit);
			result->m_plan.open(result->m_directory / "plan.csv");
			result->m_records.open(result->m_directory / "records.csv");
			result->m_frames.open(result->m_directory / "frames.csv");
			result->m_plan << "frame,substep,h_bits,body_count,rigid_count,sort_axis,stage,name,type,stride,capacity,count_kind,source_state,velocity_updates_before_copy\n";
			result->m_records << "frame,substep,stage,name,type,stride,capacity,reported_count,valid_count,offset,bytes,source_state,restored_state\n";
			result->m_frames << "frame,substeps,bytes,status,selective_passes,selective_requested\n";
			auto abi = std::ofstream{};
			abi.exceptions(std::ios::badbit | std::ios::failbit);
			abi.open(result->m_directory / "abi.csv");
			WriteContactObservationAbi(abi);
			abi.close();
			return result;
		}

		// Pin a distinct readback allocation and restore exactly the tracked source state.
		template <typename T>
		void Copy(GpuJob& job, char const* stage, char const* name, char const* type, ID3D12Resource* resource, int count, ECount count_kind = ECount::Fixed)
		{
			if (m_steps.empty() || count < 0 || (count != 0 && resource == nullptr))
				throw std::runtime_error("Missing passive contact observation input");

			if (std::ranges::any_of(m_slices, [&](Slice const& slice) { return slice.m_step == isize(m_steps) - 1 && slice.m_stage == stage && slice.m_name == name; }))
				throw std::runtime_error("Duplicate passive contact observation stage");

			auto const bytes = int64_t(sizeof(T)) * count;
			if (count != 0 && resource->GetDesc().Width < static_cast<UINT64>(bytes))
				throw std::runtime_error("Passive contact observation exceeds the source resource");

			auto allocation = ReadbackAlloc{};
			auto source_state = uint32_t{};
			if (count != 0)
			{
				auto const state = job.m_cmd_list.ResState(resource).Mip0State();
				source_state = static_cast<uint32_t>(state);
				allocation = job.m_readback.Alloc(bytes, alignof(T));
				job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				job.m_cmd_list.CopyBufferRegion(allocation.m_res, allocation.m_ofs, resource, 0, bytes);
				job.m_barriers.Transition(resource, state).Commit();
			}
			auto const& step = m_steps.back();
			m_plan << std::format("{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n", m_frame, step.m_index, std::bit_cast<uint32_t>(step.m_dt), step.m_body_count, step.m_rigid_count, step.m_sort_axis, stage, name, type, sizeof(T), count, static_cast<int>(count_kind), source_state, step.m_velocity_updates);
			m_slices.push_back({isize(m_steps) - 1, stage, name, type, static_cast<int>(sizeof(T)), count, count_kind, source_state, std::move(allocation)});
		}

		// Preserve the exact root constants passed to the observed preparation and position-candidate dispatches.
		void Parameters(char const* stage, GpuCoupledContactParams const& parameters)
		{
			auto output = std::ofstream{};
			output.exceptions(std::ios::badbit | std::ios::failbit);
			output.open(m_directory / std::format("frame-{:06}-substep-{}-{}-params.bin", m_frame, m_steps.back().m_index, stage), std::ios::binary);
			output.write(reinterpret_cast<char const*>(&parameters), sizeof(parameters));
			output.close();
		}

		// Export exact valid prefixes once the normal completion fence has made every retained copy readable.
		void Finish(int selective_passes, uint32_t selective_requested)
		{
			if (m_steps.empty())
				return;

			auto output = std::ofstream{};
			output.exceptions(std::ios::badbit | std::ios::failbit);
			output.open(m_directory / std::format("gpu-frame-{:06}.bin", m_frame), std::ios::binary);
			auto offset = size_t{};
			for (auto const& slice : m_slices)
			{
				auto const& step = m_steps[slice.m_step];
				auto reported = slice.m_capacity;
				switch (slice.m_count)
				{
					case ECount::Fixed: { break; }
					case ECount::Contacts:
					case ECount::Pairs:
					{
						if (step.m_counters < 0)
							throw std::runtime_error("Passive contact observation is missing collision-stage counters");

						auto const& counters = *m_slices[step.m_counters].m_bytes.ptr<GpuCollisionCounters>();
						switch (slice.m_count)
						{
							case ECount::Contacts: { reported = counters.contact_count; break; }
							case ECount::Pairs: { reported = counters.pair_count; break; }
							default: { throw std::logic_error("Unexpected observation prefix kind"); }
						}
						break;
					}
					default: { throw std::logic_error("Unknown observation prefix kind"); }
				}
				auto const bytes = WriteContactObservationPrefix(output, slice.m_bytes, slice.m_stride, slice.m_capacity, reported);
				m_records << std::format("{},{},{},{},{},{},{},{},{},{},{},{},{}\n", m_frame, step.m_index, slice.m_stage, slice.m_name, slice.m_type, slice.m_stride, slice.m_capacity, reported, std::min(reported, slice.m_capacity), offset, bytes, slice.m_source_state, slice.m_source_state);
				offset += bytes;
			}
			output.close();
			m_records.flush();
			m_frames << std::format("{},{},{},gpu_fence_complete,{},{}\n", m_frame, m_steps.size(), offset, selective_passes, selective_requested);
			m_frames.flush();
			m_slices.clear();
			m_steps.clear();
		}

	};

	GpuCoupledContactSolver::~GpuCoupledContactSolver() = default;

	namespace
	{
		// Native layout, not a hand-maintained imitation of HLSL packing.
		void WriteContactObservationAbi(std::ostream& output)
		{
			output << "type,field,offset,bytes,alignment\n";
			#define CONTACT_TYPE(T) output << #T ",_sizeof_,0," << sizeof(T) << ',' << alignof(T) << '\n'
			#define CONTACT_FIELD(T, F) output << #T "," #F "," << offsetof(T, F) << ',' << sizeof(((T*)nullptr)->F) << ",0\n"
			CONTACT_TYPE(float);
			CONTACT_TYPE(int);
			CONTACT_TYPE(uint32_t);
			using ContactObservationPair = std::array<uint32_t, 2>;
			CONTACT_TYPE(ContactObservationPair);
			CONTACT_TYPE(float4);
			CONTACT_TYPE(float4x4);
			CONTACT_TYPE(BBox);
			CONTACT_FIELD(BBox, m_centre);
			CONTACT_FIELD(BBox, m_radius);
			CONTACT_TYPE(GpuSleepData);
			CONTACT_FIELD(GpuSleepData, timer_s);
			CONTACT_FIELD(GpuSleepData, island_id);
			CONTACT_FIELD(GpuSleepData, generation);
			CONTACT_FIELD(GpuSleepData, flags);
			CONTACT_TYPE(GpuRigidBody);
			CONTACT_FIELD(GpuRigidBody, o2w);
			CONTACT_FIELD(GpuRigidBody, momentum_ang);
			CONTACT_FIELD(GpuRigidBody, momentum_lin);
			CONTACT_FIELD(GpuRigidBody, force_ang);
			CONTACT_FIELD(GpuRigidBody, force_lin);
			CONTACT_FIELD(GpuRigidBody, ws_gravity);
			CONTACT_FIELD(GpuRigidBody, inertia_inv_diagonal);
			CONTACT_FIELD(GpuRigidBody, inertia_inv_products);
			CONTACT_FIELD(GpuRigidBody, os_com_and_invmass);
			CONTACT_FIELD(GpuRigidBody, os_bbox);
			CONTACT_FIELD(GpuRigidBody, state_flags);
			CONTACT_FIELD(GpuRigidBody, shape_id);
			CONTACT_FIELD(GpuRigidBody, colour_used);
			CONTACT_FIELD(GpuRigidBody, articulation_collision);
			CONTACT_FIELD(GpuRigidBody, sleep);
			CONTACT_FIELD(GpuRigidBody, collision_layer);
			CONTACT_FIELD(GpuRigidBody, collision_mask);
			CONTACT_TYPE(GpuShape);
			CONTACT_FIELD(GpuShape, s2rb);
			CONTACT_FIELD(GpuShape, rb_bbox);
			CONTACT_FIELD(GpuShape, type);
			CONTACT_FIELD(GpuShape, vert_offset);
			CONTACT_FIELD(GpuShape, vert_count);
			CONTACT_FIELD(GpuShape, material_id);
			CONTACT_FIELD(GpuShape, face_offset);
			CONTACT_FIELD(GpuShape, face_count);
			CONTACT_FIELD(GpuShape, edge_offset);
			CONTACT_FIELD(GpuShape, edge_count);
			CONTACT_FIELD(GpuShape, child_offset);
			CONTACT_FIELD(GpuShape, child_count);
			CONTACT_FIELD(GpuShape, flags);
			CONTACT_FIELD(GpuShape, data);
			CONTACT_TYPE(GpuCollisionPair);
			CONTACT_FIELD(GpuCollisionPair, body_idx_a);
			CONTACT_FIELD(GpuCollisionPair, body_idx_b);
			CONTACT_FIELD(GpuCollisionPair, shape_idx_a);
			CONTACT_FIELD(GpuCollisionPair, shape_idx_b);
			CONTACT_FIELD(GpuCollisionPair, b2a);
			CONTACT_FIELD(GpuCollisionPair, child_idx_a);
			CONTACT_FIELD(GpuCollisionPair, child_idx_b);
			CONTACT_TYPE(GpuResolveContact);
			CONTACT_FIELD(GpuResolveContact, axis);
			CONTACT_FIELD(GpuResolveContact, contact_point);
			CONTACT_FIELD(GpuResolveContact, manifold);
			CONTACT_FIELD(GpuResolveContact, b2a);
			CONTACT_FIELD(GpuResolveContact, body_idx_a);
			CONTACT_FIELD(GpuResolveContact, body_idx_b);
			CONTACT_FIELD(GpuResolveContact, mat_id_a);
			CONTACT_FIELD(GpuResolveContact, mat_id_b);
			CONTACT_FIELD(GpuResolveContact, depth);
			CONTACT_FIELD(GpuResolveContact, collision_time);
			CONTACT_FIELD(GpuResolveContact, feature);
			CONTACT_FIELD(GpuResolveContact, child_idx_a);
			CONTACT_FIELD(GpuResolveContact, warmstart_impulse);
			CONTACT_FIELD(GpuResolveContact, child_idx_b);
			CONTACT_FIELD(GpuResolveContact, approach_speed);
			CONTACT_FIELD(GpuResolveContact, bias_impulse);
			CONTACT_FIELD(GpuResolveContact, friction_impulses);
			CONTACT_TYPE(GpuCoupledContactParams);
			CONTACT_FIELD(GpuCoupledContactParams, max_contacts);
			CONTACT_FIELD(GpuCoupledContactParams, body_count);
			CONTACT_FIELD(GpuCoupledContactParams, rigid_body_count);
			CONTACT_FIELD(GpuCoupledContactParams, articulation_count);
			CONTACT_FIELD(GpuCoupledContactParams, link_count);
			CONTACT_FIELD(GpuCoupledContactParams, target_count);
			CONTACT_FIELD(GpuCoupledContactParams, velocity_tolerance);
			CONTACT_FIELD(GpuCoupledContactParams, phase);
			CONTACT_FIELD(GpuCoupledContactParams, velocity_delta_count);
			CONTACT_FIELD(GpuCoupledContactParams, relaxation);
			CONTACT_FIELD(GpuCoupledContactParams, restitution_scale);
			CONTACT_FIELD(GpuCoupledContactParams, dt);
			CONTACT_FIELD(GpuCoupledContactParams, position_slop);
			CONTACT_FIELD(GpuCoupledContactParams, position_beta);
			CONTACT_FIELD(GpuCoupledContactParams, max_position_speed);
			CONTACT_FIELD(GpuCoupledContactParams, position_iteration_index);
			CONTACT_TYPE(GpuCoupledContactBlock);
			CONTACT_FIELD(GpuCoupledContactBlock, response_0);
			CONTACT_FIELD(GpuCoupledContactBlock, response_1);
			CONTACT_FIELD(GpuCoupledContactBlock, response_2);
			CONTACT_FIELD(GpuCoupledContactBlock, target_normal_speed);
			CONTACT_FIELD(GpuCoupledContactBlock, friction);
			CONTACT_FIELD(GpuCoupledContactBlock, participant_a);
			CONTACT_FIELD(GpuCoupledContactBlock, participant_b);
			CONTACT_TYPE(GpuCoupledContactScratch);
			CONTACT_FIELD(GpuCoupledContactScratch, candidate_impulse);
			CONTACT_FIELD(GpuCoupledContactScratch, position_impulse);
			CONTACT_FIELD(GpuCoupledContactScratch, candidate_position_impulse);
			CONTACT_TYPE(GpuCoupledContactKrylov);
			CONTACT_FIELD(GpuCoupledContactKrylov, velocity_derivative);
			CONTACT_FIELD(GpuCoupledContactKrylov, impulse_derivative);
			CONTACT_FIELD(GpuCoupledContactKrylov, inverse_weights);
			CONTACT_FIELD(GpuCoupledContactKrylov, residual);
			CONTACT_FIELD(GpuCoupledContactKrylov, gradient);
			CONTACT_FIELD(GpuCoupledContactKrylov, direction);
			CONTACT_FIELD(GpuCoupledContactKrylov, step);
			CONTACT_FIELD(GpuCoupledContactKrylov, operand);
			CONTACT_FIELD(GpuCoupledContactKrylov, response);
			CONTACT_FIELD(GpuCoupledContactKrylov, jacobian_product);
			CONTACT_TYPE(GpuCoupledContactState);
			CONTACT_FIELD(GpuCoupledContactState, valid);
			CONTACT_FIELD(GpuCoupledContactState, failure_flags);
			CONTACT_FIELD(GpuCoupledContactState, frame_failure_flags);
			CONTACT_TYPE(GpuConstraintFrame);
			CONTACT_FIELD(GpuConstraintFrame, rotation);
			CONTACT_FIELD(GpuConstraintFrame, position);
			CONTACT_TYPE(GpuConstraintPseudoVelocity);
			CONTACT_FIELD(GpuConstraintPseudoVelocity, angular_velocity);
			CONTACT_FIELD(GpuConstraintPseudoVelocity, linear_velocity);
			CONTACT_TYPE(GpuCollisionCounters);
			CONTACT_FIELD(GpuCollisionCounters, pair_count);
			CONTACT_FIELD(GpuCollisionCounters, contact_count);
			CONTACT_TYPE(GpuFrameForce);
			CONTACT_FIELD(GpuFrameForce, force_ang);
			CONTACT_FIELD(GpuFrameForce, force_lin);
			CONTACT_TYPE(GpuArticulation);
			CONTACT_FIELD(GpuArticulation, identity_low);
			CONTACT_FIELD(GpuArticulation, identity_high);
			CONTACT_FIELD(GpuArticulation, link_offset);
			CONTACT_FIELD(GpuArticulation, link_count);
			CONTACT_FIELD(GpuArticulation, position_offset);
			CONTACT_FIELD(GpuArticulation, position_count);
			CONTACT_FIELD(GpuArticulation, velocity_offset);
			CONTACT_FIELD(GpuArticulation, velocity_count);
			CONTACT_FIELD(GpuArticulation, dof_offset);
			CONTACT_FIELD(GpuArticulation, dof_count);
			CONTACT_FIELD(GpuArticulation, root_type);
			CONTACT_FIELD(GpuArticulation, max_depth);
			CONTACT_FIELD(GpuArticulation, root_to_world);
			CONTACT_TYPE(GpuArticulationLink);
			CONTACT_FIELD(GpuArticulationLink, parent_link_index);
			CONTACT_FIELD(GpuArticulationLink, articulation_index);
			CONTACT_FIELD(GpuArticulationLink, position_offset);
			CONTACT_FIELD(GpuArticulationLink, velocity_offset);
			CONTACT_FIELD(GpuArticulationLink, dof_offset);
			CONTACT_FIELD(GpuArticulationLink, dof_count);
			CONTACT_FIELD(GpuArticulationLink, child_offset);
			CONTACT_FIELD(GpuArticulationLink, child_count);
			CONTACT_FIELD(GpuArticulationLink, proxy_body_index);
			CONTACT_FIELD(GpuArticulationLink, depth);
			CONTACT_FIELD(GpuArticulationLink, joint_matrix_offset);
			CONTACT_FIELD(GpuArticulationLink, joint_type);
			CONTACT_FIELD(GpuArticulationLink, joint_to_parent);
			CONTACT_FIELD(GpuArticulationLink, joint_to_child);
			CONTACT_FIELD(GpuArticulationLink, shape_to_link);
			CONTACT_FIELD(GpuArticulationLink, inertia_diagonal);
			CONTACT_FIELD(GpuArticulationLink, inertia_products);
			CONTACT_FIELD(GpuArticulationLink, inertia_com_and_mass);
			CONTACT_TYPE(GpuArticulationDof);
			CONTACT_FIELD(GpuArticulationDof, axis_and_type);
			CONTACT_TYPE(GpuArticulationSpatialVector);
			CONTACT_FIELD(GpuArticulationSpatialVector, ang);
			CONTACT_FIELD(GpuArticulationSpatialVector, lin);
			CONTACT_TYPE(GpuArticulationSpatialMatrix);
			CONTACT_FIELD(GpuArticulationSpatialMatrix, columns);
			CONTACT_TYPE(GpuArticulationSpatialMobility);
			CONTACT_FIELD(GpuArticulationSpatialMobility, packed);
			CONTACT_TYPE(GpuArticulationMobilityRange);
			CONTACT_FIELD(GpuArticulationMobilityRange, articulation_index);
			CONTACT_FIELD(GpuArticulationMobilityRange, mobility_offset);
			CONTACT_FIELD(GpuArticulationMobilityRange, link_count);
			CONTACT_FIELD(GpuArticulationMobilityRange, velocity_delta_offset);
			CONTACT_TYPE(GpuArticulationAbaDofScratch);
			CONTACT_FIELD(GpuArticulationAbaDofScratch, motion_subspace);
			CONTACT_FIELD(GpuArticulationAbaDofScratch, u_column);
			CONTACT_TYPE(GpuArticulationAbaScratch);
			CONTACT_FIELD(GpuArticulationAbaScratch, child_to_parent);
			CONTACT_FIELD(GpuArticulationAbaScratch, link_to_world_rotation);
			CONTACT_FIELD(GpuArticulationAbaScratch, articulated_inertia);
			CONTACT_FIELD(GpuArticulationAbaScratch, articulated_bias_or_acceleration);
			CONTACT_FIELD(GpuArticulationAbaScratch, link_velocity);
			CONTACT_FIELD(GpuArticulationAbaScratch, joint_bias);
			CONTACT_FIELD(GpuArticulationAbaScratch, solve_valid);
			CONTACT_TYPE(GpuMaterial);
			CONTACT_FIELD(GpuMaterial, friction_static);
			CONTACT_FIELD(GpuMaterial, elasticity_norm);
			CONTACT_FIELD(GpuMaterial, elasticity_tang);
			CONTACT_FIELD(GpuMaterial, elasticity_tors);
			CONTACT_FIELD(GpuMaterial, density);
			CONTACT_TYPE(DispatchArguments);
			CONTACT_FIELD(DispatchArguments, ThreadGroupCountX);
			CONTACT_FIELD(DispatchArguments, ThreadGroupCountY);
			CONTACT_FIELD(DispatchArguments, ThreadGroupCountZ);
			#undef CONTACT_FIELD
			#undef CONTACT_TYPE
		}
	}

#if PR_UNITTESTS
	namespace tests
	{
		// CPU-only checks exercise the actual prefix writer without constructing a GPU or an Engine.
		PRUnitTestClass(ContactStageRecorderTests)
		{
			PRUnitTestMethod(RawPrefixesExcludeCapacityTails, Quick)
			{
				auto words = std::array<uint32_t, 5>{0xAAAAAAAAu, 0x11223344u, 0x55667788u, 0x99887766u, 0xBBBBBBBBu};
				auto allocation = ReadbackAlloc(nullptr, nullptr, reinterpret_cast<uint8_t*>(words.data()), 4, 12);
				auto output = std::ostringstream{};
				PR_EXPECT(WriteContactObservationPrefix(output, allocation, 4, 3, 1) == 4);
				PR_EXPECT(output.str() == std::string(reinterpret_cast<char const*>(&words[1]), 4));
				output.str("");
				PR_EXPECT(WriteContactObservationPrefix(output, allocation, 4, 3, 0) == 0 && output.str().empty());
				PR_EXPECT(WriteContactObservationPrefix(output, allocation, 4, 3, 8) == 12);
				PR_EXPECT(output.str() == std::string(reinterpret_cast<char const*>(&words[1]), 12));
				PR_THROWS(WriteContactObservationPrefix(output, allocation, 4, 3, -1), std::runtime_error);
				PR_THROWS(WriteContactObservationPrefix(output, allocation, 8, 3, 1), std::runtime_error);
				log_stream() << "CONTACT_ABI_BEGIN\n";
				WriteContactObservationAbi(log_stream());
				log_stream() << "CONTACT_ABI_END\n";
			}
		};
	}
#endif

	// Begin one passive substep record after the existing prediction/proxy refresh, before broadphase.
	void GpuCoupledContactSolver::CapturePrediction(GpuJob& job, int substep_index, float dt, GpuIntegrator& integrator, int body_count, int rigid_body_count, int sort_axis)
	{
		if (!m_observation)
			return;

		auto& observation = *m_observation;
		if (substep_index != isize(observation.m_steps))
			throw std::runtime_error("Nonconsecutive passive contact observation substeps");

		if (substep_index == 0)
			++observation.m_frame;

		observation.m_steps.push_back({substep_index, dt, body_count, rigid_body_count, sort_axis});
		CaptureState(job, "prediction");
		observation.Copy<GpuRigidBody>(job, "prediction", "bodies", "GpuRigidBody", integrator.Bodies().get(), body_count);
		observation.Copy<float>(job, "prediction", "forces", "float", m_aba.m_r_forces.get(), m_aba.m_force_count);
		observation.Copy<GpuFrameForce>(job, "prediction", "external_forces", "GpuFrameForce", m_aba.m_r_external_forces.get(), m_aba.m_link_count);
		observation.Copy<GpuFrameForce>(job, "prediction", "world_forces", "GpuFrameForce", m_aba.m_r_world_forces.get(), m_aba.m_link_count);
		if (substep_index == 0)
		{
			// Retain the actual GPU forest mapping and physical descriptors once per publication interval.
			observation.Copy<GpuArticulationLink>(job, "static", "links", "GpuArticulationLink", m_aba.m_r_links.get(), m_aba.m_link_count);
			observation.Copy<GpuArticulationDof>(job, "static", "dofs", "GpuArticulationDof", m_aba.m_r_dofs.get(), m_aba.m_dof_count);
			observation.Copy<uint32_t>(job, "static", "children", "uint32_t", m_aba.m_r_children.get(), m_aba.m_child_count);
			observation.Copy<uint32_t>(job, "static", "level_links", "uint32_t", m_aba.m_r_level_links.get(), m_aba.m_link_count);
		}
		observation.m_plan.flush();
	}

	// Retain collision-stage admission evidence and the actual witness frames, never later corrected proxy frames.
	void GpuCoupledContactSolver::CaptureCollision(GpuJob& job, GpuIntegrator& integrator, GpuCollisionDetector& detector, ID3D12Resource* pairs, int shape_count, int material_count)
	{
		if (!m_observation)
			return;

		auto& observation = *m_observation;
		auto& step = observation.m_steps.back();
		step.m_counters = isize(observation.m_slices);
		step.m_material_count = material_count;
		observation.Copy<GpuCollisionCounters>(job, "collision", "counters", "GpuCollisionCounters", integrator.Counters().get(), 1);
		observation.Copy<GpuCollisionPair>(job, "collision", "pairs", "GpuCollisionPair", pairs, m_config.max_collision_pairs, Observation::ECount::Pairs);
		observation.Copy<GpuResolveContact>(job, "collision", "contacts", "GpuResolveContact", detector.Contacts().get(), m_config.max_collision_pairs, Observation::ECount::Contacts);
		observation.Copy<DispatchArguments>(job, "collision", "dispatch", "DispatchArguments", detector.ResolveDispatchArgs().get(), 1);
		observation.Copy<GpuRigidBody>(job, "collision", "bodies", "GpuRigidBody", integrator.Bodies().get(), step.m_body_count);
		observation.Copy<BBox>(job, "collision", "bounds", "BBox", integrator.AABBBoxes().get(), step.m_body_count);
		observation.Copy<float>(job, "collision", "sort_bounds", "float", integrator.AABBSortAxis().get(), 2 * step.m_body_count);
		observation.Copy<int>(job, "collision", "sort_indices", "int", integrator.AABBBodyIndices().get(), 2 * step.m_body_count);
		if (step.m_index == 0)
			observation.Copy<GpuShape>(job, "static", "shapes", "GpuShape", detector.Shapes().get(), shape_count);

		observation.m_plan.flush();
	}

	// Keep generalized state authoritative; cache names explicitly warn that ApplyPosition has not refreshed them.
	void GpuCoupledContactSolver::CaptureState(GpuJob& job, char const* stage)
	{
		if (!m_observation)
			return;

		auto& observation = *m_observation;
		observation.Copy<GpuArticulation>(job, stage, "articulations", "GpuArticulation", m_aba.m_r_articulations.get(), m_aba.m_articulation_count);
		observation.Copy<float>(job, stage, "positions", "float", m_aba.m_r_positions.get(), m_aba.m_position_count);
		observation.Copy<float>(job, stage, "velocities", "float", m_aba.m_r_velocities.get(), m_aba.m_velocity_count);
		observation.Copy<float>(job, stage, "accelerations", "float", m_aba.m_r_accelerations.get(), m_aba.m_acceleration_count);
		observation.Copy<GpuArticulationAbaScratch>(job, stage, "aba_cache", "GpuArticulationAbaScratch", m_aba.m_r_scratch.get(), m_aba.m_link_count);
		observation.Copy<GpuArticulationAbaDofScratch>(job, stage, "dof_cache", "GpuArticulationAbaDofScratch", m_aba.m_r_dof_scratch.get(), m_aba.m_dof_count);
		observation.Copy<GpuConstraintFrame>(job, stage, "link_frame_cache", "GpuConstraintFrame", m_proxies.LinkToWorld(), m_aba.m_link_count);
	}

	// Retain actual primary state and accepted physical impulses without using candidate convergence as evidence.
	void GpuCoupledContactSolver::CapturePhysical(GpuJob& job, char const* stage)
	{
		// Ordinary momentum is primary too; proxy momentum remains only labelled auxiliary data.
		if (!m_observation)
			return;

		CaptureState(job, stage);
		auto& observation = *m_observation;
		observation.Copy<GpuRigidBody>(job, stage, "bodies", "GpuRigidBody", m_r_bodies.get(), m_body_count);
		observation.Copy<GpuResolveContact>(job, stage, "contacts", "GpuResolveContact", m_r_contacts.get(), m_max_contacts, Observation::ECount::Contacts);
		observation.Copy<GpuCoupledContactScratch>(job, stage, "scratch", "GpuCoupledContactScratch", m_r_scratch.get(), m_max_contacts, Observation::ECount::Contacts);
		observation.Copy<GpuCoupledContactState>(job, stage, "state", "GpuCoupledContactState", m_r_state.get(), 1);
	}

	// Retain initialized shared pseudo velocity and accepted normal-position support around the contact-only update.
	void GpuCoupledContactSolver::CapturePseudo(GpuJob& job, char const* stage)
	{
		if (!m_observation)
			return;

		auto& observation = *m_observation;
		observation.Copy<GpuConstraintPseudoVelocity>(job, stage, "rigid_pseudo", "GpuConstraintPseudoVelocity", m_r_rigid_pseudo.get(), m_rigid_body_count);
		observation.Copy<GpuArticulationSpatialVector>(job, stage, "link_pseudo", "GpuArticulationSpatialVector", m_r_link_pseudo.get(), m_mobility_count);
		observation.Copy<float>(job, stage, "generalized_pseudo", "float", m_r_generalized_pseudo.get(), m_velocity_delta_count);
		observation.Copy<GpuCoupledContactScratch>(job, stage, "scratch", "GpuCoupledContactScratch", m_r_scratch.get(), m_max_contacts, Observation::ECount::Contacts);
		observation.Copy<GpuCoupledContactState>(job, stage, "state", "GpuCoupledContactState", m_r_state.get(), 1);
		observation.Copy<uint32_t>(job, stage, "tree_selection", "uint32_t", m_r_tree_selection.get(), m_articulation_count);
		observation.Copy<uint32_t>(job, stage, "tree_results", "uint32_t", m_r_tree_results.get(), m_articulation_count);
		observation.m_plan.flush();
	}

	// Observe the existing post-position kinematic refresh without issuing another refresh or evaluating new dynamics.
	void GpuCoupledContactSolver::CaptureRefreshed(GpuJob& job, GpuIntegrator& integrator)
	{
		if (!m_observation)
			return;

		CaptureState(job, "refreshed");
		m_observation->Copy<GpuRigidBody>(job, "refreshed", "bodies", "GpuRigidBody", integrator.Bodies().get(), m_observation->m_steps.back().m_body_count);
		m_observation->m_plan.flush();
	}

	// Called only after the owning Engine has consumed its existing successful completion fence.
	void GpuCoupledContactSolver::CompleteCapture(int selective_passes, uint32_t selective_requested)
	{
		if (m_observation)
			m_observation->Finish(selective_passes, selective_requested);
	}

	// Create fixed pipeline state while leaving every contact-dependent resource unallocated.
	GpuCoupledContactSolver::GpuCoupledContactSolver(Gpu& gpu, GpuArticulationForceAba& aba, GpuArticulationLinkProxies& proxies, EngineConfig const& config)
		: m_observation(Observation::Create())
		, m_gpu(gpu)
		, m_config(config)
		, m_aba(aba)
		, m_proxies(proxies)
		, m_mobility(aba)
		, m_impulse_aba(gpu, aba, m_mobility)
		, m_endpoint_sorter(gpu.m_gpu)
		, m_cs_clear()
		, m_cs_prepare()
		, m_cs_prepare_position()
		, m_cs_begin()
		, m_cs_build_warm_start()
		, m_cs_build_components()
		, m_cs_build_candidates()
		, m_cs_build_position_candidates()
		, m_cs_canonicalise_endpoints()
		, m_cs_gather()
		, m_cs_validate_trees()
		, m_cs_validate_position_trees()
		, m_cs_select_trees()
		, m_cs_commit()
		, m_cs_commit_position_articulations()
		, m_cs_apply_position()
		, m_r_blocks()
		, m_r_scratch()
		, m_r_contributions()
		, m_r_endpoint_keys()
		, m_r_endpoint_order()
		, m_r_contact_order()
		, m_r_contact_nodes()
		, m_r_contact_krylov()
		, m_r_target_impulses()
		, m_r_participant_degrees()
		, m_r_tree_selection()
		, m_r_tree_results()
		, m_r_state()
		, m_r_rigid_pseudo()
		, m_r_link_pseudo()
		, m_r_generalized_pseudo()
		, m_r_uav_sentinel()
		, m_r_counters()
		, m_r_contacts()
		, m_r_bodies()
		, m_r_materials()
		, m_max_contacts()
		, m_body_count()
		, m_rigid_body_count()
		, m_articulation_count()
		, m_link_count()
		, m_target_count()
		, m_participant_count()
		, m_mobility_count()
		, m_velocity_delta_count()
		, m_work_count()
		, m_dt()
		, m_restitution_scale()
		, m_active()
		, m_position_active()
		, m_shared_rigid_pseudo()
		, m_stats()
	{
		// Derived forest counts leave space for root-state access without adding per-dispatch constant-buffer allocations.
		auto common_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<GpuCoupledContactParams>(EReg::Params)
			.SRV(EReg::Counters)
			.SRV(EReg::Materials)
			.SRV(EReg::Links)
			.SRV(EReg::LinkToWorld)
			.SRV(EReg::Mobilities)
			.SRV(EReg::AbaScratch)
			.UAV(EReg::Bodies)
			.UAV(EReg::Contacts)
			.UAV(EReg::Blocks)
			.UAV(EReg::Scratch)
			.UAV(EReg::Contributions)
			.UAV(EReg::EndpointKeys)
			.UAV(EReg::EndpointOrder)
			.UAV(EReg::TargetImpulses)
			.UAV(EReg::ParticipantDegrees)
			.UAV(EReg::LinkImpulses)
			.UAV(EReg::TreeSelection)
			.UAV(EReg::TreeResults)
			.UAV(EReg::State)
			.UAV(EReg::RigidPseudo)
			.UAV(EReg::LinkPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.UAV(EReg::Articulations)
			.Create(m_gpu, "Physics:CoupledContactSig");

		auto compile_common = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
		{
			step.m_sig = common_sig;
			step.m_pso = ComputePSO(common_sig.get(), bytecode).Create(m_gpu, name);
		};
		compile_common(m_cs_clear, shader_code::clear_coupled_contacts, "Physics:ClearCoupledContactsPSO");
		compile_common(m_cs_prepare, shader_code::prepare_coupled_contacts, "Physics:PrepareCoupledContactsPSO");
		compile_common(m_cs_prepare_position, shader_code::prepare_coupled_contact_position, "Physics:PrepareCoupledContactPositionPSO");
		compile_common(m_cs_begin, shader_code::begin_coupled_contact_transaction, "Physics:BeginCoupledContactTransactionPSO");
		compile_common(m_cs_build_warm_start, shader_code::build_coupled_contact_warm_start, "Physics:BuildCoupledContactWarmStartPSO");
		compile_common(m_cs_build_position_candidates, shader_code::build_coupled_contact_position_candidates, "Physics:BuildCoupledContactPositionCandidatesPSO");
		compile_common(m_cs_canonicalise_endpoints, shader_code::canonicalise_coupled_contact_endpoints, "Physics:CanonicaliseCoupledContactEndpointsPSO");
		compile_common(m_cs_gather, shader_code::gather_coupled_contact_targets, "Physics:GatherCoupledContactTargetsPSO");
		compile_common(m_cs_validate_trees, shader_code::validate_coupled_contact_trees, "Physics:ValidateCoupledContactTreesPSO");
		compile_common(m_cs_select_trees, shader_code::select_coupled_contact_trees, "Physics:SelectCoupledContactTreesPSO");
		compile_common(m_cs_commit, shader_code::commit_coupled_contacts, "Physics:CommitCoupledContactsPSO");

		// The matrix-free linear operator borrows retained ABA factors, not a second physical state or a new response implementation.
		auto newton_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<GpuCoupledContactParams>(ECBufReg::b0)
			.SRV(ESRVReg::t0).SRV(ESRVReg::t1).SRV(ESRVReg::t2).SRV(ESRVReg::t3).SRV(ESRVReg::t4)
			.SRV(ESRVReg::t5).SRV(ESRVReg::t6).SRV(ESRVReg::t7).SRV(ESRVReg::t8)
			.UAV(EUAVReg::u0).UAV(EUAVReg::u1).UAV(EUAVReg::u2).UAV(EUAVReg::u3).UAV(EUAVReg::u4)
			.UAV(EUAVReg::u5).UAV(EUAVReg::u6).UAV(EUAVReg::u7).UAV(EUAVReg::u8).UAV(EUAVReg::u9)
			.UAV(EUAVReg::u10).UAV(EUAVReg::u11).UAV(EUAVReg::u12)
			.Create(m_gpu, "Physics:CoupledContactNewtonSig");
		m_cs_build_components.m_sig = newton_sig;
		m_cs_build_components.m_pso = ComputePSO(newton_sig.get(), shader_code::build_coupled_contact_components).Create(m_gpu, "Physics:BuildCoupledContactComponentsPSO");
		m_cs_build_candidates.m_sig = newton_sig;
		m_cs_build_candidates.m_pso = ComputePSO(newton_sig.get(), shader_code::build_coupled_contact_candidates).Create(m_gpu, "Physics:BuildCoupledContactCandidatesPSO");

		// Complete-tree position validation and commit bind only canonical ranges and detached response streams.
		auto articulation_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<GpuCoupledContactParams>(EReg::Params)
			.SRV(EReg::Ranges)
			.UAV(EReg::TreeSelection)
			.UAV(EReg::TreeResults)
			.UAV(EReg::State)
			.UAV(EReg::ArticulationWork)
			.UAV(EReg::VelocityDeltas)
			.UAV(EReg::LinkPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.UAV(EReg::Articulations)
			.Create(m_gpu, "Physics:CoupledContactPositionArticulationSig");
		m_cs_validate_position_trees.m_sig = articulation_sig;
		m_cs_validate_position_trees.m_pso = ComputePSO(articulation_sig.get(), shader_code::validate_coupled_contact_position_trees).Create(m_gpu, "Physics:ValidateCoupledContactPositionTreesPSO");
		m_cs_commit_position_articulations.m_sig = articulation_sig;
		m_cs_commit_position_articulations.m_pso = ComputePSO(articulation_sig.get(), shader_code::commit_coupled_contact_position_articulations).Create(m_gpu, "Physics:CommitCoupledContactPositionArticulationsPSO");

		// Final coordinate application cannot bind physical articulation velocity or momentum response buffers.
		auto apply_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<GpuCoupledContactParams>(EReg::Params)
			.SRV(EReg::Links)
			.SRV(EReg::Ranges)
			.UAV(EReg::Bodies)
			.UAV(EReg::RigidPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.UAV(EReg::Articulations)
			.UAV(EReg::Positions)
			.UAV(EReg::State)
			.Create(m_gpu, "Physics:CoupledContactPositionApplySig");
		m_cs_apply_position.m_sig = apply_sig;
		m_cs_apply_position.m_pso = ComputePSO(apply_sig.get(), shader_code::apply_coupled_contact_position).Create(m_gpu, "Physics:ApplyCoupledContactPositionPSO");
	}

	// Select every packed articulation for exact whole-tree contact response in canonical forest order.
	bool GpuCoupledContactSolver::Upload(GpuJob& job, GpuArticulationUpload const& upload)
	{
		m_stats.m_dispatch_count = 0;
		m_impulse_aba.ResetDispatchCount();
		if (upload.m_articulations.empty())
		{
			Deactivate();
			return false;
		}

		// Canonical all-tree participation makes compact mobility indices identical to global packed link indices.
		auto articulation_indices = std::vector<int>(upload.m_articulations.size());
		std::iota(articulation_indices.begin(), articulation_indices.end(), 0);
		if (!m_mobility.Upload(job, upload, articulation_indices))
			throw std::logic_error("Coupled contact mobility rejected a non-empty articulation forest");

		auto const ranges = m_mobility.Ranges();
		for (int articulation_idx = 0; articulation_idx != isize(ranges); ++articulation_idx)
		{
			auto const& range = ranges[articulation_idx];
			auto const& articulation = upload.m_articulations[articulation_idx];
			if (
				range.articulation_index != articulation_idx ||
				range.mobility_offset != articulation.link_offset ||
				range.link_count != articulation.link_count)
				throw std::logic_error("Coupled contact mobility must preserve global packed-link addressing");
		}

		m_articulation_count = isize(upload.m_articulations);
		m_link_count = isize(upload.m_links);
		m_mobility_count = m_link_count;
		m_active = true;

		// A frame failure survives topology refreshes and transactions until the next public forest upload.
		auto state_capacity = m_r_state != nullptr ? 1 : 0;
		EnsureCoupledContactBuffer<GpuCoupledContactState>(m_gpu, job, m_r_state, 1, state_capacity, "Physics:CoupledContactState");
		auto state = job.m_upload.Alloc<GpuCoupledContactState>(1);
		*state.ptr<GpuCoupledContactState>() = GpuCoupledContactState{.valid = 1u};
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(m_r_state.get(), 0, state);
		return true;
	}

	// Borrow the frame-sticky failure record for the existing gathered engine readback.
	ID3D12Resource* GpuCoupledContactSolver::FrameState()
	{
		return m_active ? m_r_state.get() : nullptr;
	}

	// Borrow diagnostics without recording work or exposing an input override.
	ID3D12Resource* GpuCoupledContactSolver::CandidateWork()
	{
		return m_active ? m_r_contact_krylov.get() : nullptr;
	}

	// Release retained optional resources when the frame contains no shaped articulation links.
	void GpuCoupledContactSolver::Deactivate()
	{
		// Internal exact-self and impulse-ABA buffers are part of the optional contact feature and must not survive deactivation.
		m_endpoint_sorter.ReleaseBuffers();
		m_impulse_aba.ReleaseBuffers();
		m_mobility.ReleaseBuffers();
		m_r_blocks = nullptr;
		m_r_scratch = nullptr;
		m_r_contributions = nullptr;
		m_r_endpoint_keys = nullptr;
		m_r_endpoint_order = nullptr;
		m_r_contact_order = nullptr;
		m_r_contact_nodes = nullptr;
		m_r_contact_krylov = nullptr;
		m_r_target_impulses = nullptr;
		m_r_participant_degrees = nullptr;
		m_r_tree_selection = nullptr;
		m_r_tree_results = nullptr;
		m_r_state = nullptr;
		m_r_rigid_pseudo = nullptr;
		m_r_link_pseudo = nullptr;
		m_r_generalized_pseudo = nullptr;
		m_r_uav_sentinel = nullptr;
		m_r_counters = nullptr;
		m_r_contacts = nullptr;
		m_r_bodies = nullptr;
		m_r_materials = nullptr;
		m_max_contacts = 0;
		m_body_count = 0;
		m_rigid_body_count = 0;
		m_articulation_count = 0;
		m_link_count = 0;
		m_target_count = 0;
		m_participant_count = 0;
		m_mobility_count = 0;
		m_velocity_delta_count = 0;
		m_work_count = 0;
		m_dt = 0.0f;
		m_restitution_scale = 0.0f;
		m_active = false;
		m_position_active = false;
		m_shared_rigid_pseudo = false;
		m_stats = {};
	}

	// Build exact-self contact blocks and deterministic transient endpoint topology at the integrated configuration.
	void GpuCoupledContactSolver::PrepareVelocity(
		GpuJob& job,
		float dt,
		int body_count,
		int rigid_body_count,
		int max_contacts,
		D3DPtr<ID3D12Resource> counters,
		D3DPtr<ID3D12Resource> contacts,
		D3DPtr<ID3D12Resource> bodies,
		D3DPtr<ID3D12Resource> materials,
		float restitution_scale)
	{
		if (!m_active)
			return;
		if (!(dt > 0.0f) || !std::isfinite(dt))
			throw std::invalid_argument("Coupled contact preparation requires a finite positive timestep");
		if (rigid_body_count < 0 || body_count < rigid_body_count || body_count - rigid_body_count != m_link_count)
			throw std::invalid_argument("Coupled contact body suffix must contain exactly one proxy per packed articulation link");
		if (max_contacts <= 0 || max_contacts > std::numeric_limits<int>::max() / 2)
			throw std::invalid_argument("Coupled contact capacity must be positive and leave room for two endpoint entries per contact");
		if (counters == nullptr || contacts == nullptr || bodies == nullptr || materials == nullptr)
			throw std::invalid_argument("Coupled contact preparation requires collision, body, and material resources");
		if (!(m_config.constraint_coupled_relaxation > 0.0f) || m_config.constraint_coupled_relaxation > 1.0f || !std::isfinite(m_config.constraint_coupled_relaxation))
			throw std::invalid_argument("Coupled contact relaxation must be finite and in (0,1]");
		if (!(m_config.contact_velocity_tolerance > 0.0f) || !std::isfinite(m_config.contact_velocity_tolerance))
			throw std::invalid_argument("Coupled contact velocity tolerance must be finite and positive");

		m_body_count = body_count;
		m_rigid_body_count = rigid_body_count;
		m_max_contacts = max_contacts;
		m_target_count = rigid_body_count + m_link_count;
		m_participant_count = rigid_body_count + m_articulation_count;
		m_velocity_delta_count = m_mobility.m_velocity_delta_count;
		m_work_count = std::max({1, m_max_contacts, m_target_count, m_participant_count, m_mobility_count, m_velocity_delta_count, m_articulation_count});
		m_dt = dt;
		m_restitution_scale = restitution_scale;
		m_position_active = false;
		m_r_counters = std::move(counters);
		m_r_contacts = std::move(contacts);
		m_r_bodies = std::move(bodies);
		m_r_materials = std::move(materials);
		ResizeBuffers(job);

		// Rebuild fixed-configuration factors before contact blocks query exact self-link response.
		m_mobility.Run(job);
		if (!m_impulse_aba.Prepare(job))
			throw std::logic_error("Coupled contact preparation requires articulation impulse storage");

		DispatchCommon(job, m_cs_clear, static_cast<int>(ECoupledContactPhase::Velocity), m_work_count);
		DispatchCommon(job, m_cs_prepare, static_cast<int>(ECoupledContactPhase::Velocity), m_max_contacts);

		// Radix sorting groups targets; a geometric sort within each segment removes dependence on narrowphase append order.
		m_endpoint_sorter.Bind(job, 2 * m_max_contacts, m_r_endpoint_keys, m_r_endpoint_order);
		m_endpoint_sorter.Sort("Physics::SortCoupledContactEndpoints", job.m_cmd_list);
		m_stats.m_dispatch_count += m_endpoint_sorter.SortDispatchCount();
		job.m_barriers.UAV(m_r_endpoint_keys.get());
		job.m_barriers.UAV(m_r_endpoint_order.get());
		job.m_barriers.Commit();

		DispatchCommon(job, m_cs_canonicalise_endpoints, static_cast<int>(ECoupledContactPhase::Velocity), m_max_contacts);
		DispatchNewton(job, m_cs_build_components, {1, 1, 1});

		if (m_observation)
		{
			// Prepared rows and exact-self mobility are captured before this contact lane applies any warm impulse.
			auto& observation = *m_observation;
			observation.Copy<GpuCoupledContactBlock>(job, "prepared", "blocks", "GpuCoupledContactBlock", m_r_blocks.get(), m_max_contacts, Observation::ECount::Contacts);
			observation.Copy<GpuCoupledContactScratch>(job, "prepared", "scratch", "GpuCoupledContactScratch", m_r_scratch.get(), m_max_contacts, Observation::ECount::Contacts);
			observation.Copy<GpuResolveContact>(job, "prepared", "contacts", "GpuResolveContact", m_r_contacts.get(), m_max_contacts, Observation::ECount::Contacts);
			observation.Copy<GpuRigidBody>(job, "prepared", "bodies", "GpuRigidBody", m_r_bodies.get(), m_body_count);
			observation.Copy<GpuCoupledContactState>(job, "prepared", "state", "GpuCoupledContactState", m_r_state.get(), 1);
			observation.Copy<std::array<uint32_t, 2>>(job, "prepared", "contact_order", "ContactObservationPair", m_r_contact_order.get(), m_max_contacts, Observation::ECount::Contacts);
			observation.Copy<std::array<uint32_t, 2>>(job, "prepared", "contact_nodes", "ContactObservationPair", m_r_contact_nodes.get(), m_participant_count);
			observation.Copy<uint32_t>(job, "prepared", "degrees", "uint32_t", m_r_participant_degrees.get(), m_participant_count);
			observation.Copy<GpuArticulationMobilityRange>(job, "prepared", "ranges", "GpuArticulationMobilityRange", m_mobility.m_r_ranges.get(), m_articulation_count);
			observation.Copy<GpuArticulationSpatialMobility>(job, "prepared", "mobilities", "GpuArticulationSpatialMobility", m_mobility.m_r_mobilities.get(), m_mobility_count);
			observation.Copy<GpuMaterial>(job, "prepared", "materials", "GpuMaterial", m_r_materials.get(), observation.m_steps.back().m_material_count);
			CaptureState(job, "prepared");
		}

		if (m_shared_rigid_pseudo)
		{
			// Shared storage is reacquired for each position phase rather than retained after its constraint owner becomes inactive.
			m_r_rigid_pseudo = nullptr;
			m_shared_rigid_pseudo = false;
		}
		UpdateMemoryStats();
	}

	// Apply cached normal impulses through rigid endpoints and complete articulation trees as one finite transaction.
	void GpuCoupledContactSolver::ApplyWarmStart(GpuJob& job)
	{
		if (!m_active || !(m_config.warm_start_scale > 0.0f))
			return;

		// Observe cache application once, separately from the later constraint/contact sweeps.
		CapturePhysical(job, "warm_before");
		RunTransaction(job, m_cs_build_warm_start, static_cast<int>(ECoupledContactPhase::WarmStart), -1);
		CapturePhysical(job, "warm_after");
	}

	// Execute one component-coupled ordinary-law Newton update through the existing complete-tree transaction.
	void GpuCoupledContactSolver::SolveVelocityIteration(GpuJob& job)
	{
		if (!m_active)
			return;

		// Isolate the last contact update after its preceding constraint transaction, without changing the caller's sweep count.
		if (m_observation && m_observation->m_steps.back().m_velocity_updates + 1 == m_config.solver_iterations)
			CapturePhysical(job, "pre_final_contact");

		RunTransaction(job, m_cs_build_candidates, static_cast<int>(ECoupledContactPhase::Velocity), -1);
		if (m_observation)
			++m_observation->m_steps.back().m_velocity_updates;
	}

	// Initialize detached correction state, optionally sharing rigid pseudo storage with persistent constraints.
	bool GpuCoupledContactSolver::PreparePosition(GpuJob& job, int iteration_count, D3DPtr<ID3D12Resource> rigid_pseudo)
	{
		if (!m_active || iteration_count <= 0)
			return false;

		// The resolver has completed every physical sweep and has not prepared or applied any pseudo motion.
		CapturePhysical(job, "physical");
		if (m_observation)
		{
			// Retain original-law/Krylov work and accepted represented delta/W*delta; inverse_weights.zw records variable candidate/transpose costs.
			m_observation->Copy<GpuCoupledContactKrylov>(job, "physical", "krylov", "GpuCoupledContactKrylov", m_r_contact_krylov.get(), m_max_contacts, Observation::ECount::Contacts);
		}

		if (!(m_config.position_slop >= 0.0f) || !std::isfinite(m_config.position_slop))
			throw std::invalid_argument("Coupled contact position slop must be finite and non-negative");
		if (!(m_config.position_baumgarte >= 0.0f) || !std::isfinite(m_config.position_baumgarte))
			throw std::invalid_argument("Coupled contact position Baumgarte factor must be finite and non-negative");
		if (!(m_config.constraint_max_position_speed >= 0.0f) || !std::isfinite(m_config.constraint_max_position_speed))
			throw std::invalid_argument("Coupled contact maximum position speed must be finite and non-negative");

		// Alias the common rigid stream rather than integrating separate contact and joint corrections.
		if (rigid_pseudo != nullptr)
		{
			if (rigid_pseudo->GetDesc().Width < static_cast<UINT64>(std::max(1, m_rigid_body_count)) * sizeof(GpuConstraintPseudoVelocity))
				throw std::invalid_argument("Shared rigid pseudo storage is smaller than the rigid-body prefix");

			m_r_rigid_pseudo = rigid_pseudo;
			m_stats.m_rigid_pseudo_capacity = 0;
			m_shared_rigid_pseudo = true;
		}
		else
		{
			if (m_shared_rigid_pseudo)
			{
				m_r_rigid_pseudo = nullptr;
				m_stats.m_rigid_pseudo_capacity = 0;
				m_shared_rigid_pseudo = false;
			}
			EnsureCoupledContactBuffer<GpuConstraintPseudoVelocity>(m_gpu, job, m_r_rigid_pseudo, std::max(1, m_rigid_body_count), m_stats.m_rigid_pseudo_capacity, "Physics:CoupledContactRigidPseudo");
		}

		// Articulation contacts cover the complete packed forest, so their storage also admits every coupled joint.
		EnsureCoupledContactBuffer<GpuArticulationSpatialVector>(m_gpu, job, m_r_link_pseudo, std::max(1, m_mobility_count), m_stats.m_link_pseudo_capacity, "Physics:CoupledContactLinkPseudo");
		EnsureCoupledContactBuffer<float>(m_gpu, job, m_r_generalized_pseudo, std::max(1, m_velocity_delta_count), m_stats.m_generalized_pseudo_capacity, "Physics:CoupledContactGeneralizedPseudo");
		m_position_active = true;
		UpdateMemoryStats();
		DispatchCommon(job, m_cs_prepare_position, static_cast<int>(ECoupledContactPhase::Position), m_work_count);
		return true;
	}

	// Return the initialized rigid-prefix and canonical articulation pseudo streams.
	GpuPositionPseudoBuffers GpuCoupledContactSolver::PseudoState() const
	{
		if (!m_position_active)
			throw std::logic_error("Articulation pseudo state requires an active contact position solve");

		return GpuPositionPseudoBuffers{
			.m_rigid_velocities = m_r_rigid_pseudo,
			.m_link_velocities = m_r_link_pseudo,
			.m_generalized_velocities = m_r_generalized_pseudo,
		};
	}

	// Execute one zero-based projected detached penetration-correction iteration with acceleration confined to the initial sweep.
	void GpuCoupledContactSolver::SolvePositionIteration(GpuJob& job, int position_iteration_index)
	{
		if (!m_active)
			return;
		if (position_iteration_index < 0)
			throw std::invalid_argument("Coupled contact position iteration index must be non-negative");

		CapturePseudo(job, "pseudo_before");
		RunTransaction(job, m_cs_build_position_candidates, static_cast<int>(ECoupledContactPhase::Position), position_iteration_index);
		CapturePseudo(job, "pseudo_after");
	}

	// Apply accepted pseudo state once to rigid transforms and articulation coordinates.
	void GpuCoupledContactSolver::ApplyPosition(GpuJob& job)
	{
		if (!m_active)
			return;
		DispatchApplyPosition(job);
		CaptureState(job, "applied");
		if (m_observation)
			m_observation->Copy<GpuRigidBody>(job, "applied", "bodies", "GpuRigidBody", m_r_bodies.get(), m_body_count);

	}

	// Return aggregate logical usage, retained storage, and dispatch cost for the complete current frame.
	GpuCoupledContactStats GpuCoupledContactSolver::Stats() const
	{
		auto stats = m_stats;
		if (!m_active)
			return stats;

		// Include the whole-tree preparation, detached impulse response, and deterministic endpoint sort owned by this optional lane.
		auto const mobility = m_mobility.Stats();
		auto const impulse = m_impulse_aba.Stats();
		auto const sort_bytes = m_endpoint_sorter.AllocatedBufferBytes();
		stats.m_dispatch_count += mobility.m_dispatch_count + impulse.m_dispatch_count;
		stats.m_logical_bytes += mobility.m_logical_bytes + impulse.m_logical_buffer_bytes + sort_bytes;
		stats.m_allocated_feature_bytes += mobility.m_allocated_feature_bytes + impulse.m_allocated_feature_bytes + sort_bytes;
		return stats;
	}

	// Create or geometrically grow all contact, endpoint, target, participant, and tree transaction buffers.
	void GpuCoupledContactSolver::ResizeBuffers(GpuJob& job)
	{
		// Contact-indexed streams grow as one unit so endpoint offsets remain exactly 2*contact_idx across every retained resource.
		if (m_max_contacts > m_stats.m_contact_capacity)
		{
			auto const maximum_capacity = std::numeric_limits<int>::max() / 2;
			auto const doubled_capacity = m_stats.m_contact_capacity <= maximum_capacity / 2
				? 2 * m_stats.m_contact_capacity
				: maximum_capacity;
			m_stats.m_contact_capacity = std::max(m_max_contacts, std::max(1, doubled_capacity));
			auto const endpoint_capacity = 2 * m_stats.m_contact_capacity;
			m_r_blocks = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledContactBlock>(m_stats.m_contact_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactBlocks");
			m_r_scratch = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledContactScratch>(m_stats.m_contact_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactScratch");
			m_r_contributions = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationSpatialVector>(endpoint_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactContributions");
			m_r_endpoint_keys = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(endpoint_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactEndpointKeys");
			m_r_endpoint_order = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(endpoint_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactEndpointOrder");
			m_r_contact_order = m_gpu.CreateResource(ResDesc::Buf<std::array<uint32_t, 2>>(m_stats.m_contact_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactOrder");
			m_r_contact_krylov = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledContactKrylov>(m_stats.m_contact_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactKrylov");
		}

		EnsureCoupledContactBuffer<GpuArticulationSpatialVector>(m_gpu, job, m_r_target_impulses, m_target_count, m_stats.m_target_capacity, "Physics:CoupledContactTargetImpulses");
		EnsureCoupledContactBuffer<uint32_t>(m_gpu, job, m_r_participant_degrees, m_participant_count, m_stats.m_participant_capacity, "Physics:CoupledContactParticipantDegrees");
		if (m_r_contact_nodes == nullptr || m_r_contact_nodes->GetDesc().Width < static_cast<UINT64>(m_stats.m_participant_capacity) * 2 * sizeof(uint32_t))
			m_r_contact_nodes = m_gpu.CreateResource(ResDesc::Buf<std::array<uint32_t, 2>>(m_stats.m_participant_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactNodes");

		EnsureCoupledContactBuffer<uint32_t>(m_gpu, job, m_r_tree_selection, m_articulation_count, m_stats.m_tree_capacity, "Physics:CoupledContactTreeSelection");
		if (m_r_tree_results == nullptr || m_r_tree_results->GetDesc().Width < static_cast<UINT64>(m_stats.m_tree_capacity) * sizeof(uint32_t))
			m_r_tree_results = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(m_stats.m_tree_capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledContactTreeResults");
		auto sentinel_capacity = m_r_uav_sentinel != nullptr ? 1 : 0;
		EnsureCoupledContactBuffer<uint32_t>(m_gpu, job, m_r_uav_sentinel, 1, sentinel_capacity, "Physics:CoupledContactUavSentinel");
		UpdateMemoryStats();
	}

	// Refresh logical and retained byte counts after topology or optional position storage changes.
	void GpuCoupledContactSolver::UpdateMemoryStats()
	{
		m_stats.m_logical_bytes =
			static_cast<size_t>(m_max_contacts) * (
				sizeof(GpuCoupledContactBlock) +
				sizeof(GpuCoupledContactScratch) +
				sizeof(GpuCoupledContactKrylov) +
				2 * sizeof(GpuArticulationSpatialVector) +
				6 * sizeof(uint32_t)) +
			static_cast<size_t>(m_target_count) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_participant_count) * 3 * sizeof(uint32_t) +
			static_cast<size_t>(m_articulation_count) * 2 * sizeof(uint32_t) +
			sizeof(GpuCoupledContactState);
		if (m_position_active)
		{
			m_stats.m_logical_bytes +=
				static_cast<size_t>(m_shared_rigid_pseudo ? 0 : m_rigid_body_count) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(m_mobility_count) * sizeof(GpuArticulationSpatialVector) +
				static_cast<size_t>(m_velocity_delta_count) * sizeof(float);
		}
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_contact_capacity) * (
				sizeof(GpuCoupledContactBlock) +
				sizeof(GpuCoupledContactScratch) +
				sizeof(GpuCoupledContactKrylov) +
				2 * sizeof(GpuArticulationSpatialVector) +
				6 * sizeof(uint32_t)) +
			static_cast<size_t>(m_stats.m_target_capacity) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_stats.m_participant_capacity) * 3 * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_tree_capacity) * 2 * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_rigid_pseudo_capacity) * sizeof(GpuConstraintPseudoVelocity) +
			static_cast<size_t>(m_stats.m_link_pseudo_capacity) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_stats.m_generalized_pseudo_capacity) * sizeof(float) +
			sizeof(GpuCoupledContactState) +
			sizeof(uint32_t);
	}

	// Run one warm-start, velocity, or position candidate through deterministic gather and detached ABA evaluation.
	void GpuCoupledContactSolver::RunTransaction(GpuJob& job, ComputeStep& build_step, int phase, int position_iteration_index)
	{
		if (
			m_r_counters == nullptr ||
			m_r_contacts == nullptr ||
			m_r_bodies == nullptr ||
			m_r_materials == nullptr ||
			m_impulse_aba.LinkImpulses() == nullptr)
			throw std::logic_error("Coupled contact transaction requires prepared frame resources");

		DispatchCommon(job, m_cs_begin, phase, m_work_count);
		switch (static_cast<ECoupledContactPhase>(phase))
		{
			case ECoupledContactPhase::Velocity:
			{
				// Component-local linear products remain detached; publication still uses the shared transaction below.
				DispatchNewton(job, build_step, CoupledContactDispatchGroups(m_max_contacts));
				break;
			}
			case ECoupledContactPhase::WarmStart:
			case ECoupledContactPhase::Position:
			{
				// Cache admission and pseudo motion retain their separate existing contracts.
				DispatchCommon(job, build_step, phase, m_max_contacts, position_iteration_index);
				break;
			}
			default:
			{
				throw std::invalid_argument("Unknown coupled contact transaction phase");
			}
		}
		DispatchCommon(job, m_cs_gather, phase, m_max_contacts);
		m_impulse_aba.Evaluate(job, m_r_tree_selection.get(), m_r_tree_results.get(), m_impulse_aba.VelocityBuffers());
		switch (static_cast<ECoupledContactPhase>(phase))
		{
			case ECoupledContactPhase::WarmStart:
			case ECoupledContactPhase::Velocity:
			{
				DispatchCommon(job, m_cs_validate_trees, phase, m_articulation_count);
				DispatchCommon(job, m_cs_select_trees, phase, m_articulation_count);
				DispatchCommon(job, m_cs_commit, phase, m_work_count);
				m_impulse_aba.Commit(job, m_r_tree_selection.get(), m_r_tree_results.get(), m_impulse_aba.VelocityBuffers());
				break;
			}
			case ECoupledContactPhase::Position:
			{
				DispatchPositionArticulations(job, m_cs_validate_position_trees, phase);
				DispatchCommon(job, m_cs_select_trees, phase, m_articulation_count);
				DispatchCommon(job, m_cs_commit, phase, m_work_count);
				DispatchPositionArticulations(job, m_cs_commit_position_articulations, phase);
				break;
			}
			default:
			{
				throw std::invalid_argument("Unknown coupled contact transaction phase");
			}
		}
	}

	// Bind and dispatch one non-empty work range with an optional zero-based position-sweep index.
	void GpuCoupledContactSolver::DispatchCommon(GpuJob& job, ComputeStep& step, int phase, int item_count, int position_iteration_index)
	{
		if (item_count <= 0)
			return;

		PrepareCommonResources(job);
		auto* rigid_pseudo = m_position_active && m_r_rigid_pseudo != nullptr ? m_r_rigid_pseudo.get() : m_r_uav_sentinel.get();
		auto* link_pseudo = m_position_active && m_r_link_pseudo != nullptr ? m_r_link_pseudo.get() : m_r_uav_sentinel.get();
		auto* generalized_pseudo = m_position_active && m_r_generalized_pseudo != nullptr ? m_r_generalized_pseudo.get() : m_r_uav_sentinel.get();
		auto const constants = GpuCoupledContactParams{
			.max_contacts = m_max_contacts,
			.body_count = m_body_count,
			.rigid_body_count = m_rigid_body_count,
			.articulation_count = m_articulation_count,
			.link_count = m_link_count,
			.target_count = m_target_count,
			.velocity_tolerance = m_config.contact_velocity_tolerance,
			.phase = phase,
			.velocity_delta_count = m_velocity_delta_count,
			.relaxation = m_config.constraint_coupled_relaxation,
			.restitution_scale = m_restitution_scale,
			.dt = m_dt,
			.position_slop = m_config.position_slop,
			.position_beta = m_config.position_baumgarte,
			.max_position_speed = m_config.constraint_max_position_speed,
			.position_iteration_index = position_iteration_index,
		};
		if (m_observation && (&step == &m_cs_prepare || &step == &m_cs_build_position_candidates))
			m_observation->Parameters(&step == &m_cs_prepare ? "prepared" : "position", constants);

		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_counters);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_materials);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_proxies.m_r_link_to_world);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_mobilities);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_scratch);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bodies);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contacts);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_blocks);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_scratch);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contributions);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_endpoint_keys);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_endpoint_order);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_target_impulses);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_participant_degrees);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.m_r_link_impulses);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_selection);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_results);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(rigid_pseudo);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(link_pseudo);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(generalized_pseudo);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations);

		job.m_cmd_list.Dispatch(CoupledContactDispatchGroups(item_count));
		++m_stats.m_dispatch_count;
		CommitUavBarriers(job);
	}

	// Bind component topology or a matrix-free Newton update without changing physical state during linear products.
	void GpuCoupledContactSolver::DispatchNewton(GpuJob& job, ComputeStep& step, iv3 groups)
	{
		// Register order matches coupled_contact_newton.hlsl. Empty joint ranges retain the existing ABI-compatible UAV sentinel.
		auto srvs = std::array{
			m_r_counters.get(), m_aba.m_r_links.get(), m_proxies.m_r_link_to_world.get(),
			m_mobility.m_r_mobilities.get(), m_mobility.m_r_ranges.get(), m_aba.m_r_articulations.get(),
			m_r_blocks.get(), m_r_contacts.get(), m_r_bodies.get(),
		};
		auto uavs = std::array{
			m_r_scratch.get(), m_r_contributions.get(), m_r_target_impulses.get(), m_impulse_aba.m_r_link_impulses.get(),
			m_r_state.get(), m_r_contact_nodes.get(), m_r_contact_order.get(), m_r_contact_krylov.get(), m_aba.m_r_scratch.get(),
			m_aba.m_r_dof_scratch != nullptr ? m_aba.m_r_dof_scratch.get() : m_r_uav_sentinel.get(),
			m_aba.m_r_joint_matrix_scratch != nullptr ? m_aba.m_r_joint_matrix_scratch.get() : m_r_uav_sentinel.get(),
			m_impulse_aba.m_r_work.get(),
			m_impulse_aba.m_r_velocity_deltas != nullptr ? m_impulse_aba.m_r_velocity_deltas.get() : m_r_uav_sentinel.get(),
		};
		for (auto* resource : srvs)
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

		for (auto* resource : uavs)
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

		job.m_barriers.Commit();
		auto const constants = GpuCoupledContactParams{
			.max_contacts = m_max_contacts,
			.body_count = m_body_count,
			.rigid_body_count = m_rigid_body_count,
			.articulation_count = m_articulation_count,
			.link_count = m_link_count,
			.target_count = m_target_count,
			.velocity_tolerance = m_config.contact_velocity_tolerance,
			.phase = static_cast<int>(ECoupledContactPhase::Velocity),
			.velocity_delta_count = m_velocity_delta_count,
			.relaxation = m_config.constraint_coupled_relaxation,
			.restitution_scale = m_restitution_scale,
			.dt = m_dt,
			.position_slop = m_config.position_slop,
			.position_beta = m_config.position_baumgarte,
			.max_position_speed = m_config.constraint_max_position_speed,
			.position_iteration_index = -1,
		};

		// Only one topology dispatch or one nonlinear update is submitted here; CGLS products stay within that update.
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		for (auto* resource : srvs)
			job.m_cmd_list.AddComputeRootShaderResourceView(resource);

		for (auto* resource : uavs)
			job.m_cmd_list.AddComputeRootUnorderedAccessView(resource);

		job.m_cmd_list.Dispatch(groups);
		++m_stats.m_dispatch_count;
		for (auto* resource : uavs)
			job.m_barriers.UAV(resource);

		job.m_barriers.Commit();
	}

	// Bind and dispatch one complete-tree position validation or pseudo-state commit phase.
	void GpuCoupledContactSolver::DispatchPositionArticulations(GpuJob& job, ComputeStep& step, int phase)
	{
		if (m_articulation_count <= 0)
			return;

		auto* velocity_deltas = m_impulse_aba.m_r_velocity_deltas != nullptr
			? m_impulse_aba.m_r_velocity_deltas.get()
			: m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.m_r_work.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(velocity_deltas, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_link_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_generalized_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = GpuCoupledContactParams{
			.max_contacts = m_max_contacts,
			.body_count = m_body_count,
			.rigid_body_count = m_rigid_body_count,
			.articulation_count = m_articulation_count,
			.link_count = m_link_count,
			.target_count = m_target_count,
			.velocity_tolerance = m_config.contact_velocity_tolerance,
			.phase = phase,
			.velocity_delta_count = m_velocity_delta_count,
			.relaxation = m_config.constraint_coupled_relaxation,
			.restitution_scale = m_restitution_scale,
			.dt = m_dt,
			.position_slop = m_config.position_slop,
			.position_beta = m_config.position_baumgarte,
			.max_position_speed = m_config.constraint_max_position_speed,
			.position_iteration_index = -1,
		};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_ranges);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_selection);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_results);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.m_r_work);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(velocity_deltas);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_link_pseudo);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_generalized_pseudo);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations);
		job.m_cmd_list.Dispatch(CoupledContactDispatchGroups(m_articulation_count));
		++m_stats.m_dispatch_count;

		job.m_barriers.UAV(m_r_tree_selection.get());
		job.m_barriers.UAV(m_r_tree_results.get());
		job.m_barriers.UAV(m_r_state.get());
		job.m_barriers.UAV(m_impulse_aba.m_r_work.get());
		job.m_barriers.UAV(velocity_deltas);
		job.m_barriers.UAV(m_r_link_pseudo.get());
		job.m_barriers.UAV(m_r_generalized_pseudo.get());
		job.m_barriers.UAV(m_aba.m_r_articulations.get());
		job.m_barriers.Commit();
	}

	// Bind and dispatch final rigid and articulation coordinate integration.
	void GpuCoupledContactSolver::DispatchApplyPosition(GpuJob& job)
	{
		auto* positions = m_aba.m_r_positions != nullptr
			? m_aba.m_r_positions.get()
			: m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_rigid_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_generalized_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(positions, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = GpuCoupledContactParams{
			.max_contacts = m_max_contacts,
			.body_count = m_body_count,
			.rigid_body_count = m_rigid_body_count,
			.articulation_count = m_articulation_count,
			.link_count = m_link_count,
			.target_count = m_target_count,
			.velocity_tolerance = m_config.contact_velocity_tolerance,
			.phase = static_cast<int>(ECoupledContactPhase::Position),
			.velocity_delta_count = m_velocity_delta_count,
			.relaxation = m_config.constraint_coupled_relaxation,
			.restitution_scale = m_restitution_scale,
			.dt = m_dt,
			.position_slop = m_config.position_slop,
			.position_beta = m_config.position_baumgarte,
			.max_position_speed = m_config.constraint_max_position_speed,
			.position_iteration_index = -1,
		};
		job.m_cmd_list.SetPipelineState(m_cs_apply_position.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_apply_position.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_ranges);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bodies);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_rigid_pseudo);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_generalized_pseudo);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(positions);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state);
		job.m_cmd_list.Dispatch(CoupledContactDispatchGroups(std::max(m_rigid_body_count, m_articulation_count)));
		++m_stats.m_dispatch_count;

		job.m_barriers.UAV(m_r_bodies.get());
		job.m_barriers.UAV(m_aba.m_r_articulations.get());
		job.m_barriers.UAV(positions);
		job.m_barriers.UAV(m_r_state.get());
		job.m_barriers.Commit();
	}

	// Transition the common contact and articulation resources into the states required by contact shaders.
	void GpuCoupledContactSolver::PrepareCommonResources(GpuJob& job)
	{
		auto* rigid_pseudo = m_position_active && m_r_rigid_pseudo != nullptr ? m_r_rigid_pseudo.get() : m_r_uav_sentinel.get();
		auto* link_pseudo = m_position_active && m_r_link_pseudo != nullptr ? m_r_link_pseudo.get() : m_r_uav_sentinel.get();
		auto* generalized_pseudo = m_position_active && m_r_generalized_pseudo != nullptr ? m_r_generalized_pseudo.get() : m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_proxies.m_r_link_to_world.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_mobility.m_r_mobilities.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_contributions.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_endpoint_keys.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_endpoint_order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_target_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_participant_degrees.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.m_r_link_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(rigid_pseudo, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(link_pseudo, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(generalized_pseudo, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}

	// Order every mutable resource before the next dependent contact or ABA phase.
	void GpuCoupledContactSolver::CommitUavBarriers(GpuJob& job)
	{
		auto* rigid_pseudo = m_position_active && m_r_rigid_pseudo != nullptr ? m_r_rigid_pseudo.get() : m_r_uav_sentinel.get();
		auto* link_pseudo = m_position_active && m_r_link_pseudo != nullptr ? m_r_link_pseudo.get() : m_r_uav_sentinel.get();
		auto* generalized_pseudo = m_position_active && m_r_generalized_pseudo != nullptr ? m_r_generalized_pseudo.get() : m_r_uav_sentinel.get();
		job.m_barriers.UAV(m_r_bodies.get());
		job.m_barriers.UAV(m_r_contacts.get());
		job.m_barriers.UAV(m_r_blocks.get());
		job.m_barriers.UAV(m_r_scratch.get());
		job.m_barriers.UAV(m_r_contributions.get());
		job.m_barriers.UAV(m_r_endpoint_keys.get());
		job.m_barriers.UAV(m_r_endpoint_order.get());
		job.m_barriers.UAV(m_r_target_impulses.get());
		job.m_barriers.UAV(m_r_participant_degrees.get());
		job.m_barriers.UAV(m_impulse_aba.m_r_link_impulses.get());
		job.m_barriers.UAV(m_r_tree_selection.get());
		job.m_barriers.UAV(m_r_tree_results.get());
		job.m_barriers.UAV(m_r_state.get());
		job.m_barriers.UAV(rigid_pseudo);
		job.m_barriers.UAV(link_pseudo);
		job.m_barriers.UAV(generalized_pseudo);
		job.m_barriers.Commit();
	}

	// Destroy lazily owned contact resources where the implementation type is complete.
	void Deleter<GpuCoupledContactSolver>::operator()(GpuCoupledContactSolver* solver) const
	{
		delete solver;
	}
}
