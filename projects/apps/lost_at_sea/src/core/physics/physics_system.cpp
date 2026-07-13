//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/core/physics/physics_system.h"

namespace las
{
	namespace
	{
		static constexpr bool EnableOceanSurfaceForce = false;
		static constexpr int OceanSurfaceForceThreadCount = 64;

		struct CBufOceanSurfaceForce
		{
			float m_water_level;
			float m_spring_constant;
			float m_damping_constant;
			float m_max_force;
			int m_body_count;
			float m_pad[3];
		};
		static_assert(sizeof(CBufOceanSurfaceForce) % sizeof(uint32_t) == 0);

		// Throw if the box body descriptor would create an unusable physics body.
		void ValidateBoxBodyDesc(PhysicsSystem::BoxBodyDesc const& desc)
		{
			if (desc.m_size.x <= 0.0f || desc.m_size.y <= 0.0f || desc.m_size.z <= 0.0f || desc.m_size.w != 0.0f)
			{
				throw std::runtime_error("Box physics bodies require positive xyz dimensions and w = 0");
			}
			if (desc.m_mass_kg <= 0.0f)
			{
				throw std::runtime_error("Box physics bodies require a positive mass");
			}
		}

		// Create the compute step that applies the placeholder ocean contact force.
		::pr::compute::ComputeStep CreateOceanSurfaceForceStep(ID3D12Device* device)
		{
			auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};
			auto bytecode = ShaderCompiler{}
				.Source("src/core/physics/ocean_surface_force.hlsl", resolver)
				.HlslVersion(EHlslVersion::Hlsl2021)
				.Define(L"SHADER_BUILD")
				.Optimise(true)
				.ShaderModel(L"cs_6_6")
				.EntryPoint(L"CSOceanSurfaceForce")
				.Compile();

			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufOceanSurfaceForce>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.Create(device, "LAS.OceanSurfaceForce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), bytecode).Create(device, "LAS.OceanSurfaceForce.PSO");
			return step;
		}
	}

	struct PhysicsSystem::OceanSurfaceForce
	{
		::pr::compute::ComputeStep m_step;
		multicast::AutoSub m_external_force_sub;

		// Construct and subscribe the placeholder ocean contact force pass.
		OceanSurfaceForce(ID3D12Device* device, physics::Engine& engine)
			:m_step(CreateOceanSurfaceForceStep(device))
			,m_external_force_sub(engine.ExternalForces += [this](physics::Engine& sender, physics::Engine::ExternalForceArgs const& args)
			{
				Apply(sender, args);
			})
		{
		}

		// Apply a flat-water spring/damper contact force to bodies that intersect the ocean surface.
		void Apply(physics::Engine&, physics::Engine::ExternalForceArgs const& args)
		{
			if (args.m_body_count == 0)
			{
				return;
			}

			auto cb = CBufOceanSurfaceForce{
				.m_water_level = 0.0f,
				.m_spring_constant = 12000.0f,
				.m_damping_constant = 1800.0f,
				.m_max_force = 25000.0f,
				.m_body_count = args.m_body_count,
				.m_pad = {},
			};

			// This pass deliberately writes only the force accumulator so the physics engine remains owner of all integration state.
			args.m_job.m_cmd_list.SetPipelineState(m_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies->GetGPUVirtualAddress());
			args.m_job.m_cmd_list.Dispatch(::pr::compute::DispatchCount({ args.m_body_count, 1, 1 }, { OceanSurfaceForceThreadCount, 1, 1 }));
			args.m_job.m_barriers.UAV(args.m_bodies).Commit();
		}
	};

	// Return an invalid body handle sentinel.
	PhysicsSystem::BodyHandle PhysicsSystem::BodyHandle::Invalid()
	{
		return BodyHandle{ -1, -1 };
	}

	// Return true if this handle could refer to a registered body.
	bool PhysicsSystem::BodyHandle::IsValid() const
	{
		return m_index >= 0 && m_generation >= 0;
	}

	// Construct an invalid body snapshot.
	PhysicsSystem::BodySnapshot::BodySnapshot()
		:m_handle(BodyHandle::Invalid())
		,m_o2w(m4x4::Identity())
		,m_step_index(-1)
		,m_valid()
	{}

	// Create an owned buoyancy hull registration.
	PhysicsSystem::BuoyancyHullRegistration::BuoyancyHullRegistration(PhysicsSystem& physics, BodyHandle handle)
		:m_physics(&physics)
		,m_handle(handle)
	{
	}

	// Construct an empty buoyancy hull registration.
	PhysicsSystem::BuoyancyHullRegistration::BuoyancyHullRegistration()
		:m_physics()
		,m_handle(BodyHandle::Invalid())
	{
	}

	// Move a buoyancy hull registration without releasing it.
	PhysicsSystem::BuoyancyHullRegistration::BuoyancyHullRegistration(BuoyancyHullRegistration&& rhs) noexcept
		:m_physics(std::exchange(rhs.m_physics, nullptr))
		,m_handle(std::exchange(rhs.m_handle, BodyHandle::Invalid()))
	{
	}

	// Release the current registration and then take ownership of another one.
	PhysicsSystem::BuoyancyHullRegistration& PhysicsSystem::BuoyancyHullRegistration::operator=(BuoyancyHullRegistration&& rhs) noexcept
	{
		if (this != &rhs)
		{
			Reset();
			m_physics = std::exchange(rhs.m_physics, nullptr);
			m_handle = std::exchange(rhs.m_handle, BodyHandle::Invalid());
		}
		return *this;
	}

	// Release the buoyancy hull registration when ownership leaves scope.
	PhysicsSystem::BuoyancyHullRegistration::~BuoyancyHullRegistration()
	{
		Reset();
	}

	// Release the buoyancy hull registration if one is owned.
	void PhysicsSystem::BuoyancyHullRegistration::Reset() noexcept
	{
		auto* physics = std::exchange(m_physics, nullptr);
		auto handle = std::exchange(m_handle, BodyHandle::Invalid());
		if (physics != nullptr)
		{
			physics->ReleaseBuoyancyHull(handle);
		}
	}

	// Return true if this object owns a buoyancy hull registration.
	bool PhysicsSystem::BuoyancyHullRegistration::IsValid() const
	{
		return m_physics != nullptr && m_handle.IsValid();
	}

	// Construct the LAS physics facade on the simulation owner thread.
	PhysicsSystem::PhysicsSystem(Renderer& rdr)
		:m_owner_thread_id(std::this_thread::get_id())
		,m_engine(physics::EngineConfig{}, nullptr, rdr.D3DDevice())
		,m_ocean_surface_force(EnableOceanSurfaceForce ? std::make_unique<OceanSurfaceForce>(rdr.D3DDevice(), m_engine) : nullptr)
		,m_gpu_buoyancy(std::make_unique<physics::GpuBuoyancy>(rdr.D3DDevice(), m_engine,
			physics::GpuBuoyancy::Config{},
			[this](int body_slot_index) { return BodySlotStepIndex(body_slot_index); },
			[this](int body_slot_index) { return BodySlotState(body_slot_index); })
		)
		,m_body_slots()
		,m_free_slots()
		,m_step_bodies()
		,m_snapshot_mutex()
		,m_snapshots()
		,m_config_mutex()
		,m_gravity_ws(v4{0, 0, -9.81f, 0})
		,m_step_pending()
		,m_engine_step_pending()
	{
		// Renderer does not expose a shared compute shader cache yet. Passing nullptr keeps the physics engine on its normal uncached compiler
		// path while still sharing the renderer's D3D device.
		PublishSnapshots();
	}

	// Destroy the LAS physics facade after all in-flight work has been completed by the owner.
	PhysicsSystem::~PhysicsSystem()
	{
		PR_ASSERT(PR_DBG, !m_step_pending, "PhysicsSystem destroyed with a pending physics step");
	}

	// Create a physics-owned box body and publish its initial snapshot.
	PhysicsSystem::BodyHandle PhysicsSystem::CreateBoxBody(BoxBodyDesc const& desc)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::CreateBoxBody");
		ValidateBoxBodyDesc(desc);

		auto shape = std::make_unique<ShapeBox>(desc.m_size);
		auto inertia = Inertia::Box(desc.m_size * 0.5f, desc.m_mass_kg);
		auto body = std::make_unique<RigidBody>(shape.get(), desc.m_o2w, inertia);
		body->NeverSleep(desc.m_never_sleep);

		if (!m_free_slots.empty())
		{
			auto const slot_index = m_free_slots.back();
			m_free_slots.pop_back();

			auto& slot = m_body_slots[slot_index];
			slot.m_buoyancy_hull.Reset();
			slot.m_shape = std::move(shape);
			slot.m_body = std::move(body);
			slot.m_step_index = -1;
			auto handle = BodyHandle{ slot_index, slot.m_generation };
			PublishSnapshots();
			return handle;
		}

		auto const slot_index = static_cast<int>(m_body_slots.size());
		m_body_slots.push_back(BodySlot{
			.m_shape = std::move(shape),
			.m_body = std::move(body),
			.m_generation = 0,
			.m_step_index = -1,
			.m_buoyancy_hull = {},
		});
		auto handle = BodyHandle{ slot_index, 0 };
		PublishSnapshots();
		return handle;
	}

	// Destroy a physics-owned body and invalidate its published snapshot.
	void PhysicsSystem::DestroyBody(BodyHandle handle)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::DestroyBody");
		if (!handle.IsValid())
		{
			return;
		}

		auto& slot = Slot(handle);
		ReleaseBuoyancyHull(handle);
		slot.m_body.reset();
		slot.m_shape.reset();
		slot.m_step_index = -1;
		++slot.m_generation;
		m_free_slots.push_back(handle.m_index);
		PublishSnapshots();
	}

	// Return true if 'handle' currently refers to a registered body.
	bool PhysicsSystem::IsValid(BodyHandle handle) const
	{
		if (!handle.IsValid())
		{
			return false;
		}

		auto lock = std::lock_guard<std::mutex>(m_snapshot_mutex);
		return handle.m_index < static_cast<int>(m_snapshots.size()) &&
			m_snapshots[handle.m_index].m_valid &&
			m_snapshots[handle.m_index].m_handle.m_generation == handle.m_generation;
	}

	// Return the compact body index used by the most recent/current Engine::Step() call.
	int PhysicsSystem::StepIndex(BodyHandle handle) const
	{
		if (std::this_thread::get_id() == m_owner_thread_id)
		{
			return Slot(handle).m_step_index;
		}

		return Snapshot(handle).m_step_index;
	}

	// Return the latest published body transform and step metadata.
	PhysicsSystem::BodySnapshot PhysicsSystem::Snapshot(BodyHandle handle) const
	{
		if (!handle.IsValid())
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		auto lock = std::lock_guard<std::mutex>(m_snapshot_mutex);
		if (handle.m_index >= static_cast<int>(m_snapshots.size()))
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		auto const& snapshot = m_snapshots[handle.m_index];
		if (!snapshot.m_valid || snapshot.m_handle.m_generation != handle.m_generation)
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		return snapshot;
	}

	// Register a physics body's collision shape for buoyancy.
	PhysicsSystem::BuoyancyHullRegistration PhysicsSystem::RegisterBuoyancyHull(BodyHandle handle)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::RegisterBuoyancyHull");

		auto& slot = Slot(handle);
		if (slot.m_buoyancy_hull)
		{
			throw std::runtime_error("A buoyancy hull is already registered for this body");
		}

		slot.m_buoyancy_hull = m_gpu_buoyancy->RegisterCompositeHull(*slot.m_body, handle.m_index, handle.m_generation);
		return BuoyancyHullRegistration{ *this, handle };
	}

	// Release a buoyancy hull registration during RAII cleanup.
	void PhysicsSystem::ReleaseBuoyancyHull(BodyHandle handle) noexcept
	{
		if (!handle.IsValid())
		{
			return;
		}

		if (std::this_thread::get_id() != m_owner_thread_id)
		{
			PR_ASSERT(PR_DBG, false, "PhysicsSystem buoyancy hull registration released from the wrong thread");
			std::terminate();
		}
		if (m_step_pending)
		{
			PR_ASSERT(PR_DBG, false, "PhysicsSystem buoyancy hull registration released while a physics step is pending");
			std::terminate();
		}

		if (handle.m_index < 0 || handle.m_index >= static_cast<int>(m_body_slots.size()))
		{
			return;
		}

		auto& slot = m_body_slots[handle.m_index];
		if (slot.m_generation != handle.m_generation)
		{
			return;
		}

		slot.m_buoyancy_hull.Reset();
	}

	// Return the latest diagnostic buoyancy result for a physics body.
	physics::GpuBuoyancy::Diagnostics PhysicsSystem::BuoyancyDiagnostics(BodyHandle handle) const
	{
		if (!IsValid(handle))
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		return m_gpu_buoyancy->LatestDiagnostics(handle.m_index, handle.m_generation);
	}

	// Submit GPU physics work for all registered rigid bodies without waiting for completion.
	void PhysicsSystem::BeginStep(float dt, double time_s)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::BeginStep");

		m_step_pending = true;
		m_engine_step_pending = false;

		try
		{
			BuildStepBodyList();
			if (!m_step_bodies.empty())
			{
				m_engine.BeginStep(dt, std::span{ m_step_bodies }, time_s);
				m_engine_step_pending = true;
			}
		}
		catch (...)
		{
			m_step_pending = false;
			m_engine_step_pending = false;
			throw;
		}
	}

	// Wait for pending GPU physics work and publish immutable snapshots, returning an exception on failure.
	[[nodiscard]] std::exception_ptr PhysicsSystem::CompleteStep() noexcept
	{
		try
		{
			CheckOwnerThread();
			if (!m_step_pending)
			{
				throw std::runtime_error("PhysicsSystem::CompleteStep called without a pending step");
			}

			if (m_engine_step_pending)
			{
				m_engine.CompleteStep();
				m_engine_step_pending = false;
				m_gpu_buoyancy->CompleteStep();
			}

			PublishSnapshots();
			m_step_pending = false;
			return {};
		}
		catch (...)
		{
			return std::current_exception();
		}
	}

	// Step all registered rigid bodies synchronously.
	void PhysicsSystem::Step(float dt, double time_s)
	{
		BeginStep(dt, time_s);
		if (auto ex = CompleteStep())
		{
			std::rethrow_exception(ex);
		}
	}

	// Set the world-space gravity applied to registered bodies before each step.
	void PhysicsSystem::GravityWS(v4 gravity_ws)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::GravityWS");

		auto lock = std::lock_guard<std::mutex>(m_config_mutex);
		m_gravity_ws = gravity_ws;
	}

	// Return the world-space gravity applied before each step.
	v4 PhysicsSystem::GravityWS() const
	{
		auto lock = std::lock_guard<std::mutex>(m_config_mutex);
		return m_gravity_ws;
	}

	// Throw if mutable physics state is touched from a thread other than the owner thread.
	void PhysicsSystem::CheckOwnerThread() const
	{
		if (std::this_thread::get_id() != m_owner_thread_id)
		{
			throw std::runtime_error("PhysicsSystem mutable state accessed from the wrong thread");
		}
	}

	// Throw if an operation would mutate state while a step is in flight.
	void PhysicsSystem::CheckNoStepPending(char const* operation) const
	{
		if (m_step_pending)
		{
			throw std::runtime_error(std::string(operation) + " cannot run while a physics step is pending");
		}
	}

	// Return true if 'handle' refers to a live body slot on the owner thread.
	bool PhysicsSystem::IsValidOnOwnerThread(BodyHandle handle) const
	{
		CheckOwnerThread();
		return handle.IsValid() &&
			handle.m_index < static_cast<int>(m_body_slots.size()) &&
			m_body_slots[handle.m_index].m_generation == handle.m_generation &&
			m_body_slots[handle.m_index].m_body != nullptr;
	}

	// Return the live body slot for 'handle' on the owner thread.
	PhysicsSystem::BodySlot& PhysicsSystem::Slot(BodyHandle handle)
	{
		CheckOwnerThread();
		if (!IsValidOnOwnerThread(handle))
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		return m_body_slots[handle.m_index];
	}

	// Return the live body slot for 'handle' on the owner thread.
	PhysicsSystem::BodySlot const& PhysicsSystem::Slot(BodyHandle handle) const
	{
		CheckOwnerThread();
		if (!IsValidOnOwnerThread(handle))
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		return m_body_slots[handle.m_index];
	}

	// Return the compact body index used by the current physics step for a body slot.
	int PhysicsSystem::BodySlotStepIndex(int body_slot_index) const
	{
		CheckOwnerThread();
		if (body_slot_index < 0 || body_slot_index >= static_cast<int>(m_body_slots.size()))
		{
			return -1;
		}

		auto const& slot = m_body_slots[body_slot_index];
		return slot.m_body != nullptr ? slot.m_step_index : -1;
	}

	// Return the live body state used by owner-thread diagnostic systems.
	physics::GpuBuoyancy::BodyState PhysicsSystem::BodySlotState(int body_slot_index) const
	{
		CheckOwnerThread();
		if (body_slot_index < 0 || body_slot_index >= static_cast<int>(m_body_slots.size()))
			return physics::GpuBuoyancy::BodyState{};

		auto const& slot = m_body_slots[body_slot_index];
		if (slot.m_body == nullptr)
			return physics::GpuBuoyancy::BodyState{};

		auto state = physics::GpuBuoyancy::BodyState{};
		state.m_o2w = slot.m_body->O2W();
		state.m_centre_of_mass_os = slot.m_body->CentreOfMassOS();
		state.m_ws_gravity = slot.m_body->GravityWS();
		state.m_valid = true;
		return state;
	}

	// Rebuild the compact body list submitted to the physics engine.
	void PhysicsSystem::BuildStepBodyList()
	{
		CheckOwnerThread();

		auto const gravity_ws = GravityWS();
		m_step_bodies.clear();
		for (auto& slot : m_body_slots)
		{
			slot.m_step_index = -1;
			if (slot.m_body == nullptr)
			{
				continue;
			}

			slot.m_body->GravityWS(gravity_ws);
			slot.m_step_index = static_cast<int>(m_step_bodies.size());
			m_step_bodies.push_back(slot.m_body.get());
		}
	}

	// Publish body state for render and camera readers.
	void PhysicsSystem::PublishSnapshots()
	{
		CheckOwnerThread();

		auto lock = std::lock_guard<std::mutex>(m_snapshot_mutex);
		m_snapshots.resize(m_body_slots.size());
		for (int slot_index = 0; slot_index != static_cast<int>(m_body_slots.size()); ++slot_index)
		{
			auto const& slot = m_body_slots[slot_index];
			auto& snapshot = m_snapshots[slot_index];
			snapshot.m_handle = BodyHandle{ slot_index, slot.m_generation };
			snapshot.m_step_index = slot.m_step_index;
			snapshot.m_valid = slot.m_body != nullptr;
			snapshot.m_o2w = snapshot.m_valid ? slot.m_body->O2W() : m4x4::Identity();
		}
	}
}
