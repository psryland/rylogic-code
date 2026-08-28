//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics/physics.h"
#include "pr/physics/physics-dll.h"
#include "pr/collision/shape_array.h"
#include "pr/collision/shape_polytope.h"
#include "pr/container/byte_data.h"
#include "physics/src/compute/physics_types.h"
#include "physics/src/dll/context.h"
#include "physics/src/dll/interop.h"
#include "physics/src/utility/gpu.h"

namespace
{
	// Keep the implementation's wire-record vocabulary local while the public API uses concise names in pr::physics.
	using PhysicsEngineHandle = pr::physics::EngineHandle;
	using PhysicsShapeHandle = pr::physics::ShapeHandle;
	using PhysicsBodyHandle = pr::physics::BodyHandle;
	using PhysicsArticulationHandle = pr::physics::ArticulationHandle;
	using PhysicsConstraintHandle = pr::physics::PersistentConstraintHandle;
	using PhysicsStatus = pr::physics::EStatus;
	using PhysicsStructId = pr::physics::EStructId;
	using PhysicsMotionType = pr::physics::EMotionType;
	using PhysicsMassMode = pr::physics::EMassMode;
	using PhysicsBodyCommandType = pr::physics::ECommand;
	using PhysicsEventType = pr::physics::EEvent;
	using PhysicsBodyFlags = pr::physics::EBodyFlags;
	using PhysicsArticulationRoot = pr::physics::EArticulationRoot;
	using PhysicsArticulationAxis = pr::physics::EArticulationAxis;
	using PhysicsArticulationFlags = pr::physics::EArticulationFlags;
	using PhysicsArticulationLinkFlags = pr::physics::EArticulationLinkFlags;
	using PhysicsConstraintEndpoint = pr::physics::EConstraintEndpoint;
	using PhysicsConstraintMode = pr::physics::EConstraintMode;
	using PhysicsConstraintFlags = pr::physics::EConstraintFlags;
	using PhysicsStructHeader = pr::physics::StructHeader;
	using PhysicsVector4 = pr::physics::Vector4;
	using PhysicsMatrix4 = pr::physics::Matrix4;
	using PhysicsSpatialVector = pr::physics::SpatialVector;
	using PhysicsInertia = pr::physics::InertiaProperties;
	using PhysicsEngineConfig = pr::physics::Config;
	using PhysicsMaterial = pr::physics::MaterialProperties;
	using PhysicsShapeCommon = pr::physics::ShapeCommon;
	using PhysicsSphereShape = pr::physics::SphereShape;
	using PhysicsBoxShape = pr::physics::BoxShape;
	using PhysicsLineShape = pr::physics::LineShape;
	using PhysicsTriangleShape = pr::physics::TriangleShape;
	using PhysicsBodyDesc = pr::physics::BodyDesc;
	using PhysicsBodyState = pr::physics::BodyState;
	using PhysicsBodyCommand = pr::physics::BodyCommand;
	using PhysicsBodySnapshot = pr::physics::BodySnapshot;
	using PhysicsArticulationDesc = pr::physics::ArticulationDesc;
	using PhysicsArticulationLink = pr::physics::ArticulationLinkProperties;
	using PhysicsArticulationJoint = pr::physics::ArticulationJointProperties;
	using PhysicsArticulationState = pr::physics::ArticulationState;
	using PhysicsArticulationLinkState = pr::physics::ArticulationLinkState;
	using PhysicsConstraintFrame = pr::physics::ConstraintFrameProperties;
	using PhysicsConstraintAxis = pr::physics::ConstraintAxisProperties;
	using PhysicsD6Constraint = pr::physics::D6ConstraintProperties;
	using PhysicsEvent = pr::physics::Event;
	using PhysicsStepProfile = pr::physics::StepProfile;
	using PhysicsFeatureResourceDiagnostics = pr::physics::FeatureResourceDiagnostics;
	using PhysicsConstraintFeatureDiagnostics = pr::physics::ConstraintFeatureDiagnostics;
	using PhysicsArticulationFeatureDiagnostics = pr::physics::ArticulationFeatureDiagnostics;
	using PhysicsCoupledFeatureDiagnostics = pr::physics::CoupledFeatureDiagnostics;
	using PhysicsFrameOutputFeatureDiagnostics = pr::physics::FrameOutputFeatureDiagnostics;
	using PhysicsStepFailureDiagnostics = pr::physics::StepFailureDiagnostics;
	using PhysicsDiagnostics = pr::physics::Diagnostics;
}

namespace pr::physics
{
	thread_local std::string g_last_error;

		// Converts expected ABI failures into status codes without allowing exceptions across the C boundary.
		struct ApiException : std::runtime_error
		{
			PhysicsStatus m_status;

			ApiException(PhysicsStatus status, std::string message)
				: std::runtime_error(std::move(message))
				, m_status(status)
			{}
		};

		// A generation slot retains its generation after object destruction so stale handles remain detectable.
		template <typename T>
		struct ObjectSlot
		{
			std::uint32_t m_generation = 1;
			std::unique_ptr<T> m_object;
		};

		struct ShapeRecord
		{
			byte_data<16> m_data;

			// Child handles retained by a compound shape, in declaration order. A compound owns a private copy
			// of each child blob, but retaining the handles keeps the caller's child identities alive and
			// meaningful for as long as the compound that was built from them exists.
			std::vector<PhysicsShapeHandle> m_children;

			std::uint32_t m_body_refs = 0;
			std::uint32_t m_compound_refs = 0;
			std::uint32_t m_articulation_refs = 0;

			collision::Shape* Shape()
			{
				return m_data.begin<collision::Shape>();
			}
			collision::Shape const* Shape() const
			{
				return m_data.begin<collision::Shape>();
			}
		};

		struct BodyRecord
		{
			std::unique_ptr<RigidBody> m_body;
			PhysicsShapeHandle m_shape = {};
			std::uint64_t m_user_tag = {};
			PhysicsMotionType m_motion_type = PhysicsMotionType::Dynamic;
			bool m_enabled = true;
			bool m_sleep_before_step = false;
			std::uint32_t m_constraint_refs = 0;
		};

		// Owns one immutable articulation topology and the shape handles retained by its physical links.
		struct ArticulationRecord
		{
			std::unique_ptr<Articulation> m_articulation;
			std::vector<PhysicsShapeHandle> m_shapes;
			std::uint64_t m_user_tag = {};
			bool m_enabled = true;
			std::uint32_t m_constraint_refs = 0;
		};

		// Couples one ABI generation slot to the native persistent slot and its endpoint lifetime references.
		struct ConstraintRecord
		{
			ConstraintHandle m_constraint = {};
			PhysicsD6Constraint m_desc = {};
			std::array<PhysicsBodyHandle, 2> m_body_refs = {};
			std::array<PhysicsArticulationHandle, 2> m_articulation_refs = {};
		};

		// Owns one compute device and queue for all ABI engines created against the same caller-supplied device identity.
		struct InteropGpuBackend
		{
			void* m_external_device;
			ComGpu m_gpu;

			explicit InteropGpuBackend(void* external_device)
				: m_external_device(external_device)
				, m_gpu(static_cast<ID3D12Device4*>(external_device))
			{}
		};

		// Translate an engine-side contact child index into the ABI's child identity.
		// The engine indexes convex leaves from zero for every body, so a body with a primitive root would
		// otherwise be indistinguishable from the first child of a compound.
		std::uint32_t ContactChildId(RigidBody const* body, int child_id)
		{
			if (body == nullptr || body->Shape().m_type != collision::EShape::Array || child_id < 0)
				return PHYSICS_NO_CHILD;

			return static_cast<std::uint32_t>(child_id);
		}

		// Pack a native persistent handle into one collision-free lookup key.
		std::uint64_t NativeConstraintKey(ConstraintHandle handle)
		{
			return (std::uint64_t{handle.m_generation} << 32) | handle.m_index;
		}

		struct EngineRecord
		{
			// Guards this engine's state. It is separate from the process-wide registry lock so that a long
			// operation on one engine, notably the GPU wait inside CompleteStep, does not block calls made
			// against unrelated engines on their own simulation threads. Recursive because ABI helpers call
			// each other while already holding it.
			std::recursive_mutex m_lock;

			// Set under m_lock when the engine is destroyed or abandoned. A caller that pinned this record
			// just before destruction keeps the memory alive, so this flag is what makes its handle
			// observably stale rather than silently operating on a destroyed engine.
			bool m_retired;

			std::uint16_t m_cookie;
			DWORD m_owner_thread_id;
			std::vector<ObjectSlot<ShapeRecord>> m_shapes;
			std::vector<ObjectSlot<BodyRecord>> m_bodies;
			std::vector<ObjectSlot<ArticulationRecord>> m_articulations;
			std::vector<ObjectSlot<ConstraintRecord>> m_constraints;
			std::unordered_map<RigidBody const*, PhysicsBodyHandle> m_body_handles;
			std::unordered_map<std::uint64_t, PhysicsConstraintHandle> m_constraint_handles;
			ConstraintSet m_constraint_set;
			std::vector<RigidBody*> m_step_bodies;
			std::vector<Articulation*> m_step_articulations;
			std::vector<PhysicsEvent> m_events;
			std::uint64_t m_submitted_step;
			std::uint64_t m_completed_step;
			std::unique_ptr<Engine> m_engine;

			EngineRecord(std::uint16_t cookie, EngineConfig const& config, ID3D12Device4* device, ID3D12CommandQueue* queue)
				: m_lock()
				, m_retired(false)
				, m_cookie(cookie)
				, m_owner_thread_id(GetCurrentThreadId())
				, m_shapes()
				, m_bodies()
				, m_articulations()
				, m_constraints()
				, m_body_handles()
				, m_constraint_handles()
				, m_constraint_set()
				, m_step_bodies()
				, m_step_articulations()
				, m_events()
				, m_submitted_step()
				, m_completed_step()
				, m_engine(new Engine(config, nullptr, device, queue))
			{
				BindEvents(*m_engine);
			}

			// Bind native callbacks to ABI-owned maps and buffered events before publishing an engine instance.
			void BindEvents(Engine& engine)
			{
				// Contacts are buffered while CompleteStep unpacks native results; no managed callback occurs from stepping.
				engine.Collisions += [this](Engine&, std::span<RbContact const> contacts)
				{
					for (auto const& contact : contacts)
					{
						auto iter_a = m_body_handles.find(contact.m_objA);
						auto iter_b = m_body_handles.find(contact.m_objB);
						if (iter_a == m_body_handles.end() || iter_b == m_body_handles.end())
							continue;

						auto evt = PhysicsEvent{
							.header = {sizeof(PhysicsEvent), PHYSICS_STRUCT_VERSION},
							.type = PhysicsEventType::Contact,
							.point_count = static_cast<std::uint32_t>(contact.Count()),
							.body_a = iter_a->second,
							.body_b = iter_b->second,
							.normal = {},
							.points = {},
							.depth = contact.m_depth,
							.material_a = contact.m_mat_idA,
							.material_b = contact.m_mat_idB,
							.child_a = ContactChildId(contact.m_objA, contact.m_child_idA),
							.child_b = ContactChildId(contact.m_objB, contact.m_child_idB),
							.substep_index = contact.m_substep_index,
						};

						// Contact geometry is reported in world space so snapshots and events share one coordinate frame.
						auto const& a2w = contact.m_objA->O2W();
						auto normal = (a2w.rot * contact.m_axis).w0();
						memcpy(&evt.normal, &normal, sizeof(normal));
						for (auto i = 0; i != contact.Count(); ++i)
						{
							auto point = a2w * contact.m_manifold[i];
							memcpy(&evt.points[i], &point, sizeof(point));
						}
						m_events.push_back(evt);
					}
				};

				// Constraint overloads share the buffered event stream and expose only ABI-owned handles.
				engine.ConstraintsBroken += [this](Engine&, std::span<ConstraintBreakEvent const> events)
				{
					for (auto const& event : events)
					{
						auto iter = m_constraint_handles.find(NativeConstraintKey(event.m_constraint));
						if (iter == m_constraint_handles.end())
							continue;

						m_events.push_back(PhysicsEvent{
							.header = {sizeof(PhysicsEvent), PHYSICS_STRUCT_VERSION},
							.type = PhysicsEventType::ConstraintBreak,
							.point_count = 0,
							.body_a = {},
							.body_b = {},
							.normal = {},
							.points = {},
							.depth = 0.0f,
							.material_a = 0,
							.material_b = 0,
							.child_a = PHYSICS_NO_CHILD,
							.child_b = PHYSICS_NO_CHILD,
							.constraint = iter->second,
							.break_force = event.m_force,
							.break_torque = event.m_torque,
							.substep_index = event.m_substep_index,
							.reserved = 0,
						});
					}
				};

				// Bounded coupled-solver rejections are post-publication events and require no caller-owned object handles.
				engine.CoupledConstraintFailures += [this](Engine&, std::span<CoupledConstraintFailureEvent const> events)
				{
					for (auto const& event : events)
					{
						m_events.push_back(PhysicsEvent{
							.header = {sizeof(PhysicsEvent), PHYSICS_STRUCT_VERSION},
							.type = PhysicsEventType::CoupledConstraintFailure,
							.point_count = 0,
							.body_a = {},
							.body_b = {},
							.normal = {},
							.points = {},
							.depth = 0.0f,
							.material_a = 0,
							.material_b = 0,
							.child_a = PHYSICS_NO_CHILD,
							.child_b = PHYSICS_NO_CHILD,
							.constraint = {},
							.break_force = 0.0f,
							.break_torque = 0.0f,
							.substep_index = event.m_substep_index,
							.reserved = 0,
							.failure_flags = event.m_failure_flags,
							.failure_phase = event.m_phase,
							.failure_island_index = event.m_island_index,
							.failure_iteration_count = event.m_iteration_count,
							.failure_relaxation = event.m_relaxation,
							.failure_merit_change = event.m_merit_change,
						});
					}
				};
			}
		};

		struct EngineSlot
		{
			std::uint32_t m_generation = 1;

			// Shared rather than unique so a call in progress can pin the record and let the process-wide
			// registry lock go while it works.
			std::shared_ptr<EngineRecord> m_record;
		};

		// Engine slots use a process-level generation because engine handles are not scoped by another object.
		std::uint64_t MakeEngineHandle(std::size_t index, std::uint32_t generation)
		{
			return (std::uint64_t{generation} << 32) | (std::uint64_t{index} + 1);
		}
		std::size_t EngineIndex(PhysicsEngineHandle handle)
		{
			auto index = static_cast<std::uint32_t>(handle);
			if (index == 0)
				throw ApiException(PhysicsStatus::InvalidHandle, "Engine handle is null");

			return static_cast<std::size_t>(index - 1);
		}
		std::uint32_t EngineGeneration(PhysicsEngineHandle handle)
		{
			return static_cast<std::uint32_t>(handle >> 32);
		}

		// Child handles encode an engine cookie as well as slot generation to reject cross-engine use.
		std::uint64_t MakeChildHandle(std::uint16_t cookie, std::size_t index, std::uint32_t generation)
		{
			if (index >= 0xFFFFFFU)
				throw ApiException(PhysicsStatus::InternalError, "Physics handle capacity exhausted");

			return
				(std::uint64_t{cookie} << 48) |
				(std::uint64_t{generation & 0xFFFFFFU} << 24) |
				(std::uint64_t{index} + 1);
		}
		std::uint16_t ChildCookie(std::uint64_t handle)
		{
			return static_cast<std::uint16_t>(handle >> 48);
		}
		std::uint32_t ChildGeneration(std::uint64_t handle)
		{
			return static_cast<std::uint32_t>((handle >> 24) & 0xFFFFFFU);
		}
		std::size_t ChildIndex(std::uint64_t handle)
		{
			auto index = static_cast<std::uint32_t>(handle & 0xFFFFFFU);
			if (index == 0)
				throw ApiException(PhysicsStatus::InvalidHandle, "Physics object handle is null");

			return static_cast<std::size_t>(index - 1);
		}
		void AdvanceGeneration(std::uint32_t& generation)
		{
			generation = (generation + 1) & 0xFFFFFFU;
			if (generation == 0)
				generation = 1;
		}

		// Invoke an ABI implementation with complete C++ exception containment.
		template <typename Func>
		PhysicsStatus ApiCall(Func&& func) noexcept
		{
			try
			{
				g_last_error.clear();
				func();
				return PhysicsStatus::Success;
			}
			catch (ApiException const& ex)
			{
				g_last_error = ex.what();
				return ex.m_status;
			}
			catch (compute::DeviceRemovedException const& ex)
			{
				g_last_error = ex.what();
				return PhysicsStatus::DeviceRemoved;
			}
			catch (std::exception const& ex)
			{
				g_last_error = ex.what();
				return PhysicsStatus::InternalError;
			}
			catch (...)
			{
				g_last_error = "Unknown native physics failure";
				return PhysicsStatus::InternalError;
			}
		}

		// Validate the common size/version prefix while allowing future callers to append fields.
		template <typename T>
		T const& RequireStruct(T const* value)
		{
			if (value == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Required structure pointer is null");
			if (value->header.version != PHYSICS_STRUCT_VERSION || value->header.size < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidStruct, "Structure size or version is incompatible");

			return *value;
		}
		template <typename T>
		T& RequireOutputStruct(T* value)
		{
			if (value == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Required output structure pointer is null");
			if (value->header.version != PHYSICS_STRUCT_VERSION || value->header.size < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidStruct, "Output structure size or version is incompatible");

			return *value;
		}

		// Take a strong reference to the engine identified by 'handle'.
		// The process-wide registry lock is held only for the handle validation itself, so no caller ever
		// waits on the registry while another engine performs GPU work.
		std::shared_ptr<EngineRecord> PinEngine(PhysicsEngineHandle handle)
		{
			auto dll = PinDll();
			LockGuard lock(dll->m_mutex);
			auto& state = *dll->m_interop;
			auto index = EngineIndex(handle);
			if (index >= state.m_engines.size() || !state.m_engines[index])
				throw ApiException(PhysicsStatus::InvalidHandle, "Engine handle is invalid");

			auto& slot = *state.m_engines[index];
			if (slot.m_generation != EngineGeneration(handle) || !slot.m_record)
				throw ApiException(PhysicsStatus::StaleHandle, "Engine handle is stale");

			return slot.m_record;
		}

		// Clear an engine's registry slot so its handles become stale.
		// Separated from record teardown so the caller can retire the slot while holding only the engine
		// lock, and let the record itself be released after both locks are dropped.
		void RetireEngineSlot(PhysicsEngineHandle handle)
		{
			auto dll = PinDll();
			LockGuard lock(dll->m_mutex);
			auto& state = *dll->m_interop;
			auto index = EngineIndex(handle);
			if (index >= state.m_engines.size() || !state.m_engines[index])
				return;

			auto& slot = *state.m_engines[index];
			slot.m_record.reset();
			++slot.m_generation;
			if (slot.m_generation == 0)
				slot.m_generation = 1;
		}

		// Claim a checkpoint's engine identity so restored object handles keep working.
		// Cookies must be unique across live engines, so the uniqueness test and the assignment are made
		// together under the registry lock rather than sampling other engines' state unsynchronised.
		void ClaimEngineCookie(EngineRecord& engine, std::uint16_t cookie)
		{
			if (cookie == 0)
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint engine identity is null");

			auto dll = PinDll();
			LockGuard lock(dll->m_mutex);
			for (auto const& slot : dll->m_interop->m_engines)
			{
				if (slot->m_record && slot->m_record.get() != &engine && slot->m_record->m_cookie == cookie)
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint engine identity is already active");
			}

			engine.m_cookie = cookie;
		}

		// Allocate one nonzero child-handle namespace that cannot alias any live or restored engine.
		std::uint16_t AllocateEngineCookie(InteropState& state)
		{
			auto used = std::bitset<std::numeric_limits<std::uint16_t>::max() + 1>{};
			used.set(0);
			for (auto const& slot : state.m_engines)
			{
				if (slot->m_record)
					used.set(slot->m_record->m_cookie);
			}

			for (auto attempt = std::uint32_t{}; attempt != used.size(); ++attempt)
			{
				auto const cookie = static_cast<std::uint16_t>(state.m_next_cookie++);
				if (!used.test(cookie))
					return cookie;
			}

			throw ApiException(PhysicsStatus::InternalError, "Physics engine identity capacity is exhausted");
		}

		// Scoped access to one engine's state for the duration of an ABI call.
		// Ownership is pinned first and the engine lock is taken only after the registry lock has been
		// released, so the two are never held in that order and destruction cannot pull the record out from
		// under a concurrent call.
		struct EngineScope
		{
			// Declaration order matters: the lock is released before the pin is dropped, so the mutex is
			// never destroyed while still held.
			std::shared_ptr<EngineRecord> m_record;
			std::unique_lock<std::recursive_mutex> m_lock;

			explicit EngineScope(PhysicsEngineHandle handle)
				: m_record(PinEngine(handle))
				, m_lock(m_record->m_lock)
			{
				if (m_record->m_retired)
					throw ApiException(PhysicsStatus::StaleHandle, "Engine handle is stale");
			}
			EngineScope(EngineScope&&) = delete;
			EngineScope(EngineScope const&) = delete;
			EngineScope& operator=(EngineScope&&) = delete;
			EngineScope& operator=(EngineScope const&) = delete;

			EngineRecord& operator*() const
			{
				return *m_record;
			}
			EngineRecord* operator->() const
			{
				return m_record.get();
			}
		};

		void RequireOwner(EngineRecord const& engine)
		{
			if (engine.m_owner_thread_id != GetCurrentThreadId())
				throw ApiException(PhysicsStatus::WrongThread, "Mutable physics operation called from a non-owner OS thread");
		}
		void RequireIdle(EngineRecord const& engine)
		{
			if (engine.m_submitted_step != engine.m_completed_step)
				throw ApiException(PhysicsStatus::StepPending, "Physics operation is not valid while a step is pending");
		}

		template <typename T>
		std::pair<T&, std::size_t> RequireChild(EngineRecord& engine, std::uint64_t handle, std::vector<ObjectSlot<T>>& slots)
		{
			if (ChildCookie(handle) != engine.m_cookie)
				throw ApiException(PhysicsStatus::InvalidHandle, "Object belongs to a different physics engine");

			auto index = ChildIndex(handle);
			if (index >= slots.size())
				throw ApiException(PhysicsStatus::InvalidHandle, "Physics object handle is invalid");

			auto& slot = slots[index];
			if ((slot.m_generation & 0xFFFFFFU) != ChildGeneration(handle) || !slot.m_object)
				throw ApiException(PhysicsStatus::StaleHandle, "Physics object handle is stale");

			return {*slot.m_object, index};
		}
		ShapeRecord& RequireShape(EngineRecord& engine, PhysicsShapeHandle handle)
		{
			return RequireChild(engine, handle, engine.m_shapes).first;
		}
		BodyRecord& RequireBody(EngineRecord& engine, PhysicsBodyHandle handle)
		{
			return RequireChild(engine, handle, engine.m_bodies).first;
		}

		// Resolve one engine-owned articulation after validating its cookie, slot, and generation.
		ArticulationRecord& RequireArticulation(EngineRecord& engine, PhysicsArticulationHandle handle)
		{
			return RequireChild(engine, handle, engine.m_articulations).first;
		}

		// Resolve one ABI-owned persistent constraint after validating its cookie, slot, and generation.
		ConstraintRecord& RequireConstraint(EngineRecord& engine, PhysicsConstraintHandle handle)
		{
			return RequireChild(engine, handle, engine.m_constraints).first;
		}

		// Allocate from an inactive generation slot before growing the registry.
		template <typename T>
		std::pair<ObjectSlot<T>&, std::size_t> AllocateSlot(std::vector<ObjectSlot<T>>& slots)
		{
			for (auto i = std::size_t{}; i != slots.size(); ++i)
			{
				if (!slots[i].m_object)
					return {slots[i], i};
			}

			slots.emplace_back();
			return {slots.back(), slots.size() - 1};
		}

		m4x4 ToNative(PhysicsMatrix4 const& value)
		{
			static_assert(sizeof(value) == sizeof(m4x4));
			auto result = m4x4{};
			memcpy(&result, &value, sizeof(result));
			return result;
		}
		v4 ToNative(PhysicsVector4 const& value)
		{
			static_assert(sizeof(value) == sizeof(v4));
			auto result = v4{};
			memcpy(&result, &value, sizeof(result));
			return result;
		}
		v8force ToForce(PhysicsSpatialVector const& value)
		{
			return v8force{ToNative(value.angular), ToNative(value.linear)};
		}
		v8motion ToMotion(PhysicsSpatialVector const& value)
		{
			return v8motion{ToNative(value.angular), ToNative(value.linear)};
		}
		Inertia ToNative(PhysicsInertia const& value)
		{
			return Inertia{
				ToNative(value.diagonal),
				ToNative(value.products),
				value.centre_of_mass_and_mass.w,
				ToNative(value.centre_of_mass_and_mass).w0()};
		}

		// Copy between ABI and native records whose layouts are deliberately identical.
		template <typename TNative, typename TAbi>
		void CopyLayout(TAbi& dst, TNative const& src)
		{
			static_assert(sizeof(TNative) == sizeof(TAbi));
			memcpy(&dst, &src, sizeof(dst));
		}

		// Preserve the fixed wire/native vector layout without exposing the native SIMD type.
		PhysicsVector4 FromNative(v4 const& value)
		{
			auto result = PhysicsVector4{};
			CopyLayout(result, value);
			return result;
		}

		// Preserve the fixed wire/native matrix layout without exposing the native SIMD type.
		PhysicsMatrix4 FromNative(m4x4 const& value)
		{
			auto result = PhysicsMatrix4{};
			CopyLayout(result, value);
			return result;
		}

		// Return a wire spatial force in angular-then-linear order.
		PhysicsSpatialVector FromNative(v8force const& value)
		{
			return PhysicsSpatialVector{
				.angular = FromNative(value.ang),
				.linear = FromNative(value.lin),
			};
		}

		// Return a wire spatial motion in angular-then-linear order.
		PhysicsSpatialVector FromNative(v8motion const& value)
		{
			return PhysicsSpatialVector{
				.angular = FromNative(value.ang),
				.linear = FromNative(value.lin),
			};
		}

		// Return mass, inertia tensor, and centre of mass in the stable wire representation.
		PhysicsInertia FromNative(Inertia const& value)
		{
			return PhysicsInertia{
				.diagonal = FromNative(value.m_diagonal),
				.products = FromNative(value.m_products),
				.centre_of_mass_and_mass = FromNative(value.m_com_and_mass),
			};
		}

		EngineConfig ToNative(PhysicsEngineConfig const& value)
		{
			auto config = EngineConfig{};
			config.max_collision_pairs = value.max_collision_pairs;
			config.max_collision_events = value.max_collision_events;
			config.max_internal_substeps = value.max_internal_substeps;
			config.sleeping_enabled = value.sleeping_enabled != 0;
			config.sleep_velocity_threshold_lin = value.sleep_velocity_threshold_linear;
			config.sleep_velocity_threshold_ang = value.sleep_velocity_threshold_angular;
			config.sleep_delay_s = value.sleep_delay_seconds;
			config.solver_iterations = value.solver_iterations;
			config.push_out_iterations = value.push_out_iterations;
			config.broadphase_aabb_margin = value.broadphase_aabb_margin;
			config.contact_sort_propagation_scale = value.contact_sort_propagation_scale;
			config.contact_sort_shock_iterations = value.contact_sort_shock_iterations;
			config.contact_sort_shock_alignment = value.contact_sort_shock_alignment;
			config.contact_sort_shock_min_strength = value.contact_sort_shock_min_strength;
			config.contact_sort_shock_decay = value.contact_sort_shock_decay;
			config.penetration_slop = value.penetration_slop;
			config.velocity_baumgarte = value.velocity_baumgarte;
			config.position_slop = value.position_slop;
			config.position_baumgarte = value.position_baumgarte;
			config.contact_slop_scale = value.contact_slop_scale;
			config.support_contact_slop_scale = value.support_contact_slop_scale;
			config.warm_start_scale = value.warm_start_scale;
			config.constraint_relaxation = value.constraint_relaxation;
			config.constraint_coupled_relaxation = value.constraint_coupled_relaxation;
			config.constraint_coupled_backtrack_limit = value.constraint_coupled_backtrack_limit;
			config.constraint_position_relaxation = value.constraint_position_relaxation;
			config.constraint_position_beta = value.constraint_position_beta;
			config.constraint_max_position_speed = value.constraint_max_position_speed;
			config.constraint_regularization = value.constraint_regularization;
			config.constraint_warm_start_factor = value.constraint_warm_start_factor;
			config.deep_penetration_threshold = value.deep_penetration_threshold;
			config.deep_penetration_range = value.deep_penetration_range;
			config.deep_penetration_baumgarte_min = value.deep_penetration_baumgarte_min;
			config.deep_penetration_baumgarte_max = value.deep_penetration_baumgarte_max;
			config.selective_refresh_passes = value.selective_refresh_passes;
			config.selective_refresh_max_pairs = value.selective_refresh_max_pairs;
			config.selective_refresh_body_limit = value.selective_refresh_body_limit;
			config.selective_refresh_contact_limit = value.selective_refresh_contact_limit;
			config.selective_refresh_solver_iterations = value.selective_refresh_solver_iterations;
			config.selective_refresh_position_iterations = value.selective_refresh_position_iterations;
			config.selective_refresh_bias_scale = value.selective_refresh_bias_scale;
			config.selective_refresh_restitution_scale = value.selective_refresh_restitution_scale;
			config.selective_refresh_adaptive_body_limit = value.selective_refresh_adaptive_body_limit;
			config.selective_refresh_adaptive_solver_iterations = value.selective_refresh_adaptive_solver_iterations;
			config.selective_refresh_support_only = value.selective_refresh_support_only != 0;
			config.selective_refresh_resolve_support_only = value.selective_refresh_resolve_support_only != 0;
			config.selective_refresh_depth_slop = value.selective_refresh_depth_slop;
			config.selective_refresh_support_depth_slop = value.selective_refresh_support_depth_slop;
			config.selective_refresh_closing_speed_slop = value.selective_refresh_closing_speed_slop;
			config.selective_refresh_support_alignment = value.selective_refresh_support_alignment;
			config.selective_refresh_aabb_margin = value.selective_refresh_aabb_margin;
			return config;
		}

		void FromNative(PhysicsEngineConfig& value, EngineConfig const& config)
		{
			value = {};
			value.header = {sizeof(value), PHYSICS_STRUCT_VERSION};
			value.max_collision_pairs = config.max_collision_pairs;
			value.max_collision_events = config.max_collision_events;
			value.max_internal_substeps = config.max_internal_substeps;
			value.sleeping_enabled = config.sleeping_enabled;
			value.sleep_velocity_threshold_linear = config.sleep_velocity_threshold_lin;
			value.sleep_velocity_threshold_angular = config.sleep_velocity_threshold_ang;
			value.sleep_delay_seconds = config.sleep_delay_s;
			value.solver_iterations = config.solver_iterations;
			value.push_out_iterations = config.push_out_iterations;
			value.broadphase_aabb_margin = config.broadphase_aabb_margin;
			value.contact_sort_propagation_scale = config.contact_sort_propagation_scale;
			value.contact_sort_shock_iterations = config.contact_sort_shock_iterations;
			value.contact_sort_shock_alignment = config.contact_sort_shock_alignment;
			value.contact_sort_shock_min_strength = config.contact_sort_shock_min_strength;
			value.contact_sort_shock_decay = config.contact_sort_shock_decay;
			value.penetration_slop = config.penetration_slop;
			value.velocity_baumgarte = config.velocity_baumgarte;
			value.position_slop = config.position_slop;
			value.position_baumgarte = config.position_baumgarte;
			value.contact_slop_scale = config.contact_slop_scale;
			value.support_contact_slop_scale = config.support_contact_slop_scale;
			value.warm_start_scale = config.warm_start_scale;
			value.constraint_relaxation = config.constraint_relaxation;
			value.constraint_coupled_relaxation = config.constraint_coupled_relaxation;
			value.constraint_coupled_backtrack_limit = config.constraint_coupled_backtrack_limit;
			value.constraint_position_relaxation = config.constraint_position_relaxation;
			value.constraint_position_beta = config.constraint_position_beta;
			value.constraint_max_position_speed = config.constraint_max_position_speed;
			value.constraint_regularization = config.constraint_regularization;
			value.constraint_warm_start_factor = config.constraint_warm_start_factor;
			value.deep_penetration_threshold = config.deep_penetration_threshold;
			value.deep_penetration_range = config.deep_penetration_range;
			value.deep_penetration_baumgarte_min = config.deep_penetration_baumgarte_min;
			value.deep_penetration_baumgarte_max = config.deep_penetration_baumgarte_max;
			value.selective_refresh_passes = config.selective_refresh_passes;
			value.selective_refresh_max_pairs = config.selective_refresh_max_pairs;
			value.selective_refresh_body_limit = config.selective_refresh_body_limit;
			value.selective_refresh_contact_limit = config.selective_refresh_contact_limit;
			value.selective_refresh_solver_iterations = config.selective_refresh_solver_iterations;
			value.selective_refresh_position_iterations = config.selective_refresh_position_iterations;
			value.selective_refresh_bias_scale = config.selective_refresh_bias_scale;
			value.selective_refresh_restitution_scale = config.selective_refresh_restitution_scale;
			value.selective_refresh_adaptive_body_limit = config.selective_refresh_adaptive_body_limit;
			value.selective_refresh_adaptive_solver_iterations = config.selective_refresh_adaptive_solver_iterations;
			value.selective_refresh_support_only = config.selective_refresh_support_only;
			value.selective_refresh_resolve_support_only = config.selective_refresh_resolve_support_only;
			value.selective_refresh_depth_slop = config.selective_refresh_depth_slop;
			value.selective_refresh_support_depth_slop = config.selective_refresh_support_depth_slop;
			value.selective_refresh_closing_speed_slop = config.selective_refresh_closing_speed_slop;
			value.selective_refresh_support_alignment = config.selective_refresh_support_alignment;
			value.selective_refresh_aabb_margin = config.selective_refresh_aabb_margin;
		}

		// Translate shared optional-lane work and storage costs to fixed-width ABI fields.
		PhysicsFeatureResourceDiagnostics ToAbi(FeatureResourceStats const& value)
		{
			return PhysicsFeatureResourceDiagnostics{
				.dispatch_count = value.m_dispatch_count,
				.reserved = 0,
				.logical_bytes = value.m_logical_bytes,
				.allocated_bytes = value.m_allocated_bytes,
			};
		}

		// Translate persistent-constraint usage and retained capacities to the public diagnostics record.
		PhysicsConstraintFeatureDiagnostics ToAbi(ConstraintFeatureStats const& value)
		{
			return PhysicsConstraintFeatureDiagnostics{
				.declared_count = value.m_declared_count,
				.active_count = value.m_active_count,
				.breakable_count = value.m_breakable_count,
				.slot_capacity = value.m_slot_capacity,
				.body_capacity = value.m_body_capacity,
				.break_capacity = value.m_break_capacity,
				.resources = ToAbi(value.m_resources),
			};
		}

		// Translate reduced-coordinate topology dimensions and storage costs to fixed-width ABI fields.
		PhysicsArticulationFeatureDiagnostics ToAbi(ArticulationFeatureStats const& value)
		{
			return PhysicsArticulationFeatureDiagnostics{
				.articulation_count = value.m_articulation_count,
				.link_count = value.m_link_count,
				.dof_count = value.m_dof_count,
				.position_count = value.m_position_count,
				.velocity_count = value.m_velocity_count,
				.articulation_capacity = value.m_articulation_capacity,
				.link_capacity = value.m_link_capacity,
				.dof_capacity = value.m_dof_capacity,
				.position_capacity = value.m_position_capacity,
				.velocity_capacity = value.m_velocity_capacity,
				.resources = ToAbi(value.m_resources),
			};
		}

		// Translate coupled persistent/contact topology bounds and resource costs to the public record.
		PhysicsCoupledFeatureDiagnostics ToAbi(CoupledFeatureStats const& value)
		{
			return PhysicsCoupledFeatureDiagnostics{
				.constraint_count = value.m_constraint_count,
				.constraint_slot_capacity = value.m_constraint_slot_capacity,
				.target_capacity = value.m_target_capacity,
				.island_capacity = value.m_island_capacity,
				.island_block_capacity = value.m_island_block_capacity,
				.contact_capacity = value.m_contact_capacity,
				.contact_target_capacity = value.m_contact_target_capacity,
				.contact_participant_capacity = value.m_contact_participant_capacity,
				.contact_tree_capacity = value.m_contact_tree_capacity,
				.reserved = 0,
				.resources = ToAbi(value.m_resources),
			};
		}

		// Translate the packed output and sole-readback boundary accounting to the public record.
		PhysicsFrameOutputFeatureDiagnostics ToAbi(FrameOutputFeatureStats const& value)
		{
			return PhysicsFrameOutputFeatureDiagnostics{
				.body_count = value.m_body_count,
				.event_capacity = value.m_event_capacity,
				.articulation_count = value.m_articulation_count,
				.constraint_break_count = value.m_constraint_break_count,
				.coupled_failure_count = value.m_coupled_failure_count,
				.dispatch_count = value.m_dispatch_count,
				.readback_count = value.m_readback_count,
				.reserved1 = 0,
				.logical_bytes = value.m_logical_bytes,
				.allocated_bytes = value.m_allocated_bytes,
				.readback_bytes = value.m_readback_bytes,
			};
		}

		// Translate terminal bounded failure details without exposing native enum layout.
		PhysicsStepFailureDiagnostics ToAbi(StepFailureStats const& value)
		{
			return PhysicsStepFailureDiagnostics{
				.reason = static_cast<std::int32_t>(value.m_reason),
				.substep_index = value.m_substep_index,
				.item_index = value.m_item_index,
				.status = value.m_status,
				.iteration_count = value.m_iteration_count,
				.reserved = 0,
				.identity = value.m_identity,
				.residual = value.m_residual,
				.reserved1 = 0,
			};
		}

		// Require a material slot that can be indexed safely by both CPU and GPU material arrays.
		void RequireMaterialId(std::int32_t material_id)
		{
			if (material_id < 0 || material_id >= Material::MaxMaterialId)
				throw ApiException(PhysicsStatus::InvalidArgument, "Material ID is out of range");
		}

		PhysicsShapeCommon const& RequireShapeCommon(PhysicsShapeCommon const* common)
		{
			auto const& value = RequireStruct(common);
			RequireMaterialId(value.material_id);
			return value;
		}

		template <typename T>
		T const& RequireShapeDesc(T const* value)
		{
			if (value == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Required shape description pointer is null");
			if (value->common.header.version != PHYSICS_STRUCT_VERSION || value->common.header.size < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidStruct, "Shape description size or version is incompatible");

			RequireMaterialId(value->common.material_id);
			return *value;
		}

		// Store a complete location-independent shape blob in a generation slot.
		PhysicsShapeHandle StoreShape(EngineRecord& engine, byte_data<16> data)
		{
			auto [slot, index] = AllocateSlot(engine.m_shapes);
			slot.m_object = std::make_unique<ShapeRecord>();
			slot.m_object->m_data = std::move(data);
			return MakeChildHandle(engine.m_cookie, index, slot.m_generation);
		}

		template <collision::ShapeType TShape>
		PhysicsShapeHandle StoreShape(EngineRecord& engine, TShape const& shape)
		{
			auto data = byte_data<16>{};
			data.push_back({byte_ptr(&shape), static_cast<std::size_t>(shape.m_base.m_size)});
			return StoreShape(engine, std::move(data));
		}

		// Return the number of convex leaves that 'shape' contributes to a collision model.
		// Compounds contribute the leaves of their whole subtree because nested arrays are flattened.
		int CountShapeLeaves(collision::Shape const& shape)
		{
			if (shape.m_type != collision::EShape::Array)
				return 1;

			auto count = 0;
			auto const& arr = collision::shape_cast<collision::ShapeArray>(shape);
			for (auto const* child = arr.begin(), *end = arr.end(); child != end; child = collision::next(child))
				count += CountShapeLeaves(*child);

			return count;
		}

		// Compose 'p2r' through a copied shape subtree so it reads correctly under a new parent.
		// A shape-to-root transform is absolute within its owning root rather than relative to its immediate
		// parent, so every node in the subtree has to be rebased, not just the node being attached.
		void RebaseShapeTree(collision::Shape& shape, m4x4 const& p2r)
		{
			shape.m_s2r = p2r * shape.m_s2r;
			if (shape.m_type != collision::EShape::Array)
				return;

			auto& arr = collision::shape_cast<collision::ShapeArray>(shape);
			for (auto* child = arr.begin(), *end = arr.end(); child != end; child = collision::next(child))
				RebaseShapeTree(*child, p2r);
		}

		PhysicsBodyFlags BodyFlags(BodyRecord const& record)
		{
			auto flags = PhysicsBodyFlags::None;
			if (record.m_enabled) flags = static_cast<PhysicsBodyFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsBodyFlags::Enabled));
			if (record.m_body->Sleeping()) flags = static_cast<PhysicsBodyFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsBodyFlags::Sleeping));
			if (record.m_body->NeverSleep()) flags = static_cast<PhysicsBodyFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsBodyFlags::NeverSleep));
			return flags;
		}
		// Test one strongly typed ABI flag without relying on enum arithmetic operators.
		template <typename TEnum>
		bool HasFlag(TEnum flags, TEnum bit)
		{
			return (static_cast<std::uint32_t>(flags) & static_cast<std::uint32_t>(bit)) != 0;
		}

		// Reject flag bits that this ABI version does not define.
		template <typename TEnum>
		void RequireKnownFlags(TEnum flags, std::uint32_t known, char const* name)
		{
			if ((static_cast<std::uint32_t>(flags) & ~known) != 0)
				throw ApiException(PhysicsStatus::InvalidArgument, std::format("{} contains unknown flag bits", name));
		}

		// Sleeping and NeverSleep describe mutually exclusive policies and must be rejected before native state changes.
		template <typename TEnum>
		void RequireConsistentSleepFlags(TEnum flags, TEnum sleeping, TEnum never_sleep, char const* name)
		{
			if (HasFlag(flags, sleeping) && HasFlag(flags, never_sleep))
				throw ApiException(PhysicsStatus::InvalidArgument, std::format("{} cannot request Sleeping and NeverSleep together", name));
		}

		// Reject unknown body participation modes before constructing or mutating native state.
		void RequireMotionType(PhysicsMotionType motion_type)
		{
			switch (motion_type)
			{
				case PhysicsMotionType::Static:
				case PhysicsMotionType::Dynamic:
				case PhysicsMotionType::Kinematic:
				{
					return;
				}
				default:
				{
					throw ApiException(PhysicsStatus::InvalidArgument, "Body motion type is invalid");
				}
			}
		}

		// Return one stable native link handle from the ABI's immutable topological index.
		LinkHandle RequireLink(ArticulationRecord const& record, std::uint32_t link_index)
		{
			if (link_index >= static_cast<std::uint32_t>(record.m_articulation->LinkCount()))
				throw ApiException(PhysicsStatus::InvalidArgument, "Articulation link index is out of range");

			return record.m_articulation->LinkAt(static_cast<int>(link_index));
		}

		// Translate the wire root policy through an exhaustive switch.
		EArticulationRootType ToNative(PhysicsArticulationRoot value)
		{
			switch (value)
			{
				case PhysicsArticulationRoot::Fixed:
				{
					return EArticulationRootType::Fixed;
				}
				case PhysicsArticulationRoot::Floating:
				{
					return EArticulationRootType::Floating;
				}
				default:
				{
					throw ApiException(PhysicsStatus::InvalidArgument, "Articulation root type is invalid");
				}
			}
		}

		// Translate one wire joint-axis type through an exhaustive switch.
		EArticulationAxisType ToNative(PhysicsArticulationAxis value)
		{
			switch (value)
			{
				case PhysicsArticulationAxis::Revolute:
				{
					return EArticulationAxisType::Revolute;
				}
				case PhysicsArticulationAxis::Prismatic:
				{
					return EArticulationAxisType::Prismatic;
				}
				default:
				{
					throw ApiException(PhysicsStatus::InvalidArgument, "Articulation axis type is invalid");
				}
			}
		}

		// Translate one wire D6 mode through an exhaustive switch.
		EConstraintAxisMode ToNative(PhysicsConstraintMode value)
		{
			switch (value)
			{
				case PhysicsConstraintMode::Free:
				{
					return EConstraintAxisMode::Free;
				}
				case PhysicsConstraintMode::Locked:
				{
					return EConstraintAxisMode::Locked;
				}
				case PhysicsConstraintMode::Limited:
				{
					return EConstraintAxisMode::Limited;
				}
				case PhysicsConstraintMode::Driven:
				{
					return EConstraintAxisMode::Driven;
				}
				default:
				{
					throw ApiException(PhysicsStatus::InvalidArgument, "Constraint axis mode is invalid");
				}
			}
		}

		// Build one native immutable articulation link while retaining its ABI shape identity for ownership checks.
		ArticulationLinkDesc ToNative(EngineRecord& engine, PhysicsArticulationLink const& value, PhysicsShapeHandle& shape_handle)
		{
			RequireKnownFlags(value.flags, static_cast<std::uint32_t>(PhysicsArticulationLinkFlags::CollideParent) | static_cast<std::uint32_t>(PhysicsArticulationLinkFlags::CollideSelf), "Articulation link flags");
			auto shape = static_cast<collision::Shape const*>(nullptr);
			if (value.shape != 0)
			{
				shape = RequireShape(engine, value.shape).Shape();
				shape_handle = value.shape;
			}

			return ArticulationLinkDesc{
				.m_inertia = ToNative(value.inertia),
				.m_shape = shape,
				.m_shape_to_link = ToNative(value.shape_to_link),
				.m_collide_parent = HasFlag(value.flags, PhysicsArticulationLinkFlags::CollideParent),
				.m_collide_self = HasFlag(value.flags, PhysicsArticulationLinkFlags::CollideSelf),
			};
		}

		// Build one native reduced joint from its fixed-size wire representation.
		ArticulationJointDesc ToNative(PhysicsArticulationJoint const& value)
		{
			if (value.dof_count > 6)
				throw ApiException(PhysicsStatus::InvalidArgument, "Articulation joint degree count exceeds six");

			auto result = ArticulationJointDesc{
				.m_joint_to_parent = ToNative(value.joint_to_parent),
				.m_joint_to_child = ToNative(value.joint_to_child),
				.m_axes = {},
				.m_initial_position = {},
				.m_initial_velocity = {},
				.m_dof_count = static_cast<int>(value.dof_count),
			};
			for (auto axis_index = std::uint32_t{}; axis_index != value.dof_count; ++axis_index)
			{
				result.m_axes[axis_index] = ArticulationAxisDesc{
					.m_type = ToNative(value.axis_types[axis_index]),
					.m_axis = ToNative(value.axes[axis_index]),
				};
				result.m_initial_position[axis_index] = value.initial_positions[axis_index];
				result.m_initial_velocity[axis_index] = value.initial_velocities[axis_index];
			}
			return result;
		}

		// Return the flattened non-root joint dimension used by all articulation scalar transfers.
		std::uint32_t JointDofCount(Articulation const& articulation)
		{
			auto count = std::uint32_t{};
			for (auto link_index = 1; link_index != articulation.LinkCount(); ++link_index)
				count += static_cast<std::uint32_t>(articulation.JointDofCount(articulation.LinkAt(link_index)));

			return count;
		}

		// Return current whole-tree participation and sleeping policy in wire flags.
		PhysicsArticulationFlags ArticulationFlags(ArticulationRecord const& record)
		{
			auto flags = PhysicsArticulationFlags::None;
			if (record.m_enabled) flags = static_cast<PhysicsArticulationFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsArticulationFlags::Enabled));
			if (record.m_articulation->Sleeping()) flags = static_cast<PhysicsArticulationFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsArticulationFlags::Sleeping));
			if (record.m_articulation->NeverSleep()) flags = static_cast<PhysicsArticulationFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsArticulationFlags::NeverSleep));
			return flags;
		}

		// Fill whole-tree state without exposing the native articulation identity or storage.
		void FillArticulationState(PhysicsArticulationState& state, PhysicsArticulationHandle handle, ArticulationRecord const& record)
		{
			state = PhysicsArticulationState{
				.header = {sizeof(PhysicsArticulationState), PHYSICS_STRUCT_VERSION},
				.articulation = handle,
				.root_to_world = FromNative(record.m_articulation->RootToWorld()),
				.root_velocity = FromNative(record.m_articulation->RootVelocity()),
				.root_force = FromNative(record.m_articulation->RootForce()),
				.user_tag = record.m_user_tag,
				.link_count = static_cast<std::uint32_t>(record.m_articulation->LinkCount()),
				.joint_dof_count = JointDofCount(*record.m_articulation),
				.flags = ArticulationFlags(record),
				.reserved = 0,
			};
		}

		// Validate all wire articulation state before any native field is changed.
		void ValidateArticulationState(PhysicsArticulationState const& state, PhysicsArticulationHandle handle, ArticulationRecord const& record, float const* positions, float const* velocities, float const* forces, std::uint32_t scalar_count)
		{
			auto const expected_count = JointDofCount(*record.m_articulation);
			if (state.articulation != handle || state.link_count != static_cast<std::uint32_t>(record.m_articulation->LinkCount()) || state.joint_dof_count != expected_count)
				throw ApiException(PhysicsStatus::InvalidArgument, "Articulation state identity or dimensions do not match the target");
			if (scalar_count != expected_count || (scalar_count != 0 && (positions == nullptr || velocities == nullptr || forces == nullptr)))
				throw ApiException(PhysicsStatus::InvalidArgument, "Articulation scalar arrays do not match the topology");
			RequireKnownFlags(state.flags, static_cast<std::uint32_t>(PhysicsArticulationFlags::Enabled) | static_cast<std::uint32_t>(PhysicsArticulationFlags::Sleeping) | static_cast<std::uint32_t>(PhysicsArticulationFlags::NeverSleep), "Articulation state flags");
			RequireConsistentSleepFlags(state.flags, PhysicsArticulationFlags::Sleeping, PhysicsArticulationFlags::NeverSleep, "Articulation state flags");

			auto finite = [](float value) { return IsFinite(value); };
			if ((scalar_count != 0 &&
				(!std::ranges::all_of(std::span{positions, scalar_count}, finite) ||
				 !std::ranges::all_of(std::span{velocities, scalar_count}, finite) ||
				 !std::ranges::all_of(std::span{forces, scalar_count}, finite))) ||
				!IsFinite(ToMotion(state.root_velocity).ang) ||
				!IsFinite(ToMotion(state.root_velocity).lin) ||
				!IsFinite(ToForce(state.root_force).ang) ||
				!IsFinite(ToForce(state.root_force).lin))
				throw ApiException(PhysicsStatus::InvalidArgument, "Articulation state contains a non-finite scalar");
		}

		// Apply one fully validated whole-tree state in stable topological order.
		void ApplyArticulationState(ArticulationRecord& record, PhysicsArticulationState const& state, float const* positions, float const* velocities, float const* forces)
		{
			auto& articulation = *record.m_articulation;
			articulation.RootToWorld(ToNative(state.root_to_world));
			if (articulation.RootType() == EArticulationRootType::Floating)
			{
				articulation.RootVelocity(ToMotion(state.root_velocity));
				articulation.RootForce(ToForce(state.root_force));
			}

			auto scalar_offset = std::uint32_t{};
			for (auto link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				auto link = articulation.LinkAt(link_index);
				auto count = static_cast<std::uint32_t>(articulation.JointDofCount(link));
				if (count == 0)
					continue;

				articulation.JointPosition(link, std::span{positions + scalar_offset, count});
				articulation.JointVelocity(link, std::span{velocities + scalar_offset, count});
				articulation.JointForce(link, std::span{forces + scalar_offset, count});
				scalar_offset += count;
			}

			articulation.NeverSleep(HasFlag(state.flags, PhysicsArticulationFlags::NeverSleep));
			articulation.Sleeping(HasFlag(state.flags, PhysicsArticulationFlags::Sleeping));
			record.m_enabled = HasFlag(state.flags, PhysicsArticulationFlags::Enabled);
			record.m_user_tag = state.user_tag;
		}

		// Translate one scalar wire coordinate into the native D6 representation.
		ConstraintAxisDesc ToNative(PhysicsConstraintAxis const& value)
		{
			return ConstraintAxisDesc{
				.m_mode = ToNative(value.mode),
				.m_limits = {value.lower_limit, value.upper_limit},
				.m_target_position = value.target_position,
				.m_target_velocity = value.target_velocity,
				.m_stiffness = value.stiffness,
				.m_damping = value.damping,
				.m_max_force = value.max_force,
			};
		}

		// Resolve one wire endpoint to a stable native identity and capture the referenced ABI handle.
		BodyFrame ToNative(EngineRecord& engine, PhysicsConstraintFrame const& value, PhysicsBodyHandle& body_ref, PhysicsArticulationHandle& articulation_ref)
		{
			auto body = BodyRef::World();
			switch (value.type)
			{
				case PhysicsConstraintEndpoint::World:
				{
					if (value.object_handle != 0)
						throw ApiException(PhysicsStatus::InvalidArgument, "World constraint endpoint must not name an object");

					break;
				}
				case PhysicsConstraintEndpoint::RigidBody:
				{
					auto handle = static_cast<PhysicsBodyHandle>(value.object_handle);
					auto& record = RequireBody(engine, handle);
					body = BodyRef::Rigid(*record.m_body);
					body_ref = handle;
					break;
				}
				case PhysicsConstraintEndpoint::ArticulationLink:
				{
					auto handle = static_cast<PhysicsArticulationHandle>(value.object_handle);
					auto& record = RequireArticulation(engine, handle);
					body = BodyRef::Link(*record.m_articulation, RequireLink(record, value.link_index));
					articulation_ref = handle;
					break;
				}
				default:
				{
					throw ApiException(PhysicsStatus::InvalidArgument, "Constraint endpoint type is invalid");
				}
			}

			return BodyFrame{
				.m_body = body,
				.m_constraint_to_body = ToNative(value.constraint_to_body),
			};
		}

		// Resolve and validate one complete D6 descriptor before mutating persistent storage or references.
		D6ConstraintDesc ToNative(EngineRecord& engine, PhysicsD6Constraint const& value, std::array<PhysicsBodyHandle, 2>& body_refs, std::array<PhysicsArticulationHandle, 2>& articulation_refs)
		{
			RequireKnownFlags(value.flags, static_cast<std::uint32_t>(PhysicsConstraintFlags::Enabled) | static_cast<std::uint32_t>(PhysicsConstraintFlags::CollideConnected), "Constraint flags");
			auto result = D6ConstraintDesc{
				.m_frame_a = ToNative(engine, value.frame_a, body_refs[0], articulation_refs[0]),
				.m_frame_b = ToNative(engine, value.frame_b, body_refs[1], articulation_refs[1]),
				.m_linear = {},
				.m_angular = {},
				.m_break_force = value.break_force,
				.m_break_torque = value.break_torque,
				.m_collide_connected = HasFlag(value.flags, PhysicsConstraintFlags::CollideConnected),
				.m_enabled = HasFlag(value.flags, PhysicsConstraintFlags::Enabled),
			};
			for (auto axis_index = 0; axis_index != 3; ++axis_index)
			{
				result.m_linear[axis_index] = ToNative(value.linear[axis_index]);
				result.m_angular[axis_index] = ToNative(value.angular[axis_index]);
			}
			return result;
		}

		// Retain all endpoint owners after the native constraint has accepted its descriptor.
		void RetainConstraintReferences(EngineRecord& engine, ConstraintRecord const& constraint)
		{
			for (auto handle : constraint.m_body_refs)
			{
				if (handle != 0)
					++RequireBody(engine, handle).m_constraint_refs;
			}
			for (auto handle : constraint.m_articulation_refs)
			{
				if (handle != 0)
					++RequireArticulation(engine, handle).m_constraint_refs;
			}
		}

		// Release exactly the references retained for one accepted persistent descriptor.
		void ReleaseConstraintReferences(EngineRecord& engine, ConstraintRecord const& constraint)
		{
			for (auto handle : constraint.m_body_refs)
			{
				if (handle != 0)
					--RequireBody(engine, handle).m_constraint_refs;
			}
			for (auto handle : constraint.m_articulation_refs)
			{
				if (handle != 0)
					--RequireArticulation(engine, handle).m_constraint_refs;
			}
		}

		// Fill a complete body state without exposing the native body's address.
		void FillBodyState(PhysicsBodyState& state, PhysicsBodyHandle handle, BodyRecord const& record)
		{
			state = {};
			state.header = {sizeof(state), PHYSICS_STRUCT_VERSION};
			state.body = handle;
			state.shape = record.m_shape;
			CopyLayout(state.object_to_world, record.m_body->O2W());
			CopyLayout(state.inertia, record.m_body->InertiaOS());
			CopyLayout(state.momentum, record.m_body->MomentumWS());
			CopyLayout(state.velocity, record.m_body->VelocityWS());
			CopyLayout(state.force, record.m_body->ForceWS());
			CopyLayout(state.gravity, record.m_body->GravityWS());
			state.user_tag = record.m_user_tag;
			state.motion_type = record.m_motion_type;
			state.flags = BodyFlags(record);
		}

		void ApplyBodyState(BodyRecord& record, PhysicsBodyState const& state)
		{
			record.m_body->O2W(ToNative(state.object_to_world));
			record.m_body->SetMassProperties(ToNative(state.inertia), ToNative(state.inertia.centre_of_mass_and_mass).w0());
			record.m_body->MomentumWS(ToForce(state.momentum));
			record.m_body->ForceWS(ToForce(state.force));
			record.m_body->GravityWS(ToNative(state.gravity));
			record.m_body->NeverSleep(HasFlag(state.flags, PhysicsBodyFlags::NeverSleep));
			record.m_body->Sleeping(HasFlag(state.flags, PhysicsBodyFlags::Sleeping));
			record.m_enabled = HasFlag(state.flags, PhysicsBodyFlags::Enabled);
			record.m_user_tag = state.user_tag;
			record.m_motion_type = state.motion_type;
		}

		// Validate every command before applying any command so a failed batch has no partial effect.
		void ValidateCommands(EngineRecord& engine, PhysicsBodyCommand const* commands, std::uint32_t count)
		{
			if (count != 0 && commands == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Command buffer is null");

			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				RequireStruct(&commands[i]);
				RequireBody(engine, commands[i].body);
				switch (commands[i].type)
				{
					case PhysicsBodyCommandType::SetTransform:
					case PhysicsBodyCommandType::SetVelocity:
					case PhysicsBodyCommandType::SetMomentum:
					case PhysicsBodyCommandType::SetForce:
					case PhysicsBodyCommandType::ApplyImpulse:
					case PhysicsBodyCommandType::SetGravity:
					case PhysicsBodyCommandType::SetKinematicTransform:
					case PhysicsBodyCommandType::SetEnabled:
					case PhysicsBodyCommandType::Wake:
					case PhysicsBodyCommandType::Sleep:
						break;
					case PhysicsBodyCommandType::ApplyForce:
					{
						if (commands[i].at.w != 0.0f)
							throw ApiException(PhysicsStatus::InvalidArgument, "Force application point must be an offset from the model origin");

						break;
					}
					default:
						throw ApiException(PhysicsStatus::InvalidArgument, "Unknown body command type");
				}
			}
		}

		void ApplyCommands(EngineRecord& engine, PhysicsBodyCommand const* commands, std::uint32_t count)
		{
			ValidateCommands(engine, commands, count);
			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& command = commands[i];
				auto& record = RequireBody(engine, command.body);
				auto& body = *record.m_body;
				switch (command.type)
				{
					case PhysicsBodyCommandType::SetTransform:
					{
						body.O2W(ToNative(command.transform));
						break;
					}
					case PhysicsBodyCommandType::SetVelocity:
					{
						body.VelocityWS(ToMotion(command.value));
						break;
					}
					case PhysicsBodyCommandType::SetMomentum:
					{
						body.MomentumWS(ToForce(command.value));
						break;
					}
					case PhysicsBodyCommandType::SetForce:
					{
						body.ZeroForces();
						body.ApplyForceWS(ToForce(command.value));
						break;
					}
					case PhysicsBodyCommandType::ApplyForce:
					{
						body.ApplyForceWS(ToNative(command.value.linear), ToNative(command.value.angular), ToNative(command.at));
						break;
					}
					case PhysicsBodyCommandType::ApplyImpulse:
					{
						auto impulse = ToForce(command.value);
						body.MomentumWS(body.MomentumWS() + impulse);
						break;
					}
					case PhysicsBodyCommandType::SetGravity:
					{
						body.GravityWS(ToNative(command.value.linear));
						break;
					}
					case PhysicsBodyCommandType::SetKinematicTransform:
					{
						if (record.m_motion_type != PhysicsMotionType::Kinematic)
							throw ApiException(PhysicsStatus::InvalidArgument, "Kinematic transform command requires a kinematic body");

						body.O2W(ToNative(command.transform));
						break;
					}
					case PhysicsBodyCommandType::SetEnabled:
					{
						record.m_enabled = command.flags != 0;
						break;
					}
					case PhysicsBodyCommandType::Wake:
					{
						body.Wake();
						break;
					}
					case PhysicsBodyCommandType::Sleep:
					{
						body.Sleep();
						break;
					}
					default:
					{
						throw ApiException(PhysicsStatus::InvalidArgument, "Unknown body command type");
					}
				}
			}
		}

		// Validate, gather every enabled object lane, and submit one frame with all internal substeps GPU-resident.
		void BeginStep(EngineRecord& engine, float elapsed_seconds, double absolute_time_seconds, std::uint32_t substep_count, PhysicsBodyCommand const* commands, std::uint32_t command_count)
		{
			RequireOwner(engine);
			RequireIdle(engine);
			if (!(elapsed_seconds > 0.0f) || !std::isfinite(elapsed_seconds))
				throw ApiException(PhysicsStatus::InvalidArgument, "Step duration must be finite and positive");
			if (substep_count == 0 || substep_count > static_cast<std::uint32_t>(engine.m_engine->Config().max_internal_substeps))
				throw ApiException(PhysicsStatus::InvalidArgument, "Internal substep count is outside the configured bound");

			ApplyCommands(engine, commands, command_count);

			// Stable slot ordering makes replay checksums, topology packing, and event ordering deterministic.
			engine.m_step_bodies.clear();
			engine.m_step_articulations.clear();
			engine.m_events.clear();
			for (auto& slot : engine.m_bodies)
			{
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				slot.m_object->m_sleep_before_step = slot.m_object->m_body->Sleeping();
				engine.m_step_bodies.push_back(slot.m_object->m_body.get());
			}
			for (auto& slot : engine.m_articulations)
			{
				if (slot.m_object && slot.m_object->m_enabled)
					engine.m_step_articulations.push_back(slot.m_object->m_articulation.get());
			}

			engine.m_engine->BeginStep(Engine::StepInput{
				.m_bodies = engine.m_step_bodies,
				.m_articulations = engine.m_step_articulations,
				.m_constraints = engine.m_constraint_set.Count() != 0 ? &engine.m_constraint_set : nullptr,
				.m_elapsed_seconds = elapsed_seconds,
				.m_substep_count = static_cast<int>(substep_count),
				.m_time_s = absolute_time_seconds,
			});
			++engine.m_submitted_step;
		}

		void CompleteStep(EngineRecord& engine)
		{
			RequireOwner(engine);
			if (engine.m_submitted_step == engine.m_completed_step)
				throw ApiException(PhysicsStatus::NoStepPending, "CompleteStep called without a pending step");

			try
			{
				engine.m_engine->CompleteStep();
			}
			catch (...)
			{
				// Output validation consumes the submitted native frame; synchronize the ABI transaction so cleanup and correction remain possible.
				if (!engine.m_engine->StepPending())
				{
					engine.m_completed_step = engine.m_submitted_step;
					engine.m_events.clear();
				}
				throw;
			}

			// Lifecycle events are derived only after native readback has produced the completed state.
			for (auto i = std::size_t{}; i != engine.m_bodies.size(); ++i)
			{
				auto& slot = engine.m_bodies[i];
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				auto sleeping = slot.m_object->m_body->Sleeping();
				if (sleeping == slot.m_object->m_sleep_before_step)
					continue;

				engine.m_events.push_back(PhysicsEvent{
					.header = {sizeof(PhysicsEvent), PHYSICS_STRUCT_VERSION},
					.type = sleeping ? PhysicsEventType::Sleep : PhysicsEventType::Wake,
					.point_count = 0,
					.body_a = MakeChildHandle(engine.m_cookie, i, slot.m_generation),
					.child_a = PHYSICS_NO_CHILD,
					.child_b = PHYSICS_NO_CHILD,
				});
			}
			engine.m_completed_step = engine.m_submitted_step;
		}

		// Clear terminal GPU work before destroying children whose addresses remain in the pending-step body list.
		void PrepareChildDestroy(EngineRecord& engine)
		{
			if (engine.m_submitted_step == engine.m_completed_step)
				return;

			auto reason = engine.m_engine->Device()->GetDeviceRemovedReason();
			if (!FAILED(reason))
				throw ApiException(PhysicsStatus::StepPending, "The engine has a pending simulation step");

			engine.m_engine->AbandonStep();
			engine.m_completed_step = engine.m_submitted_step;
		}

		// Bring the engine to an idle state so that complete, self-consistent results can be serialised.
		// A pending step is drained rather than refused, because the caller has no way to describe a
		// half-submitted step and would otherwise have to complete the step itself just to save state.
		void DrainPendingStep(EngineRecord& engine)
		{
			RequireOwner(engine);
			if (engine.m_submitted_step == engine.m_completed_step)
				return;

			CompleteStep(engine);
		}

		PhysicsBodySnapshot MakeSnapshot(PhysicsBodyHandle handle, BodyRecord const& record)
		{
			auto snapshot = PhysicsBodySnapshot{};
			snapshot.header = {sizeof(snapshot), PHYSICS_STRUCT_VERSION};
			snapshot.body = handle;
			snapshot.shape = record.m_shape;
			CopyLayout(snapshot.object_to_world, record.m_body->O2W());
			CopyLayout(snapshot.momentum, record.m_body->MomentumWS());
			CopyLayout(snapshot.velocity, record.m_body->VelocityWS());
			snapshot.user_tag = record.m_user_tag;
			snapshot.motion_type = record.m_motion_type;
			snapshot.flags = BodyFlags(record);
			return snapshot;
		}

		std::uint64_t HashBytes(std::uint64_t hash, void const* data, std::size_t size)
		{
			auto bytes = static_cast<std::uint8_t const*>(data);
			for (auto i = std::size_t{}; i != size; ++i)
			{
				hash ^= bytes[i];
				hash *= 1099511628211ULL;
			}
			return hash;
		}

		std::uint64_t StateChecksum(EngineRecord const& engine)
		{
			auto hash = 14695981039346656037ULL;
			for (auto i = std::size_t{}; i != engine.m_bodies.size(); ++i)
			{
				auto const& slot = engine.m_bodies[i];
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				auto snapshot = MakeSnapshot(MakeChildHandle(engine.m_cookie, i, slot.m_generation), *slot.m_object);
				hash = HashBytes(hash, &snapshot.body, sizeof(snapshot) - offsetof(PhysicsBodySnapshot, body));
			}

			// Hash articulation state in stable slot and topology order so deterministic replay covers every simulated degree of freedom.
			for (auto i = std::size_t{}; i != engine.m_articulations.size(); ++i)
			{
				auto const& slot = engine.m_articulations[i];
				if (!slot.m_object)
					continue;

				auto const& record = *slot.m_object;
				auto const& articulation = *record.m_articulation;
				auto const handle = MakeChildHandle(engine.m_cookie, i, slot.m_generation);
				auto const enabled = std::uint32_t{record.m_enabled ? 1U : 0U};
				auto const sleeping = std::uint32_t{articulation.Sleeping() ? 1U : 0U};
				auto const never_sleep = std::uint32_t{articulation.NeverSleep() ? 1U : 0U};
				auto const root_to_world = articulation.RootToWorld();
				auto const root_velocity = articulation.RootVelocity();
				auto const root_force = articulation.RootForce();
				hash = HashBytes(hash, &handle, sizeof(handle));
				hash = HashBytes(hash, &record.m_user_tag, sizeof(record.m_user_tag));
				hash = HashBytes(hash, &enabled, sizeof(enabled));
				hash = HashBytes(hash, &sleeping, sizeof(sleeping));
				hash = HashBytes(hash, &never_sleep, sizeof(never_sleep));
				hash = HashBytes(hash, &root_to_world, sizeof(root_to_world));
				hash = HashBytes(hash, &root_velocity, sizeof(root_velocity));
				hash = HashBytes(hash, &root_force, sizeof(root_force));

				for (auto link_index = 1; link_index != articulation.LinkCount(); ++link_index)
				{
					auto const link = articulation.LinkAt(link_index);
					auto const position = articulation.JointPosition(link);
					auto const velocity = articulation.JointVelocity(link);
					auto const force = articulation.JointForce(link);
					hash = HashBytes(hash, position.data(), position.size_bytes());
					hash = HashBytes(hash, velocity.data(), velocity.size_bytes());
					hash = HashBytes(hash, force.data(), force.size_bytes());
				}
			}
			return hash;
		}

		inline constexpr std::uint64_t CheckpointMagic = 0x3154504B43595052ULL; // "RYPCKPT1"

		struct CheckpointHeader
		{
			std::uint64_t m_magic;
			std::uint32_t m_version;
			std::uint32_t m_header_size;
			std::uint64_t m_total_size;
			std::uint64_t m_payload_checksum;
			std::uint16_t m_engine_cookie;
			std::uint16_t m_reserved0;
			std::uint32_t m_shape_count;
			std::uint32_t m_body_count;
			std::uint32_t m_reserved1;
			std::uint64_t m_submitted_step;
			std::uint64_t m_completed_step;
			PhysicsEngineConfig m_config;
		};

		struct CheckpointShape
		{
			PhysicsShapeHandle m_handle;
			std::uint32_t m_size;
			std::uint32_t m_child_count;
		};

		struct CheckpointBody
		{
			PhysicsBodyState m_state;
		};

		std::uint64_t CheckpointRequiredSize(EngineRecord const& engine)
		{
			// Version three rejects optional topology explicitly rather than silently omitting unrecoverable state.
			if (std::ranges::any_of(engine.m_articulations, [](auto const& slot) { return slot.m_object != nullptr; }) ||
				std::ranges::any_of(engine.m_constraints, [](auto const& slot) { return slot.m_object != nullptr; }))
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint serialization does not yet support articulations or persistent constraints");

			auto size = std::uint64_t{sizeof(CheckpointHeader)} +
				std::uint64_t{Material::MaxMaterialId} * sizeof(PhysicsMaterial);
			for (auto const& slot : engine.m_shapes)
			{
				if (slot.m_object)
					size += sizeof(CheckpointShape) +
						slot.m_object->m_data.size() +
						std::uint64_t{slot.m_object->m_children.size()} * sizeof(PhysicsShapeHandle);
			}
			for (auto const& slot : engine.m_bodies)
			{
				if (slot.m_object)
					size += sizeof(CheckpointBody);
			}
			return size;
		}

		template <typename T>
		void WriteCheckpointValue(std::byte*& output, T const& value)
		{
			memcpy(output, &value, sizeof(value));
			output += sizeof(value);
		}

		template <typename T>
		T ReadCheckpointValue(std::byte const*& input, std::byte const* end)
		{
			if (static_cast<std::size_t>(end - input) < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint is truncated");

			auto value = T{};
			memcpy(&value, input, sizeof(value));
			input += sizeof(value);
			return value;
		}

		// Validate a location-independent shape blob before adopting it into the live registry.
		void ValidateCheckpointShape(std::span<std::byte const> data)
		{
			if (data.size() < sizeof(collision::Shape) || (data.size() & 0xFU) != 0)
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an invalid shape size");

			// Validate every nested record before allowing collision code to walk the location-independent blob.
			auto validate = [&](auto const& self, std::span<std::byte const> bytes) -> std::size_t
			{
				if (bytes.size() < sizeof(collision::Shape))
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape is truncated");

				auto const& shape = *reinterpret_cast<collision::Shape const*>(bytes.data());
				if (shape.m_size < static_cast<int>(sizeof(collision::Shape)) ||
					(shape.m_size & 0xFU) != 0 ||
					static_cast<std::size_t>(shape.m_size) > bytes.size())
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an invalid nested shape size");
				if (shape.m_type < collision::EShape::Sphere || shape.m_type >= collision::EShape::NumberOf)
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an unknown shape type");
				RequireMaterialId(shape.m_material_id);

				auto minimum_size = std::size_t{};
				switch (shape.m_type)
				{
					case collision::EShape::Sphere: { minimum_size = sizeof(collision::ShapeSphere); break; }
					case collision::EShape::Box: { minimum_size = sizeof(collision::ShapeBox); break; }
					case collision::EShape::Line: { minimum_size = sizeof(collision::ShapeLine); break; }
					case collision::EShape::Triangle: { minimum_size = sizeof(collision::ShapeTriangle); break; }
					case collision::EShape::Polytope: { minimum_size = sizeof(collision::ShapePolytope); break; }
					case collision::EShape::Array: { minimum_size = sizeof(collision::ShapeArray); break; }
					default: { throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an unsupported shape type"); }
				}
				if (static_cast<std::size_t>(shape.m_size) < minimum_size)
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape record is smaller than its type");

				if (shape.m_type == collision::EShape::Array)
				{
					if (shape.m_size < static_cast<int>(sizeof(collision::ShapeArray)))
						throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound header is truncated");

					auto const& array = reinterpret_cast<collision::ShapeArray const&>(shape);
					if (array.m_num_shapes < 0)
						throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound child count is invalid");

					auto offset = sizeof(collision::ShapeArray);
					for (auto i = 0; i != array.m_num_shapes; ++i)
					{
						if (offset >= static_cast<std::size_t>(shape.m_size))
							throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound child data is truncated");

						offset += self(self, bytes.subspan(offset, static_cast<std::size_t>(shape.m_size) - offset));
					}
					if (offset != static_cast<std::size_t>(shape.m_size))
						throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound size does not match its children");
				}

				return static_cast<std::size_t>(shape.m_size);
			};

			if (validate(validate, data) != data.size())
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape size does not match its blob");
		}
	InteropState::InteropState()
		: m_gpu_backends()
		, m_engines()
		, m_next_cookie(1)
	{}
	InteropState::~InteropState() = default;
	bool InteropState::HasLiveEngines() const
	{
		return std::ranges::any_of(m_engines, [](auto const& slot)
		{
			return slot->m_record != nullptr;
		});
	}

	// Retain one queue per device for the context lifetime so repeated engine construction cannot exhaust heavyweight D3D12 queue resources.
	InteropGpuBackend& InteropState::GpuBackend(void* external_device)
	{
		assert(external_device != nullptr);

		auto iter = std::ranges::find_if(m_gpu_backends, [=](auto const& backend)
		{
			return
				backend->m_external_device == external_device ||
				backend->m_gpu.device() == external_device;
		});
		if (iter != m_gpu_backends.end())
			return **iter;

		auto backend = std::make_unique<InteropGpuBackend>(external_device);
		auto& result = *backend;
		m_gpu_backends.push_back(std::move(backend));
		return result;
	}
}

using namespace pr::physics;

extern "C"
{
	std::uint32_t __stdcall Physics_ApiVersion()
	{
		return PHYSICS_API_VERSION;
	}

	PhysicsStatus __stdcall Physics_StructSize(PhysicsStructId struct_id, std::uint32_t* size)
	{
		return pr::physics::ApiCall([&]
		{
			if (size == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Size output pointer is null");

			switch (struct_id)
			{
				case PhysicsStructId::EngineConfig: *size = sizeof(PhysicsEngineConfig); break;
				case PhysicsStructId::ShapeCommon: *size = sizeof(PhysicsShapeCommon); break;
				case PhysicsStructId::SphereShape: *size = sizeof(PhysicsSphereShape); break;
				case PhysicsStructId::BoxShape: *size = sizeof(PhysicsBoxShape); break;
				case PhysicsStructId::LineShape: *size = sizeof(PhysicsLineShape); break;
				case PhysicsStructId::TriangleShape: *size = sizeof(PhysicsTriangleShape); break;
				case PhysicsStructId::BodyDesc: *size = sizeof(PhysicsBodyDesc); break;
				case PhysicsStructId::BodyState: *size = sizeof(PhysicsBodyState); break;
				case PhysicsStructId::BodyCommand: *size = sizeof(PhysicsBodyCommand); break;
				case PhysicsStructId::BodySnapshot: *size = sizeof(PhysicsBodySnapshot); break;
				case PhysicsStructId::Event: *size = sizeof(PhysicsEvent); break;
				case PhysicsStructId::Diagnostics: *size = sizeof(PhysicsDiagnostics); break;
				case PhysicsStructId::Material: *size = sizeof(PhysicsMaterial); break;
				case PhysicsStructId::ArticulationDesc: *size = sizeof(PhysicsArticulationDesc); break;
				case PhysicsStructId::ArticulationLink: *size = sizeof(PhysicsArticulationLink); break;
				case PhysicsStructId::ArticulationJoint: *size = sizeof(PhysicsArticulationJoint); break;
				case PhysicsStructId::ArticulationState: *size = sizeof(PhysicsArticulationState); break;
				case PhysicsStructId::ArticulationLinkState: *size = sizeof(PhysicsArticulationLinkState); break;
				case PhysicsStructId::D6Constraint: *size = sizeof(PhysicsD6Constraint); break;
				default: throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Unknown physics structure identifier");
			}
		});
	}

	PhysicsStatus __stdcall Physics_LastError(char* buffer, std::uint32_t capacity, std::uint32_t* required)
	{
		try
		{
			if (required == nullptr)
				return PhysicsStatus::InvalidArgument;

			auto size = static_cast<std::uint32_t>(pr::physics::g_last_error.size() + 1);
			*required = size;
			if (buffer == nullptr || capacity < size)
				return PhysicsStatus::BufferTooSmall;

			memcpy(buffer, pr::physics::g_last_error.c_str(), size);
			return PhysicsStatus::Success;
		}
		catch (...)
		{
			return PhysicsStatus::InternalError;
		}
	}

	PhysicsStatus __stdcall Physics_EngineCreate(pr::physics::DllHandle context, PhysicsEngineConfig const* config, void* external_d3d12_device, PhysicsEngineHandle* engine)
	{
		return pr::physics::ApiCall([&]
		{
			auto dll = pr::physics::PinDll();
			pr::physics::LockGuard lock(dll->m_mutex);
			if (!dll->m_inits.contains(context))
				throw pr::physics::ApiException(PhysicsStatus::InvalidHandle, "DLL context handle is invalid");
			if (engine == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Engine output pointer is null");

			auto native_config = config != nullptr
				? pr::physics::ToNative(pr::physics::RequireStruct(config))
				: pr::physics::EngineConfig{};

			auto& state = *dll->m_interop;
			auto index = std::size_t{};
			for (; index != state.m_engines.size(); ++index)
			{
				if (!state.m_engines[index]->m_record)
					break;
			}
			if (index == state.m_engines.size())
				state.m_engines.push_back(std::make_unique<pr::physics::EngineSlot>());

			auto const cookie = pr::physics::AllocateEngineCookie(state);

			auto& slot = *state.m_engines[index];
			if (external_d3d12_device != nullptr)
			{
				// Engines using an explicit device share its context-owned queue while retaining independent jobs, allocators, fences, and buffers.
				auto& backend = state.GpuBackend(external_d3d12_device);
				slot.m_record = std::make_unique<pr::physics::EngineRecord>(cookie, native_config, backend.m_gpu.device(), backend.m_gpu.queue());
			}
			else
			{
				// A null device preserves the public contract that each engine owns an independent D3D12 device and queue.
				slot.m_record = std::make_unique<pr::physics::EngineRecord>(cookie, native_config, nullptr, nullptr);
			}
			*engine = pr::physics::MakeEngineHandle(index, slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_EngineDestroy(PhysicsEngineHandle engine)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);

			// Release pending CPU references without resetting command state after device removal; otherwise finish the step normally.
			auto failure = std::exception_ptr{};
			try
			{
				if (record.m_submitted_step != record.m_completed_step)
				{
					if (FAILED(record.m_engine->Device()->GetDeviceRemovedReason()))
					{
						record.m_engine->AbandonStep();
						record.m_completed_step = record.m_submitted_step;
					}
					else
					{
						pr::physics::CompleteStep(record);
					}
				}
			}
			catch (...)
			{
				failure = std::current_exception();
			}

			// Retire under the engine lock so a caller that pinned this record concurrently observes a stale
			// handle. The record itself is released when the scope unwinds, after the engine lock is dropped.
			record.m_retired = true;
			pr::physics::RetireEngineSlot(engine);

			if (failure)
				std::rethrow_exception(failure);
		});
	}

	void __stdcall Physics_EngineAbandon(PhysicsEngineHandle engine)
	{
		// Finalizer-only cleanup bypasses the public owner-thread rule while still draining pending GPU work.
		try
		{
			auto dll = pr::physics::TryPinDll();
			if (!dll)
				return;

			// Pin the record under the registry lock, then drain with only the engine lock held so an
			// abandoned engine's GPU wait does not stall other engines.
			std::shared_ptr<pr::physics::EngineRecord> record;
			{
				pr::physics::LockGuard lock(dll->m_mutex);
				auto index = pr::physics::EngineIndex(engine);
				if (index >= dll->m_interop->m_engines.size())
					return;

				auto& slot = *dll->m_interop->m_engines[index];
				if (!slot.m_record || slot.m_generation != pr::physics::EngineGeneration(engine))
					return;

				record = slot.m_record;
			}

			std::unique_lock<std::recursive_mutex> lock(record->m_lock);
			if (record->m_retired)
				return;

			if (record->m_submitted_step != record->m_completed_step)
			{
				// Finalizer cleanup can run on any thread, so it waits without resetting thread-affine
				// command recording state or unpacking results into managed-owned bodies.
				try
				{
					record->m_engine->AbandonStep();
					record->m_completed_step = record->m_submitted_step;
				}
				catch (...)
				{}
			}

			record->m_retired = true;
			pr::physics::RetireEngineSlot(engine);
		}
		catch (...)
		{}
	}

	PhysicsStatus __stdcall Physics_EngineDeviceLeaseAcquire(PhysicsEngineHandle engine, void** d3d12_device)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (d3d12_device == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Device output pointer is null");

			auto device = record.m_engine->Device();
			device->AddRef();
			*d3d12_device = device;
		});
	}

	PhysicsStatus __stdcall Physics_EngineConfigGet(PhysicsEngineHandle engine, PhysicsEngineConfig* config)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (config == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Configuration output pointer is null");

			pr::physics::FromNative(*config, record.m_engine->Config());
		});
	}

	PhysicsStatus __stdcall Physics_EngineConfigSet(PhysicsEngineHandle engine, PhysicsEngineConfig const* config)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			record.m_engine->Config(pr::physics::ToNative(pr::physics::RequireStruct(config)));
		});
	}

	PhysicsStatus __stdcall Physics_MaterialGet(PhysicsEngineHandle engine, std::int32_t material_id, PhysicsMaterial* material)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (material == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Material output pointer is null");

			pr::physics::RequireMaterialId(material_id);
			auto native = record.m_engine->Material(material_id);
			*material = PhysicsMaterial{
				.header = {sizeof(PhysicsMaterial), PHYSICS_STRUCT_VERSION},
				.id = native.m_id,
				.static_friction = native.m_friction_static,
				.normal_elasticity = native.m_elasticity_norm,
				.tangential_elasticity = native.m_elasticity_tang,
				.torsional_elasticity = native.m_elasticity_tors,
				.density = native.m_density,
			};
		});
	}

	PhysicsStatus __stdcall Physics_MaterialSet(PhysicsEngineHandle engine, PhysicsMaterial const* material)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(material);
			pr::physics::RequireMaterialId(value.id);
			record.m_engine->Material(pr::physics::Material{
				value.id,
				value.static_friction,
				value.normal_elasticity,
				value.tangential_elasticity,
				value.torsional_elasticity,
				value.density});
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateSphere(PhysicsEngineHandle engine, PhysicsSphereShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr || !(value.radius > 0.0f))
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Sphere output pointer or radius is invalid");

			auto native = pr::collision::ShapeSphere{
				value.radius,
				pr::physics::ToNative(value.common.shape_to_root),
				value.hollow != 0,
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateBox(PhysicsEngineHandle engine, PhysicsBoxShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr || value.dimensions.x <= 0 || value.dimensions.y <= 0 || value.dimensions.z <= 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Box output pointer or dimensions are invalid");

			auto dimensions = pr::physics::ToNative(value.dimensions).w0();
			auto native = pr::collision::ShapeBox{
				dimensions,
				pr::physics::ToNative(value.common.shape_to_root),
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateLine(PhysicsEngineHandle engine, PhysicsLineShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr || !(value.length > 0.0f) || value.radius < 0.0f)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Line output pointer, length, or radius is invalid");

			auto native = pr::collision::ShapeLine{
				value.length,
				value.radius,
				pr::physics::ToNative(value.common.shape_to_root),
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateTriangle(PhysicsEngineHandle engine, PhysicsTriangleShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Triangle output pointer is null");

			auto native = pr::collision::ShapeTriangle{
				pr::physics::ToNative(value.a).w1(),
				pr::physics::ToNative(value.b).w1(),
				pr::physics::ToNative(value.c).w1(),
				pr::physics::ToNative(value.common.shape_to_root),
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreatePolytope(PhysicsEngineHandle engine, PhysicsShapeCommon const* common, PhysicsVector4 const* points, std::uint32_t point_count, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeCommon(common);
			if (shape == nullptr || points == nullptr || point_count < 4)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "A polytope requires an output pointer and at least four points");

			auto vertices = std::vector<pr::v4>{};
			vertices.reserve(point_count);
			for (auto i = std::uint32_t{}; i != point_count; ++i)
				vertices.push_back(pr::physics::ToNative(points[i]).w1());

			auto data = pr::collision::BuildPolytopeFromPoints(
				vertices,
				pr::physics::ToNative(value.shape_to_root),
				value.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.flags));
			*shape = pr::physics::StoreShape(record, std::move(data));
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateCompound(PhysicsEngineHandle engine, PhysicsShapeCommon const* common, PhysicsShapeHandle const* children, std::uint32_t child_count, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeCommon(common);
			if (shape == nullptr || children == nullptr || child_count == 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "A compound requires an output pointer and at least one child shape");

			// Resolve and measure every child before touching the registry so that a rejected handle leaves
			// no partially built shape behind. Cross-engine and stale handles are rejected by the shared
			// child-handle validation, and a repeated handle is a legitimate request for a repeated child.
			auto child_records = std::vector<pr::physics::ShapeRecord*>{};
			child_records.reserve(child_count);
			auto leaf_count = 0;
			for (auto i = std::uint32_t{}; i != child_count; ++i)
			{
				auto* child = &pr::physics::RequireChild(record, children[i], record.m_shapes).first;
				leaf_count += pr::physics::CountShapeLeaves(*child->Shape());
				child_records.push_back(child);
			}

			// The GPU packer identifies a child proxy by a bounded index, so refuse an oversized compound at
			// creation rather than letting it fail later when a body first adopts it.
			if (leaf_count > pr::physics::MaxCompoundChildren)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, std::format("A compound shape may contain at most {} convex children, but this one contains {}", pr::physics::MaxCompoundChildren, leaf_count));

			// Copy each child into one location-independent blob. Declaration order is preserved because it is
			// the child identity reported back through contacts, and each child keeps its own transform,
			// material, and flags; only the frame it is expressed in changes.
			auto p2r = pr::physics::ToNative(value.shape_to_root);
			auto data = pr::byte_data<16>{};
			data.push_back<pr::collision::ShapeArray>();
			for (auto* child : child_records)
			{
				auto offset = data.size();
				data.append(child->m_data);
				pr::physics::RebaseShapeTree(data.at_byte_ofs<pr::collision::Shape>(offset), p2r);
			}

			// 'Complete' derives the blob size and bounding box, so the root transform has to be in place first.
			auto& arr = data.at_byte_ofs<pr::collision::ShapeArray>(0);
			arr.m_base.m_s2r = p2r;
			arr.m_base.m_material_id = value.material_id;
			arr.m_base.m_flags = static_cast<pr::collision::Shape::EFlags>(value.flags);
			arr.Complete(child_count);

			// Packed shape data is cached by shape address, so drop the cache before a new record can reuse the
			// address of a shape that was destroyed earlier.
			record.m_engine->ResetCaches();

			auto handle = pr::physics::StoreShape(record, std::move(data));
			auto& compound = pr::physics::RequireShape(record, handle);
			compound.m_children.assign(children, children + child_count);
			for (auto* child : child_records)
				++child->m_compound_refs;

			*shape = handle;
		});
	}

	PhysicsStatus __stdcall Physics_ShapeDestroy(PhysicsEngineHandle engine, PhysicsShapeHandle shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::PrepareChildDestroy(record);
			auto [shape_record, index] = pr::physics::RequireChild(record, shape, record.m_shapes);
			if (shape_record.m_body_refs != 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Shape is still referenced by one or more rigid bodies");
			if (shape_record.m_compound_refs != 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Shape is still referenced by one or more compound shapes");
			if (shape_record.m_articulation_refs != 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Shape is still referenced by one or more articulation links");

			// Releasing a compound releases the children it retained. The retention is only a lifetime link;
			// the compound already owns a private copy of each child's collision data.
			for (auto child : shape_record.m_children)
				--pr::physics::RequireShape(record, child).m_compound_refs;

			record.m_engine->ResetCaches();
			auto& slot = record.m_shapes[index];
			slot.m_object.reset();
			pr::physics::AdvanceGeneration(slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_BodyCreate(PhysicsEngineHandle engine, PhysicsBodyDesc const* desc, PhysicsBodyHandle* body)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(desc);
			if (body == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body output pointer is null");
			pr::physics::RequireKnownFlags(value.flags, static_cast<std::uint32_t>(PhysicsBodyFlags::Enabled) | static_cast<std::uint32_t>(PhysicsBodyFlags::Sleeping) | static_cast<std::uint32_t>(PhysicsBodyFlags::NeverSleep), "Body flags");
			pr::physics::RequireConsistentSleepFlags(value.flags, PhysicsBodyFlags::Sleeping, PhysicsBodyFlags::NeverSleep, "Body flags");
			pr::physics::RequireMotionType(value.motion_type);

			auto& shape_record = pr::physics::RequireShape(record, value.shape);
			auto body_record = std::make_unique<pr::physics::BodyRecord>();
			body_record->m_body = std::make_unique<pr::physics::RigidBody>(
				shape_record.Shape(),
				pr::physics::ToNative(value.object_to_world),
				value.motion_type == PhysicsMotionType::Dynamic && value.mass_mode == PhysicsMassMode::ExplicitInertia
					? pr::physics::ToNative(value.inertia)
					: pr::physics::Inertia::Infinite());
			if (value.motion_type == PhysicsMotionType::Dynamic)
			{
				switch (value.mass_mode)
				{
					case PhysicsMassMode::ExplicitInertia:
					{
						break;
					}
					case PhysicsMassMode::Mass:
					{
						if (!(value.mass_or_density > 0.0f))
							throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body mass must be positive");

						body_record->m_body->Shape(shape_record.Shape(), value.mass_or_density, false);
						break;
					}
					case PhysicsMassMode::Density:
					{
						if (!(value.mass_or_density > 0.0f))
							throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body density must be positive");

						body_record->m_body->Shape(shape_record.Shape(), value.mass_or_density, true);
						break;
					}
					default:
					{
						throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Unknown body mass mode");
					}
				}
			}
			body_record->m_body->MomentumWS(pr::physics::ToForce(value.momentum));
			body_record->m_body->GravityWS(pr::physics::ToNative(value.gravity));
			body_record->m_body->NeverSleep(pr::physics::HasFlag(value.flags, PhysicsBodyFlags::NeverSleep));
			body_record->m_body->Sleeping(pr::physics::HasFlag(value.flags, PhysicsBodyFlags::Sleeping));
			body_record->m_shape = value.shape;
			body_record->m_user_tag = value.user_tag;
			body_record->m_motion_type = value.motion_type;
			body_record->m_enabled = pr::physics::HasFlag(value.flags, PhysicsBodyFlags::Enabled);

			auto [slot, index] = pr::physics::AllocateSlot(record.m_bodies);
			slot.m_object = std::move(body_record);
			*body = pr::physics::MakeChildHandle(record.m_cookie, index, slot.m_generation);
			record.m_body_handles.emplace(slot.m_object->m_body.get(), *body);
			++shape_record.m_body_refs;
		});
	}

	PhysicsStatus __stdcall Physics_BodyDestroy(PhysicsEngineHandle engine, PhysicsBodyHandle body)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::PrepareChildDestroy(record);
			auto [body_record, index] = pr::physics::RequireChild(record, body, record.m_bodies);
			if (body_record.m_constraint_refs != 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body is still referenced by one or more persistent constraints");

			record.m_body_handles.erase(body_record.m_body.get());
			--pr::physics::RequireShape(record, body_record.m_shape).m_body_refs;
			auto& slot = record.m_bodies[index];
			slot.m_object.reset();
			pr::physics::AdvanceGeneration(slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_BodyStateGet(PhysicsEngineHandle engine, PhysicsBodyHandle body, PhysicsBodyState* state)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (state == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body state output pointer is null");

			pr::physics::FillBodyState(*state, body, pr::physics::RequireBody(record, body));
		});
	}

	PhysicsStatus __stdcall Physics_BodyStateSet(PhysicsEngineHandle engine, PhysicsBodyHandle body, PhysicsBodyState const* state)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(state);
			auto& body_record = pr::physics::RequireBody(record, body);
			if (value.body != 0 && value.body != body)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body state identifies a different body");
			if (value.shape != body_record.m_shape)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body shape cannot be changed through BodyStateSet");
			pr::physics::RequireKnownFlags(value.flags, static_cast<std::uint32_t>(PhysicsBodyFlags::Enabled) | static_cast<std::uint32_t>(PhysicsBodyFlags::Sleeping) | static_cast<std::uint32_t>(PhysicsBodyFlags::NeverSleep), "Body state flags");
			pr::physics::RequireConsistentSleepFlags(value.flags, PhysicsBodyFlags::Sleeping, PhysicsBodyFlags::NeverSleep, "Body state flags");
			pr::physics::RequireMotionType(value.motion_type);

			pr::physics::ApplyBodyState(body_record, value);
		});
	}

	PhysicsStatus __stdcall Physics_CommandsApply(PhysicsEngineHandle engine, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			pr::physics::ApplyCommands(record, commands, command_count);
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationCreate(PhysicsEngineHandle engine, PhysicsArticulationDesc const* desc, PhysicsArticulationLink const* links, PhysicsArticulationJoint const* joints, PhysicsArticulationHandle* articulation)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(desc);
			if (articulation == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation output pointer is null");
			if (value.link_count == 0 || links == nullptr || (value.link_count > 1 && joints == nullptr))
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation topology arrays are incomplete");
			pr::physics::RequireKnownFlags(value.flags, static_cast<std::uint32_t>(PhysicsArticulationFlags::Enabled) | static_cast<std::uint32_t>(PhysicsArticulationFlags::Sleeping) | static_cast<std::uint32_t>(PhysicsArticulationFlags::NeverSleep), "Articulation flags");
			pr::physics::RequireConsistentSleepFlags(value.flags, PhysicsArticulationFlags::Sleeping, PhysicsArticulationFlags::NeverSleep, "Articulation flags");
			pr::physics::ToNative(value.root_type);

			// Validate and translate the complete topology before the builder consumes any link.
			auto native_links = std::vector<pr::physics::ArticulationLinkDesc>{};
			auto native_joints = std::vector<pr::physics::ArticulationJointDesc>{};
			auto retained_shapes = std::vector<PhysicsShapeHandle>(value.link_count);
			native_links.reserve(value.link_count);
			native_joints.reserve(value.link_count - 1);
			for (auto link_index = std::uint32_t{}; link_index != value.link_count; ++link_index)
			{
				auto const& link = pr::physics::RequireStruct(&links[link_index]);
				if ((link_index == 0 && link.parent_index != -1) ||
					(link_index != 0 && (link.parent_index < 0 || static_cast<std::uint32_t>(link.parent_index) >= link_index)))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation links must be in parent-before-child order with root parent -1");

				native_links.push_back(pr::physics::ToNative(record, link, retained_shapes[link_index]));
				if (link_index != 0)
					native_joints.push_back(pr::physics::ToNative(pr::physics::RequireStruct(&joints[link_index - 1])));
			}

			// Consume the validated topology into one immutable native tree.
			auto builder = pr::physics::ArticulationBuilder{};
			auto native_handles = std::vector<pr::physics::LinkHandle>{};
			native_handles.reserve(value.link_count);
			switch (value.root_type)
			{
				case PhysicsArticulationRoot::Fixed:
				{
					native_handles.push_back(builder.AddFixedRoot(native_links[0], pr::physics::ToNative(value.root_to_world)));
					break;
				}
				case PhysicsArticulationRoot::Floating:
				{
					native_handles.push_back(builder.AddFloatingRoot(native_links[0], pr::physics::ToNative(value.root_to_world), pr::physics::ToMotion(value.root_velocity)));
					break;
				}
				default:
				{
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation root type is invalid");
				}
			}
			for (auto link_index = std::uint32_t{1}; link_index != value.link_count; ++link_index)
			{
				auto parent_index = static_cast<std::uint32_t>(links[link_index].parent_index);
				native_handles.push_back(builder.AddLink(native_handles[parent_index], native_joints[link_index - 1], native_links[link_index]));
			}

			auto articulation_record = std::make_unique<pr::physics::ArticulationRecord>();
			articulation_record->m_articulation = std::make_unique<pr::physics::Articulation>(builder.Build());
			articulation_record->m_shapes = retained_shapes;
			articulation_record->m_user_tag = value.user_tag;
			articulation_record->m_enabled = pr::physics::HasFlag(value.flags, PhysicsArticulationFlags::Enabled);
			articulation_record->m_articulation->NeverSleep(pr::physics::HasFlag(value.flags, PhysicsArticulationFlags::NeverSleep));
			articulation_record->m_articulation->Sleeping(pr::physics::HasFlag(value.flags, PhysicsArticulationFlags::Sleeping));

			// Publish the ABI handle only after all topology construction and allocation has succeeded.
			auto [slot, index] = pr::physics::AllocateSlot(record.m_articulations);
			auto handle = pr::physics::MakeChildHandle(record.m_cookie, index, slot.m_generation);
			slot.m_object = std::move(articulation_record);
			for (auto shape : retained_shapes)
			{
				if (shape != 0)
					++pr::physics::RequireShape(record, shape).m_articulation_refs;
			}
			*articulation = handle;
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationDestroy(PhysicsEngineHandle engine, PhysicsArticulationHandle articulation)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto [articulation_record, index] = pr::physics::RequireChild(record, articulation, record.m_articulations);
			if (articulation_record.m_constraint_refs != 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation is still referenced by one or more persistent constraints");

			for (auto shape : articulation_record.m_shapes)
			{
				if (shape != 0)
					--pr::physics::RequireShape(record, shape).m_articulation_refs;
			}
			auto& slot = record.m_articulations[index];
			slot.m_object.reset();
			pr::physics::AdvanceGeneration(slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationStateGet(PhysicsEngineHandle engine, PhysicsArticulationHandle articulation, PhysicsArticulationState* state, float* positions, float* velocities, float* accelerations, float* forces, std::uint32_t scalar_capacity, std::uint32_t* scalar_required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (state == nullptr || scalar_required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation state output pointers are incomplete");

			auto& articulation_record = pr::physics::RequireArticulation(record, articulation);
			pr::physics::FillArticulationState(*state, articulation, articulation_record);
			*scalar_required = state->joint_dof_count;
			if (scalar_capacity < *scalar_required || (*scalar_required != 0 && (positions == nullptr || velocities == nullptr || accelerations == nullptr || forces == nullptr)))
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Articulation scalar buffers are too small");

			// Flatten only non-root reduced coordinates; floating-root spatial state remains in ArticulationState.
			auto scalar_offset = std::uint32_t{};
			auto& native = *articulation_record.m_articulation;
			for (auto link_index = 1; link_index != native.LinkCount(); ++link_index)
			{
				auto link = native.LinkAt(link_index);
				auto position = native.JointPosition(link);
				auto velocity = native.JointVelocity(link);
				auto acceleration = native.JointAcceleration(link);
				auto force = native.JointForce(link);
				if (position.empty())
					continue;

				std::ranges::copy(position, positions + scalar_offset);
				std::ranges::copy(velocity, velocities + scalar_offset);
				std::ranges::copy(acceleration, accelerations + scalar_offset);
				std::ranges::copy(force, forces + scalar_offset);
				scalar_offset += static_cast<std::uint32_t>(position.size());
			}
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationStateSet(PhysicsEngineHandle engine, PhysicsArticulationHandle articulation, PhysicsArticulationState const* state, float const* positions, float const* velocities, float const* forces, std::uint32_t scalar_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(state);
			auto& articulation_record = pr::physics::RequireArticulation(record, articulation);
			pr::physics::ValidateArticulationState(value, articulation, articulation_record, positions, velocities, forces, scalar_count);
			pr::physics::ApplyArticulationState(articulation_record, value, positions, velocities, forces);
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationLinksCopy(PhysicsEngineHandle engine, PhysicsArticulationHandle articulation, PhysicsArticulationLinkState* links, std::uint32_t capacity, std::uint32_t* required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation link required-count pointer is null");

			auto& articulation_record = pr::physics::RequireArticulation(record, articulation);
			auto& native = *articulation_record.m_articulation;
			*required = static_cast<std::uint32_t>(native.LinkCount());
			if (capacity < *required || (*required != 0 && links == nullptr))
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Articulation link buffer is too small");

			// Refresh derived kinematics once, then copy every link in stable topological order.
			native.UpdateKinematics();
			for (auto link_index = std::uint32_t{}; link_index != *required; ++link_index)
			{
				auto link = native.LinkAt(static_cast<int>(link_index));
				auto parent = native.Parent(link);
				links[link_index] = PhysicsArticulationLinkState{
					.header = {sizeof(PhysicsArticulationLinkState), PHYSICS_STRUCT_VERSION},
					.articulation = articulation,
					.link_index = link_index,
					.parent_index = parent ? static_cast<std::int32_t>(parent.m_index) : -1,
					.shape = articulation_record.m_shapes[link_index],
					.link_to_world = pr::physics::FromNative(native.LinkToWorld(link)),
					.velocity = pr::physics::FromNative(native.LinkVelocity(link)),
					.acceleration = pr::physics::FromNative(native.LinkAcceleration(link)),
					.external_force = pr::physics::FromNative(native.ExternalForce(link)),
					.gravity = pr::physics::FromNative(native.GravityWS(link)),
				};
			}
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationLinkForceSet(PhysicsEngineHandle engine, PhysicsArticulationHandle articulation, std::uint32_t link_index, PhysicsSpatialVector const* force)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			if (force == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation link force pointer is null");

			auto& articulation_record = pr::physics::RequireArticulation(record, articulation);
			articulation_record.m_articulation->ExternalForce(pr::physics::RequireLink(articulation_record, link_index), pr::physics::ToForce(*force));
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationLinkForceApply(PhysicsEngineHandle engine, PhysicsArticulationHandle articulation, std::uint32_t link_index, PhysicsSpatialVector const* force)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			if (force == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation link force pointer is null");

			auto& articulation_record = pr::physics::RequireArticulation(record, articulation);
			articulation_record.m_articulation->ApplyExternalForce(pr::physics::RequireLink(articulation_record, link_index), pr::physics::ToForce(*force));
		});
	}

	PhysicsStatus __stdcall Physics_ArticulationLinkGravitySet(PhysicsEngineHandle engine, PhysicsArticulationHandle articulation, std::uint32_t link_index, PhysicsVector4 const* gravity)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			if (gravity == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Articulation link gravity pointer is null");

			auto& articulation_record = pr::physics::RequireArticulation(record, articulation);
			articulation_record.m_articulation->GravityWS(pr::physics::RequireLink(articulation_record, link_index), pr::physics::ToNative(*gravity));
		});
	}

	PhysicsStatus __stdcall Physics_ConstraintCreateD6(PhysicsEngineHandle engine, PhysicsD6Constraint const* desc, PhysicsConstraintHandle* constraint)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(desc);
			if (constraint == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Constraint output pointer is null");

			auto constraint_record = std::make_unique<pr::physics::ConstraintRecord>();
			constraint_record->m_desc = value;
			constraint_record->m_desc.header = {sizeof(PhysicsD6Constraint), PHYSICS_STRUCT_VERSION};
			constraint_record->m_desc.reserved = 0;
			auto native_desc = pr::physics::ToNative(record, value, constraint_record->m_body_refs, constraint_record->m_articulation_refs);
			auto [slot, index] = pr::physics::AllocateSlot(record.m_constraints);
			auto handle = pr::physics::MakeChildHandle(record.m_cookie, index, slot.m_generation);
			constraint_record->m_constraint = record.m_constraint_set.Add(native_desc);
			try
			{
				record.m_constraint_handles.emplace(pr::physics::NativeConstraintKey(constraint_record->m_constraint), handle);
			}
			catch (...)
			{
				record.m_constraint_set.Remove(constraint_record->m_constraint);
				throw;
			}

			slot.m_object = std::move(constraint_record);
			pr::physics::RetainConstraintReferences(record, *slot.m_object);
			*constraint = handle;
		});
	}

	PhysicsStatus __stdcall Physics_ConstraintGetD6(PhysicsEngineHandle engine, PhysicsConstraintHandle constraint, PhysicsD6Constraint* desc, std::int32_t* broken)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (desc == nullptr || broken == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Constraint output pointers are incomplete");

			auto& constraint_record = pr::physics::RequireConstraint(record, constraint);
			*desc = constraint_record.m_desc;
			*broken = record.m_constraint_set.IsBroken(constraint_record.m_constraint);
		});
	}

	PhysicsStatus __stdcall Physics_ConstraintUpdateD6(PhysicsEngineHandle engine, PhysicsConstraintHandle constraint, PhysicsD6Constraint const* desc)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(desc);
			auto& constraint_record = pr::physics::RequireConstraint(record, constraint);
			auto body_refs = std::array<PhysicsBodyHandle, 2>{};
			auto articulation_refs = std::array<PhysicsArticulationHandle, 2>{};
			auto native_desc = pr::physics::ToNative(record, value, body_refs, articulation_refs);

			record.m_constraint_set.Update(constraint_record.m_constraint, native_desc);
			pr::physics::ReleaseConstraintReferences(record, constraint_record);
			constraint_record.m_desc = value;
			constraint_record.m_desc.header = {sizeof(PhysicsD6Constraint), PHYSICS_STRUCT_VERSION};
			constraint_record.m_desc.reserved = 0;
			constraint_record.m_body_refs = body_refs;
			constraint_record.m_articulation_refs = articulation_refs;
			pr::physics::RetainConstraintReferences(record, constraint_record);
		});
	}

	PhysicsStatus __stdcall Physics_ConstraintSetEnabled(PhysicsEngineHandle engine, PhysicsConstraintHandle constraint, std::int32_t enabled)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto& constraint_record = pr::physics::RequireConstraint(record, constraint);
			record.m_constraint_set.SetEnabled(constraint_record.m_constraint, enabled != 0);
			auto flags = static_cast<std::uint32_t>(constraint_record.m_desc.flags);
			auto enabled_bit = static_cast<std::uint32_t>(PhysicsConstraintFlags::Enabled);
			constraint_record.m_desc.flags = static_cast<PhysicsConstraintFlags>(enabled != 0 ? flags | enabled_bit : flags & ~enabled_bit);
		});
	}

	PhysicsStatus __stdcall Physics_ConstraintRepair(PhysicsEngineHandle engine, PhysicsConstraintHandle constraint)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto& constraint_record = pr::physics::RequireConstraint(record, constraint);
			record.m_constraint_set.Repair(constraint_record.m_constraint);
		});
	}

	PhysicsStatus __stdcall Physics_ConstraintDestroy(PhysicsEngineHandle engine, PhysicsConstraintHandle constraint)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto [constraint_record, index] = pr::physics::RequireChild(record, constraint, record.m_constraints);
			record.m_constraint_set.Remove(constraint_record.m_constraint);
			record.m_constraint_handles.erase(pr::physics::NativeConstraintKey(constraint_record.m_constraint));
			pr::physics::ReleaseConstraintReferences(record, constraint_record);
			auto& slot = record.m_constraints[index];
			slot.m_object.reset();
			pr::physics::AdvanceGeneration(slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_BeginStep(PhysicsEngineHandle engine, float elapsed_seconds, double absolute_time_seconds, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			pr::physics::BeginStep(*scope, elapsed_seconds, absolute_time_seconds, 1, commands, command_count);
		});
	}

	PhysicsStatus __stdcall Physics_BeginStepEx(PhysicsEngineHandle engine, float elapsed_seconds, double absolute_time_seconds, std::uint32_t substep_count, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			pr::physics::BeginStep(*scope, elapsed_seconds, absolute_time_seconds, substep_count, commands, command_count);
		});
	}

	PhysicsStatus __stdcall Physics_CompleteStep(PhysicsEngineHandle engine)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			pr::physics::CompleteStep(*scope);
		});
	}

	PhysicsStatus __stdcall Physics_Step(PhysicsEngineHandle engine, float elapsed_seconds, double absolute_time_seconds, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::BeginStep(record, elapsed_seconds, absolute_time_seconds, 1, commands, command_count);
			pr::physics::CompleteStep(record);
		});
	}

	PhysicsStatus __stdcall Physics_StepEx(PhysicsEngineHandle engine, float elapsed_seconds, double absolute_time_seconds, std::uint32_t substep_count, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::BeginStep(record, elapsed_seconds, absolute_time_seconds, substep_count, commands, command_count);
			pr::physics::CompleteStep(record);
		});
	}

	PhysicsStatus __stdcall Physics_SnapshotCopy(PhysicsEngineHandle engine, PhysicsBodySnapshot* snapshots, std::uint32_t capacity, std::uint32_t* required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Snapshot required-count pointer is null");

			auto count = std::uint32_t{};
			for (auto const& slot : record.m_bodies)
				count += slot.m_object && slot.m_object->m_enabled ? 1U : 0U;

			*required = count;
			if (capacity < count || (count != 0 && snapshots == nullptr))
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Snapshot buffer is too small");

			auto output = std::uint32_t{};
			for (auto i = std::size_t{}; i != record.m_bodies.size(); ++i)
			{
				auto const& slot = record.m_bodies[i];
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				snapshots[output++] = pr::physics::MakeSnapshot(
					pr::physics::MakeChildHandle(record.m_cookie, i, slot.m_generation),
					*slot.m_object);
			}
		});
	}

	PhysicsStatus __stdcall Physics_EventsCopy(PhysicsEngineHandle engine, PhysicsEvent* events, std::uint32_t capacity, std::uint32_t* required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Event required-count pointer is null");

			*required = static_cast<std::uint32_t>(record.m_events.size());
			if (capacity < record.m_events.size() || (!record.m_events.empty() && events == nullptr))
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Event buffer is too small");

			std::ranges::copy(record.m_events, events);
		});
	}

	PhysicsStatus __stdcall Physics_DiagnosticsGet(PhysicsEngineHandle engine, PhysicsDiagnostics* diagnostics)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (diagnostics == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Diagnostics output pointer is null");

			auto const& profile = record.m_engine->LastStepProfile();
			auto const& collisions = record.m_engine->LastCollisionStats();
			auto const& features = record.m_engine->LastFeatureStats();
			auto device_removed_reason = record.m_engine->Device()->GetDeviceRemovedReason();
			auto body_count = std::ranges::count_if(record.m_bodies, [](auto const& slot) { return slot.m_object != nullptr; });
			auto shape_count = std::ranges::count_if(record.m_shapes, [](auto const& slot) { return slot.m_object != nullptr; });
			auto articulation_count = std::ranges::count_if(record.m_articulations, [](auto const& slot) { return slot.m_object != nullptr; });
			auto constraint_count = std::ranges::count_if(record.m_constraints, [](auto const& slot) { return slot.m_object != nullptr; });

			*diagnostics = PhysicsDiagnostics{
				.header = {sizeof(PhysicsDiagnostics), PHYSICS_STRUCT_VERSION},
				.profile = {
					profile.m_new_frame_ms,
					profile.m_pack_ms,
					profile.m_upload_ms,
					profile.m_external_forces_ms,
					profile.m_integrate_ms,
					profile.m_sleepwake_ms,
					profile.m_broadphase_ms,
					profile.m_collide_ms,
					profile.m_resolve_ms,
					profile.m_selective_ms,
					profile.m_sleepupdate_ms,
					profile.m_readback_ms,
					profile.m_gpu_run_ms,
					profile.m_gpu_prepare_ms,
					profile.m_gpu_execute_ms,
					profile.m_gpu_wait_ms,
					profile.m_gpu_reset_ms,
					profile.m_unpack_ms,
				},
				.submitted_step = record.m_submitted_step,
				.completed_step = record.m_completed_step,
				.state_checksum = pr::physics::StateChecksum(record),
				.body_count = static_cast<std::int32_t>(body_count),
				.shape_count = static_cast<std::int32_t>(shape_count),
				.pair_count = collisions.m_pair_count,
				.contact_count = collisions.m_contact_count,
				.max_pairs = collisions.m_max_pairs,
				.max_contacts = collisions.m_max_contacts,
				.collision_event_count = std::min(collisions.m_event_count, collisions.m_event_capacity),
				.collision_event_capacity = collisions.m_event_capacity,
				.collision_event_overflow_substep = collisions.m_event_overflow_substep,
				.step_pending = record.m_submitted_step != record.m_completed_step,
				.device_removed_reason = device_removed_reason == S_OK ? 0 : device_removed_reason,
				.articulation_count = static_cast<std::int32_t>(articulation_count),
				.constraint_count = static_cast<std::int32_t>(constraint_count),
				.constraints = pr::physics::ToAbi(features.m_constraints),
				.articulations = pr::physics::ToAbi(features.m_articulations),
				.coupled = pr::physics::ToAbi(features.m_coupled),
				.frame_output = pr::physics::ToAbi(features.m_frame_output),
				.failure = pr::physics::ToAbi(features.m_failure),
			};
		});
	}

	PhysicsStatus __stdcall Physics_CheckpointSize(PhysicsEngineHandle engine, std::uint64_t* required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::DrainPendingStep(record);
			if (required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint size output pointer is null");

			*required = pr::physics::CheckpointRequiredSize(record);
		});
	}

	PhysicsStatus __stdcall Physics_CheckpointWrite(PhysicsEngineHandle engine, void* buffer, std::uint64_t capacity, std::uint64_t* written)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::DrainPendingStep(record);
			if (written == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint written-size output pointer is null");

			auto required = pr::physics::CheckpointRequiredSize(record);
			*written = required;
			if (buffer == nullptr || capacity < required)
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Checkpoint buffer is too small");

			auto shape_count = std::ranges::count_if(record.m_shapes, [](auto const& slot) { return slot.m_object != nullptr; });
			auto body_count = std::ranges::count_if(record.m_bodies, [](auto const& slot) { return slot.m_object != nullptr; });
			auto header = pr::physics::CheckpointHeader{
				.m_magic = pr::physics::CheckpointMagic,
				.m_version = PHYSICS_CHECKPOINT_VERSION,
				.m_header_size = sizeof(pr::physics::CheckpointHeader),
				.m_total_size = required,
				.m_payload_checksum = 0,
				.m_engine_cookie = record.m_cookie,
				.m_reserved0 = 0,
				.m_shape_count = static_cast<std::uint32_t>(shape_count),
				.m_body_count = static_cast<std::uint32_t>(body_count),
				.m_reserved1 = 0,
				.m_submitted_step = record.m_submitted_step,
				.m_completed_step = record.m_completed_step,
				.m_config = {},
			};
			pr::physics::FromNative(header.m_config, record.m_engine->Config());

			auto output = static_cast<std::byte*>(buffer);
			pr::physics::WriteCheckpointValue(output, header);

			// Materials and active objects are written in stable numeric order for reproducible checkpoints.
			for (auto id = 0; id != pr::physics::Material::MaxMaterialId; ++id)
			{
				auto native = record.m_engine->Material(id);
				auto material = PhysicsMaterial{
					.header = {sizeof(PhysicsMaterial), PHYSICS_STRUCT_VERSION},
					.id = native.m_id,
					.static_friction = native.m_friction_static,
					.normal_elasticity = native.m_elasticity_norm,
					.tangential_elasticity = native.m_elasticity_tang,
					.torsional_elasticity = native.m_elasticity_tors,
					.density = native.m_density,
				};
				pr::physics::WriteCheckpointValue(output, material);
			}
			for (auto i = std::size_t{}; i != record.m_shapes.size(); ++i)
			{
				auto const& slot = record.m_shapes[i];
				if (!slot.m_object)
					continue;

				auto shape_header = pr::physics::CheckpointShape{
					.m_handle = pr::physics::MakeChildHandle(record.m_cookie, i, slot.m_generation),
					.m_size = static_cast<std::uint32_t>(slot.m_object->m_data.size()),
					.m_child_count = static_cast<std::uint32_t>(slot.m_object->m_children.size()),
				};
				pr::physics::WriteCheckpointValue(output, shape_header);
				memcpy(output, slot.m_object->m_data.data(), slot.m_object->m_data.size());
				output += slot.m_object->m_data.size();
				for (auto child : slot.m_object->m_children)
					pr::physics::WriteCheckpointValue(output, child);
			}
			for (auto i = std::size_t{}; i != record.m_bodies.size(); ++i)
			{
				auto const& slot = record.m_bodies[i];
				if (!slot.m_object)
					continue;

				auto body_entry = pr::physics::CheckpointBody{};
				pr::physics::FillBodyState(
					body_entry.m_state,
					pr::physics::MakeChildHandle(record.m_cookie, i, slot.m_generation),
					*slot.m_object);
				pr::physics::WriteCheckpointValue(output, body_entry);
			}

			auto payload = static_cast<std::byte const*>(buffer) + sizeof(pr::physics::CheckpointHeader);
			header.m_payload_checksum = pr::physics::HashBytes(
				14695981039346656037ULL,
				payload,
				static_cast<std::size_t>(required - sizeof(pr::physics::CheckpointHeader)));
			memcpy(buffer, &header, sizeof(header));
		});
	}

	PhysicsStatus __stdcall Physics_CheckpointRead(PhysicsEngineHandle engine, void const* buffer, std::uint64_t size)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			if (buffer == nullptr || size < sizeof(pr::physics::CheckpointHeader))
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint buffer is null or truncated");
			if (std::ranges::any_of(record.m_shapes, [](auto const& slot) { return slot.m_object != nullptr; }) ||
				std::ranges::any_of(record.m_bodies, [](auto const& slot) { return slot.m_object != nullptr; }) ||
				std::ranges::any_of(record.m_articulations, [](auto const& slot) { return slot.m_object != nullptr; }) ||
				std::ranges::any_of(record.m_constraints, [](auto const& slot) { return slot.m_object != nullptr; }))
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint import requires an empty engine");

			auto input = static_cast<std::byte const*>(buffer);
			auto end = input + size;
			auto header = pr::physics::ReadCheckpointValue<pr::physics::CheckpointHeader>(input, end);
			if (header.m_magic != pr::physics::CheckpointMagic ||
				header.m_version != PHYSICS_CHECKPOINT_VERSION ||
				header.m_header_size != sizeof(pr::physics::CheckpointHeader))
				throw pr::physics::ApiException(PhysicsStatus::IncompatibleVersion, "Checkpoint format or version is incompatible");
			if (header.m_total_size != size)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint size does not match its header");

			auto checksum = pr::physics::HashBytes(
				14695981039346656037ULL,
				input,
				static_cast<std::size_t>(end - input));
			if (checksum != header.m_payload_checksum)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint checksum is invalid");
			if (header.m_submitted_step != header.m_completed_step)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint cannot contain an in-flight GPU step");

			// Parse and validate the complete checkpoint into temporary storage before changing live state.
			auto materials = std::array<PhysicsMaterial, pr::physics::Material::MaxMaterialId>{};
			for (auto id = std::size_t{}; id != materials.size(); ++id)
			{
				auto& material = materials[id];
				material = pr::physics::ReadCheckpointValue<PhysicsMaterial>(input, end);
				pr::physics::RequireStruct(&material);
				pr::physics::RequireMaterialId(material.id);
				if (material.id != static_cast<std::int32_t>(id))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint material slot identity is invalid");
			}

			struct PendingShape
			{
				PhysicsShapeHandle m_handle;
				pr::byte_data<16> m_data;
				std::vector<PhysicsShapeHandle> m_children;
			};
			auto shapes = std::vector<PendingShape>{};
			shapes.reserve(header.m_shape_count);
			auto max_shape_index = std::size_t{};
			for (auto count = std::uint32_t{}; count != header.m_shape_count; ++count)
			{
				auto entry = pr::physics::ReadCheckpointValue<pr::physics::CheckpointShape>(input, end);
				if (pr::physics::ChildCookie(entry.m_handle) != header.m_engine_cookie)
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape cookie is invalid");
				if (static_cast<std::uint64_t>(end - input) < entry.m_size)
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape is truncated");

				auto data = std::span<std::byte const>{input, entry.m_size};
				pr::physics::ValidateCheckpointShape(data);
				input += entry.m_size;

				auto children = std::vector<PhysicsShapeHandle>{};
				children.reserve(entry.m_child_count);
				for (auto child_index = std::uint32_t{}; child_index != entry.m_child_count; ++child_index)
					children.push_back(pr::physics::ReadCheckpointValue<PhysicsShapeHandle>(input, end));

				shapes.push_back(PendingShape{entry.m_handle, pr::byte_data<16>{data}, std::move(children)});
				max_shape_index = std::max(max_shape_index, pr::physics::ChildIndex(entry.m_handle));
			}

			// Compound dependencies must resolve to exact shape identities in the same checkpoint.
			for (auto const& shape : shapes)
			{
				for (auto child : shape.m_children)
				{
					if (child == shape.m_handle ||
						pr::physics::ChildCookie(child) != header.m_engine_cookie ||
						std::ranges::none_of(shapes, [=](auto const& candidate) { return candidate.m_handle == child; }))
						throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound child identity is invalid");
				}
			}

			auto bodies = std::vector<PhysicsBodyState>{};
			bodies.reserve(header.m_body_count);
			auto max_body_index = std::size_t{};
			for (auto count = std::uint32_t{}; count != header.m_body_count; ++count)
			{
				auto entry = pr::physics::ReadCheckpointValue<pr::physics::CheckpointBody>(input, end);
				pr::physics::RequireStruct(&entry.m_state);
				if (pr::physics::ChildCookie(entry.m_state.body) != header.m_engine_cookie)
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint body cookie is invalid");
				pr::physics::RequireKnownFlags(entry.m_state.flags, static_cast<std::uint32_t>(PhysicsBodyFlags::Enabled) | static_cast<std::uint32_t>(PhysicsBodyFlags::Sleeping) | static_cast<std::uint32_t>(PhysicsBodyFlags::NeverSleep), "Checkpoint body flags");
				pr::physics::RequireConsistentSleepFlags(entry.m_state.flags, PhysicsBodyFlags::Sleeping, PhysicsBodyFlags::NeverSleep, "Checkpoint body flags");
				pr::physics::RequireMotionType(entry.m_state.motion_type);

				bodies.push_back(entry.m_state);
				max_body_index = std::max(max_body_index, pr::physics::ChildIndex(entry.m_state.body));
			}
			if (input != end)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains trailing data");

			// Every body and dependency must resolve before any live engine state is changed.
			for (auto i = std::size_t{}; i != shapes.size(); ++i)
			{
				if (std::ranges::any_of(std::span{shapes}.first(i), [&](auto const& candidate)
					{
						return pr::physics::ChildIndex(candidate.m_handle) == pr::physics::ChildIndex(shapes[i].m_handle);
					}))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains duplicate shape slots");
			}
			for (auto i = std::size_t{}; i != bodies.size(); ++i)
			{
				if (std::ranges::none_of(shapes, [&](auto const& shape) { return shape.m_handle == bodies[i].shape; }))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint body references an unknown shape");
				if (std::ranges::any_of(std::span{bodies}.first(i), [&](auto const& candidate)
					{
						return pr::physics::ChildIndex(candidate.body) == pr::physics::ChildIndex(bodies[i].body);
					}))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains duplicate body slots");
			}

			// Materialise every owning object and lookup table off to the side so any allocation or state-validation failure leaves the live engine untouched.
			auto imported_shapes = std::vector<pr::physics::ObjectSlot<pr::physics::ShapeRecord>>(shapes.empty() ? 0 : max_shape_index + 1);
			for (auto& value : shapes)
			{
				auto index = pr::physics::ChildIndex(value.m_handle);
				auto& slot = imported_shapes[index];
				slot.m_generation = pr::physics::ChildGeneration(value.m_handle);
				slot.m_object = std::make_unique<pr::physics::ShapeRecord>();
				slot.m_object->m_data = std::move(value.m_data);
				slot.m_object->m_children = std::move(value.m_children);
			}
			auto require_imported_shape = [&](PhysicsShapeHandle handle) -> pr::physics::ShapeRecord&
			{
				auto const index = pr::physics::ChildIndex(handle);
				if (pr::physics::ChildCookie(handle) != header.m_engine_cookie ||
					index >= imported_shapes.size() ||
					imported_shapes[index].m_generation != pr::physics::ChildGeneration(handle) ||
					!imported_shapes[index].m_object)
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape identity is inconsistent");

				return *imported_shapes[index].m_object;
			};
			for (auto const& slot : imported_shapes)
			{
				if (!slot.m_object)
					continue;

				for (auto child : slot.m_object->m_children)
					++require_imported_shape(child).m_compound_refs;
			}

			auto imported_bodies = std::vector<pr::physics::ObjectSlot<pr::physics::BodyRecord>>(bodies.empty() ? 0 : max_body_index + 1);
			auto imported_body_handles = std::unordered_map<pr::physics::RigidBody const*, PhysicsBodyHandle>{};
			imported_body_handles.reserve(bodies.size());
			for (auto const& value : bodies)
			{
				auto index = pr::physics::ChildIndex(value.body);
				auto& shape_record = require_imported_shape(value.shape);
				auto& slot = imported_bodies[index];
				slot.m_generation = pr::physics::ChildGeneration(value.body);
				slot.m_object = std::make_unique<pr::physics::BodyRecord>();
				slot.m_object->m_body = std::make_unique<pr::physics::RigidBody>(
					shape_record.Shape(),
					pr::physics::ToNative(value.object_to_world),
					pr::physics::ToNative(value.inertia));
				slot.m_object->m_shape = value.shape;
				pr::physics::ApplyBodyState(*slot.m_object, value);
				imported_body_handles.emplace(slot.m_object->m_body.get(), value.body);
				++shape_record.m_body_refs;
			}

			// Build a fresh native engine on the existing D3D device so configuration and material setup also complete before the live state changes.
			auto imported_engine = std::make_unique<pr::physics::Engine>(
				pr::physics::ToNative(pr::physics::RequireStruct(&header.m_config)),
				nullptr,
				record.m_engine->Device());
			for (auto const& value : materials)
			{
				imported_engine->Material(pr::physics::Material{
					value.id,
					value.static_friction,
					value.normal_elasticity,
					value.tangential_elasticity,
					value.torsional_elasticity,
					value.density});
			}
			record.BindEvents(*imported_engine);

			// Claim the restored namespace and commit only noexcept ownership swaps after all fallible work has succeeded.
			pr::physics::ClaimEngineCookie(record, header.m_engine_cookie);
			record.m_shapes.swap(imported_shapes);
			record.m_bodies.swap(imported_bodies);
			record.m_body_handles.swap(imported_body_handles);
			record.m_articulations.clear();
			record.m_constraints.clear();
			record.m_constraint_handles.clear();
			record.m_constraint_set = {};
			record.m_step_bodies.clear();
			record.m_step_articulations.clear();
			record.m_engine.swap(imported_engine);
			record.m_submitted_step = header.m_submitted_step;
			record.m_completed_step = header.m_completed_step;
			record.m_events.clear();
		});
	}
}
