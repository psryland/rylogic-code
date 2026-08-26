//*********************************************
// Physics DLL Contract Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include <atomic>
#include <cstring>
#include <thread>
#include <unknwn.h>
#include "pr/physics/physics-dll.h"
#include "src/dll_test.h"

namespace pr::unittests
{
	using namespace pr::physics;

	namespace
	{
		// Resolve the complete public Physics ABI from the exact source-built DLL under test.
		struct PhysicsApi
		{
			DllModule m_module;
			decltype(&Physics_Initialise) Initialise;
			decltype(&Physics_Shutdown) Shutdown;
			decltype(&Physics_ApiVersion) ApiVersion;
			decltype(&Physics_StructSize) StructSize;
			decltype(&Physics_LastError) LastError;
			decltype(&Physics_EngineCreate) EngineCreate;
			decltype(&Physics_EngineDestroy) EngineDestroy;
			decltype(&Physics_EngineAbandon) EngineAbandon;
			decltype(&Physics_EngineDeviceLeaseAcquire) EngineDeviceLeaseAcquire;
			decltype(&Physics_EngineConfigGet) EngineConfigGet;
			decltype(&Physics_EngineConfigSet) EngineConfigSet;
			decltype(&Physics_MaterialGet) MaterialGet;
			decltype(&Physics_MaterialSet) MaterialSet;
			decltype(&Physics_ShapeCreateSphere) ShapeCreateSphere;
			decltype(&Physics_ShapeCreateBox) ShapeCreateBox;
			decltype(&Physics_ShapeCreateLine) ShapeCreateLine;
			decltype(&Physics_ShapeCreateTriangle) ShapeCreateTriangle;
			decltype(&Physics_ShapeCreatePolytope) ShapeCreatePolytope;
			decltype(&Physics_ShapeCreateCompound) ShapeCreateCompound;
			decltype(&Physics_ShapeDestroy) ShapeDestroy;
			decltype(&Physics_BodyCreate) BodyCreate;
			decltype(&Physics_BodyDestroy) BodyDestroy;
			decltype(&Physics_BodyStateGet) BodyStateGet;
			decltype(&Physics_BodyStateSet) BodyStateSet;
			decltype(&Physics_CommandsApply) CommandsApply;
			decltype(&Physics_ArticulationCreate) ArticulationCreate;
			decltype(&Physics_ArticulationDestroy) ArticulationDestroy;
			decltype(&Physics_ArticulationStateGet) ArticulationStateGet;
			decltype(&Physics_ArticulationStateSet) ArticulationStateSet;
			decltype(&Physics_ArticulationLinksCopy) ArticulationLinksCopy;
			decltype(&Physics_ArticulationLinkForceSet) ArticulationLinkForceSet;
			decltype(&Physics_ArticulationLinkForceApply) ArticulationLinkForceApply;
			decltype(&Physics_ArticulationLinkGravitySet) ArticulationLinkGravitySet;
			decltype(&Physics_ConstraintCreateD6) ConstraintCreateD6;
			decltype(&Physics_ConstraintGetD6) ConstraintGetD6;
			decltype(&Physics_ConstraintUpdateD6) ConstraintUpdateD6;
			decltype(&Physics_ConstraintSetEnabled) ConstraintSetEnabled;
			decltype(&Physics_ConstraintRepair) ConstraintRepair;
			decltype(&Physics_ConstraintDestroy) ConstraintDestroy;
			decltype(&Physics_BeginStep) BeginStep;
			decltype(&Physics_BeginStepEx) BeginStepEx;
			decltype(&Physics_CompleteStep) CompleteStep;
			decltype(&Physics_Step) Step;
			decltype(&Physics_StepEx) StepEx;
			decltype(&Physics_SnapshotCopy) SnapshotCopy;
			decltype(&Physics_EventsCopy) EventsCopy;
			decltype(&Physics_DiagnosticsGet) DiagnosticsGet;
			decltype(&Physics_CheckpointSize) CheckpointSize;
			decltype(&Physics_CheckpointWrite) CheckpointWrite;
			decltype(&Physics_CheckpointRead) CheckpointRead;

			PhysicsApi()
				: m_module(__FILE__, L"physics.dll", L"physics.dll")
				, Initialise(m_module.Proc<decltype(Initialise)>("Physics_Initialise"))
				, Shutdown(m_module.Proc<decltype(Shutdown)>("Physics_Shutdown"))
				, ApiVersion(m_module.Proc<decltype(ApiVersion)>("Physics_ApiVersion"))
				, StructSize(m_module.Proc<decltype(StructSize)>("Physics_StructSize"))
				, LastError(m_module.Proc<decltype(LastError)>("Physics_LastError"))
				, EngineCreate(m_module.Proc<decltype(EngineCreate)>("Physics_EngineCreate"))
				, EngineDestroy(m_module.Proc<decltype(EngineDestroy)>("Physics_EngineDestroy"))
				, EngineAbandon(m_module.Proc<decltype(EngineAbandon)>("Physics_EngineAbandon"))
				, EngineDeviceLeaseAcquire(m_module.Proc<decltype(EngineDeviceLeaseAcquire)>("Physics_EngineDeviceLeaseAcquire"))
				, EngineConfigGet(m_module.Proc<decltype(EngineConfigGet)>("Physics_EngineConfigGet"))
				, EngineConfigSet(m_module.Proc<decltype(EngineConfigSet)>("Physics_EngineConfigSet"))
				, MaterialGet(m_module.Proc<decltype(MaterialGet)>("Physics_MaterialGet"))
				, MaterialSet(m_module.Proc<decltype(MaterialSet)>("Physics_MaterialSet"))
				, ShapeCreateSphere(m_module.Proc<decltype(ShapeCreateSphere)>("Physics_ShapeCreateSphere"))
				, ShapeCreateBox(m_module.Proc<decltype(ShapeCreateBox)>("Physics_ShapeCreateBox"))
				, ShapeCreateLine(m_module.Proc<decltype(ShapeCreateLine)>("Physics_ShapeCreateLine"))
				, ShapeCreateTriangle(m_module.Proc<decltype(ShapeCreateTriangle)>("Physics_ShapeCreateTriangle"))
				, ShapeCreatePolytope(m_module.Proc<decltype(ShapeCreatePolytope)>("Physics_ShapeCreatePolytope"))
				, ShapeCreateCompound(m_module.Proc<decltype(ShapeCreateCompound)>("Physics_ShapeCreateCompound"))
				, ShapeDestroy(m_module.Proc<decltype(ShapeDestroy)>("Physics_ShapeDestroy"))
				, BodyCreate(m_module.Proc<decltype(BodyCreate)>("Physics_BodyCreate"))
				, BodyDestroy(m_module.Proc<decltype(BodyDestroy)>("Physics_BodyDestroy"))
				, BodyStateGet(m_module.Proc<decltype(BodyStateGet)>("Physics_BodyStateGet"))
				, BodyStateSet(m_module.Proc<decltype(BodyStateSet)>("Physics_BodyStateSet"))
				, CommandsApply(m_module.Proc<decltype(CommandsApply)>("Physics_CommandsApply"))
				, ArticulationCreate(m_module.Proc<decltype(ArticulationCreate)>("Physics_ArticulationCreate"))
				, ArticulationDestroy(m_module.Proc<decltype(ArticulationDestroy)>("Physics_ArticulationDestroy"))
				, ArticulationStateGet(m_module.Proc<decltype(ArticulationStateGet)>("Physics_ArticulationStateGet"))
				, ArticulationStateSet(m_module.Proc<decltype(ArticulationStateSet)>("Physics_ArticulationStateSet"))
				, ArticulationLinksCopy(m_module.Proc<decltype(ArticulationLinksCopy)>("Physics_ArticulationLinksCopy"))
				, ArticulationLinkForceSet(m_module.Proc<decltype(ArticulationLinkForceSet)>("Physics_ArticulationLinkForceSet"))
				, ArticulationLinkForceApply(m_module.Proc<decltype(ArticulationLinkForceApply)>("Physics_ArticulationLinkForceApply"))
				, ArticulationLinkGravitySet(m_module.Proc<decltype(ArticulationLinkGravitySet)>("Physics_ArticulationLinkGravitySet"))
				, ConstraintCreateD6(m_module.Proc<decltype(ConstraintCreateD6)>("Physics_ConstraintCreateD6"))
				, ConstraintGetD6(m_module.Proc<decltype(ConstraintGetD6)>("Physics_ConstraintGetD6"))
				, ConstraintUpdateD6(m_module.Proc<decltype(ConstraintUpdateD6)>("Physics_ConstraintUpdateD6"))
				, ConstraintSetEnabled(m_module.Proc<decltype(ConstraintSetEnabled)>("Physics_ConstraintSetEnabled"))
				, ConstraintRepair(m_module.Proc<decltype(ConstraintRepair)>("Physics_ConstraintRepair"))
				, ConstraintDestroy(m_module.Proc<decltype(ConstraintDestroy)>("Physics_ConstraintDestroy"))
				, BeginStep(m_module.Proc<decltype(BeginStep)>("Physics_BeginStep"))
				, BeginStepEx(m_module.Proc<decltype(BeginStepEx)>("Physics_BeginStepEx"))
				, CompleteStep(m_module.Proc<decltype(CompleteStep)>("Physics_CompleteStep"))
				, Step(m_module.Proc<decltype(Step)>("Physics_Step"))
				, StepEx(m_module.Proc<decltype(StepEx)>("Physics_StepEx"))
				, SnapshotCopy(m_module.Proc<decltype(SnapshotCopy)>("Physics_SnapshotCopy"))
				, EventsCopy(m_module.Proc<decltype(EventsCopy)>("Physics_EventsCopy"))
				, DiagnosticsGet(m_module.Proc<decltype(DiagnosticsGet)>("Physics_DiagnosticsGet"))
				, CheckpointSize(m_module.Proc<decltype(CheckpointSize)>("Physics_CheckpointSize"))
				, CheckpointWrite(m_module.Proc<decltype(CheckpointWrite)>("Physics_CheckpointWrite"))
				, CheckpointRead(m_module.Proc<decltype(CheckpointRead)>("Physics_CheckpointRead"))
			{}
		};

		// Ignore expected diagnostics while retaining the public callback contract.
		void __stdcall SwallowError(void*, char const*, char const*, int, std::int64_t)
		{}

		// Count lifecycle diagnostics reported through the process callback.
		void __stdcall CountError(void* context, char const*, char const*, int, std::int64_t)
		{
			++*static_cast<int*>(context);
		}

		// Return a column-major identity transform in the public wire layout.
		Matrix4 Identity()
		{
			return {
				.x = {1, 0, 0, 0},
				.y = {0, 1, 0, 0},
				.z = {0, 0, 1, 0},
				.w = {0, 0, 0, 1},
			};
		}

		// Return a column-major translation transform in the public wire layout.
		Matrix4 Translation(float x, float y, float z)
		{
			auto transform = Identity();
			transform.w = {x, y, z, 1};
			return transform;
		}

		// Own one process context and engine while keeping the loaded module alive until both are released.
		struct PhysicsFixture
		{
			PhysicsApi m_api;
			DllHandle m_context;
			EngineHandle m_engine;

			PhysicsFixture()
				: PhysicsFixture(ReportErrorCB{{}, &SwallowError})
			{}
			explicit PhysicsFixture(ReportErrorCB error_cb)
				: m_api()
				, m_context(m_api.Initialise(error_cb))
				, m_engine()
			{
				PR_EXPECT(m_context != nullptr);
				PR_EXPECT(m_api.EngineCreate(m_context, nullptr, nullptr, &m_engine) == EStatus::Success);
			}
			~PhysicsFixture()
			{
				if (m_engine != 0)
					m_api.EngineDestroy(m_engine);
				if (m_context != nullptr)
					m_api.Shutdown(m_context);
			}
			PhysicsFixture(PhysicsFixture const&) = delete;
			PhysicsFixture& operator=(PhysicsFixture const&) = delete;
		};

		// Build the common shape record with an optional root-relative placement and material.
		ShapeCommon MakeCommon(Matrix4 const& shape_to_root = Identity(), std::int32_t material_id = 0, std::uint32_t flags = 0)
		{
			return {
				.header = {sizeof(ShapeCommon), PHYSICS_STRUCT_VERSION},
				.shape_to_root = shape_to_root,
				.material_id = material_id,
				.flags = flags,
			};
		}

		// Create one box through the public ABI.
		ShapeHandle MakeBox(PhysicsApi const& api, EngineHandle engine, Vector4 dimensions, Matrix4 const& shape_to_root = Identity(), std::int32_t material_id = 0)
		{
			auto desc = BoxShape{
				.common = MakeCommon(shape_to_root, material_id),
				.dimensions = dimensions,
			};
			desc.common.header.size = sizeof(desc);

			auto shape = ShapeHandle{};
			PR_EXPECT(api.ShapeCreateBox(engine, &desc, &shape) == EStatus::Success);
			return shape;
		}

		// Create one rigid body through the public ABI.
		BodyHandle MakeBody(PhysicsApi const& api, EngineHandle engine, ShapeHandle shape, Matrix4 const& object_to_world, EMotionType motion_type)
		{
			auto desc = BodyDesc{
				.header = {sizeof(BodyDesc), PHYSICS_STRUCT_VERSION},
				.shape = shape,
				.object_to_world = object_to_world,
				.motion_type = motion_type,
				.mass_mode = EMassMode::Density,
				.mass_or_density = 1.0f,
				.flags = EBodyFlags::Enabled,
			};

			auto body = BodyHandle{};
			PR_EXPECT(api.BodyCreate(engine, &desc, &body) == EStatus::Success);
			return body;
		}

		// Return a finite unit-mass inertia suitable for simple ABI articulation fixtures.
		InertiaProperties UnitInertia()
		{
			return {
				.diagonal = {1, 1, 1, 0},
				.products = {0, 0, 0, 0},
				.centre_of_mass_and_mass = {0, 0, 0, 1},
			};
		}

		// Return one unconstrained D6 coordinate with finite drive parameters and unbounded limits.
		ConstraintAxisProperties FreeConstraintAxis()
		{
			return {
				.mode = EConstraintMode::Free,
				.lower_limit = -std::numeric_limits<float>::infinity(),
				.upper_limit = +std::numeric_limits<float>::infinity(),
				.target_position = 0,
				.target_velocity = 0,
				.stiffness = 0,
				.damping = 0,
				.max_force = +std::numeric_limits<float>::infinity(),
			};
		}

		// Return a fully locked D6 descriptor between two caller-selected stable endpoints.
		D6ConstraintProperties WeldConstraint(ConstraintFrameProperties frame_a, ConstraintFrameProperties frame_b)
		{
			auto axis = FreeConstraintAxis();
			axis.mode = EConstraintMode::Locked;
			auto result = D6ConstraintProperties{
				.header = {sizeof(D6ConstraintProperties), PHYSICS_STRUCT_VERSION},
				.frame_a = frame_a,
				.frame_b = frame_b,
				.linear = {},
				.angular = {},
				.break_force = +std::numeric_limits<float>::infinity(),
				.break_torque = +std::numeric_limits<float>::infinity(),
				.flags = EConstraintFlags::Enabled,
			};
			std::ranges::fill(result.linear, axis);
			std::ranges::fill(result.angular, axis);
			return result;
		}

		// Create a two-link floating articulation with one revolute child joint.
		ArticulationHandle MakeArticulation(PhysicsApi const& api, EngineHandle engine, ShapeHandle shape)
		{
			auto links = std::array{
				ArticulationLinkProperties{
					.header = {sizeof(ArticulationLinkProperties), PHYSICS_STRUCT_VERSION},
					.shape = shape,
					.inertia = UnitInertia(),
					.shape_to_link = Identity(),
					.parent_index = -1,
					.flags = EArticulationLinkFlags::CollideSelf,
				},
				ArticulationLinkProperties{
					.header = {sizeof(ArticulationLinkProperties), PHYSICS_STRUCT_VERSION},
					.shape = shape,
					.inertia = UnitInertia(),
					.shape_to_link = Identity(),
					.parent_index = 0,
					.flags = EArticulationLinkFlags::CollideSelf,
				},
			};
			auto joint = ArticulationJointProperties{
				.header = {sizeof(ArticulationJointProperties), PHYSICS_STRUCT_VERSION},
				.joint_to_parent = Translation(0, 0, 1),
				.joint_to_child = Identity(),
				.axes = {{0, 0, 1, 0}},
				.axis_types = {EArticulationAxis::Revolute},
				.dof_count = 1,
			};
			auto desc = ArticulationDesc{
				.header = {sizeof(ArticulationDesc), PHYSICS_STRUCT_VERSION},
				.root_to_world = Translation(0, 0, 2),
				.root_velocity = {},
				.user_tag = 0xA17CULL,
				.link_count = static_cast<std::uint32_t>(links.size()),
				.root_type = EArticulationRoot::Floating,
				.flags = EArticulationFlags::Enabled,
			};
			auto articulation = ArticulationHandle{};
			PR_EXPECT(api.ArticulationCreate(engine, &desc, links.data(), &joint, &articulation) == EStatus::Success);
			return articulation;
		}

		// Export one complete opaque checkpoint into caller-owned storage.
		std::vector<std::byte> WriteCheckpoint(PhysicsApi const& api, EngineHandle engine)
		{
			auto required = std::uint64_t{};
			PR_EXPECT(api.CheckpointSize(engine, &required) == EStatus::Success);
			auto checkpoint = std::vector<std::byte>(static_cast<std::size_t>(required));
			auto written = std::uint64_t{};
			PR_EXPECT(api.CheckpointWrite(engine, checkpoint.data(), checkpoint.size(), &written) == EStatus::Success);
			checkpoint.resize(static_cast<std::size_t>(written));
			return checkpoint;
		}

		// Test-only mirror of the version-three checkpoint prefix used to construct semantically invalid but checksummed input.
		struct CheckpointHeaderV3
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
			pr::physics::Config m_config;
		};

		// Test-only mirror of each variable-width shape prefix in a version-three checkpoint.
		struct CheckpointShapeV3
		{
			ShapeHandle m_handle;
			std::uint32_t m_size;
			std::uint32_t m_child_count;
		};

		// Replace the namespace component of one stable child handle without changing its slot or generation.
		std::uint64_t WithCheckpointCookie(std::uint64_t handle, std::uint16_t cookie)
		{
			return (handle & 0x0000FFFFFFFFFFFFULL) | (std::uint64_t{cookie} << 48);
		}

		// Recompute the payload checksum after a test deliberately changes semantically meaningful fields.
		void UpdateCheckpointChecksum(std::vector<std::byte>& checkpoint)
		{
			auto header = CheckpointHeaderV3{};
			memcpy(&header, checkpoint.data(), sizeof(header));

			auto hash = std::uint64_t{14695981039346656037ULL};
			for (auto i = sizeof(header); i != checkpoint.size(); ++i)
			{
				hash ^= static_cast<std::uint8_t>(checkpoint[i]);
				hash *= 1099511628211ULL;
			}
			header.m_payload_checksum = hash;
			memcpy(checkpoint.data(), &header, sizeof(header));
		}

		// Rewrite an otherwise valid checkpoint so it claims an in-flight step while retaining a valid payload checksum.
		void SetCheckpointPending(std::vector<std::byte>& checkpoint)
		{
			if (checkpoint.size() < sizeof(CheckpointHeaderV3))
				throw std::runtime_error("Checkpoint is too small for its version-three header");

			auto header = CheckpointHeaderV3{};
			memcpy(&header, checkpoint.data(), sizeof(header));
			if (header.m_submitted_step == 0)
				throw std::runtime_error("Checkpoint must contain a completed step");

			header.m_completed_step = header.m_submitted_step - 1;
			memcpy(checkpoint.data(), &header, sizeof(header));
			UpdateCheckpointChecksum(checkpoint);
		}

		// Rewrite every serialized child identity so a restored engine can claim a selected namespace.
		void SetCheckpointCookie(std::vector<std::byte>& checkpoint, std::uint16_t cookie)
		{
			constexpr auto checkpoint_material_count_v3 = std::size_t{32};
			auto header = CheckpointHeaderV3{};
			memcpy(&header, checkpoint.data(), sizeof(header));
			header.m_engine_cookie = cookie;
			memcpy(checkpoint.data(), &header, sizeof(header));

			auto offset = sizeof(header) + checkpoint_material_count_v3 * sizeof(MaterialProperties);
			for (auto shape_index = std::uint32_t{}; shape_index != header.m_shape_count; ++shape_index)
			{
				auto shape = CheckpointShapeV3{};
				memcpy(&shape, checkpoint.data() + offset, sizeof(shape));
				shape.m_handle = WithCheckpointCookie(shape.m_handle, cookie);
				memcpy(checkpoint.data() + offset, &shape, sizeof(shape));
				offset += sizeof(shape) + shape.m_size;
				for (auto child_index = std::uint32_t{}; child_index != shape.m_child_count; ++child_index)
				{
					auto child = ShapeHandle{};
					memcpy(&child, checkpoint.data() + offset, sizeof(child));
					child = WithCheckpointCookie(child, cookie);
					memcpy(checkpoint.data() + offset, &child, sizeof(child));
					offset += sizeof(child);
				}
			}
			for (auto body_index = std::uint32_t{}; body_index != header.m_body_count; ++body_index)
			{
				auto state = BodyState{};
				memcpy(&state, checkpoint.data() + offset, sizeof(state));
				state.body = WithCheckpointCookie(state.body, cookie);
				state.shape = WithCheckpointCookie(state.shape, cookie);
				memcpy(checkpoint.data() + offset, &state, sizeof(state));
				offset += sizeof(state);
			}
			if (offset != checkpoint.size())
				throw std::runtime_error("Checkpoint test mirror did not consume the complete payload");

			UpdateCheckpointChecksum(checkpoint);
		}
	}

	// Verify ABI discovery, fixed layouts, exception containment, and every required export.
	PRUnitTestClass(PhysicsDllCoreTests)
	{
		PRUnitTestMethod(VersionsSizesAndErrors, Extended)
		{
			auto api = PhysicsApi{};
			PR_EXPECT(api.ApiVersion() == PHYSICS_API_VERSION);

			auto sizes = std::array{
				std::pair{EStructId::EngineConfig, static_cast<std::uint32_t>(sizeof(pr::physics::Config))},
				std::pair{EStructId::ShapeCommon, static_cast<std::uint32_t>(sizeof(pr::physics::ShapeCommon))},
				std::pair{EStructId::SphereShape, static_cast<std::uint32_t>(sizeof(pr::physics::SphereShape))},
				std::pair{EStructId::BoxShape, static_cast<std::uint32_t>(sizeof(pr::physics::BoxShape))},
				std::pair{EStructId::LineShape, static_cast<std::uint32_t>(sizeof(pr::physics::LineShape))},
				std::pair{EStructId::TriangleShape, static_cast<std::uint32_t>(sizeof(pr::physics::TriangleShape))},
				std::pair{EStructId::BodyDesc, static_cast<std::uint32_t>(sizeof(pr::physics::BodyDesc))},
				std::pair{EStructId::BodyState, static_cast<std::uint32_t>(sizeof(pr::physics::BodyState))},
				std::pair{EStructId::BodyCommand, static_cast<std::uint32_t>(sizeof(pr::physics::BodyCommand))},
				std::pair{EStructId::BodySnapshot, static_cast<std::uint32_t>(sizeof(pr::physics::BodySnapshot))},
				std::pair{EStructId::Event, static_cast<std::uint32_t>(sizeof(pr::physics::Event))},
				std::pair{EStructId::Diagnostics, static_cast<std::uint32_t>(sizeof(pr::physics::Diagnostics))},
				std::pair{EStructId::Material, static_cast<std::uint32_t>(sizeof(pr::physics::MaterialProperties))},
				std::pair{EStructId::ArticulationDesc, static_cast<std::uint32_t>(sizeof(pr::physics::ArticulationDesc))},
				std::pair{EStructId::ArticulationLink, static_cast<std::uint32_t>(sizeof(pr::physics::ArticulationLinkProperties))},
				std::pair{EStructId::ArticulationJoint, static_cast<std::uint32_t>(sizeof(pr::physics::ArticulationJointProperties))},
				std::pair{EStructId::ArticulationState, static_cast<std::uint32_t>(sizeof(pr::physics::ArticulationState))},
				std::pair{EStructId::ArticulationLinkState, static_cast<std::uint32_t>(sizeof(pr::physics::ArticulationLinkState))},
				std::pair{EStructId::D6Constraint, static_cast<std::uint32_t>(sizeof(pr::physics::D6ConstraintProperties))},
			};
			for (auto const& [id, expected] : sizes)
			{
				auto actual = std::uint32_t{};
				PR_EXPECT(api.StructSize(id, &actual) == EStatus::Success);
				if (actual != expected)
					TestFramework::Fail(std::format("ABI struct {} has size {}, expected {}", static_cast<int>(id), actual, expected).c_str(), __FILE__, __LINE__);
			}

			// Native exceptions stay behind the ABI and leave a retrievable diagnostic.
			PR_EXPECT(api.StructSize(static_cast<EStructId>(999), nullptr) == EStatus::InvalidArgument);
			auto required = std::uint32_t{};
			PR_EXPECT(api.LastError(nullptr, 0, &required) == EStatus::BufferTooSmall);
			auto error = std::vector<char>(required);
			PR_EXPECT(api.LastError(error.data(), static_cast<std::uint32_t>(error.size()), &required) == EStatus::Success);
			PR_EXPECT(!error.empty() && error.front() != '\0');
		}
		PRUnitTestMethod(ShapesBodiesAndState, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shapes = std::vector<ShapeHandle>{};

			auto invalid_material = MaterialProperties{
				.header = {sizeof(MaterialProperties), PHYSICS_STRUCT_VERSION},
				.id = std::numeric_limits<std::int32_t>::max(),
			};
			auto material = MaterialProperties{};
			PR_EXPECT(fix.m_api.MaterialSet(fix.m_engine, &invalid_material) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.MaterialGet(fix.m_engine, -1, &material) == EStatus::InvalidArgument);

			auto sphere_desc = SphereShape{
				.common = MakeCommon(),
				.radius = 0.5f,
			};
			sphere_desc.common.header.size = sizeof(sphere_desc);
			auto sphere = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreateSphere(fix.m_engine, &sphere_desc, &sphere) == EStatus::Success);
			shapes.push_back(sphere);
			shapes.push_back(MakeBox(fix.m_api, fix.m_engine, {1, 2, 3, 0}));

			auto line_desc = LineShape{
				.common = MakeCommon(),
				.length = 2.0f,
				.radius = 0.1f,
			};
			line_desc.common.header.size = sizeof(line_desc);
			auto line = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreateLine(fix.m_engine, &line_desc, &line) == EStatus::Success);
			shapes.push_back(line);

			auto triangle_desc = TriangleShape{
				.common = MakeCommon(),
				.a = {-1, -1, 0, 1},
				.b = {+1, -1, 0, 1},
				.c = {0, +1, 0, 1},
			};
			triangle_desc.common.header.size = sizeof(triangle_desc);
			auto triangle = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreateTriangle(fix.m_engine, &triangle_desc, &triangle) == EStatus::Success);
			shapes.push_back(triangle);

			auto points = std::array{
				Vector4{-1, -1, -1, 1},
				Vector4{+1, -1, -1, 1},
				Vector4{0, +1, -1, 1},
				Vector4{0, 0, +1, 1},
			};
			auto polytope_common = MakeCommon();
			auto polytope = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreatePolytope(fix.m_engine, &polytope_common, points.data(), static_cast<std::uint32_t>(points.size()), &polytope) == EStatus::Success);
			shapes.push_back(polytope);

			// Body state round-trips without changing its immutable shape identity.
			auto body = MakeBody(fix.m_api, fix.m_engine, sphere, Identity(), EMotionType::Dynamic);
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			state.user_tag = 0x123456789ABCDEF0ULL;
			state.force.angular = {1, 2, 3, 0};
			state.force.linear = {4, 5, 6, 0};
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);
			auto readback = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &readback) == EStatus::Success);
			PR_EXPECT(readback.user_tag == state.user_tag);
			PR_EXPECT(readback.shape == sphere);
			PR_EXPECT(std::memcmp(&readback.force, &state.force, sizeof(state.force)) == 0);

			// Contradictory sleep policy is rejected before mutating the native body.
			state.flags = static_cast<EBodyFlags>(static_cast<std::uint32_t>(EBodyFlags::Sleeping) | static_cast<std::uint32_t>(EBodyFlags::NeverSleep));
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &readback) == EStatus::Success);
			PR_EXPECT((static_cast<std::uint32_t>(readback.flags) & static_cast<std::uint32_t>(EBodyFlags::NeverSleep)) == 0);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			for (auto shape : shapes)
				PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}
		PRUnitTestMethod(HandlesThreadsStepsAndBuffers, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);

			auto config = pr::physics::Config{};
			PR_EXPECT(fix.m_api.EngineConfigGet(0, &config) == EStatus::InvalidHandle);
			PR_EXPECT(fix.m_api.BeginStep(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.BeginStep(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::StepPending);
			PR_EXPECT(fix.m_api.CompleteStep(fix.m_engine) == EStatus::Success);
			PR_EXPECT(fix.m_api.CompleteStep(fix.m_engine) == EStatus::NoStepPending);

			auto required = std::uint32_t{};
			PR_EXPECT(fix.m_api.SnapshotCopy(fix.m_engine, nullptr, 0, &required) == EStatus::BufferTooSmall);
			PR_EXPECT(required == 1);
			auto snapshots = std::vector<BodySnapshot>(required);
			PR_EXPECT(fix.m_api.SnapshotCopy(fix.m_engine, snapshots.data(), required, &required) == EStatus::Success);
			PR_EXPECT(snapshots[0].body == body);

			// Every mutable call rejects a thread other than the engine's creating OS thread.
			auto wrong_thread = EStatus::Success;
			auto worker = std::thread([&]
			{
				wrong_thread = fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0);
			});
			worker.join();
			PR_EXPECT(wrong_thread == EStatus::WrongThread);

			// The ABI rejects absolute homogeneous positions before applying any command in the batch.
			auto invalid_force = BodyCommand{
				.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
				.body = body,
				.type = ECommand::ApplyForce,
				.at = {0, 0, 0, 1},
			};
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &invalid_force, 1) == EStatus::InvalidArgument);

			auto other = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, nullptr, &other) == EStatus::Success);
			auto command = BodyCommand{
				.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
				.body = body,
				.type = ECommand::Wake,
			};
			PR_EXPECT(fix.m_api.CommandsApply(other, &command, 1) == EStatus::InvalidHandle);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &command, 1) == EStatus::StaleHandle);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(other) == EStatus::Success);
		}
		PRUnitTestMethod(ConsumedCompletionFailureLeavesEngineIdle, Extended)
		{
			auto fix = PhysicsFixture{};
			auto config = pr::physics::Config{};
			PR_EXPECT(fix.m_api.EngineConfigGet(fix.m_engine, &config) == EStatus::Success);
			config.max_collision_pairs = 1;
			PR_EXPECT(fix.m_api.EngineConfigSet(fix.m_engine, &config) == EStatus::Success);
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto bodies = std::array{
				MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic),
				MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic),
				MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic),
			};

			// Collision-capacity validation consumes the submitted native output even though the public step reports failure.
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::InternalError);
			config.max_collision_pairs = 64;
			PR_EXPECT(fix.m_api.EngineConfigSet(fix.m_engine, &config) == EStatus::Success);
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 1.0 / 60.0, nullptr, 0) == EStatus::Success);

			for (auto body : bodies)
				PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		// Contact events retain their generating substep and diagnostics expose bounded queue overflow through the ABI.
		PRUnitTestMethod(ContactSubstepsAndOverflowDiagnostics, Extended)
		{
			auto fix = PhysicsFixture{};
			auto config = pr::physics::Config{};
			PR_EXPECT(fix.m_api.EngineConfigGet(fix.m_engine, &config) == EStatus::Success);
			config.max_collision_events = 1;
			PR_EXPECT(fix.m_api.EngineConfigSet(fix.m_engine, &config) == EStatus::Success);
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto bodies = std::array{
				MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic),
				MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic),
				MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic),
			};

			PR_EXPECT(fix.m_api.StepEx(fix.m_engine, 1.0f / 60.0f, 0.0, 2, nullptr, 0) == EStatus::Success);
			auto diagnostics = Diagnostics{};
			PR_EXPECT(fix.m_api.DiagnosticsGet(fix.m_engine, &diagnostics) == EStatus::Success);
			PR_EXPECT(diagnostics.collision_event_count == 1);
			PR_EXPECT(diagnostics.collision_event_capacity == 1);
			PR_EXPECT(diagnostics.collision_event_overflow_substep == 0);

			auto required = std::uint32_t{};
			PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, nullptr, 0, &required) == EStatus::BufferTooSmall);
			auto events = std::vector<Event>(required);
			PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, events.data(), required, &required) == EStatus::Success);
			auto contact = std::ranges::find_if(events, [](Event const& event)
			{
				return event.type == EEvent::Contact;
			});
			PR_EXPECT(contact != events.end());
			if (contact != events.end())
				PR_EXPECT(contact->substep_index == 0);

			for (auto body : bodies)
				PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		PRUnitTestMethod(CheckpointAndDeviceLease, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			static_cast<void>(MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic));
			auto checkpoint = WriteCheckpoint(fix.m_api, fix.m_engine);

			void* device = nullptr;
			PR_EXPECT(fix.m_api.EngineDeviceLeaseAcquire(fix.m_engine, &device) == EStatus::Success);
			PR_EXPECT(device != nullptr);
			PR_EXPECT(fix.m_api.EngineDestroy(fix.m_engine) == EStatus::Success);
			fix.m_engine = 0;

			// The independent COM reference can seed another engine after the original owner is gone.
			auto shared = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, device, &shared) == EStatus::Success);
			PR_EXPECT(fix.m_api.Step(shared, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(shared) == EStatus::Success);
			static_cast<IUnknown*>(device)->Release();

			// Corrupt data is rejected transactionally, leaving the empty engine able to read the original.
			auto restored = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, nullptr, &restored) == EStatus::Success);
			auto corrupt = checkpoint;
			corrupt[corrupt.size() / 2] ^= std::byte{0x5A};
			PR_EXPECT(fix.m_api.CheckpointRead(restored, corrupt.data(), corrupt.size()) != EStatus::Success);
			PR_EXPECT(fix.m_api.CheckpointRead(restored, checkpoint.data(), checkpoint.size()) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(restored) == EStatus::Success);
		}
		PRUnitTestMethod(RestoredCookieRemainsUnique, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);
			auto checkpoint = WriteCheckpoint(fix.m_api, fix.m_engine);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);

			// Claim the namespace that a naive monotonic allocator would otherwise issue to the next engine.
			auto restored_cookie = static_cast<std::uint16_t>((shape >> 48) + 1);
			if (restored_cookie == 0)
				restored_cookie = 1;
			SetCheckpointCookie(checkpoint, restored_cookie);
			auto const restored_shape = WithCheckpointCookie(shape, restored_cookie);
			auto const restored_body = WithCheckpointCookie(body, restored_cookie);
			PR_EXPECT(fix.m_api.CheckpointRead(fix.m_engine, checkpoint.data(), checkpoint.size()) == EStatus::Success);

			auto other = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, nullptr, &other) == EStatus::Success);
			auto other_shape = MakeBox(fix.m_api, other, {1, 1, 1, 0});
			PR_EXPECT(fix.m_api.ShapeDestroy(other, restored_shape) == EStatus::InvalidHandle);
			PR_EXPECT(fix.m_api.ShapeDestroy(other, other_shape) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(other) == EStatus::Success);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, restored_body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, restored_shape) == EStatus::Success);
		}
	};

	// Verify public articulation and persistent-constraint ownership, state, stepping, and diagnostics.
	PRUnitTestClass(PhysicsDllConstraintTests)
	{
		PRUnitTestMethod(ArticulationLifecycleStateAndSubsteps, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.25f, 0.25f, 0.25f, 0});
			auto articulation = MakeArticulation(fix.m_api, fix.m_engine, shape);
			auto diagnostics_before = Diagnostics{};
			PR_EXPECT(fix.m_api.DiagnosticsGet(fix.m_engine, &diagnostics_before) == EStatus::Success);

			// Link-owned shapes remain alive, while scalar state uses one deterministic topological stream.
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::InvalidArgument);
			auto state = ArticulationState{};
			auto scalar_required = std::uint32_t{};
			PR_EXPECT(fix.m_api.ArticulationStateGet(fix.m_engine, articulation, &state, nullptr, nullptr, nullptr, nullptr, 0, &scalar_required) == EStatus::BufferTooSmall);
			PR_EXPECT(state.articulation == articulation);
			PR_EXPECT(state.link_count == 2);
			PR_EXPECT(scalar_required == 1);

			auto positions = std::vector<float>(scalar_required);
			auto velocities = std::vector<float>(scalar_required);
			auto accelerations = std::vector<float>(scalar_required);
			auto forces = std::vector<float>(scalar_required);
			PR_EXPECT(fix.m_api.ArticulationStateGet(fix.m_engine, articulation, &state, positions.data(), velocities.data(), accelerations.data(), forces.data(), scalar_required, &scalar_required) == EStatus::Success);
			positions[0] = 0.25f;
			velocities[0] = -0.5f;
			forces[0] = 2.0f;
			state.root_force.linear = {0, 0, -9.8f, 0};
			state.flags = static_cast<EArticulationFlags>(static_cast<std::uint32_t>(state.flags) | static_cast<std::uint32_t>(EArticulationFlags::NeverSleep));
			PR_EXPECT(fix.m_api.ArticulationStateSet(fix.m_engine, articulation, &state, positions.data(), velocities.data(), forces.data(), scalar_required) == EStatus::Success);
			auto diagnostics_after = Diagnostics{};
			PR_EXPECT(fix.m_api.DiagnosticsGet(fix.m_engine, &diagnostics_after) == EStatus::Success);
			PR_EXPECT(diagnostics_after.state_checksum != diagnostics_before.state_checksum);

			// Persistent per-link fields round-trip independently of generalized state.
			auto link_force = SpatialVector{.angular = {0, 1, 0, 0}, .linear = {1, 0, 0, 0}};
			auto gravity = Vector4{0, 0, -9.8f, 0};
			PR_EXPECT(fix.m_api.ArticulationLinkForceSet(fix.m_engine, articulation, 1, &link_force) == EStatus::Success);
			PR_EXPECT(fix.m_api.ArticulationLinkForceApply(fix.m_engine, articulation, 1, &link_force) == EStatus::Success);
			PR_EXPECT(fix.m_api.ArticulationLinkGravitySet(fix.m_engine, articulation, 1, &gravity) == EStatus::Success);
			auto link_required = std::uint32_t{};
			PR_EXPECT(fix.m_api.ArticulationLinksCopy(fix.m_engine, articulation, nullptr, 0, &link_required) == EStatus::BufferTooSmall);
			auto links = std::vector<ArticulationLinkState>(link_required);
			PR_EXPECT(fix.m_api.ArticulationLinksCopy(fix.m_engine, articulation, links.data(), link_required, &link_required) == EStatus::Success);
			PR_EXPECT(link_required == 2);
			PR_EXPECT(links[0].parent_index == -1);
			PR_EXPECT(links[1].parent_index == 0);
			PR_EXPECT(links[1].external_force.linear.x == 2.0f);
			PR_EXPECT(links[1].gravity.z == gravity.z);

			// Multiple internal substeps remain within one submission/wait/readback boundary.
			PR_EXPECT(fix.m_api.StepEx(fix.m_engine, 1.0f / 60.0f, 0.0, 2, nullptr, 0) == EStatus::Success);
			auto diagnostics = Diagnostics{};
			PR_EXPECT(fix.m_api.DiagnosticsGet(fix.m_engine, &diagnostics) == EStatus::Success);
			PR_EXPECT(diagnostics.articulation_count == 1);
			PR_EXPECT(diagnostics.articulations.articulation_count == 1);
			PR_EXPECT(diagnostics.frame_output.readback_count == 1);
			PR_EXPECT(diagnostics.failure.reason == 0);

			// Unsupported checkpoint topology is rejected explicitly rather than silently omitted.
			auto checkpoint_size = std::uint64_t{};
			PR_EXPECT(fix.m_api.CheckpointSize(fix.m_engine, &checkpoint_size) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.ArticulationDestroy(fix.m_engine, articulation) == EStatus::Success);
			PR_EXPECT(fix.m_api.ArticulationLinksCopy(fix.m_engine, articulation, nullptr, 0, &link_required) == EStatus::StaleHandle);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		PRUnitTestMethod(CoupledConstraintOwnershipAndMutation, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.25f, 0.25f, 0.25f, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 1), EMotionType::Dynamic);
			auto articulation = MakeArticulation(fix.m_api, fix.m_engine, shape);
			auto rigid_frame = ConstraintFrameProperties{
				.type = EConstraintEndpoint::RigidBody,
				.object_handle = body,
				.constraint_to_body = Identity(),
			};
			auto link_frame = ConstraintFrameProperties{
				.type = EConstraintEndpoint::ArticulationLink,
				.link_index = 1,
				.object_handle = articulation,
				.constraint_to_body = Identity(),
			};
			auto desc = WeldConstraint(rigid_frame, link_frame);
			auto constraint = PersistentConstraintHandle{};
			PR_EXPECT(fix.m_api.ConstraintCreateD6(fix.m_engine, &desc, &constraint) == EStatus::Success);

			// Referenced endpoints cannot be destroyed, and descriptor mutations preserve the stable ABI handle.
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.ArticulationDestroy(fix.m_engine, articulation) == EStatus::InvalidArgument);
			auto readback = D6ConstraintProperties{};
			auto broken = std::int32_t{};
			PR_EXPECT(fix.m_api.ConstraintGetD6(fix.m_engine, constraint, &readback, &broken) == EStatus::Success);
			PR_EXPECT(readback.frame_a.object_handle == body);
			PR_EXPECT(readback.frame_b.object_handle == articulation);
			PR_EXPECT(broken == 0);
			readback.angular[0].mode = EConstraintMode::Driven;
			readback.angular[0].target_velocity = 0.25f;
			readback.angular[0].damping = 1.0f;
			PR_EXPECT(fix.m_api.ConstraintUpdateD6(fix.m_engine, constraint, &readback) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintSetEnabled(fix.m_engine, constraint, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintSetEnabled(fix.m_engine, constraint, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintRepair(fix.m_engine, constraint) == EStatus::Success);

			// A coupled rigid/link step is represented in public feature diagnostics and keeps one readback.
			PR_EXPECT(fix.m_api.StepEx(fix.m_engine, 1.0f / 120.0f, 0.0, 2, nullptr, 0) == EStatus::Success);
			auto diagnostics = Diagnostics{};
			PR_EXPECT(fix.m_api.DiagnosticsGet(fix.m_engine, &diagnostics) == EStatus::Success);
			PR_EXPECT(diagnostics.constraint_count == 1);
			PR_EXPECT(diagnostics.constraints.declared_count == 1);
			PR_EXPECT(diagnostics.coupled.constraint_count == 1);
			PR_EXPECT(diagnostics.frame_output.readback_count == 1);

			// Cross-engine and stale endpoint identities are rejected before persistent storage changes.
			auto other = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, nullptr, &other) == EStatus::Success);
			auto other_shape = MakeBox(fix.m_api, other, {1, 1, 1, 0});
			auto other_body = MakeBody(fix.m_api, other, other_shape, Identity(), EMotionType::Dynamic);
			auto foreign = desc;
			foreign.frame_a.object_handle = other_body;
			auto rejected = PersistentConstraintHandle{};
			PR_EXPECT(fix.m_api.ConstraintCreateD6(fix.m_engine, &foreign, &rejected) == EStatus::InvalidHandle);
			PR_EXPECT(fix.m_api.EngineDestroy(other) == EStatus::Success);

			PR_EXPECT(fix.m_api.ConstraintDestroy(fix.m_engine, constraint) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintRepair(fix.m_engine, constraint) == EStatus::StaleHandle);
			PR_EXPECT(fix.m_api.ArticulationDestroy(fix.m_engine, articulation) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		PRUnitTestMethod(ConstraintBreakEventsUseStableHandles, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);
			auto body_state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &body_state) == EStatus::Success);
			body_state.momentum.linear = {10, 0, 0, 0};
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &body_state) == EStatus::Success);

			auto world_frame = ConstraintFrameProperties{
				.type = EConstraintEndpoint::World,
				.object_handle = 0,
				.constraint_to_body = Identity(),
			};
			auto body_frame = ConstraintFrameProperties{
				.type = EConstraintEndpoint::RigidBody,
				.object_handle = body,
				.constraint_to_body = Identity(),
			};
			auto desc = WeldConstraint(world_frame, body_frame);
			desc.break_force = 0.01f;
			auto constraint = PersistentConstraintHandle{};
			PR_EXPECT(fix.m_api.ConstraintCreateD6(fix.m_engine, &desc, &constraint) == EStatus::Success);
			PR_EXPECT(fix.m_api.StepEx(fix.m_engine, 1.0f / 60.0f, 0.0, 2, nullptr, 0) == EStatus::Success);

			// The edge-triggered overload event maps the native stable slot back to its ABI generation handle.
			auto required = std::uint32_t{};
			auto probe = fix.m_api.EventsCopy(fix.m_engine, nullptr, 0, &required);
			PR_EXPECT(probe == EStatus::Success || probe == EStatus::BufferTooSmall);
			auto events = std::vector<Event>(required);
			PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, events.data(), required, &required) == EStatus::Success);
			auto iter = std::ranges::find_if(events, [&](Event const& event)
			{
				return event.type == EEvent::ConstraintBreak && event.constraint == constraint;
			});
			PR_EXPECT(iter != events.end());
			if (iter != events.end())
			{
				PR_EXPECT(iter->break_force >= desc.break_force);
				PR_EXPECT(iter->substep_index >= 0 && iter->substep_index < 2);
			}

			auto readback = D6ConstraintProperties{};
			auto broken = std::int32_t{};
			PR_EXPECT(fix.m_api.ConstraintGetD6(fix.m_engine, constraint, &readback, &broken) == EStatus::Success);
			PR_EXPECT(broken != 0);
			PR_EXPECT(fix.m_api.ConstraintRepair(fix.m_engine, constraint) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintDestroy(fix.m_engine, constraint) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		PRUnitTestMethod(ZeroDofStateAndSleepPolicy, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.25f, 0.25f, 0.25f, 0});
			auto links = std::array{
				ArticulationLinkProperties{
					.header = {sizeof(ArticulationLinkProperties), PHYSICS_STRUCT_VERSION},
					.shape = shape,
					.inertia = UnitInertia(),
					.shape_to_link = Identity(),
					.parent_index = -1,
				},
				ArticulationLinkProperties{
					.header = {sizeof(ArticulationLinkProperties), PHYSICS_STRUCT_VERSION},
					.shape = shape,
					.inertia = UnitInertia(),
					.shape_to_link = Identity(),
					.parent_index = 0,
				},
			};
			auto joint = ArticulationJointProperties{
				.header = {sizeof(ArticulationJointProperties), PHYSICS_STRUCT_VERSION},
				.joint_to_parent = Translation(0, 0, 1),
				.joint_to_child = Identity(),
				.dof_count = 0,
			};
			auto desc = ArticulationDesc{
				.header = {sizeof(ArticulationDesc), PHYSICS_STRUCT_VERSION},
				.root_to_world = Identity(),
				.link_count = static_cast<std::uint32_t>(links.size()),
				.root_type = EArticulationRoot::Fixed,
				.flags = EArticulationFlags::Enabled,
			};
			auto articulation = ArticulationHandle{};
			PR_EXPECT(fix.m_api.ArticulationCreate(fix.m_engine, &desc, links.data(), &joint, &articulation) == EStatus::Success);

			// Empty scalar streams remain valid without performing pointer arithmetic on null caller buffers.
			auto state = ArticulationState{};
			auto required = std::uint32_t{};
			PR_EXPECT(fix.m_api.ArticulationStateGet(fix.m_engine, articulation, &state, nullptr, nullptr, nullptr, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0);
			PR_EXPECT(fix.m_api.ArticulationStateSet(fix.m_engine, articulation, &state, nullptr, nullptr, nullptr, 0) == EStatus::Success);

			// Contradictory sleep policy is rejected both for existing state and new topology.
			state.flags = static_cast<EArticulationFlags>(static_cast<std::uint32_t>(EArticulationFlags::Sleeping) | static_cast<std::uint32_t>(EArticulationFlags::NeverSleep));
			PR_EXPECT(fix.m_api.ArticulationStateSet(fix.m_engine, articulation, &state, nullptr, nullptr, nullptr, 0) == EStatus::InvalidArgument);
			auto rejected = ArticulationHandle{};
			desc.flags = state.flags;
			PR_EXPECT(fix.m_api.ArticulationCreate(fix.m_engine, &desc, links.data(), &joint, &rejected) == EStatus::InvalidArgument);

			PR_EXPECT(fix.m_api.ArticulationDestroy(fix.m_engine, articulation) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}
	};

	// Verify compound ownership, bounded capacity, stable identities, and checkpoint behavior through the ABI.
	PRUnitTestClass(PhysicsDllCompoundTests)
	{
		PRUnitTestMethod(RejectsMalformedForeignAndStaleChildren, Extended)
		{
			auto fix = PhysicsFixture{};
			auto common = MakeCommon();
			auto child = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto shape = ShapeHandle{};

			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, nullptr, &child, 1, &shape) == EStatus::InvalidArgument);
			auto bad_version = common;
			bad_version.header.version = PHYSICS_STRUCT_VERSION + 1;
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &bad_version, &child, 1, &shape) == EStatus::InvalidStruct);
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, nullptr, 1, &shape) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, &child, 0, &shape) == EStatus::InvalidArgument);

			auto other = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, nullptr, &other) == EStatus::Success);
			auto foreign = MakeBox(fix.m_api, other, {1, 1, 1, 0});
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, &foreign, 1, &shape) == EStatus::InvalidHandle);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, child) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, &child, 1, &shape) == EStatus::StaleHandle);
			PR_EXPECT(fix.m_api.EngineDestroy(other) == EStatus::Success);
		}
		PRUnitTestMethod(RetentionCapacityAndCheckpoint, Extended)
		{
			auto fix = PhysicsFixture{};
			auto child = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto common = MakeCommon();
			ShapeHandle children[] = {child, child};
			auto compound = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, children, 2, &compound) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, child) == EStatus::InvalidArgument);

			auto checkpoint = WriteCheckpoint(fix.m_api, fix.m_engine);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, compound) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, child) == EStatus::Success);
			PR_EXPECT(fix.m_api.CheckpointRead(fix.m_engine, checkpoint.data(), checkpoint.size()) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, child) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, compound) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, child) == EStatus::Success);

			child = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto too_many = std::vector<ShapeHandle>(PHYSICS_MAX_COMPOUND_CHILDREN + 1, child);
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, too_many.data(), static_cast<std::uint32_t>(too_many.size()), &compound) == EStatus::InvalidArgument);
		}
		PRUnitTestMethod(ContactEventsReportStableChildIdentity, Extended)
		{
			auto fix = PhysicsFixture{};
			auto ground_shape = MakeBox(fix.m_api, fix.m_engine, {100, 100, 1, 0});
			auto lhs = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0}, Translation(-1, 0, 0), 1);
			auto rhs = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0}, Translation(+1, 0, 0), 2);
			ShapeHandle children[] = {lhs, rhs};
			auto compound_shape = ShapeHandle{};
			auto common = MakeCommon();
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, children, 2, &compound_shape) == EStatus::Success);

			auto ground = MakeBody(fix.m_api, fix.m_engine, ground_shape, Identity(), EMotionType::Static);
			auto compound = MakeBody(fix.m_api, fix.m_engine, compound_shape, Translation(0, 0, 0.5f), EMotionType::Dynamic);
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);

			auto required = std::uint32_t{};
			auto probe = fix.m_api.EventsCopy(fix.m_engine, nullptr, 0, &required);
			PR_EXPECT(probe == EStatus::Success || probe == EStatus::BufferTooSmall);
			auto events = std::vector<Event>(required);
			PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, events.data(), required, &required) == EStatus::Success);

			bool observed[2] = {};
			auto contacts = 0;
			for (auto const& evt : events)
			{
				if (evt.type != EEvent::Contact)
					continue;

				++contacts;
				auto compound_child = evt.body_a == compound ? evt.child_a : evt.child_b;
				auto ground_child = evt.body_a == ground ? evt.child_a : evt.child_b;
				PR_EXPECT(evt.body_a == compound || evt.body_b == compound);
				PR_EXPECT(evt.body_a == ground || evt.body_b == ground);
				PR_EXPECT(ground_child == PHYSICS_NO_CHILD);
				PR_EXPECT(compound_child < 2U);
				if (compound_child < 2U)
					observed[compound_child] = true;
			}

			PR_EXPECT(contacts != 0);
			PR_EXPECT(observed[0] && observed[1]);
		}
		PRUnitTestMethod(CheckpointDrainsPendingStep, Extended)
		{
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);
			PR_EXPECT(fix.m_api.BeginStep(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);

			auto required = std::uint64_t{};
			PR_EXPECT(fix.m_api.CheckpointSize(fix.m_engine, &required) == EStatus::Success);
			PR_EXPECT(required != 0);
			PR_EXPECT(fix.m_api.CompleteStep(fix.m_engine) == EStatus::NoStepPending);

			PR_EXPECT(fix.m_api.BeginStep(fix.m_engine, 1.0f / 60.0f, 1.0 / 60.0, nullptr, 0) == EStatus::Success);
			auto checkpoint = WriteCheckpoint(fix.m_api, fix.m_engine);
			PR_EXPECT(fix.m_api.CompleteStep(fix.m_engine) == EStatus::NoStepPending);
			PR_EXPECT(fix.m_api.CheckpointRead(fix.m_engine, checkpoint.data(), checkpoint.size()) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);

			// A valid checksum cannot make an in-flight step restorable because no GPU run exists in the new process.
			auto pending_checkpoint = checkpoint;
			SetCheckpointPending(pending_checkpoint);
			PR_EXPECT(fix.m_api.CheckpointRead(fix.m_engine, pending_checkpoint.data(), pending_checkpoint.size()) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.CheckpointRead(fix.m_engine, checkpoint.data(), checkpoint.size()) == EStatus::Success);
		}
	};

	// Verify cleanup and separate-engine concurrency using only public lifetime and stepping calls.
	PRUnitTestClass(PhysicsDllConcurrencyTests)
	{
		PRUnitTestMethod(AbandonAndFinalShutdown, Extended)
		{
			{
				auto fix = PhysicsFixture{};
				auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
				static_cast<void>(MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic));
				PR_EXPECT(fix.m_api.BeginStep(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);

				auto worker = std::thread([&]
				{
					fix.m_api.EngineAbandon(fix.m_engine);
				});
				worker.join();
				auto config = pr::physics::Config{};
				PR_EXPECT(fix.m_api.EngineConfigGet(fix.m_engine, &config) == EStatus::StaleHandle);
				fix.m_engine = 0;
			}

			// Start a new process context so this test owns the callback whose final-shutdown diagnostic it observes.
			auto errors = 0;
			auto fix = PhysicsFixture{ReportErrorCB{{&errors}, &CountError}};
			fix.m_api.Shutdown(fix.m_context);
			PR_EXPECT(errors == 1);
			auto config = pr::physics::Config{};
			config.header = {sizeof(pr::physics::Config), PHYSICS_STRUCT_VERSION};
			PR_EXPECT(fix.m_api.EngineConfigGet(fix.m_engine, &config) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(fix.m_engine) == EStatus::Success);
			fix.m_engine = 0;
			fix.m_api.Shutdown(fix.m_context);
			fix.m_context = nullptr;
			PR_EXPECT(errors == 1);
		}
		PRUnitTestMethod(SeparateEnginesStepConcurrently, Extended)
		{
			auto api = PhysicsApi{};
			auto context = api.Initialise(ReportErrorCB{{}, &SwallowError});
			PR_EXPECT(context != nullptr);
			std::atomic_int failures = 0;
			std::atomic_int steps = 0;

			auto simulate = [&]
			{
				auto ok = [&](EStatus status)
				{
					if (status == EStatus::Success)
						return true;

					++failures;
					return false;
				};

				auto engine = EngineHandle{};
				if (!ok(api.EngineCreate(context, nullptr, nullptr, &engine)))
					return;

				auto box = BoxShape{
					.common = MakeCommon(),
					.dimensions = {1, 1, 1, 0},
				};
				box.common.header.size = sizeof(box);
				auto shape = ShapeHandle{};
				auto body = BodyHandle{};
				if (ok(api.ShapeCreateBox(engine, &box, &shape)))
				{
					auto desc = BodyDesc{
						.header = {sizeof(BodyDesc), PHYSICS_STRUCT_VERSION},
						.shape = shape,
						.object_to_world = Identity(),
						.motion_type = EMotionType::Dynamic,
						.mass_mode = EMassMode::Density,
						.mass_or_density = 1.0f,
						.flags = EBodyFlags::Enabled,
					};
					ok(api.BodyCreate(engine, &desc, &body));
				}

				for (auto i = 0; i != 20; ++i)
				{
					if (!ok(api.Step(engine, 1.0f / 60.0f, i / 60.0, nullptr, 0)))
						break;

					++steps;
				}
				ok(api.EngineDestroy(engine));
			};

			auto first = std::thread(simulate);
			auto second = std::thread(simulate);
			first.join();
			second.join();
			api.Shutdown(context);
			PR_EXPECT(failures.load() == 0);
			PR_EXPECT(steps.load() == 40);
		}
	};
}
