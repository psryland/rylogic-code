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
			decltype(&Physics_BeginStep) BeginStep;
			decltype(&Physics_CompleteStep) CompleteStep;
			decltype(&Physics_Step) Step;
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
				, BeginStep(m_module.Proc<decltype(BeginStep)>("Physics_BeginStep"))
				, CompleteStep(m_module.Proc<decltype(CompleteStep)>("Physics_CompleteStep"))
				, Step(m_module.Proc<decltype(Step)>("Physics_Step"))
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
