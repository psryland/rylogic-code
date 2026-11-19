//*********************************************
// Mixed Host Process Diagnostics Tests
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/common/scope.h"
#include "pr/compute/utility/process_diagnostics.h"
#include "pr/physics/physics-dll.h"
#include "pr/view3d-12/view3d-dll.h"
#include "src/dll_test.h"

namespace pr::unittests
{
	// The isolated child leaves startup unconfigured so the physics DLL makes the one startup call.
	PRUnitTestClass(MixedHostProcessDiagnosticsTests, Serial)
	{
		PRUnitTestMethod(PhysicsBootstrapThenViewDeviceChoice, Extended)
		{
			if (RunIsolatedDllTest("MixedHostProcessDiagnosticsTests::PhysicsBootstrapThenViewDeviceChoice", "unconfigured"))
				return;

			// Reject a contaminated process rather than enabling validation after an existing device.
			if (compute::RunnerProcessDiagnostics.has_value() || GetModuleHandleW(L"physics.dll") || GetModuleHandleW(L"view3d-12.dll"))
				throw std::runtime_error("Run alone in a fresh process with -process-diagnostics:unconfigured");

			auto errors = std::vector<std::string>{};
			auto report = [](void* ctx, char const* msg, char const*, int, std::int64_t)
			{
				static_cast<std::vector<std::string>*>(ctx)->push_back(msg);
			};
			auto physics_dll = DllModule(__FILE__, L"physics.dll", L"physics.dll");
			auto view_dll = DllModule(__FILE__, L"view3d-12.dll", L"view3d-12.dll");
			auto configure = physics_dll.Proc<decltype(&Physics_ConfigureProcessDiagnostics)>("Physics_ConfigureProcessDiagnostics");
			auto initialise = physics_dll.Proc<decltype(&Physics_Initialise)>("Physics_Initialise");
			auto shutdown = physics_dll.Proc<decltype(&Physics_Shutdown)>("Physics_Shutdown");
			auto create = physics_dll.Proc<decltype(&Physics_EngineCreate)>("Physics_EngineCreate");
			auto destroy = physics_dll.Proc<decltype(&Physics_EngineDestroy)>("Physics_EngineDestroy");
			auto device_acquire = physics_dll.Proc<decltype(&Physics_EngineDeviceLeaseAcquire)>("Physics_EngineDeviceLeaseAcquire");
			auto shape_create = physics_dll.Proc<decltype(&Physics_ShapeCreateBox)>("Physics_ShapeCreateBox");
			auto body_create = physics_dll.Proc<decltype(&Physics_BodyCreate)>("Physics_BodyCreate");
			auto step = physics_dll.Proc<decltype(&Physics_Step)>("Physics_Step");
			auto view_configure = view_dll.Proc<decltype(&View3D_ConfigureDeviceDiagnostics)>("View3D_ConfigureDeviceDiagnostics");
			auto view_process = view_dll.Proc<decltype(&View3D_ConfigureProcessDiagnostics)>("View3D_ConfigureProcessDiagnostics");
			auto view_initialise = view_dll.Proc<decltype(&View3D_Initialise)>("View3D_Initialise");
			auto view_shutdown = view_dll.Proc<decltype(&View3D_Shutdown)>("View3D_Shutdown");
			auto view_acquire = view_dll.Proc<decltype(&View3D_DeviceLeaseAcquire)>("View3D_DeviceLeaseAcquire");

			// The first actual device is created after the physics-only bootstrap, with no View3D dependency.
			PR_EXPECT(configure(physics::EProcessDiagnostics::DebugLayer) == physics::EStatus::Success);
			auto context = initialise({&errors, report});
			PR_EXPECT(context != nullptr);
			auto context_cleanup = Scope<void>([&] { shutdown(context); });
			auto engine = physics::EngineHandle{};
			PR_EXPECT(create(context, nullptr, nullptr, &engine) == physics::EStatus::Success);
			auto engine_cleanup = Scope<void>([&] { destroy(engine); });
			auto identity = physics::Matrix4{.x = {1,0,0,0}, .y = {0,1,0,0}, .z = {0,0,1,0}, .w = {0,0,0,1}};
			auto box = physics::BoxShape{
				.common = {.header = {sizeof(physics::BoxShape), physics::PHYSICS_STRUCT_VERSION}, .shape_to_root = identity},
				.dimensions = {1,1,1,0},
			};
			auto shape = physics::ShapeHandle{};
			PR_EXPECT(shape_create(engine, &box, &shape) == physics::EStatus::Success);
			auto desc = physics::BodyDesc{
				.header = {sizeof(physics::BodyDesc), physics::PHYSICS_STRUCT_VERSION},
				.shape = shape, .object_to_world = identity, .gravity = {0,0,-9.8f,0},
				.motion_type = physics::EMotionType::Dynamic, .mass_mode = physics::EMassMode::Density, .mass_or_density = 1,
				.flags = physics::EBodyFlags::Enabled, .collision_layer = 1, .collision_mask = 0xFFFFFFFFU,
			};
			auto body = physics::BodyHandle{};
			PR_EXPECT(body_create(engine, &desc, &body) == physics::EStatus::Success);
			PR_EXPECT(step(engine, 1.0f / 60, 0, nullptr, 0) == physics::EStatus::Success);
			auto raw_device = static_cast<void*>(nullptr);
			PR_EXPECT(device_acquire(engine, &raw_device) == physics::EStatus::Success);
			auto device = D3DPtr<ID3D12Device>(static_cast<ID3D12Device*>(raw_device), false);
			PR_EXPECT(device != nullptr && compute::HasDebugLayer(device.get()));

			// A later renderer-local choice must not remove the live physics device.
			PR_EXPECT(view_configure(TRUE, FALSE, {&errors, report}) == TRUE);
			auto view = view_initialise({&errors, report});
			PR_EXPECT(view != nullptr);
			auto view_cleanup = Scope<void>([&] { view_shutdown(view); });
			auto view_device = D3DPtr<ID3D12Device>(static_cast<ID3D12Device*>(view_acquire(view)), false);
			PR_EXPECT(view_device != nullptr && compute::HasDebugLayer(view_device.get()));
			PR_EXPECT(errors.empty());
			PR_EXPECT(configure(physics::EProcessDiagnostics::None) == physics::EStatus::InvalidArgument);
			PR_EXPECT(view_process(view3d::EProcessDiagnostics::None, {&errors, report}) == FALSE);
			PR_EXPECT(view_configure(TRUE, FALSE, {&errors, report}) == FALSE);
			PR_EXPECT(errors.size() == 2);
			errors.clear();

			// A failed diagnostic callback cannot throw across either startup export's C boundary.
			auto reports = 0;
			auto fail_report = [](void* ctx, char const*, char const*, int, std::int64_t)
			{
				++*static_cast<int*>(ctx);
				throw std::bad_alloc{};
			};
			PR_EXPECT(view_process(view3d::EProcessDiagnostics::None, {&reports, fail_report}) == FALSE);
			PR_EXPECT(view_configure(TRUE, FALSE, {&reports, fail_report}) == FALSE);
			PR_EXPECT(reports == 2);
			PR_EXPECT(view_process(view3d::EProcessDiagnostics::None, {}) == FALSE);
			PR_EXPECT(view_configure(TRUE, FALSE, {}) == FALSE);

			// Submit real physics work with both owners alive and verify the device after teardown of the later owner.
			PR_EXPECT(step(engine, 1.0f / 60, 1.0 / 60, nullptr, 0) == physics::EStatus::Success);
			PR_EXPECT(device->GetDeviceRemovedReason() == S_OK && view_device->GetDeviceRemovedReason() == S_OK);
			view_device = nullptr;
			view_cleanup.m_dont = true;
			view_shutdown(view);
			PR_EXPECT(step(engine, 1.0f / 60, 2.0 / 60, nullptr, 0) == physics::EStatus::Success);
			PR_EXPECT(device->GetDeviceRemovedReason() == S_OK);
			device = nullptr;
			engine_cleanup.m_dont = true;
			PR_EXPECT(destroy(engine) == physics::EStatus::Success);
			context_cleanup.m_dont = true;
			shutdown(context);
			PR_EXPECT(errors.empty());
		}
	};
}
