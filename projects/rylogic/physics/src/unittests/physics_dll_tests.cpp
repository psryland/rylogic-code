//*********************************************
// Physics DLL Contract Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include <atomic>
#include <cstring>
#include <filesystem>
#include <format>
#include <mutex>
#include <thread>
#include <unknwn.h>
#include "pr/physics/physics-dll.h"
#include "pr/physics/terrain/landscape/baseline_surface.h"
#include "pr/physics/terrain/landscape/baseline_surface.hlsli"
#include "src/dll_test.h"
#include "src/unittests/shared_gpu.h"

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
			decltype(&Physics_ShaderCacheDirectorySet) ShaderCacheDirectorySet;
			decltype(&Physics_ShaderPrecompile) ShaderPrecompile;
			decltype(&Physics_ApiVersion) ApiVersion;
			decltype(&Physics_StructSize) StructSize;
			decltype(&Physics_LastError) LastError;
			decltype(&Physics_EngineCreate) EngineCreate;
			decltype(&Physics_EngineDestroy) EngineDestroy;
			decltype(&Physics_EngineAbandon) EngineAbandon;
			decltype(&Physics_EngineDeviceLeaseAcquire) EngineDeviceLeaseAcquire;
			decltype(&Physics_EngineConfigGet) EngineConfigGet;
			decltype(&Physics_EngineConfigSet) EngineConfigSet;
			decltype(&Physics_EngineTerrainSet) EngineTerrainSet;
			decltype(&Physics_EngineCylindricalBoundarySet) EngineCylindricalBoundarySet;
			decltype(&Physics_MaterialGet) MaterialGet;
			decltype(&Physics_MaterialSet) MaterialSet;
			decltype(&Physics_ShapeCreateSphere) ShapeCreateSphere;
			decltype(&Physics_ShapeCreateBox) ShapeCreateBox;
			decltype(&Physics_ShapeCreateLine) ShapeCreateLine;
			decltype(&Physics_ShapeCreateTriangle) ShapeCreateTriangle;
			decltype(&Physics_ShapeCreateCylinder) ShapeCreateCylinder;
			decltype(&Physics_ShapeCreatePolytope) ShapeCreatePolytope;
			decltype(&Physics_ShapeCreateCompound) ShapeCreateCompound;
			decltype(&Physics_ShapeDestroy) ShapeDestroy;
			decltype(&Physics_ShapeSetTransform) ShapeSetTransform;
			decltype(&Physics_BodyCreate) BodyCreate;
			decltype(&Physics_BodyDestroy) BodyDestroy;
			decltype(&Physics_BodyStateGet) BodyStateGet;
			decltype(&Physics_BodyStateSet) BodyStateSet;
			decltype(&Physics_CommandsApply) CommandsApply;
			decltype(&Physics_BodyCollisionIgnoreSet) BodyCollisionIgnoreSet;
			decltype(&Physics_BodyCollisionIgnoreCopy) BodyCollisionIgnoreCopy;
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
			decltype(&Physics_ConstraintUpdateD6Batch) ConstraintUpdateD6Batch;
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
			decltype(&Physics_StateChecksumGet) StateChecksumGet;
			decltype(&Physics_CheckpointSize) CheckpointSize;
			decltype(&Physics_CheckpointWrite) CheckpointWrite;
			decltype(&Physics_CheckpointRead) CheckpointRead;
			decltype(&Physics_AtmosphereCreate) AtmosphereCreate;
			decltype(&Physics_AtmosphereDestroy) AtmosphereDestroy;
			decltype(&Physics_AtmosphereBeginStep) AtmosphereBeginStep;
			decltype(&Physics_AtmospherePollStep) AtmospherePollStep;
			decltype(&Physics_AtmosphereProbesCopy) AtmosphereProbesCopy;
			decltype(&Physics_AtmosphereSurfaceWindCopy) AtmosphereSurfaceWindCopy;
			decltype(&Physics_AtmosphereCompleteStep) AtmosphereCompleteStep;
			decltype(&Physics_AtmosphereFloorsSet) AtmosphereFloorsSet;
			decltype(&Physics_AtmosphereFloorTemperaturesSet) AtmosphereFloorTemperaturesSet;
			decltype(&Physics_AtmosphereOutsideAirSet) AtmosphereOutsideAirSet;
			decltype(&Physics_AtmosphereTracersCopy) AtmosphereTracersCopy;
			decltype(&Physics_AtmosphereTracersAcquire) AtmosphereTracersAcquire;
			decltype(&Physics_AtmosphereTracersRelease) AtmosphereTracersRelease;
			decltype(&Physics_AtmosphereCellStatesCopy) AtmosphereCellStatesCopy;
			decltype(&Physics_LandscapeCreate) LandscapeCreate;
			decltype(&Physics_LandscapeDestroy) LandscapeDestroy;
			decltype(&Physics_LandscapeRecipe) LandscapeRecipe;
			decltype(&Physics_LandscapeLocalRecipe) LandscapeLocalRecipe;
			decltype(&Physics_LandscapeSample) LandscapeSample;

			PhysicsApi()
				: m_module(__FILE__, L"physics.dll", L"physics.dll")
				, Initialise(m_module.Proc<decltype(Initialise)>("Physics_Initialise"))
				, Shutdown(m_module.Proc<decltype(Shutdown)>("Physics_Shutdown"))
				, ShaderCacheDirectorySet(m_module.Proc<decltype(ShaderCacheDirectorySet)>("Physics_ShaderCacheDirectorySet"))
				, ShaderPrecompile(m_module.Proc<decltype(ShaderPrecompile)>("Physics_ShaderPrecompile"))
				, ApiVersion(m_module.Proc<decltype(ApiVersion)>("Physics_ApiVersion"))
				, StructSize(m_module.Proc<decltype(StructSize)>("Physics_StructSize"))
				, LastError(m_module.Proc<decltype(LastError)>("Physics_LastError"))
				, EngineCreate(m_module.Proc<decltype(EngineCreate)>("Physics_EngineCreate"))
				, EngineDestroy(m_module.Proc<decltype(EngineDestroy)>("Physics_EngineDestroy"))
				, EngineAbandon(m_module.Proc<decltype(EngineAbandon)>("Physics_EngineAbandon"))
				, EngineDeviceLeaseAcquire(m_module.Proc<decltype(EngineDeviceLeaseAcquire)>("Physics_EngineDeviceLeaseAcquire"))
				, EngineConfigGet(m_module.Proc<decltype(EngineConfigGet)>("Physics_EngineConfigGet"))
				, EngineConfigSet(m_module.Proc<decltype(EngineConfigSet)>("Physics_EngineConfigSet"))
				, EngineTerrainSet(m_module.Proc<decltype(EngineTerrainSet)>("Physics_EngineTerrainSet"))
				, EngineCylindricalBoundarySet(m_module.Proc<decltype(EngineCylindricalBoundarySet)>("Physics_EngineCylindricalBoundarySet"))
				, MaterialGet(m_module.Proc<decltype(MaterialGet)>("Physics_MaterialGet"))
				, MaterialSet(m_module.Proc<decltype(MaterialSet)>("Physics_MaterialSet"))
				, ShapeCreateSphere(m_module.Proc<decltype(ShapeCreateSphere)>("Physics_ShapeCreateSphere"))
				, ShapeCreateBox(m_module.Proc<decltype(ShapeCreateBox)>("Physics_ShapeCreateBox"))
				, ShapeCreateLine(m_module.Proc<decltype(ShapeCreateLine)>("Physics_ShapeCreateLine"))
				, ShapeCreateTriangle(m_module.Proc<decltype(ShapeCreateTriangle)>("Physics_ShapeCreateTriangle"))
				, ShapeCreateCylinder(m_module.Proc<decltype(ShapeCreateCylinder)>("Physics_ShapeCreateCylinder"))
				, ShapeCreatePolytope(m_module.Proc<decltype(ShapeCreatePolytope)>("Physics_ShapeCreatePolytope"))
				, ShapeCreateCompound(m_module.Proc<decltype(ShapeCreateCompound)>("Physics_ShapeCreateCompound"))
				, ShapeDestroy(m_module.Proc<decltype(ShapeDestroy)>("Physics_ShapeDestroy"))
				, ShapeSetTransform(m_module.Proc<decltype(ShapeSetTransform)>("Physics_ShapeSetTransform"))
				, BodyCreate(m_module.Proc<decltype(BodyCreate)>("Physics_BodyCreate"))
				, BodyDestroy(m_module.Proc<decltype(BodyDestroy)>("Physics_BodyDestroy"))
				, BodyStateGet(m_module.Proc<decltype(BodyStateGet)>("Physics_BodyStateGet"))
				, BodyStateSet(m_module.Proc<decltype(BodyStateSet)>("Physics_BodyStateSet"))
				, CommandsApply(m_module.Proc<decltype(CommandsApply)>("Physics_CommandsApply"))
				, BodyCollisionIgnoreSet(m_module.Proc<decltype(BodyCollisionIgnoreSet)>("Physics_BodyCollisionIgnoreSet"))
				, BodyCollisionIgnoreCopy(m_module.Proc<decltype(BodyCollisionIgnoreCopy)>("Physics_BodyCollisionIgnoreCopy"))
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
				, ConstraintUpdateD6Batch(m_module.Proc<decltype(ConstraintUpdateD6Batch)>("Physics_ConstraintUpdateD6Batch"))
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
				, StateChecksumGet(m_module.Proc<decltype(StateChecksumGet)>("Physics_StateChecksumGet"))
				, CheckpointSize(m_module.Proc<decltype(CheckpointSize)>("Physics_CheckpointSize"))
				, CheckpointWrite(m_module.Proc<decltype(CheckpointWrite)>("Physics_CheckpointWrite"))
				, CheckpointRead(m_module.Proc<decltype(CheckpointRead)>("Physics_CheckpointRead"))
				, AtmosphereCreate(m_module.Proc<decltype(AtmosphereCreate)>("Physics_AtmosphereCreate"))
				, AtmosphereDestroy(m_module.Proc<decltype(AtmosphereDestroy)>("Physics_AtmosphereDestroy"))
				, AtmosphereBeginStep(m_module.Proc<decltype(AtmosphereBeginStep)>("Physics_AtmosphereBeginStep"))
				, AtmospherePollStep(m_module.Proc<decltype(AtmospherePollStep)>("Physics_AtmospherePollStep"))
				, AtmosphereProbesCopy(m_module.Proc<decltype(AtmosphereProbesCopy)>("Physics_AtmosphereProbesCopy"))
				, AtmosphereSurfaceWindCopy(m_module.Proc<decltype(AtmosphereSurfaceWindCopy)>("Physics_AtmosphereSurfaceWindCopy"))
				, AtmosphereCompleteStep(m_module.Proc<decltype(AtmosphereCompleteStep)>("Physics_AtmosphereCompleteStep"))
				, AtmosphereFloorsSet(m_module.Proc<decltype(AtmosphereFloorsSet)>("Physics_AtmosphereFloorsSet"))
				, AtmosphereFloorTemperaturesSet(m_module.Proc<decltype(AtmosphereFloorTemperaturesSet)>("Physics_AtmosphereFloorTemperaturesSet"))
				, AtmosphereOutsideAirSet(m_module.Proc<decltype(AtmosphereOutsideAirSet)>("Physics_AtmosphereOutsideAirSet"))
				, AtmosphereTracersCopy(m_module.Proc<decltype(AtmosphereTracersCopy)>("Physics_AtmosphereTracersCopy"))
				, AtmosphereTracersAcquire(m_module.Proc<decltype(AtmosphereTracersAcquire)>("Physics_AtmosphereTracersAcquire"))
				, AtmosphereTracersRelease(m_module.Proc<decltype(AtmosphereTracersRelease)>("Physics_AtmosphereTracersRelease"))
				, AtmosphereCellStatesCopy(m_module.Proc<decltype(AtmosphereCellStatesCopy)>("Physics_AtmosphereCellStatesCopy"))
				, LandscapeCreate(m_module.Proc<decltype(LandscapeCreate)>("Physics_LandscapeCreate"))
				, LandscapeDestroy(m_module.Proc<decltype(LandscapeDestroy)>("Physics_LandscapeDestroy"))
				, LandscapeRecipe(m_module.Proc<decltype(LandscapeRecipe)>("Physics_LandscapeRecipe"))
				, LandscapeLocalRecipe(m_module.Proc<decltype(LandscapeLocalRecipe)>("Physics_LandscapeLocalRecipe"))
				, LandscapeSample(m_module.Proc<decltype(LandscapeSample)>("Physics_LandscapeSample"))
			{}
		};

		// Ignore expected diagnostics while retaining the public callback contract.
		void __stdcall SwallowError(void*, char const*, char const*, int, std::int64_t)
		{}

		// Express a library terrain configuration as the equivalent ABI descriptor.
		TerrainDesc MakeTerrainDesc(terrain::landscape::BaselineSurfaceConfig const& config)
		{
			// Unused shape terms take the library defaults so only the mountain band carries real values.
			auto band = [](auto const& value)
			{
				return TerrainBand{value.m_amplitude, value.m_wavelength_m, value.m_lacunarity, value.m_persistence, 0.2, 1, value.m_octave_count, 0};
			};
			auto desc = TerrainDesc{
				.header = {sizeof(TerrainDesc), PHYSICS_STRUCT_VERSION},
				.seed = config.m_seed,
				.material_id = config.m_material_id,
				.supported_coordinate = config.m_supported_coordinate_abs_m,
				.sea_level_bias = config.m_sea_level_bias_m,
				.uplift_height = config.m_uplift_height_m,
				.mountain_base = config.m_mountain_base_height_m,
				.basin_depth = config.m_basin_depth_m,
				.basin_threshold = config.m_basin_threshold,
				.regional_base = band(config.m_regional_base),
				.region_selector = band(config.m_region_selector),
				.region_uplift = band(config.m_region_uplift),
				.domain_warp = {config.m_domain_warp.m_amplitude_m, config.m_domain_warp.m_wavelength_m, config.m_domain_warp.m_lacunarity, config.m_domain_warp.m_persistence, 0.2, 1, config.m_domain_warp.m_octave_count, 0},
				.plains = band(config.m_plains),
				.hills = band(config.m_hills),
				.mountains = band(config.m_mountains),
				.basin_selector = band(config.m_basin_selector),
			};
			desc.mountains.roundness = config.m_mountains.m_roundness;
			desc.mountains.weight_gain = config.m_mountains.m_weight_gain;
			return desc;
		}

		// Count lifecycle diagnostics reported through the process callback.
		void __stdcall CountError(void* context, char const*, char const*, int, std::int64_t)
		{
			++*static_cast<int*>(context);
		}

		// Return the long-lived external device used to avoid artificial device churn in DLL contract tests.
		void* SharedPhysicsTestDevice()
		{
			return physics::tests::SharedTestGpu().m_gpu.device();
		}

		// Retain an ordinary process context so context-owned GPU backends survive independently executed test methods.
		// Test classes run in parallel, so all access to the retaining token is serialized.
		struct SharedPhysicsContextStorage
		{
			PhysicsApi m_api;
			DllHandle m_context;
			std::mutex m_mutex;

			SharedPhysicsContextStorage()
				: m_api()
				, m_context()
				, m_mutex()
			{}
			~SharedPhysicsContextStorage()
			{
				Release();
			}
			SharedPhysicsContextStorage(SharedPhysicsContextStorage const&) = delete;
			SharedPhysicsContextStorage& operator=(SharedPhysicsContextStorage const&) = delete;

			// Add one fixture-owned initialisation token while ensuring the retaining token exists first.
			DllHandle Acquire(PhysicsApi const& api)
			{
				// Create the retaining token once; an unserialized check would let racing threads each create one and leak all but the last.
				{
					auto lock = std::lock_guard{m_mutex};
					if (m_context == nullptr)
						m_context = m_api.Initialise(ReportErrorCB{{}, &SwallowError});

					if (m_context == nullptr)
						throw std::runtime_error("Failed to retain the shared Physics DLL test context");
				}

				auto context = api.Initialise(ReportErrorCB{{}, &SwallowError});
				if (context == nullptr)
					throw std::runtime_error("Failed to acquire a Physics DLL test context");

				return context;
			}

			// Release the retaining token so final-shutdown behavior can be tested in isolation.
			void Release()
			{
				// Release the token under the same lock that creates it.
				auto lock = std::lock_guard{m_mutex};
				if (m_context == nullptr)
					return;

				m_api.Shutdown(m_context);
				m_context = nullptr;
			}
		};

		// Return the process-long context owner used by ordinary DLL contract fixtures.
		SharedPhysicsContextStorage& SharedPhysicsContext()
		{
			static auto storage = SharedPhysicsContextStorage{};
			return storage;
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
			void* m_external_device;
			EngineHandle m_engine;

			PhysicsFixture()
				: PhysicsFixture(ReportErrorCB{{}, &SwallowError}, SharedPhysicsTestDevice(), true)
			{}
			explicit PhysicsFixture(ReportErrorCB error_cb)
				: PhysicsFixture(error_cb, SharedPhysicsTestDevice(), false)
			{}
			PhysicsFixture(ReportErrorCB error_cb, void* external_device)
				: PhysicsFixture(error_cb, external_device, true)
			{}
			PhysicsFixture(ReportErrorCB error_cb, void* external_device, bool retain_process_context)
				: m_api()
				, m_context(retain_process_context ? SharedPhysicsContext().Acquire(m_api) : m_api.Initialise(error_cb))
				, m_external_device(external_device)
				, m_engine()
			{
				PR_EXPECT(m_context != nullptr);
				PR_EXPECT(m_api.EngineCreate(m_context, nullptr, m_external_device, &m_engine) == EStatus::Success);
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
				.collision_layer = 1,
				.collision_mask = 0xFFFFFFFFU,
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

		// Test-only mirror of the version-four checkpoint prefix used to construct semantically invalid but checksummed input.
		struct CheckpointHeaderV4
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
			std::uint32_t m_ignore_pair_count;
			std::uint64_t m_submitted_step;
			std::uint64_t m_completed_step;
			pr::physics::Config m_config;
		};

		// Test-only mirror of each variable-width shape prefix in a version-four checkpoint.
		struct CheckpointShapeV4
		{
			ShapeHandle m_handle;
			std::uint32_t m_size;
			std::uint32_t m_child_count;
		};

		// Test-only mirror of each body entry in a version-four checkpoint.
		struct CheckpointBodyV4
		{
			BodyState m_state;
		};

		// Replace the namespace component of one stable child handle without changing its slot or generation.
		std::uint64_t WithCheckpointCookie(std::uint64_t handle, std::uint16_t cookie)
		{
			return (handle & 0x0000FFFFFFFFFFFFULL) | (std::uint64_t{cookie} << 48);
		}

		// Recompute the payload checksum after a test deliberately changes semantically meaningful fields.
		void UpdateCheckpointChecksum(std::vector<std::byte>& checkpoint)
		{
			auto header = CheckpointHeaderV4{};
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
			if (checkpoint.size() < sizeof(CheckpointHeaderV4))
				throw std::runtime_error("Checkpoint is too small for its version-four header");

			auto header = CheckpointHeaderV4{};
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
			constexpr auto checkpoint_material_count_v4 = std::size_t{32};
			auto header = CheckpointHeaderV4{};
			memcpy(&header, checkpoint.data(), sizeof(header));
			header.m_engine_cookie = cookie;
			memcpy(checkpoint.data(), &header, sizeof(header));

			auto offset = sizeof(header) + checkpoint_material_count_v4 * sizeof(MaterialProperties);
			for (auto shape_index = std::uint32_t{}; shape_index != header.m_shape_count; ++shape_index)
			{
				auto shape = CheckpointShapeV4{};
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
				auto entry = CheckpointBodyV4{};
				memcpy(&entry, checkpoint.data() + offset, sizeof(entry));
				entry.m_state.body = WithCheckpointCookie(entry.m_state.body, cookie);
				entry.m_state.shape = WithCheckpointCookie(entry.m_state.shape, cookie);
				memcpy(checkpoint.data() + offset, &entry, sizeof(entry));
				offset += sizeof(entry);
			}
			for (auto pair_index = std::uint32_t{}; pair_index != header.m_ignore_pair_count; ++pair_index)
			{
				// Changing the cookie keeps the pair order because the cookie is the same in both handles
				auto pair = BodyPair{};
				memcpy(&pair, checkpoint.data() + offset, sizeof(pair));
				pair.body_a = WithCheckpointCookie(pair.body_a, cookie);
				pair.body_b = WithCheckpointCookie(pair.body_b, cookie);
				memcpy(checkpoint.data() + offset, &pair, sizeof(pair));
				offset += sizeof(pair);
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
				std::pair{EStructId::CylinderShape, static_cast<std::uint32_t>(sizeof(pr::physics::CylinderShape))},
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
				std::pair{EStructId::CylindricalBoundary, static_cast<std::uint32_t>(sizeof(pr::physics::CylindricalBoundaryDesc))},
				std::pair{EStructId::AtmosphereDesc, static_cast<std::uint32_t>(sizeof(pr::physics::AtmosphereDesc))},
				std::pair{EStructId::AtmosphereStep, static_cast<std::uint32_t>(sizeof(pr::physics::AtmosphereStepDesc))},
				std::pair{EStructId::AtmosphereStats, static_cast<std::uint32_t>(sizeof(pr::physics::AtmosphereStats))},
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

			// Cylinders validate their dimensions at the API boundary.
			auto cylinder_desc = CylinderShape{
				.common = MakeCommon(),
				.length = 1.0f,
				.radius = 0.25f,
			};
			cylinder_desc.common.header.size = sizeof(cylinder_desc);
			auto cylinder = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreateCylinder(fix.m_engine, &cylinder_desc, &cylinder) == EStatus::Success);
			shapes.push_back(cylinder);
			auto bad_cylinder_desc = cylinder_desc;
			bad_cylinder_desc.radius = 0.0f;
			auto bad_cylinder = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreateCylinder(fix.m_engine, &bad_cylinder_desc, &bad_cylinder) == EStatus::InvalidArgument);

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
		PRUnitTestMethod(ShapeTransformMovesCollisionGeometry, Extended)
		{
			// A box resting on static ground with its shape centred on the body origin.
			auto fix = PhysicsFixture{};
			auto ground_shape = MakeBox(fix.m_api, fix.m_engine, {10, 10, 1, 0});
			auto ground = MakeBody(fix.m_api, fix.m_engine, ground_shape, Translation(0, 0, -0.5f), EMotionType::Static);
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 0.25f), EMotionType::Dynamic);
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			state.flags = static_cast<EBodyFlags>(static_cast<std::uint32_t>(state.flags) | static_cast<std::uint32_t>(EBodyFlags::NeverSleep));
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);

			// Step under gravity, which the engine consumes each step, and return the lowest height of the body origin.
			auto lowest = [&](int steps)
			{
				auto min_z = std::numeric_limits<float>::max();
				for (auto i = 0; i != steps; ++i)
				{
					// Re-apply gravity because it only lasts for one step.
					PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
					state.gravity = {0, 0, -9.8f, 0};
					PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);
					PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);

					PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
					min_z = std::min(min_z, state.object_to_world.w.z);
				}
				return min_z;
			};
			PR_EXPECT(FEqlAbsolute(lowest(60), 0.25f, 0.02f));

			// Raising the shape inside the body lets the body origin fall about 0.5 below its old rest height before the ground stops it,
			// which proves the packed collision data follows the in-place change after it was first cached.
			auto raised = Translation(0, 0, 0.5f);
			PR_EXPECT(fix.m_api.ShapeSetTransform(fix.m_engine, shape, &raised) == EStatus::Success);
			auto min_z = lowest(60);
			PR_EXPECT(min_z < -0.2f && min_z > -0.35f);

			// Invalid requests are rejected: a null transform, a compound shape, and a change during a pending step.
			auto compound = ShapeHandle{};
			auto common = MakeCommon();
			PR_EXPECT(fix.m_api.ShapeCreateCompound(fix.m_engine, &common, &shape, 1, &compound) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeSetTransform(fix.m_engine, shape, nullptr) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.ShapeSetTransform(fix.m_engine, compound, &raised) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BeginStep(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeSetTransform(fix.m_engine, shape, &raised) == EStatus::StepPending);
			PR_EXPECT(fix.m_api.CompleteStep(fix.m_engine) == EStatus::Success);

			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, compound) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, ground) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, ground_shape) == EStatus::Success);
		}
		PRUnitTestMethod(BodyStateRoundTripsCentreOfMassInertia, Extended)
		{
			// A free body whose centre of mass is offset from its model origin.
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			state.inertia = {
				.diagonal = {0.1f, 0.2f, 0.3f, 0},
				.products = {0.01f, 0.02f, 0.03f, 0},
				.centre_of_mass_and_mass = {0.1f, -0.2f, 0.5f, 2.0f},
			};
			state.flags = static_cast<EBodyFlags>(static_cast<std::uint32_t>(state.flags) | static_cast<std::uint32_t>(EBodyFlags::NeverSleep));

			// The inertia about the centre of mass and its offset survive repeated set/get round trips unchanged.
			auto expected = state.inertia;
			for (auto i = 0; i != 2; ++i)
			{
				// Write the state back exactly as it was read.
				PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);
				PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
				PR_EXPECT(FEqlAbsolute(state.inertia.diagonal.x, expected.diagonal.x, 1e-5f) && FEqlAbsolute(state.inertia.diagonal.y, expected.diagonal.y, 1e-5f) && FEqlAbsolute(state.inertia.diagonal.z, expected.diagonal.z, 1e-5f));
				PR_EXPECT(FEqlAbsolute(state.inertia.products.x, expected.products.x, 1e-5f) && FEqlAbsolute(state.inertia.products.y, expected.products.y, 1e-5f) && FEqlAbsolute(state.inertia.products.z, expected.products.z, 1e-5f));
				PR_EXPECT(FEqlAbsolute(state.inertia.centre_of_mass_and_mass.x, expected.centre_of_mass_and_mass.x, 1e-5f));
				PR_EXPECT(FEqlAbsolute(state.inertia.centre_of_mass_and_mass.y, expected.centre_of_mass_and_mass.y, 1e-5f));
				PR_EXPECT(FEqlAbsolute(state.inertia.centre_of_mass_and_mass.z, expected.centre_of_mass_and_mass.z, 1e-5f));
				PR_EXPECT(FEqlAbsolute(state.inertia.centre_of_mass_and_mass.w, expected.centre_of_mass_and_mass.w, 1e-5f));
			}

			// Gravity acts through the centre of mass, so the falling body must not start to spin.
			for (auto i = 0; i != 30; ++i)
			{
				// Re-apply gravity because it only lasts for one step.
				PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
				state.gravity = {0, 0, -9.8f, 0};
				PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);
				PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);
			}
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(FEqlAbsolute(state.object_to_world.x.x, 1.0f, 1e-4f) && FEqlAbsolute(state.object_to_world.y.y, 1.0f, 1e-4f) && FEqlAbsolute(state.object_to_world.z.z, 1.0f, 1e-4f));
			PR_EXPECT(state.object_to_world.w.z < -1.0f);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
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
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, fix.m_external_device, &other) == EStatus::Success);
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
			auto fix = PhysicsFixture{ReportErrorCB{{}, &SwallowError}, nullptr};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			static_cast<void>(MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic));
			auto checkpoint = WriteCheckpoint(fix.m_api, fix.m_engine);

			void* device = nullptr;
			PR_EXPECT(fix.m_api.EngineDeviceLeaseAcquire(fix.m_engine, &device) == EStatus::Success);
			PR_EXPECT(device != nullptr);

			// A second null-device engine must retain an independent engine-owned GPU lifetime.
			auto isolated = EngineHandle{};
			auto isolated_device = static_cast<void*>(nullptr);
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, nullptr, &isolated) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDeviceLeaseAcquire(isolated, &isolated_device) == EStatus::Success);
			PR_EXPECT(isolated_device != nullptr);
			PR_EXPECT(fix.m_api.EngineDestroy(isolated) == EStatus::Success);
			static_cast<IUnknown*>(isolated_device)->Release();
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);

			PR_EXPECT(fix.m_api.EngineDestroy(fix.m_engine) == EStatus::Success);
			fix.m_engine = 0;

			// The independent COM reference can seed another engine after the original owner is gone.
			auto shared = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, device, &shared) == EStatus::Success);
			PR_EXPECT(fix.m_api.Step(shared, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(shared) == EStatus::Success);

			// Corrupt data is rejected transactionally, leaving the empty engine able to read the original.
			auto restored = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, device, &restored) == EStatus::Success);
			auto corrupt = checkpoint;
			corrupt[corrupt.size() / 2] ^= std::byte{0x5A};
			PR_EXPECT(fix.m_api.CheckpointRead(restored, corrupt.data(), corrupt.size()) != EStatus::Success);
			PR_EXPECT(fix.m_api.CheckpointRead(restored, checkpoint.data(), checkpoint.size()) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(restored) == EStatus::Success);
			static_cast<IUnknown*>(device)->Release();
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
			// Engines created by tests running in parallel may already hold that cookie, so try successive cookies until one restores.
			auto restored_cookie = static_cast<std::uint16_t>(shape >> 48);
			auto restored = false;
			for (auto attempt = 0; !restored && attempt != 256; ++attempt)
			{
				restored_cookie = static_cast<std::uint16_t>(restored_cookie + 1);
				if (restored_cookie == 0)
					restored_cookie = 1;

				SetCheckpointCookie(checkpoint, restored_cookie);
				restored = fix.m_api.CheckpointRead(fix.m_engine, checkpoint.data(), checkpoint.size()) == EStatus::Success;
			}
			PR_EXPECT(restored);
			auto const restored_shape = WithCheckpointCookie(shape, restored_cookie);
			auto const restored_body = WithCheckpointCookie(body, restored_cookie);

			auto other = EngineHandle{};
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, fix.m_external_device, &other) == EStatus::Success);
			auto other_shape = MakeBox(fix.m_api, other, {1, 1, 1, 0});
			PR_EXPECT(fix.m_api.ShapeDestroy(other, restored_shape) == EStatus::InvalidHandle);
			PR_EXPECT(fix.m_api.ShapeDestroy(other, other_shape) == EStatus::Success);
			PR_EXPECT(fix.m_api.EngineDestroy(other) == EStatus::Success);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, restored_body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, restored_shape) == EStatus::Success);
		}
	};

	// Verify the public kinematic motion type: creation, state round trips, command rules, and release to dynamic.
	PRUnitTestClass(PhysicsDllKinematicTests)
	{
		PRUnitTestMethod(CreateStepAndCommands, Extended)
		{
			// A kinematic box created with finite mass, with gravity set and sleeping disabled
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 2), EMotionType::Kinematic);
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Kinematic);
			PR_EXPECT(state.inertia.centre_of_mass_and_mass.w > 0.0f && state.inertia.centre_of_mass_and_mass.w < 1.0e6f);
			state.gravity = {0, 0, -9.8f, 0};
			state.flags = static_cast<EBodyFlags>(static_cast<std::uint32_t>(state.flags) | static_cast<std::uint32_t>(EBodyFlags::NeverSleep));
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);

			// Gravity does not move a kinematic body
			for (auto i = 0; i != 30; ++i)
				PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);

			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Kinematic);
			PR_EXPECT(FEqlAbsolute(state.object_to_world.w.z, 2.0f, 1e-6f));

			// Snapshots report the motion type
			auto count = std::uint32_t{1};
			auto snapshot = BodySnapshot{};
			PR_EXPECT(fix.m_api.SnapshotCopy(fix.m_engine, &snapshot, 1, &count) == EStatus::Success);
			PR_EXPECT(count == 1 && snapshot.body == body && snapshot.motion_type == EMotionType::Kinematic);

			// Momentum, force and impulse commands are rejected for kinematic bodies
			for (auto type : {ECommand::SetMomentum, ECommand::SetForce, ECommand::ApplyForce, ECommand::ApplyImpulse})
			{
				// Each command would change momentum, which a kinematic body does not have
				auto command = BodyCommand{
					.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
					.body = body,
					.type = type,
					.value = {.angular = {0, 0, 0, 0}, .linear = {1, 0, 0, 0}},
					.at = {0, 0, 0, 0},
				};
				PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &command, 1) == EStatus::InvalidArgument);
			}

			// The kinematic target command is accepted for a kinematic body, and is rejected for a dynamic body.
			// The body moves to the target during the next step, so the pose is unchanged until then.
			auto other = MakeBody(fix.m_api, fix.m_engine, shape, Translation(5, 0, 0), EMotionType::Dynamic);
			auto move = BodyCommand{
				.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
				.body = body,
				.type = ECommand::SetKinematicTarget,
				.transform = Translation(1, 0, 2),
			};
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &move, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(FEqlAbsolute(state.object_to_world.w.x, 0.0f, 1e-6f));
			move.body = other;
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &move, 1) == EStatus::InvalidArgument);

			// Kinematic bodies are created at rest
			auto desc = BodyDesc{
				.header = {sizeof(BodyDesc), PHYSICS_STRUCT_VERSION},
				.shape = shape,
				.object_to_world = Identity(),
				.momentum = {.angular = {0, 0, 0, 0}, .linear = {1, 0, 0, 0}},
				.motion_type = EMotionType::Kinematic,
				.mass_mode = EMassMode::Density,
				.mass_or_density = 1.0f,
				.flags = EBodyFlags::Enabled,
			};
			auto rejected = BodyHandle{};
			PR_EXPECT(fix.m_api.BodyCreate(fix.m_engine, &desc, &rejected) == EStatus::InvalidArgument);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, other) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}
		PRUnitTestMethod(StateSwitchKeepsMassAndReleases, Extended)
		{
			// A dynamic body with known mass
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			auto const inertia = state.inertia;
			auto const mass = inertia.centre_of_mass_and_mass.w;

			// Switch to kinematic with a twist. The body keeps its mass and has no momentum.
			state.motion_type = EMotionType::Kinematic;
			state.momentum = {.angular = {0, 0, 0, 0}, .linear = {3, 0, 0, 0}};
			state.velocity = {.angular = {0, 0, 0.5f, 0}, .linear = {1, 0, 0, 0}};
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Kinematic);
			PR_EXPECT(FEqlRelative(state.inertia.centre_of_mass_and_mass.w, mass, 1e-5f) && FEqlRelative(state.inertia.diagonal.x, inertia.diagonal.x, 1e-5f));
			PR_EXPECT(state.momentum.linear.x == 0.0f && state.momentum.angular.z == 0.0f);
			PR_EXPECT(state.velocity.linear.x == 1.0f && state.velocity.angular.z == 0.5f);

			// Release to dynamic. The new velocity comes only from the given momentum.
			state.motion_type = EMotionType::Dynamic;
			state.momentum = {.angular = {0, 0, 0, 0}, .linear = {0, 2 * mass, 0, 0}};
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Dynamic);
			PR_EXPECT(FEqlRelative(state.inertia.centre_of_mass_and_mass.w, mass, 1e-5f) && FEqlRelative(state.inertia.diagonal.x, inertia.diagonal.x, 1e-5f));
			PR_EXPECT(FEqlAbsolute(state.velocity.linear.x, 0.0f, 1e-6f) && FEqlAbsolute(state.velocity.linear.y, 2.0f, 1e-5f));
			PR_EXPECT(FEqlAbsolute(state.velocity.angular.z, 0.0f, 1e-6f));

			// A dynamic state needs a finite, positive mass
			state.inertia.centre_of_mass_and_mass.w = 0.0f;
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::InvalidArgument);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		// A kinematic target is reached exactly at the end of the step for any substep count. The body reports the derived twist as its velocity,
		// and keeps moving at that twist when no new target is given.
		PRUnitTestMethod(TargetReachedWithSubsteps, Extended)
		{
			// A target that is translated and turned about z
			auto const elapsed = 1.0f / 60.0f;
			auto const angle = 0.6f;
			auto target = Translation(1.0f, -0.5f, 2.3f);
			target.x = {std::cos(angle), std::sin(angle), 0, 0};
			target.y = {-std::sin(angle), std::cos(angle), 0, 0};

			for (auto substeps : {1U, 4U, 8U})
			{
				// A fresh kinematic box for each substep count
				auto fix = PhysicsFixture{};
				auto shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
				auto body = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 2), EMotionType::Kinematic);
				auto command = BodyCommand{
					.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
					.body = body,
					.type = ECommand::SetKinematicTarget,
					.transform = target,
				};
				PR_EXPECT(fix.m_api.StepEx(fix.m_engine, elapsed, 0.0, substeps, &command, 1) == EStatus::Success);

				// The body is at the target
				auto state = BodyState{};
				PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
				auto const& o2w = state.object_to_world;
				PR_EXPECT(FEqlAbsolute(o2w.x.x, target.x.x, 1e-5f) && FEqlAbsolute(o2w.x.y, target.x.y, 1e-5f) && FEqlAbsolute(o2w.y.x, target.y.x, 1e-5f));
				PR_EXPECT(FEqlAbsolute(o2w.w.x, 1.0f, 1e-5f) && FEqlAbsolute(o2w.w.y, -0.5f, 1e-5f) && FEqlAbsolute(o2w.w.z, 2.3f, 1e-5f));

				// The snapshot velocity is the twist that reached the target
				auto count = std::uint32_t{1};
				auto snapshot = BodySnapshot{};
				PR_EXPECT(fix.m_api.SnapshotCopy(fix.m_engine, &snapshot, 1, &count) == EStatus::Success);
				PR_EXPECT(FEqlRelative(snapshot.velocity.linear.x, 1.0f / elapsed, 1e-4f));
				PR_EXPECT(FEqlRelative(snapshot.velocity.linear.y, -0.5f / elapsed, 1e-4f));
				PR_EXPECT(FEqlRelative(snapshot.velocity.linear.z, 0.3f / elapsed, 1e-4f));
				PR_EXPECT(FEqlRelative(snapshot.velocity.angular.z, angle / elapsed, 1e-4f));
				PR_EXPECT(FEqlAbsolute(snapshot.velocity.angular.x, 0.0f, 1e-3f) && FEqlAbsolute(snapshot.velocity.angular.y, 0.0f, 1e-3f));

				// With no new target, the body keeps moving at the same twist
				PR_EXPECT(fix.m_api.StepEx(fix.m_engine, elapsed, elapsed, substeps, nullptr, 0) == EStatus::Success);
				PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
				PR_EXPECT(FEqlAbsolute(state.object_to_world.w.x, 2.0f, 1e-4f) && FEqlAbsolute(state.object_to_world.w.z, 2.6f, 1e-4f));
				PR_EXPECT(FEqlAbsolute(state.object_to_world.x.x, std::cos(2 * angle), 1e-4f) && FEqlAbsolute(state.object_to_world.x.y, std::sin(2 * angle), 1e-4f));

				PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
				PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
			}
		}

		// Friction on a moving kinematic platform carries a resting dynamic box along with it.
		PRUnitTestMethod(PlatformCarriesBoxByFriction, Extended)
		{
			// A wide kinematic platform with a dynamic box resting on its top face. The default material is frictionless and elastic, so use a grippy, inelastic one.
			auto fix = PhysicsFixture{};
			auto const material = MaterialProperties{
				.header = {sizeof(MaterialProperties), PHYSICS_STRUCT_VERSION},
				.id = 0,
				.static_friction = 0.8f,
				.normal_elasticity = 0.0f,
				.tangential_elasticity = 0.0f,
				.torsional_elasticity = 0.0f,
				.density = 1000.0f,
			};
			PR_EXPECT(fix.m_api.MaterialSet(fix.m_engine, &material) == EStatus::Success);
			auto platform_shape = MakeBox(fix.m_api, fix.m_engine, {8.0f, 8.0f, 0.5f, 0});
			auto box_shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
			auto platform = MakeBody(fix.m_api, fix.m_engine, platform_shape, Identity(), EMotionType::Kinematic);
			auto box = MakeBody(fix.m_api, fix.m_engine, box_shape, Translation(0, 0, 0.5f), EMotionType::Dynamic);

			// Give the box gravity and move the platform along +x
			auto const speed = 1.0f;
			BodyCommand const commands[] = {
				BodyCommand{
					.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
					.body = box,
					.type = ECommand::SetGravity,
					.value = {.angular = {0, 0, 0, 0}, .linear = {0, 0, -9.8f, 0}},
				},
				BodyCommand{
					.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
					.body = platform,
					.type = ECommand::SetVelocity,
					.value = {.angular = {0, 0, 0, 0}, .linear = {speed, 0, 0, 0}},
				},
			};
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, commands, 2) == EStatus::Success);
			for (auto i = 1; i != 60; ++i)
				PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, i / 60.0, nullptr, 0) == EStatus::Success);

			// The box rides on the platform at the platform's speed
			auto platform_state = BodyState{};
			auto box_state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, platform, &platform_state) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box, &box_state) == EStatus::Success);
			PR_EXPECT(FEqlAbsolute(platform_state.object_to_world.w.x, speed, 1e-4f));
			PR_EXPECT(FEqlAbsolute(box_state.velocity.linear.x, speed, 0.05f));
			PR_EXPECT(FEqlAbsolute(box_state.object_to_world.w.z, 0.5f, 0.02f));
			PR_EXPECT(box_state.object_to_world.w.x > 0.5f * speed);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, platform) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, box_shape) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, platform_shape) == EStatus::Success);
		}

		// The SetMotionType command switches a body between motion types, keeping its mass, and later commands in the same batch see the new type.
		PRUnitTestMethod(SetMotionTypeCommand, Extended)
		{
			// A dynamic box with known mass
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
			auto body = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			auto const mass = state.inertia.centre_of_mass_and_mass.w;
			auto const switch_to = [&](BodyHandle target, EMotionType motion, SpatialVector value)
			{
				return BodyCommand{
					.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
					.body = target,
					.type = ECommand::SetMotionType,
					.flags = static_cast<std::uint32_t>(motion),
					.value = value,
				};
			};

			// Switch to kinematic and give a target in the same batch. The step moves the body to the target.
			BodyCommand const hold[] = {
				switch_to(body, EMotionType::Kinematic, {}),
				BodyCommand{
					.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
					.body = body,
					.type = ECommand::SetKinematicTarget,
					.transform = Translation(0.5f, 0, 0),
				},
			};
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, hold, 2) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Kinematic);
			PR_EXPECT(FEqlAbsolute(state.object_to_world.w.x, 0.5f, 1e-5f));
			PR_EXPECT(FEqlRelative(state.inertia.centre_of_mass_and_mass.w, mass, 1e-5f));

			// Release it with a chosen velocity. The velocity is exactly the release velocity, and the mass is unchanged.
			auto const release = switch_to(body, EMotionType::Dynamic, {.angular = {0, 0, 1, 0}, .linear = {0, 2, 0, 0}});
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &release, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Dynamic);
			PR_EXPECT(FEqlRelative(state.inertia.centre_of_mass_and_mass.w, mass, 1e-5f));
			PR_EXPECT(FEqlAbsolute(state.velocity.linear.x, 0.0f, 1e-6f) && FEqlAbsolute(state.velocity.linear.y, 2.0f, 1e-5f));
			PR_EXPECT(FEqlAbsolute(state.velocity.angular.z, 1.0f, 1e-5f));

			// A switch to Static cannot take a velocity. Without one, the body becomes static and can switch back to Dynamic with its own mass.
			auto fix_static = switch_to(body, EMotionType::Static, {.angular = {0, 0, 0, 0}, .linear = {1, 0, 0, 0}});
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &fix_static, 1) == EStatus::InvalidArgument);
			fix_static.value = {};
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &fix_static, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Static);
			auto const unfix = switch_to(body, EMotionType::Dynamic, {});
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &unfix, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Dynamic);
			PR_EXPECT(FEqlRelative(state.inertia.centre_of_mass_and_mass.w, mass, 1e-5f));

			// A target after a switch away from Kinematic is rejected, and the failed batch has no effect
			BodyCommand const bad_batch[] = {
				switch_to(body, EMotionType::Kinematic, {}),
				switch_to(body, EMotionType::Dynamic, {}),
				hold[1],
			};
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, bad_batch, 3) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Dynamic);

			// Unknown motion types are rejected
			auto unknown = switch_to(body, static_cast<EMotionType>(99), {});
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &unknown, 1) == EStatus::InvalidArgument);

			// A static body created without mass settings cannot become dynamic, and static bodies are created at rest
			auto desc = BodyDesc{
				.header = {sizeof(BodyDesc), PHYSICS_STRUCT_VERSION},
				.shape = shape,
				.object_to_world = Translation(5, 0, 0),
				.motion_type = EMotionType::Static,
				.mass_mode = EMassMode::ExplicitInertia,
				.flags = EBodyFlags::Enabled,
			};
			auto massless = BodyHandle{};
			PR_EXPECT(fix.m_api.BodyCreate(fix.m_engine, &desc, &massless) == EStatus::Success);
			auto const promote = switch_to(massless, EMotionType::Dynamic, {});
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &promote, 1) == EStatus::InvalidArgument);
			desc.momentum = {.angular = {0, 0, 0, 0}, .linear = {1, 0, 0, 0}};
			auto rejected = BodyHandle{};
			PR_EXPECT(fix.m_api.BodyCreate(fix.m_engine, &desc, &rejected) == EStatus::InvalidArgument);

			// A static body created with mass settings keeps them, and can become dynamic
			auto const anchored = MakeBody(fix.m_api, fix.m_engine, shape, Translation(-5, 0, 0), EMotionType::Static);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, anchored, &state) == EStatus::Success);
			PR_EXPECT(state.motion_type == EMotionType::Static);
			PR_EXPECT(FEqlRelative(state.inertia.centre_of_mass_and_mass.w, mass, 1e-5f));
			auto const promote_anchored = switch_to(anchored, EMotionType::Dynamic, {});
			PR_EXPECT(fix.m_api.CommandsApply(fix.m_engine, &promote_anchored, 1) == EStatus::Success);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, anchored) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, massless) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}
	};

	// Verify per-pair collision filtering through the public ABI.
	PRUnitTestClass(PhysicsDllCollisionFilterTests)
	{
		// Count the contact events between two bodies from the last completed step.
		static int CountContacts(PhysicsApi const& api, EngineHandle engine, BodyHandle body_a, BodyHandle body_b)
		{
			// Size the event buffer, then copy and count the matching contacts in either order
			auto required = std::uint32_t{};
			auto probe = api.EventsCopy(engine, nullptr, 0, &required);
			PR_EXPECT(probe == EStatus::Success || probe == EStatus::BufferTooSmall);
			auto events = std::vector<Event>(required);
			PR_EXPECT(api.EventsCopy(engine, events.data(), required, &required) == EStatus::Success);
			return static_cast<int>(std::ranges::count_if(events, [&](Event const& event)
			{
				// Match the pair in either order
				return event.type == EEvent::Contact && ((event.body_a == body_a && event.body_b == body_b) || (event.body_a == body_b && event.body_b == body_a));
			}));
		}

		// Read all ignore pairs.
		static std::vector<BodyPair> CopyPairs(PhysicsApi const& api, EngineHandle engine)
		{
			// Size the buffer, then copy
			auto required = std::uint32_t{};
			auto probe = api.BodyCollisionIgnoreCopy(engine, nullptr, 0, &required);
			PR_EXPECT(probe == EStatus::Success || probe == EStatus::BufferTooSmall);
			auto pairs = std::vector<BodyPair>(required);
			PR_EXPECT(api.BodyCollisionIgnoreCopy(engine, pairs.data(), required, &required) == EStatus::Success);
			return pairs;
		}

		// Overlapping bodies in an ignore pair make no contacts, and removing the pair restores contacts. This also holds for a kinematic body.
		PRUnitTestMethod(IgnorePairSuppressesContacts, Extended)
		{
			// Two overlapping dynamic boxes, and a kinematic box that overlaps the second one
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto box_a = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 0), EMotionType::Dynamic);
			auto box_b = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0.5f, 0, 0), EMotionType::Dynamic);
			auto pusher = MakeBody(fix.m_api, fix.m_engine, shape, Translation(1.0f, 0, 0), EMotionType::Kinematic);

			// Invalid requests are rejected
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, box_a, 1) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, box_b, 2) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, BodyHandle{}, 1) != EStatus::Success);
			auto required = std::uint32_t{};
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreCopy(fix.m_engine, nullptr, 0, nullptr) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreCopy(fix.m_engine, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0);

			// Ignore every overlapping pair. Pairs are unordered and setting twice has no extra effect.
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_b, box_a, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, box_b, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, pusher, box_b, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, pusher, 1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreCopy(fix.m_engine, nullptr, 0, &required) == EStatus::BufferTooSmall);
			PR_EXPECT(required == 3);
			auto pairs = CopyPairs(fix.m_api, fix.m_engine);
			PR_EXPECT(pairs.size() == 3);
			for (auto i = 0; i != isize(pairs); ++i)
			{
				// Each pair is canonical and the list is sorted
				PR_EXPECT(pairs[i].body_a < pairs[i].body_b);
				if (i != 0)
				{
					// Order by the first handle, then the second
					auto prev = std::make_pair(pairs[i - 1].body_a, pairs[i - 1].body_b);
					auto curr = std::make_pair(pairs[i].body_a, pairs[i].body_b);
					PR_EXPECT(prev < curr);
				}
			}

			// No contacts, so the overlapping bodies stay at rest
			for (auto i = 0; i != 3; ++i)
			{
				// Every step must be contact free
				PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, i / 60.0, nullptr, 0) == EStatus::Success);
				PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, box_a, box_b) == 0);
				PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, pusher, box_b) == 0);
			}
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box_b, &state) == EStatus::Success);
			PR_EXPECT(FEqlAbsolute(state.object_to_world.w.x, 0.5f, 1e-5f));

			// Removing a pair restores its contacts. Removing it again has no effect.
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_b, pusher, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_b, pusher, 0) == EStatus::Success);
			PR_EXPECT(CopyPairs(fix.m_api, fix.m_engine).size() == 2);
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 3 / 60.0, nullptr, 0) == EStatus::Success);
			PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, pusher, box_b) != 0);
			PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, box_a, box_b) == 0);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, box_b, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 4 / 60.0, nullptr, 0) == EStatus::Success);
			PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, box_a, box_b) != 0);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, pusher) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box_b) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box_a) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		// A checkpoint keeps the ignore pairs, the collision filters and the twist of a drifting kinematic body. A restored engine continues exactly like the
		// original. Every body overlaps another but is kept contact free by a pair or a filter, so no warm-start cache is involved.
		PRUnitTestMethod(CheckpointKeepsFilterAndKinematicState, Extended)
		{
			// Overlapping boxes: 'a' and 'b' ignore each other, and the layer filter keeps 'c' away from both. A far kinematic box gets a one-step target.
			auto fix = PhysicsFixture{};
			auto const elapsed = 1.0f / 60.0f;
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto box_a = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 0), EMotionType::Dynamic);
			auto box_b = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0.5f, 0, 0), EMotionType::Dynamic);
			auto box_c = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0.5f, 0), EMotionType::Dynamic);
			auto mover = MakeBody(fix.m_api, fix.m_engine, shape, Translation(10, 0, 0), EMotionType::Kinematic);
			auto bodies = std::array{box_a, box_b, box_c, mover};
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, box_b, 1) == EStatus::Success);
			auto state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box_c, &state) == EStatus::Success);
			state.collision_layer = 0x2;
			state.collision_mask = 0x2;
			PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, box_c, &state) == EStatus::Success);
			auto command = BodyCommand{
				.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
				.body = mover,
				.type = ECommand::SetKinematicTarget,
				.transform = Translation(10, 0.1f, 0),
			};
			PR_EXPECT(fix.m_api.Step(fix.m_engine, elapsed, 0.0, &command, 1) == EStatus::Success);
			auto checkpoint = WriteCheckpoint(fix.m_api, fix.m_engine);

			// Run the original on and record where every body ends up
			auto const run = [&]
			{
				// The bodies stay contact free while the kinematic box keeps its twist
				auto positions = std::vector<Vector4>{};
				for (auto i = 1; i != 4; ++i)
				{
					PR_EXPECT(fix.m_api.Step(fix.m_engine, elapsed, i * elapsed, nullptr, 0) == EStatus::Success);
					PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, box_a, box_b) == 0);
					PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, box_a, box_c) == 0);
					PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, box_b, box_c) == 0);
				}
				for (auto body : bodies)
				{
					// Final positions
					auto body_state = BodyState{};
					PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &body_state) == EStatus::Success);
					positions.push_back(body_state.object_to_world.w);
				}
				return positions;
			};
			auto expected = run();
			PR_EXPECT(FEqlAbsolute(expected[3].y, 0.4f, 1e-4f));

			// Clear the engine so the checkpoint can claim its identity again
			for (auto body : bodies)
				PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);

			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);

			// A pair that is not in canonical order is rejected, and the engine stays empty
			auto corrupt = checkpoint;
			auto pair = BodyPair{};
			memcpy(&pair, corrupt.data() + corrupt.size() - sizeof(pair), sizeof(pair));
			std::swap(pair.body_a, pair.body_b);
			memcpy(corrupt.data() + corrupt.size() - sizeof(pair), &pair, sizeof(pair));
			UpdateCheckpointChecksum(corrupt);
			PR_EXPECT(fix.m_api.CheckpointRead(fix.m_engine, corrupt.data(), corrupt.size()) == EStatus::InvalidArgument);
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box_a, &state) != EStatus::Success);

			// The restored engine has the same pairs and filters, and repeats the same motion
			PR_EXPECT(fix.m_api.CheckpointRead(fix.m_engine, checkpoint.data(), checkpoint.size()) == EStatus::Success);
			auto pairs = CopyPairs(fix.m_api, fix.m_engine);
			PR_EXPECT(pairs.size() == 1 && pairs[0].body_a == std::min(box_a, box_b) && pairs[0].body_b == std::max(box_a, box_b));
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box_c, &state) == EStatus::Success);
			PR_EXPECT(state.collision_layer == 0x2 && state.collision_mask == 0x2);
			auto actual = run();
			for (auto i = 0; i != isize(bodies); ++i)
			{
				// Same end positions as the original run
				PR_EXPECT(FEqlAbsolute(actual[i].x, expected[i].x, 1e-5f) && FEqlAbsolute(actual[i].y, expected[i].y, 1e-5f) && FEqlAbsolute(actual[i].z, expected[i].z, 1e-5f));
			}

			for (auto body : bodies)
				PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body) == EStatus::Success);

			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		// Destroying a body removes its ignore pairs, so a new body that reuses the slot collides normally.
		PRUnitTestMethod(DestroyRemovesIgnorePairs, Extended)
		{
			// Two overlapping boxes that ignore each other
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto box_a = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 0), EMotionType::Dynamic);
			auto box_b = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0.5f, 0, 0), EMotionType::Dynamic);
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, box_b, 1) == EStatus::Success);

			// Destroying either body removes the pair, and its stale handle can no longer be used in a pair
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box_b) == EStatus::Success);
			PR_EXPECT(CopyPairs(fix.m_api, fix.m_engine).empty());
			PR_EXPECT(fix.m_api.BodyCollisionIgnoreSet(fix.m_engine, box_a, box_b, 1) == EStatus::StaleHandle);

			// A replacement body in the same place does not inherit the pair
			auto box_c = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0.5f, 0, 0), EMotionType::Dynamic);
			PR_EXPECT(box_c != box_b);
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);
			PR_EXPECT(CountContacts(fix.m_api, fix.m_engine, box_a, box_c) != 0);

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box_c) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box_a) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
		}

		// Two bodies collide only when each one's layer is in the other's mask. The world surfaces use the engine's world filter in the same way.
		PRUnitTestMethod(LayerMaskMatrix, Extended)
		{
			// Set a body's collision filter through the full body state, and check that it reads back
			auto fix = PhysicsFixture{};
			auto set_filter = [&](BodyHandle body, std::uint32_t layer, std::uint32_t mask)
			{
				// Round trip the state so only the filter changes
				auto state = BodyState{};
				PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &state) == EStatus::Success);
				state.collision_layer = layer;
				state.collision_mask = mask;
				PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, body, &state) == EStatus::Success);
				auto readback = BodyState{};
				PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, body, &readback) == EStatus::Success);
				PR_EXPECT(readback.collision_layer == layer && readback.collision_mask == mask);
			};

			// One overlapping pair per filter combination, far enough apart that pairs cannot touch each other
			struct Case { std::uint32_t layer_a, mask_a, layer_b, mask_b; bool collides; };
			auto const cases = std::array{
				Case{.layer_a = 1, .mask_a = 2, .layer_b = 2, .mask_b = 1, .collides = true},
				Case{.layer_a = 1, .mask_a = 2, .layer_b = 2, .mask_b = 0, .collides = false},
				Case{.layer_a = 1, .mask_a = 0, .layer_b = 2, .mask_b = 1, .collides = false},
				Case{.layer_a = 1, .mask_a = 1, .layer_b = 2, .mask_b = 2, .collides = false},
			};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto bodies = std::vector<std::pair<BodyHandle, BodyHandle>>{};
			for (auto i = 0; i != isize(cases); ++i)
			{
				// Create the pair and apply its filters
				auto const x = 10.0f * i;
				auto body_a = MakeBody(fix.m_api, fix.m_engine, shape, Translation(x, 0, 0), EMotionType::Dynamic);
				auto body_b = MakeBody(fix.m_api, fix.m_engine, shape, Translation(x + 0.5f, 0, 0), EMotionType::Dynamic);
				set_filter(body_a, cases[i].layer_a, cases[i].mask_a);
				set_filter(body_b, cases[i].layer_b, cases[i].mask_b);
				bodies.push_back(std::make_pair(body_a, body_b));
			}

			// Only the pair whose filters accept each other makes contacts
			PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, 0.0, nullptr, 0) == EStatus::Success);
			for (auto i = 0; i != isize(cases); ++i)
			{
				// Compare the contact count with the expected result
				auto const contacts = CountContacts(fix.m_api, fix.m_engine, bodies[i].first, bodies[i].second);
				PR_EXPECT((contacts != 0) == cases[i].collides);
			}

			// Snapshots report the filter too
			auto required = std::uint32_t{};
			auto probe = fix.m_api.SnapshotCopy(fix.m_engine, nullptr, 0, &required);
			PR_EXPECT(probe == EStatus::Success || probe == EStatus::BufferTooSmall);
			auto snapshots = std::vector<BodySnapshot>(required);
			PR_EXPECT(fix.m_api.SnapshotCopy(fix.m_engine, snapshots.data(), required, &required) == EStatus::Success);
			auto const snapshot = std::ranges::find_if(snapshots, [&](BodySnapshot const& s) { return s.body == bodies[0].first; });
			PR_EXPECT(snapshot != snapshots.end() && snapshot->collision_layer == 1 && snapshot->collision_mask == 2);

			for (auto const& [body_a, body_b] : bodies)
			{
				// Release the pair
				PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body_a) == EStatus::Success);
				PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body_b) == EStatus::Success);
			}
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);

			// A sphere pressed into the cylindrical boundary on layer 2
			auto boundary = CylindricalBoundaryDesc{
				.header = {sizeof(CylindricalBoundaryDesc), PHYSICS_STRUCT_VERSION},
				.radius = 100,
				.surface_spacing = 0.05f,
			};
			PR_EXPECT(fix.m_api.EngineCylindricalBoundarySet(fix.m_engine, &boundary) == EStatus::Success);
			auto sphere = SphereShape{.common = MakeCommon(), .radius = 0.5f};
			sphere.common.header.size = sizeof(sphere);
			auto sphere_shape = ShapeHandle{};
			PR_EXPECT(fix.m_api.ShapeCreateSphere(fix.m_engine, &sphere, &sphere_shape) == EStatus::Success);
			auto ball = MakeBody(fix.m_api, fix.m_engine, sphere_shape, Translation(99.51f, 0, 0), EMotionType::Dynamic);
			set_filter(ball, 2, 0xFFFFFFFFU);

			// Count the world contacts from the last step
			auto world_contacts = [&]
			{
				// Size the event buffer, then copy and count
				auto count = std::uint32_t{};
				auto status = fix.m_api.EventsCopy(fix.m_engine, nullptr, 0, &count);
				PR_EXPECT(status == EStatus::Success || status == EStatus::BufferTooSmall);
				auto events = std::vector<Event>(count);
				PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, events.data(), count, &count) == EStatus::Success);
				return std::ranges::count_if(events, [](Event const& e) { return e.type == EEvent::WorldContact; });
			};

			// The world mask decides whether the ball touches the boundary. Each step re-places the ball, because a contact pushes it away.
			auto config = pr::physics::Config{};
			PR_EXPECT(fix.m_api.EngineConfigGet(fix.m_engine, &config) == EStatus::Success);
			PR_EXPECT(config.world_collision_layer == 1 && config.world_collision_mask == 0xFFFFFFFFU);
			auto step_time = 1.0 / 60.0;
			for (auto const [world_mask, touches] : {std::make_pair(1U, false), std::make_pair(3U, true)})
			{
				// Set the world filter, then step once
				config.world_collision_mask = world_mask;
				PR_EXPECT(fix.m_api.EngineConfigSet(fix.m_engine, &config) == EStatus::Success);
				auto state = BodyState{};
				PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, ball, &state) == EStatus::Success);
				state.object_to_world = Translation(99.51f, 0, 0);
				state.momentum = {};
				PR_EXPECT(fix.m_api.BodyStateSet(fix.m_engine, ball, &state) == EStatus::Success);
				PR_EXPECT(fix.m_api.Step(fix.m_engine, 1.0f / 60.0f, step_time, nullptr, 0) == EStatus::Success);
				PR_EXPECT((world_contacts() != 0) == touches);
				step_time += 1.0 / 60.0;
			}

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, ball) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, sphere_shape) == EStatus::Success);
		}
	};

	// Verify the impulse and relative velocity reported by contact events.
	PRUnitTestClass(PhysicsDllEventImpulseTests)
	{
		// Copy the contact events between two bodies from the last completed step, in either order.
		static std::vector<Event> PairContacts(PhysicsApi const& api, EngineHandle engine, BodyHandle body_a, BodyHandle body_b)
		{
			// Size the event buffer, then copy and keep the matching contacts
			auto required = std::uint32_t{};
			auto probe = api.EventsCopy(engine, nullptr, 0, &required);
			PR_EXPECT(probe == EStatus::Success || probe == EStatus::BufferTooSmall);
			auto events = std::vector<Event>(required);
			PR_EXPECT(api.EventsCopy(engine, events.data(), required, &required) == EStatus::Success);
			std::erase_if(events, [&](Event const& event)
			{
				// Keep only contacts of this pair
				return event.type != EEvent::Contact || !((event.body_a == body_a && event.body_b == body_b) || (event.body_a == body_b && event.body_b == body_a));
			});
			return events;
		}

		// Stop a body from sleeping so it keeps making contacts while at rest.
		static void NeverSleep(PhysicsApi const& api, EngineHandle engine, BodyHandle body)
		{
			// Read, set the flag, write back
			auto state = BodyState{};
			PR_EXPECT(api.BodyStateGet(engine, body, &state) == EStatus::Success);
			state.flags = static_cast<EBodyFlags>(static_cast<std::uint32_t>(state.flags) | static_cast<std::uint32_t>(EBodyFlags::NeverSleep));
			PR_EXPECT(api.BodyStateSet(engine, body, &state) == EStatus::Success);
		}

		// A box resting on a platform reports normal impulses that balance its weight, with no friction impulse or sideways motion.
		PRUnitTestMethod(RestingBoxReportsWeight, Extended)
		{
			// A box resting on a stationary kinematic platform. Use a grippy, inelastic material so the box settles.
			auto fix = PhysicsFixture{};
			auto const material = MaterialProperties{
				.header = {sizeof(MaterialProperties), PHYSICS_STRUCT_VERSION},
				.id = 0,
				.static_friction = 0.8f,
				.normal_elasticity = 0.0f,
				.tangential_elasticity = 0.0f,
				.torsional_elasticity = 0.0f,
				.density = 1000.0f,
			};
			PR_EXPECT(fix.m_api.MaterialSet(fix.m_engine, &material) == EStatus::Success);
			auto platform_shape = MakeBox(fix.m_api, fix.m_engine, {8.0f, 8.0f, 0.5f, 0});
			auto box_shape = MakeBox(fix.m_api, fix.m_engine, {0.5f, 0.5f, 0.5f, 0});
			auto platform = MakeBody(fix.m_api, fix.m_engine, platform_shape, Identity(), EMotionType::Kinematic);
			auto box = MakeBody(fix.m_api, fix.m_engine, box_shape, Translation(0, 0, 0.5f), EMotionType::Dynamic);
			NeverSleep(fix.m_api, fix.m_engine, box);
			auto box_state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box, &box_state) == EStatus::Success);
			auto const mass = box_state.inertia.centre_of_mass_and_mass.w;
			PR_EXPECT(mass > 0.0f);

			// Settle under gravity, re-applied every step because it lasts for one step
			auto const g = 9.8f;
			auto const dt = 1.0f / 60.0f;
			auto const substeps = 4U;
			auto const gravity = BodyCommand{
				.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
				.body = box,
				.type = ECommand::SetGravity,
				.value = {.angular = {0, 0, 0, 0}, .linear = {0, 0, -g, 0}},
			};
			for (auto i = 0; i != 30; ++i)
				PR_EXPECT(fix.m_api.StepEx(fix.m_engine, dt, i * dt, substeps, &gravity, 1) == EStatus::Success);

			// Over a window of steps, the support impulse balances gravity plus the box's change in momentum. Contacts
			// at rest come and go between substeps, so compare whole-window totals rather than single substeps.
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box, &box_state) == EStatus::Success);
			auto const vz_start = box_state.velocity.linear.z;
			auto const window = 30;
			auto total_normal_impulse = 0.0f;
			for (auto i = 0; i != window; ++i)
			{
				// Step and add up the support from every contact of the pair. There is no friction impulse because nothing slides.
				PR_EXPECT(fix.m_api.StepEx(fix.m_engine, dt, (30 + i) * dt, substeps, &gravity, 1) == EStatus::Success);
				for (auto const& contact : PairContacts(fix.m_api, fix.m_engine, platform, box))
				{
					// Every contact pushes the box up, with no friction
					auto const& t = contact.tangent_impulse;
					PR_EXPECT(contact.substep_index >= 0 && contact.substep_index < static_cast<int>(substeps));
					PR_EXPECT(contact.normal_impulse >= 0.0f);
					PR_EXPECT(std::abs(contact.normal.z) > 0.99f);
					PR_EXPECT(std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z) < 0.01f * mass * g * dt);
					PR_EXPECT(std::abs(contact.relative_velocity.x) < 1e-3f && std::abs(contact.relative_velocity.y) < 1e-3f);
					total_normal_impulse += contact.normal_impulse;
				}
			}
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box, &box_state) == EStatus::Success);
			auto const vz_end = box_state.velocity.linear.z;
			auto const expected = mass * (vz_end - vz_start) + mass * g * window * dt;
			if (!FEqlRelative(total_normal_impulse, expected, 0.02f))
				log_stream() << "Total normal impulse " << total_normal_impulse << ", expected " << expected << '\n';

			PR_EXPECT(FEqlRelative(total_normal_impulse, expected, 0.02f));

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, platform) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, box_shape) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, platform_shape) == EStatus::Success);
		}

		// A moving kinematic box hits a resting dynamic box. The event reports the impulse that changed the dynamic box's momentum, and the relative velocity after the solve.
		PRUnitTestMethod(KinematicHitReportsImpulseAndVelocity, Extended)
		{
			// A kinematic box moving along +x towards a resting dynamic box. The default material is elastic, so the boxes separate after the hit.
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {1, 1, 1, 0});
			auto pusher = MakeBody(fix.m_api, fix.m_engine, shape, Translation(-1.2f, 0, 0), EMotionType::Kinematic);
			auto box = MakeBody(fix.m_api, fix.m_engine, shape, Identity(), EMotionType::Dynamic);
			auto box_state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box, &box_state) == EStatus::Success);
			auto const mass = box_state.inertia.centre_of_mass_and_mass.w;
			auto const speed = 2.0f;
			auto const velocity = BodyCommand{
				.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
				.body = pusher,
				.type = ECommand::SetVelocity,
				.value = {.angular = {0, 0, 0, 0}, .linear = {speed, 0, 0, 0}},
			};

			// Step with one substep until the first contact, so the event velocity is the final velocity of the step
			auto const dt = 1.0f / 60.0f;
			auto contacts = std::vector<Event>{};
			for (auto i = 0; i != 30 && contacts.empty(); ++i)
			{
				// The pusher keeps its twist after the first command
				PR_EXPECT(fix.m_api.StepEx(fix.m_engine, dt, i * dt, 1, i == 0 ? &velocity : nullptr, i == 0 ? 1U : 0U) == EStatus::Success);
				contacts = PairContacts(fix.m_api, fix.m_engine, pusher, box);
			}
			PR_EXPECT(contacts.size() == 1);
			auto const& contact = contacts.front();

			// The normal impulse is the dynamic box's change in momentum, all along the contact normal
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, box, &box_state) == EStatus::Success);
			auto pusher_state = BodyState{};
			PR_EXPECT(fix.m_api.BodyStateGet(fix.m_engine, pusher, &pusher_state) == EStatus::Success);
			auto const box_speed = box_state.velocity.linear.x;
			PR_EXPECT(box_speed > speed);
			PR_EXPECT(FEqlRelative(contact.normal_impulse, mass * box_speed, 0.02f));
			PR_EXPECT(std::abs(contact.normal.x) > 0.99f);
			auto const& t = contact.tangent_impulse;
			PR_EXPECT(std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z) < 0.01f * contact.normal_impulse);

			// The relative velocity is B's velocity minus A's after the solve, so swap the sign when the box is A
			auto const sign = contact.body_b == box ? 1.0f : -1.0f;
			auto const relative_x = sign * (box_state.velocity.linear.x - pusher_state.velocity.linear.x);
			PR_EXPECT(FEqlAbsolute(contact.relative_velocity.x, relative_x, 1e-3f));
			PR_EXPECT(FEqlAbsolute(contact.relative_velocity.y, 0.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(contact.relative_velocity.z, 0.0f, 1e-3f));

			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, box) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, pusher) == EStatus::Success);
			PR_EXPECT(fix.m_api.ShapeDestroy(fix.m_engine, shape) == EStatus::Success);
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
			auto checksum_before = std::uint64_t{};
			PR_EXPECT(fix.m_api.StateChecksumGet(fix.m_engine, &checksum_before) == EStatus::Success);

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
			auto checksum_after = std::uint64_t{};
			PR_EXPECT(fix.m_api.StateChecksumGet(fix.m_engine, &checksum_after) == EStatus::Success);
			PR_EXPECT(checksum_after != checksum_before);
			PR_EXPECT(fix.m_api.StateChecksumGet(fix.m_engine, nullptr) == EStatus::InvalidArgument);

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
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, fix.m_external_device, &other) == EStatus::Success);
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

		PRUnitTestMethod(BatchedConstraintUpdatesAreAtomic, Extended)
		{
			// Two welds between distinct body pairs give independent constraints to update together
			auto fix = PhysicsFixture{};
			auto shape = MakeBox(fix.m_api, fix.m_engine, {0.25f, 0.25f, 0.25f, 0});
			auto body0 = MakeBody(fix.m_api, fix.m_engine, shape, Translation(0, 0, 1), EMotionType::Dynamic);
			auto body1 = MakeBody(fix.m_api, fix.m_engine, shape, Translation(1, 0, 1), EMotionType::Dynamic);
			auto body2 = MakeBody(fix.m_api, fix.m_engine, shape, Translation(2, 0, 1), EMotionType::Dynamic);
			auto frame = [](BodyHandle body)
			{
				return ConstraintFrameProperties{.type = EConstraintEndpoint::RigidBody, .object_handle = body, .constraint_to_body = Identity()};
			};
			auto descs = std::array{WeldConstraint(frame(body0), frame(body1)), WeldConstraint(frame(body1), frame(body2))};
			auto constraints = std::array<PersistentConstraintHandle, 2>{};
			PR_EXPECT(fix.m_api.ConstraintCreateD6(fix.m_engine, &descs[0], &constraints[0]) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintCreateD6(fix.m_engine, &descs[1], &constraints[1]) == EStatus::Success);

			// An invalid entry rejects the whole batch, leaving the valid entry unchanged
			auto updates = descs;
			updates[0].angular[0].mode = EConstraintMode::Driven;
			updates[0].angular[0].target_velocity = 0.5f;
			updates[0].angular[0].damping = 1.0f;
			updates[1].frame_b.object_handle = BodyHandle{};
			PR_EXPECT(fix.m_api.ConstraintUpdateD6Batch(fix.m_engine, constraints.data(), updates.data(), 2) != EStatus::Success);
			auto readback = D6ConstraintProperties{};
			auto broken = std::int32_t{};
			PR_EXPECT(fix.m_api.ConstraintGetD6(fix.m_engine, constraints[0], &readback, &broken) == EStatus::Success);
			PR_EXPECT(readback.angular[0].mode == descs[0].angular[0].mode);

			// A valid batch replaces every descriptor and moves endpoint references, so the released body can be destroyed
			updates[1] = WeldConstraint(frame(body1), frame(body0));
			PR_EXPECT(fix.m_api.ConstraintUpdateD6Batch(fix.m_engine, constraints.data(), updates.data(), 2) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintGetD6(fix.m_engine, constraints[0], &readback, &broken) == EStatus::Success);
			PR_EXPECT(readback.angular[0].mode == EConstraintMode::Driven);
			PR_EXPECT(readback.angular[0].target_velocity == 0.5f);
			PR_EXPECT(fix.m_api.ConstraintGetD6(fix.m_engine, constraints[1], &readback, &broken) == EStatus::Success);
			PR_EXPECT(readback.frame_b.object_handle == body0);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body2) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body1) == EStatus::InvalidArgument);

			// An empty batch is valid, and null arrays with a non-zero count are rejected
			PR_EXPECT(fix.m_api.ConstraintUpdateD6Batch(fix.m_engine, nullptr, nullptr, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintUpdateD6Batch(fix.m_engine, nullptr, nullptr, 1) == EStatus::InvalidArgument);

			// Clean up
			PR_EXPECT(fix.m_api.StepEx(fix.m_engine, 1.0f / 120.0f, 0.0, 1, nullptr, 0) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintDestroy(fix.m_engine, constraints[0]) == EStatus::Success);
			PR_EXPECT(fix.m_api.ConstraintDestroy(fix.m_engine, constraints[1]) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body1) == EStatus::Success);
			PR_EXPECT(fix.m_api.BodyDestroy(fix.m_engine, body0) == EStatus::Success);
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

	// Verify that procedural surfaces retain contact geometry when their owned endpoint has no ABI body handle.
	PRUnitTestClass(PhysicsDllWorldContactTests)
	{
		// Exercise stationary and moving contacts against both sources, then remove them to reject stale world events.
		PRUnitTestMethod(TerrainAndBoundaryEventsUseExplicitWorldEndpoint, Extended)
		{
			for (auto const [boundary, moving] : {std::pair{false, false}, std::pair{true, false}, std::pair{false, true}, std::pair{true, true}})
			{
				// Install a finite world surface without constructing a public static body.
				auto fix = PhysicsFixture{};
				auto position = Translation(99.51f, 0, 0);
				if (boundary)
				{
					auto desc = CylindricalBoundaryDesc{
						.header = {sizeof(CylindricalBoundaryDesc), PHYSICS_STRUCT_VERSION},
						.radius = 100,
						.surface_spacing = 0.05f,
					};
					PR_EXPECT(fix.m_api.EngineCylindricalBoundarySet(fix.m_engine, &desc) == EStatus::Success);
				}
				else
				{
					// Disable spatial bands so the contact normal is known independently of seeded terrain grade.
					auto config = terrain::landscape::BaselineSurfaceConfig{};
					config.m_regional_base.m_amplitude = config.m_region_selector.m_amplitude = config.m_region_uplift.m_amplitude = 0;
					config.m_plains.m_amplitude = config.m_hills.m_amplitude = config.m_mountains.m_amplitude = 0;
					config.m_domain_warp.m_amplitude_m = 0;
					config.m_uplift_height_m = config.m_mountain_base_height_m = 0;

					// Use the same immutable recipe for reference height and the ABI-owned terrain copy.
					auto desc = MakeTerrainDesc(config);
					auto surface = terrain::landscape::BaselineSurface(config);
					position = Translation(0, 0, static_cast<float>(surface.Sample({0, 0}).m_height) + 0.49f);
					PR_EXPECT(fix.m_api.EngineTerrainSet(fix.m_engine, &desc) == EStatus::Success);
				}

				// Generate a shallow contact through four actual native substeps.
				auto sphere = SphereShape{.common = MakeCommon(), .radius = 0.5f};
				sphere.common.header.size = sizeof(sphere);
				auto shape = ShapeHandle{};
				PR_EXPECT(fix.m_api.ShapeCreateSphere(fix.m_engine, &sphere, &shape) == EStatus::Success);
				auto body = MakeBody(fix.m_api, fix.m_engine, shape, position, EMotionType::Dynamic);
				auto velocity = BodyCommand{
					.header = {sizeof(BodyCommand), PHYSICS_STRUCT_VERSION},
					.body = body,
					.type = ECommand::SetVelocity,
					.value = {{0, moving ? 6.0f : 0, 0, 0}, {0, moving ? 2.0f : 0, 0, 0}},
				};
				PR_EXPECT(fix.m_api.StepEx(fix.m_engine, 1.0f / 60.0f, 0, 4, &velocity, 1) == EStatus::Success);

				// Capacity probing must retain the events, with exactly one zero endpoint and finite world geometry.
				auto required = std::uint32_t{};
				PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, nullptr, 0, &required) == EStatus::BufferTooSmall);
				auto events = std::vector<Event>(required);
				PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, events.data(), required, &required) == EStatus::Success);
				auto contacts = 0;
				auto substeps = 0U;
				for (auto const& evt : events)
				{
					switch (evt.type)
					{
						case EEvent::WorldContact: { break; }
						case EEvent::Sleep:
						case EEvent::Wake: { continue; }
						default: { throw std::runtime_error("Unexpected event in an isolated world-contact fixture"); }
					}
					++contacts;
					PR_EXPECT((evt.body_a == body && evt.body_b == 0) || (evt.body_a == 0 && evt.body_b == body));
					PR_EXPECT(evt.child_a == PHYSICS_NO_CHILD && evt.child_b == PHYSICS_NO_CHILD);
					PR_EXPECT(evt.substep_index >= 0 && evt.substep_index < 4);
					substeps |= 1U << evt.substep_index;
					PR_EXPECT(evt.point_count > 0 && evt.point_count <= 4);
					PR_EXPECT(std::isfinite(evt.depth) && evt.depth >= 0);
					auto outward = boundary ? evt.normal.x : -evt.normal.z;
					PR_EXPECT((evt.body_a == body ? outward : -outward) > 0.9f);
					PR_EXPECT(std::abs(boundary ? evt.normal.z : evt.normal.x) < 0.0001f);
					if (!boundary)
						PR_EXPECT(std::abs(evt.normal.y) < 0.0001f);

					// World-space points remain on the sampled contact midpoint despite body translation and rotation within the frame.
					for (auto i = 0U; i != evt.point_count && i != 4; ++i)
					{
						PR_EXPECT(std::isfinite(evt.points[i].x) && std::isfinite(evt.points[i].y) && std::isfinite(evt.points[i].z));
						PR_EXPECT(evt.points[i].w == 1);
						auto midpoint_depth = boundary ? std::hypot(evt.points[i].x, evt.points[i].y) - 100.0f : position.w.z - 0.49f - evt.points[i].z;
						if (std::abs(midpoint_depth - 0.5f * evt.depth) >= 0.0011f)
							log_stream() << "World contact: boundary=" << boundary << ", moving=" << moving << ", substep=" << evt.substep_index << ", midpoint depth=" << midpoint_depth << ", depth=" << evt.depth << '\n';

						PR_EXPECT(std::abs(midpoint_depth - 0.5f * evt.depth) < 0.0011f);
					}
				}
				PR_EXPECT(contacts > 0);
				PR_EXPECT((substeps & ~1U) != 0);

				// Replacing the endpoint between frames cannot retain its previous contact stream.
				PR_EXPECT(fix.m_api.EngineTerrainSet(fix.m_engine, nullptr) == EStatus::Success);
				PR_EXPECT(fix.m_api.EngineCylindricalBoundarySet(fix.m_engine, nullptr) == EStatus::Success);
				PR_EXPECT(fix.m_api.StepEx(fix.m_engine, 1.0f / 60.0f, 1.0 / 60.0, 4, nullptr, 0) == EStatus::Success);
				PR_EXPECT(fix.m_api.EventsCopy(fix.m_engine, nullptr, 0, &required) == EStatus::Success);
				PR_EXPECT(required == 0);
			}
		}
	};

	// Verify the context-free landscape matches the library surface and rejects caller contract violations.
	PRUnitTestClass(PhysicsDllLandscapeTests)
	{
		PRUnitTestMethod(SamplesAndRecipeMatchLibrarySurface, Extended)
		{
			// Build the reference surface and the DLL-owned copy from the same seeded configuration.
			auto api = PhysicsApi{};
			auto config = terrain::landscape::BaselineSurfaceConfig{.m_seed = 42};
			auto reference = terrain::landscape::BaselineSurface(config);
			auto desc = MakeTerrainDesc(config);
			auto landscape = LandscapeHandle{};
			PR_EXPECT(api.LandscapeCreate(&desc, &landscape) == EStatus::Success);
			PR_EXPECT(landscape != nullptr);

			// The recipe must be byte-identical so shader consumers evaluate the same terrain.
			auto recipe = terrain::landscape::shared::BaselineRecipe{};
			PR_EXPECT(api.LandscapeRecipe(landscape, &recipe) == EStatus::Success);
			PR_EXPECT(std::memcmp(&recipe, &reference.Recipe(), sizeof(recipe)) == 0);

			// The local recipe must also be byte-identical to the library's for the same origin and radius.
			auto const origin = std::array<double, 2>{1234.5, -678.25};
			auto local = terrain::landscape::shared::BaselineLocalRecipe{};
			PR_EXPECT(api.LandscapeLocalRecipe(landscape, origin.data(), 4096.0, &local) == EStatus::Success);
			auto const expected_local = reference.LocalRecipe({origin[0], origin[1]}, 4096.0);
			PR_EXPECT(std::memcmp(&local, &expected_local, sizeof(local)) == 0);

			// Batched samples must preserve order and agree with both scalar library sampling and recipe evaluation.
			auto const limit = config.m_supported_coordinate_abs_m;
			auto positions = std::vector<double>{0, 0, -125.5, 300.0, 1234.25, -987.75, limit, -limit, -0.5 * limit, 0.25 * limit};
			auto const count = static_cast<std::uint32_t>(positions.size() / 2);
			auto samples = std::vector<SurfaceSampleDesc>(count);
			PR_EXPECT(api.LandscapeSample(landscape, positions.data(), samples.data(), count) == EStatus::Success);
			for (auto i = 0U; i != count; ++i)
			{
				// Compare every field because the DLL converts the library sample field by field.
				auto const x = positions[2 * i + 0];
				auto const y = positions[2 * i + 1];
				auto const expected = reference.Sample({x, y});
				PR_EXPECT(samples[i].height == expected.m_height);
				PR_EXPECT(samples[i].gradient.x == expected.m_gradient_xy.x && samples[i].gradient.y == expected.m_gradient_xy.y);
				PR_EXPECT(samples[i].material_id == expected.m_material_id && samples[i].reserved == 0);

				auto const evaluated = terrain::landscape::shared::BaselineEvaluate(recipe, terrain::v2d{x, y});
				PR_EXPECT(evaluated.m_status == 0);
				PR_EXPECT(evaluated.m_height == samples[i].height);
				PR_EXPECT(evaluated.m_dx == samples[i].gradient.x && evaluated.m_dy == samples[i].gradient.y);
			}

			// An empty batch needs no buffers, and null destroy is a no-op.
			PR_EXPECT(api.LandscapeSample(landscape, nullptr, nullptr, 0) == EStatus::Success);
			api.LandscapeDestroy(landscape);
			api.LandscapeDestroy(nullptr);
		}
		PRUnitTestMethod(RejectsInvalidInputs, Extended)
		{
			// Invalid descriptors fail and leave the output handle null.
			auto api = PhysicsApi{};
			auto desc = MakeTerrainDesc(terrain::landscape::BaselineSurfaceConfig{.m_seed = 7});
			auto landscape = reinterpret_cast<LandscapeHandle>(std::uintptr_t{1});
			PR_EXPECT(api.LandscapeCreate(nullptr, &landscape) == EStatus::InvalidArgument);
			PR_EXPECT(landscape == nullptr);
			PR_EXPECT(api.LandscapeCreate(&desc, nullptr) == EStatus::InvalidArgument);
			auto bad = desc;
			bad.hills.reserved = 1;
			PR_EXPECT(api.LandscapeCreate(&bad, &landscape) == EStatus::InvalidArgument);
			PR_EXPECT(landscape == nullptr);
			bad = desc;
			bad.plains.wavelength = -1;
			PR_EXPECT(api.LandscapeCreate(&bad, &landscape) == EStatus::InvalidArgument);
			PR_EXPECT(landscape == nullptr);

			// Missing buffers, nonfinite coordinates, and coordinates outside the supported square are rejected.
			PR_EXPECT(api.LandscapeCreate(&desc, &landscape) == EStatus::Success);
			auto sample = SurfaceSampleDesc{};
			auto position = std::array<double, 2>{0, 0};
			PR_EXPECT(api.LandscapeSample(nullptr, position.data(), &sample, 1) == EStatus::InvalidArgument);
			PR_EXPECT(api.LandscapeSample(landscape, nullptr, &sample, 1) == EStatus::InvalidArgument);
			PR_EXPECT(api.LandscapeSample(landscape, position.data(), nullptr, 1) == EStatus::InvalidArgument);
			position = {std::numeric_limits<double>::quiet_NaN(), 0};
			PR_EXPECT(api.LandscapeSample(landscape, position.data(), &sample, 1) == EStatus::InvalidArgument);
			position = {2 * desc.supported_coordinate, 0};
			PR_EXPECT(api.LandscapeSample(landscape, position.data(), &sample, 1) == EStatus::InvalidArgument);
			PR_EXPECT(api.LandscapeRecipe(landscape, nullptr) == EStatus::InvalidArgument);

			// Local recipes need an origin and output, a positive radius, and a supported square.
			auto local = terrain::landscape::shared::BaselineLocalRecipe{};
			auto origin = std::array<double, 2>{0, 0};
			PR_EXPECT(api.LandscapeLocalRecipe(landscape, nullptr, 100.0, &local) == EStatus::InvalidArgument);
			PR_EXPECT(api.LandscapeLocalRecipe(landscape, origin.data(), 100.0, nullptr) == EStatus::InvalidArgument);
			PR_EXPECT(api.LandscapeLocalRecipe(landscape, origin.data(), 0.0, &local) == EStatus::InvalidArgument);
			PR_EXPECT(api.LandscapeLocalRecipe(landscape, origin.data(), 2 * desc.supported_coordinate, &local) == EStatus::InvalidArgument);
			api.LandscapeDestroy(landscape);
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
			PR_EXPECT(fix.m_api.EngineCreate(fix.m_context, nullptr, fix.m_external_device, &other) == EStatus::Success);
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
	// Serial because 'AbandonAndFinalShutdown' releases the process-wide shared physics context, which other classes use while they run.
	PRUnitTestClass(PhysicsDllConcurrencyTests, Serial)
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
			SharedPhysicsContext().Release();
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
			auto external_device = SharedPhysicsTestDevice();
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
				if (!ok(api.EngineCreate(context, nullptr, external_device, &engine)))
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
						.collision_layer = 1,
						.collision_mask = 0xFFFFFFFFU,
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

	// Verify engine-owned atmosphere lifetime, asynchronous stepping, readback, and contract errors through the public ABI.
	PRUnitTestClass(PhysicsDllAtmosphereTests)
	{
		// Return a small closed box of still air with a few tracers, matching the native solver tests' scale.
		static AtmosphereDesc MakeAtmosphereDesc()
		{
			// All sides are solid with no drag; the mild lapse keeps buoyancy easy to observe in a few steps.
			return AtmosphereDesc{
				.header = {sizeof(AtmosphereDesc), PHYSICS_STRUCT_VERSION},
				.cell_count = { 8, 8, 4, 0 },
				.origin = { 0.0f, 0.0f, 0.0f, 1.0f },
				.dx = 1.0f,
				.lid_z = 4.0f,
				.first_layer_thickness = 1.0f,
				.floor_heights = nullptr,
				.active_columns = nullptr,
				.boundaries = {},
				.wall_drag = {},
				.reference_temperature = 288.0f,
				.lapse_rate = -0.001f,
				.adiabatic_lapse_rate = -0.00976f,
				.min_temperature = 250.0f,
				.gravity = 9.8f,
				.floor_exchange_rate = 0.5f,
				.lid_temperature = 284.0f,
				.lid_relaxation_rate = 0.05f,
				.pressure_vcycles = 2,
				.pressure_pre_smooth = 4,
				.pressure_post_smooth = 4,
				.pressure_coarse_smooth = 16,
				.open_edge_band = 2,
				.vorticity_confinement = 0.0f,
				.vertical_viscosity = 0.0f,
				.tracer_count = 64,
				.tracer_seed = 1,
				.tracer_max_age = 20.0f,
				.tracer_ground_density = 1.0f,
				.tracer_break_density = 1.0f,
				.tracer_upper_density = 1.0f,
				.tracer_break_height = 0.5f,
			};
		}

		// Return a step with one hot source in the middle of the box.
		static AtmosphereStepDesc MakeStepDesc(AtmosphereHeatSource const& source)
		{
			// The step borrows 'source' only for the duration of the call.
			return AtmosphereStepDesc{
				.header = {sizeof(AtmosphereStepDesc), PHYSICS_STRUCT_VERSION},
				.dt = 0.1f,
				.heat_source_count = 1,
				.heat_sources = &source,
			};
		}

		PRUnitTestMethod(LifecycleSteppingAndReadback, Extended)
		{
			auto fix = PhysicsFixture{};
			auto& api = fix.m_api;

			// Invalid descriptions are rejected without creating anything.
			auto atmosphere = AtmosphereHandle{};
			auto bad = MakeAtmosphereDesc();
			bad.dx = 0.0f;
			PR_EXPECT(api.AtmosphereCreate(fix.m_engine, &bad, &atmosphere) == EStatus::InvalidArgument);
			bad = MakeAtmosphereDesc();
			bad.header.version = 0;
			PR_EXPECT(api.AtmosphereCreate(fix.m_engine, &bad, &atmosphere) == EStatus::InvalidStruct);
			bad = MakeAtmosphereDesc();
			bad.boundaries[0] = static_cast<EAtmosphereBoundary>(7);
			PR_EXPECT(api.AtmosphereCreate(fix.m_engine, &bad, &atmosphere) == EStatus::InvalidArgument);

			// A valid atmosphere publishes its starting tracers straight away.
			auto desc = MakeAtmosphereDesc();
			PR_EXPECT(api.AtmosphereCreate(fix.m_engine, &desc, &atmosphere) == EStatus::Success);
			auto required = std::uint32_t{};
			PR_EXPECT(api.AtmosphereTracersCopy(fix.m_engine, atmosphere, nullptr, 0, &required) == EStatus::BufferTooSmall);
			PR_EXPECT(required == 64U);

			// Only one step may be in flight, and blocking reads wait for it to be finished.
			auto const source = AtmosphereHeatSource{ .centre = { 4.0f, 4.0f, 1.0f, 1.0f }, .radius = 1.5f, .heating_rate = 5.0f, .target_temperature = 320.0f, .relaxation_rate = 1.0f };
			auto step = MakeStepDesc(source);
			auto cells = std::vector<AtmosphereCellState>(8 * 8 * 4);
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::NoStepPending);
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::StepPending);
			PR_EXPECT(api.AtmosphereCellStatesCopy(fix.m_engine, atmosphere, cells.data(), static_cast<std::uint32_t>(cells.size()), &required, nullptr) == EStatus::StepPending);
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::Success);
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::NoStepPending);

			// Polling finishes a step once the GPU has run it, without blocking the caller.
			for (auto i = 0; i != 10; ++i)
			{
				PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
				auto idle = std::int32_t{};
				for (; idle == 0;)
				{
					PR_EXPECT(api.AtmospherePollStep(fix.m_engine, atmosphere, &idle) == EStatus::Success);
					if (idle == 0)
						std::this_thread::yield();
				}
			}

			// Tracer copies require the owner thread and report finite particles.
			auto particles = std::vector<AtmosphereTracerParticle>(64);
			auto copied = EStatus::InternalError;
			std::thread([&]
			{
				copied = api.AtmosphereTracersCopy(fix.m_engine, atmosphere, particles.data(), static_cast<std::uint32_t>(particles.size()), &required);
			}).join();
			PR_EXPECT(copied == EStatus::WrongThread);
			PR_EXPECT(api.AtmosphereTracersCopy(fix.m_engine, atmosphere, particles.data(), static_cast<std::uint32_t>(particles.size()), &required) == EStatus::Success);
			for (auto const& p : particles)
				PR_EXPECT(std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) && p.position.w == 1.0f && std::isfinite(p.temperature));

			// The published GPU slot can be held across steps, which write other slots, and is released without a fence when no GPU reads it.
			{
				auto slot = AtmosphereTracerSlot{};
				PR_EXPECT(api.AtmosphereTracersAcquire(fix.m_engine, atmosphere, nullptr) == EStatus::InvalidArgument);
				PR_EXPECT(api.AtmosphereTracersAcquire(fix.m_engine, atmosphere, &slot) == EStatus::Success);
				PR_EXPECT(slot.resource != nullptr && slot.count == 64U && slot.stride == sizeof(AtmosphereTracerParticle));
				for (auto i = 0; i != 6; ++i)
				{
					// Each step must find a free slot while one is held.
					PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
					PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::Success);
				}
				PR_EXPECT(api.AtmosphereTracersRelease(fix.m_engine, atmosphere, slot.slot, nullptr, 0) == EStatus::Success);
				PR_EXPECT(api.AtmosphereTracersRelease(fix.m_engine, atmosphere, slot.slot, nullptr, 0) == EStatus::InvalidArgument);
				static_cast<IUnknown*>(slot.resource)->Release();
			}

			// Heated air near the source is warmer than the reference, and the statistics describe a finite field.
			auto stats = AtmosphereStats{ .header = {sizeof(AtmosphereStats), PHYSICS_STRUCT_VERSION} };
			PR_EXPECT(api.AtmosphereCellStatesCopy(fix.m_engine, atmosphere, nullptr, 0, &required, nullptr) == EStatus::BufferTooSmall);
			PR_EXPECT(required == static_cast<std::uint32_t>(cells.size()));
			PR_EXPECT(api.AtmosphereCellStatesCopy(fix.m_engine, atmosphere, cells.data(), static_cast<std::uint32_t>(cells.size()), &required, &stats) == EStatus::Success);
			auto const source_cell = cells[(0 * 8 + 4) * 8 + 4];
			PR_EXPECT(source_cell.temperature > 288.5f);
			PR_EXPECT(std::isfinite(stats.max_speed) && stats.max_speed > 0.0f);

			// Raised floors remap the air; a wrong column count is rejected.
			auto floors = std::vector<float>(8 * 8, 0.0f);
			floors[0] = 1.0f;
			PR_EXPECT(api.AtmosphereFloorsSet(fix.m_engine, atmosphere, floors.data(), 3) == EStatus::InvalidArgument);
			PR_EXPECT(api.AtmosphereFloorsSet(fix.m_engine, atmosphere, floors.data(), static_cast<std::int32_t>(floors.size())) == EStatus::Success);

			// Persistent floor temperatures and outside air need one finite entry per column.
			auto floor_temperatures = std::vector<float>(8 * 8, 290.0f);
			auto outside_air = std::vector<AtmosphereOutsideAir>(8 * 8, AtmosphereOutsideAir{ .wind = { 1.0f, 0.0f } });
			PR_EXPECT(api.AtmosphereFloorTemperaturesSet(fix.m_engine, atmosphere, floor_temperatures.data(), 3) == EStatus::InvalidArgument);
			PR_EXPECT(api.AtmosphereFloorTemperaturesSet(fix.m_engine, atmosphere, floor_temperatures.data(), static_cast<std::int32_t>(floor_temperatures.size())) == EStatus::Success);
			PR_EXPECT(api.AtmosphereOutsideAirSet(fix.m_engine, atmosphere, outside_air.data(), 3) == EStatus::InvalidArgument);
			outside_air[0].reserved = 1.0f;
			PR_EXPECT(api.AtmosphereOutsideAirSet(fix.m_engine, atmosphere, outside_air.data(), static_cast<std::int32_t>(outside_air.size())) == EStatus::InvalidArgument);
			outside_air[0].reserved = 0.0f;
			PR_EXPECT(api.AtmosphereOutsideAirSet(fix.m_engine, atmosphere, outside_air.data(), static_cast<std::int32_t>(outside_air.size())) == EStatus::Success);

			// Atmospheres are not part of checkpoints, so importing one would leave them inconsistent with the new engine.
			auto checkpoint = WriteCheckpoint(api, fix.m_engine);
			PR_EXPECT(api.CheckpointRead(fix.m_engine, checkpoint.data(), checkpoint.size()) == EStatus::InvalidArgument);

			// Destroying the atmosphere makes its handle stale and frees the slot for reuse.
			PR_EXPECT(api.AtmosphereDestroy(fix.m_engine, atmosphere) == EStatus::Success);
			PR_EXPECT(api.AtmosphereTracersCopy(fix.m_engine, atmosphere, particles.data(), 64, &required) == EStatus::StaleHandle);
			PR_EXPECT(api.AtmosphereDestroy(fix.m_engine, atmosphere) == EStatus::StaleHandle);
		}
		PRUnitTestMethod(EngineDestroyWaitsForStepInFlight, Extended)
		{
			auto fix = PhysicsFixture{};
			auto& api = fix.m_api;

			// Field-only atmospheres need no tracers.
			auto desc = MakeAtmosphereDesc();
			desc.tracer_count = 0;
			auto atmosphere = AtmosphereHandle{};
			PR_EXPECT(api.AtmosphereCreate(fix.m_engine, &desc, &atmosphere) == EStatus::Success);
			auto required = std::uint32_t{ 1 };
			PR_EXPECT(api.AtmosphereTracersCopy(fix.m_engine, atmosphere, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0U);

			// The engine owns the atmosphere and drains its pending step on destruction.
			auto const source = AtmosphereHeatSource{ .centre = { 4.0f, 4.0f, 1.0f, 1.0f }, .radius = 1.5f, .heating_rate = 5.0f, .target_temperature = 320.0f, .relaxation_rate = 1.0f };
			auto step = MakeStepDesc(source);
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
			PR_EXPECT(api.EngineDestroy(fix.m_engine) == EStatus::Success);
			auto idle = std::int32_t{};
			PR_EXPECT(api.AtmospherePollStep(fix.m_engine, atmosphere, &idle) == EStatus::StaleHandle);
			fix.m_engine = 0;
		}
		PRUnitTestMethod(ProbesSampleAfterEachStep, Extended)
		{
			auto fix = PhysicsFixture{};
			auto& api = fix.m_api;
			auto desc = MakeAtmosphereDesc();
			desc.tracer_count = 0;
			auto atmosphere = AtmosphereHandle{};
			PR_EXPECT(api.AtmosphereCreate(fix.m_engine, &desc, &atmosphere) == EStatus::Success);

			// No step has finished, so there are no samples.
			auto required = std::uint32_t{ 1 };
			PR_EXPECT(api.AtmosphereProbesCopy(fix.m_engine, atmosphere, nullptr, 0, nullptr) == EStatus::InvalidArgument);
			PR_EXPECT(api.AtmosphereProbesCopy(fix.m_engine, atmosphere, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0U);

			// Invalid probe lists are rejected without starting a step.
			auto const source = AtmosphereHeatSource{ .centre = { 4.0f, 4.0f, 1.0f, 1.0f }, .radius = 1.5f, .heating_rate = 5.0f, .target_temperature = 320.0f, .relaxation_rate = 1.0f };
			auto points = std::vector<Vector4>(65, Vector4{ 4.0f, 4.0f, 2.0f, 1.0f });
			points[1] = Vector4{ -5.0f, 4.0f, 2.0f, 1.0f };
			auto step = MakeStepDesc(source);
			step.probe_count = 65;
			step.probes = points.data();
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::InvalidArgument);
			step.probe_count = 2;
			step.probes = nullptr;
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::InvalidArgument);
			step.probes = points.data();
			points[0].w = 0.0f;
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::InvalidArgument);
			points[0].w = 1.0f;
			points[0].z = std::numeric_limits<float>::quiet_NaN();
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::InvalidArgument);
			points[0].z = 2.0f;

			// A valid step publishes one sample per probe once it finishes, and the samples stay readable while the next step is in flight.
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
			PR_EXPECT(api.AtmosphereProbesCopy(fix.m_engine, atmosphere, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0U);
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::Success);
			auto samples = std::vector<AtmosphereProbeSample>(2);
			PR_EXPECT(api.AtmosphereProbesCopy(fix.m_engine, atmosphere, samples.data(), 1, &required) == EStatus::BufferTooSmall);
			PR_EXPECT(required == 2U);
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
			PR_EXPECT(api.AtmosphereProbesCopy(fix.m_engine, atmosphere, samples.data(), 2, &required) == EStatus::Success);
			PR_EXPECT(samples[0].inside == 1 && std::isfinite(samples[0].velocity.x) && samples[0].temperature > 250.0f && samples[0].temperature < 330.0f);
			PR_EXPECT(samples[1].inside == 0 && samples[1].temperature == 0.0f);

			// A step without probes publishes an empty set.
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::Success);
			step.probe_count = 0;
			step.probes = nullptr;
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::Success);
			PR_EXPECT(api.AtmosphereProbesCopy(fix.m_engine, atmosphere, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0U);
			PR_EXPECT(api.AtmosphereDestroy(fix.m_engine, atmosphere) == EStatus::Success);
		}
		PRUnitTestMethod(SurfaceWindSamplesEveryColumn, Extended)
		{
			auto fix = PhysicsFixture{};
			auto& api = fix.m_api;
			auto desc = MakeAtmosphereDesc();
			desc.tracer_count = 0;
			auto atmosphere = AtmosphereHandle{};
			PR_EXPECT(api.AtmosphereCreate(fix.m_engine, &desc, &atmosphere) == EStatus::Success);

			// No step has finished, so there is no surface wind.
			auto required = std::uint32_t{ 1 };
			PR_EXPECT(api.AtmosphereSurfaceWindCopy(fix.m_engine, atmosphere, nullptr, 0, nullptr) == EStatus::InvalidArgument);
			PR_EXPECT(api.AtmosphereSurfaceWindCopy(fix.m_engine, atmosphere, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0U);

			// Negative and non-finite heights are rejected without starting a step.
			auto const source = AtmosphereHeatSource{ .centre = { 4.0f, 4.0f, 1.0f, 1.0f }, .radius = 1.5f, .heating_rate = 5.0f, .target_temperature = 320.0f, .relaxation_rate = 1.0f };
			auto step = MakeStepDesc(source);
			step.surface_wind_height = -1.0f;
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::InvalidArgument);
			step.surface_wind_height = std::numeric_limits<float>::quiet_NaN();
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::InvalidArgument);

			// A sampling step publishes one finite wind per column once it finishes.
			step.surface_wind_height = 1.5f;
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::Success);
			auto wind = std::vector<Vector2>(8 * 8);
			PR_EXPECT(api.AtmosphereSurfaceWindCopy(fix.m_engine, atmosphere, wind.data(), 1, &required) == EStatus::BufferTooSmall);
			PR_EXPECT(required == 64U);
			PR_EXPECT(api.AtmosphereSurfaceWindCopy(fix.m_engine, atmosphere, wind.data(), static_cast<std::uint32_t>(wind.size()), &required) == EStatus::Success);
			for (auto const& w : wind)
				PR_EXPECT(std::isfinite(w.x) && std::isfinite(w.y));

			// A step that does not sample publishes an empty field.
			step.surface_wind_height = 0.0f;
			PR_EXPECT(api.AtmosphereBeginStep(fix.m_engine, atmosphere, &step) == EStatus::Success);
			PR_EXPECT(api.AtmosphereCompleteStep(fix.m_engine, atmosphere) == EStatus::Success);
			PR_EXPECT(api.AtmosphereSurfaceWindCopy(fix.m_engine, atmosphere, nullptr, 0, &required) == EStatus::Success);
			PR_EXPECT(required == 0U);
			PR_EXPECT(api.AtmosphereDestroy(fix.m_engine, atmosphere) == EStatus::Success);
		}
		PRUnitTestMethod(ShaderCacheStoresAndReusesKernels, Extended)
		{
			auto fix = PhysicsFixture{};
			auto& api = fix.m_api;
			auto const cache_dir = std::filesystem::temp_directory_path() / std::format("physics-shader-cache-{}", GetCurrentProcessId());
			std::filesystem::remove_all(cache_dir);

			// Count the files currently stored in the cache directory.
			auto count_files = [&]
			{
				auto count = 0;
				for (auto const& entry : std::filesystem::recursive_directory_iterator(cache_dir))
					count += entry.is_regular_file() ? 1 : 0;

				return count;
			};

			// Create an engine and an atmosphere, so that both compile their kernels through the current cache.
			auto create_engine_and_atmosphere = [&]
			{
				auto engine = EngineHandle{};
				PR_EXPECT(api.EngineCreate(fix.m_context, nullptr, fix.m_external_device, &engine) == EStatus::Success);
				auto desc = MakeAtmosphereDesc();
				auto atmosphere = AtmosphereHandle{};
				PR_EXPECT(api.AtmosphereCreate(engine, &desc, &atmosphere) == EStatus::Success);
				PR_EXPECT(api.EngineDestroy(engine) == EStatus::Success);
			};

			// Contract errors leave the current cache unchanged.
			PR_EXPECT(api.ShaderCacheDirectorySet(nullptr, cache_dir.c_str()) == EStatus::InvalidHandle);
			PR_EXPECT(api.ShaderCacheDirectorySet(fix.m_context, L"") == EStatus::InvalidArgument);

			// The first creation fills the cache, and an identical second creation reuses it without adding entries.
			PR_EXPECT(api.ShaderCacheDirectorySet(fix.m_context, cache_dir.c_str()) == EStatus::Success);
			create_engine_and_atmosphere();
			auto const cached = count_files();
			PR_EXPECT(cached > 0);
			create_engine_and_atmosphere();
			PR_EXPECT(count_files() == cached);

			// The cache is shared by the whole process, so restore the uncached default for later tests.
			PR_EXPECT(api.ShaderCacheDirectorySet(fix.m_context, nullptr) == EStatus::Success);
			std::filesystem::remove_all(cache_dir);
		}
		PRUnitTestMethod(ShaderPrecompileFillsCacheForEngineCreation, Extended)
		{
			auto fix = PhysicsFixture{};
			auto& api = fix.m_api;
			auto const cache_dir = std::filesystem::temp_directory_path() / std::format("physics-shader-precompile-{}", GetCurrentProcessId());
			std::filesystem::remove_all(cache_dir);

			// Count the compiled kernel entries in the cache. The pipeline library file is excluded because only device users write it.
			auto count_kernels = [&]
			{
				auto count = 0;
				if (std::filesystem::exists(cache_dir))
				{
					for (auto const& entry : std::filesystem::recursive_directory_iterator(cache_dir))
						count += entry.is_regular_file() && entry.path().filename() == L"manifest.bin" ? 1 : 0;
				}
				return count;
			};

			// Record every progress report so the sequence can be checked after the call.
			struct Progress
			{
				std::vector<std::pair<int, int>> m_reports;
				int m_cancel_after = -1;

				static int __stdcall Report(void* ctx, int completed, int total)
				{
					// Continue until the requested number of completions has been reported.
					auto& self = *static_cast<Progress*>(ctx);
					self.m_reports.push_back({ completed, total });
					return self.m_cancel_after < 0 || completed < self.m_cancel_after ? 1 : 0;
				}
			};

			// Contract errors: invalid context, no worker threads, and no cache.
			PR_EXPECT(api.ShaderPrecompile(nullptr, 1, nullptr, nullptr) == EStatus::InvalidHandle);
			PR_EXPECT(api.ShaderPrecompile(fix.m_context, 0, nullptr, nullptr) == EStatus::InvalidArgument);
			PR_EXPECT(api.ShaderPrecompile(fix.m_context, 1, nullptr, nullptr) == EStatus::InvalidArgument);

			// Cancelling at the first report compiles nothing and still succeeds.
			PR_EXPECT(api.ShaderCacheDirectorySet(fix.m_context, cache_dir.c_str()) == EStatus::Success);
			auto cancelled = Progress{ .m_cancel_after = 0 };
			PR_EXPECT(api.ShaderPrecompile(fix.m_context, 4, &Progress::Report, &cancelled) == EStatus::Success);
			PR_EXPECT(cancelled.m_reports.size() == 1U && cancelled.m_reports[0].first == 0);
			PR_EXPECT(count_kernels() == 0);

			// A full run reports increasing progress that ends at the total, and stores one entry per kernel.
			auto progress = Progress{};
			PR_EXPECT(api.ShaderPrecompile(fix.m_context, 4, &Progress::Report, &progress) == EStatus::Success);
			PR_EXPECT(!progress.m_reports.empty());
			auto const total = progress.m_reports.back().second;
			PR_EXPECT(total > 0 && progress.m_reports.back().first == total);
			for (size_t i = 1; i < progress.m_reports.size(); ++i)
				PR_EXPECT(progress.m_reports[i].first > progress.m_reports[i - 1].first);

			PR_EXPECT(count_kernels() == total);

			// Engine and atmosphere creation find every kernel they compile already in the cache.
			auto engine = EngineHandle{};
			PR_EXPECT(api.EngineCreate(fix.m_context, nullptr, fix.m_external_device, &engine) == EStatus::Success);
			auto desc = MakeAtmosphereDesc();
			auto atmosphere = AtmosphereHandle{};
			PR_EXPECT(api.AtmosphereCreate(engine, &desc, &atmosphere) == EStatus::Success);
			PR_EXPECT(api.EngineDestroy(engine) == EStatus::Success);
			PR_EXPECT(count_kernels() == total);

			// The cache is shared by the whole process, so restore the uncached default for later tests.
			PR_EXPECT(api.ShaderCacheDirectorySet(fix.m_context, nullptr) == EStatus::Success);
			std::filesystem::remove_all(cache_dir);
		}
	};
}
