#include "src/scene/demo.h"
#include "src/scene/demo_builder.h"

namespace physics_sandbox
{
	// Populate a clean scene with one programmatic demonstration.
	void BuildDemo(Scene& scene, EDemo demo_id)
	{
		auto demo = DemoBuilder(scene);
		switch (demo_id)
		{
			case EDemo::Pendulum:               { BuildPendulum(demo); break; }
			case EDemo::RigidJointGallery:      { BuildRigidJointGallery(demo); break; }
			case EDemo::RigidChain:             { BuildRigidChain(demo); break; }
			case EDemo::FourBarLinkage:         { BuildFourBarLinkage(demo); break; }
			case EDemo::FixedArticulations:     { BuildFixedArticulations(demo); break; }
			case EDemo::Ragdolls:               { BuildRagdolls(demo); break; }
			case EDemo::RobotMotors:            { BuildRobotMotors(demo); break; }
			case EDemo::RobotGripper:           { BuildRobotGripper(demo); break; }
			case EDemo::VehicleSuspension:      { BuildVehicleSuspension(demo); break; }
			case EDemo::SuspensionBridge:       { BuildSuspensionBridge(demo); break; }
			case EDemo::MixedCoupling:          { BuildMixedCoupling(demo); break; }
			case EDemo::ArticulationPush:       { BuildArticulationPush(demo); break; }
			case EDemo::TwoRobotLoad:           { BuildTwoRobotLoad(demo); break; }
			case EDemo::MixedContacts:          { BuildMixedContacts(demo); break; }
			case EDemo::BuoyantArticulation:    { BuildBuoyantArticulation(demo); break; }
			case EDemo::FloatingConservation:   { BuildFloatingConservation(demo); break; }
			case EDemo::Dzhanibekov:            { BuildDzhanibekov(demo); break; }
			case EDemo::ConstraintPathologies:  { BuildConstraintPathologies(demo); break; }
			case EDemo::ConstraintStress:       { BuildConstraintStress(demo); break; }
			default:                            { throw std::runtime_error("Unknown physics demonstration"); }
		}
		demo.Finalise();
	}
}
