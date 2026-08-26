#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	struct Scene;

	// Identifies one programmatically constructed constraint or articulation demonstration.
	enum class EDemo
	{
		Pendulum,
		RigidJointGallery,
		RigidChain,
		FourBarLinkage,
		FixedArticulations,
		Ragdolls,
		RobotMotors,
		RobotGripper,
		VehicleSuspension,
		SuspensionBridge,
		MixedCoupling,
		ArticulationPush,
		TwoRobotLoad,
		MixedContacts,
		BuoyantArticulation,
		FloatingConservation,
		Dzhanibekov,
		ConstraintPathologies,
		ConstraintStress,
	};

	// Describes one stable command-line and user-interface entry in the demonstration catalogue.
	struct DemoInfo
	{
		EDemo m_demo;
		std::string_view m_name;
		std::string_view m_display_name;
		std::string_view m_description;
	};

	// Return the complete demonstration catalogue in stable menu order.
	std::span<DemoInfo const> DemoCatalogue();

	// Return metadata for one demonstration.
	DemoInfo const& GetDemoInfo(EDemo demo);

	// Resolve a case-insensitive command-line name, accepting '-' and '_' interchangeably.
	std::optional<EDemo> FindDemo(std::string_view name);

	// Populate a clean scene with one programmatic demonstration.
	void BuildDemo(Scene& scene, EDemo demo);
}
