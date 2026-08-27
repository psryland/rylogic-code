#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	struct Scene;

	// Identifies the physical behaviour demonstrated by one menu entry.
	enum class EDemoCategory
	{
		DynamicsAndConservation,
		CollisionAndContacts,
		StacksAndStability,
		ConstraintsAndMechanisms,
		ArticulationsAndRobotics,
		Buoyancy,
		StressAndScaling,
	};

	// Describes one stable demonstration-menu category.
	struct DemoCategoryInfo
	{
		EDemoCategory m_category;
		std::string_view m_display_name;
	};

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
		EDemoCategory m_category;
		std::string_view m_name;
		std::string_view m_display_name;
		std::string_view m_description;
	};

	// Describes one file-backed scene exposed through the demonstration menu.
	struct SceneDemoInfo
	{
		EDemoCategory m_category;
		std::string_view m_filename;
		std::string_view m_display_name;
	};

	// Return the stable behavior-oriented category order used by the demonstration menu.
	std::span<DemoCategoryInfo const> DemoCategoryCatalogue();

	// Return the complete demonstration catalogue in stable menu order.
	std::span<DemoInfo const> DemoCatalogue();

	// Return the complete file-backed scene catalogue in stable menu order.
	std::span<SceneDemoInfo const> SceneDemoCatalogue();

	// Return metadata for one demonstration.
	DemoInfo const& GetDemoInfo(EDemo demo);

	// Return the deployed JSON path for one file-backed scene demonstration.
	std::filesystem::path SceneDemoPath(SceneDemoInfo const& demo);

	// Resolve a case-insensitive command-line name, accepting '-' and '_' interchangeably.
	std::optional<EDemo> FindDemo(std::string_view name);

	// Populate a clean scene with one programmatic demonstration.
	void BuildDemo(Scene& scene, EDemo demo);
}
