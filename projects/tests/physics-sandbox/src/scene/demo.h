#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	enum class EDemoCategory
	{
		DynamicsAndConservation,
		CollisionAndContacts,
		StacksAndStability,
		ConstraintsAndMechanisms,
		ArticulationsAndRobotics,
		Buoyancy,
		StressAndScaling,
		FeatureShowcases,
	};

	// Metadata for one grouped demonstration menu.
	struct DemoCategoryInfo
	{
		EDemoCategory m_category;
		std::string_view m_display_name;
	};

	// Runtime metadata and deployed path for one JSON demonstration.
	struct DemoInfo
	{
		EDemoCategory m_category;
		std::string m_name;
		std::string m_display_name;
		std::string m_description;
		int m_order;
		std::filesystem::path m_filepath;
	};

	// Return demonstration categories in stable menu order.
	std::span<DemoCategoryInfo const> DemoCategoryCatalogue();

	// Discover deployed JSON demonstrations and return them in stable grouped order.
	std::span<DemoInfo const> DemoCatalogue();

	// Resolve a case-insensitive command-line name, accepting '-' and '_' interchangeably.
	DemoInfo const* FindDemo(std::string_view name);
}
