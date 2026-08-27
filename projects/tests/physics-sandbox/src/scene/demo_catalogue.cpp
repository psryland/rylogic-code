#include "src/scene/demo.h"

namespace physics_sandbox
{
	namespace
	{
		constexpr auto Categories = std::array{
			DemoCategoryInfo{EDemoCategory::DynamicsAndConservation, "Dynamics and Conservation"},
			DemoCategoryInfo{EDemoCategory::CollisionAndContacts, "Collision and Contacts"},
			DemoCategoryInfo{EDemoCategory::StacksAndStability, "Stacks and Stability"},
			DemoCategoryInfo{EDemoCategory::ConstraintsAndMechanisms, "Constraints and Mechanisms"},
			DemoCategoryInfo{EDemoCategory::ArticulationsAndRobotics, "Articulations and Robotics"},
			DemoCategoryInfo{EDemoCategory::Buoyancy, "Buoyancy"},
			DemoCategoryInfo{EDemoCategory::StressAndScaling, "Stress and Scaling"},
		};

		constexpr auto Demos = std::array{
			DemoInfo{EDemo::Pendulum, EDemoCategory::ConstraintsAndMechanisms, "pendulum", "Single Pendulum", "A passive physical pendulum with an analytic small-angle period."},
			DemoInfo{EDemo::RigidJointGallery, EDemoCategory::ConstraintsAndMechanisms, "rigid-joints", "Rigid Joint Gallery", "Ball, hinge, slider, weld, motor, and breakable D6 constraints."},
			DemoInfo{EDemo::RigidChain, EDemoCategory::ConstraintsAndMechanisms, "rigid-chain", "Long Rigid Chain", "A long graph-coloured rigid chain under gravity."},
			DemoInfo{EDemo::FourBarLinkage, EDemoCategory::ConstraintsAndMechanisms, "four-bar", "Four-Bar Closed Loop", "A planar closed-loop mechanism that cannot be represented by a tree alone."},
			DemoInfo{EDemo::FixedArticulations, EDemoCategory::ArticulationsAndRobotics, "fixed-articulations", "Fixed Articulations", "Several fixed-root Featherstone chains with different link counts."},
			DemoInfo{EDemo::Ragdolls, EDemoCategory::ArticulationsAndRobotics, "ragdolls", "Ragdoll Drop", "Many small branched floating articulations colliding with each other and the ground."},
			DemoInfo{EDemo::RobotMotors, EDemoCategory::ArticulationsAndRobotics, "robot-motors", "Robot Motors and Limits", "Reduced-coordinate links controlled by persistent driven and limited rows."},
			DemoInfo{EDemo::RobotGripper, EDemoCategory::ArticulationsAndRobotics, "robot-gripper", "Robot Arm and Gripper", "Joint motors, prismatic fingers, manipulation contact, and feedback through one tree."},
			DemoInfo{EDemo::VehicleSuspension, EDemoCategory::ConstraintsAndMechanisms, "vehicle-suspension", "Vehicle Suspension", "Driven compliant suspension rows, travel limits, and wheel contacts."},
			DemoInfo{EDemo::SuspensionBridge, EDemoCategory::ConstraintsAndMechanisms, "suspension-bridge", "Suspension Bridge", "A cyclic rigid constraint graph with pinned corners and distributed load."},
			DemoInfo{EDemo::MixedCoupling, EDemoCategory::ArticulationsAndRobotics, "mixed-coupling", "Mixed Rigid/Tree Coupling", "Rigid-to-tree and direct tree-to-tree persistent constraints."},
			DemoInfo{EDemo::ArticulationPush, EDemoCategory::ArticulationsAndRobotics, "articulation-push", "Articulation Pushes Stack", "A driven reduced-coordinate link transfers momentum into an ordinary rigid stack."},
			DemoInfo{EDemo::TwoRobotLoad, EDemoCategory::ArticulationsAndRobotics, "two-robot-load", "Two Robots Carrying a Load", "Two independent articulations support one shared constrained rigid payload."},
			DemoInfo{EDemo::MixedContacts, EDemoCategory::CollisionAndContacts, "mixed-contacts", "Mixed and Self Contacts", "Rigid/tree, tree/tree, and non-adjacent same-tree collision response."},
			DemoInfo{EDemo::BuoyantArticulation, EDemoCategory::Buoyancy, "buoyant-articulation", "Buoyant Articulation", "GPU buoyancy applied independently to the links of a floating tree."},
			DemoInfo{EDemo::FloatingConservation, EDemoCategory::DynamicsAndConservation, "floating-conservation", "Floating Conservation", "Force-free internal motion preserving Coriolis effects, momentum, and bounded energy."},
			DemoInfo{EDemo::Dzhanibekov, EDemoCategory::DynamicsAndConservation, "dzhanibekov", "Dzhanibekov Effect (Rigid and Articulated)", "Rigid and floating-root intermediate-axis instability side by side."},
			DemoInfo{EDemo::ConstraintPathologies, EDemoCategory::StressAndScaling, "constraint-pathologies", "Constraint Pathologies", "Redundant rows, extreme mass ratios, and near-singular closed loops."},
			DemoInfo{EDemo::ConstraintStress, EDemoCategory::StressAndScaling, "constraint-stress", "Constraint Stress Grid", "A cyclic two-dimensional constraint graph used to measure scaling."},
		};

		constexpr auto SceneDemos = std::array{
			SceneDemoInfo{EDemoCategory::CollisionAndContacts, "all_shapes_drop.json", "All Shape Types"},
			SceneDemoInfo{EDemoCategory::DynamicsAndConservation, "angular_integration_stress.json", "Angular Integration"},
			SceneDemoInfo{EDemoCategory::CollisionAndContacts, "box_vs_sphere.json", "Box vs Sphere"},
			SceneDemoInfo{EDemoCategory::StacksAndStability, "brick_pyramid.json", "Brick Pyramid"},
			SceneDemoInfo{EDemoCategory::StressAndScaling, "brick_wall.json", "Brick Wall"},
			SceneDemoInfo{EDemoCategory::StressAndScaling, "brick_wall_2000.json", "Brick Wall (2,000 Bricks)"},
			SceneDemoInfo{EDemoCategory::Buoyancy, "buoyancy_stress_1000.json", "Buoyancy Stress (1,000 Bodies)"},
			SceneDemoInfo{EDemoCategory::CollisionAndContacts, "drop_test.json", "Single Box Drop"},
			SceneDemoInfo{EDemoCategory::DynamicsAndConservation, "dzhanibekov_effect.json", "Dzhanibekov Effect (Rigid Body)"},
			SceneDemoInfo{EDemoCategory::Buoyancy, "floater.json", "Floating Shapes"},
			SceneDemoInfo{EDemoCategory::StressAndScaling, "generated_stress.json", "Generated Body Stress"},
			SceneDemoInfo{EDemoCategory::CollisionAndContacts, "gravity_playground.json", "Gravity and Shape Playground"},
			SceneDemoInfo{EDemoCategory::StacksAndStability, "newtons_cradle.json", "Shock Propagation"},
			SceneDemoInfo{EDemoCategory::CollisionAndContacts, "oblique_spheres.json", "Oblique Sphere Collision"},
			SceneDemoInfo{EDemoCategory::StressAndScaling, "pancake.json", "Large Column Stack"},
			SceneDemoInfo{EDemoCategory::StacksAndStability, "plate_drop.json", "Thin Plate Settling"},
			SceneDemoInfo{EDemoCategory::CollisionAndContacts, "polytope_drop.json", "Polytope Drop"},
			SceneDemoInfo{EDemoCategory::StressAndScaling, "simultaneous_impact_1000.json", "Simultaneous Impact (1,000 Bodies)"},
			SceneDemoInfo{EDemoCategory::DynamicsAndConservation, "spinning_tops.json", "Spinning Tops"},
			SceneDemoInfo{EDemoCategory::StacksAndStability, "stacked_column.json", "Stacked Column"},
			SceneDemoInfo{EDemoCategory::StressAndScaling, "stress_test_1000.json", "Mixed Shape Stress (1,000 Bodies)"},
			SceneDemoInfo{EDemoCategory::CollisionAndContacts, "three_body_chain.json", "Three-Body Collision Chain"},
		};

		// Compare command-line names without making punctuation or ASCII case significant.
		bool EquivalentName(std::string_view lhs, std::string_view rhs)
		{
			if (lhs.size() != rhs.size())
				return false;

			for (auto index = 0; index != isize(lhs); ++index)
			{
				auto const normalise = [](char ch)
				{
					return ch == '_' ? '-' : static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
				};
				if (normalise(lhs[index]) != normalise(rhs[index]))
					return false;
			}
			return true;
		}
	}

	// Return the stable behavior-oriented category order used by the demonstration menu.
	std::span<DemoCategoryInfo const> DemoCategoryCatalogue()
	{
		return Categories;
	}

	// Return the complete programmatic demonstration catalogue in stable menu order.
	std::span<DemoInfo const> DemoCatalogue()
	{
		return Demos;
	}

	// Return the complete file-backed scene catalogue in stable menu order.
	std::span<SceneDemoInfo const> SceneDemoCatalogue()
	{
		return SceneDemos;
	}

	// Return metadata for one programmatic demonstration.
	DemoInfo const& GetDemoInfo(EDemo demo)
	{
		auto const iter = std::ranges::find(Demos, demo, &DemoInfo::m_demo);
		if (iter == Demos.end())
			throw std::runtime_error("Unknown physics demonstration");

		return *iter;
	}

	// Return the deployed JSON path for one file-backed scene demonstration.
	std::filesystem::path SceneDemoPath(SceneDemoInfo const& demo)
	{
		return win32::ExeDir() / "scenes" / demo.m_filename;
	}

	// Resolve a case-insensitive command-line name, accepting '-' and '_' interchangeably.
	std::optional<EDemo> FindDemo(std::string_view name)
	{
		auto const iter = std::ranges::find_if(Demos, [&](DemoInfo const& info)
		{
			return EquivalentName(name, info.m_name);
		});
		return iter != Demos.end() ? std::optional(iter->m_demo) : std::nullopt;
	}
}
