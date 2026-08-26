#include "src/scene/demo.h"
#include "src/scene/demo_builder.h"

namespace physics_sandbox
{
	namespace
	{
		constexpr auto Demos = std::array{
			DemoInfo{EDemo::Pendulum, "pendulum", "Single Pendulum", "A passive physical pendulum with an analytic small-angle period."},
			DemoInfo{EDemo::RigidJointGallery, "rigid-joints", "Rigid Joint Gallery", "Ball, hinge, slider, weld, motor, and breakable D6 constraints."},
			DemoInfo{EDemo::RigidChain, "rigid-chain", "Long Rigid Chain", "A long graph-coloured rigid chain under gravity."},
			DemoInfo{EDemo::FourBarLinkage, "four-bar", "Four-Bar Closed Loop", "A planar closed-loop mechanism that cannot be represented by a tree alone."},
			DemoInfo{EDemo::FixedArticulations, "fixed-articulations", "Fixed Articulations", "Several fixed-root Featherstone chains with different link counts."},
			DemoInfo{EDemo::Ragdolls, "ragdolls", "Ragdoll Drop", "Many small branched floating articulations colliding with each other and the ground."},
			DemoInfo{EDemo::RobotMotors, "robot-motors", "Robot Motors and Limits", "Reduced-coordinate links controlled by persistent driven and limited rows."},
			DemoInfo{EDemo::RobotGripper, "robot-gripper", "Robot Arm and Gripper", "Joint motors, prismatic fingers, manipulation contact, and feedback through one tree."},
			DemoInfo{EDemo::VehicleSuspension, "vehicle-suspension", "Vehicle Suspension", "Driven compliant suspension rows, travel limits, and wheel contacts."},
			DemoInfo{EDemo::SuspensionBridge, "suspension-bridge", "Suspension Bridge", "A cyclic rigid constraint graph with pinned corners and distributed load."},
			DemoInfo{EDemo::MixedCoupling, "mixed-coupling", "Mixed Rigid/Tree Coupling", "Rigid-to-tree and direct tree-to-tree persistent constraints."},
			DemoInfo{EDemo::ArticulationPush, "articulation-push", "Articulation Pushes Stack", "A driven reduced-coordinate link transfers momentum into an ordinary rigid stack."},
			DemoInfo{EDemo::TwoRobotLoad, "two-robot-load", "Two Robots Carrying a Load", "Two independent articulations support one shared constrained rigid payload."},
			DemoInfo{EDemo::MixedContacts, "mixed-contacts", "Mixed and Self Contacts", "Rigid/tree, tree/tree, and non-adjacent same-tree collision response."},
			DemoInfo{EDemo::BuoyantArticulation, "buoyant-articulation", "Buoyant Articulation", "GPU buoyancy applied independently to the links of a floating tree."},
			DemoInfo{EDemo::FloatingConservation, "floating-conservation", "Floating Conservation", "Force-free internal motion preserving Coriolis effects, momentum, and bounded energy."},
			DemoInfo{EDemo::Dzhanibekov, "dzhanibekov", "Dzhanibekov Effect", "Rigid and floating-root intermediate-axis instability side by side."},
			DemoInfo{EDemo::ConstraintPathologies, "constraint-pathologies", "Constraint Pathologies", "Redundant rows, extreme mass ratios, and near-singular closed loops."},
			DemoInfo{EDemo::ConstraintStress, "constraint-stress", "Constraint Stress Grid", "A cyclic two-dimensional constraint graph used to measure scaling."},
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

	// Return the complete demonstration catalogue in stable menu order.
	std::span<DemoInfo const> DemoCatalogue()
	{
		return Demos;
	}

	// Return metadata for one demonstration.
	DemoInfo const& GetDemoInfo(EDemo demo)
	{
		auto const iter = std::ranges::find(Demos, demo, &DemoInfo::m_demo);
		if (iter == Demos.end())
			throw std::runtime_error("Unknown physics demonstration");

		return *iter;
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
