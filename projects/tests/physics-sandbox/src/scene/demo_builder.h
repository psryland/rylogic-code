#pragma once
#include "src/scene/scene.h"

namespace physics_sandbox
{
	// Return a constraint frame whose local X axis follows a selected world-space axis.
	m4x4 ConstraintAxisFrame(v4 axis, v4 position);

	// Shared construction surface for compact programmatic physics demonstrations.
	class DemoBuilder
	{
		// One deferred articulation-link buoyancy registration created after vector storage is stable.
		struct BuoyantLink
		{
			int m_articulation_index;
			physics::LinkHandle m_link;
		};

		Scene& m_scene;
		std::vector<BuoyantLink> m_buoyant_links;

		// Build all rigid-body and articulation-link renderer objects in one LDraw parse.
		void BuildGraphics();

		// Create the optional articulation-only buoyancy module after all articulation addresses are stable.
		void BuildBuoyancy();

	public:
		// Begin populating a scene that has already released its previous simulation objects.
		explicit DemoBuilder(Scene& scene);

		// Return the scene being populated for deliberate initial-state adjustments.
		Scene& SceneState();

		// Store a box collision shape for the complete lifetime of the demonstration.
		collision::ShapeBox const& BoxShape(v4 dimensions);

		// Store a spherical collision shape for the complete lifetime of the demonstration.
		collision::ShapeSphere const& SphereShape(float radius);

		// Add a box-shaped rigid body and return its stable scene index.
		int AddBox(v4 dimensions, m4x4 const& object_to_world, float mass, Colour32 colour);

		// Add a spherical rigid body and return its stable scene index.
		int AddSphere(float radius, m4x4 const& object_to_world, float mass, Colour32 colour);

		// Add a box aligned between two world-space points and return its stable scene index.
		int AddRod(v4 point_a, v4 point_b, float thickness, float mass, Colour32 colour);

		// Add a broad static floor without introducing a separate rendering path.
		int AddGround(float half_extent = 20.0f, float height = 0.0f);

		// Return a shaped articulation link with matching box mass properties.
		physics::ArticulationLinkDesc BoxLink(v4 dimensions, float mass, bool collide_parent = false, bool collide_self = true);

		// Return a shaped articulation link with matching spherical mass properties.
		physics::ArticulationLinkDesc SphereLink(float radius, float mass, bool collide_parent = false, bool collide_self = true);

		// Transfer one completed reduced-coordinate tree into scene ownership.
		int AddArticulation(physics::Articulation articulation);

		// Return one world endpoint frame expressed directly in world coordinates.
		physics::BodyFrame WorldFrame(m4x4 const& constraint_to_world) const;

		// Return one rigid-body endpoint frame corresponding to a shared world frame.
		physics::BodyFrame BodyFrame(int body_index, m4x4 const& constraint_to_world) const;

		// Return one articulation-link endpoint frame corresponding to a shared world frame.
		physics::BodyFrame LinkFrame(int articulation_index, physics::LinkHandle link, m4x4 const& constraint_to_world) const;

		// Add a ball-and-socket constraint between two prepared endpoint frames.
		physics::ConstraintHandle AddBall(physics::BodyFrame frame_a, physics::BodyFrame frame_b, bool collide_connected = false);

		// Add a hinge constraint whose free, limited, or driven axis is frame-local X.
		physics::ConstraintHandle AddHinge(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis, bool collide_connected = false);

		// Add a slider constraint whose free, limited, or driven axis is frame-local X.
		physics::ConstraintHandle AddSlider(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis, bool collide_connected = false);

		// Add a fully locked relative transform between two prepared endpoint frames.
		physics::ConstraintHandle AddWeld(physics::BodyFrame frame_a, physics::BodyFrame frame_b, float break_force = std::numeric_limits<float>::infinity());

		// Configure one angular row without duplicating reduced-coordinate locked axes.
		physics::ConstraintHandle AddAngularControl(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis);

		// Configure one linear row without duplicating reduced-coordinate locked axes.
		physics::ConstraintHandle AddLinearControl(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis);

		// Set the world gravity sampled by every rigid body and articulation link.
		void Gravity(v4 gravity);

		// Set the number of GPU-resident substeps recorded into each single submitted frame.
		void Substeps(int substep_count);

		// Configure a visible water surface for deferred articulation-link buoyancy registration.
		void Water(float level, v2 size);

		// Defer registration of one shaped articulation link with the demonstration's buoyancy module.
		void MakeBuoyant(int articulation_index, physics::LinkHandle link);

		// Complete pointer caches, renderer objects, water resources, and sleep-island preparation.
		void Finalise();
	};

	// Build the rigid-constraint demonstrations implemented in demo_constraints.cpp.
	void BuildRigidJointGallery(DemoBuilder& demo);
	void BuildRigidChain(DemoBuilder& demo);
	void BuildFourBarLinkage(DemoBuilder& demo);

	// Build the application-scale demonstrations implemented in demo_showcase.cpp.
	void BuildPendulum(DemoBuilder& demo);
	void BuildRagdolls(DemoBuilder& demo);
	void BuildRobotGripper(DemoBuilder& demo);
	void BuildVehicleSuspension(DemoBuilder& demo);
	void BuildSuspensionBridge(DemoBuilder& demo);
	void BuildArticulationPush(DemoBuilder& demo);
	void BuildTwoRobotLoad(DemoBuilder& demo);

	// Build the reduced-coordinate and coupled demonstrations implemented in demo_articulations.cpp.
	void BuildFixedArticulations(DemoBuilder& demo);
	void BuildRobotMotors(DemoBuilder& demo);
	void BuildMixedCoupling(DemoBuilder& demo);
	void BuildMixedContacts(DemoBuilder& demo);
	void BuildBuoyantArticulation(DemoBuilder& demo);
	void BuildFloatingConservation(DemoBuilder& demo);
	void BuildDzhanibekov(DemoBuilder& demo);

	// Build the difficult and scaling demonstrations implemented in demo_pathologies.cpp.
	void BuildConstraintPathologies(DemoBuilder& demo);
	void BuildConstraintStress(DemoBuilder& demo);
}
