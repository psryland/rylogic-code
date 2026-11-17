#include "src/forward.h"
#include "src/scene/scene.h"

namespace physics_sandbox
{
	namespace
	{
		// Associates one JSON link name with the stable handle returned by its articulation builder.
		struct LinkRecord
		{
			std::string_view m_name;
			physics::LinkHandle m_handle;
		};

		// Associates one JSON articulation name with its scene index and link-name namespace.
		struct ArticulationRecord
		{
			std::string_view m_name;
			int m_articulation_index;
			std::vector<LinkRecord> m_links;
		};

		// Derive mass properties in the owning body frame from an immutable collision shape.
		physics::Inertia ShapeInertia(scene_loader::BodyDesc const& body, collision::Shape const& shape, m4x4 const& shape_to_body)
		{
			if (!body.density && !(body.mass > 0.0f))
				return physics::Inertia::Infinite();

			auto mass_properties = physics::CalcMassProperties(shape, body.density.value_or(1.0f), body.mass_distribution);
			if (!body.density)
				mass_properties.m_mass = body.mass;

			// Convert the origin-measured shape tensor to its centre of mass, then express that distribution in the body frame.
			auto inertia = physics::Inertia{mass_properties.m_os_unit_inertia, mass_properties.m_mass};
			if (LengthSq(mass_properties.m_centre_of_mass) != 0.0f)
				inertia = physics::Translate(inertia, mass_properties.m_centre_of_mass, physics::ETranslateInertia::TowardCoM);

			inertia = physics::Rotate(inertia, shape_to_body.rot);
			inertia.CoM((shape_to_body * mass_properties.m_centre_of_mass.w1()).w0());
			return inertia;
		}

		// Derive articulation-link inertia from the same collision-shape mass properties used by rigid bodies.
		physics::Inertia LinkInertia(scene_loader::ArticulationChildDesc const& link, collision::Shape const* shape)
		{
			if (link.m_inertia)
				return *link.m_inertia;
			if (shape == nullptr)
				throw std::runtime_error(pr::FmtS("Articulation link '%s' has neither collision shape nor inertia", link.m_body.name.c_str()));

			return ShapeInertia(link.m_body, *shape, link.m_shape_to_link);
		}

		// Convert one parsed link into the immutable descriptor consumed by ArticulationBuilder.
		physics::ArticulationLinkDesc LinkDescription(scene_loader::ArticulationChildDesc const& link, collision::Shape const* shape)
		{
			return physics::ArticulationLinkDesc{
				.m_inertia = LinkInertia(link, shape),
				.m_shape = shape,
				.m_shape_to_link = link.m_shape_to_link,
				.m_collide_parent = link.m_collide_parent,
				.m_collide_self = link.m_collide_self,
			};
		}

		// Return a stable deterministic fallback colour for one articulation link.
		Colour32 LinkColour(int articulation_index, int link_index)
		{
			auto const colour_seed = static_cast<uint32_t>(articulation_index * 37 + link_index * 53);
			return Colour32(
				96 + static_cast<int>((colour_seed * 3) % 144),
				96 + static_cast<int>((colour_seed * 5 + 47) % 144),
				96 + static_cast<int>((colour_seed * 7 + 89) % 144),
				255);
		}

		// Resolve a previously built link by its articulation-local JSON name.
		physics::LinkHandle FindLink(ArticulationRecord const& articulation, std::string_view link_name)
		{
			auto result = physics::LinkHandle{};
			auto match_count = 0;
			for (auto const& link : articulation.m_links)
			{
				if (link.m_name != link_name)
					continue;

				result = link.m_handle;
				++match_count;
			}
			if (match_count == 0)
				throw std::runtime_error(pr::FmtS("Unknown articulation link '%.*s'", static_cast<int>(link_name.size()), link_name.data()));
			if (match_count != 1)
				throw std::runtime_error(pr::FmtS("Articulation link name '%.*s' is ambiguous", static_cast<int>(link_name.size()), link_name.data()));

			return result;
		}

		// Resolve one scene articulation by its JSON name.
		ArticulationRecord const& FindArticulation(std::span<ArticulationRecord const> articulations, std::string_view articulation_name)
		{
			auto const* result = static_cast<ArticulationRecord const*>(nullptr);
			auto match_count = 0;
			for (auto const& articulation : articulations)
			{
				if (articulation.m_name != articulation_name)
					continue;

				result = &articulation;
				++match_count;
			}
			if (match_count == 0)
				throw std::runtime_error(pr::FmtS("Unknown articulation '%.*s'", static_cast<int>(articulation_name.size()), articulation_name.data()));
			if (match_count != 1)
				throw std::runtime_error(pr::FmtS("Articulation name '%.*s' is ambiguous", static_cast<int>(articulation_name.size()), articulation_name.data()));

			return *result;
		}

		// Resolve one rigid-body name without making names mandatory for legacy JSON scenes.
		int FindRigidBody(scene_loader::SceneDesc const& scene_desc, std::string_view body_name)
		{
			auto result = -1;
			auto match_count = 0;
			for (auto body_index = 0; body_index != isize(scene_desc.bodies); ++body_index)
			{
				if (scene_desc.bodies[body_index].name != body_name)
					continue;

				result = body_index;
				++match_count;
			}
			if (match_count == 0)
				throw std::runtime_error(pr::FmtS("Unknown rigid body '%.*s'", static_cast<int>(body_name.size()), body_name.data()));
			if (match_count != 1)
				throw std::runtime_error(pr::FmtS("Rigid body name '%.*s' is ambiguous", static_cast<int>(body_name.size()), body_name.data()));

			return result;
		}

		// Resolve one parsed endpoint and convert an initial world frame into endpoint-local coordinates when requested.
		physics::BodyFrame ResolveFrame(
			Scene& scene,
			scene_loader::SceneDesc const& scene_desc,
			std::span<ArticulationRecord const> articulations,
			scene_loader::ConstraintFrameDesc const& frame)
		{
			auto body_ref = physics::BodyRef::World();
			auto body_to_world = m4x4::Identity();
			switch (frame.m_body.m_type)
			{
				case scene_loader::BodyReferenceDesc::EType::World:
				{
					break;
				}
				case scene_loader::BodyReferenceDesc::EType::RigidBody:
				{
					auto const body_index = FindRigidBody(scene_desc, frame.m_body.m_body_name);
					auto const& body = scene.m_body[body_index];
					body_ref = physics::BodyRef::Rigid(body);
					body_to_world = body.O2W();
					break;
				}
				case scene_loader::BodyReferenceDesc::EType::ArticulationLink:
				{
					auto const& record = FindArticulation(articulations, frame.m_body.m_articulation_name);
					auto const link = FindLink(record, frame.m_body.m_link_name);
					auto const& articulation = scene.m_articulation[record.m_articulation_index];
					body_ref = physics::BodyRef::Link(articulation, link);
					body_to_world = articulation.LinkToWorld(link);
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown constraint endpoint type");
				}
			}

			auto constraint_to_body = frame.m_frame_to_space;
			switch (frame.m_space)
			{
				case scene_loader::ConstraintFrameDesc::ESpace::Local:
				{
					break;
				}
				case scene_loader::ConstraintFrameDesc::ESpace::World:
				{
					constraint_to_body = body_ref.IsWorld()
						? frame.m_frame_to_space
						: InvertOrthonormal(body_to_world) * frame.m_frame_to_space;
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown constraint frame space");
				}
			}

			return physics::BodyFrame::FromMatrix(body_ref, constraint_to_body);
		}

		// Append one parsed constraint through its strongly typed public descriptor.
		void AddConstraint(Scene& scene, scene_loader::SceneDesc const& scene_desc, std::span<ArticulationRecord const> articulations, scene_loader::ConstraintDesc const& constraint)
		{
			auto const frame_a = ResolveFrame(scene, scene_desc, articulations, constraint.m_frame_a);
			auto const frame_b = ResolveFrame(scene, scene_desc, articulations, constraint.m_frame_b);
			switch (constraint.m_type)
			{
				case scene_loader::ConstraintDesc::EType::BallSocket:
				{
					scene.m_constraints.Add(physics::BallSocketConstraintDesc{
						.m_frame_a = frame_a,
						.m_frame_b = frame_b,
						.m_break_force = constraint.m_break_force,
						.m_break_torque = constraint.m_break_torque,
						.m_collide_connected = constraint.m_collide_connected,
						.m_enabled = constraint.m_enabled,
					});
					break;
				}
				case scene_loader::ConstraintDesc::EType::Hinge:
				{
					scene.m_constraints.Add(physics::HingeConstraintDesc{
						.m_frame_a = frame_a,
						.m_frame_b = frame_b,
						.m_axis = constraint.m_axis,
						.m_break_force = constraint.m_break_force,
						.m_break_torque = constraint.m_break_torque,
						.m_collide_connected = constraint.m_collide_connected,
						.m_enabled = constraint.m_enabled,
					});
					break;
				}
				case scene_loader::ConstraintDesc::EType::Slider:
				{
					scene.m_constraints.Add(physics::SliderConstraintDesc{
						.m_frame_a = frame_a,
						.m_frame_b = frame_b,
						.m_axis = constraint.m_axis,
						.m_break_force = constraint.m_break_force,
						.m_break_torque = constraint.m_break_torque,
						.m_collide_connected = constraint.m_collide_connected,
						.m_enabled = constraint.m_enabled,
					});
					break;
				}
				case scene_loader::ConstraintDesc::EType::Weld:
				{
					scene.m_constraints.Add(physics::WeldConstraintDesc{
						.m_frame_a = frame_a,
						.m_frame_b = frame_b,
						.m_break_force = constraint.m_break_force,
						.m_break_torque = constraint.m_break_torque,
						.m_collide_connected = constraint.m_collide_connected,
						.m_enabled = constraint.m_enabled,
					});
					break;
				}
				case scene_loader::ConstraintDesc::EType::D6:
				{
					scene.m_constraints.Add(physics::D6ConstraintDesc{
						.m_frame_a = frame_a,
						.m_frame_b = frame_b,
						.m_linear = constraint.m_linear,
						.m_angular = constraint.m_angular,
						.m_break_force = constraint.m_break_force,
						.m_break_torque = constraint.m_break_torque,
						.m_collide_connected = constraint.m_collide_connected,
						.m_enabled = constraint.m_enabled,
					});
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown JSON constraint type");
				}
			}
		}
	}

	// Build the conventional rigid-joint model directly from authored frames and initial motion.
	void Scene::BuildRigidRagdolls(scene_loader::SceneDesc const& scene_desc, std::span<collision::Shape const* const> shapes)
	{
		if (scene_desc.rigid_ragdolls.empty())
			return;

		if (!scene_desc.bodies.empty() || !scene_desc.articulations.empty() || !scene_desc.constraints.empty() || scene_desc.water || scene_desc.terrain || scene_desc.atmosphere)
			throw std::runtime_error("Rigid ragdoll scenes require only ragdolls and optional ground");

		auto expected_shapes = 0;
		for (auto const& ragdoll : scene_desc.rigid_ragdolls)
			expected_shapes += 1 + isize(ragdoll.m_links);

		if (isize(shapes) != expected_shapes)
			throw std::runtime_error("Rigid ragdoll shapes do not match the authored body count");

		// No later allocation may move a body after its identity is captured by a persistent constraint.
		m_body.reserve(expected_shapes + (scene_desc.ground ? 1 : 0));
		m_rigid_ragdoll_joints.reserve(expected_shapes - scene_desc.rigid_ragdolls.size());
		auto shape_index = 0;
		for (auto tree_index = 0; tree_index != isize(scene_desc.rigid_ragdolls); ++tree_index)
		{
			auto const& source = scene_desc.rigid_ragdolls[tree_index];
			auto const first_body = isize(m_body);
			auto add_body = [&](scene_loader::BodyDesc const& desc, m4x4 const& pose, v8motion velocity, int link_index)
			{
				// Preserve the original centred shape, shape-derived inertia, collision filter and sleep policy.
				auto const* shape = shapes[shape_index++];
				auto const inertia = ShapeInertia(desc, *shape, m4x4::Identity());
				if (LengthSq(inertia.CoM()) != 0.0f)
					throw std::runtime_error("Rigid ragdoll shapes require their centre of mass at the body origin");

				auto body = Body(nullptr, shape, pose, inertia);
				body.VelocityWS(pose.rot * velocity.ang, pose.rot * velocity.lin);
				body.CollisionFilter(desc.collision_layer, desc.collision_mask);
				body.NeverSleep(source.m_never_sleep);
				if (source.m_sleeping)
					body.Sleep();

				body.m_colour = desc.colour.value_or(LinkColour(tree_index, link_index));
				m_body.push_back(std::move(body));
				return inertia.Mass();
			};
			add_body(source.m_root, source.m_root_to_world, source.m_root_velocity, 0);

			auto dof_offset = 0;
			for (auto link_index = 0; link_index != isize(source.m_links); ++link_index)
			{
				// Resolve the local parent namespace before building the child's exact initial pose and body-frame velocity.
				auto const& link = source.m_links[link_index];
				auto parent_index = link.m_parent_name == source.m_root.name ? 0 : -1;
				if (link.m_body.name == source.m_root.name)
					throw std::runtime_error("Rigid ragdoll link names must be unique");

				for (auto i = 0; i != link_index; ++i)
				{
					if (source.m_links[i].m_body.name == link.m_body.name)
						throw std::runtime_error("Rigid ragdoll link names must be unique");
					if (source.m_links[i].m_body.name == link.m_parent_name)
						parent_index = i + 1;
				}
				if (parent_index == -1)
					throw std::runtime_error("Rigid ragdoll parent must precede its child");

				auto const parent = first_body + parent_index;
				auto const child = isize(m_body);
				auto const child_to_parent = link.m_joint_to_parent * m4x4::Transform(link.m_initial_rotation, v4::Origin()) * InvertOrthonormal(link.m_joint_to_child);
				auto const pose = m_body[parent].O2W() * child_to_parent;
				auto const velocity = InvertOrthonormal(child_to_parent) * m_body[parent].VelocityOS() + link.m_joint_to_child * v8motion{link.m_initial_angular_velocity, v4::Zero()};
				auto const child_mass = add_body(link.m_body, pose, velocity, link_index + 1);

				// Keep hard anchor/limit rows separate from child-first angular-only actuator rows.
				auto anchor = physics::D6ConstraintDesc{
					.m_frame_a = physics::BodyFrame::FromMatrix(physics::BodyRef::Rigid(m_body[parent]), link.m_joint_to_parent),
					.m_frame_b = physics::BodyFrame::FromMatrix(physics::BodyRef::Rigid(m_body[child]), link.m_joint_to_child),
					.m_collide_connected = link.m_collide_parent,
				};
				for (auto& axis : anchor.m_linear)
					axis.m_mode = physics::EConstraintAxisMode::Locked;

				auto dof_count = 0;
				switch (link.m_joint_type)
				{
					case scene_loader::RigidRagdollLinkDesc::EJointType::Spherical:
					{
						for (auto axis = 0; axis != 3; ++axis)
							anchor.m_angular[axis] = {.m_mode = physics::EConstraintAxisMode::Limited, .m_limits = link.m_limits[axis]};

						dof_count = 3;
						break;
					}
					case scene_loader::RigidRagdollLinkDesc::EJointType::Hinge:
					{
						anchor.m_angular[0] = {.m_mode = physics::EConstraintAxisMode::Limited, .m_limits = link.m_limits[0]};
						anchor.m_angular[1].m_mode = physics::EConstraintAxisMode::Locked;
						anchor.m_angular[2].m_mode = physics::EConstraintAxisMode::Locked;
						dof_count = 1;
						break;
					}
					default: { throw std::runtime_error("Unknown rigid ragdoll joint type"); }
				}
				auto actuator = physics::D6ConstraintDesc{
					.m_frame_a = anchor.m_frame_b,
					.m_frame_b = anchor.m_frame_a,
					.m_collide_connected = true,
				};
				for (auto axis = 0; axis != dof_count; ++axis)
					actuator.m_angular[axis] = {.m_mode = physics::EConstraintAxisMode::Driven, .m_damping = link.m_damping};

				m_rigid_ragdoll_joints.push_back(RigidRagdollJoint{
					.m_parent = parent,
					.m_child = child,
					.m_dof_count = dof_count,
					.m_dof_offset = dof_offset,
					.m_amplitude = source.m_actuation.m_torque_per_kg * child_mass,
					.m_actuation = source.m_actuation,
					.m_anchor = m_constraints.Add(anchor),
					.m_actuator = m_constraints.Add(actuator),
				});
				dof_offset += dof_count;
			}
		}
	}

	// Update only actuator target velocities; anchors and validated endpoint frames retain their stored representation.
	void Scene::ApplyRagdollActuation(double time_s)
	{
		constexpr auto golden_angle = 2.39996323f;
		for (auto const& joint : m_rigid_ragdoll_joints)
		{
			// Child-first frame A receives the negative row impulse: -tau/d gives child torque tau-d*relative_speed.
			auto const& actuator = m_constraints.Get(joint.m_actuator);
			auto target_velocities = std::array{
				actuator.m_linear[0].m_target_velocity, actuator.m_linear[1].m_target_velocity, actuator.m_linear[2].m_target_velocity,
				actuator.m_angular[0].m_target_velocity, actuator.m_angular[1].m_target_velocity, actuator.m_angular[2].m_target_velocity,
			};
			auto const cycle = static_cast<float>(std::fmod(time_s / joint.m_actuation.m_period_s, 1.0)) * constants<float>::tau;
			for (auto axis = 0; axis != joint.m_dof_count; ++axis)
			{
				auto const torque = joint.m_amplitude * std::sin(cycle + joint.m_actuation.m_phase + golden_angle * static_cast<float>(joint.m_dof_offset + axis));
				target_velocities[3 + axis] = -torque / actuator.m_angular[axis].m_damping;
			}
			m_constraints.SetTargetVelocities(joint.m_actuator, target_velocities);
		}
	}

	// Build JSON-described articulation trees and persistent constraints after all collision-shape addresses are stable.
	void Scene::BuildMultibodyObjects(scene_loader::SceneDesc const& scene_desc, std::span<collision::Shape const* const> articulation_shapes)
	{
		auto expected_shape_count = 0;
		for (auto const& articulation : scene_desc.articulations)
		{
			expected_shape_count += articulation.m_root.m_has_shape ? 1 : 0;
			for (auto const& link : articulation.m_links)
				expected_shape_count += link.m_has_shape ? 1 : 0;
		}
		if (isize(articulation_shapes) != expected_shape_count)
			throw std::runtime_error("Articulation shape mapping does not match the parsed link count");

		// Reserve all owning and lookup containers before BodyRef identities are captured by persistent constraints.
		m_articulation.reserve(scene_desc.articulations.size());
		auto articulation_records = std::vector<ArticulationRecord>{};
		articulation_records.reserve(scene_desc.articulations.size());
		auto shape_index = 0;

		// Build each tree in root-first JSON order so parent references fail locally and predictably.
		for (auto articulation_index = 0; articulation_index != isize(scene_desc.articulations); ++articulation_index)
		{
			auto const& source = scene_desc.articulations[articulation_index];
			auto builder = physics::ArticulationBuilder{};
			auto const* root_shape = source.m_root.m_has_shape ? articulation_shapes[shape_index++] : nullptr;
			auto root_desc = LinkDescription(source.m_root, root_shape);
			auto root = physics::LinkHandle{};
			switch (source.m_root_type)
			{
				case physics::EArticulationRootType::Fixed:
					{
						root = builder.AddFixedRoot(root_desc, source.m_root_to_world);
						break;
					}
				case physics::EArticulationRootType::Floating:
					{
						root = builder.AddFloatingRoot(root_desc, source.m_root_to_world, source.m_root_velocity);
						break;
					}
				default:
					{
						throw std::runtime_error("Unknown articulation root type");
					}
			}

			auto record = ArticulationRecord{
				.m_name = source.m_name,
				.m_articulation_index = articulation_index,
			};
			record.m_links.reserve(1 + source.m_links.size());
			record.m_links.push_back(LinkRecord{
				.m_name = source.m_root.m_body.name,
				.m_handle = root,
			});
			for (auto const& link : source.m_links)
			{
				auto const parent = FindLink(record, link.m_parent_name);
				auto const* shape = link.m_has_shape ? articulation_shapes[shape_index++] : nullptr;
				auto link_desc = LinkDescription(link, shape);
				auto const handle = builder.AddLink(parent, link.m_joint, link_desc);
				record.m_links.push_back(LinkRecord{
					.m_name = link.m_body.name,
					.m_handle = handle,
				});
			}

			// Scene gravity is initial state, so configure it before applying an explicit request to start asleep.
			auto articulation = builder.Build();
			for (auto link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				articulation.GravityWS(articulation.LinkAt(link_index), m_gravity);

			// Sleep policy is applied last so initial load configuration cannot undo it.
			articulation.NeverSleep(source.m_never_sleep);
			if (source.m_sleeping)
				articulation.Sleep();

			// Store coherent initial link state without changing the selected sleep policy.
			articulation.UpdateKinematics();
			m_articulation.push_back(std::move(articulation));
			articulation_records.push_back(std::move(record));
		}

		// Resolve names only after every articulation has reached its final stable scene address.
		for (auto const& constraint : scene_desc.constraints)
			AddConstraint(*this, scene_desc, articulation_records, constraint);

		// Create one renderer record per shaped link using explicit JSON colours or deterministic fallbacks.
		for (auto articulation_index = 0; articulation_index != isize(scene_desc.articulations); ++articulation_index)
		{
			auto const& source = scene_desc.articulations[articulation_index];
			auto& articulation = m_articulation[articulation_index];
			auto append_visual = [&](scene_loader::ArticulationChildDesc const& link, int link_index)
			{
				auto const handle = articulation.LinkAt(link_index);
				auto const& link_desc = articulation.LinkDescription(handle);
				if (link_desc.m_shape == nullptr)
					return;

				m_articulation_visuals.emplace_back(
					articulation_index,
					handle,
					*link_desc.m_shape,
					link_desc.m_shape_to_link,
					link.m_body.colour.value_or(LinkColour(articulation_index, link_index)));
			};

			append_visual(source.m_root, 0);
			for (auto link_index = 0; link_index != isize(source.m_links); ++link_index)
				append_visual(source.m_links[link_index], link_index + 1);
		}
	}
}

#if PR_UNITTESTS
namespace physics_sandbox::tests
{
	namespace
	{
		// Construct one parsed endpoint reference for focused runtime-resolution tests.
		scene_loader::ConstraintFrameDesc RigidFrame(std::string body_name, scene_loader::ConstraintFrameDesc::ESpace space = scene_loader::ConstraintFrameDesc::ESpace::Local, m4x4 const& frame = m4x4::Identity())
		{
			return scene_loader::ConstraintFrameDesc{
				.m_body = scene_loader::BodyReferenceDesc{
					.m_type = scene_loader::BodyReferenceDesc::EType::RigidBody,
					.m_body_name = std::move(body_name),
				},
				.m_space = space,
				.m_frame_to_space = frame,
			};
		}

		// Construct the world endpoint shared by constraint-construction tests.
		scene_loader::ConstraintFrameDesc WorldFrame()
		{
			return scene_loader::ConstraintFrameDesc{
				.m_body = scene_loader::BodyReferenceDesc{
					.m_type = scene_loader::BodyReferenceDesc::EType::World,
					.m_body_name = "world",
				},
				.m_space = scene_loader::ConstraintFrameDesc::ESpace::World,
				.m_frame_to_space = m4x4::Identity(),
			};
		}
	}

	// Protect the conventional model's child-frame drive sign and narrow target-velocity updates.
	PRUnitTestClass(RigidRagdollDriveTests)
	{
		PRUnitTestMethod(PreservesPoseAndInternalHingeDrive, Extended)
		{
			auto source = scene_loader::RigidRagdollDesc{};
			source.m_name = "drive";
			source.m_root_to_world = m4x4::TransformDeg(0.0f, 0.0f, 45.0f, v4::Origin());
			source.m_never_sleep = true;
			source.m_root.name = "parent";
			source.m_root.mass = 3.0f;
			source.m_root.box_dimensions = v4{1, 1, 1, 0};
			source.m_actuation = {.m_torque_per_kg = 2.0f, .m_period_s = 1.0f, .m_phase = constants<float>::tau_by_4};
			auto child = scene_loader::RigidRagdollLinkDesc{};
			child.m_body.name = "child";
			child.m_body.mass = 2.0f;
			child.m_body.box_dimensions = v4{1, 1, 1, 0};
			child.m_parent_name = "parent";
			child.m_limits[0] = {-1.0f, +1.0f};
			child.m_damping = 0.5f;
			source.m_links.push_back(child);
			auto scene_desc = scene_loader::SceneDesc{};
			scene_desc.gravity = v4::Zero();
			scene_desc.physics_substeps = 3;
			scene_desc.physics_solver_iterations = 10;
			scene_desc.physics_position_iterations = 1;
			scene_desc.rigid_ragdolls.push_back(source);
			auto scene = Scene(nullptr);
			scene.LoadScene(scene_desc);
			PR_EXPECT(scene.m_body.size() == 2 && scene.m_articulation.empty() && scene.m_constraints.Count() == 2);
			PR_EXPECT(FEql(scene.m_body[1].O2W(), source.m_root_to_world));

			// A rotated common origin isolates internal torque from contact, orbital momentum and world-axis mistakes.
			auto const& actuator = scene.m_constraints.Get(scene.m_rigid_ragdoll_joints[0].m_actuator);
			for (auto const& axis : actuator.m_linear)
				PR_EXPECT(axis.m_mode == physics::EConstraintAxisMode::Free);

			PR_EXPECT(actuator.m_angular[1].m_mode == physics::EConstraintAxisMode::Free && actuator.m_angular[2].m_mode == physics::EConstraintAxisMode::Free);
			scene.Step(1.0 / 60.0);
			auto const axis_ws = source.m_root_to_world.x;
			PR_EXPECT(Dot3(scene.m_body[1].VelocityWS().ang, axis_ws) > 0.0f);
			PR_EXPECT(Dot3(scene.m_body[0].VelocityWS().ang, axis_ws) < 0.0f);
			PR_EXPECT(Length(scene.m_body[0].MomentumWS().ang + scene.m_body[1].MomentumWS().ang) < 1.0e-4f);
		}

		PRUnitTestMethod(PreservesExistingActuationTargets, Extended)
		{
			auto make_link = [](std::string name, std::string parent, float mass, scene_loader::RigidRagdollLinkDesc::EJointType joint_type)
			{
				auto link = scene_loader::RigidRagdollLinkDesc{};
				link.m_body.name = std::move(name);
				link.m_body.mass = mass;
				link.m_body.box_dimensions = v4{1, 1, 1, 0};
				link.m_parent_name = std::move(parent);
				link.m_joint_type = joint_type;
				link.m_damping = 0.5f;
				link.m_limits.fill({-0.75f, +0.75f});
				return link;
			};
			auto source = scene_loader::RigidRagdollDesc{};
			source.m_name = "targets";
			source.m_root_to_world = m4x4::TransformDeg(10, 20, 30, v4::Origin());
			source.m_never_sleep = true;
			source.m_root.name = "root";
			source.m_root.mass = 4.0f;
			source.m_root.box_dimensions = v4{1, 1, 1, 0};
			source.m_links = {
				make_link("hinge", "root", 2.0f, scene_loader::RigidRagdollLinkDesc::EJointType::Hinge),
				make_link("sphere", "hinge", 3.0f, scene_loader::RigidRagdollLinkDesc::EJointType::Spherical),
			};
			source.m_actuation = {.m_torque_per_kg = 1.5f, .m_period_s = 1.3f, .m_phase = 0.3f};
			auto scene_desc = scene_loader::SceneDesc{};
			scene_desc.rigid_ragdolls.push_back(source);
			auto scene = Scene(nullptr);
			scene.LoadScene(scene_desc);
			PR_EXPECT(scene.m_rigid_ragdoll_joints.size() == 2 && scene.m_articulation.empty());

			// Inactive nonzero targets and distinct axis damping expose unintended clearing or wrong-axis reads.
			for (auto const& joint : scene.m_rigid_ragdoll_joints)
			{
				auto desc = scene.m_constraints.Get(joint.m_actuator);
				for (auto axis = 0; axis != 3; ++axis)
				{
					desc.m_linear[axis].m_target_velocity = 11.0f + axis;
					desc.m_angular[axis].m_target_velocity = -21.0f - axis;
					if (axis < joint.m_dof_count)
						desc.m_angular[axis].m_damping *= 1.0f + 0.25f * axis;
				}
				scene.m_constraints.Update(joint.m_actuator, desc);
			}
			auto reference = scene.m_constraints;
			for (auto time_s : {0.0, 0.137, 2.351, 2.351})
			{
				// Compare against the retained full-descriptor law across a cycle boundary and a repeated timestamp.
				scene.m_constraints.ClearDirty();
				reference.ClearDirty();
				for (auto const& joint : scene.m_rigid_ragdoll_joints)
				{
					auto desc = reference.Get(joint.m_actuator);
					auto const cycle = static_cast<float>(std::fmod(time_s / joint.m_actuation.m_period_s, 1.0)) * constants<float>::tau;
					for (auto axis = 0; axis != joint.m_dof_count; ++axis)
					{
						auto const torque = joint.m_amplitude * std::sin(cycle + joint.m_actuation.m_phase + 2.39996323f * static_cast<float>(joint.m_dof_offset + axis));
						desc.m_angular[axis].m_target_velocity = -torque / desc.m_angular[axis].m_damping;
					}
					reference.Update(joint.m_actuator, desc);
				}
				scene.ApplyRagdollActuation(time_s);
				PR_EXPECT(scene.m_constraints.TopologyRevision() == reference.TopologyRevision());
				PR_EXPECT(scene.m_constraints.ParameterRevision() == reference.ParameterRevision());
				auto const ranges = scene.m_constraints.DirtyRanges();
				PR_EXPECT(ranges.size() == scene.m_rigid_ragdoll_joints.size());
				for (size_t i = 0; i != scene.m_rigid_ragdoll_joints.size(); ++i)
				{
					auto const& joint = scene.m_rigid_ragdoll_joints[i];
					auto const& actual = scene.m_constraints.Get(joint.m_actuator);
					auto const& expected = reference.Get(joint.m_actuator);
					PR_EXPECT(i < ranges.size() && ranges[i].m_begin == joint.m_actuator.m_index && ranges[i].m_end == joint.m_actuator.m_index + 1);
					for (auto axis = 0; axis != 3; ++axis)
					{
						PR_EXPECT(std::memcmp(&actual.m_linear[axis].m_target_velocity, &expected.m_linear[axis].m_target_velocity, sizeof(float)) == 0);
						PR_EXPECT(std::memcmp(&actual.m_angular[axis].m_target_velocity, &expected.m_angular[axis].m_target_velocity, sizeof(float)) == 0);
						PR_EXPECT(actual.m_angular[axis].m_damping == expected.m_angular[axis].m_damping);
					}
				}
			}
		}
	};

	PRUnitTestClass(SceneMultibodyConstructionTests)
	{
		PRUnitTestMethod(ConvertsWorldFramesAndRejectsInvalidReferences, Quick)
		{
			// Resolve an initial world-space anchor through the body's actual transform rather than treating it as local data.
			auto scene = Scene(nullptr);
			scene.m_body.emplace_back(nullptr, nullptr, m4x4::Translation(2.0f, 3.0f, 4.0f), physics::Inertia::Box(v4{1, 1, 1, 0}, 1.0f));
			auto scene_desc = scene_loader::SceneDesc{};
			scene_desc.bodies.push_back(scene_loader::BodyDesc{.name = "body"});
			auto const frame_to_world = m4x4::Translation(5.0f, 7.0f, 11.0f);
			auto const resolved = ResolveFrame(scene, scene_desc, {}, RigidFrame("body", scene_loader::ConstraintFrameDesc::ESpace::World, frame_to_world));
			auto const expected = InvertOrthonormal(scene.m_body[0].O2W()) * frame_to_world;
			PR_EXPECT(Length(resolved.m_position - expected.pos) < 1.0e-6f);

			// Unknown and ambiguous names must fail before a persistent BodyRef can capture the wrong object.
			PR_THROWS(ResolveFrame(scene, scene_desc, {}, RigidFrame("missing")), std::exception);
			scene_desc.bodies.push_back(scene_loader::BodyDesc{.name = "body"});
			PR_THROWS(ResolveFrame(scene, scene_desc, {}, RigidFrame("body")), std::exception);
		}

		PRUnitTestMethod(BuildsShapeLessTreesAndEveryConstraintType, Quick)
		{
			// Build a shape-less fixed tree to prove explicit inertia is sufficient for reduced-coordinate runtime construction.
			auto scene = Scene(nullptr);
			auto scene_desc = scene_loader::SceneDesc{};
			auto articulation = scene_loader::ArticulationDesc{};
			articulation.m_name = "tree";
			articulation.m_root.m_body.name = "root";
			articulation.m_root.m_has_shape = false;
			articulation.m_root.m_inertia = physics::Inertia::Sphere(0.5f, 2.0f);
			auto child = scene_loader::ArticulationChildDesc{};
			child.m_body.name = "child";
			child.m_has_shape = false;
			child.m_parent_name = "root";
			child.m_inertia = physics::Inertia::Box(v4{0.5f, 0.25f, 0.25f, 0.0f}, 1.0f);
			articulation.m_links.push_back(std::move(child));
			scene_desc.articulations.push_back(std::move(articulation));
			scene.BuildMultibodyObjects(scene_desc, {});
			PR_EXPECT(scene.m_articulation.size() == 1);
			PR_EXPECT(scene.m_articulation[0].LinkCount() == 2);

			// Resolve all strongly typed constraint descriptors through one ordinary rigid endpoint.
			auto constraint_scene = Scene(nullptr);
			constraint_scene.m_body.emplace_back(nullptr, nullptr, m4x4::Identity(), physics::Inertia::Box(v4{1, 1, 1, 0}, 1.0f));
			auto constraint_desc = scene_loader::SceneDesc{};
			constraint_desc.bodies.push_back(scene_loader::BodyDesc{.name = "body"});
			auto const world = WorldFrame();
			auto const body = RigidFrame("body");
			constraint_desc.constraints = {
				scene_loader::ConstraintDesc{.m_type = scene_loader::ConstraintDesc::EType::BallSocket, .m_frame_a = world, .m_frame_b = body},
				scene_loader::ConstraintDesc{.m_type = scene_loader::ConstraintDesc::EType::Hinge, .m_frame_a = world, .m_frame_b = body},
				scene_loader::ConstraintDesc{.m_type = scene_loader::ConstraintDesc::EType::Slider, .m_frame_a = world, .m_frame_b = body},
				scene_loader::ConstraintDesc{.m_type = scene_loader::ConstraintDesc::EType::Weld, .m_frame_a = world, .m_frame_b = body},
				scene_loader::ConstraintDesc{.m_type = scene_loader::ConstraintDesc::EType::D6, .m_frame_a = world, .m_frame_b = body},
			};
			constraint_scene.BuildMultibodyObjects(constraint_desc, {});
			PR_EXPECT(constraint_scene.m_constraints.Count() == 5);
		}

		PRUnitTestMethod(TransformsShapeDerivedInertiaIntoLinkSpace, Quick)
		{
			// A shape offset from the link origin must carry both its rotated central tensor and translated centre of mass into reduced dynamics.
			auto link = scene_loader::ArticulationChildDesc{};
			link.m_body.mass = 2.0f;
			link.m_shape_to_link = m4x4::Transform(RotationRad<m3x3>(0.0f, 0.0f, constants<float>::tau_by_4), v4{4.0f, 5.0f, 6.0f, 1.0f});
			auto const shape = collision::ShapeBox(v4{2.0f, 4.0f, 6.0f, 0.0f});
			auto const inertia = LinkInertia(link, shape);
			auto const untransformed = physics::Inertia::Box(v4{1.0f, 2.0f, 3.0f, 0.0f}, link.m_body.mass);

			PR_EXPECT(FEql(inertia.CoM(), link.m_shape_to_link.pos.w0()));
			PR_EXPECT(FEql(inertia.Ic3x3(1.0f), physics::Rotate(untransformed, link.m_shape_to_link.rot).Ic3x3(1.0f)));
		}
	};
}
#endif
