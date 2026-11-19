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

		// Derive articulation-link inertia from the same collision-shape mass properties used by rigid bodies.
		physics::Inertia LinkInertia(scene_loader::ArticulationChildDesc const& link, collision::Shape const* shape)
		{
			if (link.m_inertia)
				return *link.m_inertia;
			if (shape == nullptr)
				throw std::runtime_error(pr::FmtS("Articulation link '%s' has neither collision shape nor inertia", link.m_body.name.c_str()));

			auto const& body = link.m_body;
			if (!body.density && !(body.mass > 0.0f))
				return physics::Inertia::Infinite();

			auto mass_properties = physics::CalcMassProperties(*shape, body.density.value_or(1.0f), body.mass_distribution);
			if (!body.density)
				mass_properties.m_mass = body.mass;

			// Convert the origin-measured shape tensor to its centre of mass, then express that physical distribution in the link frame.
			auto inertia = physics::Inertia{mass_properties.m_os_unit_inertia, mass_properties.m_mass};
			if (LengthSq(mass_properties.m_centre_of_mass) != 0.0f)
				inertia = physics::Translate(inertia, mass_properties.m_centre_of_mass, physics::ETranslateInertia::TowardCoM);

			inertia = physics::Rotate(inertia, link.m_shape_to_link.rot);
			inertia.CoM((link.m_shape_to_link * mass_properties.m_centre_of_mass.w1()).w0());
			return inertia;
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

			return physics::BodyFrame{
				.m_body = body_ref,
				.m_constraint_to_body = constraint_to_body,
			};
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
					scene.m_constraints.Add(physics::ConstraintDesc{
						.m_frame_a = frame_a,
						.m_frame_b = frame_b,
						.m_parameters = physics::D6ConstraintParameters{
							.m_linear = constraint.m_linear,
							.m_angular = constraint.m_angular,
						},
						.m_break_force = constraint.m_break_force,
						.m_break_torque = constraint.m_break_torque,
						.m_collide_connected = constraint.m_collide_connected,
						.m_enabled = constraint.m_enabled,
					});
					break;
				}
				case scene_loader::ConstraintDesc::EType::SphericalAngular:
				{
					// Scene construction explicitly owns the typed constraint; creating a quaternion joint alone adds no hidden limit or damper.
					scene.m_constraints.Add(physics::ConstraintDesc{
						.m_frame_a = frame_a,
						.m_frame_b = frame_b,
						.m_parameters = constraint.m_spherical,
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
		m_articulation_actuation.reserve(scene_desc.articulations.size());
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
			m_articulation_actuation.push_back(source.m_actuation);
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
	// Apply mass-scaled generalized forces in velocity coordinates, not quaternion component coordinates.
	void Scene::ApplyArticulationActuation(double time_s)
	{
		assert(m_articulation_actuation.size() == m_articulation.size());
		constexpr auto golden_angle = 2.39996323f;
		for (int articulation_index = 0; articulation_index != isize(m_articulation); ++articulation_index)
		{
			auto const& actuation = m_articulation_actuation[articulation_index];
			if (actuation.m_torque_per_kg == 0.0f)
				continue;

			auto& articulation = m_articulation[articulation_index];
			auto const cycle = static_cast<float>(std::fmod(time_s / actuation.m_period_s, 1.0)) * constants<float>::tau;
			auto dof_index = 0;
			for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				auto const dof_count = isize(articulation.JointForce(link));
				auto const amplitude = actuation.m_torque_per_kg * articulation.LinkDescription(link).m_inertia.Mass();
				auto force = std::array<float, 6>{};
				assert(dof_count <= isize(force));
				for (int index = 0; index != dof_count; ++index, ++dof_index)
					force[index] = amplitude * std::sin(cycle + actuation.m_phase + golden_angle * static_cast<float>(dof_index));

				articulation.JointForce(link, std::span<float const>{force.data(), static_cast<size_t>(dof_count)});
			}
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

	// Authored excitation acts on three spherical velocities and preserves independent forces when disabled.
	PRUnitTestClass(SceneArticulationActuationTests)
	{
		PRUnitTestMethod(UsesJointVelocityDimensionsAndFrameStartPhase, Quick)
		{
			auto scene = Scene(nullptr);
			auto desc = scene_loader::SceneDesc{};
			auto tree = scene_loader::ArticulationDesc{};
			tree.m_name = "actuated";
			tree.m_root.m_body.name = "root";
			tree.m_root.m_has_shape = false;
			tree.m_root.m_inertia = physics::Inertia::Sphere(0.5f, 1.0f);
			tree.m_actuation = {.m_torque_per_kg = 3.0f, .m_period_s = 2.0f, .m_phase = 0.25f};
			auto child = scene_loader::ArticulationChildDesc{};
			child.m_body.name = "child";
			child.m_parent_name = "root";
			child.m_has_shape = false;
			child.m_inertia = physics::Inertia::Sphere(0.5f, 2.0f);
			child.m_joint = physics::ArticulationJointDesc::Spherical();
			tree.m_links.push_back(child);
			desc.articulations.push_back(tree);
			scene.BuildMultibodyObjects(desc, {});
			auto& articulation = scene.m_articulation[0];
			auto const link = articulation.LinkAt(1);
			PR_EXPECT(articulation.JointPosition(link).size() == 4);
			for (auto time_s : {0.0, 0.5, 2.0})
			{
				scene.ApplyArticulationActuation(time_s);
				auto const force = articulation.JointForce(link);
				PR_EXPECT(force.size() == 3);
				for (int index = 0; index != 3; ++index)
				{
					auto const phase = static_cast<float>(time_s / 2.0) * constants<float>::tau + 0.25f + 2.39996323f * static_cast<float>(index);
					PR_EXPECT(FEqlAbsolute(force[index], 6.0f * std::sin(phase), 1.0e-5f));
				}
			}
			auto const preserved = std::array{1.0f, 2.0f, 3.0f};
			articulation.JointForce(link, preserved);
			scene.m_articulation_actuation[0].m_torque_per_kg = 0;
			scene.ApplyArticulationActuation(0.5);
			PR_EXPECT(std::ranges::equal(articulation.JointForce(link), preserved));
			scene.ClearSimulationObjects();
			PR_EXPECT(scene.m_articulation_actuation.empty());
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
			PR_EXPECT(Length(resolved.m_constraint_to_body.pos - expected.pos) < 1.0e-6f);

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
			auto parent_frame = m4x4::TransformDeg(0, 0, 30, v4::Origin());
			auto child_frame = m4x4::TransformDeg(0, 10, 0, v4::Origin());
			child.m_joint = physics::ArticulationJointDesc::Spherical(parent_frame, child_frame);
			child.m_joint.m_initial_position = {0.6f, 0, 0, 0.8f};
			child.m_joint.m_initial_velocity = {0.2f, 0.3f, 0.4f};
			articulation.m_links.push_back(std::move(child));
			auto slider = scene_loader::ArticulationChildDesc{};
			slider.m_body.name = "slider";
			slider.m_has_shape = false;
			slider.m_parent_name = "child";
			slider.m_inertia = physics::Inertia::Sphere(0.25f, 1.0f);
			slider.m_joint.m_dof_count = 1;
			slider.m_joint.m_axes[0] = physics::ArticulationAxisDesc{.m_type = physics::EArticulationAxisType::Prismatic, .m_axis = v4::YAxis()};
			slider.m_joint.m_initial_position[0] = 0.7f;
			articulation.m_links.push_back(std::move(slider));
			scene_desc.articulations.push_back(std::move(articulation));
			scene.BuildMultibodyObjects(scene_desc, {});
			PR_EXPECT(scene.m_articulation.size() == 1);
			auto const& tree = scene.m_articulation[0];
			PR_EXPECT(tree.LinkCount() == 3);
			PR_EXPECT(tree.JointPosition(tree.LinkAt(1)).size() == 4);
			PR_EXPECT(tree.JointVelocity(tree.LinkAt(1)).size() == 3);
			PR_EXPECT(tree.JointPosition(tree.LinkAt(1))[3] == 0.8f);
			PR_EXPECT(tree.JointVelocity(tree.LinkAt(1))[2] == 0.4f);
			PR_EXPECT(tree.JointPosition(tree.LinkAt(2))[0] == 0.7f);
			auto expected_rotation = parent_frame.rot * m4x4::Transform(quat{0.6f, 0, 0, 0.8f}, v4::Origin()).rot * Transpose(child_frame.rot);
			PR_EXPECT(FEqlAbsolute(tree.LinkToWorld(tree.LinkAt(1)).rot, expected_rotation, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(tree.LinkVelocity(tree.LinkAt(1)).ang, child_frame.rot * v4{0.2f, 0.3f, 0.4f, 0}, 1.0e-6f));
			PR_EXPECT(scene.m_constraints.Count() == 0);
			auto const actual_speed = tree.LinkVelocity(tree.LinkAt(1)).ang;
			log_stream() << pr::FmtS("child-joint angular input=(.2,.3,.4), child-link angular output=(%.9g,%.9g,%.9g); rotation error=%.9g; implicit constraints=%zu\n",
				actual_speed.x, actual_speed.y, actual_speed.z, Length(tree.LinkToWorld(tree.LinkAt(1)).rot.x - expected_rotation.x), scene.m_constraints.Count());

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
				scene_loader::ConstraintDesc{.m_type = scene_loader::ConstraintDesc::EType::SphericalAngular, .m_frame_a = world, .m_frame_b = body, .m_spherical = physics::SphericalAngularConstraintParameters{.m_damping = 6}},
			};
			constraint_scene.BuildMultibodyObjects(constraint_desc, {});
			PR_EXPECT(constraint_scene.m_constraints.Count() == 6);

			// Invalid physical damping must reach native typed validation rather than being discarded as an unused D6 field.
			constraint_desc.constraints.erase(constraint_desc.constraints.begin(), constraint_desc.constraints.end() - 1);
			constraint_desc.constraints[0].m_spherical.m_damping = -1;
			PR_THROWS(constraint_scene.BuildMultibodyObjects(constraint_desc, {}), std::exception);
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
