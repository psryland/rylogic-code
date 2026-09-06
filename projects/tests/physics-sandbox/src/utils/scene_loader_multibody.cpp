#include "src/forward.h"
#include "src/utils/scene_loader_internal.h"

namespace physics_sandbox::scene_loader
{
	namespace
	{
		// Return a required string property with an error that identifies its owning schema object.
		std::string ReadRequiredString(pr::json::Object const& object, char const* property_name, char const* owner_name)
		{
			auto const* value = object.find(property_name);
			if (value == nullptr)
				throw std::runtime_error(pr::FmtS("%s requires a '%s' field", owner_name, property_name));

			auto result = value->to<std::string>();
			if (result.empty())
				throw std::runtime_error(pr::FmtS("%s field '%s' cannot be empty", owner_name, property_name));

			return result;
		}

		// Read a finite direction and normalise it for a joint or frame axis.
		v4 ReadUnitAxis(pr::json::Value const& value, char const* field_name)
		{
			auto axis = ReadVec3(value, 0.0f);
			auto const length_sq = LengthSq(axis);
			if (!IsFinite(axis) || !(length_sq > Sqr(math::tiny<float>)))
				throw std::runtime_error(pr::FmtS("Field '%s' requires a finite non-zero axis", field_name));

			return axis / Sqrt(length_sq);
		}

		// Read a transform expressed by a position and one optional orientation representation.
		m4x4 ReadTransform(pr::json::Object const& object, bool allow_x_axis)
		{
			auto position = v4::Origin();
			if (auto const* value = object.find("position"))
				position = ReadVec3(*value, 1.0f);

			auto const* rotation = object.find("rotation");
			auto const* rotation_axis = object.find("rotation_axis");
			auto const* rotation_angle = object.find("rotation_angle");
			auto const* x_axis = object.find("x_axis");
			auto const orientation_count =
				(rotation != nullptr ? 1 : 0) +
				(rotation_axis != nullptr || rotation_angle != nullptr ? 1 : 0) +
				(x_axis != nullptr ? 1 : 0);
			if (orientation_count > 1)
				throw std::runtime_error("A transform accepts only one of 'rotation', axis-angle rotation, or 'x_axis'");
			if ((rotation_axis == nullptr) != (rotation_angle == nullptr))
				throw std::runtime_error("Transform fields 'rotation_axis' and 'rotation_angle' must be supplied together");
			if (x_axis != nullptr && !allow_x_axis)
				throw std::runtime_error("Field 'x_axis' is only valid for constraint frames");

			if (rotation != nullptr)
			{
				auto const euler = ReadVec3(*rotation, 0.0f);
				return m4x4::TransformDeg(euler.x, euler.y, euler.z, position);
			}
			if (rotation_axis != nullptr)
			{
				auto const axis = ReadUnitAxis(*rotation_axis, "rotation_axis");
				auto const angle = rotation_angle->to<float>();
				if (!std::isfinite(angle))
					throw std::runtime_error("Transform field 'rotation_angle' must be finite");

				return m4x4::Transform(axis, DegreesToRadians(angle), position);
			}
			if (x_axis != nullptr)
				return m4x4::Transform(v4::XAxis(), ReadUnitAxis(*x_axis, "x_axis"), position);

			return m4x4::Translation(position);
		}

		// Read one six-dimensional motion with angular and linear components.
		v8motion ReadMotion(pr::json::Value const& value)
		{
			auto const& object = value.to_object();
			auto angular = v4::Zero();
			auto linear = v4::Zero();
			if (auto const* component = object.find("angular"))
				angular = ReadVec3(*component, 0.0f);
			if (auto const* component = object.find("linear"))
				linear = ReadVec3(*component, 0.0f);

			if (!IsFinite(angular) || !IsFinite(linear))
				throw std::runtime_error("Spatial velocity components must be finite");

			return v8motion{angular, linear};
		}

		// Read and validate one D6 axis mode, limit, or drive.
		physics::ConstraintAxisDesc ReadConstraintAxis(pr::json::Value const& value)
		{
			auto const& object = value.to_object();
			auto desc = physics::ConstraintAxisDesc{};
			auto mode = std::string("free");
			if (auto const* field = object.find("mode"))
				mode = field->to<std::string>();

			if (mode == "free")
				desc.m_mode = physics::EConstraintAxisMode::Free;
			else if (mode == "locked")
				desc.m_mode = physics::EConstraintAxisMode::Locked;
			else if (mode == "limited")
				desc.m_mode = physics::EConstraintAxisMode::Limited;
			else if (mode == "driven")
				desc.m_mode = physics::EConstraintAxisMode::Driven;
			else
				throw std::runtime_error(pr::FmtS("Unknown constraint axis mode '%s'", mode.c_str()));

			if (auto const* field = object.find("limits"))
			{
				auto const& limits = field->to_array();
				if (limits.size() != 2)
					throw std::runtime_error("Constraint axis 'limits' requires exactly two values");

				desc.m_limits = Range<float>{limits[0].to<float>(), limits[1].to<float>()};
				if (!std::isfinite(desc.m_limits.m_beg) || !std::isfinite(desc.m_limits.m_end) || desc.m_limits.m_beg > desc.m_limits.m_end)
					throw std::runtime_error("Constraint axis limits must be finite and ordered");
			}
			else if (desc.m_mode == physics::EConstraintAxisMode::Limited)
			{
				throw std::runtime_error("A limited constraint axis requires 'limits'");
			}

			if (auto const* field = object.find("target_position"))
				desc.m_target_position = field->to<float>();
			if (auto const* field = object.find("target_velocity"))
				desc.m_target_velocity = field->to<float>();
			if (auto const* field = object.find("stiffness"))
				desc.m_stiffness = field->to<float>();
			if (auto const* field = object.find("damping"))
				desc.m_damping = field->to<float>();
			if (auto const* field = object.find("max_force"))
				desc.m_max_force = field->to<float>();

			if (!std::isfinite(desc.m_target_position) || !std::isfinite(desc.m_target_velocity))
				throw std::runtime_error("Constraint axis targets must be finite");
			if (!std::isfinite(desc.m_stiffness) || desc.m_stiffness < 0.0f)
				throw std::runtime_error("Constraint axis stiffness must be finite and non-negative");
			if (!std::isfinite(desc.m_damping) || desc.m_damping < 0.0f)
				throw std::runtime_error("Constraint axis damping must be finite and non-negative");
			if (std::isnan(desc.m_max_force) || desc.m_max_force < 0.0f)
				throw std::runtime_error("Constraint axis maximum force must be non-negative");

			return desc;
		}

		// Read up to three D6 axis descriptions while preserving free defaults for omitted axes.
		void ReadConstraintAxes(pr::json::Value const& value, std::array<physics::ConstraintAxisDesc, 3>& axes, char const* field_name)
		{
			auto const& array = value.to_array();
			if (array.size() > axes.size())
				throw std::runtime_error(pr::FmtS("Constraint field '%s' accepts at most three axes", field_name));

			for (auto index = size_t{}; index != array.size(); ++index)
				axes[index] = ReadConstraintAxis(array[index]);
		}

		// Read one articulation joint as an ordered sequence of scalar screw motions.
		physics::ArticulationJointDesc ReadArticulationJoint(pr::json::Value const& value)
		{
			auto const& object = value.to_object();
			auto desc = physics::ArticulationJointDesc{};
			if (auto const* frame = object.find("parent_frame"))
				desc.m_joint_to_parent = ReadTransform(frame->to_object(), false);
			if (auto const* frame = object.find("child_frame"))
				desc.m_joint_to_child = ReadTransform(frame->to_object(), false);

			auto const* axes = object.find("axes");
			if (axes == nullptr)
				return desc;

			auto const& array = axes->to_array();
			if (array.size() > desc.m_axes.size())
				throw std::runtime_error("An articulation joint accepts at most six ordered axes");
			desc.m_dof_count = static_cast<int>(array.size());
			for (auto index = size_t{}; index != array.size(); ++index)
			{
				auto const& axis_object = array[index].to_object();
				auto const type = ReadRequiredString(axis_object, "type", "Articulation joint axis");
				if (type == "revolute")
					desc.m_axes[index].m_type = physics::EArticulationAxisType::Revolute;
				else if (type == "prismatic")
					desc.m_axes[index].m_type = physics::EArticulationAxisType::Prismatic;
				else
					throw std::runtime_error(pr::FmtS("Unknown articulation joint axis type '%s'", type.c_str()));

				auto const* direction = axis_object.find("axis");
				if (direction == nullptr)
					throw std::runtime_error("Articulation joint axis requires an 'axis' field");
				desc.m_axes[index].m_axis = ReadUnitAxis(*direction, "axis");

				if (auto const* field = axis_object.find("initial_position"))
					desc.m_initial_position[index] = field->to<float>();
				if (auto const* field = axis_object.find("initial_velocity"))
					desc.m_initial_velocity[index] = field->to<float>();
				if (!std::isfinite(desc.m_initial_position[index]) || !std::isfinite(desc.m_initial_velocity[index]))
					throw std::runtime_error("Articulation joint initial state must be finite");
			}

			return desc;
		}

		// Read explicit mass properties for a link that has no collision shape or needs an override.
		physics::Inertia ReadInertia(pr::json::Value const& value)
		{
			auto const& object = value.to_object();
			auto const type = ReadRequiredString(object, "type", "Articulation link inertia");
			auto const* mass_value = object.find("mass");
			if (mass_value == nullptr)
				throw std::runtime_error("Articulation link inertia requires a 'mass' field");

			auto const mass = mass_value->to<float>();
			auto centre_of_mass = v4::Zero();
			if (auto const* field = object.find("centre_of_mass"))
				centre_of_mass = ReadVec3(*field, 0.0f);
			if (!(mass > 0.0f) || !std::isfinite(mass) || !IsFinite(centre_of_mass))
				throw std::runtime_error("Articulation link inertia mass and centre of mass must be finite");

			if (type == "sphere")
			{
				auto const* radius = object.find("radius");
				if (radius == nullptr || !(radius->to<float>() > 0.0f))
					throw std::runtime_error("Spherical link inertia requires a positive 'radius'");

				return physics::Inertia::Sphere(radius->to<float>(), mass, centre_of_mass);
			}
			if (type == "box")
			{
				auto const* dimensions = object.find("dimensions");
				if (dimensions == nullptr)
					throw std::runtime_error("Box link inertia requires 'dimensions'");

				auto const size = ReadVec3(*dimensions, 0.0f);
				if (!(size.x > 0.0f) || !(size.y > 0.0f) || !(size.z > 0.0f) || !IsFinite(size))
					throw std::runtime_error("Box link inertia dimensions must be finite and positive");

				return physics::Inertia::Box(0.5f * size, mass, centre_of_mass);
			}
			if (type == "diagonal")
			{
				auto const* diagonal = object.find("diagonal");
				if (diagonal == nullptr)
					throw std::runtime_error("Diagonal link inertia requires 'diagonal'");

				auto const inertia_diagonal = ReadVec3(*diagonal, 0.0f);
				auto products = v4::Zero();
				if (auto const* field = object.find("products"))
					products = ReadVec3(*field, 0.0f);
				if (!IsFinite(inertia_diagonal) || !IsFinite(products))
					throw std::runtime_error("Explicit link inertia components must be finite");

				return physics::Inertia{inertia_diagonal, products, mass, centre_of_mass};
			}

			throw std::runtime_error(pr::FmtS("Unknown articulation link inertia type '%s'", type.c_str()));
		}

		// Read shape, mass, collision, colour, and buoyancy data shared by articulation links.
		ArticulationChildDesc ReadArticulationLink(pr::json::Value const& value, ShapeReader const& read_shape, bool root)
		{
			auto const& object = value.to_object();
			auto desc = ArticulationChildDesc{};
			desc.m_body.name = ReadRequiredString(object, "name", "Articulation link");
			desc.m_has_shape = object.find("shape") != nullptr;
			if (desc.m_has_shape)
			{
				desc.m_body = read_shape(value);
				desc.m_body.name = ReadRequiredString(object, "name", "Articulation link");
			}
			if (auto const* inertia = object.find("inertia"))
				desc.m_inertia = ReadInertia(*inertia);
			if (!desc.m_has_shape && !desc.m_inertia)
				throw std::runtime_error(pr::FmtS("Articulation link '%s' requires a collision 'shape' or explicit 'inertia'", desc.m_body.name.c_str()));
			if (!root && !desc.m_inertia && !desc.m_body.density && !(desc.m_body.mass > 0.0f))
				throw std::runtime_error(pr::FmtS("Moving articulation link '%s' requires positive 'mass' or 'density'", desc.m_body.name.c_str()));

			if (!root)
			{
				desc.m_parent_name = ReadRequiredString(object, "parent", "Articulation child link");
				auto const* joint = object.find("joint");
				if (joint == nullptr)
					throw std::runtime_error("Articulation child link requires a 'joint' field");
				desc.m_joint = ReadArticulationJoint(*joint);
			}
			if (auto const* transform = object.find("shape_transform"))
				desc.m_shape_to_link = ReadTransform(transform->to_object(), false);
			if (auto const* field = object.find("collide_parent"))
				desc.m_collide_parent = field->to<bool>();
			if (auto const* field = object.find("collide_self"))
				desc.m_collide_self = field->to<bool>();
			if (auto const* field = object.find("buoyant"))
				desc.m_buoyant = field->to<bool>();

			return desc;
		}

		// Read one complete reduced-coordinate articulation description.
		ArticulationDesc ReadArticulation(pr::json::Value const& value, ShapeReader const& read_shape)
		{
			auto const& object = value.to_object();
			auto desc = ArticulationDesc{};
			desc.m_name = ReadRequiredString(object, "name", "Articulation");

			auto root_type = std::string("fixed");
			if (auto const* field = object.find("root_type"))
				root_type = field->to<std::string>();
			if (root_type == "fixed")
				desc.m_root_type = physics::EArticulationRootType::Fixed;
			else if (root_type == "floating")
				desc.m_root_type = physics::EArticulationRootType::Floating;
			else
				throw std::runtime_error(pr::FmtS("Unknown articulation root type '%s'", root_type.c_str()));

			if (auto const* transform = object.find("root_transform"))
				desc.m_root_to_world = ReadTransform(transform->to_object(), false);
			if (auto const* velocity = object.find("root_velocity"))
				desc.m_root_velocity = ReadMotion(*velocity);
			if (desc.m_root_type == physics::EArticulationRootType::Fixed && (LengthSq(desc.m_root_velocity.ang) != 0.0f || LengthSq(desc.m_root_velocity.lin) != 0.0f))
				throw std::runtime_error("A fixed articulation root cannot have a root velocity");
			if (auto const* field = object.find("sleeping"))
				desc.m_sleeping = field->to<bool>();
			if (auto const* field = object.find("never_sleep"))
				desc.m_never_sleep = field->to<bool>();
			if (desc.m_sleeping && desc.m_never_sleep)
				throw std::runtime_error("An articulation cannot start sleeping and be marked never_sleep");

			auto const* root = object.find("root");
			if (root == nullptr)
				throw std::runtime_error("Articulation requires a 'root' link");
			desc.m_root = ReadArticulationLink(*root, read_shape, true);
			if (desc.m_root_type == physics::EArticulationRootType::Floating && !desc.m_root.m_inertia && !desc.m_root.m_body.density && !(desc.m_root.m_body.mass > 0.0f))
				throw std::runtime_error(pr::FmtS("Floating articulation root '%s' requires positive 'mass' or 'density'", desc.m_root.m_body.name.c_str()));

			if (auto const* links = object.find("links"))
			{
				desc.m_links.reserve(links->to_array().size());
				for (auto const& link : links->to_array())
					desc.m_links.push_back(ReadArticulationLink(link, read_shape, false));
			}

			return desc;
		}

		// Read one world, rigid-body, or articulation-link constraint endpoint.
		ConstraintFrameDesc ReadConstraintFrame(pr::json::Value const& value)
		{
			auto const& object = value.to_object();
			auto desc = ConstraintFrameDesc{};
			auto const* body = object.find("body");
			auto const* articulation = object.find("articulation");
			auto const* link = object.find("link");
			if (body != nullptr && (articulation != nullptr || link != nullptr))
				throw std::runtime_error("A constraint frame cannot reference both a rigid body and an articulation link");

			if (body != nullptr)
			{
				desc.m_body.m_body_name = body->to<std::string>();
				if (desc.m_body.m_body_name.empty())
					throw std::runtime_error("Constraint body name cannot be empty");
				desc.m_body.m_type = desc.m_body.m_body_name == "world"
					? BodyReferenceDesc::EType::World
					: BodyReferenceDesc::EType::RigidBody;
			}
			else if (articulation != nullptr && link != nullptr)
			{
				desc.m_body.m_type = BodyReferenceDesc::EType::ArticulationLink;
				desc.m_body.m_articulation_name = articulation->to<std::string>();
				desc.m_body.m_link_name = link->to<std::string>();
				if (desc.m_body.m_articulation_name.empty() || desc.m_body.m_link_name.empty())
					throw std::runtime_error("Constraint articulation and link names cannot be empty");
			}
			else
			{
				throw std::runtime_error("A constraint frame requires 'body' or both 'articulation' and 'link'");
			}

			desc.m_space = desc.m_body.m_type == BodyReferenceDesc::EType::World
				? ConstraintFrameDesc::ESpace::World
				: ConstraintFrameDesc::ESpace::Local;
			if (auto const* field = object.find("space"))
			{
				auto const space = field->to<std::string>();
				if (space == "local")
					desc.m_space = ConstraintFrameDesc::ESpace::Local;
				else if (space == "world")
					desc.m_space = ConstraintFrameDesc::ESpace::World;
				else
					throw std::runtime_error(pr::FmtS("Unknown constraint frame space '%s'", space.c_str()));
			}
			if (desc.m_body.m_type == BodyReferenceDesc::EType::World && desc.m_space != ConstraintFrameDesc::ESpace::World)
				throw std::runtime_error("The world constraint endpoint requires world-space frame coordinates");

			desc.m_frame_to_space = ReadTransform(object, true);
			return desc;
		}

		// Read one persistent constraint and its type-specific axis data.
		ConstraintDesc ReadConstraint(pr::json::Value const& value)
		{
			auto const& object = value.to_object();
			auto desc = ConstraintDesc{};
			if (auto const* field = object.find("name"))
				desc.m_name = field->to<std::string>();

			auto const type = ReadRequiredString(object, "type", "Constraint");
			if (type == "ball_socket")
				desc.m_type = ConstraintDesc::EType::BallSocket;
			else if (type == "hinge")
				desc.m_type = ConstraintDesc::EType::Hinge;
			else if (type == "slider")
				desc.m_type = ConstraintDesc::EType::Slider;
			else if (type == "weld")
				desc.m_type = ConstraintDesc::EType::Weld;
			else if (type == "d6")
				desc.m_type = ConstraintDesc::EType::D6;
			else
				throw std::runtime_error(pr::FmtS("Unknown constraint type '%s'", type.c_str()));

			auto const* frame_a = object.find("frame_a");
			auto const* frame_b = object.find("frame_b");
			if (frame_a == nullptr || frame_b == nullptr)
				throw std::runtime_error("Constraint requires 'frame_a' and 'frame_b'");
			desc.m_frame_a = ReadConstraintFrame(*frame_a);
			desc.m_frame_b = ReadConstraintFrame(*frame_b);

			if (auto const* field = object.find("axis"))
				desc.m_axis = ReadConstraintAxis(*field);
			else if (desc.m_type == ConstraintDesc::EType::Hinge || desc.m_type == ConstraintDesc::EType::Slider)
				throw std::runtime_error("Hinge and slider constraints require an 'axis' description");
			if (auto const* field = object.find("linear"))
				ReadConstraintAxes(*field, desc.m_linear, "linear");
			if (auto const* field = object.find("angular"))
				ReadConstraintAxes(*field, desc.m_angular, "angular");

			if (auto const* field = object.find("break_force"))
				desc.m_break_force = field->to<float>();
			if (auto const* field = object.find("break_torque"))
				desc.m_break_torque = field->to<float>();
			if (auto const* field = object.find("collide_connected"))
				desc.m_collide_connected = field->to<bool>();
			if (auto const* field = object.find("enabled"))
				desc.m_enabled = field->to<bool>();
			if (std::isnan(desc.m_break_force) || desc.m_break_force < 0.0f || std::isnan(desc.m_break_torque) || desc.m_break_torque < 0.0f)
				throw std::runtime_error("Constraint break thresholds must be non-negative");

			return desc;
		}
	}

	#if PR_UNITTESTS
	namespace physics_sandbox::scene_loader::tests
	{
		namespace
		{
			// Parse only the multibody section from an in-memory scene document so schema tests do not depend on deployment paths.
			SceneDesc ParseMultibodyDescriptions(std::string_view text)
			{
				auto document = pr::json::Read(text);
				auto const& scene = document.to_object()["scene"].to_object();
				auto desc = SceneDesc{};
				AppendMultibodyDescriptions(desc, scene, [](pr::json::Value const& value)
				{
					return ReadBody(value);
				});
				return desc;
			}
		}

		PRUnitTestClass(SceneLoaderMultibodyTests)
		{
			PRUnitTestMethod(ParsesArticulationsAndEveryConstraintType, Quick)
			{
				// Cover shaped and shape-less links, ordered joint axes, endpoint coordinate spaces, axis modes, and every public constraint type.
				auto const desc = ParseMultibodyDescriptions(R"json(
				{
					"scene": {
						"articulations": [{
							"name": "tree",
							"root_type": "floating",
							"root_transform": { "position": [1, 2, 3] },
							"root_velocity": { "angular": [0, 0, 1], "linear": [2, 0, 0] },
							"root": {
								"name": "root",
								"inertia": { "type": "box", "dimensions": [1, 2, 3], "mass": 4 }
							},
							"links": [{
								"name": "shaped",
								"parent": "root",
								"shape": { "type": "sphere", "radius": 0.5 },
								"mass": 2,
								"collide_parent": true,
								"collide_self": true,
								"joint": {
									"parent_frame": { "position": [0, 0, 1] },
									"child_frame": { "position": [0, 0, -1] },
									"axes": [
										{ "type": "revolute", "axis": [0, 0, 2], "initial_position": 0.25 },
										{ "type": "prismatic", "axis": [1, 0, 0], "initial_velocity": 0.5 }
									]
								}
							}, {
								"name": "mass-only",
								"parent": "shaped",
								"inertia": { "type": "sphere", "radius": 0.25, "mass": 1 },
								"joint": {}
							}]
						}],
						"constraints": [
							{
								"name": "ball",
								"type": "ball_socket",
								"frame_a": { "body": "world", "position": [1, 2, 3] },
								"frame_b": { "articulation": "tree", "link": "root", "space": "world", "position": [1, 2, 3] }
							}, {
								"name": "hinge",
								"type": "hinge",
								"frame_a": { "body": "world" },
								"frame_b": { "body": "rigid" },
								"axis": { "mode": "driven", "target_velocity": 2, "damping": 3, "max_force": 4 }
							}, {
								"name": "slider",
								"type": "slider",
								"frame_a": { "body": "world" },
								"frame_b": { "body": "rigid" },
								"axis": { "mode": "limited", "limits": [-1, 1] }
							}, {
								"name": "weld",
								"type": "weld",
								"frame_a": { "body": "world" },
								"frame_b": { "body": "rigid" },
								"break_force": 10,
								"break_torque": 20,
								"collide_connected": true,
								"enabled": false
							}, {
								"name": "d6",
								"type": "d6",
								"frame_a": { "body": "world", "x_axis": [0, 1, 0] },
								"frame_b": { "body": "rigid", "space": "local" },
								"linear": [
									{ "mode": "free" },
									{ "mode": "locked" },
									{ "mode": "limited", "limits": [-2, 2] }
								],
								"angular": [
									{ "mode": "driven", "target_position": 0.5, "stiffness": 6, "damping": 7, "max_force": 8 }
								]
							}
						]
					}
				})json");

				PR_EXPECT(desc.articulations.size() == 1);
				PR_EXPECT(desc.articulations[0].m_root_type == physics::EArticulationRootType::Floating);
				PR_EXPECT(desc.articulations[0].m_links.size() == 2);
				PR_EXPECT(desc.articulations[0].m_links[0].m_joint.m_dof_count == 2);
				PR_EXPECT(desc.articulations[0].m_links[0].m_joint.m_axes[0].m_type == physics::EArticulationAxisType::Revolute);
				PR_EXPECT(desc.articulations[0].m_links[0].m_joint.m_axes[0].m_axis.z == 1.0f);
				PR_EXPECT(desc.articulations[0].m_links[0].m_collide_parent);
				PR_EXPECT(desc.articulations[0].m_links[0].m_collide_self);
				PR_EXPECT(!desc.articulations[0].m_links[1].m_has_shape);
				PR_EXPECT(desc.articulations[0].m_links[1].m_inertia.has_value());

				PR_EXPECT(desc.constraints.size() == 5);
				PR_EXPECT(desc.constraints[0].m_type == ConstraintDesc::EType::BallSocket);
				PR_EXPECT(desc.constraints[0].m_frame_b.m_space == ConstraintFrameDesc::ESpace::World);
				PR_EXPECT(desc.constraints[1].m_type == ConstraintDesc::EType::Hinge);
				PR_EXPECT(desc.constraints[1].m_axis.m_mode == physics::EConstraintAxisMode::Driven);
				PR_EXPECT(desc.constraints[2].m_type == ConstraintDesc::EType::Slider);
				PR_EXPECT(desc.constraints[2].m_axis.m_mode == physics::EConstraintAxisMode::Limited);
				PR_EXPECT(desc.constraints[3].m_type == ConstraintDesc::EType::Weld);
				PR_EXPECT(!desc.constraints[3].m_enabled);
				PR_EXPECT(desc.constraints[4].m_type == ConstraintDesc::EType::D6);
				PR_EXPECT(desc.constraints[4].m_linear[1].m_mode == physics::EConstraintAxisMode::Locked);
				PR_EXPECT(desc.constraints[4].m_angular[0].m_mode == physics::EConstraintAxisMode::Driven);
			}

			PRUnitTestMethod(RejectsInvalidMultibodyDescriptors, Quick)
			{
				// Fail locally on invalid shape-less links, endpoint spaces, and axis modes before runtime objects can capture bad references.
				PR_THROWS(ParseMultibodyDescriptions(R"json({
					"scene": {
						"articulations": [{
							"name": "tree",
							"root": { "name": "root" }
						}]
					}
				})json"), std::exception);
				PR_THROWS(ParseMultibodyDescriptions(R"json({
					"scene": {
						"constraints": [{
							"type": "weld",
							"frame_a": { "body": "world", "space": "local" },
							"frame_b": { "body": "rigid" }
						}]
					}
				})json"), std::exception);
				PR_THROWS(ParseMultibodyDescriptions(R"json({
					"scene": {
						"constraints": [{
							"type": "hinge",
							"frame_a": { "body": "world" },
							"frame_b": { "body": "rigid" },
							"axis": { "mode": "unknown" }
						}]
					}
				})json"), std::exception);
			}
		};
	}
	#endif

	// Append articulation and persistent-constraint descriptions from the scene object.
	void AppendMultibodyDescriptions(SceneDesc& desc, pr::json::Object const& scene, ShapeReader const& read_shape)
	{
		if (auto const* articulations = scene.find("articulations"))
		{
			desc.articulations.reserve(articulations->to_array().size());
			for (auto const& articulation : articulations->to_array())
				desc.articulations.push_back(ReadArticulation(articulation, read_shape));
		}

		if (auto const* constraints = scene.find("constraints"))
		{
			desc.constraints.reserve(constraints->to_array().size());
			for (auto const& constraint : constraints->to_array())
				desc.constraints.push_back(ReadConstraint(constraint));
		}
	}
}
