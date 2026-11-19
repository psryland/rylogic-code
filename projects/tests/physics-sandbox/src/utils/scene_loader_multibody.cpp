#include "src/forward.h"
#include "src/utils/scene_loader_internal.h"
#if PR_UNITTESTS
#include "src/scene/scene.h"
#endif

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

		// Read a shaped, self-colliding body whose inertia is derived from its authored mass or density.
		BodyDesc ReadRigidRagdollBody(pr::json::Value const& value, ShapeReader const& read_shape)
		{
			auto const& object = value.to_object();
			if (object.find("shape") == nullptr || object.find("inertia") != nullptr)
				throw std::runtime_error("Rigid ragdoll links require a shape and shape-derived inertia");
			if (auto const* field = object.find("shape_transform"); field != nullptr && !FEql(ReadTransform(field->to_object(), false), m4x4::Identity()))
				throw std::runtime_error("Rigid ragdoll shapes must be centred in the body frame");
			if (auto const* field = object.find("collide_self"); field != nullptr && !field->to<bool>())
				throw std::runtime_error("Rigid ragdoll links require self-collision");

			auto body = read_shape(value);
			body.name = ReadRequiredString(object, "name", "Rigid ragdoll link");
			if (!body.density && !(body.mass > 0.0f))
				throw std::runtime_error("Rigid ragdoll links require positive mass or density");

			return body;
		}

		// Read a conventional hinge or three-axis connection while preserving its initial joint-frame motion.
		RigidRagdollLinkDesc ReadRigidRagdollLink(pr::json::Value const& value, ShapeReader const& read_shape)
		{
			auto const& object = value.to_object();
			auto desc = RigidRagdollLinkDesc{
				.m_body = ReadRigidRagdollBody(value, read_shape),
				.m_parent_name = ReadRequiredString(object, "parent", "Rigid ragdoll link"),
			};
			auto const* joint_value = object.find("joint");
			if (joint_value == nullptr)
				throw std::runtime_error("Rigid ragdoll child requires a joint");

			auto const& joint = joint_value->to_object();
			if (auto const* frame = joint.find("parent_frame"))
				desc.m_joint_to_parent = ReadTransform(frame->to_object(), false);
			if (auto const* frame = joint.find("child_frame"))
				desc.m_joint_to_child = ReadTransform(frame->to_object(), false);
			if (!IsFinite(desc.m_joint_to_parent) || !IsFinite(desc.m_joint_to_child))
				throw std::runtime_error("Rigid ragdoll joint frames must be finite");
			if (auto const* field = object.find("collide_parent"))
				desc.m_collide_parent = field->to<bool>();

			auto const* type = joint.find("type");
			auto const type_name = type != nullptr ? type->to<std::string>() : std::string("serial");
			if (type_name == "spherical")
			{
				// The initial quaternion is exact; only the runtime limit law is the conventional rotation-vector approximation.
				desc.m_joint_type = RigidRagdollLinkDesc::EJointType::Spherical;
				if (joint.find("axes") != nullptr)
					throw std::runtime_error("A spherical rigid connection does not accept serial axes");
				if (auto const* field = joint.find("initial_rotation"))
				{
					auto const& rotation = field->to_array();
					if (rotation.size() != 4)
						throw std::runtime_error("Initial rotation requires four quaternion components");

					for (auto i = 0; i != 4; ++i)
						desc.m_initial_rotation.xyzw[i] = rotation[i].to<float>();
				}
				if (auto const* field = joint.find("initial_angular_velocity"))
					desc.m_initial_angular_velocity = ReadVec3(*field, 0.0f);
			}
			else if (type_name == "serial")
			{
				// The retained ragdoll hinges have one positive-X rotational degree of freedom.
				auto const* axes = joint.find("axes");
				if (axes == nullptr || axes->to_array().size() != 1)
					throw std::runtime_error("A rigid ragdoll hinge requires one positive-X revolute axis");

				auto const& axis = axes->to_array()[0].to_object();
				auto const* direction = axis.find("axis");
				if (ReadRequiredString(axis, "type", "Rigid ragdoll axis") != "revolute" || direction == nullptr || !FEql(ReadUnitAxis(*direction, "axis"), v4::XAxis()))
					throw std::runtime_error("A rigid ragdoll hinge requires one positive-X revolute axis");

				auto angle = 0.0f;
				if (auto const* field = axis.find("initial_position"))
					angle = field->to<float>();
				if (auto const* field = axis.find("initial_velocity"))
					desc.m_initial_angular_velocity.x = field->to<float>();
				if (!std::isfinite(angle))
					throw std::runtime_error("Rigid ragdoll initial angle must be finite");

				desc.m_initial_rotation = ToQuat<quat>(m4x4::Transform(v4::XAxis(), angle, v4::Origin()).rot);
			}
			else
			{
				throw std::runtime_error(std::format("Unknown rigid ragdoll joint type '{}'", type_name));
			}

			// Named source limits map to independent D6 rotation-vector components in the authored joint frame.
			if (auto const* limits = joint.find("limits"))
			{
				auto const& limits_object = limits->to_object();
				auto read_range = [&](char const* name, int axis)
				{
					auto const* field = limits_object.find(name);
					if (field == nullptr)
						return;

					auto const& range = field->to_array();
					if (range.size() != 2)
						throw std::runtime_error("Rigid ragdoll limits require two ordered values");

					desc.m_limits[axis] = {range[0].to<float>(), range[1].to<float>()};
					if (std::isnan(desc.m_limits[axis].m_beg) || std::isnan(desc.m_limits[axis].m_end) || desc.m_limits[axis].m_beg > desc.m_limits[axis].m_end)
						throw std::runtime_error("Rigid ragdoll limits must be ordered and not NaN");
				};
				switch (desc.m_joint_type)
				{
					case RigidRagdollLinkDesc::EJointType::Hinge: { read_range("position", 0); break; }
					case RigidRagdollLinkDesc::EJointType::Spherical: { read_range("swing_x", 0); read_range("twist", 1); read_range("swing_z", 2); break; }
					default: { throw std::runtime_error("Unknown rigid ragdoll joint type"); }
				}
			}
			if (auto const* field = joint.find("damping"))
				desc.m_damping = field->to<float>();
			if (!std::isfinite(desc.m_damping) || !(desc.m_damping > 0.0f))
				throw std::runtime_error("Rigid ragdoll torque-to-target conversion requires finite positive damping");
			if (!IsFinite(desc.m_initial_rotation.xyzw) || !IsNormalised(desc.m_initial_rotation.xyzw) || !IsFinite(desc.m_initial_angular_velocity))
				throw std::runtime_error("Rigid ragdoll initial motion requires a finite unit quaternion and finite angular velocity");

			return desc;
		}

		// Read the explicitly selected conventional model without changing reduced-coordinate articulation authoring.
		RigidRagdollDesc ReadRigidRagdoll(pr::json::Value const& value, ShapeReader const& read_shape)
		{
			auto const& object = value.to_object();
			auto desc = RigidRagdollDesc{};
			desc.m_name = ReadRequiredString(object, "name", "Rigid ragdoll");
			if (auto const* type = object.find("root_type"); type != nullptr && type->to<std::string>() != "floating")
				throw std::runtime_error("Rigid ragdolls require a floating root");
			if (auto const* field = object.find("root_transform"))
				desc.m_root_to_world = ReadTransform(field->to_object(), false);
			if (!IsFinite(desc.m_root_to_world))
				throw std::runtime_error("Rigid ragdoll root transform must be finite");
			if (auto const* field = object.find("root_velocity"))
				desc.m_root_velocity = ReadMotion(*field);
			if (auto const* field = object.find("sleeping"))
				desc.m_sleeping = field->to<bool>();
			if (auto const* field = object.find("never_sleep"))
				desc.m_never_sleep = field->to<bool>();
			if (desc.m_sleeping && desc.m_never_sleep)
				throw std::runtime_error("A rigid ragdoll cannot start sleeping and be marked never_sleep");

			if (auto const* field = object.find("actuation"))
			{
				auto const& actuation = field->to_object();
				if (auto const* item = actuation.find("torque_per_kg"))
					desc.m_actuation.m_torque_per_kg = item->to<float>();
				if (auto const* item = actuation.find("period_s"))
					desc.m_actuation.m_period_s = item->to<float>();
				if (auto const* item = actuation.find("phase"))
					desc.m_actuation.m_phase = item->to<float>();
				if (!std::isfinite(desc.m_actuation.m_torque_per_kg) || desc.m_actuation.m_torque_per_kg < 0.0f || !std::isfinite(desc.m_actuation.m_period_s) || !(desc.m_actuation.m_period_s > 0.0f) || !std::isfinite(desc.m_actuation.m_phase))
					throw std::runtime_error("Rigid ragdoll actuation requires finite nonnegative torque, positive period and finite phase");
			}

			auto const* root = object.find("root");
			if (root == nullptr)
				throw std::runtime_error("Rigid ragdoll requires a root body");

			desc.m_root = ReadRigidRagdollBody(*root, read_shape);
			if (auto const* links = object.find("links"))
			{
				for (auto const& link : links->to_array())
					desc.m_links.push_back(ReadRigidRagdollLink(link, read_shape));
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

		// Exercise the conventional authoring boundary without relying on zero-pose benchmark initialization.
		PRUnitTestClass(RigidRagdollAuthoringTests)
		{
			PRUnitTestMethod(LoadsBranchedFramesAndPointVelocities, Extended)
			{
				auto const desc = ParseMultibodyDescriptions(R"json({"scene":{"rigid_ragdolls":[{
					"name":"branched", "root_type":"floating", "never_sleep":true,
					"root_transform":{"position":[10,20,30],"rotation":[0,0,90]},
					"root_velocity":{"angular":[1,2,3],"linear":[4,5,6]},
					"actuation":{"torque_per_kg":1.5,"period_s":1.3,"phase":0.3},
					"root":{"name":"root","mass":4,"shape":{"type":"box","dimensions":[1,1,1]}},
					"links":[
						{"name":"hinge","parent":"root","mass":2,"shape":{"type":"box","dimensions":[1,1,1]},
						 "joint":{"type":"serial","parent_frame":{"position":[1,0,0],"rotation":[0,90,0]},
						 "child_frame":{"position":[0,2,0],"rotation":[0,0,90]},
						 "axes":[{"type":"revolute","axis":[1,0,0],"initial_position":1.5707963267948966,"initial_velocity":2}],
						 "limits":{"position":[-2,2]},"damping":0.5}},
						{"name":"sphere","parent":"root","mass":3,"shape":{"type":"box","dimensions":[1,1,1]},
						 "joint":{"type":"spherical","parent_frame":{"position":[0,0,3],"rotation":[90,0,0]},
						 "child_frame":{"position":[1,0,0],"rotation":[0,90,0]},
						 "initial_rotation":[0,0,0.7071067811865476,0.7071067811865476],
						 "initial_angular_velocity":[1,-1,2],
						 "limits":{"swing_x":[-2,2],"twist":[-1,1],"swing_z":[-0.5,0.5]},"damping":0.75}}
					]}]}})json");
				auto scene = Scene(nullptr);
				scene.LoadScene(desc);
				PR_EXPECT(scene.m_articulation.empty() && scene.m_body.size() == 3 && scene.m_constraints.Count() == 4);
				PR_EXPECT(scene.m_rigid_ragdoll_joints.size() == 2);
				PR_EXPECT(scene.m_rigid_ragdoll_joints[0].m_parent == 0 && scene.m_rigid_ragdoll_joints[1].m_parent == 0);
				PR_EXPECT(scene.m_rigid_ragdoll_joints[0].m_dof_count == 1 && scene.m_rigid_ragdoll_joints[1].m_dof_count == 3);
				PR_EXPECT(scene.m_rigid_ragdoll_joints[1].m_dof_offset == 1);
				PR_EXPECT(scene.m_rigid_ragdoll_joints[0].m_amplitude == 3.0f && scene.m_rigid_ragdoll_joints[1].m_amplitude == 4.5f);

				// These world-space values follow from the authored quarter-turn axes and coincident joint-point velocities.
				// They do not reuse the builder's child-to-parent transform or spatial-motion adjoint.
				auto const poses = std::array{
					m4x4{v4{0,1,0,0}, v4{-1,0,0,0}, v4{0,0,1,0}, v4{10,20,30,1}},
					m4x4{v4{0,-1,0,0}, v4{0,0,-1,0}, v4{1,0,0,0}, v4{10,21,32,1}},
					m4x4{v4{1,0,0,0}, v4{0,-1,0,0}, v4{0,0,-1,0}, v4{9,20,33,1}},
				};
				auto const angular = std::array{v4{-2,1,3,0}, v4{-2,1,1,0}, v4{0,2,4,0}};
				auto const linear = std::array{v4{-5,4,6,0}, v4{-6,8,4,0}, v4{-2,6,8,0}};
				for (auto i = 0; i != 3; ++i)
				{
					auto const& body = scene.m_body[i];
					for (auto axis = 0; axis != 4; ++axis)
						PR_EXPECT(Length(body.O2W()[axis] - poses[i][axis]) < 1.0e-5f);

					PR_EXPECT(Length(body.VelocityWS().ang - angular[i]) < 1.0e-5f);
					PR_EXPECT(Length(body.VelocityWS().lin - linear[i]) < 1.0e-5f);
				}
				auto const hinge_motion = scene.m_body[1].VelocityWS();
				auto const sphere_motion = scene.m_body[2].VelocityWS();
				PR_EXPECT(Length(hinge_motion.lin + Cross(hinge_motion.ang, scene.m_body[1].O2W().x) - v4{-5,8,6,0}) < 1.0e-5f);
				PR_EXPECT(Length(sphere_motion.lin + Cross(sphere_motion.ang, scene.m_body[2].O2W().y) - v4{2,6,8,0}) < 1.0e-5f);
				auto const& anchor = scene.m_constraints.Get(scene.m_rigid_ragdoll_joints[1].m_anchor);
				auto const& actuator = scene.m_constraints.Get(scene.m_rigid_ragdoll_joints[1].m_actuator);
				PR_EXPECT(anchor.m_angular[1].m_limits.m_beg == -1.0f && anchor.m_angular[2].m_limits.m_end == 0.5f);
				PR_EXPECT(actuator.m_angular[2].m_damping == 0.75f);
				PR_EXPECT(actuator.m_frame_a.m_body == physics::BodyRef::Rigid(scene.m_body[2]));
				PR_EXPECT(actuator.m_frame_b.m_body == physics::BodyRef::Rigid(scene.m_body[0]));
			}

			PRUnitTestMethod(RejectsUnsupportedConventionalInputs, Quick)
			{
				auto const prefix = std::string(R"json({"scene":{"rigid_ragdolls":[{"name":"test","root":{"name":"root","mass":1,"shape":{"type":"box","dimensions":[1,1,1]}},"links":[{"name":"child","parent":"root","mass":1,"shape":{"type":"box","dimensions":[1,1,1]},"joint":)json");
				auto const suffix = std::string("}]}]}}");
				PR_EXPECT(ParseMultibodyDescriptions(prefix + R"json({"type":"spherical","damping":1})json" + suffix).rigid_ragdolls[0].m_links.size() == 1);
				PR_THROWS(ParseMultibodyDescriptions(prefix + R"json({"type":"spherical","damping":0})json" + suffix), std::exception);
				PR_THROWS(ParseMultibodyDescriptions(prefix + R"json({"type":"spherical","damping":1,"initial_rotation":[0,0,0,2]})json" + suffix), std::exception);
				PR_THROWS(ParseMultibodyDescriptions(prefix + R"json({"type":"serial","damping":1,"axes":[{"type":"revolute","axis":[0,1,0]}]})json" + suffix), std::exception);
				PR_THROWS(ParseMultibodyDescriptions(prefix + R"json({"type":"spherical","damping":1,"limits":{"twist":[2,-2]}})json" + suffix), std::exception);
			}
		};

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

		if (auto const* ragdolls = scene.find("rigid_ragdolls"))
		{
			for (auto const& ragdoll : ragdolls->to_array())
				desc.rigid_ragdolls.push_back(ReadRigidRagdoll(ragdoll, read_shape));
		}
	}
}
