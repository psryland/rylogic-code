//*****************************************************************************
// Procedural animation - rig description
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include "pr/animation/forward.h"

namespace pr::animation
{
	// The role of a chain of bones. The role decides which solver poses the chain.
	enum class EChainRole
	{
		// Bones from the pelvis to the chest, fitted to a smooth curve between the pelvis and chest frames
		Spine,

		// Bones from the base of the neck to the head. The head turns to face an aim point, with the rotation shared along the chain.
		Neck,

		// Three bones (upper, lower, end) posed by two-bone IK. For example: thigh, shin, foot.
		Leg,

		// Three bones (upper, lower, end) posed by two-bone IK. For example: upper arm, forearm, hand.
		Arm,

		// Bones from the tail base to the tip, posed by FABRIK
		Tail,

		// Toes and fingers. Not posed by the core solvers; their local transforms are kept.
		Digit,
	};

	// Which side of the body a chain is on
	enum class ESide
	{
		Centre,
		Left,
		Right,
	};

	// A bone's rotation limit relative to its rest orientation under its parent.
	//  - The bone's rotation from rest is split into a swing and a twist. The twist is the rotation about the bone's own +Y axis. The swing then moves
	//    the bone's +Y axis, and is limited about the bone's rest +X and +Z axes (see 'fabrik::SwingLimit'). So a hinge is a swing range on one axis
	//    with a zero-width range on the other, and a ball joint such as a hip or shoulder uses both swing ranges and a twist range.
	//  - Bone roll decides the swing axes, so roll a bone until its rest +X axis is the main bend axis of its joint.
	//  - Every range must lie in [-pi, pi] and contain zero, so the rest pose is always allowed. The default allows any rotation.
	//  - Tail chains pass the swing limit to FABRIK, which only solves bone directions and so ignores twist (see fabrik.h).
	//  - Neck chains limit both swing and twist.
	//  - Other chains ignore limits.
	struct JointLimit
	{
		static constexpr float pi = math::constants<float>::tau_by_2;

		fabrik::SwingLimit<float> swing;  // The allowed swing of the bone's +Y axis about its rest +X and +Z axes
		float min_twist = -pi;            // The allowed twist (radians) about the bone's +Y axis
		float max_twist = +pi;

		// True if the limit allows any rotation
		bool IsFree() const
		{
			return swing.IsFree() && min_twist <= -pi && max_twist >= pi;
		}

		// True if the ranges are within [-pi, pi] and contain zero
		bool IsValid() const
		{
			return swing.IsValid() && -pi <= min_twist && min_twist <= 0 && 0 <= max_twist && max_twist <= pi;
		}

		// A bone rotation split into a swing vector (radians about the bone's +X and +Z axes, see 'fabrik::SwingLimit') and a twist about +Y
		struct SwingTwist
		{
			v2 swing;
			float twist;
		};

		// Split 'rot', a rotation from the rest orientation in bone space ('~rest.rot * local.rot'), into its swing and twist. 'rot' is 'swing * twist'.
		static SwingTwist Decompose(quat rot)
		{
			// Use the positive-w form so that the twist angle is within [-pi, pi]
			if (rot.w < 0)
				rot = quat(-rot.x, -rot.y, -rot.z, -rot.w);

			// The twist is the part of the rotation about +Y. When that part is zero the bone is swung by pi, and the twist is taken as zero.
			auto twist_len = std::sqrt(rot.y * rot.y + rot.w * rot.w);
			auto twist = twist_len > 0 ? quat(0, rot.y / twist_len, 0, rot.w / twist_len) : Identity<quat>();
			auto twist_angle = 2.0f * std::atan2(twist.y, twist.w);

			// The swing is what remains. Its axis is in the bone's XZ plane, and its w is not negative.
			auto sw = rot * ~twist;
			auto sin_half = std::sqrt(sw.x * sw.x + sw.z * sw.z);
			auto swing_angle = 2.0f * std::atan2(sin_half, sw.w);
			auto scale = sin_half > 0 ? swing_angle / sin_half : 2.0f;
			return SwingTwist{ .swing = v2(sw.x * scale, sw.z * scale), .twist = twist_angle };
		}

		// Combine a swing and twist into a rotation from the rest orientation in bone space. The inverse of 'Decompose'.
		static quat Compose(SwingTwist st)
		{
			// Build the swing from its swing vector, then apply the twist first
			auto swing_angle = Length(st.swing);
			auto swing_rot = swing_angle > 0 ? quat(v4(st.swing.x / swing_angle, 0, st.swing.y / swing_angle, 0), swing_angle) : Identity<quat>();
			return Normalise(swing_rot * quat(v4::YAxis(), st.twist));
		}

		// Return the allowed rotation nearest to 'rot', a rotation from the rest orientation in bone space. The swing and twist are clamped separately.
		quat Clamp(quat rot) const
		{
			// Clamp each part within its own range
			auto st = Decompose(rot);
			st.swing = swing.Clamp(st.swing);
			st.twist = math::Clamp(st.twist, min_twist, max_twist);
			return Compose(st);
		}
	};

	// One bone of the skeleton
	struct Bone
	{
		std::string name;         // Unique, non-empty name
		int parent = -1;          // The index of the parent bone, or -1 for a root bone. Parents come before their children.
		xform rest;               // The rest transform relative to the parent (model space for root bones). Unit scale.
		float length = 0.0f;      // The distance from the bone's head to its tip along its local +Y axis. Must be positive.
		JointLimit limit;         // Rotation limit relative to the parent
		float stiffness = 0.0f;   // In [0,1]. Tail chains: how strongly the bone returns towards its rest direction (see fabrik.h).
	};

	// A chain of bones with a role
	struct Chain
	{
		EChainRole role = EChainRole::Spine;
		ESide side = ESide::Centre;
		std::vector<int> bones;   // Bone indices from the chain's base to its end. Each bone is the parent of the next.
		v4 pole = {};             // Leg, Arm: the rest-pose model-space direction that the middle joint bends towards (unit, w = 0)
		v4 forward = {};          // Neck: the rest-pose model-space direction that the last bone faces (unit, w = 0)
	};

	// The part of the body that a contact point belongs to
	enum class EContactKind
	{
		Sole,
		Toe,
		Palm,
		Finger,
		Tail,
	};

	// A named point on a bone where the body touches the ground or an object. For example: a sole, a toe tip, or a palm.
	struct Contact
	{
		std::string name;                         // Unique, non-empty name
		int bone = -1;                            // The index of the bone that the contact moves with
		v4 offset = v4::Origin();                 // The contact point in bone space (w = 1)
		v4 normal = v4::ZAxis();                  // The outward surface direction at the contact, in bone space (unit, w = 0)
		EContactKind kind = EContactKind::Sole;
	};

	// The line through the body that a collision shape follows. It runs from the head of bone 'from' to the head of bone 'to'.
	struct BodyAxis
	{
		int from = -1;            // The bone whose head is the start of the axis, e.g. the hips
		int to = -1;              // The bone whose head is the end of the axis, e.g. the neck
	};

	// A complete rig description, as authored and stored. 'Rig' is the validated runtime form.
	struct RigDesc
	{
		std::vector<Bone> bones;
		std::vector<Chain> chains;
		std::vector<Contact> contacts;
		BodyAxis body_axis;
	};

	// Return the model-space rest transforms of 'bones'. Parents must come before their children.
	inline std::vector<xform> RestModelTransforms(std::span<Bone const> bones)
	{
		// Compose each bone's rest transform with its parent's
		std::vector<xform> rest_model(bones.size());
		for (int i = 0, iend = static_cast<int>(bones.size()); i != iend; ++i)
		{
			auto const& bone = bones[i];
			rest_model[i] = bone.parent < 0 ? bone.rest : rest_model[bone.parent] * bone.rest;
		}
		return rest_model;
	}

	// Return the FABRIK joints for a tail chain. Joint 0 is the head of the chain's first bone. Joint 'k' (k >= 1) is the end of chain bone 'k - 1':
	// the head of the next chain bone, or the tip of the last bone.
	inline std::vector<fabrik::Joint<float>> TailJoints(std::span<Bone const> bones, std::span<xform const> rest_model, Chain const& chain)
	{
		// The rest positions of the joints
		auto count = static_cast<int>(chain.bones.size());
		std::vector<v4> positions(count + 1);
		for (int k = 0; k != count; ++k)
			positions[k] = rest_model[chain.bones[k]].pos;

		auto const& last = rest_model[chain.bones.back()];
		positions[count] = last * v4(0, bones[chain.bones.back()].length, 0, 1);

		// One FABRIK joint per position, each with the limit and stiffness of the bone that ends there.
		// The swing axis is the bone's rest +X axis, made exactly perpendicular to the link in case the link is not along the bone's +Y axis.
		std::vector<fabrik::Joint<float>> joints(count + 1);
		for (int k = 1; k <= count; ++k)
		{
			auto const& bone = bones[chain.bones[k - 1]];
			auto offset = positions[k] - positions[k - 1];
			auto rest_dir = Normalise(offset);
			auto bone_x = Rotate(rest_model[chain.bones[k - 1]].rot, v4::XAxis());
			joints[k] = fabrik::Joint<float>{
				.parent = k - 1,
				.rest_dir = rest_dir,
				.length = Length(offset),
				.swing_axis = Normalise(bone_x - rest_dir * Dot(bone_x, rest_dir), Perpendicular(rest_dir)),
				.limit = bone.limit.swing,
				.stiffness = bone.stiffness,
			};
		}
		return joints;
	}

	// Check a rig description. Returns a description of each problem found, or an empty list if the rig is valid.
	inline std::vector<std::string> Validate(RigDesc const& desc)
	{
		std::vector<std::string> errors;
		auto const& bones = desc.bones;
		auto const& chains = desc.chains;
		auto bone_count = static_cast<int>(bones.size());

		// Check each bone on its own
		if (bones.empty())
			errors.push_back("The rig has no bones");

		for (int i = 0; i != bone_count; ++i)
		{
			// Names must be present and unique so that the rig can be matched to its skin
			auto const& bone = bones[i];
			if (bone.name.empty())
				errors.push_back(std::format("Bone {} has no name", i));
			else if (std::any_of(bones.begin(), bones.begin() + i, [&](Bone const& b) { return b.name == bone.name; }))
				errors.push_back(std::format("Bone '{}' is not uniquely named", bone.name));

			// Parents must come first so that transforms can be composed in one pass
			if (bone.parent < -1 || bone.parent >= i)
				errors.push_back(std::format("Bone '{}' has parent {}, which is not an earlier bone", bone.name, bone.parent));

			// The rest transform must be a rigid transform
			if (!FEqlAbsolute(Length(bone.rest.rot), 1.0f, 1e-4f))
				errors.push_back(std::format("Bone '{}' has a non-unit rest rotation", bone.name));
			if (!FEqlAbsolute(bone.rest.scl.xyz, v4::One().xyz, 1e-4f))
				errors.push_back(std::format("Bone '{}' has a non-unit rest scale", bone.name));
			if (!(bone.length > 0))
				errors.push_back(std::format("Bone '{}' has no length", bone.name));
			if (bone.stiffness < 0 || bone.stiffness > 1)
				errors.push_back(std::format("Bone '{}' has stiffness outside [0,1]", bone.name));

			// The limit ranges must contain the rest pose
			if (!bone.limit.IsValid())
				errors.push_back(std::format("Bone '{}' has a limit range outside [-pi, pi] or not containing zero", bone.name));
		}

		// Chain checks need a consistent hierarchy
		if (!errors.empty())
			return errors;

		auto rest_model = RestModelTransforms(bones);
		std::vector<int> owner(bone_count, -1);
		for (int c = 0, cend = static_cast<int>(chains.size()); c != cend; ++c)
		{
			// Each chain is a run of parent-child bones, and each bone belongs to at most one chain
			auto const& chain = chains[c];
			if (chain.bones.empty())
			{
				errors.push_back(std::format("Chain {} has no bones", c));
				continue;
			}
			auto valid_indices = true;
			for (int k = 0, kend = static_cast<int>(chain.bones.size()); k != kend; ++k)
			{
				// Bone indices must be in range before the hierarchy can be checked
				auto b = chain.bones[k];
				if (b < 0 || b >= bone_count)
				{
					errors.push_back(std::format("Chain {} refers to bone {}, which does not exist", c, b));
					valid_indices = false;
					continue;
				}
				if (owner[b] != -1)
					errors.push_back(std::format("Bone '{}' is in chains {} and {}", bones[b].name, owner[b], c));
				if (k != 0 && chain.bones[k - 1] >= 0 && chain.bones[k - 1] < bone_count && bones[b].parent != chain.bones[k - 1])
					errors.push_back(std::format("Chain {}: bone '{}' is not a child of the previous bone in the chain", c, bones[b].name));

				owner[b] = c;
			}
			if (!valid_indices)
				continue;

			// Check the role-specific requirements
			auto head = [&](int k) { return rest_model[chain.bones[k]].pos; };
			auto count = static_cast<int>(chain.bones.size());
			switch (chain.role)
			{
				case EChainRole::Spine:
				{
					// The curve fit needs at least the pelvis and chest, with separated joints
					if (count < 2)
						errors.push_back(std::format("Spine chain {} needs at least two bones", c));

					for (int k = 1; k < count; ++k)
					{
						if (FEqlAbsolute(head(k), head(k - 1), 1e-6f))
							errors.push_back(std::format("Spine chain {}: bone '{}' has the same head as its parent", c, bones[chain.bones[k]].name));
					}
					break;
				}
				case EChainRole::Neck:
				{
					// The aim needs a facing direction
					if (!FEqlAbsolute(Length(chain.forward), 1.0f, 1e-4f) || chain.forward.w != 0)
						errors.push_back(std::format("Neck chain {} needs a unit 'forward' direction", c));

					break;
				}
				case EChainRole::Leg:
				case EChainRole::Arm:
				{
					// Two-bone IK needs exactly three bones and a pole that defines a bend plane
					if (count != 3)
					{
						errors.push_back(std::format("Limb chain {} needs exactly three bones (upper, lower, end)", c));
						break;
					}
					if (FEqlAbsolute(head(1), head(0), 1e-6f) || FEqlAbsolute(head(2), head(1), 1e-6f))
						errors.push_back(std::format("Limb chain {} has a zero-length upper or lower bone", c));
					if (!FEqlAbsolute(Length(chain.pole), 1.0f, 1e-4f) || chain.pole.w != 0)
						errors.push_back(std::format("Limb chain {} needs a unit 'pole' direction", c));
					else if (Length(Cross(Normalise(head(2) - head(0), v4::Zero()), chain.pole)) < 0.1f)
						errors.push_back(std::format("Limb chain {} has a pole that is (nearly) parallel to the limb", c));

					break;
				}
				case EChainRole::Tail:
				{
					// The tail joints must form a valid FABRIK chain
					auto joints = TailJoints(bones, rest_model, chain);
					if (!fabrik::IsValid<float>(joints))
						errors.push_back(std::format("Tail chain {} has zero-length links or invalid limits", c));

					break;
				}
				case EChainRole::Digit:
				{
					break;
				}
				default:
				{
					throw std::invalid_argument(std::format("Chain {} has an unknown role", c));
				}
			}
		}

		// Contacts must name a bone and give a surface direction so that they can be placed on terrain
		for (int i = 0, iend = static_cast<int>(desc.contacts.size()); i != iend; ++i)
		{
			auto const& contact = desc.contacts[i];
			if (contact.name.empty())
				errors.push_back(std::format("Contact {} has no name", i));
			else if (std::any_of(desc.contacts.begin(), desc.contacts.begin() + i, [&](Contact const& c) { return c.name == contact.name; }))
				errors.push_back(std::format("Contact '{}' is not uniquely named", contact.name));

			if (contact.bone < 0 || contact.bone >= bone_count)
				errors.push_back(std::format("Contact '{}' refers to bone {}, which does not exist", contact.name, contact.bone));
			if (contact.offset.w != 1)
				errors.push_back(std::format("Contact '{}' has an offset that is not a point (w != 1)", contact.name));
			if (!FEqlAbsolute(Length(contact.normal), 1.0f, 1e-4f) || contact.normal.w != 0)
				errors.push_back(std::format("Contact '{}' needs a unit 'normal' direction", contact.name));

			switch (contact.kind)
			{
				case EContactKind::Sole:
				case EContactKind::Toe:
				case EContactKind::Palm:
				case EContactKind::Finger:
				case EContactKind::Tail:
				{
					break;
				}
				default:
				{
					throw std::invalid_argument(std::format("Contact '{}' has an unknown kind", contact.name));
				}
			}
		}

		// The body axis needs two existing bones with separate heads, so that it has a direction
		auto const& axis = desc.body_axis;
		if (axis.from < 0 || axis.from >= bone_count || axis.to < 0 || axis.to >= bone_count)
			errors.push_back(std::format("The body axis refers to bones {} and {}, which must both exist", axis.from, axis.to));
		else if (FEqlAbsolute(rest_model[axis.from].pos, rest_model[axis.to].pos, 1e-6f))
			errors.push_back(std::format("The body axis bones '{}' and '{}' have the same head", bones[axis.from].name, bones[axis.to].name));

		return errors;
	}

	// Return 'desc' with its bones reordered to match 'bone_order', which must name each bone exactly once. Bone references are remapped.
	// Throws 'std::invalid_argument' if the names do not match. The result needs validating, because the new order may put a child before its parent.
	inline RigDesc ReorderBones(RigDesc const& desc, std::span<std::string const> bone_order)
	{
		// Map each old bone index to its new index
		auto bone_count = static_cast<int>(desc.bones.size());
		if (static_cast<int>(bone_order.size()) != bone_count)
			throw std::invalid_argument(std::format("The rig has {} bones but the new order has {}", bone_count, bone_order.size()));

		std::vector<int> new_index(bone_count, -1);
		for (int n = 0; n != bone_count; ++n)
		{
			// Each name in the new order must match exactly one unclaimed rig bone
			auto it = std::find_if(desc.bones.begin(), desc.bones.end(), [&](Bone const& b) { return b.name == bone_order[n]; });
			if (it == desc.bones.end())
				throw std::invalid_argument(std::format("Bone '{}' is not in the rig", bone_order[n]));

			auto old = static_cast<int>(it - desc.bones.begin());
			if (new_index[old] != -1)
				throw std::invalid_argument(std::format("Bone '{}' appears more than once in the new order", bone_order[n]));

			new_index[old] = n;
		}
		auto remap = [&](int b) { return b < 0 || b >= bone_count ? b : new_index[b]; };

		// Rest transforms are parent-relative, so they are unchanged when the parents are unchanged
		RigDesc result = desc;
		for (int old = 0; old != bone_count; ++old)
		{
			auto& bone = result.bones[new_index[old]];
			bone = desc.bones[old];
			bone.parent = remap(bone.parent);
		}
		for (auto& chain : result.chains)
		{
			for (auto& b : chain.bones)
				b = remap(b);
		}
		for (auto& contact : result.contacts)
			contact.bone = remap(contact.bone);

		result.body_axis = BodyAxis{ .from = remap(desc.body_axis.from), .to = remap(desc.body_axis.to) };
		return result;
	}

	// A validated rig: the skeleton plus its chains, contacts and body axis, with derived rest-pose data
	struct Rig
	{
		std::vector<Bone> bones;
		std::vector<Chain> chains;
		std::vector<Contact> contacts;
		BodyAxis body_axis;
		std::vector<xform> rest_model;                          // The model-space rest transform of each bone
		std::vector<int> solve_order;                          // Chain indices ordered so that ancestor chains are solved first
		std::vector<std::vector<fabrik::Joint<float>>> tails;  // FABRIK joints for each tail chain (empty for other roles)

		// Construct from a rig description. Throws 'std::invalid_argument' listing every problem if the description is invalid.
		explicit Rig(RigDesc desc)
			: bones()
			, chains()
			, contacts()
			, body_axis()
			, rest_model()
			, solve_order()
			, tails()
		{
			// Reject invalid descriptions at the load boundary so the solvers can rely on them
			auto errors = Validate(desc);
			if (!errors.empty())
			{
				std::string msg = "Invalid rig description:";
				for (auto const& e : errors)
					msg.append("\n  ").append(e);

				throw std::invalid_argument(msg);
			}
			bones = std::move(desc.bones);
			chains = std::move(desc.chains);
			contacts = std::move(desc.contacts);
			body_axis = desc.body_axis;

			// Cache the rest pose and per-chain solver data
			rest_model = RestModelTransforms(bones);
			tails.resize(chains.size());
			for (int c = 0, cend = static_cast<int>(chains.size()); c != cend; ++c)
			{
				if (chains[c].role == EChainRole::Tail)
					tails[c] = TailJoints(bones, rest_model, chains[c]);

				solve_order.push_back(c);
			}

			// A chain's base bone comes after every ancestor's bones, so ordering by base bone solves ancestors first
			std::sort(solve_order.begin(), solve_order.end(), [&](int l, int r) { return chains[l].bones.front() < chains[r].bones.front(); });
		}

		// Return the index of the bone called 'name', or -1 if there is none
		int BoneIndex(std::string_view name) const
		{
			// Linear search; rigs are small and this is not for per-frame use
			for (int i = 0, iend = static_cast<int>(bones.size()); i != iend; ++i)
			{
				if (bones[i].name == name)
					return i;
			}
			return -1;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	// A small quadruped for testing. It faces -Y with +Z up, the spine is horizontal at z = 1, legs hang down, and the tail points along +Y.
	// Chains: 0 spine, 1 neck, 2 left leg, 3 left arm, 4 tail.
	inline RigDesc TestRigDesc()
	{
		// Bones are given by model-space head and tip positions, then converted to parent-relative transforms
		struct Def { char const* name; int parent; v4 head; v4 tip; };
		Def const defs[] = {
			{ "root",     -1, v4(0.0f,  0.0f, 0.0f, 1), v4(0.0f, -0.1f, 0.0f, 1) }, // 0
			{ "hips",      0, v4(0.0f,  0.0f, 1.0f, 1), v4(0.0f, -0.3f, 1.0f, 1) }, // 1
			{ "spine",     1, v4(0.0f, -0.3f, 1.0f, 1), v4(0.0f, -0.6f, 1.0f, 1) }, // 2
			{ "chest",     2, v4(0.0f, -0.6f, 1.0f, 1), v4(0.0f, -0.9f, 1.0f, 1) }, // 3
			{ "neck",      3, v4(0.0f, -0.9f, 1.0f, 1), v4(0.0f, -1.1f, 1.2f, 1) }, // 4
			{ "head",      4, v4(0.0f, -1.1f, 1.2f, 1), v4(0.0f, -1.3f, 1.2f, 1) }, // 5
			{ "thigh.L",   1, v4(0.2f,  0.0f, 1.0f, 1), v4(0.2f, -0.1f, 0.5f, 1) }, // 6
			{ "shin.L",    6, v4(0.2f, -0.1f, 0.5f, 1), v4(0.2f,  0.0f, 0.0f, 1) }, // 7
			{ "foot.L",    7, v4(0.2f,  0.0f, 0.0f, 1), v4(0.2f, -0.2f, 0.0f, 1) }, // 8
			{ "arm.L",     3, v4(0.2f, -0.9f, 1.0f, 1), v4(0.2f, -0.8f, 0.5f, 1) }, // 9
			{ "forearm.L", 9, v4(0.2f, -0.8f, 0.5f, 1), v4(0.2f, -0.9f, 0.0f, 1) }, // 10
			{ "hand.L",   10, v4(0.2f, -0.9f, 0.0f, 1), v4(0.2f, -1.0f, 0.0f, 1) }, // 11
			{ "tail.0",    1, v4(0.0f,  0.0f, 1.0f, 1), v4(0.0f,  0.3f, 1.0f, 1) }, // 12
			{ "tail.1",   12, v4(0.0f,  0.3f, 1.0f, 1), v4(0.0f,  0.6f, 1.0f, 1) }, // 13
			{ "tail.2",   13, v4(0.0f,  0.6f, 1.0f, 1), v4(0.0f,  0.9f, 1.0f, 1) }, // 14
		};

		// Each bone's local +Y points from its head to its tip
		std::vector<Bone> bones;
		std::vector<xform> model;
		for (auto const& def : defs)
		{
			auto dir = def.tip - def.head;
			auto m = xform(def.head, quat(v4::YAxis(), Normalise(dir)));
			auto rest = def.parent < 0 ? m : Invert(model[def.parent]) * m;
			bones.push_back(Bone{ .name = def.name, .parent = def.parent, .rest = rest, .length = Length(dir) });
			model.push_back(m);
		}

		// Knees bend forward (-Y), elbows bend backward (+Y), and the head faces -Y
		std::vector<Chain> chains = {
			Chain{ .role = EChainRole::Spine, .bones = { 1, 2, 3 } },
			Chain{ .role = EChainRole::Neck, .bones = { 4, 5 }, .forward = v4(0, -1, 0, 0) },
			Chain{ .role = EChainRole::Leg, .side = ESide::Left, .bones = { 6, 7, 8 }, .pole = v4(0, -1, 0, 0) },
			Chain{ .role = EChainRole::Arm, .side = ESide::Left, .bones = { 9, 10, 11 }, .pole = v4(0, 1, 0, 0) },
			Chain{ .role = EChainRole::Tail, .bones = { 12, 13, 14 } },
		};

		// The foot's sole is under its head, facing down, and the hand's palm likewise
		std::vector<Contact> contacts = {
			Contact{ .name = "sole.L", .bone = 8, .offset = v4(0, 0.1f, 0, 1), .normal = v4(0, 0, 1, 0), .kind = EContactKind::Sole },
			Contact{ .name = "palm.L", .bone = 11, .offset = v4(0, 0.05f, 0, 1), .normal = v4(0, 0, 1, 0), .kind = EContactKind::Palm },
		};
		// The body axis runs from the hips to the neck
		return RigDesc{ .bones = std::move(bones), .chains = std::move(chains), .contacts = std::move(contacts), .body_axis = { .from = 1, .to = 4 } };
	}
	inline Rig TestRig()
	{
		return Rig(TestRigDesc());
	}

	PRUnitTestClass(AnimationRigTests)
	{
		PRUnitTestMethod(RestTransforms, Quick)
		{
			// The cached rest transforms put each bone's head and tip where they were defined
			auto rig = TestRig();
			auto shin = rig.BoneIndex("shin.L");
			PR_EXPECT(FEqlAbsolute(rig.rest_model[shin].pos, v4(0.2f, -0.1f, 0.5f, 1), 1e-5f));
			PR_EXPECT(FEqlAbsolute(rig.rest_model[shin] * v4(0, rig.bones[shin].length, 0, 1), v4(0.2f, 0.0f, 0.0f, 1), 1e-5f));
		}
		PRUnitTestMethod(SolveOrder, Quick)
		{
			// The spine is solved before the chains attached to it
			auto rig = TestRig();
			PR_EXPECT(rig.solve_order.front() == 0);
			PR_EXPECT(rig.tails[4].size() == 4);
			PR_EXPECT(rig.tails[0].empty());
		}
		PRUnitTestMethod(ValidationErrors, Quick)
		{
			// Each kind of description error is reported
			auto const desc = TestRigDesc();
			auto expect_error = [&](auto&& modify)
			{
				// Apply one error to a copy of the valid description
				auto bad = desc;
				modify(bad);
				PR_EXPECT(!Validate(bad).empty());
			};
			expect_error([](RigDesc& d) { d.bones[3].parent = 5; });
			expect_error([](RigDesc& d) { d.chains[2].bones.pop_back(); });
			expect_error([](RigDesc& d) { d.chains[2].pole = v4(0, 0, 1, 0); });
			expect_error([](RigDesc& d) { d.chains[4].bones = { 3, 4 }; });
			expect_error([](RigDesc& d) { d.bones[4].limit = JointLimit{ .swing = { .min_x = 0.1f } }; });
			expect_error([](RigDesc& d) { d.bones[4].limit = JointLimit{ .min_twist = -1, .max_twist = -0.5f }; });
			expect_error([](RigDesc& d) { d.contacts[1].name = "sole.L"; });
			expect_error([](RigDesc& d) { d.contacts[0].bone = 99; });
			expect_error([](RigDesc& d) { d.contacts[0].normal = v4::Zero(); });
			expect_error([](RigDesc& d) { d.body_axis.to = 99; });
			expect_error([](RigDesc& d) { d.body_axis.to = 12; });
			PR_EXPECT(Validate(desc).empty());
		}
		PRUnitTestMethod(InvalidRigThrows, Quick)
		{
			// Constructing a rig from an invalid description throws
			auto desc = TestRigDesc();
			desc.chains[1].forward = v4::Zero();
			PR_THROWS((void)Rig(desc), std::invalid_argument);
		}
		PRUnitTestMethod(ReorderBones, Quick)
		{
			// Reversing the tail bones' order puts children before parents, which validation reports, and the round trip restores the description
			auto const desc = TestRigDesc();
			std::vector<std::string> order;
			for (auto const& bone : desc.bones)
				order.push_back(bone.name);

			std::swap(order[12], order[14]);
			auto reordered = ReorderBones(desc, order);
			PR_EXPECT(reordered.bones[14].name == "tail.0");
			PR_EXPECT(reordered.chains[4].bones == std::vector<int>({ 14, 13, 12 }));
			PR_EXPECT(!Validate(reordered).empty());

			std::swap(order[12], order[14]);
			auto restored = ReorderBones(reordered, std::vector<std::string>{ order.begin(), order.end() });
			PR_EXPECT(restored.bones[12].name == "tail.0" && restored.bones[13].parent == 12);
			PR_EXPECT(restored.contacts[0].bone == 8);
			PR_EXPECT(restored.body_axis.from == 1 && restored.body_axis.to == 4);
			PR_EXPECT(Validate(restored).empty());

			order.back() = "missing";
			PR_THROWS((void)ReorderBones(desc, order), std::invalid_argument);
		}
	};
}
#endif