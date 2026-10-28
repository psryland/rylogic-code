//*****************************************************************************
// Procedural animation - aim chain
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/pose.h"

namespace pr::animation
{
	// Turn a Neck chain so that its last bone faces 'target' (a model-space point, w = 1).
	//  - The turn is split into a yaw about the model's up axis (+Z) and a change of elevation. So a target behind the head turns it sideways towards
	//    the target and never pitches it over or under. When the limits stop the yaw short, the head still matches the target's elevation.
	//  - The turn is shared out along the chain: each bone, from the base, takes an equal share of the turn that is still needed.
	//    So a neck curves smoothly instead of bending at one joint.
	//  - Each bone's limit caps its swing and twist from its rest orientation relative to its parent (see 'JointLimit'). If a limit is hit, later bones
	//    take more of the turn, and the last bone may not reach the target.
	//  - The chain's bones keep their parent-relative positions. Only their rotations change, starting from the current pose.
	//  - The model transforms of the chain's ancestors must be current. The chain's descendants are not updated.
	inline void AimChain(Rig const& rig, Chain const& chain, Pose& pose, v4 target)
	{
		pr_assert(chain.role == EChainRole::Neck && "Not a neck chain");
		auto const& bones = chain.bones;
		auto count = static_cast<int>(bones.size());
		auto last = bones.back();

		auto refresh = [&](int b)
		{
			// Compose the bone's local transform with its parent's model transform
			auto parent = rig.bones[b].parent;
			pose.model[b] = parent < 0 ? pose.local[b] : pose.model[parent] * pose.local[b];
		};

		// Make the chain's model transforms current from its parent
		for (auto b : bones)
			refresh(b);


		// Turn each bone about its head by its share of the remaining turn
		for (int i = 0; i != count; ++i)
		{
			// The remaining turn rotates the last bone's facing direction onto the direction from its head to the target. Measure it as the yaw between
			// the two directions seen from above, and the change of elevation between them. The shortest arc between the directions is not used
			// because, for a target behind the head, it swings the head over the top or down under the body.
			auto face = Rotate(pose.model[last].rot * ~rig.rest_model[last].rot, chain.forward);
			auto want = Normalise(target - pose.model[last].pos, face);
			auto yaw = std::atan2(face.x * want.y - face.y * want.x, face.x * want.x + face.y * want.y);
			auto rise = std::asin(Clamp(want.z, -1.0f, 1.0f)) - std::asin(Clamp(face.z, -1.0f, 1.0f));
			auto side = Normalise(Cross(face, v4::ZAxis()), v4::XAxis());

			// This bone's share is the same fraction of the yaw and of the elevation change
			auto fraction = 1.0f / (count - i);
			auto yaw_share = yaw * fraction;
			auto rise_share = rise * fraction;

			// Apply the share in model space, as a rotation from the bone's rest orientation in bone space. The elevation change turns about the
			// facing's horizontal side axis after the yaw, so it raises or lowers the head whatever part of the yaw is applied.
			auto b = bones[i];
			auto const& bone = rig.bones[b];
			auto parent_rot = bone.parent < 0 ? Identity<quat>() : pose.model[bone.parent].rot;
			auto from_rest = [&](float yaw_angle)
			{
				auto turn = quat(v4::ZAxis(), yaw_angle);
				return ~bone.rest.rot * ~parent_rot * quat(Rotate(turn, side), rise_share) * turn * pose.model[b].rot;
			};
			auto within = [&](quat rot)
			{
				return FEqlOrientation(bone.limit.Clamp(rot), rot, 1e-5f);
			};

			// A limit stops the yaw short rather than clamping it, because clamping the swing and twist separately turns a clipped yaw into a pitch or
			// a roll. Find the largest part of the yaw share that the limit allows by bisection, then clamp anything that is still outside the limit.
			auto rot = from_rest(yaw_share);
			if (!bone.limit.IsFree())
			{
				// Search between no yaw and the full yaw share
				if (!within(rot))
				{
					auto lo = 0.0f, hi = 1.0f;
					for (int k = 0; k != 16; ++k)
					{
						auto mid = 0.5f * (lo + hi);
						(within(from_rest(yaw_share * mid)) ? lo : hi) = mid;
					}
					rot = from_rest(yaw_share * lo);
				}
				rot = bone.limit.Clamp(rot);
			}

			pose.local[b].rot = Normalise(bone.rest.rot * rot);

			// Recompute the model transforms of this bone and the rest of the chain
			for (int j = i; j != count; ++j)
				refresh(bones[j]);
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	PRUnitTestClass(AimChainTests)
	{
		PRUnitTestMethod(RestTarget, Quick)
		{
			// A target straight ahead of the head leaves the rest pose unchanged
			auto rig = TestRig();
			auto pose = Pose(rig);
			auto const& neck = rig.chains[1];
			auto head = rig.rest_model[neck.bones.back()].pos;
			AimChain(rig, neck, pose, head + neck.forward * 2.0f);
			for (auto b : neck.bones)
				PR_EXPECT(FEqlAbsolute(pose.model[b].pos, rig.rest_model[b].pos, 1e-5f) && FEqlOrientation(pose.model[b].rot, rig.rest_model[b].rot, 1e-4f));
		}
		PRUnitTestMethod(FacesTarget, Quick)
		{
			// The head faces a target to the side, and both neck bones share the turn
			auto rig = TestRig();
			auto pose = Pose(rig);
			auto const& neck = rig.chains[1];
			auto target = v4(1.0f, -1.5f, 1.4f, 1);
			AimChain(rig, neck, pose, target);

			auto head = neck.bones.back();
			auto face = Rotate(pose.model[head].rot * ~rig.rest_model[head].rot, neck.forward);
			PR_EXPECT(FEqlAbsolute(face, Normalise(target - pose.model[head].pos), 1e-4f));

			auto turn0 = AxisAngle(pose.local[neck.bones[0]].rot * ~rig.bones[neck.bones[0]].rest.rot).angle;
			auto turn1 = AxisAngle(pose.local[head].rot * ~rig.bones[head].rest.rot).angle;
			PR_EXPECT(turn0 > 0.1f && turn1 > 0.1f);
		}
		PRUnitTestMethod(SwingTwistLimit, Quick)
		{
			// A swing-twist limit caps each bone's swing and twist separately
			auto desc = TestRigDesc();
			auto limit = JointLimit{ .swing = { .min_x = -0.1f, .max_x = 0.2f, .min_z = -0.15f, .max_z = 0.15f }, .min_twist = -0.05f, .max_twist = 0.05f };
			for (auto b : desc.chains[1].bones)
				desc.bones[b].limit = limit;

			auto limited = Rig(std::move(desc));
			auto pose = Pose(limited);
			auto const& neck = limited.chains[1];
			AimChain(limited, neck, pose, v4(3.0f, -2.0f, 1.0f, 1));
			for (auto b : neck.bones)
			{
				// The final rotation from rest must be within the limit
				auto st = JointLimit::Decompose(~limited.bones[b].rest.rot * pose.local[b].rot);
				auto rx = st.swing.x >= 0 ? limit.swing.max_x : -limit.swing.min_x;
				auto rz = st.swing.y >= 0 ? limit.swing.max_z : -limit.swing.min_z;
				PR_EXPECT(Sqr(st.swing.x / rx) + Sqr(st.swing.y / rz) <= 1.0f + 1e-3f);
				PR_EXPECT(st.twist >= limit.min_twist - 1e-4f && st.twist <= limit.max_twist + 1e-4f);
			}
		}
		PRUnitTestMethod(TargetBehindTurnsSideways, Quick)
		{
			// A target behind and a little below a limited neck turns the head sideways, at the target's elevation, instead of pitching it over or under
			auto desc = TestRigDesc();
			auto limit = JointLimit{ .swing = { .min_x = -0.4f, .max_x = 0.4f, .min_z = -0.4f, .max_z = 0.4f }, .min_twist = -0.4f, .max_twist = 0.4f };
			for (auto b : desc.chains[1].bones)
				desc.bones[b].limit = limit;

			auto limited = Rig(std::move(desc));
			auto pose = Pose(limited);
			auto const& neck = limited.chains[1];
			auto head = neck.bones.back();
			auto target = limited.rest_model[head].pos - neck.forward * 5.0f + v4(0.3f, 0, -0.5f, 0);
			AimChain(limited, neck, pose, target);

			// The head matches the target's elevation and turns as far as the limits allow towards the target's side
			auto face = Rotate(pose.model[head].rot * ~limited.rest_model[head].rot, neck.forward);
			auto want = Normalise(target - pose.model[head].pos);
			PR_EXPECT(FEqlAbsolute(face.z, want.z, 0.01f));
			PR_EXPECT(face.x > 0.3f);
		}
		PRUnitTestMethod(SwingTwistRoundTrip, Quick)
		{
			// Decompose and Compose are inverses, and the swing never has a component about +Y
			auto rot = Normalise(quat(v4::XAxis(), 0.7f) * quat(v4::ZAxis(), -0.4f) * quat(v4::YAxis(), 1.1f));
			auto st = JointLimit::Decompose(rot);
			PR_EXPECT(FEqlOrientation(JointLimit::Compose(st), rot, 1e-4f));

			auto twist_only = JointLimit::Decompose(quat(v4::YAxis(), -0.6f));
			PR_EXPECT(FEqlAbsolute(twist_only.twist, -0.6f, 1e-5f) && FEqlAbsolute(Length(twist_only.swing), 0.0f, 1e-5f));

			auto swing_only = JointLimit::Decompose(quat(v4::XAxis(), 0.5f));
			PR_EXPECT(FEqlAbsolute(swing_only.swing, v2(0.5f, 0.0f), 1e-5f) && FEqlAbsolute(swing_only.twist, 0.0f, 1e-5f));
		}
	};
}
#endif
