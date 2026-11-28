//*****************************************************************************
// Procedural animation - two-bone IK
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/pose.h"

namespace pr::animation
{
	// The solved joint positions of a two-bone chain
	struct TwoBonePositions
	{
		v4 mid;   // The joint between the two bones (knee, elbow)
		v4 end;   // The end of the second bone (ankle, wrist)
	};

	// Place the joints of a two-bone chain rooted at 'root' so that its end reaches 'target'.
	//  - 'len0' and 'len1' are the bone lengths (positive).
	//  - 'pole' is the direction the middle joint bends towards (w = 0). Only its part perpendicular to the root-to-target line matters.
	//  - A target closer than |len0 - len1| or further than len0 + len1 is out of reach. The end then stops at the nearest reachable point on
	//    the line from the root towards the target.
	inline TwoBonePositions SolveTwoBone(v4 root, float len0, float len1, v4 target, v4 pole)
	{
		// The reach direction, and the in-plane direction the middle joint bends towards.
		// Degenerate directions (target at the root, pole along the reach line) fall back to an arbitrary perpendicular.
		auto dir = Normalise(target - root, Normalise(Perpendicular(pole)));
		auto bend = Normalise(pole - dir * Dot3(pole, dir), Normalise(Perpendicular(dir)));

		// Clamp the reach, then find the angle at the root. The sine comes from the triangle's side factors (Heron's form) rather than
		// sqrt(1 - cos^2), so that a fully extended or fully folded limb is exactly straight instead of amplifying rounding error.
		auto dist = std::clamp(Length(target - root), std::abs(len0 - len1), len0 + len1);
		auto area16 = (len0 + len1 + dist) * (-len0 + len1 + dist) * (len0 - len1 + dist) * (len0 + len1 - dist);
		auto cos_a = dist > 0 ? std::clamp((len0 * len0 + dist * dist - len1 * len1) / (2 * len0 * dist), -1.0f, 1.0f) : 0.0f;
		auto sin_a = dist > 0 ? std::min(std::sqrt(std::max(area16, 0.0f)) / (2 * len0 * dist), 1.0f) : 1.0f;

		return TwoBonePositions{
			.mid = root + (dir * cos_a + bend * sin_a) * len0,
			.end = root + dir * dist,
		};
	}

	// Return the orientation whose X axis is 'x' and whose Z axis is 'normal' made perpendicular to 'x'. 'x' must be a unit direction.
	inline quat BendFrame(v4 x, v4 normal)
	{
		// Complete a right-handed orthonormal basis
		auto z = Normalise(normal - x * Dot3(normal, x), Normalise(Perpendicular(x)));
		auto y = Cross(z, x);
		return quat(math::Mat3x3<float>(x, y, z));
	}

	// Return the normal of the plane a limb bends in, given its root-to-end direction and pole
	inline v4 BendNormal(v4 reach, v4 pole)
	{
		// The pole is not parallel to the reach for valid rigs; fall back to any perpendicular otherwise
		return Normalise(Cross(reach, pole), Normalise(Perpendicular(reach)));
	}

	// Return how far 'rot', a rotation from a bone's rest orientation in bone space, is outside 'limit'. The result is the sum of the swing and twist
	// angles (radians) beyond their ranges, so it is zero when 'rot' is allowed.
	inline float LimitExcess(JointLimit const& limit, quat rot)
	{
		// Measure the swing and twist parts separately, as 'JointLimit::Clamp' limits them
		auto st = JointLimit::Decompose(rot);
		auto swing = Length(st.swing - limit.swing.Clamp(st.swing));
		auto twist = std::abs(st.twist - math::Clamp(st.twist, limit.min_twist, limit.max_twist));
		return swing + twist;
	}

	// Pose a Leg or Arm chain so that its end bone has the model-space frame 'end', with the middle joint bending towards 'pole' (model space, w = 0).
	//  - The model transforms of the chain's ancestors must be current. The chain's descendants are not updated.
	//  - If 'end' is out of reach, the end bone keeps the orientation of 'end' at the nearest reachable position (see 'SolveTwoBone').
	//  - The upper and lower bones turn from their rest orientations by the rotation that maps the rest bend plane onto the solved bend plane.
	//    The end bone's roll about the lower bone (for example, turning the hand about the forearm) is then shared along the limb: the lower bone
	//    rolls about its own axis by half of it and the upper bone by a quarter, so no single joint carries the whole twist. The joint positions
	//    and the end bone's frame are unchanged by this.
	//  - Arm chains honour their bones' joint limits (see 'JointLimit'). If the solve with 'pole' breaks a limit, the middle joint is swung around
	//    the root-to-target line to the position that breaks the limits least, preferring positions near 'pole'. Any rotation still outside a limit
	//    is then clamped, from the base of the chain down, so the end bone may stop short of 'end' or turn away from it.
	inline void SolveLimb(Rig const& rig, Chain const& chain, Pose& pose, xform const& end, v4 pole)
	{
		// The rest geometry gives the bone lengths and the rest bend plane
		pr_assert((chain.role == EChainRole::Leg || chain.role == EChainRole::Arm) && "Not a limb chain");
		auto b0 = chain.bones[0];
		auto b1 = chain.bones[1];
		auto b2 = chain.bones[2];
		auto r0 = rig.rest_model[b0].pos;
		auto r1 = rig.rest_model[b1].pos;
		auto r2 = rig.rest_model[b2].pos;
		auto rest_normal = BendNormal(r2 - r0, chain.pole);

		// The upper bone's head follows its parent
		auto parent = rig.bones[b0].parent;
		auto parent_rot = parent < 0 ? Identity<quat>() : pose.model[parent].rot;
		auto root = (parent < 0 ? pose.local[b0] : pose.model[parent] * pose.local[b0]).pos;

		// The joint positions and the model-space orientations of the upper and lower bones for one choice of pole
		struct Solution
		{
			TwoBonePositions joints;
			quat upper;
			quat lower;
		};
		auto solve = [&](v4 pole_dir)
		{
			// Rotate each bone by the change of its bend frame
			auto joints = SolveTwoBone(root, Length(r1 - r0), Length(r2 - r1), end.pos, pole_dir);
			auto normal = BendNormal(joints.end - root, pole_dir);
			auto upper = BendFrame(Normalise(joints.mid - root), normal) * ~BendFrame(Normalise(r1 - r0), rest_normal);
			auto lower = BendFrame(Normalise(joints.end - joints.mid), normal) * ~BendFrame(Normalise(r2 - r1), rest_normal);

			// Measure how far the end bone rolls about the lower bone's axis, relative to where the untwisted lower bone would carry it. The positive-w
			// form keeps the angle within [-pi, pi]. Rolling a bone about its own axis leaves its tip in place, so the joints do not move.
			auto lower_axis = Normalise(joints.end - joints.mid, Normalise(Perpendicular(normal)));
			auto upper_axis = Normalise(joints.mid - root, Normalise(Perpendicular(normal)));
			auto residual = end.rot * ~(lower * rig.rest_model[b2].rot);
			if (residual.w < 0)
				residual = quat(-residual.x, -residual.y, -residual.z, -residual.w);

			auto roll = 2.0f * std::atan2(Dot3(v4(residual.x, residual.y, residual.z, 0), lower_axis), residual.w);
			upper = quat(upper_axis, 0.25f * roll) * upper;
			lower = quat(lower_axis, 0.5f * roll) * lower;
			return Solution{
				.joints = joints,
				.upper = Normalise(upper * rig.rest_model[b0].rot),
				.lower = Normalise(lower * rig.rest_model[b1].rot),
			};
		};
		auto sol = solve(pole);

		// Without limits, or when the solve keeps within them, the end bone takes the frame 'end' exactly
		auto const& bone0 = rig.bones[b0];
		auto const& bone1 = rig.bones[b1];
		auto const& bone2 = rig.bones[b2];
		auto limited = chain.role == EChainRole::Arm && (!bone0.limit.IsFree() || !bone1.limit.IsFree() || !bone2.limit.IsFree());
		auto excess = [&](Solution const& s)
		{
			// Each bone's rotation from rest is measured relative to its parent's solved orientation
			return
				LimitExcess(bone0.limit, ~bone0.rest.rot * ~parent_rot * s.upper) +
				LimitExcess(bone1.limit, ~bone1.rest.rot * ~s.upper * s.lower) +
				LimitExcess(bone2.limit, ~bone2.rest.rot * ~s.lower * end.rot);
		};
		if (!limited || excess(sol) == 0)
		{
			// Write the results from the base of the chain down
			pose.SetModel(rig, b0, xform(root, sol.upper));
			pose.SetModel(rig, b1, xform(sol.joints.mid, sol.lower));
			pose.SetModel(rig, b2, xform(sol.joints.end, end.rot));
			return;
		}

		// Swinging the middle joint around the root-to-target line keeps the end on target, so search that circle for the pole that breaks the limits
		// least. A small cost per radian of swing prefers the requested pole among poles that are equally good. The search steps around the whole
		// circle, alternating either way from the requested pole, then refines around the best step by halving.
		constexpr int PoleSteps = 12;
		constexpr float PoleStep = math::constants<float>::tau_by_2 / PoleSteps;
		constexpr float SwingCost = 0.02f;
		auto reach = Normalise(end.pos - root, v4::Zero());
		if (LengthSq(reach) != 0)
		{
			// Coarse search, including a half turn
			auto cost = [&](float angle)
			{
				return excess(solve(Rotate(quat(reach, angle), pole))) + SwingCost * std::abs(angle);
			};
			auto best_angle = 0.0f;
			auto best_cost = excess(sol);
			for (int k = 0; k != 2 * PoleSteps; ++k)
			{
				// Angles +1, -1, +2, -2, ... steps, ending with +PoleSteps (a half turn)
				auto angle = PoleStep * static_cast<float>(k / 2 + 1) * (k % 2 == 0 ? 1.0f : -1.0f);
				if (auto c = cost(angle); c < best_cost)
				{
					best_angle = angle;
					best_cost = c;
				}
			}

			// Refine either side of the best step
			for (auto step = 0.5f * PoleStep; step > 0.005f; step *= 0.5f)
			{
				for (auto angle : { best_angle - step, best_angle + step })
				{
					if (auto c = cost(angle); c < best_cost)
					{
						best_angle = angle;
						best_cost = c;
					}
				}
			}
			sol = solve(Rotate(quat(reach, best_angle), pole));
		}

		// Clamp each bone's rotation from rest relative to its parent's clamped orientation, from the base of the chain down. Each joint is placed at
		// the tip of its clamped parent, so the chain stays connected.
		auto clamp_to = [&](Bone const& bone, quat parent_model, quat model)
		{
			auto from_rest = ~bone.rest.rot * ~parent_model * model;
			return Normalise(parent_model * bone.rest.rot * bone.limit.Clamp(from_rest));
		};
		auto upper = clamp_to(bone0, parent_rot, sol.upper);
		pose.SetModel(rig, b0, xform(root, upper));
		auto lower = clamp_to(bone1, upper, sol.lower);
		pose.SetModel(rig, b1, xform(pose.model[b0] * bone1.rest.pos, lower));
		pose.SetModel(rig, b2, xform(pose.model[b1] * bone2.rest.pos, clamp_to(bone2, lower, end.rot)));
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	PRUnitTestClass(TwoBoneIKTests)
	{
		PRUnitTestMethod(Reachable, Quick)
		{
			// The end reaches the target, the bones keep their lengths, and the middle joint is on the pole side
			auto root = v4(0, 0, 0, 1);
			auto target = v4(1.2f, 0.3f, -0.2f, 1);
			auto pole = v4(0, 1, 0, 0);
			auto j = SolveTwoBone(root, 1.0f, 0.8f, target, pole);
			PR_EXPECT(FEqlAbsolute(j.end, target, 1e-5f));
			PR_EXPECT(FEqlAbsolute(Length(j.mid - root), 1.0f, 1e-5f));
			PR_EXPECT(FEqlAbsolute(Length(j.end - j.mid), 0.8f, 1e-5f));

			auto dir = Normalise(target - root);
			auto offset = j.mid - root;
			PR_EXPECT(Dot3(offset - dir * Dot3(offset, dir), pole) > 0);
		}
		PRUnitTestMethod(Unreachable, Quick)
		{
			// Too far: the limb is straight and points at the target. Too near: the end stops at the minimum reach.
			auto root = v4(0, 0, 0, 1);
			auto too_far = SolveTwoBone(root, 1.0f, 0.8f, v4(5, 0, 0, 1), v4(0, 1, 0, 0));
			PR_EXPECT(FEqlAbsolute(too_far.end, v4(1.8f, 0, 0, 1), 1e-5f));
			PR_EXPECT(FEqlAbsolute(too_far.mid, v4(1.0f, 0, 0, 1), 1e-5f));

			auto too_near = SolveTwoBone(root, 1.0f, 0.8f, v4(0.1f, 0, 0, 1), v4(0, 1, 0, 0));
			PR_EXPECT(FEqlAbsolute(too_near.end, v4(0.2f, 0, 0, 1), 1e-5f));
		}
		PRUnitTestMethod(LimbAtRest, Quick)
		{
			// Targeting the rest foot frame with the rest pole reproduces the rest pose
			auto rig = TestRig();
			auto pose = Pose(rig);
			auto const& leg = rig.chains[2];
			SolveLimb(rig, leg, pose, rig.rest_model[leg.bones[2]], leg.pole);
			for (auto b : leg.bones)
				PR_EXPECT(FEqlAbsolute(pose.model[b], rig.rest_model[b], 1e-4f));
		}
		PRUnitTestMethod(LimbReachesTarget, Quick)
		{
			// The foot reaches a new frame, the knee stays forward, and the shin's tip meets the foot
			auto rig = TestRig();
			auto pose = Pose(rig);
			auto const& leg = rig.chains[2];
			auto foot_frame = xform(v4(0.3f, -0.3f, 0.2f, 1), quat(v4::ZAxis(), 0.3f) * rig.rest_model[leg.bones[2]].rot);
			SolveLimb(rig, leg, pose, foot_frame, leg.pole);
			pose.UpdateModel(rig);

			PR_EXPECT(FEqlAbsolute(pose.model[leg.bones[2]], foot_frame, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.Tip(rig, leg.bones[0]), pose.model[leg.bones[1]].pos, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.Tip(rig, leg.bones[1]), pose.model[leg.bones[2]].pos, 1e-4f));
			PR_EXPECT(pose.model[leg.bones[1]].pos.y < Lerp(pose.model[leg.bones[0]].pos, foot_frame.pos, 0.5f).y);
		}
		PRUnitTestMethod(LimbSharesEndRoll, Quick)
		{
			// Rolling the end bone about the lower bone rolls the lower bone by half as much and the upper bone by a quarter. The end frame and
			// the joint positions are exact.
			auto rig = TestRig();
			auto const& leg = rig.chains[2];
			auto b0 = leg.bones[0];
			auto b1 = leg.bones[1];
			auto b2 = leg.bones[2];
			auto lower_axis = Normalise(rig.rest_model[b2].pos - rig.rest_model[b1].pos);
			auto upper_axis = Normalise(rig.rest_model[b1].pos - rig.rest_model[b0].pos);
			auto roll = 0.8f;

			auto pose = Pose(rig);
			auto foot_frame = xform(rig.rest_model[b2].pos, quat(lower_axis, roll) * rig.rest_model[b2].rot);
			SolveLimb(rig, leg, pose, foot_frame, leg.pole);
			pose.UpdateModel(rig);

			PR_EXPECT(FEqlAbsolute(pose.model[b2], foot_frame, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.model[b1].rot, quat(lower_axis, 0.5f * roll) * rig.rest_model[b1].rot, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.model[b0].rot, quat(upper_axis, 0.25f * roll) * rig.rest_model[b0].rot, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.model[b1].pos, rig.rest_model[b1].pos, 1e-4f));
		}
		PRUnitTestMethod(ArmSwingsElbowToHonourLimits, Quick)
		{
			// The forearm can only bend a little from rest, so a pole on the wrong side would break its limit. The elbow swings back to the allowed side
			// and the hand still reaches its target exactly.
			auto desc = TestRigDesc();
			auto const& arm_desc = desc.chains[3];
			desc.bones[arm_desc.bones[1]].limit = JointLimit{ .swing = { .min_x = -0.3f, .max_x = 0.3f, .min_z = 0, .max_z = 0 }, .min_twist = 0, .max_twist = 0 };
			auto rig = Rig(std::move(desc));
			auto const& arm = rig.chains[3];
			auto hand = rig.rest_model[arm.bones[2]];

			auto pose = Pose(rig);
			SolveLimb(rig, arm, pose, hand, -arm.pole);
			pose.UpdateModel(rig);
			PR_EXPECT(FEqlAbsolute(pose.model[arm.bones[2]], hand, 1e-3f));
			PR_EXPECT(FEqlAbsolute(pose.model[arm.bones[1]].pos, rig.rest_model[arm.bones[1]].pos, 1e-3f));
		}
		PRUnitTestMethod(ArmClampsToLimits, Quick)
		{
			// A target the limits cannot reach leaves every arm bone within its limit and the chain connected
			auto desc = TestRigDesc();
			auto limit = JointLimit{ .swing = { .min_x = -0.2f, .max_x = 0.2f, .min_z = -0.2f, .max_z = 0.2f }, .min_twist = -0.2f, .max_twist = 0.2f };
			for (auto b : desc.chains[3].bones)
				desc.bones[b].limit = limit;

			auto rig = Rig(std::move(desc));
			auto const& arm = rig.chains[3];
			auto pose = Pose(rig);
			auto target = xform(v4(0.6f, -1.4f, 0.9f, 1), quat(v4::XAxis(), 1.5f) * rig.rest_model[arm.bones[2]].rot);
			SolveLimb(rig, arm, pose, target, arm.pole);
			pose.UpdateModel(rig);
			for (auto b : arm.bones)
				PR_EXPECT(LimitExcess(limit, ~rig.bones[b].rest.rot * pose.local[b].rot) < 1e-4f);

			PR_EXPECT(FEqlAbsolute(pose.Tip(rig, arm.bones[0]), pose.model[arm.bones[1]].pos, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.Tip(rig, arm.bones[1]), pose.model[arm.bones[2]].pos, 1e-4f));
		}
	};
}
#endif
