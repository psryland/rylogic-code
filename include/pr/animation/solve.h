//*****************************************************************************
// Procedural animation - pose solve
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/pose.h"
#include "pr/animation/targets.h"
#include "pr/animation/two_bone_ik.h"
#include "pr/animation/spine_fit.h"
#include "pr/animation/aim_chain.h"

namespace pr::animation
{
	// Pose every Spine, Neck, Leg and Arm chain of 'rig' to meet 'targets' (one per chain, see 'Target').
	//  - Chains are solved base-first, so a limb follows the spine it hangs from.
	//  - Bones outside these chains keep their local transforms. Tail chains are posed afterwards by 'Locomotion'.
	//  - On return, all model transforms in 'pose' are current.
	inline void SolvePose(Rig const& rig, std::span<Target const> targets, Pose& pose)
	{
		// Start from current model transforms, then solve each chain after its ancestors
		pr_assert(targets.size() == rig.chains.size() && "One target per chain is required");
		pose.UpdateModel(rig);
		for (auto c : rig.solve_order)
		{
			// Each role has its own solver
			auto const& chain = rig.chains[c];
			auto const& target = targets[c];
			switch (chain.role)
			{
				case EChainRole::Spine:
				{
					FitSpine(rig, chain, pose, target.base, target.frame);
					break;
				}
				case EChainRole::Neck:
				{
					AimChain(rig, chain, pose, target.frame.pos);
					break;
				}
				case EChainRole::Leg:
				case EChainRole::Arm:
				{
					SolveLimb(rig, chain, pose, target.frame, target.pole);
					break;
				}
				case EChainRole::Tail:
				case EChainRole::Digit:
				{
					continue;
				}
				default:
				{
					throw std::invalid_argument("Unknown chain role");
				}
			}

			// Bring the chain's descendants along before solving them
			pose.UpdateModel(rig);
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	PRUnitTestClass(SolvePoseTests)
	{
		PRUnitTestMethod(RestTargetsGiveRestPose, Quick)
		{
			// Solving rest targets reproduces the rest pose
			auto rig = TestRig();
			auto pose = Pose(rig);
			auto targets = PoseTargets(rig, rig.rest_model);
			SolvePose(rig, targets, pose);
			for (int b = 0, bend = static_cast<int>(rig.bones.size()); b != bend; ++b)
				PR_EXPECT(FEqlAbsolute(pose.model[b].pos, rig.rest_model[b].pos, 1e-4f) && FEqlOrientation(pose.model[b].rot, rig.rest_model[b].rot, 1e-3f));
		}
		PRUnitTestMethod(PlantedFeetWithMovedBody, Quick)
		{
			// Lowering and leaning the body keeps the foot and hand planted, and the limbs follow the spine
			auto rig = TestRig();
			auto pose = Pose(rig);
			auto targets = PoseTargets(rig, rig.rest_model);
			auto lean = quat(v4::XAxis(), 0.2f);
			auto const& spine = rig.chains[0];
			targets[0].base = xform(v4(0, 0.0f, 0.9f, 1), lean * rig.rest_model[spine.bones.front()].rot);
			targets[0].frame = xform(v4(0, -0.58f, 1.0f, 1), lean * rig.rest_model[spine.bones.back()].rot);
			SolvePose(rig, targets, pose);

			auto foot = rig.chains[2].bones.back();
			auto hand = rig.chains[3].bones.back();
			PR_EXPECT(FEqlAbsolute(pose.model[foot].pos, rig.rest_model[foot].pos, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.model[hand].pos, rig.rest_model[hand].pos, 1e-4f));
			PR_EXPECT(FEqlAbsolute(pose.model[spine.bones.front()].pos, targets[0].base.pos, 1e-5f));
		}
	};
}
#endif
