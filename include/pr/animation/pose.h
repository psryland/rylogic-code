//*****************************************************************************
// Procedural animation - pose
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"

namespace pr::animation
{
	// The transforms of every bone in a rig.
	//  - 'local' is the source of truth: each bone's transform relative to its parent. Bones that no solver poses keep their local transform.
	//  - 'model' caches the model-space transforms. 'UpdateModel' recomputes it from 'local'.
	//  - Solvers write model-space results with 'SetModel', which needs the parent's model transform to be current.
	struct Pose
	{
		std::vector<xform> local;
		std::vector<xform> model;

		// Construct in the rig's rest pose
		explicit Pose(Rig const& rig)
			: local()
			, model(rig.rest_model)
		{
			// The rest pose's local transforms come from the rig description
			local.reserve(rig.bones.size());
			for (auto const& bone : rig.bones)
				local.push_back(bone.rest);
		}

		// Recompute the model-space transforms from the local transforms
		void UpdateModel(Rig const& rig)
		{
			// Parents come before children, so one pass composes the whole hierarchy
			for (int i = 0, iend = static_cast<int>(local.size()); i != iend; ++i)
			{
				auto parent = rig.bones[i].parent;
				model[i] = parent < 0 ? local[i] : model[parent] * local[i];
			}
		}

		// Recompute the model-space transforms of 'bones' from their local transforms. 'bones' must be in bone order,
		// and the parent of each listed bone must either be listed or already have a current model transform.
		void UpdateModel(Rig const& rig, std::span<int const> bones)
		{
			// Listed parents are updated before their children because the list is in bone order
			for (auto i : bones)
			{
				auto parent = rig.bones[i].parent;
				model[i] = parent < 0 ? local[i] : model[parent] * local[i];
			}
		}

		// Set the model-space transform of 'bone'. The parent's model transform must be current. Descendants are not updated.
		void SetModel(Rig const& rig, int bone, xform const& xf)
		{
			// Keep the local transform consistent so that a later 'UpdateModel' reproduces 'xf'
			auto parent = rig.bones[bone].parent;
			model[bone] = xf;
			local[bone] = parent < 0 ? xf : Invert(model[parent]) * xf;
		}

		// The model-space position of the tip of 'bone'
		v4 Tip(Rig const& rig, int bone) const
		{
			// Bones point along their local +Y axis
			return model[bone] * v4(0, rig.bones[bone].length, 0, 1);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	PRUnitTestClass(AnimationPoseTests)
	{
		PRUnitTestMethod(SetModelUpdatesLocal, Quick)
		{
			// A model transform set on a bone survives a recompute from local transforms, and moves its children
			auto rig = TestRig();
			auto pose = Pose(rig);
			auto shin = rig.BoneIndex("shin.L");
			auto foot = rig.BoneIndex("foot.L");

			auto xf = xform(v4(1, 2, 3, 1), quat(v4::XAxis(), 0.5f));
			pose.SetModel(rig, shin, xf);
			pose.UpdateModel(rig);
			PR_EXPECT(FEqlAbsolute(pose.model[shin], xf, 1e-5f));

			auto foot_rel = Invert(rig.rest_model[shin]) * rig.rest_model[foot];
			PR_EXPECT(FEqlAbsolute(pose.model[foot], xf * foot_rel, 1e-5f));
		}
		PRUnitTestMethod(Tip, Quick)
		{
			// The tip of the rest-pose shin is the foot's head
			auto rig = TestRig();
			auto pose = Pose(rig);
			PR_EXPECT(FEqlAbsolute(pose.Tip(rig, rig.BoneIndex("shin.L")), rig.rest_model[rig.BoneIndex("foot.L")].pos, 1e-5f));
		}
	};
}
#endif
