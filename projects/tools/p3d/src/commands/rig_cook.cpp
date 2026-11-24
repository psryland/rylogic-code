//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2026
//**********************************************

#include "src/forward.h"
#include "src/commands/rig_cook.h"

using namespace pr::animation;

void CookRig(p3d::File const& p3d, std::filesystem::path const& infile, std::filesystem::path const& outfile, int verbosity)
{
	// The rig is cooked against the model's only skeleton, so a model with several skeletons is ambiguous
	auto const& skeletons = p3d.m_scene.m_skeletons;
	if (skeletons.size() != 1)
		throw std::runtime_error(std::format("Rig '{}': the model must contain exactly one skeleton (it has {})", infile.string(), skeletons.size()));

	auto const& skel = skeletons[0];
	if (verbosity >= 1)
		std::cout << "Cooking rig '" << infile.string() << "' against skeleton '" << skel.m_name << "'." << std::endl;

	// Read and validate the rig on its own before comparing it with the skeleton
	auto desc = LoadRigDesc(infile);
	std::vector<std::string> errors = Validate(desc);
	auto fail = [&]
	{
		// Report every problem at once so the rig can be fixed in one pass
		std::string msg = std::format("Rig '{}' cannot be cooked against skeleton '{}':", infile.string(), skel.m_name);
		for (auto const& error : errors)
			msg.append("\n  ").append(error);

		throw std::runtime_error(msg);
	};
	if (!errors.empty())
		fail();

	// Skeleton bones are depth-first, so each bone's parent is the nearest earlier bone with a smaller level
	auto bone_count = static_cast<int>(skel.m_bone_names.size());
	std::vector<int> skel_parent(bone_count, -1);
	for (int i = 0; i != bone_count; ++i)
	{
		for (int p = i - 1; p >= 0 && skel_parent[i] < 0; --p)
		{
			if (skel.m_hierarchy[p] < skel.m_hierarchy[i])
				skel_parent[i] = p;
		}
	}

	// The rig must describe exactly the skeleton's bones, because the runtime indexes both with the same bone index
	auto rig_index = [&](std::string const& name)
	{
		// Return the rig index of a bone name, or -1 if the rig does not have it
		auto it = std::find_if(desc.bones.begin(), desc.bones.end(), [&](Bone const& b) { return b.name == name; });
		return it == desc.bones.end() ? -1 : static_cast<int>(it - desc.bones.begin());
	};
	for (int i = 0; i != bone_count; ++i)
	{
		if (rig_index(skel.m_bone_names[i]) < 0)
			errors.push_back(std::format("Skeleton bone '{}' is not in the rig", skel.m_bone_names[i]));
	}
	for (auto const& bone : desc.bones)
	{
		if (std::find(skel.m_bone_names.begin(), skel.m_bone_names.end(), bone.name) == skel.m_bone_names.end())
			errors.push_back(std::format("Rig bone '{}' is not in the skeleton", bone.name));
	}
	if (!errors.empty())
		fail();

	// Put the rig in skeleton order, then compare hierarchies and rest poses bone by bone
	desc = ReorderBones(desc, skel.m_bone_names);
	auto rest_model = RestModelTransforms(desc.bones);

	// Positions are compared relative to the skeleton's size so the check does not depend on the model's units
	auto extent = 1.0f;
	for (int i = 0; i != bone_count; ++i)
		extent = std::max(extent, Length(Invert(skel.m_o2bp[i]).pos.xyz));

	auto pos_tol = 1e-4f * extent;
	auto rot_tol = 1e-3f; // radians
	for (int i = 0; i != bone_count; ++i)
	{
		// The rig's parent must be the skeleton's parent
		auto const& bone = desc.bones[i];
		auto skel_parent_name = skel_parent[i] < 0 ? std::string("(none)") : skel.m_bone_names[skel_parent[i]];
		auto rig_parent_name = bone.parent < 0 ? std::string("(none)") : desc.bones[bone.parent].name;
		if (rig_parent_name != skel_parent_name)
			errors.push_back(std::format("Bone '{}': the rig parent is '{}' but the skeleton parent is '{}'", bone.name, rig_parent_name, skel_parent_name));

		// The rig's model-space rest transform must be the skeleton's bind pose (the inverse of 'o2bp').
		// The rotation error is the angle of the relative rotation. 'atan2' keeps small angles accurate (where 'acos' of a dot product near 1 does not),
		// and using '|w|' ignores the sign ambiguity of 'q' and '-q'.
		auto bp2o = Invert(skel.m_o2bp[i]);
		auto rel = rest_model[i].rot * ~quat(bp2o.rot);
		auto pos_err = Length(rest_model[i].pos.xyz - bp2o.pos.xyz);
		auto rot_err = 2.0f * std::atan2(Length(rel.xyzw.xyz), std::abs(rel.w));
		if (pos_err > pos_tol)
			errors.push_back(std::format("Bone '{}': the rig rest position differs from the skeleton bind pose by {}", bone.name, pos_err));
		if (rot_err > rot_tol)
			errors.push_back(std::format("Bone '{}': the rig rest rotation differs from the skeleton bind pose by {} radians", bone.name, rot_err));
	}
	if (!errors.empty())
		fail();

	// Write the cooked rig
	SaveRigDesc(outfile, desc);
	if (verbosity >= 1)
		std::cout << "Cooked rig written to '" << outfile.string() << "' (" << bone_count << " bones, " << desc.chains.size() << " chains, " << desc.contacts.size() << " contacts, " << desc.attachments.size() << " attachments)." << std::endl;
}
