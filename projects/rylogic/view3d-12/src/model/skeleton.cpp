//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/skeleton.h"

namespace pr::rdr12
{
	Skeleton::Skeleton(uint32_t id, std::span<uint32_t const> bone_ids, std::span<string32 const> names, std::span<m4x4 const> o2bp, std::span<uint8_t const> hierarchy)
		: m_id(id)
		, m_bone_ids(bone_ids)
		, m_names(names)
		, m_o2bp(o2bp)
		, m_hierarchy(hierarchy)
	{
	}

	// The ID of the root bone
	uint32_t Skeleton::Id() const
	{
		return m_id;
	}

	// The number of bones in this skeleton
	int Skeleton::BoneCount() const
	{
		return isize(m_bone_ids);
	}

	// Convert bone-to-parent transforms to object-space skinning matrices in place.
	void Skeleton::SkinningMatrices(std::span<m4x4> pose) const
	{
		// Each bone's deformed bone-to-object transform is its parent's times its own bone-to-parent. The skinning matrix takes object space verts
		// to bind-pose bone-relative space, then from deformed bone space back to object space: 'cp2o * bp2cp * o2bp'.
		assert(isize(pose) == BoneCount() && "Pose buffer should have one matrix per bone");
		WalkHierarchy<m4x4>([ptr = pose.data(), &o2bp = m_o2bp](int idx, m4x4 const* p2o) -> m4x4
		{
			// Return 'b2o' as the parent of any child bones
			auto b2o = p2o ? *p2o * ptr[idx] : ptr[idx];
			ptr[idx] = b2o * o2bp[idx];
			return b2o;
		});
	}

	// Check if this skeleton is structurally compatible with another (same bone count and names)
	bool Skeleton::IsCompatible(Skeleton const& other) const
	{
		if (BoneCount() != other.BoneCount())
			return false;

		for (int i = 0, iend = BoneCount(); i != iend; ++i)
		{
			if (m_names[i] != other.m_names[i])
				return false;
		}
		return true;
	}

	// Ref-counting clean up function
	void Skeleton::RefCountZero(RefCounted<Skeleton>* doomed)
	{
		auto skel = static_cast<Skeleton*>(doomed);
		::pr::compute::Delete(skel);
	}
}
