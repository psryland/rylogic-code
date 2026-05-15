//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	struct Skeleton: RefCounted<Skeleton>
	{
		// See description in "animation.h"
		using Ids = pr::vector<uint32_t>;
		using Bones = pr::vector<m4x4>;
		using Names = pr::vector<string32>;
		using Hierarchy = pr::vector<uint8_t>;

		uint32_t m_id;         // A unique ID for the skeleton
		Ids m_bone_ids;        // A unique ID for each bone
		Names m_names;         // A name for each bone (debugging mostly)
		Bones m_o2bp;          // The inverse of the bind-pose to object-space transform for each bone
		Hierarchy m_hierarchy; // Depth-first ordered list of bones. First == root == 0.

		Skeleton(uint32_t id, std::span<uint32_t const> bone_ids, std::span<string32 const> names, std::span<m4x4 const> o2bp, std::span<uint8_t const> hierarchy);

		// The ID of the root bone
		uint32_t Id() const;

		// The number of bones in this skeleton
		int BoneCount() const;

		// Walk the skeleton hierarchy calling 'func' for each bone.
		// The caller decides what is pushed to the stack at each level (Ret).
		// 'Func' is Ret Func(int bone_index, Ret const* parent) {...}
		template <typename Ret, std::invocable<int, Ret const*> Func>
		requires (std::same_as<std::invoke_result_t<Func, int, Ret const*>, Ret>)
		void WalkHierarchy(Func func) const
		{
			::pr::compute::WalkHierarchy<Ret, std::span<uint8_t const>, Func>(m_hierarchy, func);
		}

		// Check if this skeleton is structurally compatible with another (same bone count and names)
		bool IsCompatible(Skeleton const& other) const;

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<Skeleton>* doomed);
	};
}
