//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/skeleton.h"
#include "pr/view3d-12/model/animator.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/resource/resource_store.h"
#include "pr/view3d-12/utility/update_resource.h"
#include "pr/view3d-12/utility/utility.h"

namespace pr::rdr12
{
	Pose::Pose(ResourceFactory& factory, SkeletonPtr skeleton, AnimatorPtr animator, EAnimStyle style, EAnimFlags flags, TimeRange time_range, double stretch, double bias)
		: m_animator(animator)
		, m_skeleton(skeleton)
		, m_res()
		, m_srv()
		, m_time_range(time_range)
		, m_time0(-1.0)
		, m_time1(0.0)
		, m_stretch(stretch)
		, m_bias(bias)
		, m_style(style)
		, m_flags(flags)
	{
		ResourceStore::Access store(factory.rdr());

		// Create the buffer for the bone matrices
		ResDesc rdesc = ResDesc::Buf<m4x4>(skeleton->BoneCount(), {}).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		m_res = factory.CreateResource(rdesc, "pose");

		// Create the pose SRV
		D3D12_SHADER_RESOURCE_VIEW_DESC srv_desc = {
			.Format = DXGI_FORMAT::DXGI_FORMAT_UNKNOWN,
			.ViewDimension = D3D12_SRV_DIMENSION::D3D12_SRV_DIMENSION_BUFFER,
			.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
			.Buffer = {
				.FirstElement = 0,
				.NumElements = s_cast<UINT>(BoneCount()),
				.StructureByteStride = sizeof(m4x4),
				.Flags = D3D12_BUFFER_SRV_FLAGS::D3D12_BUFFER_SRV_FLAG_NONE,
			},
		};
		m_srv = store.Descriptors().Create(m_res.get(), srv_desc);
	}
	
	// The root bone transform in animation space at 'time'
	m4x4 Pose::RootToAnim(double time, EAnimFlags flags) const
	{
		// Use the same 'm_flags' that are used for 'Update' because this will give the current root bone position
		m4x4 root;
		m_animator->Animate({ &root, 1ULL }, static_cast<float>(SrcAnimTime(time)), flags);
		return root;
	}
	m4x4 Pose::RootToAnim() const
	{
		return RootToAnim(m_time1, m_flags);
	}

	// Set the animation time
	void Pose::AnimTime(double time_s)
	{
		m_time1 = time_s;
	}

	// Number of bones in this pose
	int Pose::BoneCount() const
	{
		return m_skeleton->BoneCount();
	}

	// True if the current animation time is within the time range of this animation
	bool Pose::IsAnimating() const
	{
		switch (m_style)
		{
			// Never animating
			case EAnimStyle::NoAnimation:
			{
				return false;
			}

			// Only animating if the time is within the time range
			case EAnimStyle::Once:
			{
				auto time_s = (m_time1 - m_bias) * m_stretch + m_time_range.begin();
				return m_time_range.contains(time_s);
			}

			// This are all periodic so are always animating
			case EAnimStyle::Repeat:
			case EAnimStyle::Continuous:
			case EAnimStyle::PingPong:
			{
				return true;
			}

			default:
			{
				throw std::runtime_error("Unknown animation style");
			}
		}
	}
	
	// Return the time value relative to 'm_time_range' from the source animatino
	double Pose::SrcAnimTime(double time) const
	{
		return AdjTime((time - m_bias) * m_stretch + m_time_range.begin(), m_time_range, m_style);
	}

	// Reset to the rest pose
	void Pose::ResetPose(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer)
	{
		auto update = UpdateSubresourceScope(cmd_list, upload_buffer, m_res.get(), alignof(m4x4), 0, BoneCount() * sizeof(m4x4));
		m4x4* ptr = update.ptr<m4x4>();
		for (int i = 0, iend = BoneCount(); i != iend; ++i)
			ptr[i] = InvertOrthonormal(m_skeleton->m_o2bp[i]);

		update.Commit();
	}

	// Update the bone transforms
	void Pose::Update(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer)
	{
		// No change in time, assume up to date already
		if (m_time1 == m_time0)
			return;

		// No animator, return to the rest pose
		if (m_animator == nullptr)
		{
			ResetPose(cmd_list, upload_buffer);
			return;
		}

		assert(m_animator->SkelId() == m_skeleton->Id() && "Skeleton mismatch");

		// Populate the pose buffer. These are transforms from object-space to deformed-object-space.
		// I.e. object space verts are transformed to be bone relative, then deformed, then back to object space.
		//    `os_vert = cp2o * bp2cp * o2bp * os_vert`, where cp = current-pose, and bp = bind-pose.
		//               [ animation ] [skel]  [model]
		auto update = UpdateSubresourceScope(cmd_list, upload_buffer, m_res.get(), alignof(m4x4), 0, BoneCount() * sizeof(m4x4));
		auto ptr = update.ptr<m4x4>();
		{
			// Read the deformed bone transforms into the buffer to start with.
			// These are bone-to-parent transforms for each bone.
			m_animator->Animate({ ptr, s_cast<size_t>(BoneCount()) }, static_cast<float>(SrcAnimTime(m_time1)), m_flags);
			m_time0 = m_time1;

			// Convert the pose into object space transforms
			m_skeleton->WalkHierarchy<m4x4>([ptr, &o2bp = m_skeleton->m_o2bp](int idx, m4x4 const* p2o) -> m4x4
			{
				// Find the deformed bone-to-object space transform
				auto b2o = p2o ? *p2o * ptr[idx] : ptr[idx];

				// Update the pose buffer with the transform that takes object space verts, transforms them to
				// bind pose bone-relative, then from deformed bone space back to object space.
				ptr[idx] = b2o * o2bp[idx];

				// Return 'b2o' as the parent of any child bones
				return b2o;
			});
		}
		update.Commit();
	}

	// Ref-counting clean up function
	void Pose::RefCountZero(RefCounted<Pose>* doomed)
	{
		auto pose = static_cast<Pose*>(doomed);
		rdr12::Delete<Pose>(pose);
	}
}
