//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/model/animation.h"

namespace pr::rdr12
{
	// Arguments for the PoseUpdated event.
	struct PoseUpdatedArgs
	{
		std::span<m4x4 const> m_pose; // Object-space skinning matrices. Valid only during the synchronous event call.
		uint64_t m_revision;          // The pose revision represented by 'm_pose'
	};

	// A runtime version of a skeleton
	struct Pose : RefCounted<Pose>
	{
		using Descriptor = ::pr::compute::Descriptor;
		using GfxCmdList = ::pr::compute::GfxCmdList;
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;

		// See description in "animation.h"
		AnimatorPtr m_animator;       // The driver of the animation
		SkeletonPtr m_skeleton;       // The skeleton (in rest-pose)
		D3DPtr<ID3D12Resource> m_res; // The runtime bone buffer (i.e. m4x4[])
		Descriptor m_srv;             // SRV of the bone buffer
		TimeRange m_time_range;       // The time span from the animation to use
		double m_time0;               // The animation time last recorded for upload
		double m_time1;               // The animation time to display next
		double m_stretch;             // Playback speed multiplier
		double m_bias;                // Time offset bias
		EAnimStyle m_style;           // The style of animation
		EAnimFlags m_flags;           // Behaviour flags
		uint64_t m_revision;          // Incremented when a pose upload is recorded, not a GPU-completion proof
		bool m_gpu_update_required;  // Re-record GPU data even at unchanged animation time after an abandoned recording

		Pose(ResourceFactory& factory, SkeletonPtr skeleton, AnimatorPtr animator, EAnimStyle style, EAnimFlags flags, TimeRange time_range, double stretch, double bias);

		// The root bone transform in animation space at 'time'
		m4x4 RootToAnim(double time, EAnimFlags flags) const;
		m4x4 RootToAnim() const;

		// Set the animation time
		void AnimTime(double time_s);

		// Number of bones in this pose
		int BoneCount() const;

		// True if the current animation time is within the time range of this animation
		bool IsAnimating() const;

		// Return the time value relative to 'm_time_range' from the source animatino
		double SrcAnimTime(double time) const;

		// Revision of the most recently recorded pose data, not GPU completion.
		uint64_t Revision() const;

		// Calculate object-space skinning matrices for the current animation time without updating GPU resources.
		void EvaluatePose(std::span<m4x4> pose) const;

		// Reset to the rest pose
		void ResetPose(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer);

		// Record updated bone transforms. InvalidateGpuData must be called if these commands are abandoned before execution.
		void Update(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer);

		// Require a fresh upload after the caller abandons unsubmitted pose commands. Does not alter or free in-flight resources.
		void InvalidateGpuData();
    
		// Raised after CPU pose matrices are calculated.
		EventHandler<Pose&, PoseUpdatedArgs const&, true> PoseUpdated;

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<Pose>* doomed);
	};
}
