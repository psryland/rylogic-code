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
		double m_time0;               // The animation time of the evaluated matrices in 'm_matrices'
		double m_time1;               // The animation time to display next
		double m_stretch;             // Playback speed multiplier
		double m_bias;                // Time offset bias
		EAnimStyle m_style;           // The style of animation
		EAnimFlags m_flags;           // Behaviour flags
		uint64_t m_revision;          // Incremented each time the pose is evaluated, not a GPU-completion proof
		uint64_t m_uploaded_revision; // The revision whose matrices were last recorded for upload
		bool m_gpu_update_required;  // Re-evaluate and re-record GPU data even at unchanged animation time after an abandoned recording
		vector<m4x4> m_matrices;      // The evaluated skinning matrices. Upload memory is slow to read, so the pose is built here, then copied.

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

		// Return the animation time (in seconds) at which one pass through 'm_time_range' ends, including the bias. Returns zero for 'NoAnimation'.
		double Duration() const;

		// Return the time value relative to 'm_time_range' from the source animatino
		double SrcAnimTime(double time) const;

		// Revision of the most recently evaluated pose data, not GPU completion.
		uint64_t Revision() const;

		// Calculate object-space skinning matrices for the current animation time into 'pose', without changing this pose.
		void EvaluatePose(std::span<m4x4> pose) const;

		// Evaluate the skinning matrices at the current animation time into 'm_matrices' and raise PoseUpdated, unless they are already current.
		// Several render steps need the pose in one frame; the first to call this does the work and the others reuse it.
		void Evaluate();

		// Evaluate the pose (see Evaluate) and record the upload of its bone transforms, unless that revision is already recorded.
		// InvalidateGpuData must be called if these commands are abandoned before execution.
		void Update(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer);

		// Require a fresh evaluation and upload at the next Update, even if the animation time is unchanged. Use this after abandoning unsubmitted
		// pose commands, or after the animator's source data changes. Does not alter or free in-flight resources.
		void InvalidateGpuData();
    
		// Raised after Evaluate calculates new CPU pose matrices.
		EventHandler<Pose&, PoseUpdatedArgs const&, true> PoseUpdated;

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<Pose>* doomed);
	};
}
