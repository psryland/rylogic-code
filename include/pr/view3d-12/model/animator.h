//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/model/animation.h"

namespace pr::rdr12
{
	struct Animator :RefCounted<Animator>
	{
		// Notes:
		//  - This is a base class for a type that can update 'Skinning' instances.
		//  - The idea is that this could actually be a graph of Animator derived types
		//    that all feed into one to handle state machines, blend spaces, etc.
		//  - A skinning instance has an animator. It asks the animator to update its
		//    bone transforms as needed.
		//  - Animators should be state less because one Animator might be used my multiple
		//    skinning instances.

		Animator();
		virtual ~Animator() = default;

		// Return the ID of the skeleton we're animating
		virtual uint64_t SkelId() const = 0;

		// Return the frame rate of the underlying animation
		virtual double FrameRate() const = 0;

		// The length of the underlying animation
		virtual double Duration() const = 0;

		// Apply an animation to the given bones
		virtual void Animate(std::span<m4x4> bones, float time_s, EAnimFlags flags) = 0;

		// Clone this animator
		virtual AnimatorPtr Clone() const = 0;

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<Animator>* doomed);
	};

	struct Animator_KeyFrameAnimation : Animator
	{
		// Notes:
		//  - This animator reads from a single key frame animation

		KeyFrameAnimationPtr m_anim;   // The animation sequence to read from

		Animator_KeyFrameAnimation(KeyFrameAnimationPtr anim);

		// Return the ID of the skeleton we're animating
		uint64_t SkelId() const override;

		// Return the frame rate of the underlying animation
		double FrameRate() const override;
		
		// The length of the underlying animation
		double Duration() const override;

		// Apply an animation to the given bones
		void Animate(std::span<m4x4> bones, float time_s, EAnimFlags flags) override;

		// Clone this animator
		AnimatorPtr Clone() const override;
	};

	struct Animator_InterpolatedAnimation : Animator
	{
		// Notes:
		//  - This animator reads from a single key frame animation
		struct Interpolators
		{
			InterpolateRotation<float> rot;
			InterpolateVector<float> pos;
		};

		KinematicKeyFrameAnimationPtr m_anim; // The animation sequence to read from
		vector<Interpolators, 0> m_interp; // Interpolators for each track
		vector<KinematicKey, 0> m_keys; // A recycling buffer for reading key frames into
		TimeRange m_interp_time_range; // The time range of the current interpolation period

		Animator_InterpolatedAnimation(KinematicKeyFrameAnimationPtr anim);

		// Return the ID of the skeleton we're animating
		uint64_t SkelId() const override;

		// Return the frame rate of the underlying animation
		double FrameRate() const override;

		// The length of the underlying animation
		double Duration() const override;

		// Apply an animation to the given bones
		void Animate(std::span<m4x4> bones, float time_s, EAnimFlags flags) override;

		// Clone this animator
		AnimatorPtr Clone() const override;	};
}
