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
		// Interpolate sparse kinematic keys while preserving endpoint velocities and accelerations.
		using HermiteTransform = Hermite5Transform<float>;

		KinematicKeyFrameAnimationPtr m_anim; // The animation sequence to read from
		vector<HermiteTransform, 0> m_interp; // Interpolators for each track
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
		AnimatorPtr Clone() const override;
	};
}

#if PR_UNITTESTS
namespace pr::rdr12::tests
{
	// Verify sparse playback preserves sampled motion, including at the sequence boundaries.
	PRUnitTestClass(InterpolatedAnimationTests)
	{
		// Construct non-uniform keys from polynomial motion with nonzero endpoint derivatives.
		static KinematicKeyFrameAnimationPtr MakeAnimation()
		{
			auto anim = KinematicKeyFrameAnimationPtr{ ::pr::compute::New<KinematicKeyFrameAnimation>(1), true };
			anim->m_key_count = 3;
			anim->m_native_duration = 1.0;
			anim->m_native_frame_rate = 2.0;
			anim->m_bone_map = {0};
			anim->m_times = {0.f, 0.35f, 1.f};
			for (auto t : anim->m_times)
			{
				anim->m_position.push_back(v3(1 + 2*t + 2*t*t + t*t*t*t, 0, 0));
				anim->m_lin_vel.push_back(v3(2 + 4*t + 4*t*t*t, 0, 0));
				anim->m_lin_acc.push_back(v3(4 + 12*t*t, 0, 0));
				anim->m_rotation.push_back(quat(v4::ZAxis(), 0.1f + 0.2f*t + 0.3f*t*t + 0.1f*t*t*t));
				anim->m_ang_vel.push_back(v3(0, 0, 0.2f + 0.6f*t + 0.3f*t*t));
				anim->m_ang_acc.push_back(v3(0, 0, 0.6f + 0.6f*t));
			}
			return anim;
		}

		// Quintic playback reproduces polynomial motion when seeking forward and backward across keys.
		PRUnitTestMethod(PreservesKinematicConstraints, Quick)
		{
			auto anim = MakeAnimation();
			auto animator = Animator_InterpolatedAnimation(anim);
			auto bones = std::array<m4x4, 1>{};
			for (auto t : {1.f, 0.f, 0.1f, 0.35f, 0.6f, 0.99f, 1.f, 0.7f, 0.2f, 0.f})
			{
				animator.Animate(bones, t, EAnimFlags::None);
				auto local_time = t - s_cast<float>(animator.m_interp_time_range.begin());
				auto const& interp = animator.m_interp[0];
				PR_EXPECT(FEqlAbsolute(bones[0].pos, v4(1 + 2*t + 2*t*t + t*t*t*t, 0, 0, 1), 0.0001f));
				PR_EXPECT(FEqlAbsolute(interp.pos.EvalDerivative(local_time), v4(2 + 4*t + 4*t*t*t, 0, 0, 0), 0.0001f));
				PR_EXPECT(FEqlAbsolute(interp.pos.EvalDerivative2(local_time), v4(4 + 12*t*t, 0, 0, 0), 0.001f));
				PR_EXPECT(FEqlAbsolute(interp.rot.Eval(local_time), quat(v4::ZAxis(), 0.1f + 0.2f*t + 0.3f*t*t + 0.1f*t*t*t), 0.0001f));
				PR_EXPECT(FEqlAbsolute(interp.rot.EvalDerivative(local_time), v4(0, 0, 0.2f + 0.6f*t + 0.3f*t*t, 0), 0.0001f));
				PR_EXPECT(FEqlAbsolute(interp.rot.EvalDerivative2(local_time), v4(0, 0, 0.6f + 0.6f*t, 0), 0.001f));
			}
		}

		// Out-of-range seeks hold the endpoint pose instead of extrapolating sampled derivatives.
		PRUnitTestMethod(SeeksHoldEndpoints, Quick)
		{
			auto animator = Animator_InterpolatedAnimation(MakeAnimation());
			auto bones = std::array<m4x4, 1>{};
			animator.Animate(bones, 2.f, EAnimFlags::None);
			PR_EXPECT(FEqlAbsolute(bones[0].pos, v4(6, 0, 0, 1), 0.0001f));
			PR_EXPECT(FEqlAbsolute(s_cast<float>(animator.m_interp_time_range.begin()), 0.35f, 0.0001f));
			animator.Animate(bones, -1.f, EAnimFlags::None);
			PR_EXPECT(FEqlAbsolute(bones[0].pos, v4(1, 0, 0, 1), 0.0001f));
		}

		// Root-motion display flags do not change the underlying interpolated motion.
		PRUnitTestMethod(RootMotionFlagsPreserveCurves, Quick)
		{
			auto animator = Animator_InterpolatedAnimation(MakeAnimation());
			auto bones = std::array<m4x4, 1>{};
			animator.Animate(bones, 0.2f, EAnimFlags::NoTranslation | EAnimFlags::NoRotation);
			PR_EXPECT(FEqlAbsolute(bones[0], m4x4::Identity(), 0.0001f));
			animator.Animate(bones, 0.2f, EAnimFlags::None);
			PR_EXPECT(bones[0].pos.x > 1.f);
		}
	};
}
#endif
