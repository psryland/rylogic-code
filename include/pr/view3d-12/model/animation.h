//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Notes:
	//  - Bones belong to Skeletons.
	//  - Tracks belong to Animations.
	//  - Animation has these parts:
	//    - KeyFrameAnimation = a buffer of KeyFrames where each KeyFrame contains transforms for one or more bones in a skeleton.
	//        KeyFrames can be sparse.
	//        There can be less tracks than bones in the skeleton. In this case, bones without tracks use their rest pose transform.
	//        The transform at each key is the *local transform* of that bone — that is: the transform from
	//        the parent bone’s space to bone space at that time. Therefore, in the rest pose (i.e., bind pose),
	//        these transforms match the skeleton's local bone transforms.
	//    - Skeleton = Source data describing a hierarchy of bone transforms. Bone transforms are parent relative
	//        with the root bone in animation space.
	//    - Pose = A runtime skeleton instance, updated by an Animator using interpolated transforms from an Animation.
	//        The pose transform array is used in the shader to skin the model so the transforms must be from object
	//        space to deformed object space. They are then transformed using o2w into world space.
	//    - Skin = A set of bones indices and weights for each unique vertex of a model.
	//        Since model verts can be duplicated because of different normals, UVs, etc, each vert in the vertex buffer
	//        should have an 'original vert index' value. This is used to look up the skin vert which then gives the
	//        bone indices and weights.
	//    - Animator = The class that determines the pose at a given time. Animator is intended to be a base class that
	//        might one day support blend spaces or other things. For now, it just interpolates KeyFrameAnimation instances.
	//
	//  - Terms:
	//    - An animation with frame rate 'FPS' has 'time_length / FPS' frames.
	//    - Only some frames are 'KeyFrames' (aka Keys). An animation can have N KeyFrames != FrameCount.
	//    - For looped animations, the last KeyFrame should match the first KeyFrame
	//
	//  - Graphics Models contain a skin because the skin is 1:1 with the model and doesn't change.
	//  - Instances contain a pose because poses change with time and can use the same model but at different animation times.
	//  - Poses reference an Animator, a Skeleton, and an animation time. Multiple Instances can reference the same pose.
	//    Think of a Pose as a dynamic instance of a skeleton.
	//  - A skeleton can be referenced by many poses. A skeleton is basically static data that animations are relative to.
	//  - An Animator is used when updating the transforms in a Pose. The Animator interpolates the bone offsets which are
	//    then used when calculating the pose's bone-to-object space transforms.
	//  - Model hierarchies need to include a transform from child model to root model space, because the pose transforms
	//    are the same for all models in the hierarchy, i.e., in object space.

	// The root bone is always track 0
	static constexpr int RootBoneTrack = 0;

	// Simple animation styles
	enum class EAnimStyle : uint8_t
	{
		// Animation stays as if at time = 0
		NoAnimation,

		// Animation plays through to the end then stops
		Once,

		// Animation plays through to the end, then jumps back to the start
		Repeat,

		// Same as repeat, except that the root motion continues from the end
		Continuous,

		// Animation bounces from start to end to start continuously
		PingPong,
	};

	// Different methods for interpolating between key-frames
	enum class EAnimInterpolation
	{
		Constant = 0,
		Linear = 1,
		Cubic = 2,
	};

	// Behaviour flags for an animation
	enum class EAnimFlags : uint32_t
	{
		None = 0,
		NoRootTranslation = 1 << 0,
		NoRootRotation = 1 << 1,
		_flags_enum = 0,
	};

	// Interface for reading animation data from various sources
	struct IAnimSource
	{
		virtual ~IAnimSource() = default;
		virtual int key_count() const noexcept = 0;
		virtual int track_count() const noexcept = 0;
		virtual int fcurve_count() const noexcept = 0;
		virtual int tcurve_count() const noexcept = 0;
		virtual double frame_rate() const noexcept = 0;
		virtual int key_to_frame(int key_index) const = 0;
		virtual uint16_t track_to_bone(int track_index) const = 0;
		virtual void ReadTrackValues(int frame_index, int track_index, std::span<xform> samples) const = 0;
		virtual void ReadFCurveValues(int frame_index, int track_index, std::span<float> samples) const = 0;
		virtual void ReadTCurveValues(int frame_index, int track_index, std::span<xform> samples) const = 0;
	};

	// A clip within an animation
	struct Clip
	{
		float m_start = 0; // The time offset to start the clip from
		float m_duration = std::numeric_limits<float>::max(); // The length of the clip
		float m_bias = 0; // The offset to apply when playing the clip
	};

	// Transient type for a bone transform
	struct BoneKey
	{
		quat m_rot = quat::Identity();
		v3 m_pos = v3::Zero();
		v3 m_scl = v3::One();
		float m_time = 0; // seconds
		EAnimInterpolation m_interp = EAnimInterpolation::Linear;
		int m_idx = 0;

		// Convert to an affine transform
		operator m4x4() const
		{
			return m4x4(ToMatrix<m3x4>(m_rot) * m3x4::Scale(m_scl), m_pos.w1());
		}

		// Convert to xform
		operator xform() const
		{
			return xform{m_pos.w1(), m_rot, m_scl.w1()};
		}

		// Interpolate between two key frames
		friend BoneKey Interp(BoneKey const& lhs, BoneKey const& rhs, float frac, EAnimInterpolation interp);
	};

	// A single key frame that includes kinematic data
	struct KinematicKey
	{
		quat m_rot = quat::Identity();
		v3 m_pos = v3::Zero();
		v3 m_scl = v3::Zero();
		v3 m_lin_vel = v3::Zero();
		v3 m_ang_vel = v3::Zero();
		v3 m_lin_acc = v3::Zero();
		v3 m_ang_acc = v3::Zero();
		float m_time = 0; // seconds
		int m_idx = 0;

		// Convert to an affine transform
		operator m4x4() const
		{
			return m4x4(ToMatrix<m3x4>(m_rot) * m3x4::Scale(m_scl), m_pos.w1());
		}

		// Convert to xform
		operator xform() const
		{
			return xform{m_pos.w1(), m_rot, m_scl.w1()};
		}

		// Convert to a BoneKey
		operator BoneKey() const
		{
			return BoneKey{ m_rot, m_pos, m_scl, m_time, EAnimInterpolation::Linear, m_idx };
		}

		// Interpolate between two key frames
		friend KinematicKey Interp(KinematicKey const& lhs, KinematicKey const& rhs, float frac, EAnimInterpolation interp);
	};

	// A reference to a specific key in one of multiple animation sources
	struct KeyRef
	{
		int source_index; // Index into an array of animation sources
		int key_index;  // Key number within that source (clamped to valid range)
	};

	// Simple root motion polynomial animation
	struct RootAnimation : RefCounted<RootAnimation>
	{
		v4         m_vel;    // Linear velocity of the animation in m/s
		v4         m_acc;    // Linear velocity of the animation in m/s
		v4         m_avel;   // Angular velocity of the animation in rad/s
		v4         m_aacc;   // Angular velocity of the animation in rad/s
		double     m_period; // Time range in seconds
		EAnimStyle m_style;  // The animation style

		RootAnimation();

		// Return a transform representing the offset added by this object at time 'time_s'
		m4x4 EvaluateAtTime(double time_s) const;

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<RootAnimation>* doomed);
	};

	// Animation using key frame data
	struct KeyFrameAnimation : RefCounted<KeyFrameAnimation>
	{
		// Notes:
		//  - "Frames" are the spans of time between "Keys".
		//    Frames ==> 0   1   2   3   4   5
		//             |...|...|...|...|...|...|
		//    Keys ==> 0   1   2   3   4   5   6
		//  - If the frame rate is 24 fps, then at t == 1sec, key24 is about to start (because key0 is at t = 0).
		//    Put another way, a 24fps animation clip requires 25 keys in order to last for 1 second.
		//  - Track data are stored interleaved for each key, e.g.,
		//      m_rotation: [key0:(track0,track1,track2,..)][key1:(track0,track1,track2,..)][...
		//      m_position: [key0:(track0,track1,track2,..)][key1:(track0,track1,track2,..)][...
		//      m_scale:    [key0:(track0,track1,track2,..)][key1:(track0,track1,track2,..)][...
		//    This is because it's more cache friendly to have all data for a key local in memory.
		//  - Any of the tracks can be empty. The lengths will be either equal or zero.
		using Sample = vector<BoneKey, 0>;

		uint32_t m_skel_id;         // The skeleton that this animation is intended for (mainly for debugging)
		double m_native_duration;   // The length (in seconds) of this animation
		double m_native_frame_rate; // The native frame rate of the animation (for reference. frame rate is implied key_count and duration)

		vector<uint16_t, 0> m_bone_map; // The bone id for each track. Length = track count.

		// Any of these tracks can be empty. Length = track count * key count
		vector<quat, 0> m_rotation;
		vector<v3, 0> m_position;
		vector<v3, 0> m_scale;

		KeyFrameAnimation(uint32_t skel_id, double native_duration, double native_frame_rate);

		// Number of tracks in this animation
		int track_count() const;

		// Number of float curves in this animation
		int fcurve_count() const;

		// Number of transform curves in this animation
		int tcurve_count() const;

		// Number of keys in this animation
		int key_count() const;

		// The length (in seconds) of this animation
		double duration() const;

		// The frame rate of this animation
		double frame_rate() const;

		// Convert a time in seconds to a key index. Returns the key with time just less that 'time_s'
		int TimeToKeyIndex(float time_s) const;

		// Converts a key index to a time in seconds
		float KeyIndexToTime(int key_index) const;

		// Read keys starting at 'key_idx' for all tracks. 'out.size()' should be a multiple of the track count
		void ReadKeys(int key_idx, std::span<BoneKey> out) const;
		void ReadKeys(int key_idx, std::span<xform> out) const;
		void ReadKeys(int key_idx, std::span<m4x4> out) const;

		// Read keys starting at 'key_idx' for the given 'track_index'. 'out.size()' is the number of keys to read
		void ReadKeys(int key_idx, int track_index, std::span<BoneKey> out) const;
		void ReadKeys(int key_idx, int track_index, std::span<xform> out) const;
		void ReadKeys(int key_idx, int track_index, std::span<m4x4> out) const;

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<KeyFrameAnimation>* doomed);
	};

	// Animation data where each key frame also contains velocities and accelerations
	struct KinematicKeyFrameAnimation : RefCounted<KinematicKeyFrameAnimation>
	{
		// Notes:
		//  - See "KeyFrameAnimation", however, keys != frames + 1 here because keys are sparse.
		using Sample = vector<KinematicKey, 0>;

		uint32_t m_skel_id;         // The skeleton that this animation is intended for (mainly for debugging)
		double m_native_duration;   // The length (in seconds) of this animation
		double m_native_frame_rate; // The native frame rate of the animation. This doesn't really have meaning when the keys are not evenly spaced
		int m_key_count;            // The number of kinematic frames. Track lengths should match this or be empty

		vector<uint16_t, 0> m_bone_map;  // The bone id for each track. Length = track count.
		vector<uint8_t, 0> m_fcurve_ids; // Identifiers for the float curves. Length = fcurve count
		vector<uint8_t, 0> m_tcurve_ids; // Identifiers for the transform curves. Length = tcurve count

		// Any of these tracks can be empty. Length = track count * key count
		vector<quat, 0> m_rotation;   // Bone rotation data per frame.
		vector<v3, 0> m_ang_vel;    // Angular velocity per track, per frame.
		vector<v3, 0> m_ang_acc;    // Angular acceleration per track, per frame.
		vector<v3, 0> m_position;   // Bone position data per track, per frame.
		vector<v3, 0> m_lin_vel;    // Linear velocity per track, per frame.
		vector<v3, 0> m_lin_acc;    // Linear Acceleration per track, per frame.
		vector<v3, 0> m_scale;      // Bone scale data per track, per frame.
		vector<float, 0> m_fcurves; // Float curve data per fcurve id, per frame.
		vector<xform, 0> m_tcurves; // Transform curve data per tcurve id, per frame.
		vector<float, 0> m_times;   // Time (in seconds) of each key. Empty if a fixed frame rate.
		vector<int, 0> m_fidxs;     // Frame index of each key frame. Empty if one key per frame.

		explicit KinematicKeyFrameAnimation(uint32_t skel_id);

		// Number of tracks in this animation
		int track_count() const;

		// Number of float curves in this animation
		int fcurve_count() const;

		// Number of transform curves in this animation
		int tcurve_count() const;

		// Number of keys in this animation
		int key_count() const;

		// Get the frame number in the source animation for the given key index
		int src_frame(int key_index) const;

		// Ranged for helper. Returns pairs of src frame number and animaion time for that frame
		auto src_frames() const;

		// The length (in seconds) of this animation
		double duration() const;

		// The effective frame rate implied by the duration and number of keys
		double frame_rate() const;

		// Get the root to animation space transform for 'key_index'
		xform root_to_anim(int key_index) const;

		// Convert a time in seconds to a key index. Returns the key with time just less that 'time_s'
		int TimeToKeyIndex(float time_s) const;

		// Converts a key index to a time in seconds
		float KeyIndexToTime(int key_index) const;

		// Read keys starting at 'key_idx' for all tracks. 'out.size()' should be a multiple of the track count
		void ReadKeys(int key_idx, std::span<KinematicKey> out) const;
		void ReadKeys(int key_idx, std::span<xform> out) const;
		void ReadKeys(int key_idx, std::span<m4x4> out) const;

		// Read keys starting at 'key_idx' for the given 'track_index'. 'out.size()' is the number of keys to read
		void ReadKeys(int key_idx, int track_index, std::span<KinematicKey> out) const;
		void ReadKeys(int key_idx, int track_index, std::span<xform> out) const;
		void ReadKeys(int key_idx, int track_index, std::span<m4x4> out) const;

		// Populate this kinematic animation from 'src' using the given 'frames' and 'durations'.
		void Populate(IAnimSource const& src, std::span<int const> frames, std::span<float const> durations, std::span<m4x4 const> per_frame_r2a = {});
		void Populate(KeyFrameAnimation const& kfa, std::span<int const> frames, std::span<float const> durations, std::span<m4x4 const> per_frame_r2a = {});
		void Populate(std::span<KeyFrameAnimationPtr const> sources, std::span<KeyRef const> key_refs, std::span<float const> durations, std::span<m4x4 const> per_frame_r2a = {});

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<KinematicKeyFrameAnimation>* doomed);
	};

	// Use 'style' to adjust 'time_s' so that it is within the given time range
	double AdjTime(double time_s, TimeRange time_range, EAnimStyle style);

	// Convert a frame range to a time range based on the given frame rate
	inline TimeRange ToTimeRange(FrameRange frames, double frame_rate)
	{
		assert(frame_rate > 0.0);
		return TimeRange {
			static_cast<double>(frames.m_beg / frame_rate),
			static_cast<double>(frames.m_end / frame_rate),
		};
	}

	// Convert a frame range to a time range based on the given frame rate
	inline FrameRange ToFrameRange(TimeRange times, double frame_rate)
	{
		assert(frame_rate > 0.0);
		return FrameRange {
			static_cast<int>(times.m_beg * frame_rate),
			static_cast<int>(times.m_end * frame_rate),
		};
	}
}
