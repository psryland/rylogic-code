//*****************************************************************************
// Procedural animation - locomotion controller
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// Poses a character's body to follow its motion through the world, guided by a gait description (see gait.h):
//  - The caller chooses a movement family, such as "bipedal" or "quadruped". A change of family blends smoothly from the old family's stance
//    and walking limbs to the new family's.
//  - Each step builds a guide pose: the family's stance plus the repeating body motion of its cycles, blended between the two cycles either side
//    of the body's speed and scaled down towards a standstill. The guide sets the shape of the body, the necks, the tails, any digits, and the
//    hanging limbs. It never moves the feet.
//  - Walking limbs step in the footfall order of the family's cycles. Planted feet stay fixed in the world, swinging feet arc to a predicted
//    landing point on the ground, and a standing character steps until its feet are back beneath it. Stride and cadence follow the body's
//    actual speed, so the movement is never a playback of the authored cycles.
//  - The pelvis and chest follow the height of the ground under the hind and fore feet, so the body pitches on slopes. The cycles' hip height
//    sets how high the legs carry the body above the ground, blended by speed.
//  - Neck chains turn to look at a world point, fading in and out.
//  - Tail chains take their shape from the guide and are lifted out of the ground.
//  - A body that stays unsupported falls with its walking limbs spread out from the body, and they swing down to the ground when it lands.
//
// The body's motion comes from outside (for example, a physics simulation); the controller does not move the character's origin.
// Model space has +Z up. The body's up is the model's +Z axis in world space, so the body may stand on any surface: a slope, a wall, or a ceiling.
// The ground is found by short rays cast along the body's down direction, each about as long as the limb or tail that it serves. The rays are
// gathered so that each step queries the ground at most three times: once for the feet beneath the body, once for the landings of swinging feet,
// and once for the tail joints.
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/pose.h"
#include "pr/animation/targets.h"
#include "pr/animation/target_path.h"
#include "pr/animation/solve.h"
#include "pr/animation/tail.h"
#include "pr/animation/gait.h"

namespace pr::animation
{
	// A world-space ray that looks for the ground
	struct GroundRay
	{
		v4 origin = v4::Origin();     // The start of the ray (w = 1)
		v4 direction = -v4::ZAxis();  // Unit direction (w = 0)
		float length = 0.0f;          // The distance searched from the origin (> 0)
	};

	// Where a ground ray first meets a surface, in world space
	struct GroundHit
	{
		v4 point = v4::Origin();    // The surface point
		v4 normal = v4::ZAxis();    // The surface's unit normal, on the side that the ray came from
		bool hit = false;           // False if the ray met no surface within its length; 'point' and 'normal' are then unspecified
	};

	// The surfaces that a character can stand on
	struct IGround
	{
		virtual ~IGround() = default;

		// Write into 'hits[i]' the first surface that 'rays[i]' meets within its length. A ray that starts inside a solid meets it at its origin.
		// The spans have equal lengths. Callers gather their queries so that one call answers many rays.
		virtual void Probe(std::span<GroundRay const> rays, std::span<GroundHit> hits) const = 0;
	};

	// The world-space motion of a character's model origin
	struct BodyState
	{
		xform model_to_world = xform::Identity();
		v4 velocity = {};           // Linear velocity (w = 0)
		v4 angular_velocity = {};   // Axis * radians per second (w = 0)
		bool supported = true;      // False while the body is not resting on the ground, for example when falling. Brief losses are ignored, see 'Locomotion::AirborneDelay'.
	};

	// What a walking limb's foot is doing
	enum class EFootState
	{
		// Planted on the ground, fixed in world space
		Stance,

		// Moving through the air from its lift-off frame to its landing frame
		Swing,

		// Held beneath the body because the body is not supported
		Air,
	};

	// The state of one limb that walks in at least one family. Frames are world-space transforms of the limb's end bone.
	struct LimbState
	{
		int chain = -1;                       // The limb's chain index in the rig
		int contact = -1;                     // The contact that the limb stands on
		Footfall footfall;                    // The limb's timing at the current speed
		EFootState state = EFootState::Stance;
		float swing = 0.0f;                   // Swing progress in [0,1]
		float swing_window = 1.0f;            // The fraction of the step cycle that the current swing lasts
		xform foot = xform::Identity();       // The end bone's goal this step
		xform lift = xform::Identity();       // Swing: where the foot left the ground
		xform land = xform::Identity();       // Swing: where the foot will touch down
		v4 ground = v4::Origin();             // The point on the ground beneath the foot's contact, ignoring the swing's arc
		v4 stance_contact = v4::Origin();     // The limb's contact point in the family's stance, in model space, on the ground plane (z = 0)
		quat stance_rot = Identity<quat>();   // The end bone's model-space orientation in the family's stance
		float support = 1.0f;                 // In [0,1]. How much the limb stands on 'foot'. Otherwise it hangs from the body as in the guide pose.
		float reach = 0.0f;                   // The limb's length from its root joint to its contact. Ground rays for the foot reach this far either side of it.
	};

	// Animates a rig from its body's motion. See the top of this file.
	struct Locomotion
	{
		// Construct for 'rig', which must outlive the controller, with a gait that is valid for it. Throws 'std::invalid_argument' otherwise.
		// The character starts in the gait's first family.
		Locomotion(Rig const& rig, GaitDesc gait)
			: m_rig(rig)
			, m_gait(std::move(gait))
			, m_families()
			, m_pose(rig)
			, m_guide(rig)
			, m_guide_targets(rig.chains.size())
			, m_targets(rig.chains.size())
			, m_limbs()
			, m_neutral()
			, m_landings()
			, m_tail_joints()
			, m_probe_rays()
			, m_probe_hits()
			, m_rot_offset(rig.bones.size())
			, m_pos_offset(rig.bones.size())
			, m_tail()
			, m_tail_effectors()
			, m_legs()
			, m_leg_reach(0.0f)
			, m_spine(-1)
			, m_terrain()
			, m_family(0)
			, m_prev_family(0)
			, m_family_weight(1.0f)
			, m_gait_speed(0.0f)
			, m_look(v4::Origin())
			, m_look_weight(0.0f)
			, m_phase(0.0f)
			, m_unsupported_s(0.0f)
			, m_fall(0.0f)
			, m_up(v4::ZAxis())
			, m_started(false)
		{
			// Reject gaits that do not fit the rig at the construction boundary so that stepping can rely on them
			auto errors = Validate(m_gait, rig);
			if (!errors.empty())
			{
				std::string msg = "Invalid gait description:";
				for (auto const& e : errors)
					msg.append("\n  ").append(e);

				throw std::invalid_argument(msg);
			}

			// Collect every limb that walks in any family. A limb takes its timing from the first family that it walks in until it steps.
			for (auto const& family : m_gait.families)
			{
				for (auto const& foot : family.cycles[0].footfalls)
				{
					auto chain = LimbChain(rig, foot.role, foot.side);
					if (std::none_of(m_limbs.begin(), m_limbs.end(), [=](LimbState const& l) { return l.chain == chain; }))
					{
						auto contact = LimbContact(rig, rig.chains[chain]);
						m_limbs.push_back(LimbState{ .chain = chain, .contact = contact, .footfall = foot, .reach = ChainLength(chain, rig.rest_model[rig.contacts[contact].bone] * rig.contacts[contact].offset) });
					}
				}
			}
			m_neutral.resize(m_limbs.size());

			// The legs carry the hips at the cycles' hip height. Rigs that walk on no legs measure the hip height on all of their walking limbs.
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
			{
				if (rig.chains[m_limbs[i].chain].role == EChainRole::Leg)
					m_legs.push_back(i);
			}
			if (m_legs.empty())
			{
				for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
					m_legs.push_back(i);
			}
			for (auto i : m_legs)
				m_leg_reach += m_limbs[i].reach / static_cast<float>(m_legs.size());

			// Resolve each family's names to bone and limb indices
			for (auto const& family : m_gait.families)
				m_families.push_back(ResolveFamily(family));

			// The first spine chain carries the pelvis and chest
			for (int c = 0, cend = static_cast<int>(rig.chains.size()); c != cend && m_spine < 0; ++c)
			{
				if (rig.chains[c].role == EChainRole::Spine)
					m_spine = c;
			}

			// Stand in the first family's stance
			UseStance(m_family);
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
				m_limbs[i].support = m_families[m_family].limb_footfall[i] >= 0 ? 1.0f : 0.0f;
		}

		// The gait that guides the movement
		GaitDesc const& gait() const
		{
			return m_gait;
		}

		// The current pose, the chain targets that produced it, and the limbs that walk in any family
		Pose const& pose() const
		{
			return m_pose;
		}
		std::span<Target const> targets() const
		{
			return m_targets;
		}
		std::span<LimbState const> limbs() const
		{
			return m_limbs;
		}

		// The step cycle position in [0,1)
		float phase() const
		{
			return m_phase;
		}

		// The index of the movement family in 'gait().families' that the character is moving in, or blending to
		int family() const
		{
			return m_family;
		}

		// Blend to the movement family at 'index' in 'gait().families' over 'blend_s' seconds (> 0). A change during a blend starts from the
		// family that was being blended to.
		void SetFamily(int index, float blend_s)
		{
			// Choosing the current family changes nothing
			pr_assert(index >= 0 && index < static_cast<int>(m_families.size()) && "Family index out of range");
			if (index == m_family)
				return;

			// Feet find their new neutral positions from the new stance, and step to them as the limbs' support changes
			m_prev_family = m_family;
			m_family = index;
			m_family_weight = Fade(0.0f);
			m_family_weight.Start(1.0f, blend_s);
			UseStance(m_family);
		}

		// Turn the neck chains towards 'world_point', or back to rest if it is empty, over 'fade_s' seconds (> 0)
		void SetLook(std::optional<v4> world_point, float fade_s)
		{
			// Fading out keeps the last point so that the head turns back along the same path
			if (world_point)
				m_look = *world_point;

			m_look_weight.Start(world_point ? 1.0f : 0.0f, fade_s);
		}

		// Advance by 'dt' seconds (>= 0) and pose the rig for 'body' on 'ground'
		void Step(float dt, BodyState const& body, IGround const& ground)
		{
			// Find where each foot would stand beneath the body now. Planting, settling and lifting all compare the feet with these frames.
			m_up = Rotate(body.model_to_world.rot, v4::ZAxis());
			FindNeutralFeet(body.model_to_world, ground);

			// The first step plants every foot beneath the body
			if (!m_started)
			{
				for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
				{
					m_limbs[i].foot = m_neutral[i];
					m_limbs[i].state = EFootState::Stance;
				}
				m_started = true;
			}

			// Contact with the ground can drop out for a moment, for example as the body bumps over uneven ground, while the feet are still on it.
			// Only a support loss that lasts longer than 'AirborneDelay' lifts the feet off the ground, so brief losses do not interrupt the stepping.
			m_unsupported_s = body.supported ? 0.0f : m_unsupported_s + dt;
			auto airborne = m_unsupported_s > AirborneDelay;
			m_fall = Clamp(m_fall + (airborne ? dt : -dt) / FallResponse, 0.0f, 1.0f);

			// Measure the speed along the surface that the feet must keep up with, including turning on the spot. The family's slowest cycle sets
			// what counts as moving.
			auto const& family = m_gait.families[m_family];
			auto const& slowest = family.cycles.front();
			auto tangential = body.velocity - m_up * Dot(body.velocity, m_up);
			auto turn = Dot(body.angular_velocity, m_up) * TurnRadius();
			auto speed = Sqrt(LengthSq(tangential) + Sqr(turn));
			auto moving = !airborne && speed >= StandSpeedFraction * slowest.speed;

			// The cycle blend and the body motion follow a smoothed speed, so that footfall timing and body motion change gradually
			m_gait_speed += ((moving ? speed : 0.0f) - m_gait_speed) * std::min(dt / BodyResponse, 1.0f);

			// Blend the limbs' support and timing towards the chosen family
			m_family_weight.Step(dt);
			UpdateLimbs();

			// Advance the cycle. A standing body that is out of place steps at the slowest cycle's cadence.
			auto cadence = airborne ? 0.0f : moving ? speed / Stride(family, speed) : NeedsSettling() ? slowest.speed / slowest.stride : 0.0f;
			auto dphase = cadence * dt;
			m_phase = Frac(m_phase + dphase);

			// Advance each limb's stance and swing, collecting the landings that swinging feet still need to aim at
			m_landings.clear();
			m_probe_rays.clear();
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
				AdvanceLimb(i, body, airborne, moving, cadence, dphase);

			// Aim the swinging feet at the ground beneath their predicted landings
			auto hits = ProbeGround(ground);
			for (int k = 0, kend = static_cast<int>(m_landings.size()); k != kend; ++k)
			{
				auto& limb = m_limbs[m_landings[k].limb];
				limb.land = FootOnGround(limb, m_probe_rays[k], hits[k], m_landings[k].body_rot);
			}

			// Move each foot along its swing, then find the ground beneath it
			auto swing_height = SwingHeight(family);
			for (auto& limb : m_limbs)
			{
				MoveSwingingFoot(limb, swing_height * m_up);
				limb.ground = GroundUnder(limb);
			}

			// Build the guide pose. Its motion grows with speed up to the slowest cycle's speed, so a standing character holds the stance.
			BuildGuide(std::min(m_gait_speed / slowest.speed, 1.0f));
			PoseTargets(m_rig, m_guide.model, m_guide_targets);
			std::copy(m_guide_targets.begin(), m_guide_targets.end(), m_targets.begin());

			// Pose the body, then the chains that hang from it. A limb's goal moves from its hanging frame to its foot as it takes weight.
			auto world_to_model = Invert(body.model_to_world);
			StepSpine(dt, world_to_model);
			for (auto const& limb : m_limbs)
			{
				auto foot = world_to_model * limb.foot;
				m_targets[limb.chain].frame = limb.support == 1.0f ? foot : BlendFrame(HangingFrame(limb), foot, limb.support);
			}
			StepNecks(dt, world_to_model);

			// Bones that no chain solver poses, such as digits and tails, take their shape from the guide
			std::copy(m_guide.local.begin(), m_guide.local.end(), m_pose.local.begin());
			SolvePose(m_rig, m_targets, m_pose);
			StepTails(body.model_to_world, world_to_model, ground);
		}

	private:

		// The fraction of the slowest cycle's speed below which the body is standing
		static constexpr float StandSpeedFraction = 0.1f;

		// How long, in seconds, the body must be continuously unsupported before its feet leave the ground
		static constexpr float AirborneDelay = 0.1f;

		// The fraction of the slowest cycle's stride that a planted foot may be from its neutral position before a standing body steps
		static constexpr float SettleFraction = 0.15f;

		// The seconds that the body takes to follow changes of ground height and speed
		static constexpr float BodyResponse = 0.2f;

		// The seconds that the limbs take to spread out once airborne, or to fold back in on landing
		static constexpr float FallResponse = 0.3f;

		// A bone motion with its bone index resolved
		struct BoneTrack
		{
			int bone;
			EMotionChannel channel;
			std::array<Harmonic, GaitHarmonics> harmonics;
		};

		// A family with its names resolved
		struct FamilyData
		{
			std::vector<xform> stance_local;             // Per bone, the stance transform relative to the parent bone
			std::vector<xform> stance_model;             // Per bone, the stance transform in model space
			std::vector<std::vector<BoneTrack>> tracks;  // Per cycle, the bone motions
			std::vector<int> limb_footfall;              // Per limb in 'm_limbs', its footfall index in the family's cycles, or -1 if it does not walk
		};

		// The two cycles of a family either side of a speed, and the blend from the first (0) to the second (1)
		struct CycleBlend
		{
			int lo;
			int hi;
			float t;
		};

		// A swinging foot whose landing is found by the next ground query
		struct Landing
		{
			int limb;      // Index into 'm_limbs'
			quat body_rot; // The body's predicted orientation at the landing
		};

		// A tail joint whose height is checked against the ground
		struct TailJoint
		{
			int chain;     // The tail's chain index
			int joint;     // The joint's index within the chain, where the chain's base is 0
		};

		Rig const& m_rig;
		GaitDesc m_gait;
		std::vector<FamilyData> m_families;        // Parallel to 'm_gait.families'
		Pose m_pose;
		Pose m_guide;                              // This step's guide pose
		std::vector<Target> m_guide_targets;       // The chain targets that reproduce the guide pose
		std::vector<Target> m_targets;
		std::vector<LimbState> m_limbs;
		std::vector<xform> m_neutral;              // Per limb, the foot frame beneath the body this step
		std::vector<Landing> m_landings;           // The landings that this step's second ground query answers
		std::vector<TailJoint> m_tail_joints;      // The tail joints that this step's third ground query answers
		std::vector<GroundRay> m_probe_rays;       // The rays of the next ground query
		std::vector<GroundHit> m_probe_hits;       // The answers to the latest ground query
		std::vector<v4> m_rot_offset;              // Per bone, this step's rotation-vector motion (scratch)
		std::vector<v4> m_pos_offset;              // Per bone, this step's position motion (scratch)
		TailSolver m_tail;
		std::vector<fabrik::Effector<float>> m_tail_effectors;
		std::vector<int> m_legs;    // Indices into 'm_limbs' of the limbs that carry the hips at the cycles' hip height
		float m_leg_reach;          // The mean reach of 'm_legs', which scales the cycles' hip height
		int m_spine;                // The spine chain index, or -1 if the rig has none
		TargetPath m_terrain;       // The smoothed ground height under the hind feet (pos.x) and fore feet (pos.y), in model space
		int m_family;               // The family being moved in, or blended to
		int m_prev_family;          // The family being blended from
		Fade m_family_weight;       // From the previous family (0) to the current family (1)
		float m_gait_speed;         // The smoothed speed that selects cycles and scales body motion
		v4 m_look;                  // World-space look point
		Fade m_look_weight;         // How strongly the necks face 'm_look'
		float m_phase;              // The step cycle position in [0,1)
		float m_unsupported_s;      // How long the body has been continuously unsupported, in seconds
		float m_fall;               // In [0,1]. How far the limbs are spread out for falling.
		v4 m_up;                    // The body's up direction (model +Z) in world space this step
		bool m_started;               // False until the feet have been planted

		// Return the fractional part of 'x'
		static float Frac(float x)
		{
			return x - std::floor(x);
		}

		// Blend from frame 'a' (frac = 0) to frame 'b' (frac = 1)
		static xform BlendFrame(xform const& a, xform const& b, float frac)
		{
			// Positions blend linearly, rotations by the shortest arc
			return xform(Lerp(a.pos, b.pos, frac), Slerp(a.rot, b.rot, frac));
		}

		// Resolve the bone and limb names of 'family'
		FamilyData ResolveFamily(GaitFamily const& family) const
		{
			// The stance starts from the rest pose. Listed bones replace their rotation, and root bones may also replace their position.
			FamilyData data;
			for (auto const& bone : m_rig.bones)
				data.stance_local.push_back(bone.rest);

			for (auto const& bone : family.stance)
			{
				auto& local = data.stance_local[m_rig.BoneIndex(bone.bone)];
				local.rot = bone.rotation;
				if (bone.position)
					local.pos = *bone.position;
			}
			data.stance_model = ModelFrames(data.stance_local);

			// Raise or lower the root bones so that the stance carries the hip joints at the slowest cycle's hip height. The authored height
			// of the hips is ignored because the body's height should follow the legs, not the pose that the cycles happened to be made in.
			if (!m_legs.empty())
			{
				auto hip_z = 0.0f;
				for (auto i : m_legs)
					hip_z += data.stance_model[m_rig.chains[m_limbs[i].chain].bones.front()].pos.z / static_cast<float>(m_legs.size());

				auto lift = family.cycles.front().hip_height * m_leg_reach - hip_z;
				for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
				{
					if (m_rig.bones[b].parent < 0)
						data.stance_local[b].pos.z += lift;
				}
				data.stance_model = ModelFrames(data.stance_local);
			}

			// Each cycle's bone motions
			for (auto const& cycle : family.cycles)
			{
				auto& tracks = data.tracks.emplace_back();
				for (auto const& motion : cycle.motion)
					tracks.push_back(BoneTrack{ .bone = m_rig.BoneIndex(motion.bone), .channel = motion.channel, .harmonics = motion.harmonics });
			}

			// Map the controller's limbs to the family's footfalls
			auto const& footfalls = family.cycles[0].footfalls;
			for (auto const& limb : m_limbs)
			{
				auto it = std::find_if(footfalls.begin(), footfalls.end(), [&](Footfall const& f) { return LimbChain(m_rig, f.role, f.side) == limb.chain; });
				data.limb_footfall.push_back(it != footfalls.end() ? static_cast<int>(it - footfalls.begin()) : -1);
			}
			return data;
		}

		// The model-space frame of each bone, given each bone's frame relative to its parent
		std::vector<xform> ModelFrames(std::span<xform const> local) const
		{
			// Parents come before their children
			std::vector<xform> model;
			model.reserve(local.size());
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
			{
				auto parent = m_rig.bones[b].parent;
				model.push_back(parent < 0 ? local[b] : model[parent] * local[b]);
			}
			return model;
		}

		// Set each limb's stance contact and orientation from the stance of family 'f'
		void UseStance(int f)
		{
			// Contacts stand on the ground, so only their position across the ground plane is kept
			auto const& model = m_families[f].stance_model;
			for (auto& limb : m_limbs)
			{
				auto const& contact = m_rig.contacts[limb.contact];
				limb.stance_contact = model[contact.bone] * contact.offset;
				limb.stance_contact.z = 0.0f;
				limb.stance_rot = model[m_rig.chains[limb.chain].bones.back()].rot;
			}
		}

		// Find the cycles of 'family' either side of 'speed'
		static CycleBlend Bracket(GaitFamily const& family, float speed)
		{
			// Speeds outside the family's range use its slowest or fastest cycle
			auto const& cycles = family.cycles;
			auto hi = 0;
			for (auto hend = static_cast<int>(cycles.size()) - 1; hi != hend && cycles[hi].speed < speed; ++hi) {}
			auto lo = std::max(hi - 1, 0);
			if (lo == hi)
				return CycleBlend{ .lo = lo, .hi = hi, .t = 0.0f };

			return CycleBlend{ .lo = lo, .hi = hi, .t = Clamp((speed - cycles[lo].speed) / (cycles[hi].speed - cycles[lo].speed), 0.0f, 1.0f) };
		}

		// The distance the body moves per cycle at 'speed'
		static float Stride(GaitFamily const& family, float speed)
		{
			// Below the slowest cycle, shorter strides keep the cadence from slowing in proportion to the speed. Above the fastest cycle, the stride
			// stays at its longest authored length and the cadence rises, so that feet are never asked to reach further than in an example.
			auto const& cycles = family.cycles;
			if (speed <= cycles.front().speed)
				return cycles.front().stride * Sqrt(speed / cycles.front().speed);
			if (speed >= cycles.back().speed)
				return cycles.back().stride;

			auto b = Bracket(family, speed);
			return Lerp(cycles[b.lo].stride, cycles[b.hi].stride, b.t);
		}

		// The height that swinging feet are lifted at the current speed
		float SwingHeight(GaitFamily const& family) const
		{
			// Blend between the cycles either side of the smoothed speed
			auto b = Bracket(family, m_gait_speed);
			return Lerp(family.cycles[b.lo].swing_height, family.cycles[b.hi].swing_height, b.t);
		}

		// The model-space height by which family 'f' raises its hips above its stance at the current speed
		float HipLift(int f) const
		{
			// The stance already carries the hips at the slowest cycle's hip height
			auto const& family = m_gait.families[f];
			auto b = Bracket(family, m_gait_speed);
			auto hip_height = Lerp(family.cycles[b.lo].hip_height, family.cycles[b.hi].hip_height, b.t);
			return (hip_height - family.cycles.front().hip_height) * m_leg_reach;
		}

		// Blend each limb's support between the previous and current families, and take its timing from the family that it walks in
		void UpdateLimbs()
		{
			// A limb that walks in the current family uses its timing there; otherwise it keeps the previous family's timing while it fades out
			auto w = m_family_weight.Value();
			auto const& prev = m_families[m_prev_family];
			auto const& curr = m_families[m_family];
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
			{
				auto& limb = m_limbs[i];
				auto in_prev = prev.limb_footfall[i] >= 0 ? 1.0f : 0.0f;
				auto in_curr = curr.limb_footfall[i] >= 0 ? 1.0f : 0.0f;
				limb.support = in_prev + (in_curr - in_prev) * w;

				auto f = curr.limb_footfall[i] >= 0 ? m_family : prev.limb_footfall[i] >= 0 ? m_prev_family : -1;
				if (f < 0)
					continue;

				// Blend the timing between the cycles either side of the speed. Touchdowns blend the short way around the cycle.
				auto const& family = m_gait.families[f];
				auto index = m_families[f].limb_footfall[i];
				auto b = Bracket(family, m_gait_speed);
				auto const& lo = family.cycles[b.lo].footfalls[index];
				auto const& hi = family.cycles[b.hi].footfalls[index];
				auto shift = Frac(hi.touchdown - lo.touchdown + 0.5f) - 0.5f;
				limb.footfall.touchdown = Frac(lo.touchdown + shift * b.t);
				limb.footfall.duty = Lerp(lo.duty, hi.duty, b.t);
			}
		}

		// Set 'm_guide' to the blended family stance plus the cycles' body motion, scaled by 'amplitude'
		void BuildGuide(float amplitude)
		{
			// Blend the stances of the previous and current families
			auto w = m_family_weight.Value();
			auto const& prev = m_families[m_prev_family].stance_local;
			auto const& curr = m_families[m_family].stance_local;
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
				m_guide.local[b] = w == 1.0f ? curr[b] : BlendFrame(prev[b], curr[b], w);

			// Sum the motion of both families, each blended between its cycles either side of the speed
			std::fill(m_rot_offset.begin(), m_rot_offset.end(), v4::Zero());
			std::fill(m_pos_offset.begin(), m_pos_offset.end(), v4::Zero());
			AddMotion(m_prev_family, (1.0f - w) * amplitude);
			AddMotion(m_family, w * amplitude);

			// Root bones carry the hips at the hip height of the cycles either side of the speed
			auto lift = v4(0, 0, Lerp(HipLift(m_prev_family), HipLift(m_family), w), 0);
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
			{
				if (m_rig.bones[b].parent < 0)
					m_pos_offset[b] += lift;
			}

			// Rotations apply in the bone's own frame after the stance rotation. Positions are model-space offsets of root bones.
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
			{
				auto& local = m_guide.local[b];
				local.rot = Normalise(local.rot * RotationFromVector(m_rot_offset[b]));
				local.pos += m_pos_offset[b];
			}
			m_guide.UpdateModel(m_rig);
		}

		// Add the motion of family 'f' at the current phase, scaled by 'weight', to the motion offsets
		void AddMotion(int f, float weight)
		{
			// A family with no weight adds nothing
			if (weight == 0)
				return;

			auto b = Bracket(m_gait.families[f], m_gait_speed);
			auto const& tracks = m_families[f].tracks;
			for (auto [cycle, frac] : { std::pair{ b.lo, 1.0f - b.t }, std::pair{ b.hi, b.t } })
			{
				// The bracketing cycles are the same cycle outside the family's speed range
				if (frac == 0)
					continue;

				for (auto const& track : tracks[cycle])
				{
					auto value = Evaluate(track.harmonics, m_phase) * (weight * frac);
					switch (track.channel)
					{
						case EMotionChannel::Rotation:
						{
							m_rot_offset[track.bone] += value;
							break;
						}
						case EMotionChannel::Position:
						{
							m_pos_offset[track.bone] += value;
							break;
						}
						default:
						{
							throw std::invalid_argument("Unknown motion channel");
						}
					}
				}
			}
		}

		// The end bone frame of a hanging limb: its frame in the guide pose, carried by the body. Legs hang from the pelvis and arms from the chest.
		xform HangingFrame(LimbState const& limb) const
		{
			// Without a spine the body does not move relative to the guide
			auto guide_end = m_guide.model[m_rig.chains[limb.chain].bones.back()];
			if (m_spine < 0)
				return guide_end;

			auto leg = limb.footfall.role == EChainRole::Leg;
			auto const& actual = leg ? m_targets[m_spine].base : m_targets[m_spine].frame;
			auto const& guide = leg ? m_guide_targets[m_spine].base : m_guide_targets[m_spine].frame;
			return actual * Invert(guide) * guide_end;
		}

		// The distance from the model origin at which turning on the spot moves the feet: the mean horizontal distance of the stance
		// contacts, weighted by how much each limb supports the body
		float TurnRadius() const
		{
			// Hanging limbs do not need to keep up with turning
			auto radius = 0.0f;
			auto support = 0.0f;
			for (auto const& limb : m_limbs)
			{
				radius += limb.support * Sqrt(Sqr(limb.stance_contact.x) + Sqr(limb.stance_contact.y));
				support += limb.support;
			}
			return support > 0 ? radius / support : 0.0f;
		}

		// The world-space contact point of a foot frame
		v4 ContactOf(LimbState const& limb, xform const& foot) const
		{
			return foot * m_rig.contacts[limb.contact].offset;
		}

		// Query the ground along every ray in 'm_probe_rays' in one call. The answers stay valid until the next query.
		std::span<GroundHit const> ProbeGround(IGround const& ground)
		{
			// An empty query does not reach the ground at all
			m_probe_hits.resize(m_probe_rays.size());
			if (!m_probe_rays.empty())
				ground.Probe(m_probe_rays, m_probe_hits);

			return m_probe_hits;
		}

		// The ray that looks for the ground beneath 'point' for a body whose up direction is 'up'. It starts 'reach' above the point and
		// searches as far below it, so that ground that rises or falls by up to 'reach' is found.
		static GroundRay RayBeneath(v4 point, v4 up, float reach)
		{
			return GroundRay{ .origin = point + up * reach, .direction = -up, .length = 2.0f * reach };
		}

		// The end bone frame that puts the limb's contact on the ground at 'hit', with the stance's orientation relative to the body, tilted from
		// the body's up to the ground's normal. A ray that found no ground leaves the foot reaching to the end of the ray.
		xform FootOnGround(LimbState const& limb, GroundRay const& ray, GroundHit const& hit, quat const& body_rot) const
		{
			// Turn the stance orientation from the body's up to the ground's normal
			auto const& contact = m_rig.contacts[limb.contact];
			auto point = hit.hit ? hit.point : ray.origin + ray.direction * ray.length;
			auto normal = hit.hit ? hit.normal : -ray.direction;
			auto rot = Normalise(quat(Rotate(body_rot, v4::ZAxis()), normal) * body_rot * limb.stance_rot);
			return xform(point - Rotate(rot, contact.offset.w0()), rot);
		}

		// Find the foot frame beneath each stance contact for a body at 'model_to_world'
		void FindNeutralFeet(xform const& model_to_world, IGround const& ground)
		{
			// One query covers every limb
			m_probe_rays.clear();
			for (auto const& limb : m_limbs)
				m_probe_rays.push_back(RayBeneath(model_to_world * limb.stance_contact, m_up, limb.reach));

			auto hits = ProbeGround(ground);
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
				m_neutral[i] = FootOnGround(m_limbs[i], m_probe_rays[i], hits[i], model_to_world.rot);
		}

		// True if limb 'i' is planted further than the settle distance from its neutral position
		bool IsDisplaced(int i) const
		{
			// Only the distance along the surface matters; the neutral frame is already on the ground
			auto const& limb = m_limbs[i];
			auto offset = ContactOf(limb, limb.foot) - ContactOf(limb, m_neutral[i]);
			offset -= m_up * Dot(offset, m_up);
			auto settle = SettleFraction * m_gait.families[m_family].cycles.front().stride;
			return LengthSq(offset) > Sqr(settle);
		}

		// True if a standing body must keep stepping: a supporting foot is swinging or is away from its neutral position
		bool NeedsSettling() const
		{
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
			{
				if (m_limbs[i].support != 0 && (m_limbs[i].state != EFootState::Stance || IsDisplaced(i)))
					return true;
			}
			return false;
		}

		// The body's model-to-world transform after 't' seconds, assuming constant velocity
		static xform PredictBody(BodyState const& body, float t)
		{
			// Rotate about the angular velocity's axis, if there is any rotation
			auto rot = body.model_to_world.rot;
			auto angle = Length(body.angular_velocity) * t;
			if (angle > tiny<float>)
				rot = Normalise(quat(Normalise(body.angular_velocity), angle) * rot);

			return xform(body.model_to_world.pos + body.velocity * t, rot);
		}

		// Advance limb 'i' through its stance and swing. A swinging foot that still needs a landing adds its query to 'm_probe_rays'.
		void AdvanceLimb(int i, BodyState const& body, bool airborne, bool moving, float cadence, float dphase)
		{
			// An airborne body spreads its feet out from its sides, arms forwards and legs backwards, which steadies a falling body. On regaining
			// support, the feet swing down to the ground.
			auto& limb = m_limbs[i];
			auto duty = limb.footfall.duty;
			if (airborne)
			{
				limb.state = EFootState::Air;
				limb.foot = body.model_to_world * BlendFrame(m_families[m_family].stance_model[m_rig.chains[limb.chain].bones.back()], FallFrame(limb), m_fall);
				return;
			}
			if (limb.state == EFootState::Air)
				StartSwing(limb, 1.0f - duty);

			// A limb that carries no weight does not step. Its foot follows the ground beneath its stance contact, ready to take weight.
			if (limb.support == 0)
			{
				limb.state = EFootState::Stance;
				limb.foot = m_neutral[i];
				return;
			}

			// A planted foot lifts once the cycle reaches its swing, if the body is moving or the foot is out of place. A swing that would be
			// too short to look like a step waits for the next cycle.
			auto local = Frac(m_phase - limb.footfall.touchdown);
			constexpr float min_swing_window = 0.05f;
			if (limb.state == EFootState::Stance && local >= duty && 1.0f - local > min_swing_window && (moving || IsDisplaced(i)))
				StartSwing(limb, 1.0f - local);

			if (limb.state != EFootState::Swing)
				return;

			// Aim the landing at the neutral position at the middle of the next stance, so that the foot passes beneath the body. The landing
			// is fixed late in the swing so that the foot settles onto a steady point.
			limb.swing = std::min(limb.swing + dphase / limb.swing_window, 1.0f);
			constexpr float land_freeze = 0.7f;
			if (limb.swing < land_freeze)
			{
				auto to_landing = cadence > 0 ? (1.0f - limb.swing) * limb.swing_window / cadence : 0.0f;
				auto half_stance = moving && cadence > 0 ? 0.5f * duty / cadence : 0.0f;
				auto predicted = PredictBody(body, to_landing + half_stance);
				m_landings.push_back(Landing{ .limb = i, .body_rot = predicted.rot });
				m_probe_rays.push_back(RayBeneath(predicted * limb.stance_contact, Rotate(predicted.rot, v4::ZAxis()), limb.reach));
			}
		}

		// The model-space end bone frame of a limb spread out for falling: reaching out from its root joint, sideways and slightly upwards,
		// arms angled forwards and legs backwards, in the stance's orientation
		xform FallFrame(LimbState const& limb) const
		{
			// Model space faces -Y. The side comes from the limb's root joint, so left and right limbs spread apart.
			auto const& model = m_families[m_family].stance_model;
			auto const& chain = m_rig.chains[limb.chain];
			auto root = model[chain.bones.front()].pos;
			auto side = root.x < 0 ? -1.0f : 1.0f;
			auto fore = limb.footfall.role == EChainRole::Leg ? 1.0f : -1.0f;
			auto direction = Normalise(v4(side, 0.6f * fore, 0.2f, 0));

			// Keep a little bend in the limb, and place the end bone so that its contact is at the reach point
			auto rot = model[chain.bones.back()].rot;
			auto contact = root + direction * (0.9f * limb.reach);
			return xform(contact - Rotate(rot, m_rig.contacts[limb.contact].offset.w0()), rot);
		}

		// The length of chain 'c' in the rest pose: the sum of its bone lengths, ending at 'tip' (model space)
		float ChainLength(int c, v4 tip) const
		{
			// Each bone's length is the distance to the next joint
			auto const& bones = m_rig.chains[c].bones;
			auto length = 0.0f;
			for (int k = 0, kend = static_cast<int>(bones.size()); k != kend; ++k)
			{
				auto next = k + 1 != kend ? m_rig.rest_model[bones[k + 1]].pos : tip;
				length += Length(next - m_rig.rest_model[bones[k]].pos);
			}
			return length;
		}

		// Move a swinging foot from lift-off towards its landing, raised by up to 'lift', touching down at the end of the swing
		static void MoveSwingingFoot(LimbState& limb, v4 lift)
		{
			// Only swinging feet move
			if (limb.state != EFootState::Swing)
				return;

			if (limb.swing >= 1.0f)
			{
				limb.foot = limb.land;
				limb.state = EFootState::Stance;
				return;
			}

			// Move smoothly from lift-off to landing, raised by an arc that is flat at both ends
			auto s = limb.swing;
			auto ease = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
			auto arc = 64.0f * Cube(s * (1.0f - s));
			limb.foot = xform(limb.lift.pos + (limb.land.pos - limb.lift.pos) * ease + lift * arc, Slerp(limb.lift.rot, limb.land.rot, ease));
		}

		// Lift a foot from where it is, for a swing lasting 'window' of the step cycle
		static void StartSwing(LimbState& limb, float window)
		{
			limb.state = EFootState::Swing;
			limb.swing = 0.0f;
			limb.swing_window = window;
			limb.lift = limb.foot;
			limb.land = limb.foot;
		}

		// The point on the ground beneath a limb's contact, ignoring the swing's arc
		v4 GroundUnder(LimbState const& limb) const
		{
			switch (limb.state)
			{
				case EFootState::Stance:
				case EFootState::Air:
				{
					return ContactOf(limb, limb.foot);
				}
				case EFootState::Swing:
				{
					auto s = limb.swing;
					auto ease = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
					return ContactOf(limb, limb.lift) + (ContactOf(limb, limb.land) - ContactOf(limb, limb.lift)) * ease;
				}
				default:
				{
					throw std::invalid_argument("Unknown foot state");
				}
			}
		}

		// Raise or lower the guide's pelvis and chest by the height of the ground under the hind and fore feet, pitching the body between them
		void StepSpine(float dt, xform const& world_to_model)
		{
			// Hind feet are legs and fore feet are arms. With only one group, the body moves without pitching.
			if (m_spine < 0)
				return;

			float height[2] = {};
			int count[2] = {};
			auto arm_support = 0.0f;
			for (auto const& limb : m_limbs)
			{
				if (limb.state == EFootState::Air)
					continue;

				auto i = limb.footfall.role == EChainRole::Leg ? 0 : 1;
				height[i] += (world_to_model * limb.ground).z;
				count[i] += 1;
				arm_support += i == 1 ? limb.support : 0.0f;
			}

			// Hanging arms do not set the height of the chest, so the fore height moves from the hind height to the arms' height as they
			// take weight
			auto hind = count[0] != 0 ? height[0] / count[0] : count[1] != 0 ? height[1] / count[1] : 0.0f;
			auto fore = count[1] != 0 ? hind + (height[1] / count[1] - hind) * (count[0] != 0 ? arm_support / count[1] : 1.0f) : hind;

			// Follow the ground heights smoothly so that steps on uneven ground do not jolt the body. The guide's own motion is not smoothed.
			m_terrain.Retarget(xform(v4(hind, fore, 0, 1), Identity<quat>()), BodyResponse);
			m_terrain.Step(dt);
			hind = m_terrain.Frame().pos.x;
			fore = m_terrain.Frame().pos.y;

			// Each end moves by its own height, and both turn by the pitch of the line between them
			auto const& pelvis0 = m_guide_targets[m_spine].base;
			auto const& chest0 = m_guide_targets[m_spine].frame;
			auto span = chest0.pos - pelvis0.pos;
			auto pitch = quat(Normalise(span), Normalise(span + v4::ZAxis() * (fore - hind)));
			m_targets[m_spine].base = xform(pelvis0.pos + v4::ZAxis() * hind, Normalise(pitch * pelvis0.rot));
			m_targets[m_spine].frame = xform(chest0.pos + v4::ZAxis() * fore, Normalise(pitch * chest0.rot));
		}

		// Turn each neck from its guide aim towards the look point by the look weight
		void StepNecks(float dt, xform const& world_to_model)
		{
			m_look_weight.Step(dt);
			auto weight = m_look_weight.Value();
			auto look = world_to_model * m_look;
			for (int c = 0, cend = static_cast<int>(m_rig.chains.size()); c != cend; ++c)
			{
				// Rotate the guide aim point about the neck's base, so that a zero weight gives the guide aim exactly
				auto const& chain = m_rig.chains[c];
				if (chain.role != EChainRole::Neck)
					continue;

				auto base = m_guide.model[chain.bones.front()].pos;
				auto aim = m_guide_targets[c].frame.pos - base;
				auto turn = Slerp(Identity<quat>(), quat(Normalise(aim), Normalise(look - base, Normalise(aim))), weight);
				m_targets[c].frame.pos = base + Rotate(turn, aim);
			}
		}

		// Lift any part of each tail that is below the ground back onto the ground, measured along the body's up direction
		void StepTails(xform const& model_to_world, xform const& world_to_model, IGround const& ground)
		{
			// Gather the joints of every tail, so that one ground query covers all of them. Each joint's ray comes down to the joint from as
			// far above it as the joint is from the tail's base, which is attached to the body, so a hit means the joint is buried.
			m_tail_joints.clear();
			m_probe_rays.clear();
			for (int c = 0, cend = static_cast<int>(m_rig.chains.size()); c != cend; ++c)
			{
				// Check each joint after the base, which is attached to the body
				auto const& chain = m_rig.chains[c];
				if (chain.role != EChainRole::Tail)
					continue;

				auto count = static_cast<int>(chain.bones.size());
				auto base = model_to_world * m_pose.model[chain.bones.front()].pos;
				for (int k = 1; k <= count; ++k)
				{
					auto joint = model_to_world * (k != count ? m_pose.model[chain.bones[k]].pos : m_pose.Tip(m_rig, chain.bones.back()));
					auto reach = Length(joint - base);
					m_tail_joints.push_back(TailJoint{ .chain = c, .joint = k });
					m_probe_rays.push_back(GroundRay{ .origin = joint + m_up * reach, .direction = -m_up, .length = reach });
				}
			}

			// Pull each buried joint up to the ground above it. The joints of one tail are adjacent in the query.
			auto hits = ProbeGround(ground);
			for (int j = 0, jend = static_cast<int>(m_tail_joints.size()); j != jend;)
			{
				// Gather the effectors of one tail, then solve it if any joint was buried
				auto chain = m_tail_joints[j].chain;
				m_tail_effectors.clear();
				for (; j != jend && m_tail_joints[j].chain == chain; ++j)
				{
					if (hits[j].hit)
						m_tail_effectors.push_back(fabrik::Effector<float>{ .joint = m_tail_joints[j].joint, .target = world_to_model * hits[j].point });
				}
				if (!m_tail_effectors.empty())
					m_tail.Solve(m_rig, chain, m_pose, m_tail_effectors, fabrik::Settings<float>{});
			}
			m_pose.UpdateModel(m_rig);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	// A solid ground plane through the origin with unit normal 'm_normal'. Counts the queries it answers.
	struct TestGround : IGround
	{
		v4 m_normal;
		mutable int m_queries;

		// A plane with height 'slope' * y
		explicit TestGround(float slope = 0.0f)
			: TestGround(Normalise(v4(0, -slope, 1, 0)))
		{
		}
		explicit TestGround(v4 normal)
			: m_normal(normal)
			, m_queries(0)
		{
		}

		void Probe(std::span<GroundRay const> rays, std::span<GroundHit> hits) const override
		{
			// Intersect each ray with the plane. A ray that starts inside the ground hits at its origin.
			for (size_t i = 0; i != rays.size(); ++i)
			{
				// Measure heights above the plane along its normal
				auto const& ray = rays[i];
				auto h0 = Dot(ray.origin.w0(), m_normal);
				auto rate = Dot(ray.direction, m_normal);
				auto t = h0 <= 0 ? 0.0f : rate < 0 ? -h0 / rate : ray.length + 1;
				hits[i] = t <= ray.length
					? GroundHit{ .point = ray.origin + ray.direction * t, .normal = m_normal, .hit = true }
					: GroundHit{ .point = ray.origin + ray.direction * ray.length, .normal = -ray.direction, .hit = false };
			}
			++m_queries;
		}
	};

	PRUnitTestClass(LocomotionTests)
	{
		PRUnitTestMethod(StandingAtRest, Quick)
		{
			// A still body on flat ground holds the stance, which for the test gait is the rest pose, and does not step
			auto rig = TestRig();
			auto loco = Locomotion(rig, TestGait());
			auto ground = TestGround();
			for (int i = 0; i != 60; ++i)
				loco.Step(1.0f / 60, BodyState{}, ground);

			PR_EXPECT(loco.phase() == 0.0f);
			for (int b = 0, bend = static_cast<int>(rig.bones.size()); b != bend; ++b)
				PR_EXPECT(FEqlAbsolute(loco.pose().model[b].pos, rig.rest_model[b].pos, 1e-3f));
		}
		PRUnitTestMethod(WalkingPlantsFeet, Quick)
		{
			// Walking forward (-Y) at the walk cycle's speed, planted feet stay fixed and on the ground, every foot steps, and the cycle advances
			// by distance over the walk cycle's stride
			auto rig = TestRig();
			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };
			loco.Step(0, body, ground);
			auto previous = std::vector<LimbState>(loco.limbs().begin(), loco.limbs().end());
			std::vector<int> touchdowns(previous.size());
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				for (size_t l = 0; l != previous.size(); ++l)
				{
					auto const& limb = loco.limbs()[l];
					if (limb.state == EFootState::Stance && previous[l].state == EFootState::Stance)
						PR_EXPECT(FEqlAbsolute(limb.foot, previous[l].foot, 1e-6f));
					if (limb.state == EFootState::Stance && previous[l].state == EFootState::Swing)
						touchdowns[l] += 1;
					if (limb.state == EFootState::Stance)
						PR_EXPECT(FEqlAbsolute(limb.ground.z, 0.0f, 1e-5f));

					previous[l] = limb;
				}
			}
			for (auto n : touchdowns)
				PR_EXPECT(n >= 2);

			// Each step queries the ground at most three times however many points it needs
			PR_EXPECT(ground.m_queries <= 3 * 121);

			auto cycles = walk.speed * 120 * dt / walk.stride;
			PR_EXPECT(FEqlAbsolute(loco.phase(), cycles - std::floor(cycles), 1e-3f));
		}
		PRUnitTestMethod(StoppingSettles, Quick)
		{
			// After walking then stopping, the feet step back beneath the body and the cycle stops
			auto rig = TestRig();
			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };
			for (int i = 0; i != 50; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			body.velocity = v4::Zero();
			for (int i = 0; i != 300; ++i)
				loco.Step(dt, body, ground);

			auto phase = loco.phase();
			loco.Step(dt, body, ground);
			PR_EXPECT(loco.phase() == phase);
			for (auto const& limb : loco.limbs())
			{
				auto neutral = body.model_to_world * limb.stance_contact;
				PR_EXPECT(limb.state == EFootState::Stance);
				PR_EXPECT(Length((limb.ground - neutral).w0()) <= 0.15f * walk.stride + 1e-4f);
			}
		}
		PRUnitTestMethod(BriefSupportLossKeepsFeetPlanted, Quick)
		{
			// A walking body that loses support for single steps keeps its planted feet fixed, but a sustained loss lifts every foot into the air
			auto rig = TestRig();
			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };
			loco.Step(0, body, ground);
			auto previous = std::vector<LimbState>();
			for (int i = 0; i != 120; ++i)
			{
				// Drop support on every fourth step
				previous.assign(loco.limbs().begin(), loco.limbs().end());
				body.model_to_world.pos += body.velocity * dt;
				body.supported = i % 4 != 3;
				loco.Step(dt, body, ground);
				for (size_t l = 0; l != previous.size(); ++l)
				{
					auto const& limb = loco.limbs()[l];
					PR_EXPECT(limb.state != EFootState::Air);
					if (limb.state == EFootState::Stance && previous[l].state == EFootState::Stance)
						PR_EXPECT(FEqlAbsolute(limb.foot, previous[l].foot, 1e-6f));
				}
			}

			// A loss that lasts longer than the delay is a fall
			body.supported = false;
			for (int i = 0; i != 10; ++i)
				loco.Step(dt, body, ground);

			for (auto const& limb : loco.limbs())
				PR_EXPECT(limb.state == EFootState::Air);
		}
		PRUnitTestMethod(SlopePitchesBody, Quick)
		{
			// Walking across ground that falls away ahead (-Y is downhill when the slope is positive), then standing, the supporting feet
			// rest on the ground and the chest drops
			auto rig = TestRig();
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround(0.2f);
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(gait.families[0].cycles[0].speed, 0, 0, 0) };
			for (int i = 0; i != 60; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			body.velocity = v4::Zero();
			for (int i = 0; i != 180; ++i)
				loco.Step(dt, body, ground);

			for (auto const& limb : loco.limbs())
			{
				auto contact = body.model_to_world * (loco.pose().model[rig.contacts[limb.contact].bone] * rig.contacts[limb.contact].offset);
				PR_EXPECT(FEqlAbsolute(contact.z, 0.2f * contact.y, 1e-3f));
			}
			auto chest = rig.BoneIndex("chest");
			PR_EXPECT(loco.pose().model[chest].pos.z < rig.rest_model[chest].pos.z - 0.05f);
		}
		PRUnitTestMethod(HipHeightFollowsLegs, Quick)
		{
			// The legs carry the hip joint at the cycles' hip height, as a fraction of the leg's reach: the slowest cycle's height when standing,
			// and the fastest cycle's height at its speed. The feet stay on the ground.
			auto rig = TestRig();
			auto gait = TestGait();
			auto& quad = gait.families[0];
			quad.cycles[0].hip_height = 0.5f;
			quad.cycles[1].hip_height = 0.7f;
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto thigh = rig.BoneIndex("thigh.L");
			auto reach = loco.limbs()[0].reach;
			auto dt = 1.0f / 60;
			auto body = BodyState{};
			for (int i = 0; i != 60; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(FEqlAbsolute(loco.pose().model[thigh].pos.z, 0.5f * reach, 1e-3f));

			body.velocity = v4(0, -quad.cycles[1].speed, 0, 0);
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			PR_EXPECT(FEqlAbsolute(loco.pose().model[thigh].pos.z, 0.7f * reach, 1e-2f));
			for (auto const& limb : loco.limbs())
			{
				if (limb.state == EFootState::Stance)
					PR_EXPECT(FEqlAbsolute(limb.ground.z, 0.0f, 1e-5f));
			}
		}
		PRUnitTestMethod(WallWalkingPlantsFeetOnWall, Quick)
		{
			// A body on a vertical wall (the ground is x < 0), with its up along +X, walks along the wall and then stands. The feet rest on the wall.
			auto rig = TestRig();
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround(v4::XAxis());
			auto dt = 1.0f / 60;
			auto body = BodyState{ .model_to_world = xform(v4::Origin(), quat(v4::ZAxis(), v4::XAxis())), .velocity = v4(0, -gait.families[0].cycles[0].speed, 0, 0) };
			for (int i = 0; i != 60; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				for (auto const& limb : loco.limbs())
				{
					if (limb.state == EFootState::Stance)
						PR_EXPECT(FEqlAbsolute(limb.ground.x, 0.0f, 1e-4f));
				}
			}
			body.velocity = v4::Zero();
			for (int i = 0; i != 180; ++i)
				loco.Step(dt, body, ground);

			for (auto const& limb : loco.limbs())
			{
				auto contact = body.model_to_world * (loco.pose().model[rig.contacts[limb.contact].bone] * rig.contacts[limb.contact].offset);
				PR_EXPECT(limb.state == EFootState::Stance);
				PR_EXPECT(FEqlAbsolute(contact.x, 0.0f, 1e-3f));
			}
		}
		PRUnitTestMethod(FallingSpreadsLimbs, Quick)
		{
			// A body that stays unsupported spreads its feet out to its sides, and folds them back to the ground after landing
			auto rig = TestRig();
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .supported = false };
			for (int i = 0; i != 60; ++i)
				loco.Step(dt, body, ground);

			for (auto const& limb : loco.limbs())
			{
				auto contact = loco.pose().model[rig.contacts[limb.contact].bone] * rig.contacts[limb.contact].offset;
				PR_EXPECT(limb.state == EFootState::Air);
				PR_EXPECT(Abs(contact.x) > Abs(limb.stance_contact.x) + 0.1f * limb.reach);
				PR_EXPECT(contact.z > limb.stance_contact.z + 0.1f * limb.reach);
			}

			body.supported = true;
			for (int i = 0; i != 180; ++i)
				loco.Step(dt, body, ground);

			for (auto const& limb : loco.limbs())
			{
				PR_EXPECT(limb.state == EFootState::Stance);
				PR_EXPECT(FEqlAbsolute(limb.ground.z, 0.0f, 1e-4f));
			}
		}
		PRUnitTestMethod(BodyMotionFollowsSpeed, Quick)
		{
			// The walk cycle sways the chest sideways and bobs the hips, but only while moving
			auto rig = TestRig();
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto chest = rig.BoneIndex("chest");
			auto hips = rig.BoneIndex("hips");
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -gait.families[0].cycles[0].speed, 0, 0) };
			for (int i = 0; i != 60; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}

			// Over the next cycles the chest swings to both sides and the hips rise and fall
			auto sway = v2(0, 0);
			auto bob = v2(1e10f, -1e10f);
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				auto x = loco.pose().model[chest].pos.x;
				auto z = loco.pose().model[hips].pos.z;
				sway = v2(std::min(sway.x, x), std::max(sway.y, x));
				bob = v2(std::min(bob.x, z), std::max(bob.y, z));
			}
			PR_EXPECT(sway.x < -0.02f && sway.y > 0.02f);
			PR_EXPECT(bob.y - bob.x > 0.02f);

			// Standing still, the motion dies away to the stance
			body.velocity = v4::Zero();
			for (int i = 0; i != 300; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(FEqlAbsolute(loco.pose().model[chest].pos.x, 0.0f, 1e-3f));
		}
		PRUnitTestMethod(FamilyChangesWalkingLimbs, Quick)
		{
			// Changing from the quadruped to the bipedal family fades the arm's support out, after which it hangs in the guide pose and only
			// the leg steps
			auto rig = TestRig();
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -gait.families[0].cycles[0].speed, 0, 0) };
			loco.Step(0, body, ground);
			PR_EXPECT(loco.limbs().size() == 2 && loco.limbs()[1].support == 1.0f);

			loco.SetFamily(FamilyIndex(gait, "bipedal"), 0.5f);
			PR_EXPECT(loco.family() == 1);
			for (int i = 0; i != 15; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			auto const& arm = loco.limbs()[1];
			PR_EXPECT(arm.support > 0.0f && arm.support < 1.0f);

			for (int i = 0; i != 60; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			PR_EXPECT(arm.support == 0.0f && arm.state == EFootState::Stance);
			PR_EXPECT(loco.limbs()[0].support == 1.0f);

			// Standing still, the hanging hand returns to its place in the bipedal stance, which is the rest pose
			body.velocity = v4::Zero();
			for (int i = 0; i != 300; ++i)
				loco.Step(dt, body, ground);

			auto hand = rig.BoneIndex("hand.L");
			PR_EXPECT(FEqlAbsolute(loco.pose().model[hand].pos, rig.rest_model[hand].pos, 1e-3f));
		}
		PRUnitTestMethod(LookFadesInAndOut, Quick)
		{
			// The head turns to face a look point, then back to rest
			auto rig = TestRig();
			auto loco = Locomotion(rig, TestGait());
			auto ground = TestGround();
			auto head = rig.BoneIndex("head");
			auto const& neck = rig.chains[1];
			auto facing = [&]
			{
				// The direction the head faces, from its rotation away from rest
				return Rotate(loco.pose().model[head].rot * ~rig.rest_model[head].rot, neck.forward);
			};

			auto look = v4(3, -1.1f, 1.2f, 1);
			loco.SetLook(look, 0.25f);
			for (int i = 0; i != 60; ++i)
				loco.Step(1.0f / 60, BodyState{}, ground);

			PR_EXPECT(Dot(facing(), Normalise(look - loco.pose().model[head].pos)) > 0.95f);

			loco.SetLook(std::nullopt, 0.25f);
			for (int i = 0; i != 60; ++i)
				loco.Step(1.0f / 60, BodyState{}, ground);

			PR_EXPECT(Dot(facing(), neck.forward) > 0.999f);
		}
	};
}
#endif
