//*****************************************************************************
// Procedural animation - locomotion controller
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// Poses a character's body to follow its motion through the world, guided by a gait description (see gait.h):
//  - The caller requests a movement family, such as "bipedal" or "quadruped". The character moves in the requested family while the ground
//    beneath its feet is no steeper than the family's slope limit, and otherwise in the first family of the gait that the ground allows. A
//    change of family blends smoothly from the old family's stance and walking limbs to the new family's.
//  - Each step builds a guide pose: the stance plus the repeating body motion of the family's cycles, both blended between the two cycles either side
//    of the body's speed, with the motion scaled down towards a standstill. The guide sets the shape of the body, the necks, the tails, any digits, and the
//    hanging limbs. It never moves the feet.
//  - Walking limbs step in the footfall order of the family's cycles. Planted feet stay fixed in the world, swinging feet arc up and out from
//    the body to a predicted landing point on the ground, and a standing character steps until its feet are back beneath it. Stride and cadence
//    follow the body's actual speed, so the movement is never a playback of the authored cycles.
//  - A raised foot turns with its limb, keeping the stance's angle at the ankle or wrist, and turns back to the ground's orientation as it comes
//    down to land.
//  - A planted foot or hand lies flat on the ground. Its sole is the plane through its contact and the contacts at the tips of its digits in
//    the rest pose. The digits of a planted foot lie along that plane with their tips on it.
//  - The pelvis and chest follow the height of the ground under the hind and fore feet, so the body pitches on slopes. The cycles' hip height
//    sets how high the legs carry the body above the ground, blended by speed.
//  - The pelvis and chest turn about the up axis in time with the legs and arms by the cycles' spine sway, so the spine bends from side to
//    side as the limbs step. While the body turns, the chest leads into the turn and the pelvis trails it.
//  - Neck chains turn to look at a world point, fading in and out.
//  - The body breathes at a rate and depth set by the caller. Each breath turns the chest bone about its head so that its tip moves out from the
//    front of the body, so the rib cage swells, and rolls the shoulders back. The necks keep aiming where the guide aims, so the head moves only a little.
//    Callers can move things held against the chest with the chest's motion, see 'Locomotion::chest_motion'.
//  - Each tail trails behind its base along the ground like a rope, as one smooth curve: the first bone continues the line of the spine with a
//    small bend down, and each later bone bends a little at a time, steeply down while it can and then flatter, until the tail's skin rests
//    on the ground. The first bone heads across the ground with the swaying hips, and the dragged rope behind it sets which way each later bone
//    heads, so the sway travels down the tail as an 'S' wave. The first bone's joint limits are measured from the guide's tail shape.
//  - A body that stays unsupported falls with its walking limbs spread out from the body, and they swing down to the ground when it lands.
//  - A swimming family (medium "water") has no walking limbs, so every limb hangs from the body in the guide pose, and the body is never
//    airborne. The body and tails ignore the ground beneath them, and the family's body wave bends the spine and tails from side to side.
//    Its cycles play at the caller's stroke speed rather than the body's measured speed, because strokes push the body rather than follow it.
//    While the body enters or leaves the water, the body's height and the tails blend between following the ground and floating.
//    A swimmer that the caller asks to right itself turns its limbs towards its lowest side as it leans, so the low limbs reach out and the high limbs fold in.
//  - The caller can move an arm's palm to a grasp: a world point that the palm touches, the direction that it faces, and the direction that its
//    fingers point, so the grasp sets the hand's whole orientation and the arm's usual swing does not turn it. The hand fades to and
//    from the grasp. A grasp beyond the arms' reach makes the body stoop: the hips sink and move back and the spine pitches forward, by the
//    least amount that brings every grasping arm within reach. A grasping arm's elbow flares out from the body and stays outside the torso, which
//    is the capsule around the spine that just reaches the arms' roots in the rest pose. A grasp with a wrap radius closes the hand's digits
//    around a bar of that radius that lies against the palm at the grip point, across the fingers.
//  - An action family (see gait.h) plays a timed action on top of its base family's stance, holding the body in place:
//     - "collapse" plays once over its duration and then holds. The knees buckle as the body sinks, the body topples sideways onto its left side
//       until it lies flat, and the limbs go limp towards the stance. The arms, legs and head then sag under gravity until they rest on the ground
//       or reach their joint limits. Every bone keeps within its joint limits. Leaving the family plays the
//       collapse backwards, so the body stands back up the way it fell, before any other action starts. The feet do not step while collapsed.
//       The family's posture (see 'Locomotion::posture') is its base stance lying on its side.
//     - "punches" loops its sequence of punches. The feet take a boxing stance with the left foot forward, and the body crouches with its hands
//       holding a guard in front of the chin. Each punch drives one fist out and back while the torso turns the punching shoulder forward and
//       leans after the fist, carrying the hips and head with it. The punches fade in and out with the family's blend time.
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
		v4 gravity = {};            // The world-space acceleration due to gravity (w = 0), or zero if there is none. Families with a slope limit need it.
		float stroke_speed = 0.0f;  // The speed that a water family's strokes would drive the body at, in m/s (>= 0). Water families play their cycles at this speed rather than the measured velocity, and never slower than their slowest cycle, see 'Locomotion::Advance'.
		float calm_tilt = 0.0f;     // The tilt (radians) up to which a swimming body makes no effort to right itself, see 'Locomotion::RightingReach'. Needs 'gravity'.
		float rough_tilt = 0.0f;    // The tilt (radians, > 'calm_tilt' to enable righting) at which a swimming body's limbs reach fully towards its lowest side. Zero disables righting.
	};

	// A breathing motion. Each breath eases in and out, peaking halfway through the breath. Zero depths give no motion.
	struct Breath
	{
		float rate = 0.0f;       // Breaths per second (>= 0)
		float chest = 0.0f;      // How far the tip of the chest bone moves out from the front of the body at the peak of a breath, in metres (>= 0)
		float shoulders = 0.0f;  // How far the shoulders roll back at the peak of a breath, in radians (>= 0)
	};

	// Where a hand holds something, in world space
	struct Grasp
	{
		v4 point = v4::Origin();  // The point that the palm's grip point touches (w = 1), see 'LimbState::grip'
		v4 palm = -v4::ZAxis();   // The unit direction that the palm faces, towards the held thing (w = 0)
		v4 fingers = -v4::YAxis(); // The direction that the fingers point (w = 0), not parallel to 'palm'. Only its part across the palm is used.
		float wrap = 0.0f;        // The radius (metres, >= 0) of a bar that the digits close around, such as a handle, or 0 to leave the digits open
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
		quat stance_bend = Identity<quat>();  // The end bone's orientation relative to the limb's middle bone in the family's stance
		float free = 0.0f;                    // In [0,1]. How much a raised foot turns with its limb, from the orientation of 'foot' (0) to 'stance_bend' (1).
		float support = 1.0f;                  // In [0,1]. How much the limb stands on 'foot'. Otherwise it hangs from the body as in the guide pose.
		float reach = 0.0f;                   // The limb's length from its root joint to its contact. Ground rays for the foot reach this far either side of it.
		v4 sole_up = v4::YAxis();             // The unit normal of the sole or palm in the end bone's space, pointing away from the ground. See 'Locomotion::FindSole'.
		v4 grip = v4::Origin();               // The point on the hand that a grasp holds, in the end bone's space. See 'Locomotion::FindSole'.
		float skin = 0.0f;                    // The depth (metres, >= 0) of the skin under the digits' bones on the palm side, see 'Locomotion::FindSole'
		std::vector<int> digits;              // The digit chains attached to the limb's end bone
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
			, m_tail_bones()
			, m_tail_ground()
			, m_ropes(rig.chains.size())
			, m_finish_refresh()
			, m_legs()
			, m_leg_reach(0.0f)
			, m_spine(-1)
			, m_terrain()
			, m_family(0)
			, m_prev_family(0)
			, m_requested(0)
			, m_request_blend_s(1.0f)
			, m_request_pending(false)
			, m_family_weight(1.0f)
			, m_action(-1)
			, m_action_time(0.0f)
			, m_action_weight(0.0f)
			, m_gait_speed(0.0f)
			, m_turn_rate(0.0f)
			, m_look()
			, m_look_weight(0.0f)
			, m_breath()
			, m_breath_phase(0.0f)
			, m_chest_stance(xform::Identity())
			, m_chest_motion(xform::Identity())
			, m_reaches()
			, m_stoop(0.0f)
			, m_torso_radius(0.0f)
			, m_shoulders()
			, m_wave_bones()
			, m_phase(0.0f)
			, m_unsupported_s(0.0f)
			, m_fall(0.0f)
			, m_lean()
			, m_up(v4::ZAxis())
			, m_ground_up(v4::ZAxis())
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
					auto chain = LimbChain(rig, foot.contact);
					if (std::none_of(m_limbs.begin(), m_limbs.end(), [=](LimbState const& l) { return l.chain == chain; }))
					{
						auto contact = LimbContact(rig, rig.chains[chain]);
						m_limbs.push_back(LimbState{ .chain = chain, .contact = contact, .footfall = foot, .reach = ChainLength(chain, rig.rest_model[rig.contacts[contact].bone] * rig.contacts[contact].offset) });
						FindSole(m_limbs.back());
					}
				}
			}
			m_neutral.resize(m_limbs.size());

			// Every arm among the limbs can reach for a grasp
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
			{
				if (rig.chains[m_limbs[i].chain].role == EChainRole::Arm)
					m_reaches.push_back(Reach{ .limb = i, .grasp = Grasp{}, .weight = Fade(), .held = false });
			}

			// Finishing the pose moves the limbs' end bones and digits, and the tails, after the chains are solved. Cache those bones and every
			// descendant of them, so that one pass over just these bones makes the pose's model transforms current again. Parents come before children.
			{
				std::vector<bool> moved(rig.bones.size());
				for (auto const& limb : m_limbs)
					moved[rig.chains[limb.chain].bones.back()] = true;
				for (auto const& chain : rig.chains)
				{
					if (chain.role == EChainRole::Tail)
						moved[chain.bones.front()] = true;
				}
				for (int b = 0, bend = static_cast<int>(rig.bones.size()); b != bend; ++b)
				{
					moved[b] = moved[b] || (rig.bones[b].parent >= 0 && moved[rig.bones[b].parent]);
					if (moved[b])
						m_finish_refresh.push_back(b);
				}
			}
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

			// Grasping elbows stay outside the torso: the capsule around the spine whose radius reaches the nearest arm root in the rest pose
			if (m_spine >= 0 && !m_reaches.empty())
			{
				auto [a, b] = SpineSegment(rig.rest_model);
				m_torso_radius = std::numeric_limits<float>::max();
				for (auto const& reach : m_reaches)
					m_torso_radius = std::min(m_torso_radius, DistanceToSegment(rig.rest_model[rig.chains[m_limbs[reach.limb].chain].bones.front()].pos, a, b));
			}

			// Breathing rolls each shoulder back, where a shoulder is the bone that carries an arm and belongs to no chain. Back is away from the
			// front of the body, which faces along the spine turned a quarter turn forward about model +X. The roll turns the arm's root joint
			// back about the shoulder's head.
			if (m_spine >= 0)
			{
				auto const& spine = rig.chains[m_spine].bones;
				auto back = -Normalise(Cross(v4::XAxis(), rig.rest_model[spine.back()].pos - rig.rest_model[spine.front()].pos));
				for (auto const& chain : rig.chains)
				{
					auto shoulder = chain.role == EChainRole::Arm ? rig.bones[chain.bones.front()].parent : -1;
					if (shoulder < 0 || std::any_of(rig.chains.begin(), rig.chains.end(), [=](Chain const& c) { return std::find(c.bones.begin(), c.bones.end(), shoulder) != c.bones.end(); }))
						continue;

					auto reach = rig.rest_model[chain.bones.front()].pos - rig.rest_model[shoulder].pos;
					auto axis = Normalise(Cross(reach, back));
					m_shoulders.push_back(Shoulder{ .bone = shoulder, .axis = Rotate(~rig.rest_model[shoulder].rot, axis) });
				}
			}

			// The body wave bends the joints along the spine, from the chest end to the pelvis, and on down each tail. A bone's rotation bends the
			// body at its head. The spine's base bone is left out, because turning it would turn the whole body rather than bend it.
			auto spine_length = 0.0f;
			auto tail_length = 0.0f;
			if (m_spine >= 0)
			{
				// Spine bones run from the pelvis to the chest, so a bone's head is the sum of its own and later bone lengths from the chest end.
				// Turning a spine bone turns the head end of the body relative to the tail end, which bends the body the opposite way to a tail bone.
				auto const& spine = rig.chains[m_spine].bones;
				for (auto b : spine)
					spine_length += rig.bones[b].length;

				auto s = spine_length;
				for (size_t k = 0; k != spine.size(); ++k)
				{
					if (k != 0)
						m_wave_bones.push_back(WaveBone{ .bone = spine[k], .s = s, .length = rig.bones[spine[k]].length, .sign = -1.0f });

					s -= rig.bones[spine[k]].length;
				}
			}
			for (auto const& chain : rig.chains)
			{
				// Tail bones run from the base to the tip
				if (chain.role != EChainRole::Tail)
					continue;

				auto s = spine_length;
				for (auto b : chain.bones)
				{
					m_wave_bones.push_back(WaveBone{ .bone = b, .s = s, .length = rig.bones[b].length, .sign = 1.0f });
					s += rig.bones[b].length;
				}
				tail_length = std::max(tail_length, s - spine_length);
			}
			if (auto body_length = spine_length + tail_length; body_length > 0)
			{
				// Distances and lengths are measured in body lengths
				for (auto& bone : m_wave_bones)
				{
					bone.s /= body_length;
					bone.length /= body_length;
				}
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

		// The index of the movement family in 'gait().families' that the character is moving in, or blending to. This is the requested family
		// unless the ground is too steep for it.
		int family() const
		{
			return m_family;
		}

		// The model-space frame of each bone, in rig bone order, when the character holds the family at 'index' in 'gait().families'. For most families
		// this is the stance of the family's slowest cycle, with the hip joints at that cycle's hip height above the model origin. A collapse action
		// family lies that stance on its side on the ground (see the file header).
		std::span<xform const> posture(int index) const
		{
			pr_assert(index >= 0 && index < static_cast<int>(m_families.size()) && "Family index out of range");
			return m_families[index].posture_model;
		}

		// Request the movement family at 'index' in 'gait().families'. Each step blends to it over 'blend_s' seconds (> 0) once the ground allows
		// it, and the same blend time is used when the ground forces a change to another family. A change during a blend starts from the family
		// that was being blended to.
		void RequestFamily(int index, float blend_s)
		{
			pr_assert(index >= 0 && index < static_cast<int>(m_families.size()) && "Family index out of range");
			pr_assert(blend_s > 0 && "Family blend time must be positive");
			m_requested = index;
			m_request_blend_s = blend_s;
			m_request_pending = true;
		}

		// Forget the body's motion history so that the next step starts afresh, as the first step does: every foot plants beneath the body, tails take
		// the guide's shape, and the smoothed speed, turning, ground height and fall start from rest. Use this when the body has moved without being
		// stepped, for example after it was not animated for a while. The family, look, breath and grasps are kept.
		void Restart()
		{
			// The first step after this plants the feet and rebuilds the world-space state from the body
			m_started = false;
			for (auto& rope : m_ropes)
				rope.clear();

			m_terrain = TargetPath();
			m_gait_speed = 0.0f;
			m_turn_rate = 0.0f;
			m_unsupported_s = 0.0f;
			m_fall = 0.0f;
		}

		// True once the latest requested family has been chosen by a step and any blend to it has finished. The chosen family is 'family()', which
		// differs from the request on ground too steep for it.
		bool family_settled() const
		{
			return !m_request_pending && m_family_weight.Finished();
		}

		// Turn the neck chains to face 'world_point', or back to rest if it is empty, over 'fade_s' seconds (> 0). A head that already faces a point
		// moves its gaze smoothly to the new point over the same time.
		void SetLook(std::optional<v4> world_point, float fade_s)
		{
			// Fading out keeps the last point so that the head turns back along the same path. A head at rest starts facing the new point at once,
			// because its weight hides the jump.
			if (world_point && m_look_weight.Value() == 0 && m_look_weight.Finished())
				m_look = TargetPath(xform(*world_point, Identity<quat>()));
			else if (world_point)
				m_look.Retarget(xform(*world_point, Identity<quat>()), fade_s);

			m_look_weight.Start(world_point ? 1.0f : 0.0f, fade_s);
		}

		// Breathe with 'breath' from the next step. The breath continues from where it is, so changes of rate do not jump, but changes of depth
		// show at once; callers that change depth suddenly should ease it.
		void SetBreath(Breath const& breath)
		{
			pr_assert(breath.rate >= 0 && breath.chest >= 0 && breath.shoulders >= 0 && "Breath values must not be negative");
			m_breath = breath;
		}

		// Move the palm of the arm on 'side' to 'grasp', given in world space, or back to the arm's usual pose if it is empty, over 'fade_s' seconds (> 0).
		// The rig must have a walking arm on 'side'. A grasp on something that moves should be set again every step; only a change between holding
		// and letting go starts a new fade. A grasp beyond the arm's reach makes the body stoop (see the file header).
		void SetReach(ESide side, std::optional<Grasp> const& grasp, float fade_s)
		{
			// Find the arm. A hand that lets go keeps its last grasp so that it fades back from where it was.
			pr_assert(fade_s > 0 && "Fade time must be positive");
			auto it = std::find_if(m_reaches.begin(), m_reaches.end(), [&](Reach const& r) { return m_rig.chains[m_limbs[r.limb].chain].side == side; });
			pr_assert(it != m_reaches.end() && "The rig has no walking arm on this side");
			if (grasp)
				it->grasp = *grasp;

			if (it->held != grasp.has_value())
				it->weight.Start(grasp ? 1.0f : 0.0f, fade_s);

			it->held = grasp.has_value();
		}

		// How far the body is stooping for its grasps, in [0,1]
		float stoop() const
		{
			return m_stoop;
		}

		// The model-space transform that moves the chest from its stance to where this step put it: the body's sway, weight shift, lean into turns,
		// pitch over the ground, the cycles' motion, and the breath, but not the stoop for grasps. Applying it to a model-space point held against the
		// still chest moves the point with the chest. It is the identity for a rig without a spine and for a body standing still on level ground without breathing.
		xform const& chest_motion() const
		{
			return m_chest_motion;
		}

		// Advance by 'dt' seconds (>= 0) and pose the rig for 'body' on 'ground'
		void Step(float dt, BodyState const& body, IGround const& ground)
		{
			// Find where each foot would stand beneath the body now. Planting, settling and lifting all compare the feet with these frames.
			m_up = Rotate(body.model_to_world.rot, v4::ZAxis());
			FindNeutralFeet(body.model_to_world, ground);

			// Move in the requested family if the ground beneath the feet allows it, otherwise in the first family that it allows
			ChooseFamily(body.gravity);
			StepAction(dt);

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
			// A swimming body is held up by the water, so it is never airborne.
			auto const& family = m_gait.families[m_family];
			m_unsupported_s = body.supported ? 0.0f : m_unsupported_s + dt;
			auto airborne = family.medium != EFamilyMedium::Water && m_unsupported_s > AirborneDelay;
			m_fall = Clamp(m_fall + (airborne ? dt : -dt) / FallResponse, 0.0f, 1.0f);

			// Find how a righting body leans: world down seen from the body, flattened onto the body's horizontal plane, points at its lowest side, which
			// may be to the side, forward, or back. The limbs reach towards that side by a fraction that grows from 0 at the calm tilt to 1 at the rough
			// tilt. The tilt is measured 'RightingLead' seconds ahead at the body's current rate of turning, so the limbs ease off before the body swings
			// back through upright rather than after, which damps the rocking instead of feeding it.
			m_lean = v4{};
			if (body.rough_tilt > body.calm_tilt && LengthSq(body.gravity) > 0)
			{
				auto down = Rotate(~PredictBody(body, RightingLead).rot, Normalise(body.gravity));
				auto across = v4(down.x, down.y, 0, 0);
				auto tilt = std::atan2(Length(across), -down.z);
				if (LengthSq(across) > Sqr(tiny<float>))
					m_lean = Normalise(across) * Clamp((tilt - body.calm_tilt) / (body.rough_tilt - body.calm_tilt), 0.0f, 1.0f);
			}

			// Measure the speed along the surface that the feet must keep up with, including turning on the spot. The family's slowest cycle sets
			// what counts as moving. A swimmer's strokes cause its motion through the water rather than follow it, and waves move the body without
			// any stroking, so a water family uses the caller's stroke speed instead of the measured velocity. A swimmer must keep stroking to hold
			// its place in the water, so it never strokes slower than its slowest cycle.
			auto const& slowest = family.cycles.front();
			auto tangential = body.velocity - m_up * Dot(body.velocity, m_up);
			auto travel_sq = family.medium == EFamilyMedium::Water ? Sqr(std::max(body.stroke_speed, slowest.speed)) : LengthSq(tangential);
			auto turn = Dot(body.angular_velocity, m_up) * TurnRadius();
			auto speed = Sqrt(travel_sq + Sqr(turn));
			auto moving = !airborne && speed >= StandSpeedFraction * slowest.speed;

			// The cycle blend and the body motion follow a smoothed speed, so that footfall timing and body motion change gradually
			m_gait_speed += ((moving ? speed : 0.0f) - m_gait_speed) * std::min(dt / BodyResponse, 1.0f);
			m_turn_rate += (Dot(body.angular_velocity, m_up) - m_turn_rate) * std::min(dt / BodyResponse, 1.0f);

			// Blend the limbs' support and timing towards the chosen family
			m_family_weight.Step(dt);
			UpdateLimbs();

			// Advance the cycle. A standing body that is out of place steps at the slowest cycle's cadence.
			auto cadence = airborne ? 0.0f : moving ? speed / Stride(family, speed) : NeedsSettling() ? slowest.speed / slowest.stride : 0.0f;
			auto dphase = cadence * dt;
			m_phase = Frac(m_phase + dphase);

			// Advance each limb's stance and swing, collecting the landings that swinging feet still need to aim at. A collapsing body's feet stay
			// where they are, because the body no longer stands on them.
			m_landings.clear();
			m_probe_rays.clear();
			auto collapse = CollapseProgress();
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend && collapse == 0; ++i)
				AdvanceLimb(i, body, airborne, moving, cadence, dphase);

			// Aim the swinging feet at the ground beneath their predicted landings
			auto hits = ProbeGround(ground);
			for (int k = 0, kend = static_cast<int>(m_landings.size()); k != kend; ++k)
			{
				auto& limb = m_limbs[m_landings[k].limb];
				limb.land = FootOnGround(limb, m_probe_rays[k], hits[k], m_landings[k].body_rot);
			}

			// Move each foot along its swing, then find the ground beneath it. Feet swing outward, away from the body's midline, which is the
			// model's X = 0 plane.
			for (auto& limb : m_limbs)
			{
				auto side = limb.stance_contact.x > 0 ? 1.0f : limb.stance_contact.x < 0 ? -1.0f : 0.0f;
				MoveSwingingFoot(limb, limb.footfall.swing_height * m_up, Rotate(body.model_to_world.rot, v4(side * limb.footfall.swing_out, 0, 0, 0)));
				limb.ground = GroundUnder(limb);
			}

			// Build the guide pose. Its motion grows with speed up to the slowest cycle's speed, so a standing character holds the stance.
			m_breath_phase = Frac(m_breath_phase + m_breath.rate * dt);
			BuildGuide(std::min(m_gait_speed / slowest.speed, 1.0f));
			PoseTargets(m_rig, m_guide.model, m_guide_targets);
			std::copy(m_guide_targets.begin(), m_guide_targets.end(), m_targets.begin());

			// Pose the body, then the chains that hang from it. A limb's goal moves from its hanging frame to its foot as it takes weight.
			auto world_to_model = Invert(body.model_to_world);
			for (auto& reach : m_reaches)
				reach.weight.Step(dt);

			StepSpine(dt, world_to_model);
			DriveTorso();
			for (auto const& limb : m_limbs)
			{
				auto foot = world_to_model * limb.foot;
				m_targets[limb.chain].frame = limb.support == 1.0f ? foot : BlendFrame(HangingFrame(limb), foot, limb.support);
			}

			// Grasping hands move from their usual goals to their grasps
			for (auto const& reach : m_reaches)
			{
				auto weight = reach.weight.Value();
				if (weight == 0)
					continue;

				auto const& limb = m_limbs[reach.limb];
				auto& frame = m_targets[limb.chain].frame;
				frame = BlendFrame(frame, GraspFrame(limb, reach.grasp, world_to_model), weight);
			}
			AimPunches();
			StepNecks(dt, world_to_model);

			// Bones that no chain solver poses, such as digits and tails, take their shape from the guide. The guide's model transforms are
			// already current for its local transforms, so the pose starts from both.
			std::copy(m_guide.local.begin(), m_guide.local.end(), m_pose.local.begin());
			std::copy(m_guide.model.begin(), m_guide.model.end(), m_pose.model.begin());
			SolvePose(m_rig, m_targets, m_pose);
			ClearElbows();
			TurnRaisedFeet();
			PlantDigits();
			CurlDigits();
			ClenchFists();
			StepTails(body.model_to_world, world_to_model, ground);

			// Bring the bones moved since the solve, and their descendants, up to date. A collapse moves every bone.
			if (collapse == 0)
				m_pose.UpdateModel(m_rig, m_finish_refresh);
			else
				Collapse(collapse);
		}

	private:

		// The fraction of the slowest cycle's speed below which the body is standing
		static constexpr float StandSpeedFraction = 0.1f;

		// How long, in seconds, the body must be continuously unsupported before its feet leave the ground
		static constexpr float AirborneDelay = 0.1f;

		// The fraction of the slowest cycle's stride that a planted foot may be from its neutral position before a standing body steps
		static constexpr float SettleFraction = 0.15f;

		// The angle (radians) that a planted foot may be turned from its neutral orientation before a standing body steps
		static constexpr float SettleTwist = 0.2f;

		// How far a planted foot may drift from its neutral position, as a fraction of the current stride, or turn from its neutral orientation
		// (radians), before it steps out of turn, and the largest fraction of the supporting limbs that may be swinging when it does
		static constexpr float StrainFraction = 0.75f;
		static constexpr float StrainTwist = 0.5f;
		static constexpr float MaxEarlySwingFraction = 0.5f;

		// The seconds that the body takes to follow changes of ground height and speed
		static constexpr float BodyResponse = 0.2f;

		// The seconds that the limbs take to spread out once airborne, or to fold back in on landing
		static constexpr float FallResponse = 0.3f;

		// The furthest that a swimmer's limb ends reach towards its lowest side, as a fraction of each limb's length, and the seconds ahead at which its
		// lean is measured, see 'RightingOffset'
		static constexpr float RightingReach = 0.5f;
		static constexpr float RightingLead = 0.3f;

		// How a full stoop moves the body: the pelvis sinks and moves back (+Y) by these fractions of the legs' reach, and the spine turns forward about
		// the pelvis by 'StoopPitch' radians, the pelvis itself turning by 'StoopPelvisShare' of that so that the hips bend. See 'Stooped'.
		static constexpr float StoopDrop = 0.45f;
		static constexpr float StoopBack = 0.2f;
		static constexpr float StoopPitch = 1.0f;
		static constexpr float StoopPelvisShare = 0.5f;

		// The fraction of an arm's reach within which a body stoops to bring its grasp, the seconds that the stoop takes to follow what the grasps need,
		// and the number of even steps, then halvings, used to find the least stoop that brings a grasp within reach
		static constexpr float StoopReach = 0.95f;
		static constexpr float StoopResponse = 0.3f;
		static constexpr int StoopSamples = 8;
		static constexpr int StoopBisections = 8;

		// How far a grasping elbow flares out from the body, as the length of the outward push added to the arm's unit pole at full grasp, and the step
		// (radians) and number of steps either way around the arm's reach that are searched for an elbow outside the torso, see 'ClearElbows'
		static constexpr float ElbowFlare = 0.5f;
		static constexpr float ElbowStep = 0.08f;
		static constexpr int ElbowSteps = 39;

		// The largest angle (radians) by which a grasping digit bone curls towards the palm, see 'CurlDigits'
		static constexpr float MaxCurl = 1.4f;

		// The seconds of turning by which the chest leads into a turn and the pelvis trails it, and the largest angle (radians) of each
		static constexpr float SpineLeadTime = 0.25f;
		static constexpr float MaxSpineLead = 0.5f;

		// The fraction of the swing height above which a raised foot turns fully with its limb. Below it, the foot turns towards the ground's orientation.
		static constexpr float FreeFootHeight = 0.5f;

		// The largest angle (radians) by which each tail joint bends away from the bone before it, so that a trailing tail curves smoothly to the ground
		static constexpr float TailBend = 0.35f;

		// The number of steps over the first tail bone's swing range that are checked for its droop limits
		static constexpr int TailSwingSamples = 32;

		// The number of halvings used to find each tail bone's steepest droop that stays clear of the ground
		static constexpr int TailBisections = 16;

		// The angle (radians) by which the ground must be less steep than a family's slope limit before the character changes to that family,
		// so that ground close to the limit does not switch families back and forth
		static constexpr float SlopeMargin = 0.05f;

		// How a collapse moves the body over its progress in [0,1] (see 'Collapse'):
		//  - The knees buckle as the body stoops by up to 'CollapseStoop' (see 'Stooped'), until 'CollapseBuckleEnd'.
		//  - The body topples sideways from 'CollapseToppleStart' and lies flat by 'CollapseToppleEnd', falling faster as it goes, like a falling pole.
		//  - The joints relax from 'CollapseRelaxStart', ending 'CollapseRelax' of the way from the solved pose to the stance.
		//  - The joint limits apply fully from 'CollapseLimitEnd', so that a standing body is not changed at the start.
		//  - The limbs and head sag under gravity from 'CollapseRelaxStart', until they rest on the ground or reach their joint limits (see 'Sag').
		static constexpr float CollapseStoop = 0.6f;
		static constexpr float CollapseBuckleEnd = 0.5f;
		static constexpr float CollapseToppleStart = 0.3f;
		static constexpr float CollapseToppleEnd = 0.85f;
		static constexpr float CollapseRelaxStart = 0.55f;
		static constexpr float CollapseRelax = 0.6f;
		static constexpr float CollapseLimitEnd = 0.2f;

		// The distance that a boxing stance moves each foot forward (left) or back (right), as a fraction of the legs' reach
		static constexpr float BoxingStagger = 0.12f;

		// The angle (radians) about model +Z by which the torso turns its left shoulder forward when on guard, and the fractions of each punch at which
		// the fist is fully out and then back on guard
		static constexpr float GuardYaw = 0.25f;
		static constexpr float PunchOut = 0.35f;
		static constexpr float PunchBack = 0.75f;

		// The fraction of the chest's punch turn that the pelvis also turns
		static constexpr float PunchPelvisShare = 0.4f;

		// The angle (radians) by which the torso of a body on guard leans forward about the pelvis, and the distance that the hips drop, as a fraction
		// of the spine's length, so that the body crouches behind its guard
		static constexpr float GuardLean = 0.12f;
		static constexpr float GuardCrouch = 0.1f;

		// The fraction of the chest's forward travel, as the torso leans into a punch, by which the hips also move forward over the front foot
		static constexpr float PunchHipDrive = 0.4f;

		// The angle (radians) by which each bone of a punching hand's digits curls towards the palm to make a fist
		static constexpr float FistCurl = 1.2f;

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
			std::vector<std::vector<xform>> stance_local; // Per cycle, per bone, the stance transform relative to the parent bone
			std::vector<xform> stance_model;             // Per bone, the slowest cycle's stance transform in model space
			std::vector<xform> posture_model;            // Per bone, the model-space transform that the family holds, see 'posture'
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

		// An arm that can reach for a grasp
		struct Reach
		{
			int limb;      // Index into 'm_limbs'
			Grasp grasp;   // The latest grasp, in world space
			Fade weight;   // How strongly the hand holds 'grasp'
			bool held;     // True while the caller asks the hand to hold 'grasp'
		};

		// A bone that breathing rolls back
		struct Shoulder
		{
			int bone;
			v4 axis;       // The unit axis, in the bone's own frame, about which a positive angle rolls the shoulder back
		};

		// A bone that the body wave bends
		struct WaveBone
		{
			int bone;
			float s;       // The distance of the bone's head from the head end of the spine, in body lengths
			float length;  // The bone's length, in body lengths
			float sign;    // +1 if turning the bone bends the body the way that a tail bone does, -1 for the opposite way
		};

		// A tail joint whose height is checked against the ground
		struct TailJoint
		{
			int chain;     // The tail's chain index
			int joint;     // The joint's index within the chain, where the chain's base is 0
		};

		// One bone of a tail being planned over the ground
		struct TailBone
		{
			v4 across;     // The bone's unit heading across the ground (world space)
			float length;  // The bone's length
			float along;   // The distance along the bone from its head to the skin point that must stay above the ground
			float depth;   // The distance of that skin point below the line of the bone
			float ground;  // The droop of the bone when the skin beneath both of its joints rests on the ground. The tail flattens no further than this.
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
		std::vector<TailBone> m_tail_bones;              // The bones of the tail being planned (scratch)
		std::vector<float> m_tail_ground;                // The ground height beneath each joint of the tail being planned (scratch)
		std::vector<std::vector<v4>> m_ropes; // Per chain, the world-space tail joint positions from the previous step. Empty for other chains, and until a tail is first posed.
		std::vector<int> m_finish_refresh;    // The bones, in bone order, whose model transforms can be stale after the pose is finished
		std::vector<int> m_legs;    // Indices into 'm_limbs' of the limbs that carry the hips at the cycles' hip height
		float m_leg_reach;          // The mean reach of 'm_legs', which scales the cycles' hip height
		int m_spine;                // The spine chain index, or -1 if the rig has none
		TargetPath m_terrain;       // The smoothed ground height under the hind feet (pos.x) and fore feet (pos.y), in model space
		int m_family;               // The family being moved in, or blended to
		int m_prev_family;          // The family being blended from
		int m_requested;            // The family that the caller wants to move in
		float m_request_blend_s;    // The seconds over which a change of family blends
		bool m_request_pending;     // True from 'RequestFamily' until the next step has chosen a family
		Fade m_family_weight;       // From the previous family (0) to the current family (1)
		int m_action;               // The action family whose action is playing or fading out, or -1 if none
		float m_action_time;        // Collapse: the seconds into the collapse. Punches: the seconds into the punch sequence.
		float m_action_weight;      // In [0,1]. Collapse: how far through the collapse the body is. Punches: how strongly the punches show.
		float m_gait_speed;         // The smoothed speed that selects cycles and scales body motion
		float m_turn_rate;          // The smoothed rate of turning about the body's up direction, in radians per second
		TargetPath m_look;          // The world-space look point (pos), which moves smoothly to each new point
		Fade m_look_weight;         // How strongly the necks face 'm_look'
		Breath m_breath;            // The caller's breathing motion
		float m_breath_phase;       // The breath cycle position in [0,1)
		xform m_chest_stance;       // The chest bone's model-space frame in this step's stance, before any motion
		xform m_chest_motion;       // The model-space move of the chest from 'm_chest_stance' to this step's chest target, see 'chest_motion'
		std::vector<Reach> m_reaches; // The walking arms, which can reach for grasps
		float m_stoop;              // In [0,1]. How far the body stoops for its grasps, see 'Stooped'.
		float m_torso_radius;       // The radius of the capsule around the spine that grasping elbows stay outside, or 0 if there is none, see 'ClearElbows'
		std::vector<Shoulder> m_shoulders; // The bones that breathing rolls back
		std::vector<WaveBone> m_wave_bones; // The bones that the body wave bends
		float m_phase;               // The step cycle position in [0,1)
		float m_unsupported_s;      // How long the body has been continuously unsupported, in seconds
		float m_fall;               // In [0,1]. How far the limbs are spread out for falling.
		v4 m_lean;                  // The model-space direction (z = 0) of a righting swimmer's lowest side, with length in [0,1] for how far its limbs reach that way, see 'RightingOffset'
		v4 m_up;                    // The body's up direction (model +Z) in world space this step
		v4 m_ground_up;             // The mean unit normal of the ground last found beneath the feet, in world space
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
			FamilyData data;
			for (auto const& cycle : family.cycles)
			{
				// The stance starts from the rest pose. Listed bones replace their rotation, and root bones may also replace their position.
				auto& stance = data.stance_local.emplace_back();
				for (auto const& bone : m_rig.bones)
					stance.push_back(bone.rest);

				for (auto const& bone : cycle.stance)
				{
					auto& local = stance[m_rig.BoneIndex(bone.bone)];
					local.rot = bone.rotation;
					if (bone.position)
						local.pos = *bone.position;
				}

				// Raise or lower the root bones so that every stance carries the hip joints at the slowest cycle's hip height. The authored height
				// of the hips is ignored because the body's height should follow the legs, not the pose that the cycles happened to be made in.
				// 'HipLift' adds the change in hip height between the cycles.
				if (!m_legs.empty())
				{
					auto model = ModelFrames(stance);
					auto hip_z = 0.0f;
					for (auto i : m_legs)
						hip_z += model[m_rig.chains[m_limbs[i].chain].bones.front()].pos.z / static_cast<float>(m_legs.size());

					auto lift = family.cycles.front().hip_height * m_leg_reach - hip_z;
					for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
					{
						if (m_rig.bones[b].parent < 0)
							stance[b].pos.z += lift;
					}
				}

				// Each cycle's bone motions
				auto& tracks = data.tracks.emplace_back();
				for (auto const& motion : cycle.motion)
					tracks.push_back(BoneTrack{ .bone = m_rig.BoneIndex(motion.bone), .channel = motion.channel, .harmonics = motion.harmonics });
			}

			// The slowest cycle's stance places the limbs' neutral contacts, because a standing body holds it
			data.stance_model = ModelFrames(data.stance_local.front());

			// A collapsed body holds the stance lying on its side
			data.posture_model = data.stance_model;
			if (family.action && family.action->kind == EActionKind::Collapse)
				Topple(data.posture_model, math::constants<float>::tau_by_4);

			// Map the controller's limbs to the family's footfalls
			auto const& footfalls = family.cycles[0].footfalls;
			for (auto const& limb : m_limbs)
			{
				auto it = std::find_if(footfalls.begin(), footfalls.end(), [&](Footfall const& f) { return LimbChain(m_rig, f.contact) == limb.chain; });
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

		// Set each limb's stance contact and orientation from the stance of family 'f'. A punches action family stands in a boxing stance, with the
		// left foot forward and the right foot back.
		void UseStance(int f)
		{
			// Contacts stand on the ground, so only their position across the ground plane is kept
			auto const& model = m_families[f].stance_model;
			auto const& action = m_gait.families[f].action;
			auto boxing = action && action->kind == EActionKind::Punches;
			for (auto& limb : m_limbs)
			{
				auto const& contact = m_rig.contacts[limb.contact];
				limb.stance_contact = model[contact.bone] * contact.offset;
				limb.stance_contact.z = 0.0f;
				limb.stance_rot = model[m_rig.chains[limb.chain].bones.back()].rot;
				limb.stance_bend = ~model[m_rig.chains[limb.chain].bones[1]].rot * limb.stance_rot;

				// Forward is model -Y and left is +X
				if (boxing && m_rig.chains[limb.chain].role == EChainRole::Leg)
					limb.stance_contact.y -= SideSign(m_rig.chains[limb.chain].side) * BoxingStagger * m_leg_reach;
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

		// The model-space height by which family 'f' raises its hips above its stance at the current speed
		float HipLift(int f) const
		{
			// The stance already carries the hips at the slowest cycle's hip height
			auto const& family = m_gait.families[f];
			auto b = Bracket(family, m_gait_speed);
			auto hip_height = Lerp(family.cycles[b.lo].hip_height, family.cycles[b.hi].hip_height, b.t);
			return (hip_height - family.cycles.front().hip_height) * m_leg_reach;
		}

		// The value of a cycle parameter of family 'f' at the current speed
		float CycleValue(int f, float GaitCycle::* value) const
		{
			// Blend between the cycles either side of the smoothed speed
			auto const& family = m_gait.families[f];
			auto b = Bracket(family, m_gait_speed);
			return Lerp(family.cycles[b.lo].*value, family.cycles[b.hi].*value, b.t);
		}

		// The size of a cycle's walking motion, such as its spine sway, blended between the previous and current families. It grows with speed up to
		// the slowest cycle's speed, like the guide's motion, so that a standing body holds still.
		float WalkAmplitude(float GaitCycle::* value) const
		{
			// Blend the families, then scale by how fast the body walks
			auto w = m_family_weight.Value();
			auto slowest = m_gait.families[m_family].cycles.front().speed;
			return Lerp(CycleValue(m_prev_family, value), CycleValue(m_family, value), w) * std::min(m_gait_speed / slowest, 1.0f);
		}

		// Blend each limb's support between the previous and current families, and take its timing and swing from the family that it walks in
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

				// Blend the timing and swing between the cycles either side of the speed. Touchdowns blend the short way around the cycle.
				auto const& family = m_gait.families[f];
				auto index = m_families[f].limb_footfall[i];
				auto b = Bracket(family, m_gait_speed);
				auto const& lo = family.cycles[b.lo].footfalls[index];
				auto const& hi = family.cycles[b.hi].footfalls[index];
				auto shift = Frac(hi.touchdown - lo.touchdown + 0.5f) - 0.5f;
				limb.footfall.touchdown = Frac(lo.touchdown + shift * b.t);
				limb.footfall.duty = Lerp(lo.duty, hi.duty, b.t);
				limb.footfall.swing_height = Lerp(lo.swing_height, hi.swing_height, b.t);
				limb.footfall.swing_out = Lerp(lo.swing_out, hi.swing_out, b.t);
			}
		}

		// Set 'm_guide' to the blended stance plus the cycles' body motion, scaled by 'amplitude'
		void BuildGuide(float amplitude)
		{
			// Blend each family's stance between its cycles either side of the speed, then blend the previous and current families
			auto w = m_family_weight.Value();
			auto prev = StanceBlend(m_prev_family);
			auto curr = StanceBlend(m_family);
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
				m_guide.local[b] = w == 1.0f ? curr(b) : BlendFrame(prev(b), curr(b), w);

			// Sum the motion of both families, each blended between its cycles either side of the speed
			std::fill(m_rot_offset.begin(), m_rot_offset.end(), v4::Zero());
			std::fill(m_pos_offset.begin(), m_pos_offset.end(), v4::Zero());
			AddMotion(m_prev_family, (1.0f - w) * amplitude);
			AddMotion(m_family, w * amplitude);
			for (auto const& shoulder : m_shoulders)
				m_rot_offset[shoulder.bone] += shoulder.axis * (m_breath.shoulders * Inhale() * Breathing());

			// Root bones carry the hips at the hip height of the cycles either side of the speed
			auto lift = v4(0, 0, Lerp(HipLift(m_prev_family), HipLift(m_family), w), 0);
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
			{
				if (m_rig.bones[b].parent < 0)
					m_pos_offset[b] += lift;
			}

			// Record the chest's frame in the stance at this hip height, before any motion, so 'chest_motion' can measure the chest's movement from it.
			// Parents come before their children, so the frame builds from the chest bone up to its root.
			if (m_spine >= 0)
			{
				auto b = m_rig.chains[m_spine].bones.back();
				m_chest_stance = m_guide.local[b];
				for (; m_rig.bones[b].parent >= 0; b = m_rig.bones[b].parent)
					m_chest_stance = m_guide.local[m_rig.bones[b].parent] * m_chest_stance;

				m_chest_stance.pos += lift;
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

		// A function that returns the local stance frame of a bone in family 'f', blended between the cycles either side of the current speed
		auto StanceBlend(int f) const
		{
			// The bracketing cycles are the same cycle outside the family's speed range
			auto b = Bracket(m_gait.families[f], m_gait_speed);
			auto const& lo = m_families[f].stance_local[b.lo];
			auto const& hi = m_families[f].stance_local[b.hi];
			return [&lo, &hi, t = b.t](int bone)
			{
				return t == 0 ? lo[bone] : BlendFrame(lo[bone], hi[bone], t);
			};
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
			AddWave(f, weight);
		}

		// Add the body wave of family 'f' at the current phase, scaled by 'weight', to the rotation offsets. See the top of gait.h.
		void AddWave(int f, float weight)
		{
			// Each bone bends about the body's up axis (model +Z) in the family's stance, expressed in the bone's own frame
			auto const& wave = m_gait.families[f].wave;
			if (weight == 0 || wave.amplitude == 0)
				return;

			auto const& model = m_families[f].stance_model;
			for (auto const& bone : m_wave_bones)
			{
				auto amplitude = wave.amplitude * Lerp(1.0f, wave.growth, bone.s) * bone.length;
				auto angle = bone.sign * amplitude * std::sin(math::constants<float>::tau * (m_phase - bone.s / wave.wavelength));
				m_rot_offset[bone.bone] += Rotate(~model[bone.bone].rot, v4::ZAxis()) * (angle * weight);
			}
		}

		// How much the body swims, blended between the previous family (0 or 1) and the current family (0 or 1)
		float WaterWeight() const
		{
			auto water = [&](int f) { return m_gait.families[f].medium == EFamilyMedium::Water ? 1.0f : 0.0f; };
			return Lerp(water(m_prev_family), water(m_family), m_family_weight.Value());
		}

		// The end bone frame of a hanging limb: its frame in the guide pose, carried by the body. Legs hang from the pelvis and arms from the chest.
		// Hanging arms also swing with the walk (see 'ArmSwing').
		xform HangingFrame(LimbState const& limb) const
		{
			// Swing the guide frame about the limb's root joint
			auto const& bones = m_rig.chains[limb.chain].bones;
			auto guide_end = m_guide.model[bones.back()];
			if (m_rig.chains[limb.chain].role == EChainRole::Arm)
			{
				auto root = m_guide.model[bones.front()].pos;
				auto swing = quat(v4::XAxis(), ArmSwing(limb));
				guide_end = xform(root + Rotate(swing, guide_end.pos - root), Normalise(swing * guide_end.rot));
			}

			// A swimmer that leans moves each limb end towards its lowest side. The limbs on the low side reach out, where the water lifts them
			// furthest from the body, and the limbs on the high side fold in, so the limbs' buoyancy turns the body back upright. The chain solver
			// keeps an end that moves out of reach at full stretch.
			guide_end.pos += RightingOffset(limb);

			// Without a spine the body does not move relative to the guide
			if (m_spine < 0)
				return guide_end;

			auto leg = m_rig.chains[limb.chain].role == EChainRole::Leg;
			auto const& actual = leg ? m_targets[m_spine].base : m_targets[m_spine].frame;
			auto const& guide = leg ? m_guide_targets[m_spine].base : m_guide_targets[m_spine].frame;
			return actual * Invert(guide) * guide_end;
		}

		// The end bone frame (model space) that puts the grip point of 'limb' on the grasp point, with the palm facing the grasp's direction and the
		// fingers pointing as near to the grasp's finger direction as the palm allows. The grasp sets the whole orientation, so the hand does not
		// follow its usual goal's swing. The fingers run along the end bone (+Y), and the palm faces away from its 'sole_up' normal.
		xform GraspFrame(LimbState const& limb, Grasp const& grasp, xform const& world_to_model) const
		{
			// Turn the bone's palm direction onto the grasp's palm direction
			auto palm = -limb.sole_up;
			auto facing = Rotate(world_to_model.rot, grasp.palm);
			auto turn = quat(palm, facing);

			// Then twist about the palm direction so the fingers point along the part of the grasp's finger direction that lies across the palm
			auto fingers = Rotate(turn, Normalise(v4::YAxis() - palm * Dot(v4::YAxis(), palm)));
			auto aim = Rotate(world_to_model.rot, grasp.fingers);
			aim = Normalise(aim - facing * Dot(aim, facing));
			auto twist = quat(facing, std::atan2(Dot(Cross(fingers, aim), facing), Dot(fingers, aim)));
			auto rot = Normalise(twist * turn);

			// Place the bone so that its grip point is on the grasp point
			auto point = world_to_model * grasp.point;
			return xform(point - Rotate(rot, limb.grip.w0()), rot);
		}

		// The pelvis and chest frames of the spine after stooping by 's' in [0,1], from its 'pelvis' and 'chest' frames when upright. The pelvis sinks and
		// moves back, and the spine turns forward about it, so planted legs bend at the knees and the shoulders move down and forward.
		std::pair<xform, xform> Stooped(float s, xform const& pelvis, xform const& chest) const
		{
			// Turning about model +X by a positive angle moves an upright spine forward (-Y)
			auto shift = v4(0, StoopBack * s, -StoopDrop * s, 0) * m_leg_reach;
			auto pitch = quat(v4::XAxis(), StoopPitch * s);
			auto hips = quat(v4::XAxis(), StoopPelvisShare * StoopPitch * s);
			auto base = xform(pelvis.pos + shift, Normalise(hips * pelvis.rot));
			auto frame = xform(base.pos + Rotate(pitch, chest.pos - pelvis.pos), Normalise(pitch * chest.rot));
			return { base, frame };
		}

		// The stoop, in [0,1], that the grasping hands need from the spine's upright targets this step. Each hand needs the least stoop that brings its
		// grasp within 'StoopReach' of its arm's reach from the arm's root, or the stoop that brings it closest if none does, scaled by how strongly
		// the hand holds its grasp. The body stoops as far as the hand that needs most.
		float StoopNeeded(xform const& world_to_model) const
		{
			// The arm roots move with the chest, like hanging arms (see 'HangingFrame')
			if (m_spine < 0)
				return 0.0f;

			auto const& pelvis = m_targets[m_spine].base;
			auto const& chest = m_targets[m_spine].frame;
			auto chest_from_guide = Invert(m_guide_targets[m_spine].frame);
			auto need = 0.0f;
			for (auto const& reach : m_reaches)
			{
				// Measure how far beyond reach the grasp is at a stoop of 's'
				auto weight = reach.weight.Value();
				if (weight == 0)
					continue;

				auto const& limb = m_limbs[reach.limb];
				auto root = chest_from_guide * m_guide.model[m_rig.chains[limb.chain].bones.front()].pos;
				auto point = world_to_model * reach.grasp.point;
				auto excess = [&](float s)
				{
					// Carry the root with the stooped chest
					auto stooped = Stooped(s, pelvis, chest).second;
					return Length(stooped * root - point) - StoopReach * limb.reach;
				};
				if (excess(0.0f) <= 0)
					continue;

				// Step through the stoops evenly for the first within reach, keeping the closest in case none is
				auto lo = 0.0f, hi = 1.0f, best = 0.0f, best_excess = excess(0.0f);
				auto found = false;
				for (int k = 1; k <= StoopSamples && !found; ++k)
				{
					auto s = static_cast<float>(k) / StoopSamples;
					auto e = excess(s);
					found = e <= 0;
					if (found)
						hi = s;
					else
						lo = s;

					if (e < best_excess)
					{
						best = s;
						best_excess = e;
					}
				}

				// Narrow the step that comes within reach down to its least stoop
				if (found)
				{
					for (int k = 0; k != StoopBisections; ++k)
					{
						auto mid = 0.5f * (lo + hi);
						if (excess(mid) <= 0)
							hi = mid;
						else
							lo = mid;
					}
					best = hi;
				}
				need = std::max(need, weight * best);
			}
			return need;
		}

		// The model-space offset of a hanging limb's end towards the body's lowest side: up to 'RightingReach' of the limb's length as the body tilts
		// from its calm tilt to its rough tilt (see 'BodyState'), scaled by how much it swims. Zero when the body does not right itself.
		v4 RightingOffset(LimbState const& limb) const
		{
			// Scale the reach by the limb's own length so that every limb reaches in proportion to its size
			auto length = 0.0f;
			for (auto bone : m_rig.chains[limb.chain].bones)
				length += m_rig.bones[bone].length;

			return m_lean * (RightingReach * length * WaterWeight());
		}

		// The angle (radians, about model +X) by which a hanging arm swings: the cycles' arm swing times cos(2 pi (phase - touchdown)) of the
		// walking leg on the same side. A positive angle moves the hand backwards (+Y), so the arm is furthest back as that leg's foot lands
		// and swings forward as the leg swings back. Arms with no walking leg on their side do not swing.
		float ArmSwing(LimbState const& arm) const
		{
			// Follow the most supporting leg on the arm's side
			auto side = arm.stance_contact.x < 0;
			LimbState const* leg = nullptr;
			for (auto const& limb : m_limbs)
			{
				if (m_rig.chains[limb.chain].role != EChainRole::Leg || (limb.stance_contact.x < 0) != side || limb.support == 0)
					continue;
				if (leg == nullptr || limb.support > leg->support)
					leg = &limb;
			}
			if (leg == nullptr)
				return 0.0f;

			// The swing grows with speed, like the guide's motion
			return WalkAmplitude(&GaitCycle::arm_swing) * leg->support * std::cos(math::constants<float>::tau * (m_phase - leg->footfall.touchdown));
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

		// The end bone frame that puts the limb's contact on the ground at 'hit', with the sole flat on the ground and turned like the stance's
		// orientation relative to the body. A ray that found no ground leaves the foot reaching to the end of the ray.
		xform FootOnGround(LimbState const& limb, GroundRay const& ray, GroundHit const& hit, quat const& body_rot) const
		{
			// Turn the stance orientation by the shortest arc that lays the sole on the ground, which keeps the stance's heading
			auto const& contact = m_rig.contacts[limb.contact];
			auto point = hit.hit ? hit.point : ray.origin + ray.direction * ray.length;
			auto normal = hit.hit ? hit.normal : -ray.direction;
			auto stance = body_rot * limb.stance_rot;
			auto rot = Normalise(quat(Rotate(stance, limb.sole_up), normal) * stance);
			return xform(point - Rotate(rot, contact.offset.w0()), rot);
		}

		// Set the limb's digit chains, sole normal and grip point from the rig's rest pose
		void FindSole(LimbState& limb) const
		{
			// Gather the contacts in the end bone's space. Digit chains hang directly from the end bone.
			auto end = m_rig.chains[limb.chain].bones.back();
			auto to_end = Invert(m_rig.rest_model[end]);
			std::vector<v4> points = { m_rig.contacts[limb.contact].offset };
			for (int c = 0, cend = static_cast<int>(m_rig.chains.size()); c != cend; ++c)
			{
				auto const& chain = m_rig.chains[c];
				if (chain.role != EChainRole::Digit || m_rig.bones[chain.bones.front()].parent != end)
					continue;

				limb.digits.push_back(c);
				if (auto tip = DigitContact(c); tip >= 0)
					points.push_back(to_end * m_rig.rest_model[chain.bones.back()] * m_rig.contacts[tip].offset);
			}
			limb.sole_up = SoleNormal(limb, points);

			// A grasp holds the end bone's authored grip contact, which lies on the skin that wraps around a held thing. A limb without one grips at its contact.
			limb.grip = m_rig.contacts[limb.contact].offset;
			for (auto const& contact : m_rig.contacts)
			{
				// The first grip contact on the end bone is used
				if (contact.kind != EContactKind::Grip || contact.bone != end)
					continue;

				limb.grip = contact.offset;
				break;
			}

			// The skin depth is how far the grip point, which is on the palm's skin, lies on the palm side of the base of the digit nearest it
			auto nearest = std::numeric_limits<float>::max();
			for (auto c : limb.digits)
			{
				auto base = to_end * m_rig.rest_model[m_rig.chains[c].bones.front()].pos;
				if (auto d = LengthSq(base - limb.grip); d < nearest)
				{
					nearest = d;
					limb.skin = std::max(Dot(limb.grip - base, -limb.sole_up), 0.0f);
				}
			}
		}

		// The unit normal of the sole of 'limb' in the end bone's space, pointing away from the ground. The sole is the plane through 'points', which
		// are the limb's contact and the contacts at the tips of its digits. Fewer than three points, or points in a line, use the contact's normal instead.
		v4 SoleNormal(LimbState const& limb, std::span<v4 const> points) const
		{
			// Fit a plane by least squares. Its normal is the direction in which the points spread least: the cross product of the two
			// directions in which they spread most, taken from whichever pair of axes gives the best conditioned answer.
			auto centre = v4::Zero();
			for (auto p : points)
				centre += p.w0();

			centre /= static_cast<float>(points.size());
			auto xx = 0.0f, xy = 0.0f, xz = 0.0f, yy = 0.0f, yz = 0.0f, zz = 0.0f;
			for (auto p : points)
			{
				// Accumulate the spread about the centre
				auto r = p.w0() - centre;
				xx += r.x * r.x; xy += r.x * r.y; xz += r.x * r.z;
				yy += r.y * r.y; yz += r.y * r.z; zz += r.z * r.z;
			}
			auto det_x = yy * zz - yz * yz;
			auto det_y = xx * zz - xz * xz;
			auto det_z = xx * yy - xy * xy;
			auto det_max = std::max({ det_x, det_y, det_z });
			if (points.size() < 3 || det_max <= 1e-6f * Sqr(xx + yy + zz))
			{
				// The contact's normal points out of the skin, towards the ground
				return -m_rig.contacts[limb.contact].normal;
			}
			auto normal =
				det_max == det_x ? v4(det_x, xz * yz - xy * zz, xy * yz - xz * yy, 0) :
				det_max == det_y ? v4(xz * yz - xy * zz, det_y, xy * xz - yz * xx, 0) :
				v4(xy * yz - xz * yy, xy * xz - yz * xx, det_z, 0);

			// The end bone is above its sole, so the normal points from the plane towards the bone's origin
			normal = Normalise(normal);
			return Dot(normal, centre) > 0 ? -normal : normal;
		}

		// The contact on the last bone of digit chain 'c', or -1 if it has none
		int DigitContact(int c) const
		{
			// The first contact found on the bone is used
			auto tip = m_rig.chains[c].bones.back();
			for (int k = 0, kend = static_cast<int>(m_rig.contacts.size()); k != kend; ++k)
			{
				if (m_rig.contacts[k].bone == tip)
					return k;
			}
			return -1;
		}

		// Find the foot frame beneath each stance contact for a body at 'model_to_world', and the mean normal of the ground found there
		void FindNeutralFeet(xform const& model_to_world, IGround const& ground)
		{
			// One query covers every limb
			m_probe_rays.clear();
			for (auto const& limb : m_limbs)
				m_probe_rays.push_back(RayBeneath(model_to_world * limb.stance_contact, m_up, limb.reach));

			auto hits = ProbeGround(ground);
			auto normal = v4::Zero();
			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
			{
				// Rays that find no ground say nothing about its slope
				m_neutral[i] = FootOnGround(m_limbs[i], m_probe_rays[i], hits[i], model_to_world.rot);
				if (hits[i].hit)
					normal += hits[i].normal;
			}

			// When no foot finds ground, the last measured slope is the best evidence of it. The body's up is not: an upright body would make any slope look level.
			if (LengthSq(normal) != 0)
				m_ground_up = Normalise(normal);
		}

		// True if family 'f' can be used on the ground beneath the feet. A family that is not the current one needs ground that is less steep
		// by 'SlopeMargin'. Families with a slope limit cannot be used without gravity.
		bool AllowsGround(int f, v4 gravity) const
		{
			// A family without a slope limit can be used anywhere
			auto const& family = m_gait.families[f];
			if (family.max_slope >= math::constants<float>::tau_by_2)
				return true;
			if (LengthSq(gravity) == 0)
				return false;

			auto limit = family.max_slope - (f == m_family ? 0.0f : SlopeMargin);
			return Dot(m_ground_up, -Normalise(gravity)) >= std::cos(limit);
		}

		// Change to the requested family if the ground allows it, otherwise to the first family in the gait that it allows, other than an action
		// family. If the ground allows no family, the current one is kept.
		void ChooseFamily(v4 gravity)
		{
			// Prefer the request, then the gait's order
			m_request_pending = false;
			auto f = m_requested;
			for (int i = 0, iend = static_cast<int>(m_families.size()); i != iend && !AllowsGround(f, gravity); ++i)
			{
				// Try each family in order until the ground allows one. Actions are only played on request.
				if (!m_gait.families[i].action)
					f = i;
			}
			if (f == m_family || !AllowsGround(f, gravity))
				return;

			// Feet find their new neutral positions from the new stance, and step to them as the limbs' support changes
			m_prev_family = m_family;
			m_family = f;
			m_family_weight = Fade(0.0f);
			m_family_weight.Start(1.0f, m_request_blend_s);
			UseStance(m_family);
		}

		// The distance along the surface from limb 'i's neutral contact to its foot's contact
		float FootDrift(int i) const
		{
			// Only the distance along the surface matters; the neutral frame is already on the ground
			auto const& limb = m_limbs[i];
			auto offset = ContactOf(limb, limb.foot) - ContactOf(limb, m_neutral[i]);
			offset -= m_up * Dot(offset, m_up);
			return Length(offset);
		}

		// The angle (radians, in [0, pi]) by which limb 'i's foot is turned about the body's up axis away from its neutral orientation
		float FootTwist(int i) const
		{
			// Take the part of the foot's rotation, relative to its neutral frame, that turns about up
			auto rel = m_limbs[i].foot.rot * ~m_neutral[i].rot;
			auto s = rel.x * m_up.x + rel.y * m_up.y + rel.z * m_up.z;
			return std::abs(2.0f * std::atan2(rel.w < 0 ? -s : s, std::abs(rel.w)));
		}

		// True if limb 'i' is planted further than the settle distance from its neutral position, or turned further than the settle angle
		bool IsDisplaced(int i) const
		{
			// The settle distance scales with the slowest cycle so that small and large bodies settle alike
			auto settle = SettleFraction * m_gait.families[m_family].cycles.front().stride;
			return FootDrift(i) > settle || FootTwist(i) > SettleTwist;
		}

		// True if planted limb 'i' has drifted or turned so far from its neutral frame that it must step now rather than wait for its swing.
		// This happens when the body turns or speeds up faster than the stepping can follow.
		bool IsStrained(int i) const
		{
			// The drift limit scales with the current stride, because a planted foot normally drifts by up to half a stride before it lifts
			auto const& family = m_gait.families[m_family];
			auto stride = Stride(family, std::max(m_gait_speed, family.cycles.front().speed));
			return FootDrift(i) > StrainFraction * stride || FootTwist(i) > StrainTwist;
		}

		// True if limb 'i' can lift without leaving too little support: no more than 'MaxEarlySwingFraction' of the supporting limbs may swing at once
		bool CanLiftEarly(int i) const
		{
			// Count the limbs carrying weight, and those of them already in the air
			auto supporting = 0;
			auto swinging = 0;
			for (int k = 0, kend = static_cast<int>(m_limbs.size()); k != kend; ++k)
			{
				if (m_limbs[k].support == 0)
					continue;

				supporting += 1;
				swinging += m_limbs[k].state != EFootState::Stance ? 1 : 0;
			}
			return m_limbs[i].support != 0 && swinging + 1 <= static_cast<int>(MaxEarlySwingFraction * supporting);
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

			// A planted foot that the body has dragged or turned too far steps now, out of turn, so that the limb does not twist under the body.
			// The early swing lasts as long as a normal swing, and only while enough other feet stay planted.
			if (limb.state == EFootState::Stance && local < duty && IsStrained(i) && CanLiftEarly(i))
				StartSwing(limb, 1.0f - duty);

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
			auto fore = m_rig.chains[limb.chain].role == EChainRole::Leg ? 1.0f : -1.0f;
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

		// The fraction of the way from lift-off to landing that a swinging foot has moved at swing progress 's'. It starts and ends at rest.
		static float SwingEase(float s)
		{
			return s * s * (3.0f - 2.0f * s);
		}

		// Move a swinging foot from lift-off towards its landing, raised by up to 'lift' and moved out by up to 'out', touching down at the end
		// of the swing. Sets how freely the foot turns with its limb from how high it is raised.
		static void MoveSwingingFoot(LimbState& limb, v4 lift, v4 out)
		{
			// Only swinging feet move
			limb.free = 0.0f;
			if (limb.state != EFootState::Swing)
				return;

			if (limb.swing >= 1.0f)
			{
				limb.foot = limb.land;
				limb.state = EFootState::Stance;
				return;
			}

			// Move smoothly from lift-off to landing, raised by an arc that is flat at both ends, and bowed outwards by a broader arc so that the
			// foot moves out as it lifts and back in as it lands
			auto s = limb.swing;
			auto ease = SwingEase(s);
			auto arc = 64.0f * Cube(s * (1.0f - s));
			auto bow = 16.0f * Sqr(s * (1.0f - s));
			limb.foot = xform(limb.lift.pos + (limb.land.pos - limb.lift.pos) * ease + lift * arc + out * bow, Slerp(limb.lift.rot, limb.land.rot, ease));

			// The foot only needs to match the ground near it
			auto h = Clamp(arc / FreeFootHeight, 0.0f, 1.0f);
			limb.free = h * h * (3.0f - 2.0f * h);
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

		// Turn the end bone of each raised foot from its goal orientation towards the stance's bend at the ankle or wrist, by how freely the foot
		// turns with its limb. The end bone keeps its position, so the limb's pose does not change.
		void TurnRaisedFeet()
		{
			// The end bone's descendants, such as digits, are updated once the pose is finished (see 'm_finish_refresh')
			for (auto const& limb : m_limbs)
			{
				if (limb.free == 0)
					continue;

				auto const& bones = m_rig.chains[limb.chain].bones;
				auto const& end = m_pose.model[bones.back()];
				auto bent = m_pose.model[bones[1]].rot * limb.stance_bend;
				m_pose.SetModel(m_rig, bones.back(), xform(end.pos, Normalise(Slerp(end.rot, bent, limb.free * limb.support))));
			}
		}

		// Lay the digits of each planted foot or hand on its sole's plane, by how firmly the foot is planted. Each digit bone turns to lie
		// parallel to the plane, then the whole digit hinges at its base so that its tip contact touches the plane. Raised feet keep the guide's digits.
		void PlantDigits()
		{
			// Digit bones are posed from their parents' current frames. Their descendants are updated once the pose is finished (see 'm_finish_refresh').
			for (auto const& limb : m_limbs)
			{
				// A foot is planted by how much it carries the limb and how close it is to the ground
				auto planted = limb.support * (1.0f - limb.free);
				if (planted == 0 || limb.digits.empty())
					continue;

				auto const& end = m_pose.model[m_rig.chains[limb.chain].bones.back()];
				auto up = Rotate(end.rot, limb.sole_up);
				auto sole = end * m_rig.contacts[limb.contact].offset;
				for (auto c : limb.digits)
				{
					// Turn each bone by the shortest arc that removes the part of its direction across the plane
					auto const& bones = m_rig.chains[c].bones;
					for (auto b : bones)
					{
						auto frame = m_pose.model[m_rig.bones[b].parent] * m_pose.local[b];
						auto dir = Rotate(frame.rot, v4::YAxis());
						auto flat = dir - up * Dot(dir, up);
						auto turn = LengthSq(flat) > tiny<float> ? Slerp(Identity<quat>(), quat(dir, Normalise(flat)), planted) : Identity<quat>();
						m_pose.SetModel(m_rig, b, xform(frame.pos, Normalise(turn * frame.rot)));
					}

					// Hinge the digit at its base so that its tip contact is on the plane, keeping the tip's direction across the plane.
					// A base too high above the plane for the tip to reach it is left as it is.
					auto contact = DigitContact(c);
					if (contact < 0)
						continue;

					auto const& base = m_pose.model[bones.front()];
					auto to_tip = m_pose.model[bones.back()] * m_rig.contacts[contact].offset - base.pos;
					auto height = Dot(base.pos - sole, up);
					auto across = to_tip - up * Dot(to_tip, up);
					auto reach_sq = LengthSq(to_tip) - Sqr(height);
					if (reach_sq <= 0 || LengthSq(across) <= tiny<float>)
						continue;

					auto target = Normalise(across) * Sqrt(reach_sq) - up * height;
					auto hinge = Slerp(Identity<quat>(), quat(to_tip, target), planted);
					m_pose.SetModel(m_rig, bones.front(), xform(base.pos, Normalise(hinge * base.rot)));
				}
			}
		}

		// The model-space line through the spine, from the head of its first bone to the tip of its last bone, for the bone transforms 'model'.
		// The rig must have a spine.
		std::pair<v4, v4> SpineSegment(std::vector<xform> const& model) const
		{
			// The spine chain's bones run from the pelvis to the chest
			auto const& bones = m_rig.chains[m_spine].bones;
			return { model[bones.front()].pos, model[bones.back()] * v4(0, m_rig.bones[bones.back()].length, 0, 1) };
		}

		// The distance from point 'p' to the line segment from 'a' to 'b'
		static float DistanceToSegment(v4 p, v4 a, v4 b)
		{
			// Project onto the line and clamp to the segment's ends
			auto ab = b - a;
			auto len_sq = LengthSq(ab);
			auto t = len_sq > tiny<float> ? Clamp(Dot(p - a, ab) / len_sq, 0.0f, 1.0f) : 0.0f;
			return Length(p - (a + ab * t));
		}

		// Turn the elbow of each grasping arm about the line from its shoulder to its wrist, so that the elbow flares out from the body and stays
		// outside the torso (see 'm_torso_radius'). The hand keeps its frame, so only the upper and lower arm move. By how strongly each hand holds
		// its grasp.
		void ClearElbows()
		{
			// A rig without a spine has no torso to keep the elbows out of
			if (m_spine < 0 || m_torso_radius <= 0)
				return;

			auto [a, b] = SpineSegment(m_pose.model);
			for (auto const& reach : m_reaches)
			{
				// The elbow can be anywhere on the circle around the shoulder-to-wrist line that keeps both bones' lengths. A straight arm has no choice.
				auto weight = reach.weight.Value();
				if (weight == 0)
					continue;

				auto const& chain = m_rig.chains[m_limbs[reach.limb].chain];
				auto root = m_pose.model[chain.bones[0]].pos;
				auto mid = m_pose.model[chain.bones[1]].pos;
				auto end = m_pose.model[chain.bones[2]];
				auto dir = Normalise(end.pos - root, v4::Zero());
				auto centre = root + dir * Dot(mid - root, dir);
				auto radius = Length(mid - centre);
				if (radius < 1e-4f || LengthSq(dir) == 0)
					continue;

				// The preferred elbow is the solved one pushed outward, away from the body's midline (the model's X = 0 plane) on the arm's side
				auto bend = (mid - centre) / radius;
				auto side = root.x > 0 ? 1.0f : root.x < 0 ? -1.0f : 0.0f;
				auto flared = bend + v4(side * ElbowFlare * weight, 0, 0, 0);
				flared = Normalise(flared - dir * Dot(flared, dir), bend);

				// Search around the circle, alternating either way from the preferred elbow, for the nearest elbow outside the torso. Keep the elbow
				// with the most clearance in case none is outside.
				auto best = flared;
				auto best_clearance = -std::numeric_limits<float>::max();
				for (int k = 0; k <= 2 * ElbowSteps; ++k)
				{
					auto angle = ElbowStep * static_cast<float>((k + 1) / 2) * (k % 2 == 0 ? 1.0f : -1.0f);
					auto candidate = Rotate(quat(dir, angle), flared);
					auto clearance = DistanceToSegment(centre + candidate * radius, a, b) - m_torso_radius;
					if (clearance >= 0)
					{
						best = candidate;
						break;
					}
					if (clearance > best_clearance)
					{
						best = candidate;
						best_clearance = clearance;
					}
				}

				// Re-solve the arm with the chosen elbow, faded in with the grasp. The hand's frame is unchanged, so its digits need no update here.
				auto pole = Normalise(Lerp(bend, best, weight), best);
				SolveLimb(m_rig, chain, m_pose, end, pole);
			}
		}

		// Curl the digits of each hand whose grasp has a wrap radius around a bar of that radius, by how strongly the hand holds its grasp. The bar lies
		// against the palm at the hand's grip point, across the fingers. Each digit bone in turn, from the base, turns towards the palm about the axis
		// across its digit until its tip is the limb's skin depth from the bar's surface, or as near to that as it can get, so the digit closes on the bar.
		void CurlDigits()
		{
			// Digit bones are posed from their parents' current frames. Their descendants are updated once the pose is finished (see 'm_finish_refresh').
			for (auto const& reach : m_reaches)
			{
				auto weight = reach.weight.Value();
				auto const& limb = m_limbs[reach.limb];
				if (weight == 0 || reach.grasp.wrap <= 0 || limb.digits.empty())
					continue;

				// The bar follows the posed hand rather than the grasp, so the digits close on it even when the arm cannot quite reach the grasp
				auto const& end = m_pose.model[m_rig.chains[limb.chain].bones.back()];
				auto palm = Rotate(end.rot, -limb.sole_up);
				auto along = Rotate(end.rot, Normalise(Cross(-limb.sole_up, v4::YAxis())));
				auto centre = end * limb.grip + palm * reach.grasp.wrap;
				auto touch = reach.grasp.wrap + limb.skin;
				for (auto c : limb.digits)
				{
					// Every bone of the digit turns about the same axis, so the digit curls in one plane. A digit that already points into the palm stays as it is.
					auto const& bones = m_rig.chains[c].bones;
					auto first = m_pose.model[m_rig.bones[bones.front()].parent] * m_pose.local[bones.front()];
					auto axis = Cross(Rotate(first.rot, v4::YAxis()), palm);
					if (LengthSq(axis) < 1e-6f)
						continue;

					axis = Normalise(axis);
					for (auto b : bones)
					{
						// Turn the bone by the least angle that brings its tip onto the bar's skin surface, or the angle that brings it nearest
						auto frame = m_pose.model[m_rig.bones[b].parent] * m_pose.local[b];
						auto tip = Rotate(frame.rot, v4(0, m_rig.bones[b].length, 0, 0));
						auto gap = [&](float angle)
						{
							// The tip's distance from the bar's axis line, less the touching distance
							auto p = frame.pos + Rotate(quat(axis, angle), tip) - centre;
							return Length(p - along * Dot(p, along)) - touch;
						};
						auto angle = CurlToTouch(gap);
						m_pose.SetModel(m_rig, b, xform(frame.pos, Normalise(quat(axis, angle * weight) * frame.rot)));
					}
				}
			}
		}

		// The least angle in [0, 'MaxCurl'] at which 'gap' (the distance beyond touching, as a function of the curl angle) is not positive, or the angle
		// at which it is smallest if it is positive throughout. A digit already touching does not curl.
		template <typename Gap>
		static float CurlToTouch(Gap gap)
		{
			// Step through the range for the first touching angle, keeping the nearest in case none touches
			constexpr int Samples = 16;
			constexpr int Bisections = 10;
			if (gap(0.0f) <= 0)
				return 0.0f;

			auto best = 0.0f;
			auto best_gap = gap(0.0f);
			for (int k = 1; k <= Samples; ++k)
			{
				auto a = MaxCurl * k / Samples;
				auto g = gap(a);
				if (g <= 0)
				{
					// Narrow the step that crosses the surface down to the touching angle
					auto lo = MaxCurl * (k - 1) / Samples;
					auto hi = a;
					for (int i = 0; i != Bisections; ++i)
					{
						auto mid = 0.5f * (lo + hi);
						(gap(mid) <= 0 ? hi : lo) = mid;
					}
					return hi;
				}
				if (g < best_gap)
				{
					best = a;
					best_gap = g;
				}
			}
			return best;
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
					auto ease = SwingEase(limb.swing);
					return ContactOf(limb, limb.lift) + (ContactOf(limb, limb.land) - ContactOf(limb, limb.lift)) * ease;
				}
				default:
				{
					throw std::invalid_argument("Unknown foot state");
				}
			}
		}

		// Raise or lower the guide's pelvis and chest by the height of the ground under the hind and fore feet, pitching the body between them,
		// shift them sideways over the walking limbs, and turn them about the up axis with the walking limbs and into turns
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

				auto i = m_rig.chains[limb.chain].role == EChainRole::Leg ? 0 : 1;
				height[i] += (world_to_model * limb.ground).z;
				count[i] += 1;
				arm_support += i == 1 ? limb.support : 0.0f;
			}

			// Hanging arms do not set the height of the chest, so the fore height moves from the hind height to the arms' height as they
			// take weight
			auto hind = count[0] != 0 ? height[0] / count[0] : count[1] != 0 ? height[1] / count[1] : 0.0f;
			auto fore = count[1] != 0 ? hind + (height[1] / count[1] - hind) * (count[0] != 0 ? arm_support / count[1] : 1.0f) : hind;

			// A swimming body floats, so the ground beneath it does not move it
			auto dry = 1.0f - WaterWeight();
			hind *= dry;
			fore *= dry;

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

			// Each end also turns about the up axis by its girdle's sway, and the chest leads into a turn while the pelvis trails. The ends
			// turn about their own positions, so the spine bends between them without moving its ends.
			auto [pelvis_yaw, chest_yaw] = GirdleSway();
			auto [pelvis_shift, chest_shift] = WeightShift();
			auto lead = Clamp(m_turn_rate * SpineLeadTime, -MaxSpineLead, MaxSpineLead);
			auto pelvis_turn = quat(v4::ZAxis(), pelvis_yaw - lead);
			auto chest_turn = quat(v4::ZAxis(), chest_yaw + lead);
			m_targets[m_spine].base = xform(pelvis0.pos + v4(pelvis_shift, 0, hind, 0), Normalise(pelvis_turn * pitch * pelvis0.rot));
			m_targets[m_spine].frame = xform(chest0.pos + v4(chest_shift, 0, fore, 0), Normalise(chest_turn * pitch * chest0.rot));

			// Breathing turns the chest bone about its head so that its tip moves out from the front of the body, which faces along the spine
			// turned a quarter turn forward about model +X. The chest's head stays in place, so the rib cage swells rather than the belly.
			auto breath = m_breath.chest * Inhale() * Breathing();
			if (breath != 0)
			{
				auto& chest = m_targets[m_spine].frame;
				auto front = Normalise(Cross(v4::XAxis(), span));
				auto tip = Rotate(chest.rot, v4::YAxis()) * m_rig.bones[m_rig.chains[m_spine].bones.back()].length;
				auto swell = quat(Normalise(tip), Normalise(tip + front * breath));
				chest.rot = Normalise(swell * chest.rot);
			}

			// Record how far the chest has moved from its stance, so callers can move held things with it. The stoop is left out because callers
			// place the grasps that cause it: a grasp that moved with the stoop would keep needing it.
			m_chest_motion = m_targets[m_spine].frame * Invert(m_chest_stance);

			// Stoop as far as the grasping hands need, or as far as a collapse buckles the body. The stoop follows the need smoothly, so a hand may
			// briefly stretch for a grasp before the body brings it within reach.
			auto need = std::max(StoopNeeded(world_to_model), CollapseStoop * Ease(0.0f, CollapseBuckleEnd, CollapseProgress()));
			m_stoop += (need - m_stoop) * std::min(dt / StoopResponse, 1.0f);
			if (m_stoop > 0)
				std::tie(m_targets[m_spine].base, m_targets[m_spine].frame) = Stooped(m_stoop, m_targets[m_spine].base, m_targets[m_spine].frame);
		}

		// How far through its in-and-out motion the current breath is: 0 at the start and end of the breath, and 1 halfway
		float Inhale() const
		{
			return 0.5f * (1.0f - std::cos(math::constants<float>::tau * m_breath_phase));
		}

		// A smooth step from 0 at 'lo' to 1 at 'hi', clamped outside that range
		static float Ease(float lo, float hi, float x)
		{
			auto t = Clamp((x - lo) / (hi - lo), 0.0f, 1.0f);
			return t * t * (3.0f - 2.0f * t);
		}

		// +1 for the left side of the body (model +X), -1 for the right side, and 0 for the middle
		static float SideSign(ESide side)
		{
			switch (side)
			{
				case ESide::Centre: { return 0.0f; }
				case ESide::Left: { return +1.0f; }
				case ESide::Right: { return -1.0f; }
				default: throw std::runtime_error("Unknown body side");
			}
		}

		// Start, advance, or fade out the action of the current family. An action that is fading out finishes before another starts, so a collapsed
		// body stands back up before it does anything else.
		void StepAction(float dt)
		{
			// Start the current family's action when no action is playing
			if (m_action < 0 && m_gait.families[m_family].action)
			{
				m_action = m_family;
				m_action_time = 0.0f;
				m_action_weight = 0.0f;
			}
			if (m_action < 0)
				return;

			// An action plays forward while its family is current, and backs out otherwise
			auto const& action = *m_gait.families[m_action].action;
			auto playing = m_action == m_family;
			switch (action.kind)
			{
				case EActionKind::Collapse:
				{
					// The collapse plays backwards in real time, so the body gets up the way it fell
					m_action_time = Clamp(m_action_time + (playing ? dt : -dt), 0.0f, action.duration);
					m_action_weight = m_action_time / action.duration;
					break;
				}
				case EActionKind::Punches:
				{
					// The punches fade in and out with the family blend, and keep looping while they fade out
					auto fade = m_request_blend_s > 0 ? dt / m_request_blend_s : 1.0f;
					m_action_weight = Clamp(m_action_weight + (playing ? fade : -fade), 0.0f, 1.0f);
					m_action_time = std::fmod(m_action_time + dt, SequenceDuration(action));
					break;
				}
				default: throw std::runtime_error("Unknown action kind");
			}

			// A finished action makes way for the next
			if (!playing && m_action_weight == 0)
				m_action = -1;
		}

		// The action that is playing or fading out, if it is of 'kind', otherwise null
		GaitAction const* ActiveAction(EActionKind kind) const
		{
			// Only the active action family's action shows
			if (m_action < 0)
				return nullptr;

			auto const& action = m_gait.families[m_action].action;
			return action->kind == kind ? &*action : nullptr;
		}

		// How far through a collapse the body is, in [0,1]. 0 when no collapse is playing.
		float CollapseProgress() const
		{
			return ActiveAction(EActionKind::Collapse) ? m_action_weight : 0.0f;
		}

		// How much the body breathes and looks around, in [0,1]. A collapsed body goes still.
		float Breathing() const
		{
			return 1.0f - Ease(CollapseRelaxStart, 1.0f, CollapseProgress());
		}

		// The seconds that one pass through the punch sequence of 'action' takes
		static float SequenceDuration(GaitAction const& action)
		{
			auto total = 0.0f;
			for (auto const& punch : action.punches)
				total += punch.duration;

			return total;
		}

		// Finish the pose of a body that is 'u' of the way through a collapse, where u is in (0,1]. The joints go limp towards the stance and are
		// held within their limits, then the whole body topples sideways about its lowest outside edge (see 'Topple').
		void Collapse(float u)
		{
			// Relax each joint towards the stance and keep it within its limits. Root bones place the body, so they are moved only by the topple.
			auto relax = CollapseRelax * Ease(CollapseRelaxStart, 1.0f, u);
			auto limit = Ease(0.0f, CollapseLimitEnd, u);
			auto const& stance = m_families[m_action].stance_local.front();
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
			{
				// Limits are relative to the rest orientation under the parent
				auto const& bone = m_rig.bones[b];
				if (bone.parent < 0)
					continue;

				auto& local = m_pose.local[b];
				local.rot = Normalise(Slerp(local.rot, stance[b].rot, relax));
				if (!bone.limit.IsFree())
				{
					// Blend the limits in so that the standing pose at the start of the collapse is not changed suddenly
					auto allowed = Normalise(bone.rest.rot * bone.limit.Clamp(~bone.rest.rot * local.rot));
					local.rot = Normalise(Slerp(local.rot, allowed, limit));
				}
			}
			m_pose.UpdateModel(m_rig);

			// The topple falls slowly at first, then faster, until the body lies flat
			auto t = Clamp((u - CollapseToppleStart) / (CollapseToppleEnd - CollapseToppleStart), 0.0f, 1.0f);
			Topple(m_pose.model, math::constants<float>::tau_by_4 * t * t);
			for (int b = 0, bend = static_cast<int>(m_rig.bones.size()); b != bend; ++b)
			{
				// Root bones' local transforms are their model transforms
				if (m_rig.bones[b].parent < 0)
					m_pose.local[b] = m_pose.model[b];
			}

			// Limp limbs and the head flop down as the body lands
			Sag(Ease(CollapseRelaxStart, 1.0f, u));
		}

		// Let the arms, legs and necks hang under gravity, which is model -Z, by 'weight' of the way, where weight is in [0,1]. Each bone turns
		// down until its tip rests on the ground (model z = 0), or it reaches its joint limits. A bone that points straight up falls forward.
		//  - The pose's model transforms must be current for its local transforms. On return, they are current again.
		void Sag(float weight)
		{
			// Chains are visited parents first, so each bone turns from its parent's settled frame
			if (weight == 0)
				return;

			for (auto c : m_rig.solve_order)
			{
				// Only limbs and necks hang. The spine lies on the ground and carries them.
				auto const& chain = m_rig.chains[c];
				if (chain.role != EChainRole::Arm && chain.role != EChainRole::Leg && chain.role != EChainRole::Neck)
					continue;

				for (auto b : chain.bones)
				{
					// Bring the bone along with its parent, which may have sagged
					auto const& bone = m_rig.bones[b];
					auto const& parent = m_pose.model[bone.parent];
					auto& model = m_pose.model[b];
					model = parent * m_pose.local[b];

					// The lowest the bone may point is the direction that puts its tip on the ground. A bone that already points that low, or lower,
					// is left alone.
					auto dir = Rotate(model.rot, v4::YAxis());
					if (bone.length <= 0)
						continue;

					auto low = Clamp(-model.pos.z / bone.length, -1.0f, 1.0f);
					if (low >= dir.z)
						continue;

					// Turn the bone down within its vertical plane, through the horizontal so that neither part of the turn is a half turn, then hold it
					// within its joint limits (relative to the rest orientation under the parent)
					auto flat = Normalise(v4(dir.x, dir.y, 0, 0), v4(0, -1, 0, 0));
					auto down = flat * std::sqrt(1 - low * low) + v4::ZAxis() * low;
					auto turn = quat(flat, down) * quat(dir, flat);
					auto rot = Normalise(Slerp(Identity<quat>(), turn, weight) * model.rot);
					auto local = Normalise(~parent.rot * rot);
					if (!bone.limit.IsFree())
						local = Normalise(bone.rest.rot * bone.limit.Clamp(~bone.rest.rot * local));

					m_pose.local[b].rot = local;
					model = parent * m_pose.local[b];
				}
			}

			// Bring the bones outside the hanging chains, such as digits, along with them
			m_pose.UpdateModel(m_rig);
		}

		// Turn the model-space bone frames 'model' by 'angle' (radians) about a line along model Y through the lowest joint height and the furthest
		// joint position towards model +X. A positive angle turns model +Z towards +X, so the body falls onto its left side about the outside of its
		// left side. Joints below the ground are lifted onto it as the body falls, so at a quarter turn no joint is below the ground.
		static void Topple(std::span<xform> model, float angle)
		{
			// The pivot line is the body's lowest outside edge on its left
			if (model.empty())
				return;

			auto pivot = v4(-std::numeric_limits<float>::max(), 0, std::numeric_limits<float>::max(), 1);
			for (auto const& xf : model)
			{
				pivot.x = std::max(pivot.x, xf.pos.x);
				pivot.z = std::min(pivot.z, xf.pos.z);
			}

			// Turn every frame about the pivot line
			auto rot = quat(v4::YAxis(), angle);
			auto lowest = std::numeric_limits<float>::max();
			for (auto& xf : model)
			{
				xf.pos = pivot + Rotate(rot, xf.pos - pivot);
				xf.rot = Normalise(rot * xf.rot);
				lowest = std::min(lowest, xf.pos.z);
			}

			// A joint that was below the ground, such as a hanging hand as the knees buckle, is not left there by the topple. The body lifts so that
			// its lowest joint is on the ground.
			auto lift = std::max(0.0f, -lowest) * Clamp(angle / math::constants<float>::tau_by_4, 0.0f, 1.0f);
			for (auto& xf : model)
				xf.pos.z += lift;
		}

		// The punch of the active punches action that is being thrown, and how far out its fist is, in [0,1]
		std::pair<Punch const*, float> CurrentPunch() const
		{
			// Walk the sequence to the punch that the action time is in
			auto const* action = ActiveAction(EActionKind::Punches);
			if (action == nullptr)
				return { nullptr, 0.0f };

			auto time = m_action_time;
			for (auto const& punch : action->punches)
			{
				// The fist drives out quickly, then comes back to guard more slowly
				if (time < punch.duration || &punch == &action->punches.back())
				{
					auto u = Clamp(time / punch.duration, 0.0f, 1.0f);
					auto extend = u < PunchOut ? Ease(0.0f, PunchOut, u) : 1.0f - Ease(PunchOut, PunchBack, u);
					return { &punch, extend };
				}
				time -= punch.duration;
			}
			return { nullptr, 0.0f };
		}

		// How the torso of a body throwing punches moves from its guide pose. It is still when no punches show.
		struct TorsoDrive
		{
			float yaw = 0;          // The chest's turn (radians) about model +Z. The pelvis turns 'PunchPelvisShare' of this.
			float lean = 0;         // The chest's forward lean (radians) about the pelvis
			v4 hips = v4::Zero();   // The pelvis's model-space shift
			v4 chest = v4::Zero();  // The chest's model-space shift, from both the lean and the hips' shift
		};

		// The torso motion of a body throwing punches. On guard, the left shoulder leads and the body crouches forward. Into each punch, the
		// punching shoulder turns forward and the chest leans after the fist, carrying the hips forward over the front foot, so the body's weight
		// goes into the punch and the fist reaches further.
		TorsoDrive PunchDrive() const
		{
			// The torso moves only while punches show
			auto [punch, extend] = CurrentPunch();
			if (punch == nullptr || m_spine < 0)
				return {};

			// Turning about model +Z by a positive angle moves the right shoulder (-X) forward (-Y), and leaning about model +X by a positive angle
			// moves the chest forward
			auto weight = m_action_weight;
			auto drive = TorsoDrive{
				.yaw = (-GuardYaw - SideSign(punch->hand) * PunchYaw(punch->kind) * extend) * weight,
				.lean = (GuardLean + PunchLean(punch->kind) * extend) * weight,
			};

			// The hips drop behind the guard and move forward with a share of the punch's lean. The chest swings forward about the hips.
			auto const& guide = m_guide_targets[m_spine];
			auto span = guide.frame.pos - guide.base.pos;
			auto length = Length(span);
			drive.hips = v4(0, -PunchHipDrive * length * PunchLean(punch->kind) * extend, -GuardCrouch * length, 0) * weight;
			drive.chest = drive.hips + Rotate(quat(v4::XAxis(), drive.lean), span) - span;
			return drive;
		}

		// Move the torso of a body throwing punches (see 'PunchDrive'). The chest and pelvis turn about their own positions, so the spine bends
		// between them.
		void DriveTorso()
		{
			// Nothing moves when no punches show
			auto drive = PunchDrive();
			if (m_spine < 0 || (drive.yaw == 0 && drive.lean == 0))
				return;

			auto& spine = m_targets[m_spine];
			spine.frame = xform(spine.frame.pos + drive.chest, Normalise(quat(v4::ZAxis(), drive.yaw) * quat(v4::XAxis(), drive.lean) * spine.frame.rot));
			spine.base = xform(spine.base.pos + drive.hips, Normalise(quat(v4::ZAxis(), PunchPelvisShare * drive.yaw) * spine.base.rot));
		}

		// The angle (radians) by which the torso turns the punching shoulder forward at the full extent of each kind of punch
		static float PunchYaw(EPunchKind kind)
		{
			switch (kind)
			{
				case EPunchKind::Jab: { return 0.25f; }
				case EPunchKind::Cross: { return 0.75f; }
				case EPunchKind::Hook: { return 0.65f; }
				case EPunchKind::Uppercut: { return 0.45f; }
				default: throw std::runtime_error("Unknown punch kind");
			}
		}

		// The angle (radians) by which the torso leans forward, beyond its guard, at the full extent of each kind of punch
		static float PunchLean(EPunchKind kind)
		{
			switch (kind)
			{
				case EPunchKind::Jab: { return 0.12f; }
				case EPunchKind::Cross: { return 0.3f; }
				case EPunchKind::Hook: { return 0.2f; }
				case EPunchKind::Uppercut: { return 0.05f; }
				default: throw std::runtime_error("Unknown punch kind");
			}
		}

		// The model-space position of the shoulder joint of the arm 'limb', carried by the body like a hanging arm (see 'HangingFrame')
		v4 ArmRoot(LimbState const& limb) const
		{
			// Without a spine the body does not move relative to the guide
			auto root = m_guide.model[m_rig.chains[limb.chain].bones.front()];
			if (m_spine < 0)
				return root.pos;

			return (m_targets[m_spine].frame * Invert(m_guide_targets[m_spine].frame) * root).pos;
		}

		// Aim the hands of a body throwing punches. Both hands hold a guard in front of the chin, and the punching hand drives out to the punch and
		// back. Offsets are in model axes from each shoulder, in units of the arm's reach, with 'side' being +1 for the left arm and -1 for the right.
		void AimPunches()
		{
			// Arms that are not punching stay on guard
			auto [punch, extend] = CurrentPunch();
			if (punch == nullptr)
				return;

			for (auto const& limb : m_limbs)
			{
				// Only arms box
				auto const& chain = m_rig.chains[limb.chain];
				if (chain.role != EChainRole::Arm)
					continue;

				// The guard holds the fist up in front of the chin, the forearm upright and the elbow down
				auto side = SideSign(chain.side);
				auto guard = v4(-side * 0.2f, -0.4f, 0.2f, 0);
				auto guard_dir = Normalise(v4(0, -0.4f, 1, 0));
				auto guard_pole = Normalise(v4(side * 0.5f, 0, -1, 0));

				// The punch's path from the guard, through an optional control point, to its full extent
				auto ctrl = guard;
				auto end = guard;
				auto dir = guard_dir;
				auto pole = guard_pole;
				auto strike = 0.0f;
				if (chain.side == punch->hand)
				{
					strike = extend;
					switch (punch->kind)
					{
						case EPunchKind::Jab:
						{
							// Straight out from the lead side
							end = v4(-side * 0.25f, -0.9f, 0.15f, 0);
							ctrl = (guard + end) * 0.5f;
							dir = v4(0, -1, 0, 0);
							pole = Normalise(v4(side * 0.5f, 0, -1, 0));
							break;
						}
						case EPunchKind::Cross:
						{
							// Straight out across the body's midline
							end = v4(-side * 0.35f, -0.9f, 0.15f, 0);
							ctrl = (guard + end) * 0.5f;
							dir = v4(0, -1, 0, 0);
							pole = Normalise(v4(side * 0.5f, 0, -1, 0));
							break;
						}
						case EPunchKind::Hook:
						{
							// Out to the side, then around in front, with the elbow raised level with the fist
							ctrl = v4(side * 0.25f, -0.45f, 0.25f, 0);
							end = v4(-side * 0.45f, -0.6f, 0.2f, 0);
							dir = Normalise(v4(-side, -0.6f, 0, 0));
							pole = Normalise(v4(side, 0, 0.3f, 0));
							break;
						}
						case EPunchKind::Uppercut:
						{
							// Down, then up and forward under the chin of the opponent
							ctrl = v4(-side * 0.1f, -0.35f, -0.15f, 0);
							end = v4(-side * 0.25f, -0.55f, 0.35f, 0);
							dir = Normalise(v4(0, -0.3f, 1, 0));
							pole = Normalise(v4(side * 0.3f, 0, -1, 0));
							break;
						}
						default: throw std::runtime_error("Unknown punch kind");
					}
				}

				// Follow the curved path from the guard to the punch's extent, turning the fingers from the guard direction to the punch direction
				auto s = strike;
				auto offset = guard * ((1 - s) * (1 - s)) + ctrl * (2 * s * (1 - s)) + end * (s * s);
				auto aim = Normalise(Lerp(guard_dir, dir, s));
				auto fingers = Rotate(limb.stance_rot, v4::YAxis());
				auto fist = xform(ArmRoot(limb) + offset * limb.reach, Normalise(quat(fingers, aim) * limb.stance_rot));

				// Fade in over the current goal
				auto& target = m_targets[limb.chain];
				target.frame = BlendFrame(target.frame, fist, m_action_weight);
				target.pole = Normalise(Lerp(target.pole, Normalise(Lerp(guard_pole, pole, s)), m_action_weight), target.pole);
			}
		}

		// Clench the digits of the arms of a body throwing punches into fists. Each digit bone turns towards the palm by 'FistCurl', scaled by how
		// strongly the punches show. Hands that hold a grasp are left to their grasps.
		void ClenchFists()
		{
			// Digit bones are posed from their parents' current frames. Their descendants are updated once the pose is finished (see 'm_finish_refresh').
			if (ActiveAction(EActionKind::Punches) == nullptr || m_action_weight == 0)
				return;

			for (int i = 0, iend = static_cast<int>(m_limbs.size()); i != iend; ++i)
			{
				// Only free hands make fists
				auto const& limb = m_limbs[i];
				if (m_rig.chains[limb.chain].role != EChainRole::Arm || limb.digits.empty())
					continue;
				if (std::ranges::any_of(m_reaches, [&](auto const& reach) { return reach.limb == i && reach.weight.Value() != 0; }))
					continue;

				auto const& end = m_pose.model[m_rig.chains[limb.chain].bones.back()];
				auto palm = Rotate(end.rot, -limb.sole_up);
				for (auto c : limb.digits)
				{
					// Every bone of the digit turns about the same axis, so the digit curls in one plane
					auto const& bones = m_rig.chains[c].bones;
					auto first = m_pose.model[m_rig.bones[bones.front()].parent] * m_pose.local[bones.front()];
					auto axis = Cross(Rotate(first.rot, v4::YAxis()), palm);
					if (LengthSq(axis) < 1e-6f)
						continue;

					axis = Normalise(axis);
					for (auto b : bones)
					{
						// Each bone turns from its parent's current frame
						auto frame = m_pose.model[m_rig.bones[b].parent] * m_pose.local[b];
						m_pose.SetModel(m_rig, b, xform(frame.pos, Normalise(quat(axis, FistCurl * m_action_weight) * frame.rot)));
					}
				}
			}
		}

		// The sideways offsets (model X) of the pelvis and the chest that move the body's weight over the walking limbs. Each limb pulls its girdle
		// towards its side by the cycles' weight shift times its distance from the midline times cos(2 pi (phase - touchdown - duty/2)), most
		// strongly at the middle of its stance, so that two limbs half a cycle apart carry the girdle from over one foot to over the other.
		// Each girdle takes the mean over its limbs, weighted by how much each limb walks. A girdle whose limbs hang moves with the other girdle,
		// so that a body standing on its hind legs shifts as a whole.
		std::pair<float, float> WeightShift() const
		{
			// The shift grows with speed, like the guide's motion
			auto amplitude = WalkAmplitude(&GaitCycle::weight_shift);
			if (amplitude == 0)
				return {};

			float shift[2] = {};
			float support[2] = {};
			int count[2] = {};
			for (auto const& limb : m_limbs)
			{
				auto i = m_rig.chains[limb.chain].role == EChainRole::Leg ? 0 : 1;
				auto mid_stance = limb.footfall.touchdown + 0.5f * limb.footfall.duty;
				shift[i] += limb.support * limb.stance_contact.x * std::cos(math::constants<float>::tau * (m_phase - mid_stance));
				support[i] += limb.support;
				count[i] += 1;
			}

			// The fore girdle moves from the hind girdle's shift to its own as its limbs take weight, like its height (see 'StepSpine')
			auto hind = count[0] != 0 ? amplitude * shift[0] / count[0] : 0.0f;
			auto fore = count[1] != 0 ? hind + (amplitude * shift[1] / count[1] - hind) * (count[0] != 0 ? support[1] / count[1] : 1.0f) : hind;
			if (count[0] == 0)
				hind = fore;

			return { hind, fore };
		}

		// The yaw angles (radians, about model +Z) of the pelvis and the chest that swing each walking limb's root joint forward as its foot lands
		// and back as it lifts. Each limb turns its girdle by the cycles' spine sway times cos(2 pi (phase - touchdown)), in the direction that
		// moves its side forward (-Y) at touchdown. Each girdle takes the mean over its limbs, weighted by how much each limb walks, so that
		// a pair of limbs half a cycle apart turn it together and limbs that hang do not turn it.
		std::pair<float, float> GirdleSway() const
		{
			// The sway grows with speed, like the guide's motion
			auto amplitude = WalkAmplitude(&GaitCycle::spine_sway);

			// Turning a girdle by a small angle 'a' about +Z moves a point at lateral offset 'x' by 'a * x' along Y, so a root joint on the +X
			// side moves forward for a negative angle
			float yaw[2] = {};
			int count[2] = {};
			for (auto const& limb : m_limbs)
			{
				auto i = m_rig.chains[limb.chain].role == EChainRole::Leg ? 0 : 1;
				auto side = limb.stance_contact.x > 0 ? 1.0f : limb.stance_contact.x < 0 ? -1.0f : 0.0f;
				yaw[i] -= side * limb.support * std::cos(math::constants<float>::tau * (m_phase - limb.footfall.touchdown));
				count[i] += 1;
			}
			return {
				count[0] != 0 ? amplitude * yaw[0] / count[0] : 0.0f,
				count[1] != 0 ? amplitude * yaw[1] / count[1] : 0.0f,
			};
		}

		// Turn each neck from its guide aim towards the look point by the look weight. The aim moves with a torso that drives into punches, so the
		// head goes with the body and keeps facing forward.
		void StepNecks(float dt, xform const& world_to_model)
		{
			m_look_weight.Step(dt);
			m_look.Step(dt);
			auto weight = m_look_weight.Value() * Breathing();
			auto look = world_to_model * m_look.Frame().pos;
			auto drive = PunchDrive().chest;
			for (int c = 0, cend = static_cast<int>(m_rig.chains.size()); c != cend; ++c)
			{
				// Rotate the guide aim point about the neck's base onto the look direction, so that a zero weight gives the guide aim exactly
				auto const& chain = m_rig.chains[c];
				if (chain.role != EChainRole::Neck)
					continue;

				auto base = m_guide.model[chain.bones.front()].pos;
				auto aim = m_guide_targets[c].frame.pos - base;
				auto turn = Slerp(Identity<quat>(), quat(Normalise(aim), Normalise(look - base, Normalise(aim))), weight);
				m_targets[c].frame.pos = base + Rotate(turn, aim) + drive;
			}
		}

		// Trail each tail behind the body like a rope lying on the ground, as one smooth curve that continues the spine, bends down, and flattens onto the ground.
		//  - The tail's base keeps its position from the guide, and the first bone keeps its heading across the ground from the swaying hips. Each
		//    later joint is pulled from where it was last step towards the joint in front of it until it is one bone length away, so the hips' sway
		//    travels down the tail as an 'S' wave. This dragged rope sets only the heading of each bone across the ground.
		//  - The first bone droops as steeply as its joint limits allow about the guide's first bone, but bends down from the line of the spine
		//    (from the chest back through the pelvis) by no more than 'TailBend'. Each later bone bends up or down from the bone before it by at
		//    most 'TailBend', but no flatter than it would rest on the ground with its tapering skin, so the tail never curls up off the ground. It droops no more than the first
		//    bone's joint limits allow. Each bone takes the steepest droop that keeps the tail clear of the ground, assuming the bones after it flatten
		//    as fast as they can.
		//  - A bone is clear of the ground when its Tail contact, a point on the skin beneath the bone, is above the ground. A bone without a Tail
		//    contact keeps its tip above the ground instead. The ground is measured along the body's up direction beneath each joint of the rope.
		void StepTails(xform const& model_to_world, xform const& world_to_model, IGround const& ground)
		{
			// A swimming tail keeps the guide's shape, which carries the body wave, and does not trail on the ground. Its rope restarts from the
			// guide's shape when the body leaves the water.
			auto water = WaterWeight();
			if (water == 1.0f)
			{
				for (int c = 0, cend = static_cast<int>(m_rig.chains.size()); c != cend; ++c)
					m_ropes[c].clear();

				return;
			}

			// Drag each tail's joints after the joint in front of them, and gather them so that one ground query covers every tail
			m_tail_joints.clear();
			m_probe_rays.clear();
			for (int c = 0, cend = static_cast<int>(m_rig.chains.size()); c != cend; ++c)
			{
				// Only tails trail
				auto const& chain = m_rig.chains[c];
				if (chain.role != EChainRole::Tail)
					continue;

				// A tail that has not been posed yet, or that the body has moved away from at once, starts from the guide's shape
				auto count = static_cast<int>(chain.bones.size());
				auto& rope = m_ropes[c];
				auto length = TailLength(c, model_to_world);
				if (static_cast<int>(rope.size()) != count + 1 || LengthSq(rope[0] - TailJointPos(c, 0, model_to_world)) > Sqr(length))
				{
					rope.resize(count + 1);
					for (int k = 0; k <= count; ++k)
						rope[k] = TailJointPos(c, k, model_to_world);
				}

				// The base follows the body, and the first bone takes the guide's heading across the ground, turned with the hips as they sway. It keeps
				// its reach and height from last step, which the plan below set. Updating in order means each later joint follows the new position of
				// the one in front, so each swing of the first bone travels down the tail as a wave.
				auto first = rope[1] - rope[0];
				auto parent = m_rig.bones[chain.bones[0]].parent;
				auto sway = parent >= 0 ? m_pose.model[parent].rot * ~m_guide.model[parent].rot : Identity<quat>();
				auto link = model_to_world * Rotate(sway, GuideLink(c));
				rope[0] = TailJointPos(c, 0, model_to_world);
				rope[1] = rope[0] + Normalise(Across(link), Normalise(Across(first), Normalise(Perpendicular(m_up)))) * Length(Across(first)) + m_up * Dot(first, m_up);
				for (int k = 2; k <= count; ++k)
				{
					auto guide = TailJointPos(c, k, model_to_world) - TailJointPos(c, k - 1, model_to_world);
					rope[k] = rope[k - 1] + Normalise(rope[k] - rope[k - 1], Normalise(guide)) * Length(guide);
				}
				for (int k = 0; k <= count; ++k)
				{
					m_tail_joints.push_back(TailJoint{ .chain = c, .joint = k });
					m_probe_rays.push_back(RayBeneath(rope[k], m_up, length));
				}
			}

			// Plan each tail's curve over the ground, then turn each bone onto the plan. The joints of one tail are adjacent in the query.
			auto hits = ProbeGround(ground);
			for (int j = 0, jend = static_cast<int>(m_tail_joints.size()); j != jend;)
			{
				// Record the ground height beneath each joint. Where no ground is in reach, the ground is far enough below that the tail hangs freely.
				auto c = m_tail_joints[j].chain;
				auto const& chain = m_rig.chains[c];
				auto count = static_cast<int>(chain.bones.size());
				auto& rope = m_ropes[c];
				auto length = TailLength(c, model_to_world);
				auto base_height = Height(rope[0]);
				m_tail_ground.resize(count + 1);
				for (; j != jend && m_tail_joints[j].chain == c; ++j)
					m_tail_ground[m_tail_joints[j].joint] = hits[j].hit ? Height(hits[j].point) : base_height - 2 * length;

				// Measure each bone, its heading across the ground from the dragged rope (or the guide where the rope is vertical), and the point of its
				// skin that must stay above the ground: 'along' the bone from its head and 'depth' below the line of the bone.
				m_tail_bones.resize(count);
				for (int i = 0; i != count; ++i)
				{
					auto guide = TailJointPos(c, i + 1, model_to_world) - TailJointPos(c, i, model_to_world);
					auto bone_length = Length(guide);
					auto& bone = m_tail_bones[i];
					bone = TailBone{
						.across = Normalise(Across(rope[i + 1] - rope[i]), Normalise(Across(guide), Normalise(Perpendicular(m_up)))),
						.length = bone_length,
						.along = bone_length,
						.depth = 0.0f,
						.ground = 0.0f,
					};
					for (auto const& contact : m_rig.contacts)
					{
						// The contact's depth is its distance from the bone's line, because the skin beneath the bone faces the ground
						if (contact.kind != EContactKind::Tail || contact.bone != chain.bones[i])
							continue;

						bone.along = contact.offset.y;
						bone.depth = Sqrt(Sqr(contact.offset.x) + Sqr(contact.offset.z));
						break;
					}
				}

				// Find the droop of each bone when the tail rests on the ground. A tapering tail is thinner at each joint than the one before, so its line
				// slopes down even on flat ground. The skin depth at a joint is interpolated between the skin points on either side of it. The tip is a point.
				auto joint_depth = [&](int j)
				{
					// The base and tip have skin on one side only
					if (j == 0)
						return m_tail_bones[0].depth;
					if (j == count)
						return 0.0f;

					auto const& a = m_tail_bones[j - 1];
					auto const& b = m_tail_bones[j];
					auto gap = (a.length - a.along) + b.along;
					return gap > 0 ? a.depth + (b.depth - a.depth) * (a.length - a.along) / gap : a.depth;
				};
				for (int i = 0; i != count; ++i)
				{
					// The drop between the lowest heights of the bone's joints sets its resting droop
					auto& bone = m_tail_bones[i];
					auto drop = (m_tail_ground[i] + joint_depth(i)) - (m_tail_ground[i + 1] + joint_depth(i + 1));
					bone.ground = std::asin(std::clamp(drop / bone.length, -1.0f, 1.0f));
				}

				// The first bone's droop (radians below the ground plane) is limited by its swing about the guide's first bone.
				// Sample the swing range because the droop is not monotonic over a wide range, and a free joint may hang straight down.
				// The swing axis is the bone's +X axis, made exactly perpendicular to the link in case the link is not along the bone's +Y axis.
				auto const& first = m_guide.model[chain.bones[0]];
				auto link = GuideLink(c);
				auto bone_x = Rotate(first.rot, v4::XAxis());
				auto link_dir = Normalise(link);
				auto const& limit = m_rig.bones[chain.bones[0]].limit.swing;
				auto rest_dir = Normalise(model_to_world * link_dir);
				auto swing_axis = Normalise(model_to_world * Normalise(bone_x - link_dir * Dot(bone_x, link_dir), Perpendicular(link_dir)));
				auto droop_lo = std::numeric_limits<float>::max();
				auto droop_hi = std::numeric_limits<float>::lowest();
				for (int s = 0; s <= TailSwingSamples; ++s)
				{
					auto angle = limit.min_x + (limit.max_x - limit.min_x) * s / TailSwingSamples;
					auto droop = Droop(Rotate(quat(swing_axis, angle), rest_dir));
					droop_lo = std::min(droop_lo, droop);
					droop_hi = std::max(droop_hi, droop);
				}

				// The first bone also bends down from the line of the spine, continued back past the pelvis, by no more than the bend limit, so that
				// the spine and tail form one smooth curve. Later bones may droop as far as the first bone's joint limits allow.
				auto droop_max = std::min(droop_hi, math::constants<float>::tau_by_4);
				if (m_spine >= 0)
				{
					auto spine = model_to_world * (m_targets[m_spine].base.pos - m_targets[m_spine].frame.pos);
					droop_hi = std::max(droop_lo, std::min(droop_hi, Droop(Normalise(spine)) + TailBend));
				}

				// The flattest droop of bone 'i' after a bone drooping by 'prev'. It bends up by at most the bend limit, but no flatter than it would rest
				// on the ground, so that the tail never curls up off the ground. A bone after one that is already flatter than that keeps its line.
				auto flattest = [&](int i, float prev)
				{
					return std::max(prev - TailBend, std::min(m_tail_bones[i].ground, prev));
				};

				// True if bone 'i', with its head at 'height' and drooping by 'droop', and the bones after it flattening as fast as they can, stay clear of the ground
				auto clear = [&](int i, float droop, float height)
				{
					// Walk down the rest of the tail, checking each bone's skin point against the ground interpolated between its joints
					for (;; droop = flattest(i, droop))
					{
						auto const& bone = m_tail_bones[i];
						auto sin = std::sin(droop);
						auto skin = height - bone.along * sin - bone.depth * std::cos(droop);
						auto t = bone.along / bone.length;
						if (skin < m_tail_ground[i] + (m_tail_ground[i + 1] - m_tail_ground[i]) * t)
							return false;

						height -= bone.length * sin;
						if (++i == count)
							return true;
					}
				};

				// Choose each bone's droop in turn: the steepest that stays clear, or the flattest allowed where none does. Clearance only improves as a
				// bone flattens, so a bisection finds the steepest clear droop.
				auto droop = 0.0f;
				auto height = base_height;
				for (int i = 0; i != count; ++i)
				{
					// Bones after the first bend up or down from the bone before by at most the bend limit
					auto lo = i == 0 ? droop_lo : flattest(i, droop);
					auto hi = i == 0 ? std::min(droop_hi, math::constants<float>::tau_by_4) : std::max(droop, std::min(droop + TailBend, droop_max));
					if (clear(i, hi, height))
					{
						droop = hi;
					}
					else if (!clear(i, lo, height))
					{
						droop = lo;
					}
					else
					{
						for (int n = 0; n != TailBisections; ++n)
						{
							auto mid = 0.5f * (lo + hi);
							(clear(i, mid, height) ? lo : hi) = mid;
						}
						droop = lo;
					}

					// Place the bone's end along its heading at the chosen droop
					auto const& bone = m_tail_bones[i];
					auto direction = bone.across * std::cos(droop) - m_up * std::sin(droop);
					rope[i + 1] = rope[i] + direction * bone.length;
					height -= bone.length * std::sin(droop);

					// Turn the bone by the shortest arc from its guide direction onto the planned direction. Its head follows its parent's new pose,
					// which keeps the bone on the rope because the rope's bone lengths come from the same pose.
					auto b = chain.bones[i];
					auto parent = m_rig.bones[b].parent;
					auto frame = parent < 0 ? m_pose.local[b] : m_pose.model[parent] * m_pose.local[b];
					auto turn = quat(Rotate(frame.rot, v4::YAxis()), Normalise(world_to_model * direction));
					m_pose.SetModel(m_rig, b, xform(frame.pos, Normalise(turn * frame.rot)));
				}
			}

			// While the body enters or leaves the water, each tail bone turns from the plan towards its guide shape by how much the body swims
			if (water != 0)
			{
				for (auto const& chain : m_rig.chains)
				{
					if (chain.role != EChainRole::Tail)
						continue;

					for (auto b : chain.bones)
						m_pose.local[b].rot = Slerp(m_pose.local[b].rot, m_guide.local[b].rot, water);
				}
			}
		}

		// The model-space vector of the first bone of tail chain 'c' in the guide pose, from its head to the next joint
		v4 GuideLink(int c) const
		{
			// The next joint is the head of the second bone, or the first bone's tip when there is only one
			auto const& bones = m_rig.chains[c].bones;
			auto const& first = m_guide.model[bones[0]];
			return (bones.size() != 1 ? m_guide.model[bones[1]].pos : first * v4(0, m_rig.bones[bones[0]].length, 0, 1)) - first.pos;
		}

		// The world-space position of joint 'k' of tail chain 'c' in the current pose. Joint 0 is the head of the chain's first bone, and joint 'k'
		// (k >= 1) is the end of chain bone 'k - 1': the head of the next chain bone, or the tip of the last bone.
		v4 TailJointPos(int c, int k, xform const& model_to_world) const
		{
			auto const& bones = m_rig.chains[c].bones;
			auto count = static_cast<int>(bones.size());
			return model_to_world * (k != count ? m_pose.model[bones[k]].pos : m_pose.Tip(m_rig, bones.back()));
		}

		// The length of tail chain 'c' in the current pose, from its base to its tip
		float TailLength(int c, xform const& model_to_world) const
		{
			auto length = 0.0f;
			for (int k = 1, kend = static_cast<int>(m_rig.chains[c].bones.size()); k <= kend; ++k)
				length += Length(TailJointPos(c, k, model_to_world) - TailJointPos(c, k - 1, model_to_world));

			return length;
		}

		// The height of the world-space point 'point' along the body's up direction
		float Height(v4 point) const
		{
			// Measure from the world origin so that heights of different points can be compared
			return Dot(point - v4::Origin(), m_up);
		}

		// The part of the world-space vector 'vec' across the ground, perpendicular to the body's up direction
		v4 Across(v4 vec) const
		{
			// Remove the part along the up direction
			return vec - m_up * Dot(vec, m_up);
		}

		// The angle (radians) by which the unit world-space direction 'dir' points below the ground plane. Negative for directions that point up.
		float Droop(v4 dir) const
		{
			// Clamp against rounding outside the domain of asin
			return std::asin(Clamp(-Dot(dir, m_up), -1.0f, 1.0f));
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
			// A still body on flat ground holds the stance, which for the test gait is the rest pose, and does not step. The tail, which cannot
			// reach the ground, curves down from its base instead.
			auto rig = TestRig();
			auto loco = Locomotion(rig, TestGait());
			auto ground = TestGround();
			for (int i = 0; i != 60; ++i)
				loco.Step(1.0f / 60, BodyState{}, ground);

			PR_EXPECT(loco.phase() == 0.0f);
			auto const& tail = rig.chains[4].bones;
			for (int b = 0, bend = static_cast<int>(rig.bones.size()); b != bend; ++b)
			{
				if (std::find(tail.begin() + 1, tail.end(), b) != tail.end())
					continue;

				PR_EXPECT(FEqlAbsolute(loco.pose().model[b].pos, rig.rest_model[b].pos, 1e-3f));
			}
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
		PRUnitTestMethod(RestartPlantsFeetBeneathBody, Quick)
		{
			// A body that moved far without being stepped plants every foot beneath itself on the first step after a restart
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

			// Jump the body away, then restart and step once
			body.model_to_world.pos += v4(20, 0, 0, 0);
			body.velocity = v4::Zero();
			loco.Restart();
			loco.Step(dt, body, ground);
			for (auto const& limb : loco.limbs())
			{
				auto neutral = body.model_to_world * limb.stance_contact;
				PR_EXPECT(limb.state == EFootState::Stance);
				PR_EXPECT(Length((limb.ground - neutral).w0()) <= 0.15f * walk.stride + 1e-4f);
			}
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
		PRUnitTestMethod(SolesAndDigitsLieOnSlope, Quick)
		{
			// A foot with three toes, the middle one curled up in the rest pose, stands on a slope. The sole faces along the ground's normal
			// and every toe tip touches the ground.
			auto desc = TestRigDesc();
			auto foot = 8;
			auto foot_model = xform::Identity();
			for (auto b = foot; b != -1; b = desc.bones[b].parent)
				foot_model = desc.bones[b].rest * foot_model;

			auto add_toe = [&](char const* name, v4 head, v4 tip)
			{
				// Each toe is one bone hanging from the foot, with a contact at its tip
				auto dir = tip - head;
				auto model = xform(head, quat(v4::YAxis(), Normalise(dir)));
				desc.bones.push_back(Bone{ .name = name, .parent = foot, .rest = Invert(foot_model) * model, .length = Length(dir) });
				auto bone = static_cast<int>(desc.bones.size()) - 1;
				desc.chains.push_back(Chain{ .role = EChainRole::Digit, .side = ESide::Left, .bones = { bone } });
				desc.contacts.push_back(Contact{ .name = std::string(name) + ".tip", .bone = bone, .offset = v4(0, Length(dir), 0, 1), .normal = Invert(model) * v4(0, 0, -1, 0), .kind = EContactKind::Sole });
			};
			add_toe("toe.0.L", v4(0.15f, -0.2f, 0, 1), v4(0.1f, -0.3f, 0, 1));
			add_toe("toe.1.L", v4(0.2f, -0.2f, 0, 1), v4(0.2f, -0.3f, 0.05f, 1));
			add_toe("toe.2.L", v4(0.25f, -0.2f, 0, 1), v4(0.3f, -0.3f, 0, 1));
			desc.contacts.push_back(Contact{ .name = "grip.L", .bone = foot, .offset = v4(0.02f, 0.15f, -0.01f, 1), .normal = v4(0, 0, -1, 0), .kind = EContactKind::Grip });
			auto rig = Rig(desc);

			// Stand still on the slope until the feet settle
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround(0.2f);
			auto normal = Normalise(v4(0, -0.2f, 1, 0));
			auto dt = 1.0f / 60;
			auto body = BodyState{};
			for (int i = 0; i != 180; ++i)
				loco.Step(dt, body, ground);

			auto const& leg = loco.limbs()[0];
			PR_EXPECT(leg.digits.size() == 3);
			PR_EXPECT(FEqlAbsolute(Rotate(loco.pose().model[foot].rot, leg.sole_up), normal, 1e-3f));
			for (auto const& contact : rig.contacts)
			{
				if (contact.name.ends_with(".tip"))
				{
					auto tip = loco.pose().model[contact.bone] * contact.offset;
					PR_EXPECT(FEqlAbsolute(Dot(tip.w0(), normal), 0.0f, 1e-3f));
				}
			}

			// The grip point is the end bone's authored grip contact
			PR_EXPECT(FEqlAbsolute(leg.grip, rig.contacts.back().offset, 1e-6f));
		}
		PRUnitTestMethod(ModelTransformsAreCurrent, Quick)
		{
			// While walking on a slope, every bone's model transform after each step matches its local transform under its parent's model transform,
			// including a digit under the foot and the tail
			auto desc = TestRigDesc();
			auto foot = 8;
			auto foot_model = xform::Identity();
			for (auto b = foot; b != -1; b = desc.bones[b].parent)
				foot_model = desc.bones[b].rest * foot_model;

			auto toe_model = xform(v4(0.2f, -0.2f, 0, 1), quat(v4::YAxis(), v4(0, -1, 0, 0)));
			desc.bones.push_back(Bone{ .name = "toe.L", .parent = foot, .rest = Invert(foot_model) * toe_model, .length = 0.1f });
			desc.chains.push_back(Chain{ .role = EChainRole::Digit, .side = ESide::Left, .bones = { static_cast<int>(desc.bones.size()) - 1 } });
			auto rig = Rig(desc);

			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround(0.2f);
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };
			for (int i = 0; i != 120; ++i)
			{
				// Compare each bone with a recompute from its parent
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				auto const& pose = loco.pose();
				for (int b = 0, bend = static_cast<int>(rig.bones.size()); b != bend; ++b)
				{
					auto parent = rig.bones[b].parent;
					auto expected = parent < 0 ? pose.local[b] : pose.model[parent] * pose.local[b];
					PR_EXPECT(FEqlAbsolute(pose.model[b].pos, expected.pos, 1e-4f) && FEqlOrientation(pose.model[b].rot, expected.rot, 1e-3f));
				}
			}
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
		PRUnitTestMethod(SpineSwaysWithLimbs, Quick)
		{
			// Walking straight, the pelvis turns with the leg and the chest with the arm, half a cycle apart, so they turn in opposite directions.
			// Each girdle moves its limb's side forward when the foot lands. Standing still, the sway dies away.
			auto rig = TestRig();
			auto gait = TestGait();
			auto& walk = gait.families[0].cycles[0];
			walk.motion.clear();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };

			// The pelvis and chest bones point forward (-Y), so a turn by 'a' about +Z points them along (sin a, -cos a)
			auto yaw = [](xform const& frame)
			{
				auto dir = Rotate(frame.rot, v4::YAxis());
				return std::atan2(dir.x, -dir.y);
			};
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}

			// The leg is on the +X side and lands at phase 0; the arm lands half a cycle later
			for (int i = 0; i != 60; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				auto expected = walk.spine_sway * std::cos(math::constants<float>::tau * loco.phase());
				PR_EXPECT(FEqlAbsolute(yaw(loco.targets()[0].base), -expected, 5e-3f));
				PR_EXPECT(FEqlAbsolute(yaw(loco.targets()[0].frame), +expected, 5e-3f));
			}

			// Standing still, the sway dies away
			body.velocity = v4::Zero();
			for (int i = 0; i != 300; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(FEqlAbsolute(yaw(loco.targets()[0].base), 0.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(yaw(loco.targets()[0].frame), 0.0f, 1e-3f));
		}
		PRUnitTestMethod(BipedShiftsWeightAndSwingsArms, Quick)
		{
			// Walking on the hind legs, the pelvis and chest move sideways over the planted leg, most strongly at the middle of its stance, and
			// the hanging arm swings about its shoulder against the leg on its side. Standing still, both die away.
			auto rig = TestRig();
			auto gait = TestGait();
			auto biped = FamilyIndex(gait, "bipedal");
			auto const& walk = gait.families[biped].cycles[0];
			auto const& leg = walk.footfalls[0];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0), .gravity = v4(0, 0, -9.8f, 0) };
			loco.RequestFamily(biped, 0.5f);
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			PR_EXPECT(loco.family_settled() && loco.family() == biped);

			// The arm hangs straight down from its shoulder at (0.2,-0.9,1) to the hand at (0.2,-0.9,0), so turning it by 'a' about +X moves
			// the hand to y = -0.9 + sin(a)
			auto const& arm = loco.limbs()[1];
			auto contact_x = loco.limbs()[0].stance_contact.x;
			for (int i = 0; i != 60; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				auto tau = math::constants<float>::tau;
				auto shift = walk.weight_shift * contact_x * std::cos(tau * (loco.phase() - leg.touchdown - 0.5f * leg.duty));
				auto swing = walk.arm_swing * std::cos(tau * (loco.phase() - leg.touchdown));
				PR_EXPECT(FEqlAbsolute(loco.targets()[0].base.pos.x, shift, 5e-3f));
				PR_EXPECT(FEqlAbsolute(loco.targets()[0].frame.pos.x, shift, 5e-3f));
				PR_EXPECT(FEqlAbsolute(loco.targets()[arm.chain].frame.pos.y, -0.9f + std::sin(swing), 5e-3f));
			}

			// Standing still, the body returns over the midline and the arm hangs straight
			body.velocity = v4::Zero();
			for (int i = 0; i != 300; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(FEqlAbsolute(loco.targets()[0].base.pos.x, 0.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(loco.targets()[arm.chain].frame.pos.y, -0.9f, 1e-3f));
		}
		PRUnitTestMethod(SpineLeadsTurns, Quick)
		{
			// Turning on the spot to the left (+Z), the chest turns into the turn and the pelvis away from it. They straighten once the turn stops.
			auto rig = TestRig();
			auto gait = TestGait();
			for (auto& cycle : gait.families[0].cycles)
			{
				cycle.motion.clear();
				cycle.spine_sway = 0.0f;
			}
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .angular_velocity = v4(0, 0, 1.0f, 0) };
			auto yaw = [](xform const& frame)
			{
				auto dir = Rotate(frame.rot, v4::YAxis());
				return std::atan2(dir.x, -dir.y);
			};
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.rot = Normalise(quat(v4::ZAxis(), body.angular_velocity.z * dt) * body.model_to_world.rot);
				loco.Step(dt, body, ground);
			}
			PR_EXPECT(yaw(loco.targets()[0].frame) > 0.2f);
			PR_EXPECT(yaw(loco.targets()[0].base) < -0.2f);

			body.angular_velocity = v4::Zero();
			for (int i = 0; i != 300; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(FEqlAbsolute(yaw(loco.targets()[0].base), 0.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(yaw(loco.targets()[0].frame), 0.0f, 1e-3f));
		}
		PRUnitTestMethod(SuddenTurnsStepTwistedFeet, Quick)
		{
			// A body that turns suddenly while both feet are planted lifts one twisted foot out of turn at once, keeping the other planted for support.
			// Once the body stands still, the feet settle square beneath it, including after a turn too small to strain them.
			auto rig = TestRig();
			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };
			loco.Step(0, body, ground);
			auto rest = std::vector<quat>{};
			for (auto const& limb : loco.limbs())
				rest.push_back(limb.foot.rot);

			auto twist = [&](int l)
			{
				// The foot's yaw, about world up, away from its rest orientation carried round with the body
				auto rel = loco.limbs()[l].foot.rot * ~(body.model_to_world.rot * rest[l]);
				return std::abs(2.0f * std::atan2(rel.w < 0 ? -rel.z : rel.z, std::abs(rel.w)));
			};
			auto planted = [&]
			{
				// True while every foot is on the ground
				return std::all_of(loco.limbs().begin(), loco.limbs().end(), [](LimbState const& limb) { return limb.state == EFootState::Stance; });
			};

			// Walk until both feet are planted, then turn the body sharply in one step
			for (int i = 0; i != 60 || !planted(); ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			body.model_to_world.rot = Normalise(quat(v4::ZAxis(), 0.8f) * body.model_to_world.rot);
			loco.Step(dt, body, ground);

			auto swinging = 0;
			for (auto const& limb : loco.limbs())
				swinging += limb.state == EFootState::Swing ? 1 : 0;

			PR_EXPECT(swinging == 1);

			// Stand still until the feet settle
			body.velocity = v4::Zero();
			for (int i = 0; i != 600; ++i)
				loco.Step(dt, body, ground);

			for (int l = 0, lend = static_cast<int>(loco.limbs().size()); l != lend; ++l)
			{
				PR_EXPECT(loco.limbs()[l].state == EFootState::Stance);
				PR_EXPECT(twist(l) <= 0.2f + 1e-3f);
			}

			// A standing body that turns a little on the spot steps its turned feet square again
			body.model_to_world.rot = Normalise(quat(v4::ZAxis(), 0.3f) * body.model_to_world.rot);
			for (int i = 0; i != 600; ++i)
				loco.Step(dt, body, ground);

			for (int l = 0, lend = static_cast<int>(loco.limbs().size()); l != lend; ++l)
			{
				PR_EXPECT(loco.limbs()[l].state == EFootState::Stance);
				PR_EXPECT(twist(l) <= 0.2f + 1e-3f);
			}
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
			auto body = BodyState{ .velocity = v4(0, -gait.families[0].cycles[0].speed, 0, 0), .gravity = v4(0, 0, -9.8f, 0) };
			loco.Step(0, body, ground);
			PR_EXPECT(loco.limbs().size() == 2 && loco.limbs()[1].support == 1.0f);

			loco.RequestFamily(FamilyIndex(gait, "bipedal"), 0.5f);
			PR_EXPECT(loco.family() == 0);
			PR_EXPECT(!loco.family_settled());
			for (int i = 0; i != 15; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			PR_EXPECT(loco.family() == 1);
			PR_EXPECT(!loco.family_settled());
			auto const& arm = loco.limbs()[1];
			PR_EXPECT(arm.support > 0.0f && arm.support < 1.0f);

			for (int i = 0; i != 60; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			PR_EXPECT(loco.family_settled());
			PR_EXPECT(arm.support == 0.0f && arm.state == EFootState::Stance);
			PR_EXPECT(loco.limbs()[0].support == 1.0f);

			// Standing still, the hanging hand returns to its place in the bipedal stance, which is the rest pose
			body.velocity = v4::Zero();
			for (int i = 0; i != 300; ++i)
				loco.Step(dt, body, ground);

			auto hand = rig.BoneIndex("hand.L");
			PR_EXPECT(FEqlAbsolute(loco.pose().model[hand].pos, rig.rest_model[hand].pos, 1e-3f));
		}
		PRUnitTestMethod(FamilySlopeLimit, Quick)
		{
			// The bipedal family is refused without gravity and on ground steeper than its slope limit, where the character falls back to the
			// quadruped family. It is used again once the ground is level enough, while the request stands.
			auto rig = TestRig();
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto biped = FamilyIndex(gait, "bipedal");
			auto limit = gait.families[biped].max_slope;
			auto dt = 1.0f / 60;
			auto level = TestGround();
			auto steep = TestGround(std::tan(limit + 0.1f));
			auto body = BodyState{};
			loco.RequestFamily(biped, 0.5f);
			loco.Step(dt, body, level);
			PR_EXPECT(loco.family() == 0);

			body.gravity = v4(0, 0, -9.8f, 0);
			loco.Step(dt, body, level);
			PR_EXPECT(loco.family() == biped);

			loco.Step(dt, body, steep);
			PR_EXPECT(loco.family() == 0);

			// A request that the ground refuses still settles, in the family that the ground allows
			for (int i = 0; i != 60; ++i)
				loco.Step(dt, body, steep);

			PR_EXPECT(loco.family_settled() && loco.family() == 0);
			// Ground just inside the limit does not bring the bipedal family back, because changing family needs a margin
			auto inside = TestGround(std::tan(limit - 0.01f));
			loco.Step(dt, body, inside);
			PR_EXPECT(loco.family() == 0);

			loco.Step(dt, body, level);
			PR_EXPECT(loco.family() == biped);
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

			// A new look point moves the gaze there smoothly rather than at once
			auto before = facing();
			auto moved = v4(3, 1.1f, 1.2f, 1);
			loco.SetLook(moved, 0.25f);
			loco.Step(1.0f / 60, BodyState{}, ground);
			PR_EXPECT(Dot(facing(), before) > 0.99f);
			for (int i = 0; i != 60; ++i)
				loco.Step(1.0f / 60, BodyState{}, ground);

			PR_EXPECT(Dot(facing(), Normalise(moved - loco.pose().model[head].pos)) > 0.95f);

			loco.SetLook(std::nullopt, 0.25f);
			for (int i = 0; i != 60; ++i)
				loco.Step(1.0f / 60, BodyState{}, ground);

			PR_EXPECT(Dot(facing(), neck.forward) > 0.999f);
		}
		PRUnitTestMethod(HandsReachForGrasps, Quick)
		{
			// Standing on its hind legs, a hand moves its palm onto a grasp facing the grasp's direction. A grasp beyond the arm's reach makes the body
			// stoop just enough, with the foot kept planted, and letting go returns the body and the hand to where they were.
			auto rig = TestRig();
			auto gait = TestGait();
			auto biped = FamilyIndex(gait, "bipedal");
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .gravity = v4(0, 0, -9.8f, 0) };
			loco.RequestFamily(biped, 0.25f);
			for (int i = 0; i != 120; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(loco.family_settled() && loco.family() == biped);
			auto rest = loco.pose().model;

			// The grip point is on the hand bone, and the palm faces away from the limb's sole normal. The test rig's hand has no digits, so it grips at its
			// contact. The body is at the origin, so model space is world space.
			auto const& arm = loco.limbs()[1];
			auto hand = rig.chains[arm.chain].bones.back();
			auto foot = rig.chains[loco.limbs()[0].chain].bones.back();
			PR_EXPECT(FEqlAbsolute(arm.grip, rig.contacts[arm.contact].offset, 1e-6f));
			auto palm_point = [&] { return loco.pose().model[hand] * arm.grip; };
			auto palm_facing = [&] { return Rotate(loco.pose().model[hand].rot, -arm.sole_up); };
			auto finger_pointing = [&] { return Rotate(loco.pose().model[hand].rot, v4::YAxis()); };

			// A grasp within reach needs no stoop, and the hand turns so its fingers point along the grasp's finger direction
			auto within = Grasp{ .point = v4(0.2f, -1.3f, 0.5f, 1), .palm = v4(-1, 0, 0, 0), .fingers = v4(0, -1, 0, 0) };
			loco.SetReach(ESide::Left, within, 0.25f);
			for (int i = 0; i != 60; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(FEqlAbsolute(palm_point(), within.point, 1e-2f));
			PR_EXPECT(Dot(palm_facing(), within.palm) > 0.99f);
			PR_EXPECT(Dot(finger_pointing(), within.fingers) > 0.9f);
			PR_EXPECT(loco.stoop() == 0.0f);

			// A grasp near the ground beyond the arm's reach lowers the body, with the foot where it was
			auto low = Grasp{ .point = v4(0.2f, -1.3f, 0.02f, 1), .palm = v4(-1, 0, 0, 0) };
			loco.SetReach(ESide::Left, low, 0.25f);
			for (int i = 0; i != 120; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(loco.stoop() > 0.01f && loco.stoop() < 0.5f);
			PR_EXPECT(loco.targets()[0].base.pos.z < rest[rig.chains[0].bones.front()].pos.z - 0.01f);
			PR_EXPECT(FEqlAbsolute(palm_point(), low.point, 1e-2f));
			PR_EXPECT(FEqlAbsolute(loco.pose().model[foot].pos, rest[foot].pos, 1e-3f));

			// Letting go returns to the pose before the reach
			loco.SetReach(ESide::Left, std::nullopt, 0.25f);
			for (int i = 0; i != 180; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(loco.stoop() < 1e-3f);
			for (int b = 0, bend = static_cast<int>(rig.bones.size()); b != bend; ++b)
				PR_EXPECT(FEqlAbsolute(loco.pose().model[b].pos, rest[b].pos, 2e-3f));
		}
		PRUnitTestMethod(GraspingElbowsStayOutsideTorso, Quick)
		{
			// A hand that grasps a point close in front of the body keeps its palm on the grasp, while its elbow stays outside the capsule around
			// the spine that reaches the arm's root in the rest pose. The test rig's arm root is 0.2 from its spine.
			auto rig = TestRig();
			auto gait = TestGait();
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .gravity = v4(0, 0, -9.8f, 0) };
			loco.RequestFamily(FamilyIndex(gait, "bipedal"), 0.25f);
			for (int i = 0; i != 120; ++i)
				loco.Step(dt, body, ground);

			// Grasp just beside the chest, nearer the midline than the arm's root
			auto const& arm = loco.limbs()[1];
			auto const& bones = rig.chains[arm.chain].bones;
			auto spine_a = loco.pose().model[1].pos;
			auto spine_b = loco.pose().Tip(rig, 3);
			auto grasp = Grasp{ .point = Lerp(spine_a, spine_b, 0.6f) + v4(0.1f, -0.25f, 0, 0), .palm = v4(-1, 0, 0, 0), .fingers = v4(0, -1, 0, 0) };
			loco.SetReach(ESide::Left, grasp, 0.25f);
			for (int i = 0; i != 60; ++i)
				loco.Step(dt, body, ground);

			// Measure the elbow against the posed spine
			spine_a = loco.pose().model[1].pos;
			spine_b = loco.pose().Tip(rig, 3);
			auto elbow = loco.pose().model[bones[1]].pos;
			auto ab = spine_b - spine_a;
			auto t = Clamp(Dot(elbow - spine_a, ab) / LengthSq(ab), 0.0f, 1.0f);
			PR_EXPECT(Length(elbow - (spine_a + ab * t)) >= 0.2f - 1e-3f);
			PR_EXPECT(FEqlAbsolute(loco.pose().model[bones[2]] * arm.grip, grasp.point, 1e-2f));
		}
		PRUnitTestMethod(GraspWrapCurlsDigits, Quick)
		{
			// A hand with two-bone fingers grasps the same point with and without a wrap radius. With one, every finger tip turns in towards the palm.
			auto desc = TestRigDesc();
			auto hand = 11;
			auto hand_model = xform::Identity();
			for (auto b = hand; b != -1; b = desc.bones[b].parent)
				hand_model = desc.bones[b].rest * hand_model;

			auto add_finger = [&](char const* name, float x)
			{
				// Each finger is two bones continuing from the hand's tip
				auto model0 = xform(v4(x, -1.0f, 0, 1), quat(v4::YAxis(), v4(0, -1, 0, 0)));
				auto model1 = xform(v4(x, -1.05f, 0, 1), model0.rot);
				desc.bones.push_back(Bone{ .name = std::string(name) + ".0", .parent = hand, .rest = Invert(hand_model) * model0, .length = 0.05f });
				auto b0 = static_cast<int>(desc.bones.size()) - 1;
				desc.bones.push_back(Bone{ .name = std::string(name) + ".1", .parent = b0, .rest = Invert(model0) * model1, .length = 0.05f });
				desc.chains.push_back(Chain{ .role = EChainRole::Digit, .side = ESide::Left, .bones = { b0, b0 + 1 } });
				return b0 + 1;
			};
			int const tips[] = { add_finger("finger.a.L", 0.18f), add_finger("finger.b.L", 0.22f) };
			auto rig = Rig(desc);

			// Hold the grasp until it settles, then measure how far each tip is towards the palm from the grip point, and how far it is outside the bar's skin
			struct Tip { float towards_palm; float outside_bar; };
			auto measure = [&](float wrap)
			{
				auto gait = TestGait();
				auto loco = Locomotion(rig, gait);
				auto ground = TestGround();
				auto dt = 1.0f / 60;
				auto body = BodyState{ .gravity = v4(0, 0, -9.8f, 0) };
				loco.RequestFamily(FamilyIndex(gait, "bipedal"), 0.25f);
				loco.SetReach(ESide::Left, Grasp{ .point = v4(0.2f, -1.3f, 0.5f, 1), .palm = v4(-1, 0, 0, 0), .fingers = v4(0, -1, 0, 0), .wrap = wrap }, 0.25f);
				for (int i = 0; i != 120; ++i)
					loco.Step(dt, body, ground);

				auto const& arm = loco.limbs()[1];
				auto const& h = loco.pose().model[hand];
				auto palm = Rotate(h.rot, -arm.sole_up);
				auto along = Rotate(h.rot, Normalise(Cross(-arm.sole_up, v4::YAxis())));
				auto centre = h * arm.grip + palm * wrap;
				std::vector<Tip> d;
				for (auto tip : tips)
				{
					auto p = loco.pose().Tip(rig, tip);
					auto r = p - centre;
					d.push_back(Tip{
						.towards_palm = Dot(p - h * arm.grip, palm),
						.outside_bar = Length(r - along * Dot(r, along)) - wrap - arm.skin,
					});
				}
				return d;
			};

			// With a wrap radius the tips curl in until they touch the bar, or as near to it as they reach, without passing into it
			auto open = measure(0.0f);
			auto closed = measure(0.03f);
			for (int i = 0; i != 2; ++i)
			{
				PR_EXPECT(closed[i].towards_palm > open[i].towards_palm + 0.03f);
				PR_EXPECT(closed[i].outside_bar > -2e-3f && closed[i].outside_bar < 0.01f);
			}
		}
		PRUnitTestMethod(BreathingSwellsTheChest, Quick)
		{
			// At the peak of a breath the chest bone has turned about its head so that its tip has moved out from the front of the body, which for
			// the test rig's level spine is down, and it is back where it started at the end of the breath
			auto rig = TestRig();
			auto still = Locomotion(rig, TestGait());
			auto breathing = Locomotion(rig, TestGait());
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto tip = [&](Locomotion const& loco)
			{
				// The model-space tip of the chest bone (test rig bone 3)
				auto const& chest = loco.targets()[0].frame;
				return chest.pos + Rotate(chest.rot, v4::YAxis()) * rig.bones[3].length;
			};
			breathing.SetBreath(Breath{ .rate = 1.0f, .chest = 0.02f, .shoulders = 0.1f });
			for (int i = 0; i != 30; ++i)
			{
				still.Step(dt, BodyState{}, ground);
				breathing.Step(dt, BodyState{}, ground);
			}
			PR_EXPECT(FEqlAbsolute(breathing.targets()[0].frame.pos, still.targets()[0].frame.pos, 1e-4f));
			PR_EXPECT(FEqlAbsolute(tip(breathing) - tip(still), v4(0, 0, -0.02f, 0), 1e-3f));

			// The reported chest motion moves a point on the still chest to where the breathing chest has it
			PR_EXPECT(FEqlAbsolute(breathing.chest_motion() * tip(still), tip(breathing), 1e-4f));
			PR_EXPECT(FEqlAbsolute(still.chest_motion().pos, v4::Origin(), 1e-6f));

			for (int i = 0; i != 30; ++i)
			{
				still.Step(dt, BodyState{}, ground);
				breathing.Step(dt, BodyState{}, ground);
			}
			PR_EXPECT(FEqlAbsolute(tip(breathing), tip(still), 1e-4f));
		}
		PRUnitTestMethod(SwingingFeetMoveOutAndTurnFreely, Quick)
		{
			// Mid-swing, feet are further from the body's centre line than where they lift off and land, and turn freely with their limb.
			// Feet on the ground are not free to turn.
			auto rig = TestRig();
			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };
			auto mid_swings = 0;
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				for (auto const& limb : loco.limbs())
				{
					if (limb.state != EFootState::Swing)
					{
						PR_EXPECT(limb.free == 0.0f);
						continue;
					}
					if (limb.swing < 0.4f || limb.swing > 0.6f)
						continue;

					PR_EXPECT(std::abs(limb.foot.pos.x) > std::max(std::abs(limb.lift.pos.x), std::abs(limb.land.pos.x)) + 0.5f * limb.footfall.swing_out);
					PR_EXPECT(limb.free > 0.9f);
					++mid_swings;
				}
			}
			PR_EXPECT(mid_swings != 0);
		}
		PRUnitTestMethod(TailTrailsBehind, Quick)
		{
			// The test rig's tail is too short to reach flat ground from the hips, so it curves down towards it. With a raised ridge beneath the tail,
			// between the feet, the tail curves down onto the ridge behind the body when walking forward, and a high ridge makes it lie along the ridge.
			// In every case, the tail bends by no more than the bend limit at each joint, droops no more than the first bone's joint limits, never
			// flattens and then droops again in a hump, and the tail's skin contacts stay above the ground.
			struct RidgeGround : IGround
			{
				// Flat ground at height 0, with a ridge of height 'm_height' along the Y axis that is narrower than the stance
				float m_height;

				explicit RidgeGround(float height)
					: m_height(height)
				{
				}
				void Probe(std::span<GroundRay const> rays, std::span<GroundHit> hits) const override
				{
					// Only vertical rays are needed by the tail, so measure each ray's drop to the ground beneath its origin
					for (size_t i = 0; i != rays.size(); ++i)
					{
						auto const& ray = rays[i];
						auto height = std::abs(ray.origin.x) < 0.15f ? m_height : 0.0f;
						auto t = Clamp((ray.origin.z - height) / -ray.direction.z, 0.0f, ray.length + 1);
						hits[i] = t <= ray.length
							? GroundHit{ .point = ray.origin + ray.direction * t, .normal = v4::ZAxis(), .hit = true }
							: GroundHit{ .point = ray.origin + ray.direction * ray.length, .normal = -ray.direction, .hit = false };
					}
				}
			};

			auto rig = TestRig();
			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto const& tail = rig.chains[4];
			auto dt = 1.0f / 60;

			// The first tail bone can droop by up to 0.6 radians, and each tail bone has a skin contact beneath its middle. The tail tapers, so the
			// skin is shallower beneath each bone than the one before.
			auto depth = [](int k) { return 0.06f - 0.02f * k; };
			auto max_droop = 0.6f;
			for (int k = 0; k != static_cast<int>(tail.bones.size()); ++k)
			{
				auto b = tail.bones[k];
				rig.bones[b].limit.swing = { .min_x = -max_droop, .max_x = max_droop, .min_z = -max_droop, .max_z = max_droop };
				rig.contacts.push_back(Contact{ .name = rig.bones[b].name + ".skin", .bone = b, .offset = v4(0, 0.15f, -depth(k), 1), .normal = v4(0, 0, -1, 0), .kind = EContactKind::Tail });
			}

			for (auto ridge : { 0.0f, 0.7f, 0.85f })
			{
				auto loco = Locomotion(rig, gait);
				auto ground = RidgeGround(ridge);
				auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };
				for (int i = 0; i != 120; ++i)
				{
					body.model_to_world.pos += body.velocity * dt;
					loco.Step(dt, body, ground);
				}

				// Check that the tail bends within the bend limit, runs behind the body, and keeps its skin above the ground
				auto count = static_cast<int>(tail.bones.size());
				auto joint = [&](int k) { return k != count ? loco.pose().model[tail.bones[k]].pos : loco.pose().Tip(rig, tail.bones.back()); };
				auto droop = [&](int k) { auto d = Normalise(joint(k + 1) - joint(k)); return std::asin(-d.z); };
				auto clearance = std::numeric_limits<float>::max();
				for (int k = 0; k != count; ++k)
				{
					PR_EXPECT(droop(k) < max_droop + 0.02f);
					PR_EXPECT(k == 0 || droop(k) < droop(k - 1) + 0.35f + 0.02f);
					PR_EXPECT(k == 0 || droop(k) > droop(k - 1) - 0.35f - 0.02f);
					PR_EXPECT(joint(k + 1).y > joint(k).y);
					PR_EXPECT(k == 0 || k == count - 1 || droop(k) > std::min(droop(k - 1), droop(k + 1)) - 0.02f);
					clearance = std::min(clearance, (loco.pose().model[tail.bones[k]] * v4(0, 0.15f, -depth(k), 1)).z - ridge);
				}
				PR_EXPECT(clearance > -0.01f);
				if (ridge == 0)
					PR_EXPECT(joint(count).z < joint(0).z - 0.3f);
				else
					PR_EXPECT(clearance < 0.01f);

				// The first bone bends down from the line of the spine, from the chest back through the pelvis, by no more than the bend limit
				auto spine = Normalise(loco.targets()[0].base.pos - loco.targets()[0].frame.pos);
				PR_EXPECT(droop(0) < std::asin(-spine.z) + 0.35f + 0.02f);
			}
		}
		PRUnitTestMethod(TailWavesWithHips, Quick)
		{
			// Walking straight, the first tail bone turns with the swaying hips, and the sway travels down the tail, so the tip swings too and at
			// times bends the other way from the base
			auto rig = TestRig();
			auto gait = TestGait();
			auto const& walk = gait.families[0].cycles[0];
			auto const& tail = rig.chains[4];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -walk.speed, 0, 0) };

			// The tail points backwards (+Y), so a turn by 'a' about +Z points a bone across the ground along (-sin a, cos a)
			auto count = static_cast<int>(tail.bones.size());
			auto joint = [&](int k) { return k != count ? loco.pose().model[tail.bones[k]].pos : loco.pose().Tip(rig, tail.bones.back()); };
			auto heading = [&](int k) { auto d = joint(k + 1) - joint(k); return std::atan2(-d.x, d.y); };
			auto pelvis_yaw = [&] { auto d = Rotate(loco.targets()[0].base.rot, v4::YAxis()); return std::atan2(d.x, -d.y); };
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}

			// Over two cycles, compare the first bone with the hips, and record the swing of the last bone
			auto tip_lo = std::numeric_limits<float>::max();
			auto tip_hi = std::numeric_limits<float>::lowest();
			auto opposed = false;
			auto base_hi = 0.0f;
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
				auto tip = heading(count - 1);
				base_hi = std::max(base_hi, std::abs(heading(0)));
				tip_lo = std::min(tip_lo, tip);
				tip_hi = std::max(tip_hi, tip);
				opposed |= heading(0) * tip < -1e-4f;
				PR_EXPECT(FEqlAbsolute(heading(0), pelvis_yaw(), 0.02f));
			}
			PR_EXPECT(base_hi > 0.5f * walk.spine_sway);
			PR_EXPECT(tip_lo < -0.02f && tip_hi > 0.02f);
			PR_EXPECT(opposed);
		}
		PRUnitTestMethod(SwimmingWavesTheBody, Quick)
		{
			// A swimming body, held up by the water, never spreads its limbs as if falling. No limb walks, so every limb hangs. The body wave
			// swings the tail tip from side to side, further than the tail base, and the tail floats rather than curving down to the ground.
			// The strokes follow the stroke speed rather than the measured velocity, so a body held still in the water still waves. A swimmer that
			// is not stroking anywhere keeps stroking at its slowest cycle, so it waves too.
			auto rig = TestRig();
			auto gait = TestGait();
			auto swim = FamilyIndex(gait, "swim");
			auto const& glide = gait.families[swim].cycles.back();
			auto const& tail = rig.chains[4];
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			for (auto stroke_speed : { glide.speed, 0.0f })
			{
				// Settle into the swim family
				auto loco = Locomotion(rig, gait);
				auto body = BodyState{ .supported = false, .stroke_speed = stroke_speed };
				loco.RequestFamily(swim, 0.5f);
				for (int i = 0; i != 120; ++i)
					loco.Step(dt, body, ground);

				PR_EXPECT(loco.family() == swim && loco.family_settled());

				// Over two cycles, record the sideways swing of the tail's base and tip
				auto tip_lo = std::numeric_limits<float>::max();
				auto tip_hi = std::numeric_limits<float>::lowest();
				auto base_hi = 0.0f;
				for (int i = 0; i != 120; ++i)
				{
					loco.Step(dt, body, ground);
					for (auto const& limb : loco.limbs())
						PR_EXPECT(limb.support == 0.0f && limb.state == EFootState::Stance);

					auto tip = loco.pose().Tip(rig, tail.bones.back());
					tip_lo = std::min(tip_lo, tip.x);
					tip_hi = std::max(tip_hi, tip.x);
					base_hi = std::max(base_hi, std::abs(loco.pose().model[tail.bones.front()].pos.x));
					PR_EXPECT(tip.z > rig.rest_model[tail.bones.front()].pos.z - 0.5f * rig.bones[tail.bones.front()].length);
				}
				PR_EXPECT(tip_lo < -0.02f && tip_hi > 0.02f);
				PR_EXPECT(tip_hi - tip_lo > 2 * base_hi);
			}
		}
		PRUnitTestMethod(SwimmerRightsItself, Quick)
		{
			// A swimmer rolled onto its +X side and nose down reaches every limb towards its lowest side when it rights itself: each limb end moves
			// towards +X and forwards (-Y) compared with the same body that does not right itself. A body swinging back through upright reaches the other way.
			auto rig = TestRig();
			auto gait = TestGait();
			auto swim = FamilyIndex(gait, "swim");
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto rot = quat(v4::YAxis(), 0.6f) * quat(v4::XAxis(), 0.3f);
			auto still = Locomotion(rig, gait);
			auto righting = Locomotion(rig, gait);
			auto body = BodyState{ .model_to_world = xform(v4::Origin(), rot), .supported = false, .gravity = v4(0, 0, -9.8f, 0) };
			still.RequestFamily(swim, 0.5f);
			righting.RequestFamily(swim, 0.5f);
			for (int i = 0; i != 120; ++i)
			{
				still.Step(dt, body, ground);
				righting.Step(dt, BodyState{ .model_to_world = body.model_to_world, .supported = false, .gravity = body.gravity, .calm_tilt = 0.0f, .rough_tilt = 0.1f }, ground);
			}

			// Compare the limb ends of the two bodies
			PR_EXPECT(!righting.limbs().empty());
			for (auto const& limb : righting.limbs())
			{
				auto end = rig.chains[limb.chain].bones.back();
				auto delta = righting.pose().model[end].pos - still.pose().model[end].pos;
				PR_EXPECT(delta.x > 0.01f);
				PR_EXPECT(delta.y < -0.005f);
			}

			// The same lean while the body swings back upright fast enough to pass through it within the lead time turns the limbs the other way
			auto swinging = Locomotion(rig, gait);
			swinging.RequestFamily(swim, 0.5f);
			for (int i = 0; i != 120; ++i)
				swinging.Step(dt, BodyState{ .model_to_world = body.model_to_world, .angular_velocity = v4(0, -4.0f, 0, 0), .supported = false, .gravity = body.gravity, .calm_tilt = 0.0f, .rough_tilt = 0.1f }, ground);

			for (auto const& limb : swinging.limbs())
			{
				auto end = rig.chains[limb.chain].bones.back();
				PR_EXPECT(swinging.pose().model[end].pos.x - still.pose().model[end].pos.x < -0.01f);
			}
		}
		PRUnitTestMethod(CollapseLiesDownAndGetsUp, Quick)
		{
			// A collapse ends with the body lying flat on its side on the ground, as its posture describes. Leaving the family plays the collapse
			// backwards, so the body stands up again in the stance.
			auto rig = TestRig();
			auto gait = TestActionGait();
			auto death = FamilyIndex(gait, "death");
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .gravity = v4(0, 0, -9.8f, 0) };
			auto tilt = [&](std::span<xform const> model)
			{
				// The angle of the leg, from the foot up to the hip, from model +Z
				auto from = model[rig.BoneIndex("foot.L")].pos;
				auto to = model[rig.BoneIndex("thigh.L")].pos;
				return std::acos(Clamp(Normalise(to - from).z, -1.0f, 1.0f));
			};
			for (int i = 0; i != 60; ++i)
				loco.Step(dt, body, ground);

			auto stand_tilt = tilt(loco.pose().model);
			loco.RequestFamily(death, 0.5f);
			for (int i = 0; i != 180; ++i)
				loco.Step(dt, body, ground);

			// Lying flat, with no joint below the ground
			PR_EXPECT(loco.family() == death);
			PR_EXPECT(std::abs(tilt(loco.pose().model) - math::constants<float>::tau_by_4) < 0.3f);
			PR_EXPECT(std::abs(tilt(loco.posture(death)) - math::constants<float>::tau_by_4) < 0.3f);
			for (auto const& xf : loco.pose().model)
				PR_EXPECT(xf.pos.z > -0.01f);

			// The limp head sags until it rests on the ground
			auto head = rig.BoneIndex("head");
			PR_EXPECT(loco.pose().Tip(rig, head).z < 0.05f);

			// Standing up again
			loco.RequestFamily(FamilyIndex(gait, "bipedal"), 0.5f);
			for (int i = 0; i != 30; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(tilt(loco.pose().model) > stand_tilt + 0.3f);
			for (int i = 0; i != 270; ++i)
				loco.Step(dt, body, ground);

			PR_EXPECT(std::abs(tilt(loco.pose().model) - stand_tilt) < 0.05f);
			auto hand = rig.BoneIndex("hand.L");
			PR_EXPECT(FEqlAbsolute(loco.pose().model[hand].pos, rig.rest_model[hand].pos, 1e-2f));
		}
		PRUnitTestMethod(PunchesLoop, Quick)
		{
			// A body throwing punches drives its fist forward from the guard and back, with its chest moving into each punch, and repeats the sequence
			auto rig = TestRig();
			auto gait = TestActionGait();
			auto fight = FamilyIndex(gait, "fist-fight");
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .gravity = v4(0, 0, -9.8f, 0) };
			auto hand = rig.BoneIndex("hand.L");
			auto chest = rig.BoneIndex("chest");
			loco.RequestFamily(fight, 0.5f);
			for (int i = 0; i != 120; ++i)
				loco.Step(dt, body, ground);

			// One pass of the sequence, which is 1.8 seconds long
			auto start = loco.pose().model[hand].pos;
			auto y_lo = std::numeric_limits<float>::max();
			auto y_hi = std::numeric_limits<float>::lowest();
			auto chest_lo = std::numeric_limits<float>::max();
			auto chest_hi = std::numeric_limits<float>::lowest();
			for (int i = 0; i != 108; ++i)
			{
				loco.Step(dt, body, ground);
				y_lo = std::min(y_lo, loco.pose().model[hand].pos.y);
				y_hi = std::max(y_hi, loco.pose().model[hand].pos.y);
				chest_lo = std::min(chest_lo, loco.pose().model[chest].pos.y);
				chest_hi = std::max(chest_hi, loco.pose().model[chest].pos.y);
			}

			auto const& arm = loco.limbs()[1];
			PR_EXPECT(rig.chains[arm.chain].role == EChainRole::Arm);
			PR_EXPECT(y_hi - y_lo > 0.2f * arm.reach);
			PR_EXPECT(chest_hi - chest_lo > 0.02f);
			PR_EXPECT(FEqlAbsolute(loco.pose().model[hand].pos, start, 0.02f));
		}
	};
}
#endif
