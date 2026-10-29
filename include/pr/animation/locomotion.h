//*****************************************************************************
// Procedural animation - locomotion controller
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// Poses a character's body to follow its motion through the world, guided by a gait description (see gait.h):
//  - The caller requests a movement family, such as "bipedal" or "quadruped". The character moves in the requested family while the ground
//    beneath its feet is no steeper than the family's slope limit, and otherwise in the first family of the gait that the ground allows. A
//    change of family blends smoothly from the old family's stance and walking limbs to the new family's.
//  - Each step builds a guide pose: the family's stance plus the repeating body motion of its cycles, blended between the two cycles either side
//    of the body's speed and scaled down towards a standstill. The guide sets the shape of the body, the necks, the tails, any digits, and the
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
//  - The body breathes at a rate and depth set by the caller. Each breath moves the chest out from the front of the body and tilts it back, so
//    the spine arches, and rolls the shoulders back. The necks keep aiming where the guide aims, so the head moves only a little.
//  - Each tail trails behind its base along the ground like a rope, as one smooth curve: the first bone continues the line of the spine with a
//    small bend down, and each later bone bends a little at a time, steeply down while it can and then flatter, until the tail's skin rests
//    on the ground. The first bone heads across the ground with the swaying hips, and the dragged rope behind it sets which way each later bone
//    heads, so the sway travels down the tail as an 'S' wave. The first bone's joint limits are measured from the guide's tail shape.
//  - A body that stays unsupported falls with its walking limbs spread out from the body, and they swing down to the ground when it lands.
//  - A swimming family (medium "water") has no walking limbs, so every limb hangs from the body in the guide pose, and the body is never
//    airborne. The body and tails ignore the ground beneath them, and the family's body wave bends the spine and tails from side to side.
//    While the body enters or leaves the water, the body's height and the tails blend between following the ground and floating.
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
	};

	// A breathing motion. Each breath eases in and out, peaking halfway through the breath. Zero depths give no motion.
	struct Breath
	{
		float rate = 0.0f;       // Breaths per second (>= 0)
		float chest = 0.0f;      // How far the chest moves out from the front of the body at the peak of a breath, in metres (>= 0)
		float shoulders = 0.0f;  // How far the shoulders roll back at the peak of a breath, in radians (>= 0)
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
			, m_gait_speed(0.0f)
			, m_turn_rate(0.0f)
			, m_look()
			, m_look_weight(0.0f)
			, m_breath()
			, m_breath_phase(0.0f)
			, m_shoulders()
			, m_wave_bones()
			, m_phase(0.0f)
			, m_unsupported_s(0.0f)
			, m_fall(0.0f)
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
					auto chain = LimbChain(rig, foot.role, foot.side);
					if (std::none_of(m_limbs.begin(), m_limbs.end(), [=](LimbState const& l) { return l.chain == chain; }))
					{
						auto contact = LimbContact(rig, rig.chains[chain]);
						m_limbs.push_back(LimbState{ .chain = chain, .contact = contact, .footfall = foot, .reach = ChainLength(chain, rig.rest_model[rig.contacts[contact].bone] * rig.contacts[contact].offset) });
						FindSole(m_limbs.back());
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

		// Advance by 'dt' seconds (>= 0) and pose the rig for 'body' on 'ground'
		void Step(float dt, BodyState const& body, IGround const& ground)
		{
			// Find where each foot would stand beneath the body now. Planting, settling and lifting all compare the feet with these frames.
			m_up = Rotate(body.model_to_world.rot, v4::ZAxis());
			FindNeutralFeet(body.model_to_world, ground);

			// Move in the requested family if the ground beneath the feet allows it, otherwise in the first family that it allows
			ChooseFamily(body.gravity);

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

			// Measure the speed along the surface that the feet must keep up with, including turning on the spot. The family's slowest cycle sets
			// what counts as moving.
			auto const& slowest = family.cycles.front();
			auto tangential = body.velocity - m_up * Dot(body.velocity, m_up);
			auto turn = Dot(body.angular_velocity, m_up) * TurnRadius();
			auto speed = Sqrt(LengthSq(tangential) + Sqr(turn));
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
			TurnRaisedFeet();
			PlantDigits();
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
		float m_gait_speed;         // The smoothed speed that selects cycles and scales body motion
		float m_turn_rate;          // The smoothed rate of turning about the body's up direction, in radians per second
		TargetPath m_look;          // The world-space look point (pos), which moves smoothly to each new point
		Fade m_look_weight;         // How strongly the necks face 'm_look'
		Breath m_breath;            // The caller's breathing motion
		float m_breath_phase;       // The breath cycle position in [0,1)
		std::vector<Shoulder> m_shoulders; // The bones that breathing rolls back
		std::vector<WaveBone> m_wave_bones; // The bones that the body wave bends
		float m_phase;               // The step cycle position in [0,1)
		float m_unsupported_s;      // How long the body has been continuously unsupported, in seconds
		float m_fall;               // In [0,1]. How far the limbs are spread out for falling.
		v4 m_up;                    // The body's up direction (model +Z) in world space this step
		v4 m_ground_up;             // The mean unit normal of the ground beneath the feet this step, in world space
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
				limb.stance_bend = ~model[m_rig.chains[limb.chain].bones[1]].rot * limb.stance_rot;
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
			for (auto const& shoulder : m_shoulders)
				m_rot_offset[shoulder.bone] += shoulder.axis * (m_breath.shoulders * Inhale());

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
			if (limb.footfall.role == EChainRole::Arm)
			{
				auto root = m_guide.model[bones.front()].pos;
				auto swing = quat(v4::XAxis(), ArmSwing(limb));
				guide_end = xform(root + Rotate(swing, guide_end.pos - root), Normalise(swing * guide_end.rot));
			}

			// Without a spine the body does not move relative to the guide
			if (m_spine < 0)
				return guide_end;

			auto leg = limb.footfall.role == EChainRole::Leg;
			auto const& actual = leg ? m_targets[m_spine].base : m_targets[m_spine].frame;
			auto const& guide = leg ? m_guide_targets[m_spine].base : m_guide_targets[m_spine].frame;
			return actual * Invert(guide) * guide_end;
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
				if (limb.footfall.role != EChainRole::Leg || (limb.stance_contact.x < 0) != side || limb.support == 0)
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

		// Set the limb's digit chains and sole normal from the rig's rest pose. The sole is the plane through the limb's contact and the contacts
		// at the tips of its digits. A limb with fewer than three such contacts, or with contacts in a line, uses its contact's normal instead.
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
				limb.sole_up = -m_rig.contacts[limb.contact].normal;
				return;
			}
			auto normal =
				det_max == det_x ? v4(det_x, xz * yz - xy * zz, xy * yz - xz * yy, 0) :
				det_max == det_y ? v4(xz * yz - xy * zz, det_y, xy * xz - yz * xx, 0) :
				v4(xy * yz - xz * yy, xy * xz - yz * xx, det_z, 0);

			// The end bone is above its sole, so the normal points from the plane towards the bone's origin
			normal = Normalise(normal);
			limb.sole_up = Dot(normal, centre) > 0 ? -normal : normal;
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
			m_ground_up = Normalise(normal, m_up);
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

		// Change to the requested family if the ground allows it, otherwise to the first family in the gait that it allows. If the ground allows
		// no family, the current one is kept.
		void ChooseFamily(v4 gravity)
		{
			// Prefer the request, then the gait's order
			m_request_pending = false;
			auto f = m_requested;
			for (int i = 0, iend = static_cast<int>(m_families.size()); i != iend && !AllowsGround(f, gravity); ++i)
			{
				// Try each family in order until the ground allows one
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
			// The end bone's descendants, such as digits, are updated with the rest of the pose afterwards
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
			// Digit bones are posed from their parents' current frames. Their descendants are updated with the rest of the pose afterwards.
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

				auto i = limb.footfall.role == EChainRole::Leg ? 0 : 1;
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

			// Breathing moves the chest out from the front of the body, which faces along the spine turned a quarter turn forward about model +X,
			// and tilts it back by the angle that the move turns the spine through, so the spine arches
			auto breath = m_breath.chest * Inhale();
			if (breath != 0)
			{
				auto& chest = m_targets[m_spine].frame;
				auto front = Normalise(Cross(v4::XAxis(), span));
				auto arch = quat(Normalise(span), Normalise(span - front * breath));
				chest = xform(chest.pos + front * breath, Normalise(arch * chest.rot));
			}
		}

		// How far through its in-and-out motion the current breath is: 0 at the start and end of the breath, and 1 halfway
		float Inhale() const
		{
			return 0.5f * (1.0f - std::cos(math::constants<float>::tau * m_breath_phase));
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
				auto i = limb.footfall.role == EChainRole::Leg ? 0 : 1;
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
				auto i = limb.footfall.role == EChainRole::Leg ? 0 : 1;
				auto side = limb.stance_contact.x > 0 ? 1.0f : limb.stance_contact.x < 0 ? -1.0f : 0.0f;
				yaw[i] -= side * limb.support * std::cos(math::constants<float>::tau * (m_phase - limb.footfall.touchdown));
				count[i] += 1;
			}
			return {
				count[0] != 0 ? amplitude * yaw[0] / count[0] : 0.0f,
				count[1] != 0 ? amplitude * yaw[1] / count[1] : 0.0f,
			};
		}

		// Turn each neck from its guide aim towards the look point by the look weight
		void StepNecks(float dt, xform const& world_to_model)
		{
			m_look_weight.Step(dt);
			m_look.Step(dt);
			auto weight = m_look_weight.Value();
			auto look = world_to_model * m_look.Frame().pos;
			for (int c = 0, cend = static_cast<int>(m_rig.chains.size()); c != cend; ++c)
			{
				// Rotate the guide aim point about the neck's base onto the look direction, so that a zero weight gives the guide aim exactly
				auto const& chain = m_rig.chains[c];
				if (chain.role != EChainRole::Neck)
					continue;

				auto base = m_guide.model[chain.bones.front()].pos;
				auto aim = m_guide_targets[c].frame.pos - base;
				auto turn = Slerp(Identity<quat>(), quat(Normalise(aim), Normalise(look - base, Normalise(aim))), weight);
				m_targets[c].frame.pos = base + Rotate(turn, aim);
			}
		}

		// Trail each tail behind the body like a rope lying on the ground, as one smooth curve that continues the spine, bends down, and flattens onto the ground.
		//  - The tail's base keeps its position from the guide, and the first bone keeps its heading across the ground from the swaying hips. Each
		//    later joint is pulled from where it was last step towards the joint in front of it until it is one bone length away, so the hips' sway
		//    travels down the tail as an 'S' wave. This dragged rope sets only the heading of each bone across the ground.
		//  - The first bone droops as steeply as its joint limits allow about the guide's first bone, but bends down from the line of the spine
		//    (from the chest back through the pelvis) by no more than 'TailBend'. Each later bone bends up or down from the bone before it by at
		//    most 'TailBend', and droops no more than the first bone's joint limits allow. Each bone takes the steepest droop that keeps the tail
		//    clear of the ground, assuming the bones after it flatten as fast as they can.
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

				// True if bone 'i', with its head at 'height' and drooping by 'droop', and the bones after it flattening as fast as they can, stay clear of the ground
				auto clear = [&](int i, float droop, float height)
				{
					// Walk down the rest of the tail, checking each bone's skin point against the ground interpolated between its joints
					for (; i != count; ++i, droop = std::max(droop - TailBend, -math::constants<float>::tau_by_4))
					{
						auto const& bone = m_tail_bones[i];
						auto sin = std::sin(droop);
						auto skin = height - bone.along * sin - bone.depth * std::cos(droop);
						auto t = bone.along / bone.length;
						if (skin < m_tail_ground[i] + (m_tail_ground[i + 1] - m_tail_ground[i]) * t)
							return false;

						height -= bone.length * sin;
					}
					return true;
				};

				// Choose each bone's droop in turn: the steepest that stays clear, or the flattest allowed where none does. Clearance only improves as a
				// bone flattens, so a bisection finds the steepest clear droop.
				auto droop = 0.0f;
				auto height = base_height;
				for (int i = 0; i != count; ++i)
				{
					// Bones after the first bend up or down from the bone before by at most the bend limit
					auto lo = i == 0 ? droop_lo : std::max(droop - TailBend, -math::constants<float>::tau_by_4);
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
			m_pose.UpdateModel(m_rig);
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
		PRUnitTestMethod(BreathingMovesTheChest, Quick)
		{
			// At the peak of a breath the chest has moved out from the front of the body, which for the test rig's level spine is down, and it is
			// back where it started at the end of the breath
			auto rig = TestRig();
			auto still = Locomotion(rig, TestGait());
			auto breathing = Locomotion(rig, TestGait());
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			breathing.SetBreath(Breath{ .rate = 1.0f, .chest = 0.02f, .shoulders = 0.1f });
			for (int i = 0; i != 30; ++i)
			{
				still.Step(dt, BodyState{}, ground);
				breathing.Step(dt, BodyState{}, ground);
			}
			PR_EXPECT(FEqlAbsolute(breathing.targets()[0].frame.pos - still.targets()[0].frame.pos, v4(0, 0, -0.02f, 0), 1e-4f));

			for (int i = 0; i != 30; ++i)
			{
				still.Step(dt, BodyState{}, ground);
				breathing.Step(dt, BodyState{}, ground);
			}
			PR_EXPECT(FEqlAbsolute(breathing.targets()[0].frame.pos, still.targets()[0].frame.pos, 1e-4f));
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
			// between the feet, the tail curves down onto the ridge behind the body when walking forward. Either way, the tail bends by no more than the
			// bend limit at each joint, droops no more than the first bone's joint limits, and the tail's skin contacts stay above the ground.
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

			// The first tail bone can droop by up to 0.6 radians, and each tail bone has a skin contact beneath its middle
			auto depth = 0.04f;
			auto max_droop = 0.6f;
			for (auto b : tail.bones)
			{
				rig.bones[b].limit.swing = { .min_x = -max_droop, .max_x = max_droop, .min_z = -max_droop, .max_z = max_droop };
				rig.contacts.push_back(Contact{ .name = rig.bones[b].name + ".skin", .bone = b, .offset = v4(0, 0.15f, -depth, 1), .normal = v4(0, 0, -1, 0), .kind = EContactKind::Tail });
			}

			for (auto ridge : { 0.0f, 0.6f })
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
					clearance = std::min(clearance, (loco.pose().model[tail.bones[k]] * v4(0, 0.15f, -depth, 1)).z - ridge);
				}
				PR_EXPECT(clearance > -0.01f);
				PR_EXPECT(joint(count).z < joint(0).z - 0.3f);
				if (ridge != 0)
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
			auto rig = TestRig();
			auto gait = TestGait();
			auto swim = FamilyIndex(gait, "swim");
			auto const& glide = gait.families[swim].cycles.back();
			auto const& tail = rig.chains[4];
			auto loco = Locomotion(rig, gait);
			auto ground = TestGround();
			auto dt = 1.0f / 60;
			auto body = BodyState{ .velocity = v4(0, -glide.speed, 0, 0), .supported = false };
			loco.RequestFamily(swim, 0.5f);
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
				loco.Step(dt, body, ground);
			}
			PR_EXPECT(loco.family() == swim && loco.family_settled());

			// Over two cycles, record the sideways swing of the tail's base and tip
			auto tip_lo = std::numeric_limits<float>::max();
			auto tip_hi = std::numeric_limits<float>::lowest();
			auto base_hi = 0.0f;
			for (int i = 0; i != 120; ++i)
			{
				body.model_to_world.pos += body.velocity * dt;
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
	};
}
#endif
