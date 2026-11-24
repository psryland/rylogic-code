//*****************************************************************************
// Procedural animation - the strike activity
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// A "strike" activity loops one swinging blow, such as an overhead chop with an axe or a two-fisted hammer blow. Each cycle has four phases:
//  - Wind-up: the arm swings up and back from its ready position, bending at the elbow, and the torso turns and leans back.
//  - Strike: the arm swings forward and down, speeding up all the way, and the torso leans and turns into the blow. The tool point (the axe
//    head, or the fist without a tool) reaches the strike target at the "contact" mark.
//  - Follow-through: the arm carries on past the contact a little, while the body stays leaned into the blow.
//  - Recovery: the arm and torso return to the ready position, where the next cycle starts.
// The arm and the held tool swing as one rigid piece about the shoulder, in a plane through the shoulder and the contact point. The feet stay
// planted, with the foot opposite the striking hand a little forward. To reach a low target, the torso leans (stoops) further forward at contact
// until the wrist is within the arm's reach. The striking hands clench.
//
// Parameters ('.gait' file, see activities.h). Angles are in radians. Every parameter is required.
//  "params": {
//    "hands": "left" | "right" | "both",  // the striking arm(s). The rig must have an arm chain on each striking side.
//    "duration": 1.2,       // > 0. The seconds for one cycle.
//    "windup": 0.45,        // the fraction of the cycle at which the wind-up ends and the strike starts
//    "contact": 0.6,        // the fraction of the cycle at which the tool point reaches the target. windup < contact.
//    "follow": 0.75,        // the fraction of the cycle at which the follow-through ends and the recovery starts. contact < follow < 1.
//    "ready_angle": 0.6,    // the arm's swing angle in the ready position, up and back from its angle at contact
//    "ready_bend": 0.8,     // in (0,1]. The wrist's distance from the shoulder in the ready position, as a fraction of its distance at contact.
//    "windup_angle": 2.2,   // the arm's swing angle at the top of the wind-up
//    "windup_bend": 0.65,   // in (0,1]. The wrist's distance from the shoulder at the top of the wind-up, as a fraction of its distance at contact.
//    "follow_angle": 0.4,   // the arm's swing angle past the contact at the end of the follow-through
//    "cock": 1.0,           // the wrist's turn at contact from a thumb-up grip towards the arm's line, which puts the tool's handle along the arm
//    "swing_tilt": 0.0,     // the turn of the swing plane about the arm's line at contact, tilting the swing across the body. One-hand strikes only.
//    "spacing": 0.3,        // the distance between the fists of a two-hand strike, as a fraction of the arm's reach
//    "lean": 0.25,          // the torso's forward lean at contact
//    "windup_lean": 0.15,   // the torso's backward lean at the top of the wind-up
//    "twist": 0.4,          // the torso's turn at contact, which brings the striking shoulder forward. The wind-up turns half as far back. One-hand strikes only.
//    "crouch": 0.08,        // the hips' drop at contact, as a fraction of the spine's length
//    "max_stoop": 0.6,      // >= 0. The most extra forward lean at contact for reaching a low or distant target.
//    "stagger": 0.1         // the distance that the foot opposite a one-hand strike stands forward, and the other foot back, as a fraction of the legs' reach
//  }
//
// Timeline: the length is 'duration' and the activity loops. Each cycle has one "contact" mark, 'contact' of the way through it.
//
// Targets:
//  - 'strike': the tool point meets the target's surface at contact, on the side facing the body and at the height of the target's centre. The
//    fists of a two-hand strike meet the surface either side of that point. Without a strike, the blow lands straight ahead of the shoulders,
//    a little below them.
//  - 'tool': the tool that a one-hand strike holds. Its head is the tool point. Without a tool, or for a two-hand strike, the tool point is the
//    hand's grip point.
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/rig_json.h"
#include "pr/animation/targets.h"
#include "pr/animation/activity.h"
#include "pr/animation/activity_kit.h"

namespace pr::animation
{
	// The arms that make a strike
	enum class EStrikeHands
	{
		Left,
		Right,
		Both,
	};

	// The names of 'EStrikeHands' values in the file, in enum order
	inline constexpr std::array<std::string_view, 3> StrikeHandsNames = { "left", "right", "both" };

	// The shape and timing of a strike activity's blow. See the top of activity_strike.h for each parameter's meaning.
	struct StrikeSwing
	{
		EStrikeHands hands = EStrikeHands::Right;
		float duration = 1.2f;
		float windup = 0.45f;
		float contact = 0.6f;
		float follow = 0.75f;
		float ready_angle = 0.6f;
		float ready_bend = 0.8f;
		float windup_angle = 2.2f;
		float windup_bend = 0.65f;
		float follow_angle = 0.4f;
		float cock = 1.0f;
		float swing_tilt = 0.0f;
		float spacing = 0.3f;
		float lean = 0.25f;
		float windup_lean = 0.15f;
		float twist = 0.4f;
		float crouch = 0.08f;
		float max_stoop = 0.6f;
		float stagger = 0.1f;
	};

	// A looping swinging blow. See the top of this file.
	struct Strikes : IActivity
	{
		// The fraction of the chest's turn that the pelvis also turns
		static constexpr float PelvisShare = 0.4f;

		// The furthest that the wrist goes from the shoulder at contact, as a fraction of the arm's length to the wrist. Slightly less than a straight
		// arm, so the elbow keeps a small bend.
		static constexpr float MaxExtension = 0.98f;

		// The number of halvings in the search for the stoop that brings a target within reach
		static constexpr int StoopSearchSteps = 12;

		// Where a blow lands without a strike target: this far forward of and below the shoulders, in units of the arm's reach
		static constexpr float DefaultAhead = 0.75f;
		static constexpr float DefaultDrop = 0.3f;

		// The angle (radians) by which each bone of a striking hand's digits curls towards the palm to hold a handle or make a fist
		static constexpr float GripCurl = 1.2f;

		// Construct from the blow's shape. 'Validate' checks it against a rig.
		explicit Strikes(StrikeSwing swing)
			: m_swing(swing)
			, m_timeline()
		{
			// One contact per cycle
			m_timeline.length = m_swing.duration;
			m_timeline.loop = true;
			m_timeline.marks.push_back(ActivityMark{ .name = "contact", .time = m_swing.contact * m_swing.duration });
		}

		// The blow's shape and timing
		StrikeSwing const& swing() const
		{
			return m_swing;
		}

		// Read the parameters of a strike activity (see the top of this file). Throws 'std::runtime_error' if they do not follow the schema.
		static std::shared_ptr<IActivity const> Read(json::Object const& params, std::string const& context)
		{
			// Every parameter is a named member of 'params'
			using namespace rig_json;
			auto real = [&](std::string_view name)
			{
				// Read one number
				auto ctx = std::format("{}.{}", context, name);
				return ReadFloat(Member(params, name, context), ctx);
			};
			auto swing = StrikeSwing{
				.hands = ParseEnum<EStrikeHands>(ReadString(Member(params, "hands", context), context + ".hands"), StrikeHandsNames, context + ".hands"),
				.duration = real("duration"),
				.windup = real("windup"),
				.contact = real("contact"),
				.follow = real("follow"),
				.ready_angle = real("ready_angle"),
				.ready_bend = real("ready_bend"),
				.windup_angle = real("windup_angle"),
				.windup_bend = real("windup_bend"),
				.follow_angle = real("follow_angle"),
				.cock = real("cock"),
				.swing_tilt = real("swing_tilt"),
				.spacing = real("spacing"),
				.lean = real("lean"),
				.windup_lean = real("windup_lean"),
				.twist = real("twist"),
				.crouch = real("crouch"),
				.max_stoop = real("max_stoop"),
				.stagger = real("stagger"),
			};
			return std::make_shared<Strikes const>(swing);
		}

		// Describe the parameters for editors: their names, kinds, ranges and defaults (see activities.h)
		static json::Object Describe()
		{
			// The hands, then one number per shape parameter
			auto defaults = StrikeSwing{};
			json::Array hands;
			for (auto name : StrikeHandsNames)
				hands.push_back(name);

			json::Array params;
			params.push_back(json::Object{ { "name", "hands" }, { "kind", "enum" }, { "items", std::move(hands) }, { "default", rig_json::EnumName(defaults.hands, StrikeHandsNames) } });
			auto real = [&](std::string_view name, float lo, float hi, float value)
			{
				// One number with its editing range
				params.push_back(json::Object{ { "name", name }, { "kind", "float" }, { "min", lo }, { "max", hi }, { "default", value } });
			};
			real("duration", 0.2f, 5.0f, defaults.duration);
			real("windup", 0.05f, 0.9f, defaults.windup);
			real("contact", 0.1f, 0.95f, defaults.contact);
			real("follow", 0.15f, 0.99f, defaults.follow);
			real("ready_angle", -1.0f, 3.0f, defaults.ready_angle);
			real("ready_bend", 0.3f, 1.0f, defaults.ready_bend);
			real("windup_angle", 0.0f, 3.1f, defaults.windup_angle);
			real("windup_bend", 0.3f, 1.0f, defaults.windup_bend);
			real("follow_angle", 0.0f, 1.5f, defaults.follow_angle);
			real("cock", -0.5f, 1.6f, defaults.cock);
			real("swing_tilt", -1.0f, 1.0f, defaults.swing_tilt);
			real("spacing", 0.0f, 1.0f, defaults.spacing);
			real("lean", 0.0f, 1.0f, defaults.lean);
			real("windup_lean", 0.0f, 0.6f, defaults.windup_lean);
			real("twist", 0.0f, 1.0f, defaults.twist);
			real("crouch", 0.0f, 0.4f, defaults.crouch);
			real("max_stoop", 0.0f, 1.2f, defaults.max_stoop);
			real("stagger", 0.0f, 0.4f, defaults.stagger);
			return json::Object{ { "root_mode", rig_json::EnumName(ERootMode::Planted, RootModeNames) }, { "params", std::move(params) } };
		}

		// The type's name in the registry
		std::string_view Type() const override
		{
			return "strike";
		}

		// The feet stay where they are
		ERootMode RootMode() const override
		{
			return ERootMode::Planted;
		}

		// One cycle, looping, with one "contact" mark
		ActivityTimeline const& Timeline() const override
		{
			return m_timeline;
		}

		// The striking arms and the first spine chain
		std::vector<int> Chains(Rig const& rig) const override
		{
			// The spine carries the body drive
			std::vector<int> chains;
			for (auto side : HandSides())
			{
				// A missing arm is reported by 'Validate'
				if (auto arm = ArmChain(rig, side); arm >= 0)
					chains.push_back(arm);
			}
			if (auto spine = std::ranges::find(rig.chains, EChainRole::Spine, &Chain::role); spine != rig.chains.end())
				chains.push_back(static_cast<int>(spine - rig.chains.begin()));

			std::ranges::sort(chains);
			return chains;
		}

		// The foot opposite the striking hand stands forward (-Y) and the other foot back. A two-hand strike keeps the stance square.
		v4 StanceShift(Rig const& rig, int chain, float leg_reach) const override
		{
			// The product of the side signs is -1 for the opposite foot
			return v4(0, SideSign(rig.chains[chain].side) * HandSign() * m_swing.stagger * leg_reach, 0, 0);
		}

		// The phases must be in order and the striking arms must exist
		void Validate(Rig const& rig, std::string_view context, std::vector<std::string>& errors) const override
		{
			// Check the timing and the shape
			auto const& s = m_swing;
			if (!(s.duration > 0))
				errors.push_back(std::format("{} needs a duration greater than zero", context));
			if (!(0 < s.windup && s.windup < s.contact && s.contact < s.follow && s.follow < 1))
				errors.push_back(std::format("{} needs 0 < windup < contact < follow < 1", context));
			if (!(s.ready_bend > 0 && s.ready_bend <= 1 && s.windup_bend > 0 && s.windup_bend <= 1))
				errors.push_back(std::format("{} needs ready_bend and windup_bend in (0, 1]", context));
			if (!(s.max_stoop >= 0))
				errors.push_back(std::format("{} needs a max_stoop of zero or more", context));

			// Each striking side needs an arm
			for (auto side : HandSides())
			{
				if (ArmChain(rig, side) < 0)
					errors.push_back(std::format("{} strikes with the {} hand, so it needs an arm chain on that side", context, rig_json::EnumName(side, rig_json::SideNames)));
			}
		}

		// The blow's shape and timing
		json::Object WriteParams() const override
		{
			// One member per parameter
			auto const& s = m_swing;
			return json::Object{
				{ "hands", rig_json::EnumName(s.hands, StrikeHandsNames) },
				{ "duration", s.duration },
				{ "windup", s.windup },
				{ "contact", s.contact },
				{ "follow", s.follow },
				{ "ready_angle", s.ready_angle },
				{ "ready_bend", s.ready_bend },
				{ "windup_angle", s.windup_angle },
				{ "windup_bend", s.windup_bend },
				{ "follow_angle", s.follow_angle },
				{ "cock", s.cock },
				{ "swing_tilt", s.swing_tilt },
				{ "spacing", s.spacing },
				{ "lean", s.lean },
				{ "windup_lean", s.windup_lean },
				{ "twist", s.twist },
				{ "crouch", s.crouch },
				{ "max_stoop", s.max_stoop },
				{ "stagger", s.stagger },
			};
		}

		// Pose the body 'time' seconds into the cycle. The blow is planned at contact first: where each tool point lands, how far the torso stoops to
		// reach it, and how each hand and its tool are held there. Then the arm and tool swing about the shoulder from that pose by the phase's angle,
		// with the wrist pulled in by the phase's bend, while the torso blends between its wind-up and contact drives.
		void Evaluate(float time, ActivityInput const& input, ActivityOutput const& output) const override
		{
			// Find the phase, and plan the blow
			auto phase = PhaseAt(Clamp(time / m_swing.duration, 0.0f, 1.0f));
			auto plan = PlanBlow(input);

			// Blend the torso's drives. At contact the contact drive applies alone, so the shoulders are where the plan put them.
			if (input.spine >= 0)
			{
				auto const& guide = input.guide[input.spine];
				auto drive = TorsoDrive{
					.yaw = phase.windup_weight * plan.windup.yaw + phase.contact_weight * plan.contact.yaw,
					.lean = phase.windup_weight * plan.windup.lean + phase.contact_weight * plan.contact.lean,
					.tilt = 0,
					.hips = plan.windup.hips * phase.windup_weight + plan.contact.hips * phase.contact_weight,
				};
				DriveChest(drive, guide.base.pos, guide.frame.pos);
				ApplyTorsoDrive(output.targets[input.spine], drive, PelvisShare);
			}

			// Swing each striking arm and its tool as one piece about its shoulder
			auto chest = input.spine >= 0 ? output.targets[input.spine].frame : xform::Identity();
			for (auto const& hand : plan.hands)
			{
				// The elbow bends below the arm in the swing plane, and out to the arm's side
				auto root = LimbRoot(input, hand.limb->chain, chest);
				auto swing = quat(hand.axis, phase.angle);
				auto& target = output.targets[hand.limb->chain];
				target.frame = xform(root + Rotate(swing, hand.wrist - hand.shoulder) * phase.bend, Normalise(swing * hand.orientation));
				target.pole = Normalise(Rotate(swing, -hand.up) + v4(hand.side * 0.5f, 0, 0, 0));
				output.curl[hand.limb->chain] = GripCurl;
			}
		}

	private:

		// Where the swing is at one moment of the cycle
		struct Phase
		{
			float angle;           // The arm's swing angle about the swing axis, positive up and back from its angle at contact
			float bend;            // The wrist's distance from the shoulder as a fraction of its distance at contact
			float windup_weight;   // How much of the wind-up drive the torso has
			float contact_weight;  // How much of the contact drive the torso has
		};

		// How one striking hand meets the target at contact. Positions and directions are in model space.
		struct HandPlan
		{
			ActivityLimb const* limb;  // The striking arm
			float side;                // The arm's side sign (see 'SideSign')
			v4 shoulder;               // The arm's root joint at contact
			v4 wrist;                  // The end bone's origin at contact
			quat orientation;          // The end bone's orientation at contact
			v4 axis;                   // The swing axis. Turning about it by a positive angle lifts the arm up and back.
			v4 up;                     // The direction in the swing plane, perpendicular to the arm's line, that a positive turn moves the tool point
		};

		// The blow at contact: the torso's drives, and each striking hand
		struct BlowPlan
		{
			TorsoDrive windup;
			TorsoDrive contact;
			std::vector<HandPlan> hands;
		};

		StrikeSwing m_swing;
		ActivityTimeline m_timeline;

		// The sides of the striking arms
		std::vector<ESide> HandSides() const
		{
			switch (m_swing.hands)
			{
				case EStrikeHands::Left: { return { ESide::Left }; }
				case EStrikeHands::Right: { return { ESide::Right }; }
				case EStrikeHands::Both: { return { ESide::Left, ESide::Right }; }
				default: throw std::runtime_error("Unknown strike hands");
			}
		}

		// The side sign of a one-hand strike's arm, or 0 for a two-hand strike, which twists and tilts neither way
		float HandSign() const
		{
			switch (m_swing.hands)
			{
				case EStrikeHands::Left: { return SideSign(ESide::Left); }
				case EStrikeHands::Right: { return SideSign(ESide::Right); }
				case EStrikeHands::Both: { return 0.0f; }
				default: throw std::runtime_error("Unknown strike hands");
			}
		}

		// The swing at 'u' in [0,1] through the cycle. Each phase starts where the previous one ends, and the recovery ends where the wind-up starts,
		// so the loop has no jumps. The strike phase speeds up all the way to contact, where the angle is exactly zero, the bend exactly one, and the
		// contact drive applies alone.
		Phase PhaseAt(float u) const
		{
			auto const& s = m_swing;
			if (u < s.windup)
			{
				// Ease from the ready position up to the top of the wind-up, leaning back
				auto e = Ease(0, s.windup, u);
				return Phase{ .angle = Lerp(s.ready_angle, s.windup_angle, e), .bend = Lerp(s.ready_bend, s.windup_bend, e), .windup_weight = e, .contact_weight = 0 };
			}
			if (u < s.contact)
			{
				// Swing down, speeding up, while the torso turns from leaning back to leaning into the blow
				auto x = (u - s.windup) / (s.contact - s.windup);
				auto x2 = x * x;
				return Phase{ .angle = s.windup_angle * (1 - x2), .bend = Lerp(s.windup_bend, 1.0f, x2), .windup_weight = 1 - x2, .contact_weight = x2 };
			}
			if (u < s.follow)
			{
				// Carry on past the contact, slowing down
				auto x = (u - s.contact) / (s.follow - s.contact);
				return Phase{ .angle = -s.follow_angle * x * (2 - x), .bend = 1, .windup_weight = 0, .contact_weight = 1 };
			}

			// Ease back to the ready position
			auto e = Ease(s.follow, 1, u);
			return Phase{ .angle = Lerp(-s.follow_angle, s.ready_angle, e), .bend = Lerp(1.0f, s.ready_bend, e), .windup_weight = 0, .contact_weight = 1 - e };
		}

		// The torso drives at the top of the wind-up and at contact, with 'stoop' extra forward lean at contact
		BlowPlan Drives(ActivityInput const& input, float stoop) const
		{
			// The hips drop in proportion to the spine's length
			auto plan = BlowPlan{
				.windup = TorsoDrive{ .yaw = 0.5f * HandSign() * m_swing.twist, .lean = -m_swing.windup_lean },
				.contact = TorsoDrive{ .yaw = -HandSign() * m_swing.twist, .lean = m_swing.lean + stoop },
				.hands = {},
			};
			if (input.spine >= 0)
			{
				// The chest swings about the hips
				auto const& guide = input.guide[input.spine];
				plan.contact.hips = v4(0, 0, -m_swing.crouch * Length(guide.frame.pos - guide.base.pos), 0);
				DriveChest(plan.windup, guide.base.pos, guide.frame.pos);
				DriveChest(plan.contact, guide.base.pos, guide.frame.pos);
			}
			return plan;
		}

		// The chest frame that the contact drive of 'plan' gives, or identity without a spine
		static xform ContactChest(ActivityInput const& input, BlowPlan const& plan)
		{
			// Apply the drive to a copy of the guide's spine goal, as 'Evaluate' does
			if (input.spine < 0)
				return xform::Identity();

			auto spine = input.guide[input.spine];
			ApplyTorsoDrive(spine, plan.contact, PelvisShare);
			return spine.frame;
		}

		// Plan the blow at contact (see 'BlowPlan')
		BlowPlan PlanBlow(ActivityInput const& input) const
		{
			// Find the striking arms among the limbs. Arms that do not walk in any family are not limbs, and do not strike.
			struct Arm
			{
				ActivityLimb const* limb;
				ESide side;
			};
			std::vector<Arm> arms;
			for (auto side : HandSides())
			{
				auto chain = ArmChain(input.rig, side);
				auto limb = std::ranges::find(input.limbs, chain, &ActivityLimb::chain);
				if (limb != input.limbs.end())
					arms.push_back(Arm{ .limb = &*limb, .side = side });
			}
			if (arms.empty())
				return Drives(input, 0);

			// Choose the contact points from the shoulders before any stoop. The tool point lands on the target's surface, on the side facing the
			// shoulders, at the height of the target's centre. Two fists land either side of that point, across the line to the body.
			auto chest = ContactChest(input, Drives(input, 0));
			auto centre = v4::Zero();
			auto reach = 0.0f;
			for (auto const& arm : arms)
			{
				centre += LimbRoot(input, arm.limb->chain, chest).w0();
				reach += arm.limb->reach;
			}
			centre = (centre / static_cast<float>(arms.size())).w1();
			reach /= static_cast<float>(arms.size());

			auto point = centre + v4(0, -DefaultAhead * reach, -DefaultDrop * reach, 0);
			auto radius = 0.0f;
			if (input.targets.strike)
			{
				// Measure the target in model space
				point = input.world_to_model * input.targets.strike->point;
				radius = input.targets.strike->radius;
			}
			auto toward = v4(centre.x - point.x, centre.y - point.y, 0, 0);
			toward = LengthSq(toward) > 1e-8f ? Normalise(toward) : v4::YAxis();
			auto across = Cross(toward, v4::ZAxis());
			auto contacts = std::vector<v4>{};
			for (auto const& arm : arms)
			{
				// A lone hand lands at the centre of its side of the target. Paired fists are spread across, then put back on the surface.
				auto offset = arms.size() > 1 ? across * (SideSign(arm.side) * 0.5f * m_swing.spacing * reach) : v4::Zero();
				auto flat = toward * radius + offset;
				if (radius > 0 && LengthSq(flat) > 1e-8f)
					flat = Normalise(flat) * radius;

				contacts.push_back(point + flat);
			}

			// Plan each hand at contact for a given stoop. The wrist goes where the tool point meets the target.
			auto tool = arms.size() == 1 ? input.targets.tool : std::nullopt;
			auto plan_hands = [&](float stoop)
			{
				// The shoulders move with the stooping chest
				auto plan = Drives(input, stoop);
				auto stooped = ContactChest(input, plan);
				for (int i = 0, iend = static_cast<int>(arms.size()); i != iend; ++i)
				{
					auto shoulder = LimbRoot(input, arms[i].limb->chain, stooped);
					plan.hands.push_back(PlanHand(*arms[i].limb, arms[i].side, shoulder, contacts[i], tool));
				}
				return plan;
			};
			auto reachable = [&](BlowPlan const& plan)
			{
				// Every wrist is within its arm's reach
				for (auto const& hand : plan.hands)
				{
					if (Length(hand.wrist - hand.shoulder) > MaxExtension * WristReach(input.rig, hand.limb->chain))
						return false;
				}
				return true;
			};

			// Stoop no more than needed. Leaning further forward brings the shoulders closer to a target that is low or ahead.
			auto plan = plan_hands(0);
			if (reachable(plan) || m_swing.max_stoop <= 0)
				return plan;

			auto most = plan_hands(m_swing.max_stoop);
			if (!reachable(most))
				return most;

			auto lo = 0.0f;
			auto hi = m_swing.max_stoop;
			for (int i = 0; i != StoopSearchSteps; ++i)
			{
				// Keep 'hi' reachable
				auto mid = 0.5f * (lo + hi);
				if (reachable(plan_hands(mid)))
					hi = mid;
				else
					lo = mid;
			}
			return plan_hands(hi);
		}

		// Plan one hand on 'side', whose shoulder is at 'shoulder' at contact, so that its tool point is at 'contact'
		HandPlan PlanHand(ActivityLimb const& limb, ESide side, v4 const& shoulder, v4 const& contact, std::optional<Tool> const& tool) const
		{
			// The swing plane contains the line from the shoulder to the contact, and is upright unless tilted. A target directly above or below
			// the shoulder swings in the body's side-to-side axis.
			auto sign = SideSign(side);
			auto line = Normalise((contact - shoulder).w0());
			auto axis = Cross(line, v4::ZAxis());
			axis = LengthSq(axis) > 1e-6f ? Normalise(axis) : -v4::XAxis();
			axis = Rotate(quat(line, HandSign() * m_swing.swing_tilt), axis);
			auto up = Cross(axis, line);

			// At contact the fingers point along the arm's line and the thumb up, turned towards the line by the cock, so a held handle runs
			// out beyond the hand and its face points along the blow
			auto grip = GripAxes(limb, side);
			auto handle = up * std::cos(m_swing.cock) + line * std::sin(m_swing.cock);
			auto face = line * std::cos(m_swing.cock) - up * std::sin(m_swing.cock);
			auto orientation = HandOrientation(grip, face, handle);
			auto wrist = contact - Rotate(orientation, ToolPoint(limb, grip, tool).w0());
			return HandPlan{ .limb = &limb, .side = sign, .shoulder = shoulder, .wrist = wrist, .orientation = orientation, .axis = axis, .up = up };
		}

		// The length of the arm 'chain' from its root joint to its wrist: every bone except the end bone
		static float WristReach(Rig const& rig, int chain)
		{
			// The bones run from the root to the end bone
			auto length = 0.0f;
			auto const& bones = rig.chains[chain].bones;
			for (size_t i = 0; i + 1 != bones.size(); ++i)
				length += rig.bones[bones[i]].length;

			return length;
		}
	};
}
