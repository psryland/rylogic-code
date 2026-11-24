//*****************************************************************************
// Procedural animation - the punches activity
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// A "punches" activity loops a sequence of punches. The feet take a boxing stance with the left foot forward, and the body crouches with its
// hands holding a guard in front of the chin. Each punch drives one fist out and back while the torso turns the punching shoulder forward and
// leans after the fist, carrying the hips and head with it. Depending on the kind of punch, the knees dip or rise, the chest bends towards the
// punching side, and the punching side's foot turns on its ball or steps forward and back. The feet return to their stance after each punch, so
// the body stays in place. The hands clench into fists.
//
// Parameters ('.gait' file, see activities.h):
//  "params": {
//    "punches": [{                       // not empty. The sequence of punches, which repeats.
//      "hand": "left" | "right",         // the side of the arm that punches. The rig must have an arm chain on that side.
//      "kind": "jab" | "cross" | "hook" | "uppercut" | "body_hook" | "body_jab",
//      "duration": 0.6                   // > 0. The time, in seconds, for the punch and the return to the guard.
//    }]
//  }
//
// Timeline: the length is the sum of the punch durations, and the activity loops. Each punch has a "contact" mark at the moment its fist is
// fully out ('PunchOut' of the way through the punch), in sequence order.
//
// Targets: 'strike'. Each punch drives its fist forward to the strike's surface instead of to its usual extent, as far as the arm can reach and
// never back behind the guard. Without a strike, punches go to their usual extent.
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/rig_json.h"
#include "pr/animation/targets.h"
#include "pr/animation/activity.h"
#include "pr/animation/activity_kit.h"

namespace pr::animation
{
	// The kind of one punch
	enum class EPunchKind
	{
		// A straight punch with the lead hand
		Jab,

		// A straight punch with the rear hand, turning the torso further
		Cross,

		// A horizontal swing at head height, with the elbow out to the side
		Hook,

		// An upward punch from low to chin height
		Uppercut,

		// A hook to the body, bending the knees and leaning to the punching side
		BodyHook,

		// A straight punch to the body, stepping in with the punching side's foot
		BodyJab,
	};

	// The names of 'EPunchKind' values in the file, in enum order
	inline constexpr std::array<std::string_view, 6> PunchKindNames = { "jab", "cross", "hook", "uppercut", "body_hook", "body_jab" };

	// One punch of a punches activity
	struct Punch
	{
		ESide hand = ESide::Left;          // Left or Right. The side of the arm that punches.
		EPunchKind kind = EPunchKind::Jab;
		float duration = 0.6f;             // > 0. The time, in seconds, for the punch and the return to the guard.
	};

	// A looping sequence of punches. See the top of this file.
	struct Punches : IActivity
	{
		// The distance that a boxing stance moves each foot forward (left) or back (right), as a fraction of the legs' reach
		static constexpr float BoxingStagger = 0.12f;

		// The angle (radians) about model +Z by which the torso turns its left shoulder forward when on guard, and the fractions of each punch at which
		// the fist is fully out and then back on guard
		static constexpr float GuardYaw = 0.25f;
		static constexpr float PunchOut = 0.35f;
		static constexpr float PunchBack = 0.75f;

		// The furthest forward that a fist aimed at a strike target goes, in units of the arm's reach from the shoulder. A straight arm reaches 1.
		static constexpr float StrikeMaxReach = 1.0f;

		// The fraction of the chest's punch turn that the pelvis also turns
		static constexpr float PunchPelvisShare = 0.4f;

		// The angle (radians) by which the torso of a body on guard leans forward about the pelvis, and the distance that the hips drop, as a fraction
		// of the spine's length, so that the body crouches behind its guard
		static constexpr float GuardLean = 0.12f;
		static constexpr float GuardCrouch = 0.1f;

		// The fraction of the chest's forward travel, as the torso leans into a punch, by which the hips also move forward over the front foot
		static constexpr float PunchHipDrive = 0.4f;

		// The fraction of a punching foot's step by which the hips move forward with it, and the height of the step's arc as a fraction of its length
		static constexpr float StepSurge = 0.5f;
		static constexpr float StepLift = 0.3f;

		// The angle (radians) by which each bone of a punching hand's digits curls towards the palm to make a fist
		static constexpr float FistCurl = 1.2f;

		// The range and default of a punch's duration offered to editors, in seconds
		static constexpr float EditMinDuration = 0.05f;
		static constexpr float EditMaxDuration = 5.0f;

		// How the body moves into one kind of punch. Angles and distances are at the punch's full extent unless stated. The fist's path is in
		// model axes from the shoulder, in units of the arm's reach, for a left-hand punch. A right-hand punch mirrors its X values.
		struct Style
		{
			float yaw;     // The angle (radians) by which the torso turns the punching shoulder forward
			float lean;    // The angle (radians) by which the torso leans forward, beyond its guard
			float tilt;    // The angle (radians) by which the chest bends sideways, dropping the punching shoulder
			float dip;     // The distance that the hips drop, as a fraction of the spine's length. Negative values rise out of the guard's crouch.
			float sink;    // The distance that the hips drop half way out, and rise from again by full extent, as a fraction of the spine's length
			float pivot;   // The angle (radians) by which the punching side's foot turns on its ball, turning its heel out
			float step;    // The distance that the punching side's foot steps forward, as a fraction of the leg's reach
			v4 ctrl;       // The control point of the fist's curved path from the guard
			v4 end;        // The fist's position at full extent
			v4 dir;        // The direction that the fingers point at full extent
			v4 pole;       // The direction that the elbow bends towards at full extent
		};

		// Construct from a sequence of punches. 'Validate' checks it against a rig.
		explicit Punches(std::vector<Punch> punches)
			: m_punches(std::move(punches))
			, m_timeline()
		{
			// Each punch lands when its fist is fully out
			m_timeline.loop = true;
			for (auto const& punch : m_punches)
			{
				m_timeline.marks.push_back(ActivityMark{ .name = "contact", .time = m_timeline.length + PunchOut * punch.duration });
				m_timeline.length += punch.duration;
			}
		}

		// The sequence of punches
		std::span<Punch const> punches() const
		{
			return m_punches;
		}

		// Read the parameters of a punches activity (see the top of this file). Throws 'std::runtime_error' if they do not follow the schema.
		static std::shared_ptr<IActivity const> Read(json::Object const& params, std::string const& context)
		{
			// Each punch names its hand and kind
			using namespace rig_json;
			std::vector<Punch> punches;
			auto const& list = ReadArray(Member(params, "punches", context), context + ".punches");
			for (size_t p = 0; p != list.size(); ++p)
			{
				auto pctx = std::format("{}.punches[{}]", context, p);
				auto const& pobj = ReadObject(list[p], pctx);
				punches.push_back(Punch{
					.hand = ParseEnum<ESide>(ReadString(Member(pobj, "hand", pctx), pctx + ".hand"), SideNames, pctx + ".hand"),
					.kind = ParseEnum<EPunchKind>(ReadString(Member(pobj, "kind", pctx), pctx + ".kind"), PunchKindNames, pctx + ".kind"),
					.duration = ReadFloat(Member(pobj, "duration", pctx), pctx + ".duration"),
				});
			}
			return std::make_shared<Punches const>(std::move(punches));
		}

		// Describe the parameters for editors: their names, kinds, ranges and defaults (see activities.h)
		static json::Object Describe()
		{
			// One list parameter whose items are punches
			json::Array kinds;
			for (auto name : PunchKindNames)
				kinds.push_back(name);

			json::Array fields;
			fields.push_back(json::Object{ { "name", "hand" }, { "kind", "enum" }, { "items", json::Array{ "left", "right" } }, { "default", "left" } });
			fields.push_back(json::Object{ { "name", "kind" }, { "kind", "enum" }, { "items", std::move(kinds) }, { "default", "jab" } });
			fields.push_back(json::Object{ { "name", "duration" }, { "kind", "float" }, { "min", EditMinDuration }, { "max", EditMaxDuration }, { "default", Punch{}.duration } });

			json::Array params;
			params.push_back(json::Object{ { "name", "punches" }, { "kind", "list" }, { "min_items", 1 }, { "fields", std::move(fields) } });
			return json::Object{ { "root_mode", rig_json::EnumName(ERootMode::Planted, RootModeNames) }, { "params", std::move(params) } };
		}

		// The type's name in the registry
		std::string_view Type() const override
		{
			return "punches";
		}

		// The feet stay where they are
		ERootMode RootMode() const override
		{
			return ERootMode::Planted;
		}

		// The sequence's length, looping, and one "contact" mark per punch
		ActivityTimeline const& Timeline() const override
		{
			return m_timeline;
		}

		// The spine, the arms and the legs
		std::vector<int> Chains(Rig const& rig) const override
		{
			// The whole body joins in, except the necks and tails
			std::vector<int> chains;
			for (int c = 0, cend = static_cast<int>(rig.chains.size()); c != cend; ++c)
			{
				auto role = rig.chains[c].role;
				if (role == EChainRole::Spine || role == EChainRole::Arm || role == EChainRole::Leg)
					chains.push_back(c);
			}
			return chains;
		}

		// The boxing stance puts the left foot forward and the right foot back. Forward is model -Y and left is +X.
		v4 StanceShift(Rig const& rig, int chain, float leg_reach) const override
		{
			return v4(0, -SideSign(rig.chains[chain].side) * BoxingStagger * leg_reach, 0, 0);
		}

		// Each punch needs a positive duration and an arm on its side
		void Validate(Rig const& rig, std::string_view context, std::vector<std::string>& errors) const override
		{
			// A sequence needs at least one punch
			if (m_punches.empty())
				errors.push_back(std::format("{} throws punches, so it needs at least one punch", context));

			for (size_t p = 0; p != m_punches.size(); ++p)
			{
				// Check the punch's timing and hand
				auto const& punch = m_punches[p];
				if (!(punch.duration > 0))
					errors.push_back(std::format("{} punch {} needs a duration greater than zero", context, p));
				if (punch.hand == ESide::Centre || ArmChain(rig, punch.hand) < 0)
					errors.push_back(std::format("{} punch {} needs the left or right hand of an arm chain in the rig", context, p));
			}
		}

		// The sequence of punches
		json::Object WriteParams() const override
		{
			// Write the punches in sequence order
			using namespace rig_json;
			json::Array punches;
			for (auto const& punch : m_punches)
				punches.push_back(json::Object{ { "hand", EnumName(punch.hand, SideNames) }, { "kind", EnumName(punch.kind, PunchKindNames) }, { "duration", punch.duration } });

			return json::Object{ { "punches", std::move(punches) } };
		}

		// The punch that is being thrown at 'time' seconds into the sequence, and how far out its fist is, in [0,1]. Times beyond the sequence's
		// length belong to the last punch.
		std::pair<Punch const*, float> PunchAt(float time) const
		{
			// Walk the sequence to the punch that the time is in. The fist accelerates all the way out, so it is at its fastest when it lands,
			// then comes back to guard more slowly.
			for (auto const& punch : m_punches)
			{
				// The last punch takes any time left over
				if (time < punch.duration || &punch == &m_punches.back())
				{
					auto u = Clamp(time / punch.duration, 0.0f, 1.0f);
					return { &punch, SwingExtend(u, PunchOut, PunchBack) };
				}
				time -= punch.duration;
			}
			return { nullptr, 0.0f };
		}

		// The movement of each kind of punch (see 'Style')
		static Style StyleOf(EPunchKind kind)
		{
			switch (kind)
			{
				case EPunchKind::Jab:
				{
					// Straight out from the lead side, with a short step in
					auto end = v4(-0.25f, -0.9f, 0.15f, 0);
					return Style{ .yaw = 0.25f, .lean = 0.12f, .tilt = 0, .dip = 0, .sink = 0, .pivot = 0, .step = 0.06f,
						.ctrl = (GuardFist() + end) * 0.5f, .end = end, .dir = v4(0, -1, 0, 0), .pole = Normalise(v4(0.5f, 0, -1, 0)) };
				}
				case EPunchKind::Cross:
				{
					// Straight out across the body's midline, with the hips and shoulders turned fully and the rear heel turned out
					auto end = v4(-0.35f, -0.9f, 0.15f, 0);
					return Style{ .yaw = 0.75f, .lean = 0.3f, .tilt = 0, .dip = 0, .sink = 0, .pivot = 0.6f, .step = 0,
						.ctrl = (GuardFist() + end) * 0.5f, .end = end, .dir = v4(0, -1, 0, 0), .pole = Normalise(v4(0.5f, 0, -1, 0)) };
				}
				case EPunchKind::Hook:
				{
					// Out to the side, then around in front, with the elbow raised level with the fist and the foot turning with the hips
					return Style{ .yaw = 0.65f, .lean = 0.2f, .tilt = 0, .dip = 0.05f, .sink = 0, .pivot = 0.5f, .step = 0,
						.ctrl = v4(0.25f, -0.45f, 0.25f, 0), .end = v4(-0.45f, -0.6f, 0.2f, 0), .dir = Normalise(v4(-1, -0.6f, 0, 0)), .pole = Normalise(v4(1, 0, 0.3f, 0)) };
				}
				case EPunchKind::Uppercut:
				{
					// The knees dip with the fist, then the body rises, driving the fist up and forward under the chin of the opponent
					return Style{ .yaw = 0.45f, .lean = 0.05f, .tilt = 0.12f, .dip = -0.08f, .sink = 0.3f, .pivot = 0.3f, .step = 0,
						.ctrl = v4(-0.1f, -0.35f, -0.15f, 0), .end = v4(-0.25f, -0.55f, 0.35f, 0), .dir = Normalise(v4(0, -0.3f, 1, 0)), .pole = Normalise(v4(0.3f, 0, -1, 0)) };
				}
				case EPunchKind::BodyHook:
				{
					// A hook to the body. The knees bend deeply and the body leans to the punching side, so the fist swings in at the opponent's ribs.
					return Style{ .yaw = 0.5f, .lean = 0.25f, .tilt = 0.25f, .dip = 0.35f, .sink = 0, .pivot = 0.4f, .step = 0,
						.ctrl = v4(0.25f, -0.4f, -0.1f, 0), .end = v4(-0.35f, -0.55f, -0.3f, 0), .dir = Normalise(v4(-1, -0.6f, 0.2f, 0)), .pole = Normalise(v4(1, 0, 0.3f, 0)) };
				}
				case EPunchKind::BodyJab:
				{
					// A straight punch to the body. The punching side's foot lunges forward and the body drops level with the opponent's chest.
					auto end = v4(-0.25f, -0.95f, -0.2f, 0);
					return Style{ .yaw = 0.3f, .lean = 0.35f, .tilt = 0, .dip = 0.4f, .sink = 0, .pivot = 0, .step = 0.25f,
						.ctrl = (GuardFist() + end) * 0.5f, .end = end, .dir = Normalise(v4(0, -1, -0.2f, 0)), .pole = Normalise(v4(0.5f, 0, -1, 0)) };
				}
				default: throw std::runtime_error("Unknown punch kind");
			}
		}

		// The left fist's guard position, in front of the chin, in model axes from the shoulder in units of the arm's reach
		static v4 GuardFist()
		{
			return v4(-0.2f, -0.4f, 0.2f, 0);
		}

		// Pose the body throwing the punch at 'time'. On guard, the left shoulder leads and the body crouches forward. Into each punch, the
		// punching shoulder turns forward and the chest leans after the fist, carrying the hips forward over the front foot, so the body's weight
		// goes into the punch and the fist reaches further. Punches to the body bend the knees and lean to the punching side.
		void Evaluate(float time, ActivityInput const& input, ActivityOutput const& output) const override
		{
			// Find the punch being thrown
			auto [punch, extend] = PunchAt(time);
			auto style = StyleOf(punch->kind);
			auto side = SideSign(punch->hand);
			auto leg = LegOn(input.rig, input.limbs, punch->hand);
			if (input.spine >= 0)
			{
				// The hips drop behind the guard, drop further or rise with the punch, and move forward with a share of the punch's lean. A stepping
				// foot carries the hips forward by a share of its step. The chest swings about the hips.
				auto const& guide = input.guide[input.spine];
				auto length = Length(guide.frame.pos - guide.base.pos);
				auto surge = PunchHipDrive * length * style.lean + (leg != nullptr ? StepSurge * style.step * leg->reach : 0.0f);
				auto drop = GuardCrouch + style.dip * extend + style.sink * 4 * extend * (1 - extend);
				auto drive = TorsoDrive{
					.yaw = -GuardYaw - side * style.yaw * extend,
					.lean = GuardLean + style.lean * extend,
					.tilt = side * style.tilt * extend,
					.hips = v4(0, -surge * extend, -drop * length, 0),
				};
				DriveChest(drive, guide.base.pos, guide.frame.pos);
				ApplyTorsoDrive(output.targets[input.spine], drive, PunchPelvisShare);
			}

			// The punching side's foot turns on its ball as the hips turn, turning its heel out (the same sign as the torso's turn), and steps
			// forward (-Y) and back along a low arc for punches that step in, lifting most half way out. The foot's planted position does not
			// change, so it returns to it.
			if (leg != nullptr)
			{
				auto step = style.step * leg->reach;
				auto offset = v4(0, -step * extend, StepLift * step * 4 * extend * (1 - extend), 0);
				PivotStep(input.rig, leg->chain, output.targets[leg->chain], -side * style.pivot * extend, offset);
			}

			// Aim the hands. Both hands hold a guard in front of the chin, and the punching hand drives out to the punch and back.
			auto chest = input.spine >= 0 ? output.targets[input.spine].frame : xform::Identity();
			for (auto const& limb : input.limbs)
			{
				// Only arms box. Their fists clench.
				auto const& chain = input.rig.chains[limb.chain];
				if (chain.role != EChainRole::Arm)
					continue;

				output.curl[limb.chain] = FistCurl;
				AimFist(input, limb, chain.side == punch->hand ? punch : nullptr, extend, chest, output.targets[limb.chain]);
			}
		}

	private:

		std::vector<Punch> m_punches;
		ActivityTimeline m_timeline;

		// Set the goal of the arm 'limb' for 'punch', or for the guard if it is null, with the fist 'extend' of the way out. The shoulder is carried
		// by the chest at 'chest'. Offsets are in model axes from the shoulder, in units of the arm's reach. Left-hand offsets mirror for the right arm.
		static void AimFist(ActivityInput const& input, ActivityLimb const& limb, Punch const* punch, float extend, xform const& chest, Target& target)
		{
			// The guard holds the fist up in front of the chin, the forearm upright and the elbow down
			auto side = SideSign(input.rig.chains[limb.chain].side);
			auto mirror = [side](v4 v)
			{
				// Reflect across the body's midline for the right arm
				return v4(side * v.x, v.y, v.z, v.w);
			};
			auto guard = mirror(GuardFist());
			auto arc = SwingArc{
				.rest = guard,
				.ctrl = guard,
				.end = guard,
				.rest_dir = Normalise(v4(0, -0.4f, 1, 0)),
				.end_dir = Normalise(v4(0, -0.4f, 1, 0)),
				.rest_pole = Normalise(v4(side * 0.5f, 0, -1, 0)),
				.end_pole = Normalise(v4(side * 0.5f, 0, -1, 0)),
			};
			auto root = LimbRoot(input, limb.chain, chest);
			auto s = 0.0f;
			if (punch != nullptr)
			{
				// The punch's path runs from the guard, through a control point, to its full extent
				auto style = StyleOf(punch->kind);
				arc.ctrl = mirror(style.ctrl);
				arc.end = mirror(style.end);
				arc.end_dir = mirror(style.dir);
				arc.end_pole = mirror(style.pole);
				s = extend;

				// Scale the punch's forward (-Y) path so its fist stops at the target's surface ahead of the shoulder. The shoulder moves with the
				// torso as the punch goes out, so the fist meets the surface when the punch is fully out. A target closer than the guard does not
				// pull the fist back behind it, and one beyond reach gets a straight arm, allowing for the punch's sideways and upward offset.
				if (input.targets.strike && arc.end.y < 0)
				{
					auto const& strike = *input.targets.strike;
					auto point = input.world_to_model * strike.point;
					auto ahead = (root.y - point.y - strike.radius) / limb.reach;
					auto most = std::sqrt(std::max(Sqr(StrikeMaxReach) - Sqr(arc.end.x) - Sqr(arc.end.z), 0.0f));
					auto forward = Clamp(ahead, std::min(-GuardFist().y, most), most);
					auto scale = forward / -arc.end.y;
					arc.ctrl.y *= scale;
					arc.end.y *= scale;
				}
			}

			// Follow the curved path, turning the fingers from the guard direction to the punch direction
			auto fingers = Rotate(limb.stance_rot, v4::YAxis());
			target.frame = xform(root + arc.Point(s) * limb.reach, Normalise(quat(fingers, arc.Direction(s)) * limb.stance_rot));
			target.pole = arc.Pole(s);
		}
	};
}
