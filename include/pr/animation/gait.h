//*****************************************************************************
// Procedural animation - gait description
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// A gait description guides how a character moves. It is extracted from example movement cycles (for example, walk cycles authored in
// Blender), but it holds only guidance: the stance, the footfall pattern, the stride, and the size and timing of the body's motion. The
// locomotion controller (see locomotion.h) generates the actual movement procedurally from this guidance and from the body's motion.
//
// The movement is grouped into families, such as "bipedal" and "quadruped". The caller chooses the family. Within a family, the cycles are
// ordered by speed, and the controller blends between the two cycles either side of the current speed.
//
// Lengths are in model units, speeds in model units per second, and angles in radians. A cycle's phase is in [0,1), and phase 0 is the start of
// the example cycle. Footfalls and body motion share this phase.
//
// A '.gait' file is a JSON gait description:
//
//  {
//    "schema_version": 3,
//    "families": [{
//      "name": "quadruped",
//      "stance": [{                        // The family's standing pose. Bones that are not listed keep their rest rotation.
//        "bone": "hips",
//        "rotation": [x, y, z, w],         // rotation relative to the parent bone, as a unit quaternion
//        "position": [x, y, z]             // root bones only, and optional: the model-space position
//      }],
//      "cycles": [{
//        "name": "walk",
//        "speed": 0.5,                     // the body's speed in the example cycle
//        "stride": 0.3,                    // the distance the body moves in one cycle of the example
//        "swing_height": 0.04,             // the height that a foot is lifted in the middle of its swing
//        "hip_height": 0.5,                // in (0,1]. The height of the hip joints above the ground, as a fraction of the legs' reach.
//        "footfalls": [{                   // the walking limbs. Every cycle of a family has the same limbs in the same order.
//          "role": "leg" | "arm",          // the limb is the rig chain with this role and side
//          "side": "left" | "right",
//          "touchdown": 0.0,               // in [0,1). The phase at which the foot touches the ground.
//          "duty": 0.75                    // in (0,1). The fraction of the cycle that the foot is on the ground.
//        }],
//        "motion": [{                      // the repeating motion of bones outside the walking limbs
//          "bone": "spine",
//          "channel": "rotation" | "position",
//          "harmonics": [                  // the first and second harmonics of the offset from the stance
//            { "cos": [x, y, z], "sin": [x, y, z] },
//            { "cos": [x, y, z], "sin": [x, y, z] }
//          ]
//        }]
//      }]
//    }]
//  }
//
// A "rotation" motion is a rotation vector (axis * angle) in the bone's own frame, applied after the bone's stance rotation. A "position" motion
// is a model-space offset of a root bone's stance position. Either is the sum over harmonics k = 1, 2 of cos(2 pi k phase) * cos_k + sin(2 pi k phase) * sin_k.
//
// The hip height sets how high the body is carried, independently of the authored poses: a crouching quadruped might use 0.5, and an upright
// biped 0.9. The legs are the walking limbs with the "leg" role (all walking limbs if there are none). A leg's reach is its rest length from its
// root joint to its contact, and its hip joint is the root joint of its chain.
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/rig_json.h"

namespace pr::animation
{
	// The '.gait' schema version that this code reads and writes
	inline constexpr int GaitSchemaVersion = 3;

	// The number of harmonics in each bone motion
	inline constexpr int GaitHarmonics = 2;

	// When a walking limb's foot is on the ground during a cycle
	struct Footfall
	{
		EChainRole role = EChainRole::Leg;
		ESide side = ESide::Left;
		float touchdown = 0.0f;     // In [0,1). The phase at which the foot touches the ground.
		float duty = 0.75f;         // In (0,1). The fraction of the cycle that the foot is on the ground.
	};

	// What a bone motion moves
	enum class EMotionChannel
	{
		// A rotation vector in the bone's own frame, applied after its stance rotation
		Rotation,

		// A model-space offset from a root bone's stance position
		Position,
	};

	// One harmonic of a bone motion. Directions have w = 0.
	struct Harmonic
	{
		v4 cos = {};
		v4 sin = {};
	};

	// The repeating motion of one bone over a cycle, as an offset from the stance
	struct BoneMotion
	{
		std::string bone;
		EMotionChannel channel = EMotionChannel::Rotation;
		std::array<Harmonic, GaitHarmonics> harmonics = {};
	};

	// One example movement cycle
	struct GaitCycle
	{
		std::string name;
		float speed = 1.0f;                // The body's speed in the example. Must be positive.
		float stride = 0.5f;               // The distance the body moves per cycle in the example. Must be positive.
		float swing_height = 0.05f;        // The height that a foot is lifted in the middle of its swing. Must not be negative.
		float hip_height = 0.5f;           // In (0,1]. The height of the hip joints above the ground, as a fraction of the legs' reach.
		std::vector<Footfall> footfalls;   // The walking limbs
		std::vector<BoneMotion> motion;    // The repeating motion of bones outside the walking limbs
	};

	// One bone of a family's stance
	struct StanceBone
	{
		std::string bone;
		quat rotation = Identity<quat>();  // Relative to the parent bone
		std::optional<v4> position;        // Root bones only: the model-space position (w = 1)
	};

	// A group of movement cycles that share a stance and a set of walking limbs
	struct GaitFamily
	{
		std::string name;
		std::vector<StanceBone> stance;
		std::vector<GaitCycle> cycles;     // Ordered by increasing speed
	};

	// How a character moves. See the top of this file.
	struct GaitDesc
	{
		std::vector<GaitFamily> families;
	};

	// The names of 'EMotionChannel' values in the file, in enum order
	inline constexpr std::array<std::string_view, 2> MotionChannelNames = { "rotation", "position" };

	// Evaluate a bone motion at 'phase'
	inline v4 Evaluate(std::array<Harmonic, GaitHarmonics> const& harmonics, float phase)
	{
		// Harmonic k repeats k times per cycle
		auto value = v4::Zero();
		for (int k = 0; k != GaitHarmonics; ++k)
		{
			auto angle = math::constants<float>::tau * static_cast<float>(k + 1) * phase;
			value += harmonics[k].cos * std::cos(angle) + harmonics[k].sin * std::sin(angle);
		}
		return value;
	}

	// The rotation given by a rotation vector (axis * angle, w = 0)
	inline quat RotationFromVector(v4 const& rotation_vector)
	{
		// A zero vector has no axis
		auto angle = Length(rotation_vector);
		return angle > tiny<float> ? quat(rotation_vector / angle, angle) : Identity<quat>();
	}

	// Return the index of the contact that a limb chain stands on (a sole for legs, a palm for arms), or -1 if it has none
	inline int LimbContact(Rig const& rig, Chain const& chain)
	{
		// The contact is on the chain's end bone
		auto kind = chain.role == EChainRole::Leg ? EContactKind::Sole : EContactKind::Palm;
		for (int i = 0, iend = static_cast<int>(rig.contacts.size()); i != iend; ++i)
		{
			if (rig.contacts[i].bone == chain.bones.back() && rig.contacts[i].kind == kind)
				return i;
		}
		return -1;
	}

	// Return the index of the Leg or Arm chain of 'rig' with 'role' and 'side', or -1 if there is none
	inline int LimbChain(Rig const& rig, EChainRole role, ESide side)
	{
		// Rigs have at most one chain per role and side
		for (int c = 0, cend = static_cast<int>(rig.chains.size()); c != cend; ++c)
		{
			if (rig.chains[c].role == role && rig.chains[c].side == side)
				return c;
		}
		return -1;
	}

	// Return the index of the family called 'name', or -1 if there is none
	inline int FamilyIndex(GaitDesc const& gait, std::string_view name)
	{
		// Gaits have few families
		for (int i = 0, iend = static_cast<int>(gait.families.size()); i != iend; ++i)
		{
			if (gait.families[i].name == name)
				return i;
		}
		return -1;
	}

	// Check 'gait' against 'rig'. Returns a description of each problem, or an empty list if the gait is valid.
	inline std::vector<std::string> Validate(GaitDesc const& gait, Rig const& rig)
	{
		std::vector<std::string> errors;

		// A gait needs at least one family, and families are chosen by name
		if (gait.families.empty())
			errors.push_back("The gait has no families");

		for (size_t f = 0; f != gait.families.size(); ++f)
		{
			// Family names must be unique and each family needs a cycle
			auto const& family = gait.families[f];
			auto ctx = std::format("Family '{}'", family.name);
			if (FamilyIndex(gait, family.name) != static_cast<int>(f))
				errors.push_back(std::format("{} repeats an earlier family's name", ctx));
			if (family.cycles.empty())
				errors.push_back(std::format("{} has no cycles", ctx));

			// Stance bones must exist, appear once, and give positions only for root bones
			for (size_t s = 0; s != family.stance.size(); ++s)
			{
				auto const& bone = family.stance[s];
				auto index = rig.BoneIndex(bone.bone);
				if (index < 0)
					errors.push_back(std::format("{} stance names unknown bone '{}'", ctx, bone.bone));
				else if (bone.position && rig.bones[index].parent >= 0)
					errors.push_back(std::format("{} stance gives a position for '{}', which is not a root bone", ctx, bone.bone));

				for (size_t t = 0; t != s; ++t)
				{
					if (family.stance[t].bone == bone.bone)
						errors.push_back(std::format("{} stance repeats bone '{}'", ctx, bone.bone));
				}
			}

			for (size_t c = 0; c != family.cycles.size(); ++c)
			{
				// Cycles are ordered by speed so that the controller can find the two either side of a speed
				auto const& cycle = family.cycles[c];
				auto cctx = std::format("{} cycle '{}'", ctx, cycle.name);
				if (!(cycle.speed > 0) || (c != 0 && !(cycle.speed > family.cycles[c - 1].speed)))
					errors.push_back(std::format("{} must have a positive speed, faster than the cycle before it", cctx));
				if (!(cycle.stride > 0) || !(cycle.swing_height >= 0))
					errors.push_back(std::format("{} must have a positive stride and a swing height that is not negative", cctx));
				if (!(cycle.hip_height > 0 && cycle.hip_height <= 1))
					errors.push_back(std::format("{} must have a hip height in (0,1]", cctx));

				// The walking limbs must match the family's first cycle, so that cycles blend limb by limb
				auto const& first = family.cycles[0].footfalls;
				if (cycle.footfalls.size() != first.size())
					errors.push_back(std::format("{} has different walking limbs to the family's first cycle", cctx));

				for (size_t i = 0; i != cycle.footfalls.size(); ++i)
				{
					// Each footfall names a distinct limb chain that has a contact to stand on
					auto const& foot = cycle.footfalls[i];
					auto fctx = std::format("{} footfall {}", cctx, i);
					if (foot.role != EChainRole::Leg && foot.role != EChainRole::Arm)
					{
						errors.push_back(std::format("{} is not a leg or an arm", fctx));
						continue;
					}
					auto chain = LimbChain(rig, foot.role, foot.side);
					if (chain < 0)
						errors.push_back(std::format("{} has no matching chain in the rig", fctx));
					else if (LimbContact(rig, rig.chains[chain]) < 0)
						errors.push_back(std::format("{} has no sole or palm contact on its end bone", fctx));
					if (!(foot.touchdown >= 0 && foot.touchdown < 1) || !(foot.duty > 0 && foot.duty < 1))
						errors.push_back(std::format("{} must have a touchdown in [0,1) and a duty in (0,1)", fctx));
					if (i < first.size() && (first[i].role != foot.role || first[i].side != foot.side))
						errors.push_back(std::format("{} is not the same limb as in the family's first cycle", fctx));

					for (size_t j = 0; j != i; ++j)
					{
						if (cycle.footfalls[j].role == foot.role && cycle.footfalls[j].side == foot.side)
							errors.push_back(std::format("{} repeats footfall {}", fctx, j));
					}
				}

				for (auto const& motion : cycle.motion)
				{
					// Motion bones must exist, and only root bones have a position motion
					auto index = rig.BoneIndex(motion.bone);
					if (index < 0)
						errors.push_back(std::format("{} motion names unknown bone '{}'", cctx, motion.bone));
					else if (motion.channel == EMotionChannel::Position && rig.bones[index].parent >= 0)
						errors.push_back(std::format("{} has a position motion for '{}', which is not a root bone", cctx, motion.bone));
				}
			}
		}
		return errors;
	}

	// Read a gait description from a parsed '.gait' document. Throws 'std::runtime_error' if the document does not follow the schema.
	// The result is not checked against a rig; call 'Validate'.
	inline GaitDesc ReadGaitDesc(json::Value const& doc)
	{
		using namespace rig_json;
		GaitDesc gait;

		// Check the schema version before reading anything else
		auto const& root = ReadObject(doc, "gait");
		auto version = static_cast<int>(ReadFloat(Member(root, "schema_version", "gait"), "gait.schema_version"));
		if (version != GaitSchemaVersion)
			throw std::runtime_error(std::format("gait.schema_version: unsupported version {} (expected {})", version, GaitSchemaVersion));

		auto const& families = ReadArray(Member(root, "families", "gait"), "gait.families");
		for (size_t f = 0; f != families.size(); ++f)
		{
			// Read the family's name and stance
			auto fctx = std::format("gait.families[{}]", f);
			auto const& fobj = ReadObject(families[f], fctx);
			auto& family = gait.families.emplace_back(GaitFamily{ .name = ReadString(Member(fobj, "name", fctx), fctx + ".name") });
			auto const& stance = ReadArray(Member(fobj, "stance", fctx), fctx + ".stance");
			for (size_t s = 0; s != stance.size(); ++s)
			{
				// The position is optional
				auto sctx = std::format("{}.stance[{}]", fctx, s);
				auto const& sobj = ReadObject(stance[s], sctx);
				auto& bone = family.stance.emplace_back(StanceBone{
					.bone = ReadString(Member(sobj, "bone", sctx), sctx + ".bone"),
					.rotation = ReadQuat(Member(sobj, "rotation", sctx), sctx + ".rotation"),
				});
				if (auto position = sobj.find("position"))
					bone.position = ReadVec(*position, 1, sctx + ".position");
			}

			auto const& cycles = ReadArray(Member(fobj, "cycles", fctx), fctx + ".cycles");
			for (size_t c = 0; c != cycles.size(); ++c)
			{
				// Read the cycle's measurements
				auto cctx = std::format("{}.cycles[{}]", fctx, c);
				auto const& cobj = ReadObject(cycles[c], cctx);
				auto& cycle = family.cycles.emplace_back(GaitCycle{
					.name = ReadString(Member(cobj, "name", cctx), cctx + ".name"),
					.speed = ReadFloat(Member(cobj, "speed", cctx), cctx + ".speed"),
					.stride = ReadFloat(Member(cobj, "stride", cctx), cctx + ".stride"),
					.swing_height = ReadFloat(Member(cobj, "swing_height", cctx), cctx + ".swing_height"),
					.hip_height = ReadFloat(Member(cobj, "hip_height", cctx), cctx + ".hip_height"),
				});

				// Read the footfalls
				auto const& footfalls = ReadArray(Member(cobj, "footfalls", cctx), cctx + ".footfalls");
				for (size_t i = 0; i != footfalls.size(); ++i)
				{
					auto ictx = std::format("{}.footfalls[{}]", cctx, i);
					auto const& obj = ReadObject(footfalls[i], ictx);
					cycle.footfalls.push_back(Footfall{
						.role = ParseEnum<EChainRole>(ReadString(Member(obj, "role", ictx), ictx + ".role"), ChainRoleNames, ictx + ".role"),
						.side = ParseEnum<ESide>(ReadString(Member(obj, "side", ictx), ictx + ".side"), SideNames, ictx + ".side"),
						.touchdown = ReadFloat(Member(obj, "touchdown", ictx), ictx + ".touchdown"),
						.duty = ReadFloat(Member(obj, "duty", ictx), ictx + ".duty"),
					});
				}

				// Read the bone motions, which have exactly one entry per harmonic
				auto const& motions = ReadArray(Member(cobj, "motion", cctx), cctx + ".motion");
				for (size_t m = 0; m != motions.size(); ++m)
				{
					auto mctx = std::format("{}.motion[{}]", cctx, m);
					auto const& obj = ReadObject(motions[m], mctx);
					auto& motion = cycle.motion.emplace_back(BoneMotion{
						.bone = ReadString(Member(obj, "bone", mctx), mctx + ".bone"),
						.channel = ParseEnum<EMotionChannel>(ReadString(Member(obj, "channel", mctx), mctx + ".channel"), MotionChannelNames, mctx + ".channel"),
					});
					auto const& harmonics = ReadArray(Member(obj, "harmonics", mctx), mctx + ".harmonics");
					if (harmonics.size() != GaitHarmonics)
						throw std::runtime_error(std::format("{}.harmonics: expected {} harmonics", mctx, GaitHarmonics));

					for (int k = 0; k != GaitHarmonics; ++k)
					{
						auto hctx = std::format("{}.harmonics[{}]", mctx, k);
						auto const& hobj = ReadObject(harmonics[k], hctx);
						motion.harmonics[k].cos = ReadVec(Member(hobj, "cos", hctx), 0, hctx + ".cos");
						motion.harmonics[k].sin = ReadVec(Member(hobj, "sin", hctx), 0, hctx + ".sin");
					}
				}
			}
		}
		return gait;
	}

	// Write a gait description as a '.gait' document
	inline json::Document WriteGaitDesc(GaitDesc const& gait)
	{
		using namespace rig_json;
		json::Document doc;
		auto& root = doc.root();
		root["schema_version"] = GaitSchemaVersion;

		json::Array families;
		for (auto const& family : gait.families)
		{
			// Write the stance. Only root bones may have a position.
			json::Array stance;
			for (auto const& bone : family.stance)
			{
				json::Object obj{ { "bone", bone.bone }, { "rotation", WriteQuat(bone.rotation) } };
				if (bone.position)
					obj["position"] = WriteVec(*bone.position);

				stance.push_back(std::move(obj));
			}

			json::Array cycles;
			for (auto const& cycle : family.cycles)
			{
				// Write the footfalls and the bone motions
				json::Array footfalls;
				for (auto const& foot : cycle.footfalls)
				{
					footfalls.push_back(json::Object{
						{ "role", EnumName(foot.role, ChainRoleNames) },
						{ "side", EnumName(foot.side, SideNames) },
						{ "touchdown", foot.touchdown },
						{ "duty", foot.duty },
					});
				}
				json::Array motions;
				for (auto const& motion : cycle.motion)
				{
					json::Array harmonics;
					for (auto const& h : motion.harmonics)
						harmonics.push_back(json::Object{ { "cos", WriteVec(h.cos) }, { "sin", WriteVec(h.sin) } });

					motions.push_back(json::Object{
						{ "bone", motion.bone },
						{ "channel", EnumName(motion.channel, MotionChannelNames) },
						{ "harmonics", std::move(harmonics) },
					});
				}
				cycles.push_back(json::Object{
					{ "name", cycle.name },
					{ "speed", cycle.speed },
					{ "stride", cycle.stride },
					{ "swing_height", cycle.swing_height },
					{ "hip_height", cycle.hip_height },
					{ "footfalls", std::move(footfalls) },
					{ "motion", std::move(motions) },
				});
			}
			families.push_back(json::Object{
				{ "name", family.name },
				{ "stance", std::move(stance) },
				{ "cycles", std::move(cycles) },
			});
		}
		root["families"] = std::move(families);
		return doc;
	}

	// Read and write '.gait' files
	inline GaitDesc LoadGaitDesc(std::filesystem::path const& filepath)
	{
		// Schema errors name the file so that cook errors are actionable
		try
		{
			return ReadGaitDesc(json::Read(filepath));
		}
		catch (std::exception const& ex)
		{
			throw std::runtime_error(std::format("{}: {}", filepath.string(), ex.what()));
		}
	}
	inline void SaveGaitDesc(std::filesystem::path const& filepath, GaitDesc const& gait)
	{
		// Indented output keeps cooked gaits readable and diffable
		json::Write(filepath, WriteGaitDesc(gait));
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	// A gait for the test rig (see 'TestRigDesc'), whose stances are its rest pose:
	//  - "quadruped": the leg and arm step half a cycle apart, in a walk and a faster trot. The walk sways the spine and bobs the root.
	//  - "bipedal": only the leg steps, and the arm hangs.
	inline GaitDesc TestGait()
	{
		// Both families stand in the rest pose, so their stances are empty. The rest pose carries the leg's hip joint at z = 1, and the leg reaches
		// from (0.2,0,1) through (0.2,-0.1,0.5) and (0.2,0,0) to its sole at (0.2,-0.1,0).
		auto rest_hip_height = 1.0f / (2.0f * Sqrt(0.26f) + 0.1f);
		auto walk = GaitCycle{
			.name = "walk",
			.speed = 0.8f,
			.stride = 0.5f,
			.swing_height = 0.1f,
			.hip_height = rest_hip_height,
			.footfalls = {
				Footfall{ .role = EChainRole::Leg, .side = ESide::Left, .touchdown = 0.0f, .duty = 0.75f },
				Footfall{ .role = EChainRole::Arm, .side = ESide::Left, .touchdown = 0.5f, .duty = 0.75f },
			},
			.motion = {
				BoneMotion{ .bone = "spine", .channel = EMotionChannel::Rotation, .harmonics = { Harmonic{ .sin = v4(0, 0, 0.1f, 0) }, Harmonic{} } },
				BoneMotion{ .bone = "root", .channel = EMotionChannel::Position, .harmonics = { Harmonic{}, Harmonic{ .cos = v4(0, 0, 0.02f, 0) } } },
			},
		};
		auto trot = GaitCycle{
			.name = "trot",
			.speed = 1.6f,
			.stride = 0.8f,
			.swing_height = 0.15f,
			.hip_height = rest_hip_height,
			.footfalls = {
				Footfall{ .role = EChainRole::Leg, .side = ESide::Left, .touchdown = 0.0f, .duty = 0.5f },
				Footfall{ .role = EChainRole::Arm, .side = ESide::Left, .touchdown = 0.5f, .duty = 0.5f },
			},
		};
		auto biped = GaitCycle{
			.name = "walk",
			.speed = 0.8f,
			.stride = 0.5f,
			.swing_height = 0.1f,
			.hip_height = rest_hip_height,
			.footfalls = { Footfall{ .role = EChainRole::Leg, .side = ESide::Left, .touchdown = 0.0f, .duty = 0.6f } },
		};
		return GaitDesc{
			.families = {
				GaitFamily{ .name = "quadruped", .cycles = { walk, trot } },
				GaitFamily{ .name = "bipedal", .cycles = { biped } },
			},
		};
	}

	PRUnitTestClass(AnimationGaitTests)
	{
		PRUnitTestMethod(RoundTrip, Quick)
		{
			// Writing then reading a gait reproduces it, including stance positions and harmonics
			auto rig = TestRig();
			auto gait = TestGait();
			gait.families[0].stance.push_back(StanceBone{ .bone = "root", .rotation = quat(v4::ZAxis(), 0.1f), .position = v4(0, 0, 0.1f, 1) });
			gait.families[0].stance.push_back(StanceBone{ .bone = "spine", .rotation = quat(v4::XAxis(), 0.2f) });
			PR_EXPECT(Validate(gait, rig).empty());

			auto text = json::Write(WriteGaitDesc(gait));
			auto read = ReadGaitDesc(json::Read(std::string_view(text)));
			PR_EXPECT(Validate(read, rig).empty());
			PR_EXPECT(read.families.size() == 2);
			PR_EXPECT(read.families[0].name == "quadruped" && read.families[1].name == "bipedal");

			auto const& stance = read.families[0].stance;
			PR_EXPECT(stance.size() == 2 && stance[0].position && !stance[1].position);
			PR_EXPECT(FEqlAbsolute(*stance[0].position, v4(0, 0, 0.1f, 1), 1e-6f));
			PR_EXPECT(FEqlOrientation(stance[1].rotation, quat(v4::XAxis(), 0.2f), 1e-6f));

			auto const& l = read.families[0].cycles[0];
			auto const& r = gait.families[0].cycles[0];
			PR_EXPECT(l.name == r.name && l.speed == r.speed && l.stride == r.stride && l.swing_height == r.swing_height && l.hip_height == r.hip_height);
			PR_EXPECT(l.footfalls.size() == 2 && l.footfalls[1].role == EChainRole::Arm && l.footfalls[1].touchdown == 0.5f && l.footfalls[1].duty == 0.75f);
			PR_EXPECT(l.motion.size() == 2 && l.motion[1].channel == EMotionChannel::Position);
			PR_EXPECT(FEqlAbsolute(l.motion[0].harmonics[0].sin, r.motion[0].harmonics[0].sin, 1e-6f));
			PR_EXPECT(FEqlAbsolute(l.motion[1].harmonics[1].cos, r.motion[1].harmonics[1].cos, 1e-6f));
		}
		PRUnitTestMethod(EvaluateHarmonics, Quick)
		{
			// The first harmonic repeats once per cycle and the second twice
			auto harmonics = std::array<Harmonic, GaitHarmonics>{ Harmonic{ .sin = v4(1, 0, 0, 0) }, Harmonic{ .cos = v4(0, 1, 0, 0) } };
			PR_EXPECT(FEqlAbsolute(Evaluate(harmonics, 0.0f), v4(0, 1, 0, 0), 1e-6f));
			PR_EXPECT(FEqlAbsolute(Evaluate(harmonics, 0.25f), v4(1, -1, 0, 0), 1e-5f));
			PR_EXPECT(FEqlAbsolute(Evaluate(harmonics, 0.5f), v4(0, 1, 0, 0), 1e-5f));
		}
		PRUnitTestMethod(Errors, Quick)
		{
			// Schema errors name the field
			auto rig = TestRig();
			auto text = json::Write(WriteGaitDesc(TestGait()));
			auto bad = text;
			bad.replace(bad.find("\"leg\""), 5, "\"wing\"");
			try
			{
				(void)ReadGaitDesc(json::Read(std::string_view(bad)));
				PR_EXPECT(false && "Expected a schema error");
			}
			catch (std::runtime_error const& ex)
			{
				PR_EXPECT(std::string_view(ex.what()).find("gait.families[0].cycles[0].footfalls[0].role") != std::string_view::npos);
			}

			// Gaits that do not fit the rig are reported: unordered speeds, a limb missing from one cycle, and a position on a non-root bone
			auto gait = TestGait();
			auto& quad = gait.families[0];
			quad.cycles[1].speed = quad.cycles[0].speed;
			quad.cycles[1].footfalls.pop_back();
			quad.cycles[0].motion[0].channel = EMotionChannel::Position;
			PR_EXPECT(Validate(gait, rig).size() == 3);
		}
	};
}
#endif
