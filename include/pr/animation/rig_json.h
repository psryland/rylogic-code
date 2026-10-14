//*****************************************************************************
// Procedural animation - rig description files
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// A '.rig' file is a JSON rig description. Bones, chains and contacts refer to bones by name. Angles are in radians and lengths are in model units.
//
//  {
//    "schema_version": 3,
//    "bones": [{
//      "name": "thigh.L",
//      "parent": "hips",                 // or null for a root bone
//      "position": [x, y, z],            // rest position relative to the parent bone (model space for root bones)
//      "rotation": [x, y, z, w],         // rest rotation relative to the parent bone, as a unit quaternion
//      "length": 0.1,                    // head to tip, along the bone's local +Y axis
//      "limit": {                        // optional. The rotation range from the rest pose (see 'JointLimit'). No limit allows any rotation.
//        "swing_x": [-0.3, 1.2],         //   [min, max] swing about the bone's rest +X axis
//        "swing_z": [0.0, 0.0],          //   [min, max] swing about the bone's rest +Z axis. A zero-width range makes a hinge about +X.
//        "twist": [-0.1, 0.1]            //   [min, max] twist about the bone's +Y axis. Each range is within [-pi, pi] and contains 0.
//      },
//      "stiffness": 0.2                  // optional, in [0,1]
//    }],
//    "chains": [{
//      "role": "spine" | "neck" | "leg" | "arm" | "tail" | "digit",
//      "side": "centre" | "left" | "right",
//      "bones": ["thigh.L", "shin.L", "foot.L"],  // from base to end
//      "pole": [x, y, z],                // leg and arm chains: rest-pose model-space direction that the middle joint bends towards
//      "forward": [x, y, z]              // neck chains: rest-pose model-space direction that the last bone faces
//    }],
//    "contacts": [{
//      "name": "sole.L",
//      "bone": "foot.L",
//      "kind": "sole" | "toe" | "palm" | "finger" | "tail",
//      "offset": [x, y, z],              // bone-space point
//      "normal": [x, y, z]               // bone-space outward surface direction
//    }],
//    "body_axis": {                      // the line from the head of bone 'from' to the head of bone 'to' that a body collision shape follows
//      "from": "hips",
//      "to": "neck"
//    }
//  }
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"

namespace pr::animation
{
	// The '.rig' schema version that this code reads and writes
	inline constexpr int RigSchemaVersion = 3;

	namespace rig_json
	{
		// The file names of the description enums. Each list is in enum order.
		inline constexpr std::array<std::string_view, 6> ChainRoleNames = { "spine", "neck", "leg", "arm", "tail", "digit" };
		inline constexpr std::array<std::string_view, 3> SideNames = { "centre", "left", "right" };
		inline constexpr std::array<std::string_view, 5> ContactKindNames = { "sole", "toe", "palm", "finger", "tail" };

		// Return the file name of an enum value
		template <typename E, size_t N>
		inline std::string_view EnumName(E value, std::array<std::string_view, N> const& names)
		{
			// The enum values are the indices into 'names'
			auto index = static_cast<size_t>(value);
			if (index >= N)
				throw std::invalid_argument(std::format("Enum value {} has no file name", index));

			return names[index];
		}

		// Return the enum value for a file name
		template <typename E, size_t N>
		inline E ParseEnum(std::string_view name, std::array<std::string_view, N> const& names, std::string_view context)
		{
			// The enum values are the indices into 'names'
			auto it = std::find(names.begin(), names.end(), name);
			if (it == names.end())
				throw std::runtime_error(std::format("{}: unknown value '{}'", context, name));

			return static_cast<E>(it - names.begin());
		}

		// Return the member 'key' of 'obj', throwing if it is missing
		inline json::Value const& Member(json::Object const& obj, std::string_view key, std::string_view context)
		{
			// Required members must be present
			auto value = obj.find(key);
			if (value == nullptr)
				throw std::runtime_error(std::format("{}: missing '{}'", context, key));

			return *value;
		}

		// Read typed values, throwing with 'context' if the value has the wrong type
		inline json::Object const& ReadObject(json::Value const& value, std::string_view context)
		{
			// Objects are the only compound value with named members
			auto obj = value.as<json::Object>();
			if (obj == nullptr)
				throw std::runtime_error(std::format("{}: expected an object", context));

			return *obj;
		}
		inline json::Array const& ReadArray(json::Value const& value, std::string_view context)
		{
			// Lists of bones, chains and contacts are arrays
			auto arr = value.as<json::Array>();
			if (arr == nullptr)
				throw std::runtime_error(std::format("{}: expected an array", context));

			return *arr;
		}
		inline std::string const& ReadString(json::Value const& value, std::string_view context)
		{
			// Names and enum values are strings
			auto str = value.as<std::string>();
			if (str == nullptr)
				throw std::runtime_error(std::format("{}: expected a string", context));

			return *str;
		}
		inline float ReadFloat(json::Value const& value, std::string_view context)
		{
			// JSON numbers are doubles; the rig uses floats
			auto num = value.as<double>();
			if (num == nullptr)
				throw std::runtime_error(std::format("{}: expected a number", context));

			return static_cast<float>(*num);
		}
		inline v4 ReadVec(json::Value const& value, float w, std::string_view context)
		{
			// Points and directions are stored as three numbers. 'w' says which they are.
			auto const& arr = ReadArray(value, context);
			if (arr.size() != 3)
				throw std::runtime_error(std::format("{}: expected 3 numbers", context));

			return v4(ReadFloat(arr[0], context), ReadFloat(arr[1], context), ReadFloat(arr[2], context), w);
		}
		inline quat ReadQuat(json::Value const& value, std::string_view context)
		{
			// Rotations are stored as [x, y, z, w]
			auto const& arr = ReadArray(value, context);
			if (arr.size() != 4)
				throw std::runtime_error(std::format("{}: expected 4 numbers", context));

			return quat(ReadFloat(arr[0], context), ReadFloat(arr[1], context), ReadFloat(arr[2], context), ReadFloat(arr[3], context));
		}
		inline std::pair<float, float> ReadRange(json::Value const& value, std::string_view context)
		{
			// Ranges are stored as [min, max]
			auto const& arr = ReadArray(value, context);
			if (arr.size() != 2)
				throw std::runtime_error(std::format("{}: expected 2 numbers", context));

			return { ReadFloat(arr[0], context), ReadFloat(arr[1], context) };
		}

		// Write vectors and rotations in the file layout
		inline json::Array WriteVec(v4 const& v)
		{
			// The 'w' component is implied by the field
			return json::Array{ v.x, v.y, v.z };
		}
		inline json::Array WriteQuat(quat const& q)
		{
			// Rotations are stored as [x, y, z, w]
			return json::Array{ q.x, q.y, q.z, q.w };
		}
	}

	// Read a rig description from a parsed '.rig' document. Throws 'std::runtime_error' if the document does not follow the schema.
	// The result is not validated; call 'Validate' or construct a 'Rig' from it.
	inline RigDesc ReadRigDesc(json::Value const& doc)
	{
		using namespace rig_json;
		RigDesc desc;

		// Check the schema version before reading anything else
		auto const& root = ReadObject(doc, "rig");
		auto version = static_cast<int>(ReadFloat(Member(root, "schema_version", "rig"), "rig.schema_version"));
		if (version != RigSchemaVersion)
			throw std::runtime_error(std::format("rig.schema_version: unsupported version {} (expected {})", version, RigSchemaVersion));

		// Bones are referred to by name, so read all bone names before resolving any references
		auto const& bones = ReadArray(Member(root, "bones", "rig"), "rig.bones");
		std::vector<std::string> names;
		for (size_t i = 0; i != bones.size(); ++i)
		{
			auto context = std::format("rig.bones[{}]", i);
			names.push_back(ReadString(Member(ReadObject(bones[i], context), "name", context), context + ".name"));
		}
		auto bone_index = [&](json::Value const& value, std::string_view context)
		{
			// Return the index of a referenced bone, which must exist
			auto const& name = ReadString(value, context);
			auto it = std::find(names.begin(), names.end(), name);
			if (it == names.end())
				throw std::runtime_error(std::format("{}: unknown bone '{}'", context, name));

			return static_cast<int>(it - names.begin());
		};

		// Read the bones
		for (size_t i = 0; i != bones.size(); ++i)
		{
			// Required members
			auto context = std::format("rig.bones[{}]", i);
			auto const& obj = ReadObject(bones[i], context);
			auto const& parent = Member(obj, "parent", context);
			auto& bone = desc.bones.emplace_back(Bone{
				.name = names[i],
				.parent = parent == nullptr ? -1 : bone_index(parent, context + ".parent"),
				.rest = xform(ReadVec(Member(obj, "position", context), 1, context + ".position"), ReadQuat(Member(obj, "rotation", context), context + ".rotation")),
				.length = ReadFloat(Member(obj, "length", context), context + ".length"),
			});

			// Optional members
			if (auto stiffness = obj.find("stiffness"))
				bone.stiffness = ReadFloat(*stiffness, context + ".stiffness");

			if (auto limit_value = obj.find("limit"))
			{
				// A limit gives all three ranges
				auto limit_context = context + ".limit";
				auto const& limit = ReadObject(*limit_value, limit_context);
				std::tie(bone.limit.swing.min_x, bone.limit.swing.max_x) = ReadRange(Member(limit, "swing_x", limit_context), limit_context + ".swing_x");
				std::tie(bone.limit.swing.min_z, bone.limit.swing.max_z) = ReadRange(Member(limit, "swing_z", limit_context), limit_context + ".swing_z");
				std::tie(bone.limit.min_twist, bone.limit.max_twist) = ReadRange(Member(limit, "twist", limit_context), limit_context + ".twist");
			}
		}

		// Read the chains
		auto const& chains = ReadArray(Member(root, "chains", "rig"), "rig.chains");
		for (size_t i = 0; i != chains.size(); ++i)
		{
			// Required members
			auto context = std::format("rig.chains[{}]", i);
			auto const& obj = ReadObject(chains[i], context);
			auto& chain = desc.chains.emplace_back(Chain{
				.role = ParseEnum<EChainRole>(ReadString(Member(obj, "role", context), context + ".role"), ChainRoleNames, context + ".role"),
				.side = ParseEnum<ESide>(ReadString(Member(obj, "side", context), context + ".side"), SideNames, context + ".side"),
			});
			auto const& chain_bones = ReadArray(Member(obj, "bones", context), context + ".bones");
			for (size_t k = 0; k != chain_bones.size(); ++k)
				chain.bones.push_back(bone_index(chain_bones[k], std::format("{}.bones[{}]", context, k)));

			// Role-specific members. Validation reports them missing where the role needs them.
			if (auto pole = obj.find("pole"))
				chain.pole = ReadVec(*pole, 0, context + ".pole");
			if (auto forward = obj.find("forward"))
				chain.forward = ReadVec(*forward, 0, context + ".forward");
		}

		// Read the contacts
		auto const& contacts = ReadArray(Member(root, "contacts", "rig"), "rig.contacts");
		for (size_t i = 0; i != contacts.size(); ++i)
		{
			// All contact members are required
			auto context = std::format("rig.contacts[{}]", i);
			auto const& obj = ReadObject(contacts[i], context);
			desc.contacts.push_back(Contact{
				.name = ReadString(Member(obj, "name", context), context + ".name"),
				.bone = bone_index(Member(obj, "bone", context), context + ".bone"),
				.offset = ReadVec(Member(obj, "offset", context), 1, context + ".offset"),
				.normal = ReadVec(Member(obj, "normal", context), 0, context + ".normal"),
				.kind = ParseEnum<EContactKind>(ReadString(Member(obj, "kind", context), context + ".kind"), ContactKindNames, context + ".kind"),
			});
		}

		// Read the body axis
		auto const& body_axis = ReadObject(Member(root, "body_axis", "rig"), "rig.body_axis");
		desc.body_axis = BodyAxis{
			.from = bone_index(Member(body_axis, "from", "rig.body_axis"), "rig.body_axis.from"),
			.to = bone_index(Member(body_axis, "to", "rig.body_axis"), "rig.body_axis.to"),
		};
		return desc;
	}

	// Write a rig description as a '.rig' document. Bone references must be valid indices.
	inline json::Document WriteRigDesc(RigDesc const& desc)
	{
		using namespace rig_json;
		json::Document doc;
		auto& root = doc.root();
		root["schema_version"] = RigSchemaVersion;

		// Write the bones, with optional members only when they differ from their defaults
		json::Array bones;
		for (auto const& bone : desc.bones)
		{
			json::Object obj;
			obj["name"] = bone.name;
			obj["parent"] = bone.parent < 0 ? json::Value(nullptr) : json::Value(desc.bones[bone.parent].name);
			obj["position"] = WriteVec(bone.rest.pos);
			obj["rotation"] = WriteQuat(bone.rest.rot);
			obj["length"] = bone.length;
			if (!bone.limit.IsFree())
			{
				obj["limit"] = json::Object{
					{ "swing_x", json::Array{ bone.limit.swing.min_x, bone.limit.swing.max_x } },
					{ "swing_z", json::Array{ bone.limit.swing.min_z, bone.limit.swing.max_z } },
					{ "twist", json::Array{ bone.limit.min_twist, bone.limit.max_twist } },
				};
			}
			if (bone.stiffness != 0)
				obj["stiffness"] = bone.stiffness;

			bones.push_back(std::move(obj));
		}
		root["bones"] = std::move(bones);

		// Write the chains, with the direction that each role uses
		json::Array chains;
		for (auto const& chain : desc.chains)
		{
			json::Object obj;
			obj["role"] = EnumName(chain.role, ChainRoleNames);
			obj["side"] = EnumName(chain.side, SideNames);

			json::Array chain_bones;
			for (auto b : chain.bones)
				chain_bones.push_back(desc.bones[b].name);

			obj["bones"] = std::move(chain_bones);
			switch (chain.role)
			{
				case EChainRole::Leg:
				case EChainRole::Arm:
				{
					obj["pole"] = WriteVec(chain.pole);
					break;
				}
				case EChainRole::Neck:
				{
					obj["forward"] = WriteVec(chain.forward);
					break;
				}
				case EChainRole::Spine:
				case EChainRole::Tail:
				case EChainRole::Digit:
				{
					break;
				}
				default:
				{
					throw std::invalid_argument("Chain has an unknown role");
				}
			}
			chains.push_back(std::move(obj));
		}
		root["chains"] = std::move(chains);

		// Write the contacts
		json::Array contacts;
		for (auto const& contact : desc.contacts)
		{
			contacts.push_back(json::Object{
				{ "name", contact.name },
				{ "bone", desc.bones[contact.bone].name },
				{ "kind", EnumName(contact.kind, ContactKindNames) },
				{ "offset", WriteVec(contact.offset) },
				{ "normal", WriteVec(contact.normal) },
			});
		}
		root["contacts"] = std::move(contacts);

		// Write the body axis
		root["body_axis"] = json::Object{
			{ "from", desc.bones[desc.body_axis.from].name },
			{ "to", desc.bones[desc.body_axis.to].name },
		};
		return doc;
	}

	// Read and write '.rig' files
	inline RigDesc LoadRigDesc(std::filesystem::path const& filepath)
	{
		// Schema errors name the file so that cook errors are actionable
		try
		{
			return ReadRigDesc(json::Read(filepath));
		}
		catch (std::exception const& ex)
		{
			throw std::runtime_error(std::format("{}: {}", filepath.string(), ex.what()));
		}
	}
	inline void SaveRigDesc(std::filesystem::path const& filepath, RigDesc const& desc)
	{
		// Indented output keeps cooked rigs readable and diffable
		json::Write(filepath, WriteRigDesc(desc));
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	PRUnitTestClass(AnimationRigJsonTests)
	{
		PRUnitTestMethod(RoundTrip, Quick)
		{
			// Writing then reading a description reproduces it
			auto desc = TestRigDesc();
			desc.bones[13].limit = JointLimit{ .swing = { .min_x = -0.5f, .max_x = 0.25f, .min_z = 0.0f, .max_z = 0.0f } };
			desc.bones[14].limit = JointLimit{ .swing = { .min_x = -0.3f, .max_x = 0.3f, .min_z = -0.2f, .max_z = 0.4f }, .min_twist = -0.1f, .max_twist = 0.15f };
			desc.bones[14].stiffness = 0.4f;

			auto text = json::Write(WriteRigDesc(desc));
			auto read = ReadRigDesc(json::Read(std::string_view(text)));
			PR_EXPECT(Validate(read).empty());
			PR_EXPECT(read.bones.size() == desc.bones.size());
			for (size_t i = 0; i != desc.bones.size(); ++i)
			{
				auto const& l = read.bones[i];
				auto const& r = desc.bones[i];
				PR_EXPECT(l.name == r.name && l.parent == r.parent && l.length == r.length && l.stiffness == r.stiffness);
				PR_EXPECT(FEql(l.rest.pos, r.rest.pos) && FEql(l.rest.rot, r.rest.rot));
				PR_EXPECT(l.limit.swing.min_x == r.limit.swing.min_x && l.limit.swing.max_x == r.limit.swing.max_x);
				PR_EXPECT(l.limit.swing.min_z == r.limit.swing.min_z && l.limit.swing.max_z == r.limit.swing.max_z);
				PR_EXPECT(l.limit.min_twist == r.limit.min_twist && l.limit.max_twist == r.limit.max_twist);
			}
			PR_EXPECT(read.chains.size() == desc.chains.size());
			for (size_t i = 0; i != desc.chains.size(); ++i)
			{
				auto const& l = read.chains[i];
				auto const& r = desc.chains[i];
				PR_EXPECT(l.role == r.role && l.side == r.side && l.bones == r.bones && FEql(l.pole, r.pole) && FEql(l.forward, r.forward));
			}
			PR_EXPECT(read.contacts.size() == desc.contacts.size());
			for (size_t i = 0; i != desc.contacts.size(); ++i)
			{
				auto const& l = read.contacts[i];
				auto const& r = desc.contacts[i];
				PR_EXPECT(l.name == r.name && l.bone == r.bone && l.kind == r.kind && FEql(l.offset, r.offset) && FEql(l.normal, r.normal));
			}
			PR_EXPECT(read.body_axis.from == desc.body_axis.from && read.body_axis.to == desc.body_axis.to);
		}
		PRUnitTestMethod(SchemaErrors, Quick)
		{
			// Schema errors name the offending field
			auto desc = TestRigDesc();
			desc.bones[14].limit = JointLimit{ .min_twist = -0.1f, .max_twist = 0.1f };
			auto text = json::Write(WriteRigDesc(desc));
			auto expect_error = [&](std::string_view from, std::string_view to, std::string_view field)
			{
				// Replace the first occurrence of 'from' and check the reported field
				auto bad = text;
				auto ofs = bad.find(from);
				PR_EXPECT(ofs != std::string::npos);
				bad.replace(ofs, from.size(), to);
				try
				{
					(void)ReadRigDesc(json::Read(std::string_view(bad)));
					PR_EXPECT(false && "Expected a schema error");
				}
				catch (std::runtime_error const& ex)
				{
					PR_EXPECT(std::string_view(ex.what()).find(field) != std::string_view::npos);
				}
			};
			expect_error("\"schema_version\": 3", "\"schema_version\": 2", "schema_version");
			expect_error("\"parent\": \"root\"", "\"parent\": \"nobody\"", "rig.bones[1].parent");
			expect_error("\"role\": \"spine\"", "\"role\": \"wing\"", "rig.chains[0].role");
			expect_error("\"kind\": \"sole\"", "\"kind\": \"hoof\"", "rig.contacts[0].kind");
			expect_error("\"twist\"", "\"spin\"", "rig.bones[14].limit");
			expect_error("\"to\": \"neck\"", "\"to\": \"nobody\"", "rig.body_axis.to");
		}
	};
}
#endif
