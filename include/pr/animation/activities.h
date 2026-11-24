//*****************************************************************************
// Procedural animation - the activity type registry
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// The list of activity types (see activity.h). The registry reads each type's parameters from a '.gait' file, and describes every type's
// parameters so that tools, such as the Blender add-on, can build editors for them without knowing the types.
//
// The description ('ActivityTypesJson') is an object with one member per type:
//  { "<type>": {
//      "root_mode": "planted" | "travel" | "free" | "mounted",
//      "params": [ <param>... ]
//  }}
// where each <param> is one of:
//  { "name": "...", "kind": "float", "min": <number>, "max": <number>, "default": <number> }
//  { "name": "...", "kind": "int", "min": <number>, "max": <number>, "default": <number> }
//  { "name": "...", "kind": "bool", "default": true | false }
//  { "name": "...", "kind": "enum", "items": ["...", ...], "default": "..." }
//  { "name": "...", "kind": "list", "min_items": <number>, "fields": [ <param>... ] }   // A list of objects, each with the given fields
// Each parameter appears in the activity's "params" object under its name.
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/activity.h"
#include "pr/animation/activity_punches.h"

namespace pr::animation
{
	// One entry in the registry of activity types
	struct ActivityType
	{
		std::string_view name;  // The type's name in '.gait' files

		// Describe the type's parameters for editors (see the top of this file)
		json::Object (*describe)();

		// Read an activity of this type from its parameters. Throws 'std::runtime_error' if they do not follow the type's schema. 'context' names
		// the parameters' place in the file for error messages.
		std::shared_ptr<IActivity const> (*read)(json::Object const& params, std::string const& context);
	};

	// Every activity type
	inline constexpr std::array<ActivityType, 1> ActivityTypes = {
		ActivityType{ .name = "punches", .describe = &Punches::Describe, .read = &Punches::Read },
	};

	// Read an activity of the type 'type' from its parameters. Throws 'std::runtime_error' if the type is not in the registry or the parameters
	// do not follow its schema.
	inline std::shared_ptr<IActivity const> ReadActivity(std::string_view type, json::Object const& params, std::string const& context)
	{
		// Find the type by name
		for (auto const& entry : ActivityTypes)
		{
			if (entry.name == type)
				return entry.read(params, context);
		}
		throw std::runtime_error(std::format("{}: unknown activity type '{}'", context, type));
	}

	// The description of every activity type's parameters, as JSON text (see the top of this file)
	inline std::string ActivityTypesJson()
	{
		// One member per type, in registry order
		json::Object types;
		for (auto const& entry : ActivityTypes)
			types[std::string(entry.name)] = entry.describe();

		return json::Write(json::Value(std::move(types)));
	}
}
