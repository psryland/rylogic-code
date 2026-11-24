"""The activity types that a '.gait' can list, and the editors for their parameters.

The runtime owns the activity types (see 'pr/animation/activities.h'). Each type publishes a description of its parameters: their names, kinds,
ranges, defaults and enum items. 'activity_types.json' is a copy of that description, which a native unit test keeps up to date. The add-on builds
each type's parameter editor from the description, so a new type or parameter needs no add-on code.

This module does not use Blender, so it can be tested outside it. 'props.py' turns the fields returned by 'EditorFields' into Blender properties.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# The description of every activity type, read once when the module loads
DESCRIPTION_PATH = Path(__file__).with_name("activity_types.json")


@dataclass
class EditorField:
	"""One editable parameter of an activity type.

	'kind' is the parameter's kind in the description: "float", "int", "bool", "enum" or "list". 'options' are the keyword arguments for the
	Blender property that edits it, e.g. its label, range and default. A "list" field is edited as a list of items, each with the 'fields' editors.
	"""

	name: str
	kind: str
	options: dict[str, Any]
	fields: list["EditorField"] = field(default_factory=list)
	min_items: int = 0


def LoadDescription(path: Path = DESCRIPTION_PATH) -> dict[str, Any]:
	"""Return the activity type description: one member per type, each with its 'root_mode' and 'params'."""
	with open(path, "r", encoding="utf-8") as file:
		return json.load(file)


def TypeItems(desc: dict[str, Any]) -> list[tuple[str, str, str]]:
	"""Return the Blender enum items for choosing an activity type. The identifiers are the type names used in the '.gait'."""
	return [(name, _Label(name), f"A '{name}' activity, which moves the body with root mode '{info['root_mode']}'") for name, info in desc.items()]


def EditorFields(desc: dict[str, Any], type_name: str) -> list[EditorField]:
	"""Return the editors for the parameters of the activity type 'type_name', in the order the description lists them."""
	return [_EditorField(param) for param in desc[type_name]["params"]]


def ReadParams(fields: list[EditorField], values: Any) -> dict[str, Any]:
	"""Return the '.gait' "params" object for the parameter values in 'values'.

	'values' has an attribute per field, e.g. the Blender property group built from the same fields. A list field's attribute is a sequence of
	objects with an attribute per item field. Enum values are written as they are stored, so their identifiers must be the description's names.
	"""
	# Lists hold objects with the item fields; other kinds are written as they are
	params: dict[str, Any] = {}
	for f in fields:
		value = getattr(values, f.name)
		params[f.name] = [ReadParams(f.fields, item) for item in value] if f.kind == "list" else value

	return params


def ValidateParams(fields: list[EditorField], params: dict[str, Any], ctx: str) -> list[str]:
	"""Check '.gait' parameters against their editors' ranges. Returns a description of each problem, or an empty list if they are valid."""
	# The runtime checks the same rules, and more, when it reads the '.gait'
	errors: list[str] = []
	for f in fields:
		value = params[f.name]
		match f.kind:
			case "list":
				if len(value) < f.min_items:
					errors.append(f"{ctx} needs at least {f.min_items} '{f.name}' item(s)")
				for i, item in enumerate(value):
					errors.extend(ValidateParams(f.fields, item, f"{ctx} {f.name} {i + 1}"))
			case "float" | "int":
				if not f.options["min"] <= value <= f.options["max"]:
					errors.append(f"{ctx} needs '{f.name}' in [{f.options['min']:g}, {f.options['max']:g}], not {value:g}")
			case "enum":
				if value not in (item[0] for item in f.options["items"]):
					errors.append(f"{ctx} has an unknown '{f.name}' value '{value}'")
			case "bool":
				pass
			case _:
				raise ValueError(f"Unknown parameter kind '{f.kind}'")

	return errors


def _EditorField(param: dict[str, Any]) -> EditorField:
	"""Return the editor for one parameter of the description."""
	# The property options mirror the description; Blender stores floats in single precision, so ranges and defaults are rounded to it
	name, kind = param["name"], param["kind"]
	options: dict[str, Any] = {"name": _Label(name)}
	match kind:
		case "float":
			options.update(min=_Single(param["min"]), max=_Single(param["max"]), default=_Single(param["default"]))
		case "int":
			options.update(min=int(param["min"]), max=int(param["max"]), default=int(param["default"]))
		case "bool":
			options.update(default=bool(param["default"]))
		case "enum":
			options.update(items=[(item, _Label(item), "") for item in param["items"]], default=param["default"])
		case "list":
			# A list's items are edited with their own fields. Lists of lists have no editor.
			fields = [_EditorField(f) for f in param["fields"]]
			if any(f.kind == "list" for f in fields):
				raise ValueError(f"Parameter '{name}' is a list of lists, which has no editor")
			return EditorField(name=name, kind=kind, options=options, fields=fields, min_items=int(param["min_items"]))
		case _:
			raise ValueError(f"Parameter '{name}' has an unknown kind '{kind}'")

	return EditorField(name=name, kind=kind, options=options)


def _Label(name: str) -> str:
	"""Return a display label for a description name, e.g. "Body hook" for "body_hook"."""
	return name.replace("_", " ").capitalize()


def _Single(value: float) -> float:
	"""Return 'value' rounded to the 7 significant digits that a single precision float holds, e.g. 0.6 for 0.6000000238418579."""
	return float(f"{value:.7g}")
