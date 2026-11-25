"""Tests for 'activity_types.py', which builds the activity parameter editors from the runtime's type description. Run with 'python -m pytest'."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

# The add-on's folder name is not a valid package name, and its package imports Blender, so the module is loaded from its file
_PATH = Path(__file__).resolve().parents[1] / "rylogic-procedural-animation" / "activity_types.py"
_SPEC = importlib.util.spec_from_file_location("activity_types", _PATH)
activity_types = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(activity_types)

# A description with every kind of parameter, so that the editors are not tied to the shipped types
_DESC = {
	"dance": {
		"root_mode": "free",
		"params": [
			{"name": "tempo", "kind": "float", "min": 0.5, "max": 2.0, "default": 1.0000000149011612},
			{"name": "repeats", "kind": "int", "min": 1, "max": 8, "default": 2},
			{"name": "mirror", "kind": "bool", "default": False},
			{"name": "style", "kind": "enum", "items": ["waltz", "body_pop"], "default": "waltz"},
			{"name": "steps", "kind": "list", "min_items": 1, "fields": [
				{"name": "foot", "kind": "enum", "items": ["left", "right"], "default": "right"},
				{"name": "length", "kind": "float", "min": 0.1, "max": 1.0, "default": 0.3},
			]},
		],
	},
}


def test_editor_is_built_from_the_description():
	"""Each parameter's editor has the name, kind, range, default and items that the description gives it, in the description's order."""
	fields = activity_types.EditorFields(_DESC, "dance")
	assert [f.name for f in fields] == ["tempo", "repeats", "mirror", "style", "steps"]
	assert [f.kind for f in fields] == ["float", "int", "bool", "enum", "list"]
	assert fields[0].options == {"name": "Tempo", "min": 0.5, "max": 2.0, "default": 1.0}
	assert fields[1].options == {"name": "Repeats", "min": 1, "max": 8, "default": 2}
	assert fields[2].options == {"name": "Mirror", "default": False}
	assert fields[3].options == {"name": "Style", "items": [("waltz", "Waltz", ""), ("body_pop", "Body pop", "")], "default": "waltz"}

	# A list's items have their own editors
	steps = fields[4]
	assert steps.min_items == 1
	assert [(f.name, f.kind) for f in steps.fields] == [("foot", "enum"), ("length", "float")]
	assert steps.fields[0].options["default"] == "right"
	assert steps.fields[1].options["max"] == 1.0


def test_params_are_read_and_validated_through_the_editors():
	"""Values stored with the editors' names become the '.gait' params object, and out-of-range values are reported."""
	fields = activity_types.EditorFields(_DESC, "dance")
	values = SimpleNamespace(tempo=1.5, repeats=3, mirror=True, style="body_pop", steps=[SimpleNamespace(foot="left", length=0.4)])
	params = activity_types.ReadParams(fields, values)
	assert params == {"tempo": 1.5, "repeats": 3, "mirror": True, "style": "body_pop", "steps": [{"foot": "left", "length": 0.4}]}
	assert activity_types.ValidateParams(fields, params, "Activity 'a'") == []

	# An empty list and an out-of-range float are both reported
	params["steps"] = []
	params["tempo"] = 3.0
	assert len(activity_types.ValidateParams(fields, params, "Activity 'a'")) == 2


def test_shipped_description_has_punches():
	"""The shipped description, kept up to date by the native 'ActivityTypesMatchAddon' test, gives punches a list of punches."""
	desc = activity_types.LoadDescription()
	assert activity_types.TypeItems(desc)[0][:2] == ("punches", "Punches")
	punches = activity_types.EditorFields(desc, "punches")[0]
	assert punches.kind == "list" and punches.min_items == 1
	assert [f.name for f in punches.fields] == ["hand", "kind", "duration"]
	assert punches.fields[2].options["default"] == 0.6

def test_shipped_description_has_strike():
	"""The shipped description gives a strike its hands and a flat set of numbers for the blow's shape and timing, ready to edit with their defaults."""
	desc = activity_types.LoadDescription()
	assert [item[0] for item in activity_types.TypeItems(desc)] == ["punches", "strike"]
	fields = activity_types.EditorFields(desc, "strike")
	assert fields[0].name == "hands" and fields[0].options["default"] == "right"
	assert [item[0] for item in fields[0].options["items"]] == ["left", "right", "both"]
	assert all(f.kind == "float" for f in fields[1:])
	contact = next(f for f in fields if f.name == "contact")
	assert contact.options["default"] == 0.6
	values = SimpleNamespace(**{f.name: f.options["default"] for f in fields})
	assert activity_types.ValidateParams(fields, activity_types.ReadParams(fields, values), "Activity 'chop'") == []
