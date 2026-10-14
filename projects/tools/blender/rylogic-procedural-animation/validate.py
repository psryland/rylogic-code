"""Check a '.rig' description for the rules that the runtime relies on.

This mirrors 'Validate' in 'pr/animation/rig.h' so that problems are reported instantly while authoring. The native check, run by the p3d tool's '-Rig'
command, is canonical: it also checks the rig against the skinned model's skeleton.
"""

import math
from typing import Any

from mathutils import Matrix, Quaternion, Vector

LIMIT_RANGES = ("swing_x", "swing_z", "twist")
CHAIN_ROLES = ("spine", "neck", "leg", "arm", "tail", "digit")
SIDES = ("centre", "left", "right")
CONTACT_KINDS = ("sole", "toe", "palm", "finger", "tail")


def ValidateRigDesc(desc: dict[str, Any]) -> list[str]:
	"""Return a description of each problem in a '.rig' description, or an empty list if it is valid."""
	# The file-level structure must match the schema before its contents can be checked
	errors: list[str] = []
	if desc.get("schema_version") != 3:
		errors.append(f"Unsupported schema_version {desc.get('schema_version')!r}")

	bones = desc.get("bones", [])
	chains = desc.get("chains", [])
	contacts = desc.get("contacts", [])
	if not bones:
		errors.append("The rig has no bones")

	# Check each bone on its own
	index_of: dict[str, int] = {}
	for i, bone in enumerate(bones):
		# Names must be present and unique so that the rig can be matched to its skin
		name = bone.get("name", "")
		if not name:
			errors.append(f"Bone {i} has no name")
		elif name in index_of:
			errors.append(f"Bone '{name}' is not uniquely named")

		# Parents must come first so that transforms can be composed in one pass
		parent = bone.get("parent")
		if parent is not None and parent not in index_of:
			errors.append(f"Bone '{name}' has parent '{parent}', which is not an earlier bone")

		index_of.setdefault(name, i)

		# The rest transform must be rigid, and the joint parameters must be in range
		if not math.isclose(Vector(bone["rotation"]).length, 1.0, abs_tol=1e-4):
			errors.append(f"Bone '{name}' has a non-unit rest rotation")
		if not bone.get("length", 0) > 0:
			errors.append(f"Bone '{name}' has no length")
		if not 0 <= bone.get("stiffness", 0) <= 1:
			errors.append(f"Bone '{name}' has stiffness outside [0,1]")

		errors.extend(_ValidateLimit(name, bone.get("limit")))

	# Chain checks need a consistent hierarchy
	if errors:
		return errors

	rest_model = _RestModelTransforms(bones)
	owner: dict[str, int] = {}
	for c, chain in enumerate(chains):
		errors.extend(_ValidateChain(c, chain, bones, index_of, rest_model, owner))

	# Contacts must name a bone and give a surface direction so that they can be placed on terrain
	contact_names: set[str] = set()
	for i, contact in enumerate(contacts):
		name = contact.get("name", "")
		if not name:
			errors.append(f"Contact {i} has no name")
		elif name in contact_names:
			errors.append(f"Contact '{name}' is not uniquely named")

		contact_names.add(name)
		if contact.get("bone") not in index_of:
			errors.append(f"Contact '{name}' refers to bone '{contact.get('bone')}', which does not exist")
		if not math.isclose(Vector(contact["normal"]).length, 1.0, abs_tol=1e-4):
			errors.append(f"Contact '{name}' needs a unit 'normal' direction")
		if contact.get("kind") not in CONTACT_KINDS:
			errors.append(f"Contact '{name}' has an unknown kind '{contact.get('kind')}'")

	# The body axis needs two bones with separate heads to define a direction
	body_axis = desc.get("body_axis", {})
	ends = [body_axis.get("from", ""), body_axis.get("to", "")]
	for key, name in zip(("from", "to"), ends):
		if name not in index_of:
			errors.append(f"The body axis '{key}' bone '{name}' does not exist. Set it in the rig's Body Axis panel")
	if all(name in index_of for name in ends):
		heads = [rest_model[index_of[name]].to_translation() for name in ends]
		if (heads[1] - heads[0]).length <= 1e-6:
			errors.append(f"The body axis bones '{ends[0]}' and '{ends[1]}' have the same head")

	return errors


def _ValidateLimit(name: str, limit: dict[str, Any] | None) -> list[str]:
	"""Check that each of a bone's limit ranges lies in [-pi, pi] and contains the rest pose."""
	# A missing limit means the joint is free
	if limit is None:
		return []

	errors: list[str] = []
	for key in LIMIT_RANGES:
		bounds = limit.get(key)
		if not isinstance(bounds, list) or len(bounds) != 2:
			errors.append(f"Bone '{name}' has a limit without a '{key}' [min, max] range")
		elif not -math.pi - 1e-6 <= bounds[0] <= 0 <= bounds[1] <= math.pi + 1e-6:
			errors.append(f"Bone '{name}' has a '{key}' range that is outside [-pi, pi] or does not contain 0")

	return errors


def _ValidateChain(c: int, chain: dict[str, Any], bones: list[dict[str, Any]], index_of: dict[str, int], rest_model: list[Matrix], owner: dict[str, int]) -> list[str]:
	"""Check one chain. 'owner' maps each bone name to the first chain that contains it."""
	# Each chain is a run of parent-child bones, and each bone belongs to at most one chain
	errors: list[str] = []
	names = chain.get("bones", [])
	if not names:
		return [f"Chain {c} has no bones"]

	missing = [name for name in names if name not in index_of]
	for name in missing:
		errors.append(f"Chain {c} refers to bone '{name}', which does not exist")
	for k, name in enumerate(names):
		if name in owner:
			errors.append(f"Bone '{name}' is in chains {owner[name]} and {c}")
		if k != 0 and name in index_of and bones[index_of[name]].get("parent") != names[k - 1]:
			errors.append(f"Chain {c}: bone '{name}' is not a child of the previous bone in the chain")

		owner.setdefault(name, c)

	if missing:
		return errors

	# Check the role-specific requirements
	heads = [rest_model[index_of[name]].to_translation() for name in names]
	role = chain.get("role")
	match role:
		case "spine":
			# The curve fit needs at least the pelvis and chest, with separated joints
			if len(names) < 2:
				errors.append(f"Spine chain {c} needs at least two bones")
			for k in range(1, len(names)):
				if (heads[k] - heads[k - 1]).length <= 1e-6:
					errors.append(f"Spine chain {c}: bone '{names[k]}' has the same head as its parent")
		case "neck":
			# The aim needs a facing direction
			if not math.isclose(Vector(chain.get("forward", (0, 0, 0))).length, 1.0, abs_tol=1e-4):
				errors.append(f"Neck chain {c} needs a unit 'forward' direction")
		case "leg" | "arm":
			# Two-bone IK needs exactly three bones and a pole that defines a bend plane
			if len(names) != 3:
				errors.append(f"Limb chain {c} needs exactly three bones (upper, lower, end)")
				return errors

			pole = Vector(chain.get("pole", (0, 0, 0)))
			if (heads[1] - heads[0]).length <= 1e-6 or (heads[2] - heads[1]).length <= 1e-6:
				errors.append(f"Limb chain {c} has a zero-length upper or lower bone")
			if not math.isclose(pole.length, 1.0, abs_tol=1e-4):
				errors.append(f"Limb chain {c} needs a unit 'pole' direction")
			elif (heads[2] - heads[0]).normalized().cross(pole).length < 0.1:
				errors.append(f"Limb chain {c} has a pole that is (nearly) parallel to the limb")
		case "tail":
			# The tail joints must form a valid FABRIK chain. Each link runs from one joint to the next, ending at the last bone's tip.
			last = rest_model[index_of[names[-1]]]
			joints = heads + [last @ Vector((0, bones[index_of[names[-1]]]["length"], 0))]
			for k, name in enumerate(names):
				if (joints[k + 1] - joints[k]).length <= 1e-6:
					errors.append(f"Tail chain {c}: bone '{name}' has a zero-length link")
		case "digit":
			pass
		case _:
			errors.append(f"Chain {c} has an unknown role '{role}'")

	if chain.get("side") not in SIDES:
		errors.append(f"Chain {c} has an unknown side '{chain.get('side')}'")

	return errors


def _RestModelTransforms(bones: list[dict[str, Any]]) -> list[Matrix]:
	"""Return the model-space rest transform of each bone. Parents must precede their children."""
	# Compose each bone's parent-relative rest transform with its parent's model-space transform
	index_of: dict[str, int] = {}
	model: list[Matrix] = []
	for i, bone in enumerate(bones):
		x, y, z, w = bone["rotation"]
		local = Matrix.LocRotScale(Vector(bone["position"]), Quaternion((w, x, y, z)).normalized(), None)
		parent = bone.get("parent")
		model.append(local if parent is None else model[index_of[parent]] @ local)
		index_of[bone["name"]] = i

	return model
