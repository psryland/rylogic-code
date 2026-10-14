"""Build a '.rig' description from an armature's properties, and write it to a file.

The '.rig' schema is owned by 'pr/animation/rig_json.h' in the Rylogic code library. All values are in model space, which is the space that the skinned
model is exported in: Blender world space, with the armature's object transform applied. Each bone points along its local +Y axis, which is Blender's
bone convention and is preserved by the glTF export.
"""

import json
from pathlib import Path
from typing import Any

import bpy
from mathutils import Matrix, Vector

SCHEMA_VERSION = 3


class ModelSpace:
	"""Converts armature data to the rigid model space used by the '.rig' file.

	Blender bones can be scaled by the armature's object transform, but '.rig' rest transforms are rigid. So the object scale must be uniform, and it is
	applied to positions and lengths instead.
	"""

	def __init__(self, armature_obj: bpy.types.Object):
		"""Capture the armature's object transform. Raises ValueError if it has a non-uniform scale."""
		# Split the object transform into a rigid part and a uniform scale
		location, rotation, scale = armature_obj.matrix_world.decompose()
		if abs(scale.x - scale.y) > 1e-6 * abs(scale.x) or abs(scale.x - scale.z) > 1e-6 * abs(scale.x):
			raise ValueError(f"Armature '{armature_obj.name}' has a non-uniform scale {tuple(scale)}")

		self.scale = scale.x
		self.rotation = rotation
		self.location = location

	def BoneRest(self, bone: bpy.types.Bone) -> Matrix:
		"""Return the rigid model-space rest transform of 'bone'."""
		# The bone's armature-space rest matrix has unit scale, so only the object transform's scale needs separating
		location, rotation, _ = bone.matrix_local.decompose()
		return Matrix.LocRotScale(self.location + self.rotation @ (location * self.scale), self.rotation @ rotation, None)

	def BonePose(self, pose_bone: bpy.types.PoseBone) -> Matrix:
		"""Return the rigid model-space transform of 'pose_bone' in its current pose. Any pose scale is dropped."""
		# Pose matrices are in armature space, like rest matrices, so the same conversion applies
		location, rotation, _ = pose_bone.matrix.decompose()
		return Matrix.LocRotScale(self.location + self.rotation @ (location * self.scale), self.rotation @ rotation, None)

	def Direction(self, direction: Vector) -> Vector:
		"""Return an armature-space direction in model space, normalised."""
		return (self.rotation @ Vector(direction)).normalized()


def BonesParentFirst(armature: bpy.types.Armature) -> list[bpy.types.Bone]:
	"""Return the armature's bones in depth-first order, so that each parent precedes its children."""
	# Walk from each root bone, visiting children in Blender's order so that the result is deterministic
	ordered: list[bpy.types.Bone] = []
	stack = [bone for bone in reversed(armature.bones) if bone.parent is None]
	while stack:
		bone = stack.pop()
		ordered.append(bone)
		stack.extend(reversed(bone.children))

	return ordered


def BuildRigDesc(armature_obj: bpy.types.Object) -> dict[str, Any]:
	"""Return the '.rig' description of an armature object as a JSON-compatible dictionary.

	The description is not validated; see 'validate.ValidateRigDesc'.
	"""
	# Describe each bone by its rest transform relative to its parent, with its joint properties
	space = ModelSpace(armature_obj)
	armature = armature_obj.data
	bones: list[dict[str, Any]] = []
	for bone in BonesParentFirst(armature):
		rest = space.BoneRest(bone)
		local = rest if bone.parent is None else space.BoneRest(bone.parent).inverted() @ rest
		position = local.to_translation()
		rotation = local.to_quaternion().normalized()
		desc: dict[str, Any] = {
			"name": bone.name,
			"parent": bone.parent.name if bone.parent else None,
			"position": _List(position),
			"rotation": [rotation.x, rotation.y, rotation.z, rotation.w],
			"length": bone.length * space.scale,
		}

		# Add the optional joint limit. Its ranges are measured about the bone's own rest axes, which the export keeps unchanged.
		joint = bone.rylogic_rig
		if joint.use_limit:
			desc["limit"] = {
				"swing_x": [joint.swing_x_min, joint.swing_x_max],
				"swing_z": [joint.swing_z_min, joint.swing_z_max],
				"twist": [joint.twist_min, joint.twist_max],
			}

		if joint.stiffness != 0:
			desc["stiffness"] = joint.stiffness

		bones.append(desc)

	# Describe each chain. Directions are authored in armature space.
	chains: list[dict[str, Any]] = []
	for chain in armature.rylogic_rig.chains:
		desc = {"role": chain.role.lower(), "side": chain.side.lower(), "bones": [ref.name for ref in chain.bones]}
		match chain.role:
			case "LEG" | "ARM":
				desc["pole"] = _List(space.Direction(chain.pole))
			case "NECK":
				desc["forward"] = _List(space.Direction(chain.forward))
			case "SPINE" | "TAIL" | "DIGIT":
				pass
			case _:
				raise ValueError(f"A chain has an unknown role '{chain.role}'")

		chains.append(desc)

	# Describe each contact. Offsets and normals are in bone space, which is rigid apart from the uniform scale.
	contacts: list[dict[str, Any]] = []
	for contact in armature.rylogic_rig.contacts:
		contacts.append({
			"name": contact.name,
			"bone": contact.bone,
			"kind": contact.kind.lower(),
			"offset": _List(Vector(contact.offset) * space.scale),
			"normal": _List(Vector(contact.normal).normalized()),
		})

	# The body axis is the line between two bone heads that the collision shape follows
	rig = armature.rylogic_rig
	body_axis = {"from": rig.body_axis_from, "to": rig.body_axis_to}
	return {"schema_version": SCHEMA_VERSION, "bones": bones, "chains": chains, "contacts": contacts, "body_axis": body_axis}


def WriteRigDesc(desc: dict[str, Any], path: Path) -> None:
	"""Write a '.rig' description to 'path' as deterministic JSON."""
	path = Path(path)
	path.parent.mkdir(parents=True, exist_ok=True)
	path.write_text(json.dumps(desc, indent=1), encoding="utf-8")


def _List(vector: Vector) -> list[float]:
	"""Return a vector as a list of floats."""
	return [float(value) for value in vector]
