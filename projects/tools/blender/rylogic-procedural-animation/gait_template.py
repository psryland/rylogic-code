"""Generate starting-point movement cycles for an armature, to be refined by hand.

A template moves the root bone forward along the model's forward axis (-Y) by one stride per cycle, and steps each walking limb (see
'gait_desc.WalkingLimbs') in a standard footfall pattern: a planted foot stays still on the ground, and a swinging foot moves to its next landing in
a smooth arc. Templates that walk on arms pitch the spine forward until it is level, so that the arms can reach the ground. The body is lowered until
every walking limb reaches the ground under it with its joints bent, and the stride is shortened if a limb cannot reach the ends of its stance. The
rest of the body keeps its rest pose. Limb lengths set the stride and swing height, so the same template suits any size of rig.
"""

import math
from dataclasses import dataclass

import bpy
from mathutils import Matrix, Vector

from .gait_desc import WalkingLimbs, _Limb
from .rig_desc import BonesParentFirst

# The fraction of a limb's reach at which it stands, and the most that it stretches to at the ends of its stance
STANCE_REACH = 0.85
MAX_REACH = 0.95


@dataclass(frozen=True)
class _Template:
	"""A standard movement cycle."""

	family: str
	name: str
	frames: int                                     # Frames per cycle
	stride: float                                   # The distance moved per cycle, as a multiple of the leg length, if the limbs can reach it
	swing_height: float                             # The foot lift, as a multiple of the leg length
	hip_height: float                               # The cycle's hip height (see 'RYLOGIC_PG_GaitCycle.hip_height')
	duty: float                                     # The fraction of the cycle that each foot is planted
	touchdowns: dict[tuple[str, str], float]        # The touchdown phase of each walking limb, by (role, side)


TEMPLATES = {
	"BIPEDAL_WALK": _Template("bipedal", "walk", 24, 1.0, 0.15, 0.9, 0.6, {("LEG", "LEFT"): 0.0, ("LEG", "RIGHT"): 0.5}),
	"QUADRUPED_WALK": _Template("quadruped", "walk", 24, 1.0, 0.15, 0.5, 0.75, {("LEG", "LEFT"): 0.0, ("ARM", "LEFT"): 0.25, ("LEG", "RIGHT"): 0.5, ("ARM", "RIGHT"): 0.75}),
	"QUADRUPED_TROT": _Template("quadruped", "trot", 16, 1.4, 0.2, 0.5, 0.5, {("LEG", "LEFT"): 0.0, ("ARM", "RIGHT"): 0.0, ("LEG", "RIGHT"): 0.5, ("ARM", "LEFT"): 0.5}),
}
TEMPLATE_ITEMS = [
	("BIPEDAL_WALK", "Bipedal Walk", "Legs only, alternating"),
	("QUADRUPED_WALK", "Quadruped Walk", "Four-beat walk: left hind, left fore, right hind, right fore"),
	("QUADRUPED_TROT", "Quadruped Trot", "Diagonal pairs step together"),
]


def RotationPath(pose_bone: bpy.types.PoseBone) -> str:
	"""Return the data path of the pose bone's rotation channel for its rotation mode."""
	match pose_bone.rotation_mode:
		case "QUATERNION":
			return "rotation_quaternion"
		case "AXIS_ANGLE":
			return "rotation_axis_angle"
		case "XYZ" | "XZY" | "YXZ" | "YZX" | "ZXY" | "ZYX":
			return "rotation_euler"
		case _:
			raise ValueError(f"Bone '{pose_bone.name}' has an unknown rotation mode '{pose_bone.rotation_mode}'")


def GenerateTemplate(armature_obj: bpy.types.Object, context: bpy.types.Context, template_id: str) -> bpy.types.Action:
	"""Key a new action with the template's movement cycle, add it to the armature's movement cycles, and return it.

	Raises ValueError if the armature lacks a walking limb that the template steps. The armature's current action and frame are restored.
	"""
	# Find the limbs that the template steps, in the template's order
	template = TEMPLATES[template_id]
	limbs = {(limb.role.upper(), limb.side.upper()): limb for limb in WalkingLimbs(armature_obj.data)}
	missing = [f"{role.lower()} {side.lower()}" for role, side in template.touchdowns if (role, side) not in limbs]
	if missing:
		raise ValueError(f"The template needs walking limbs (a chain with a sole or palm contact on its end bone) for: {', '.join(missing)}")

	# Template sizes are in armature space, relative to the length of the first leg
	armature = armature_obj.data
	legs = [limb for (role, _), limb in limbs.items() if role == "LEG"]
	leg_length = sum(armature.bones[name].length for name in legs[0].bones)
	to_armature = armature_obj.matrix_world.to_3x3().inverted()
	forward = (to_armature @ Vector((0, -1, 0))).normalized()
	up = (to_armature @ Vector((0, 0, 1))).normalized()

	# Pose the body for the template's limbs, then take the longest stride that every limb can reach
	steps = [(limbs[key], touchdown) for key, touchdown in template.touchdowns.items()]
	body = _BodyPose(armature, [limb for limb, _ in steps], forward, up)
	stride = min([template.stride * leg_length] + [_MaxStride(body, limb, forward, template.duty) for limb, _ in steps])

	# Pose every frame with temporary IK, then key the poses once the constraints are gone
	scene = context.scene
	animation_data = armature_obj.animation_data or armature_obj.animation_data_create()
	saved_action, saved_frame, saved_pose = animation_data.action, scene.frame_current, _SavePose(armature_obj)
	try:
		poses = _SolvePoses(armature_obj, context, template, steps, body, forward, up, stride, template.swing_height * leg_length)
		action = bpy.data.actions.new(f"{armature_obj.name}.{template.family}.{template.name}")
		animation_data.action = action
		_KeyPoses(armature_obj, context, poses)
		action.use_frame_range = True
		action.frame_start, action.frame_end = 1, 1 + template.frames
	finally:
		# Leave the authoring state as it was
		animation_data.action = saved_action
		_RestorePose(armature_obj, saved_pose)
		scene.frame_set(saved_frame)

	# Register the cycle
	cycle = armature.rylogic_rig.cycles.add()
	cycle.name, cycle.family, cycle.action, cycle.hip_height = template.name, template.family, action, template.hip_height
	armature.rylogic_rig.active_cycle = len(armature.rylogic_rig.cycles) - 1
	return action


@dataclass
class _Body:
	"""The pose of the body at the start of a cycle, in armature space."""

	bones: dict[str, Matrix]       # The posed body bones, parents first; the bones below them follow at rest
	neutral: dict[str, Matrix]     # Each walking limb's end bone, standing on the ground at the middle of its stance
	base: dict[str, Vector]        # Each walking limb's chain base: the head of its first bone
	reach: dict[str, float]        # Each walking limb's reach: the length of the chain above its end bone


def _BodyPose(armature: bpy.types.Armature, limbs: list[_Limb], forward: Vector, up: Vector) -> _Body:
	"""Pose the body so that each limb stands on the ground under its chain base, as described in the module comment.

	Raises ValueError if the template walks on arms and the rig has no spine chain, or if a limb cannot reach the ground under it.
	"""
	# The ground is at the legs' rest contact height
	rest = {bone.name: bone.matrix_local.copy() for bone in armature.bones}
	legs = [limb for limb in limbs if limb.role == "leg"]
	ground = sum((rest[limb.bones[-1]] @ Vector(limb.contact.offset)).dot(up) for limb in legs) / len(legs)

	# Pitch the spine forward about its base until the arms' chain bases are level with it. Arms are below the spine, which keeps the legs at rest.
	pitch, pitched = Matrix.Identity(4), None
	arms = [limb for limb in limbs if limb.role == "arm"]
	if arms:
		spine = next((chain for chain in armature.rylogic_rig.chains if chain.role == "SPINE" and chain.bones), None)
		if spine is None:
			raise ValueError("A template that walks on arms needs a spine chain to pitch the body forward")

		pitched = armature.bones[spine.bones[0].name]
		pivot = pitched.head_local.copy()
		shoulders = sum((armature.bones[limb.bones[0]].head_local for limb in arms), Vector()) / len(arms)
		axis = up.cross(forward).normalized()

		# The shoulders' height falls as the pitch increases towards a quarter turn, so bisect for the level pitch
		low, high = 0.0, math.pi / 2
		for _ in range(40):
			angle = (low + high) / 2
			height = (Matrix.Rotation(angle, 3, axis) @ (shoulders - pivot)).dot(up)
			low, high = (angle, high) if height > 0 else (low, angle)

		pitch = Matrix.Translation(pivot) @ Matrix.Rotation(low, 4, axis) @ Matrix.Translation(-pivot)

	def Posed(name: str) -> Matrix:
		"""The armature-space matrix of a bone in the pitched body, before it is lowered."""
		bone = armature.bones[name]
		return pitch @ rest[name] if pitched is not None and (bone == pitched or pitched in bone.parent_recursive) else rest[name]

	# Stand each limb's contact on the ground under its chain base, beside it as at rest, with the contact's normal pointing down
	lateral = forward.cross(up).normalized()
	neutral, base, reach = {}, {}, {}
	drop = 0.0
	for limb in limbs:
		end = limb.bones[-1]
		rotation = Posed(end).to_3x3()
		normal = (rotation @ Vector(limb.contact.normal)).normalized()
		rotation = normal.rotation_difference(-up).to_matrix() @ rotation
		head = Posed(limb.bones[0]).to_translation()
		contact = forward * head.dot(forward) + lateral * (rest[end] @ Vector(limb.contact.offset)).dot(lateral) + up * ground
		neutral[end] = Matrix.Translation(contact - rotation @ Vector(limb.contact.offset)) @ rotation.to_4x4()
		base[end] = head
		reach[end] = sum(armature.bones[name].length for name in limb.bones[:-1])

		# Lower the body until the limb stands at 'STANCE_REACH' of its reach
		offset = head - neutral[end].to_translation()
		horizontal = (offset - up * offset.dot(up)).length
		if horizontal > STANCE_REACH * reach[end]:
			raise ValueError(f"The {limb.side} {limb.role} cannot reach the ground under it")

		drop = max(drop, offset.dot(up) - math.sqrt((STANCE_REACH * reach[end]) ** 2 - horizontal ** 2))

	# Lower the root, which carries the whole body
	lower = Matrix.Translation(-up * drop)
	root = BonesParentFirst(armature)[0].name
	bones = {root: lower @ rest[root]}
	if pitched is not None:
		bones[pitched.name] = lower @ pitch @ rest[pitched.name]

	return _Body(bones=bones, neutral=neutral, base={end: head - up * drop for end, head in base.items()}, reach=reach)


def _MaxStride(body: _Body, limb: _Limb, forward: Vector, duty: float) -> float:
	"""The longest stride for which the limb reaches both ends of its stance within 'MAX_REACH' of its reach."""
	# A planted foot moves from 'stride * duty / 2' ahead of its neutral point to the same distance behind it, relative to the body.
	# Solve |v + forward * t|^2 = R^2 for the largest |t| that both ends allow.
	end = limb.bones[-1]
	v = body.neutral[end].to_translation() - body.base[end]
	along = abs(v.dot(forward))
	radius = MAX_REACH * body.reach[end]
	return 2 * (math.sqrt(along * along - v.length_squared + radius * radius) - along) / duty


def _SolvePoses(armature_obj, context, template: _Template, steps, body: _Body, forward: Vector, up: Vector, stride: float, swing_height: float) -> list[dict[str, Matrix]]:
	"""Return the armature-space matrices of the body and walking-limb bones at each frame, including the last, which repeats the first one stride on."""
	# One IK target per limb, held by the bone above the end bone so the whole chain above the end bone bends to reach it
	scene = context.scene
	targets, constraints = [], []
	try:
		for limb, _ in steps:
			target = bpy.data.objects.new(f"_rylogic_ik_{limb.bones[-1]}", None)
			scene.collection.objects.link(target)
			constraint = armature_obj.pose.bones[limb.bones[-2]].constraints.new("IK")
			constraint.target, constraint.chain_count = target, len(limb.bones) - 1
			targets.append(target)
			constraints.append(constraint)

		# Solve each frame from the body pose, carried forward steadily. Pose bone matrices are set relative to their parent's current pose.
		poses = []
		for i in range(template.frames + 1):
			phase = i / template.frames
			travel = Matrix.Translation(forward * stride * phase)
			_ResetPose(armature_obj)
			for name, matrix in body.bones.items():
				armature_obj.pose.bones[name].matrix = travel @ matrix
				context.view_layer.update()

			for (limb, touchdown), target in zip(steps, targets):
				foot = _FootPosition(body.neutral[limb.bones[-1]].to_translation(), phase, touchdown, template.duty, forward, up, stride, swing_height)
				target.location = armature_obj.matrix_world @ foot

			context.view_layer.update()

			# The end bone keeps its standing orientation, so a planted foot stays flat and still
			pose = {name: armature_obj.pose.bones[name].matrix.copy() for name in body.bones}
			for limb, _ in steps:
				for name in limb.bones[:-1]:
					pose[name] = armature_obj.pose.bones[name].matrix.copy()

				end = limb.bones[-1]
				head = armature_obj.pose.bones[end].matrix.to_translation()
				pose[end] = Matrix.Translation(head) @ body.neutral[end].to_3x3().to_4x4()

			poses.append(pose)

		return poses
	finally:
		# Remove the temporary IK even if solving fails
		for (limb, _), constraint in zip(steps, constraints):
			armature_obj.pose.bones[limb.bones[-2]].constraints.remove(constraint)
		for target in targets:
			bpy.data.objects.remove(target)


def _FootPosition(neutral: Vector, phase: float, touchdown: float, duty: float, forward: Vector, up: Vector, stride: float, swing_height: float) -> Vector:
	"""The armature-space position of a foot at 'phase' of a cycle in which the body moves forward by 'stride'.

	A planted foot stays where it is under its neutral position at the middle of its stance. A swinging foot moves to the next such point, lifting by
	'swing_height' at the middle of the swing.
	"""
	# Find the start of the most recent touchdown, and how far through the cycle the foot is since then
	local = (phase - touchdown) % 1.0
	landed = phase - local
	planted = neutral + forward * stride * (landed + duty / 2)
	if local < duty:
		return planted

	# Ease between the landings, so the foot starts and stops smoothly, with a lift that is zero and flat at both ends
	s = (local - duty) / (1.0 - duty)
	ease = s * s * (3 - 2 * s)
	return planted + forward * stride * ease + up * swing_height * 64 * (s * (1 - s)) ** 3


def _KeyPoses(armature_obj: bpy.types.Object, context: bpy.types.Context, poses: list[dict[str, Matrix]]) -> None:
	"""Key the location and rotation of every bone at frames 1, 2, ..., one pose per frame. Bones without a pose are keyed at rest."""
	ordered = [armature_obj.pose.bones[bone.name] for bone in BonesParentFirst(armature_obj.data)]
	for frame, pose in enumerate(poses, start=1):
		# Set parents before children, because a pose bone's matrix is set relative to its parent's current pose
		_ResetPose(armature_obj)
		for pose_bone in ordered:
			if pose_bone.name in pose:
				pose_bone.matrix = pose[pose_bone.name]
				context.view_layer.update()

		# Key every bone, so that the cycle does not depend on the pose that the armature had before
		for pose_bone in ordered:
			pose_bone.keyframe_insert("location", frame=frame, group=pose_bone.name)
			pose_bone.keyframe_insert(RotationPath(pose_bone), frame=frame, group=pose_bone.name)


def _ResetPose(armature_obj: bpy.types.Object) -> None:
	"""Put every bone of the armature in its rest pose."""
	for pose_bone in armature_obj.pose.bones:
		pose_bone.location = (0, 0, 0)
		pose_bone.rotation_quaternion = (1, 0, 0, 0)
		pose_bone.rotation_euler = (0, 0, 0)
		pose_bone.rotation_axis_angle = (0, 0, 1, 0)
		pose_bone.scale = (1, 1, 1)


def _SavePose(armature_obj: bpy.types.Object) -> list[tuple]:
	"""Return every bone's pose channels, for '_RestorePose'."""
	return [(b.location.copy(), b.rotation_quaternion.copy(), b.rotation_euler.copy(), tuple(b.rotation_axis_angle), b.scale.copy()) for b in armature_obj.pose.bones]


def _RestorePose(armature_obj: bpy.types.Object, saved: list[tuple]) -> None:
	"""Restore the pose channels returned by '_SavePose'."""
	for pose_bone, (location, quaternion, euler, axis_angle, scale) in zip(armature_obj.pose.bones, saved):
		pose_bone.location, pose_bone.rotation_quaternion, pose_bone.rotation_euler = location, quaternion, euler
		pose_bone.rotation_axis_angle, pose_bone.scale = axis_angle, scale
