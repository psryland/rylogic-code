"""Generate starting-point movement cycles for an armature, to be refined by hand.

A template moves the root bone forward along the model's forward axis (-Y) by one stride per cycle, and steps each walking limb (see
'gait_desc.WalkingLimbs') in a standard footfall pattern: a planted foot stays still on the ground, and a swinging foot moves to its next landing in
a smooth arc. Templates that walk on arms pitch the spine forward until it is level, so that the arms can reach the ground. The body is lowered until
every walking limb reaches the ground under it with its joints bent, and the stride is shortened if a limb cannot reach the ends of its stance. Arms
that do not walk hang down at the body's sides. The rest of the body keeps its rest pose. Limb lengths set the stride and swing height, so the same
template suits any size of rig. Swimming templates walk on no limbs: the root moves forward, the spine pitches level with the tails straight behind
it, the neck curves up to hold the head level above the water, and each limb folds back along the body with its hand or foot flat, paddling gently
in the slower cycle.
"""

import math
from dataclasses import dataclass

import bpy
from mathutils import Matrix, Quaternion, Vector

from .gait_desc import WalkingLimbs, _Limb
from .rig_desc import BonesParentFirst

# The fraction of a limb's reach at which it stands, and the most that it stretches to at the ends of its stance
STANCE_REACH = 0.85
MAX_REACH = 0.95

# The directions in which arms that do not walk hang: down, plus these fractions of outward, back, and forward
HANG_UPPER_OUT = 0.30
HANG_UPPER_BACK = 0.15
HANG_FORE_OUT = 0.18
HANG_FORE_FORWARD = 0.35


@dataclass(frozen=True)
class _Template:
	"""A standard movement cycle."""

	family: str
	name: str
	frames: int                                     # Frames per cycle
	stride: float                                   # The distance moved per cycle, as a multiple of the leg length, if the limbs can reach it
	swing_height: float                             # The foot lift, as a multiple of the leg length
	hip_height: float                               # The cycle's hip height (see 'RYLOGIC_PG_GaitCycle.hip_height')
	spine_sway: float                               # The cycle's spine sway in radians (see 'RYLOGIC_PG_GaitCycle.spine_sway')
	weight_shift: float                             # The cycle's weight shift (see 'RYLOGIC_PG_GaitCycle.weight_shift')
	arm_swing: float                                # The cycle's arm swing in radians (see 'RYLOGIC_PG_GaitCycle.arm_swing')
	swing_out: float                                # The walking legs' swing out, as a multiple of the leg length; walking arms swing out half as far (see 'RYLOGIC_PG_GaitCycle.leg_swing_out')
	duty: float                                     # The fraction of the cycle that each foot is planted
	touchdowns: dict[tuple[str, str], float]        # The touchdown phase of each walking limb, by (role, side)


TEMPLATES = {
	"BIPEDAL_WALK": _Template("bipedal", "walk", 24, 1.0, 0.15, 0.9, 0.1, 0.42, 0.4, 0.0, 0.6, {("LEG", "LEFT"): 0.0, ("LEG", "RIGHT"): 0.5}),
	"QUADRUPED_WALK": _Template("quadruped", "walk", 24, 1.0, 0.15, 0.5, 0.25, 0.0, 0.0, 0.2, 0.75, {("LEG", "LEFT"): 0.0, ("ARM", "LEFT"): 0.25, ("LEG", "RIGHT"): 0.5, ("ARM", "RIGHT"): 0.75}),
	"QUADRUPED_TROT": _Template("quadruped", "trot", 16, 1.4, 0.2, 0.5, 0.35, 0.0, 0.0, 0.2, 0.5, {("LEG", "LEFT"): 0.0, ("ARM", "RIGHT"): 0.0, ("LEG", "RIGHT"): 0.5, ("ARM", "LEFT"): 0.5}),
}
TEMPLATE_ITEMS = [
	("BIPEDAL_WALK", "Bipedal Walk", "Legs only, alternating"),
	("QUADRUPED_WALK", "Quadruped Walk", "Four-beat walk: left hind, left fore, right hind, right fore"),
	("QUADRUPED_TROT", "Quadruped Trot", "Diagonal pairs step together"),
	("SWIM_PADDLE", "Swim Paddle", "Slow swimming and floating: the limbs spread out to the sides and scull gently"),
	("SWIM_GLIDE", "Swim Glide", "Fast swimming: the limbs tuck in, with bent elbows and knees, and the hands and feet against the body"),
]


@dataclass(frozen=True)
class _SwimTemplate:
	"""A standard swimming cycle. No limb walks; the runtime adds the body wave (see the family's wave properties)."""

	name: str
	frames: int          # Frames per cycle
	stride: float        # The distance moved per cycle, as a multiple of the leg length
	paddle: float        # The angle, in radians, by which each limb sweeps forward and back about the body's up axis
	limb: tuple[tuple[float, float, float], ...]  # Per bone of a limb chain, its direction as (back, out, down) amounts. Later bones use the last entry.


SWIM_TEMPLATES = {
	# Floating: the limbs reach out to the sides, a little back and down, so that they steady the body like outriggers
	"SWIM_PADDLE": _SwimTemplate("paddle", 24, 0.8, math.radians(25), ((0.3, 1.0, 0.25),)),

	# Swimming fast: the upper bone points back and out, the lower bone folds back in towards the body, and the hand or foot lies back along the flank
	"SWIM_GLIDE": _SwimTemplate("glide", 16, 1.6, 0.0, ((0.6, 0.7, 0.3), (1.0, -0.5, 0.0), (1.0, -0.1, 0.0))),
}


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
	# Swimming templates have no walking limbs, so they are posed without IK
	if template_id in SWIM_TEMPLATES:
		return _GenerateSwim(armature_obj, context, SWIM_TEMPLATES[template_id])

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

	# Register the cycle, adding its family if the rig does not have it yet. Upright bipeds stand against gravity on gentle ground; others follow the ground.
	rig = armature.rylogic_rig
	if template.family not in rig.families:
		family = rig.families.add()
		family.name = template.family
		family.up, family.max_slope = ("GRAVITY", math.radians(25)) if template.family == "bipedal" else ("SURFACE", math.pi)

	cycle = rig.cycles.add()
	cycle.name, cycle.family, cycle.action = template.name, template.family, action
	cycle.hip_height, cycle.spine_sway = template.hip_height, template.spine_sway
	cycle.weight_shift, cycle.arm_swing = template.weight_shift, template.arm_swing
	cycle.leg_swing_out, cycle.arm_swing_out = template.swing_out * leg_length, 0.5 * template.swing_out * leg_length
	rig.active_cycle = len(rig.cycles) - 1
	return action


def _GenerateSwim(armature_obj: bpy.types.Object, context: bpy.types.Context, template: _SwimTemplate) -> bpy.types.Action:
	"""Key a new swimming cycle, add it to the armature's movement cycles, and return it. See 'GenerateTemplate'.

	The body lies along the water: the spine pitches about its base until it points forward and level, and each tail runs straight back from its base
	in line with the spine. The neck curves up from the spine so that the head keeps its rest orientation, facing forward and level. Each bone of each
	leg and arm points in the template's direction for it, on the limb's own side, except that the end bone lies level with its palm or sole facing
	down. A paddling cycle sweeps each limb forward and back about the body's up axis, with each side and each girdle half a cycle apart.
	"""
	# Template sizes are in armature space, relative to the length of the first leg chain
	armature = armature_obj.data
	rig = armature.rylogic_rig
	limbs = [chain for chain in rig.chains if chain.role in ("LEG", "ARM") and chain.bones]
	legs = [chain for chain in limbs if chain.role == "LEG"]
	if not legs:
		raise ValueError("A swimming template needs a leg chain to size the stride")

	leg_length = sum(armature.bones[ref.name].length for ref in legs[0].bones)
	stride = template.stride * leg_length
	to_armature = armature_obj.matrix_world.to_3x3().inverted()
	forward = (to_armature @ Vector((0, -1, 0))).normalized()
	up = (to_armature @ Vector((0, 0, 1))).normalized()
	lateral = forward.cross(up).normalized()

	# Pitch the spine about the head of its first bone until the line from there to the end of its last bone points forward. The pitch turns
	# everything that hangs from the first spine bone, including the tails, limbs, neck, and head. A rig without a spine keeps its rest pose.
	rest = {bone.name: bone.matrix_local.copy() for bone in armature.bones}
	pitch, pitched = Matrix.Identity(4), None
	spine = next((chain for chain in rig.chains if chain.role == "SPINE" and chain.bones), None)
	if spine is not None:
		pitched = armature.bones[spine.bones[0].name]
		pivot = pitched.head_local.copy()
		along = armature.bones[spine.bones[-1].name].tail_local - pivot
		along -= lateral * along.dot(lateral)
		pitch = Matrix.Translation(pivot) @ along.rotation_difference(forward).to_matrix().to_4x4() @ Matrix.Translation(-pivot)

	def Posed(name: str) -> Matrix:
		"""The armature-space matrix of a bone in the pitched body."""
		bone = armature.bones[name]
		return pitch @ rest[name] if pitched is not None and (bone == pitched or pitched in bone.parent_recursive) else rest[name]

	# Lay each tail bone straight back from the end of the one before it, so the tail continues the line of the spine
	body = {} if pitched is None else {pitched.name: pitch @ rest[pitched.name]}
	for chain in rig.chains:
		if chain.role != "TAIL" or not chain.bones:
			continue

		head = Posed(chain.bones[0].name).to_translation()
		for ref in chain.bones:
			posed = Posed(ref.name).to_3x3()
			turn = (posed @ Vector((0, 1, 0))).rotation_difference(-forward).to_matrix()
			body[ref.name] = Matrix.Translation(head) @ (turn @ posed).to_4x4()
			head = head - forward * armature.bones[ref.name].length

	# Hold the head up out of the water, facing forward. Each neck bone takes back an equal share of the spine's pitch, so the neck curves up from
	# the levelled spine and the last bone, the head, returns to its rest orientation, which faces forward and level.
	unpitch = pitch.to_quaternion().inverted()
	for chain in rig.chains:
		if chain.role != "NECK" or not chain.bones:
			continue

		head = Posed(chain.bones[0].name).to_translation()
		for b, ref in enumerate(chain.bones):
			rotation = (Quaternion().slerp(unpitch, (b + 1) / len(chain.bones)) @ Posed(ref.name).to_quaternion()).to_matrix()
			body[ref.name] = Matrix.Translation(head) @ rotation.to_4x4()
			head = head + rotation @ Vector((0, armature.bones[ref.name].length, 0))

	# The palm or sole of each limb's end bone faces down, so that the hand or foot lies flat on the water
	contacts = {contact.bone: Vector(contact.normal) for contact in rig.contacts if contact.kind in ("SOLE", "PALM")}

	# Key each frame: the root carried forward, the levelled body, and each limb posed in the template's directions plus its paddle sweep
	scene = context.scene
	root = BonesParentFirst(armature)[0].name
	animation_data = armature_obj.animation_data or armature_obj.animation_data_create()
	saved_action, saved_frame, saved_pose = animation_data.action, scene.frame_current, _SavePose(armature_obj)
	try:
		poses = []
		for i in range(template.frames + 1):
			# Move the root first, because the body's rest positions are relative to it
			phase = i / template.frames
			travel = Matrix.Translation(forward * stride * phase)
			pose = {root: travel @ rest[root]} | {name: travel @ matrix for name, matrix in body.items()}
			for chain in limbs:
				# Each bone of the chain points in its template direction on the limb's own side, swept about the body's up axis, starting from the
				# end of the one before it. The bones below the chain, such as digits, keep their rest pose relative to it.
				head = Posed(chain.bones[0].name).to_translation()
				out = lateral if head.dot(lateral) > 0 else -lateral
				offset = (0.0 if chain.side != "RIGHT" else 0.5) + (0.0 if chain.role == "LEG" else 0.5)
				sweep = Matrix.Rotation(template.paddle * math.sin(2 * math.pi * (phase - offset)), 3, up)
				for b, ref in enumerate(chain.bones):
					# The end bone lies level, so that its hand or foot is flat on the water
					back, outward, down = template.limb[min(b, len(template.limb) - 1)]
					end = b == len(chain.bones) - 1
					direction = sweep @ (-forward * back + out * outward - up * (0.0 if end else down)).normalized()
					posed = Posed(ref.name).to_3x3()
					turn = (posed @ Vector((0, 1, 0))).rotation_difference(direction).to_matrix()
					oriented = turn @ posed

					# Roll the end bone about its length until its palm or sole faces as nearly down as it can
					if end and ref.name in contacts:
						normal = oriented @ contacts[ref.name]
						facing, target = normal - direction * normal.dot(direction), -up - direction * -up.dot(direction)
						if facing.length > 1e-6 and target.length > 1e-6:
							oriented = facing.rotation_difference(target).to_matrix() @ oriented

					pose[ref.name] = travel @ Matrix.Translation(head) @ oriented.to_4x4()
					head = head + direction * armature.bones[ref.name].length

			poses.append(pose)

		action = bpy.data.actions.new(f"{armature_obj.name}.swim.{template.name}")
		animation_data.action = action
		_KeyPoses(armature_obj, context, poses)
		action.use_frame_range = True
		action.frame_start, action.frame_end = 1, 1 + template.frames
	finally:
		# Leave the authoring state as it was
		animation_data.action = saved_action
		_RestorePose(armature_obj, saved_pose)
		scene.frame_set(saved_frame)

	# Register the cycle, adding a swimming family if the rig does not have it yet. Swimmers follow the water's surface on any slope.
	if "swim" not in rig.families:
		family = rig.families.add()
		family.name = "swim"
		family.up, family.max_slope, family.medium = "SURFACE", math.pi, "WATER"
		family.wave_amplitude, family.wave_wavelength, family.wave_growth = 1.0, 1.0, 2.0

	cycle = rig.cycles.add()
	cycle.name, cycle.family, cycle.action = template.name, "swim", action
	cycle.hip_height, cycle.spine_sway, cycle.weight_shift, cycle.arm_swing = 0.5, 0.0, 0.0, 0.0
	cycle.leg_swing_out, cycle.arm_swing_out = 0.0, 0.0
	rig.active_cycle = len(rig.cycles) - 1
	return action


@dataclass
class _Body:
	"""The pose of the body at the start of a cycle, in armature space."""

	bones: dict[str, Matrix]       # The posed body bones and hanging arms, parents first; the bones below them follow at rest
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

	# Hang the arms that do not walk down at the body's sides, with the elbows a little back and bent, rather than leaving them at rest
	walking = {limb.bones[0] for limb in limbs}
	for chain in armature.rylogic_rig.chains:
		if chain.role != "ARM" or len(chain.bones) < 3 or chain.bones[0].name in walking:
			continue

		upper, fore = chain.bones[0].name, chain.bones[1].name
		head = Posed(upper).to_translation()
		out = lateral if head.dot(lateral) > 0 else -lateral
		hang = {upper: (-up + out * HANG_UPPER_OUT - forward * HANG_UPPER_BACK).normalized(), fore: (-up + out * HANG_FORE_OUT + forward * HANG_FORE_FORWARD).normalized()}

		# Turn the upper arm, then the forearm from the upper arm's new end. The bones below the forearm keep their rest pose relative to it.
		for name in (upper, fore):
			bone, posed = armature.bones[name], Posed(name)
			turn = (posed.to_3x3() @ Vector((0, 1, 0))).rotation_difference(hang[name]).to_matrix()
			matrix = Matrix.Translation(head) @ (turn @ posed.to_3x3()).to_4x4()
			bones[name] = lower @ matrix
			head = head + hang[name] * bone.length

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
