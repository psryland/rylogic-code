"""Swing-twist joint limits, and their derivation from a range-of-motion action.

A joint limit bounds a bone's rotation from its rest pose, in the bone's rest space. The rotation is split into a twist about the bone's +Y axis followed
by a swing that moves +Y. The swing is a vector in the bone's XZ plane, whose direction is the swing axis and whose length is the swing angle. Its X and
Z components are limited by an ellipse with a separate radius in each quadrant. This matches 'JointLimit' in 'pr/animation/rig.h', which is canonical.

A range-of-motion action poses the armature at the extremes of its joints, one pose per keyframe. Deriving limits from it sets each posed bone's ranges
to the smallest that contain every sampled pose, plus some padding.

Left and right bones are mirror partners when their names differ only by side (see 'bpy.utils.flip_name') and their rest transforms are mirror images
across the armature's X = 0 plane, as Blender's Symmetrize makes them. Mirroring a rotation across that plane keeps its swing X and negates its swing Z
and twist, so a mirrored limit keeps its swing X range and negates and swaps its swing Z and twist ranges.
"""

import math
import re
from dataclasses import dataclass

import bpy
from mathutils import Matrix, Quaternion, Vector

BONE_PATH = re.compile(r'^pose\.bones\["(?P<name>(?:[^"\\]|\\.)*)"\]\.')

# Rotations within this angle of a joint's limit count as inside it, so that poses keyed exactly at a limit are not reported
LIMIT_TOLERANCE = math.radians(0.1)


@dataclass
class SwingTwist:
	"""A rotation from the rest pose as a swing vector (X, Z components) and a twist angle, all in radians."""

	swing_x: float
	swing_z: float
	twist: float


def Decompose(rot: Quaternion) -> SwingTwist:
	"""Split a bone-space rotation from the rest pose into its swing and twist. See 'JointLimit::Decompose'."""
	# Use the positive-w form so that the twist angle is within [-pi, pi]
	if rot.w < 0:
		rot = -rot

	# The twist is the part of the rotation about +Y. When that part is zero the bone is swung by pi, and the twist is taken as zero.
	twist_len = math.hypot(rot.y, rot.w)
	twist = Quaternion((rot.w / twist_len, 0, rot.y / twist_len, 0)) if twist_len > 0 else Quaternion()
	twist_angle = 2.0 * math.atan2(twist.y, twist.w)

	# The swing is what remains. Its axis is in the bone's XZ plane, and its w is not negative.
	sw = rot @ twist.conjugated()
	sin_half = math.hypot(sw.x, sw.z)
	swing_angle = 2.0 * math.atan2(sin_half, sw.w)
	scale = swing_angle / sin_half if sin_half > 0 else 2.0
	return SwingTwist(sw.x * scale, sw.z * scale, twist_angle)


def MirrorSample(st: SwingTwist) -> SwingTwist:
	"""Return the swing and twist of the mirror image of a rotation, as seen by the bone's mirror partner."""
	return SwingTwist(st.swing_x, -st.swing_z, -st.twist)


def MirrorPartners(armature: bpy.types.Armature) -> tuple[dict[str, str], list[str]]:
	"""Return a map from each bone's name to its mirror partner's name, and the names of side-named bones whose partner is missing or not a mirror image.

	Centre bones, and bones that are not true mirror images of their partner, have no entry in the map.
	"""
	# Mirroring across X = 0 is a reflection on both sides of the rest transform. The bone-side reflection keeps the bone frame right-handed.
	reflect = Matrix.Diagonal((-1, 1, 1, 1))
	partners: dict[str, str] = {}
	unmatched: list[str] = []
	for bone in armature.bones:
		name = bpy.utils.flip_name(bone.name)
		if name == bone.name:
			continue

		partner = armature.bones.get(name)
		tolerance = 1e-4 * max(1.0, bone.length)
		expected = reflect @ bone.matrix_local @ reflect
		if partner is None or any(abs(a - b) > tolerance for row_a, row_b in zip(expected, partner.matrix_local) for a, b in zip(row_a, row_b)):
			unmatched.append(bone.name)
		else:
			partners[bone.name] = name

	return partners, unmatched


def MirrorJoint(source, target) -> None:
	"""Set the 'target' joint's limit to the mirror image of 'source'."""
	# Swing X is unchanged, while swing Z and twist ranges are negated, which also swaps their ends
	target.use_limit = source.use_limit
	target.swing_x_min, target.swing_x_max = source.swing_x_min, source.swing_x_max
	target.swing_z_min, target.swing_z_max = -source.swing_z_max, -source.swing_z_min
	target.twist_min, target.twist_max = -source.twist_max, -source.twist_min


def SwingRotation(swing_x: float, swing_z: float) -> Quaternion:
	"""Return the rotation that swings the bone by the given swing vector."""
	angle = math.hypot(swing_x, swing_z)
	return Quaternion(Vector((swing_x / angle, 0, swing_z / angle)), angle) if angle > 0 else Quaternion()


def SwingExcess(joint, swing_x: float, swing_z: float) -> float:
	"""Return how far a swing vector is outside the joint's swing ellipse, as the squared ellipse radius. Values above 1 are outside."""
	# Each quadrant of the ellipse has its own radii. A zero radius means no swing that way, which is checked against the range directly.
	excess = 0.0
	for value, low, high in ((swing_x, joint.swing_x_min, joint.swing_x_max), (swing_z, joint.swing_z_min, joint.swing_z_max)):
		radius = high if value >= 0 else -low
		if radius > 0:
			excess += (value / radius) ** 2
		elif value != 0:
			return math.inf

	return excess


def PoseRotation(pose_bone: bpy.types.PoseBone) -> Quaternion:
	"""Return the bone's current rotation from its rest pose, in its rest space, including the effect of constraints and IK."""
	# The rest frame follows the parent's current pose, which is how the solver applies limits. Scale is removed from both frames.
	bone = pose_bone.bone
	parent_pose = pose_bone.parent.matrix if pose_bone.parent else Matrix.Identity(4)
	parent_rest = bone.parent.matrix_local if bone.parent else Matrix.Identity(4)
	rest = (parent_pose @ parent_rest.inverted() @ bone.matrix_local).to_3x3().normalized()
	return (rest.inverted() @ pose_bone.matrix.to_3x3().normalized()).to_quaternion()


def LimitExcess(joint, rot: Quaternion) -> float:
	"""Return roughly how far, in radians, a rotation from rest is outside the joint's limit. Zero when it is inside, or the joint is unlimited."""
	# An unlimited joint accepts every rotation
	if not joint.use_limit:
		return 0.0

	# Twist is outside its range by a plain angle difference
	st = Decompose(rot)
	twist = max(joint.twist_min - st.twist, st.twist - joint.twist_max, 0.0)

	# Swing on an axis with no range that way is all outside. On the other axes, swing is outside by the part of its length beyond the ellipse.
	blocked = 0.0
	inside: list[tuple[float, float]] = []
	for value, low, high in ((st.swing_x, joint.swing_x_min, joint.swing_x_max), (st.swing_z, joint.swing_z_min, joint.swing_z_max)):
		radius = high if value >= 0 else -low
		if radius > 0:
			inside.append((value, radius))
		else:
			blocked = math.hypot(blocked, value)

	excess = sum((value / radius) ** 2 for value, radius in inside)
	beyond = math.hypot(*(value for value, _ in inside)) * (1.0 - 1.0 / math.sqrt(excess)) if excess > 1.0 else 0.0
	return max(math.hypot(blocked, beyond), twist)


@dataclass
class LimitViolation:
	"""A bone that leaves its joint limit during an action."""

	bone: str
	frames: list[int]
	worst_frame: int
	worst_excess: float


def CheckAction(armature_obj: bpy.types.Object, scene: bpy.types.Scene, action: bpy.types.Action) -> list[LimitViolation]:
	"""Play 'action' on the armature over its frame range and return each bone that leaves its joint limit, worst first.

	The evaluated pose is checked, so constraints and IK count. The armature's action and the scene's frame are restored afterwards.
	"""
	# Evaluate every frame of the range, including the last
	start, end = (int(round(f)) for f in action.frame_range)
	animation_data = armature_obj.animation_data or armature_obj.animation_data_create()
	saved_action, saved_frame = animation_data.action, scene.frame_current
	found: dict[str, LimitViolation] = {}
	try:
		animation_data.action = action
		for frame in range(start, end + 1):
			# Record each bone outside its limit, keeping its worst frame
			scene.frame_set(frame)
			for pose_bone in armature_obj.pose.bones:
				excess = LimitExcess(pose_bone.bone.rylogic_rig, PoseRotation(pose_bone))
				if excess <= LIMIT_TOLERANCE:
					continue

				violation = found.setdefault(pose_bone.name, LimitViolation(pose_bone.name, [], frame, 0.0))
				violation.frames.append(frame)
				if excess > violation.worst_excess:
					violation.worst_frame, violation.worst_excess = frame, excess
	finally:
		# Leave the authoring state as it was
		animation_data.action = saved_action
		scene.frame_set(saved_frame)

	return sorted(found.values(), key=lambda violation: -violation.worst_excess)


def CopyToIkLimits(armature_obj: bpy.types.Object) -> int:
	"""Set each pose bone's Blender IK limits from its joint limit, so that Blender's IK, including Auto IK, keeps roughly within it.

	Swing X and Z map to the IK X and Z ranges and twist maps to the IK Y range. Blender limits each axis separately, so the swing ellipse becomes the
	box around it, and the result is an approximation. Blender's ranges must contain the rest pose, so they are widened to include zero. A zero-width
	range locks its axis. Bones without a joint limit have their IK limits cleared. Returns the number of bones with IK limits.
	"""
	# Configure the three IK axes of every bone
	count = 0
	for pose_bone in armature_obj.pose.bones:
		joint = pose_bone.bone.rylogic_rig
		ranges = {"x": (joint.swing_x_min, joint.swing_x_max), "y": (joint.twist_min, joint.twist_max), "z": (joint.swing_z_min, joint.swing_z_max)}
		for axis, (low, high) in ranges.items():
			low, high = min(low, 0.0), max(high, 0.0)
			locked = joint.use_limit and high - low <= LIMIT_TOLERANCE
			setattr(pose_bone, f"lock_ik_{axis}", locked)
			setattr(pose_bone, f"use_ik_limit_{axis}", joint.use_limit and not locked)
			setattr(pose_bone, f"ik_min_{axis}", low if joint.use_limit else -math.pi)
			setattr(pose_bone, f"ik_max_{axis}", high if joint.use_limit else math.pi)

		count += joint.use_limit

	return count


def FrameRanges(frames: list[int]) -> str:
	"""Describe ascending frame numbers as runs, e.g. '3-7, 12'."""
	# Start a new run at each gap
	runs: list[list[int]] = []
	for frame in frames:
		if runs and frame == runs[-1][1] + 1:
			runs[-1][1] = frame
		else:
			runs.append([frame, frame])

	return ", ".join(f"{low}-{high}" if high != low else f"{low}" for low, high in runs)


def RangeOfMotionFrames(action: bpy.types.Action) -> tuple[list[float], set[str]]:
	"""Return the keyframe times of 'action' in order, and the names of the bones that it animates."""
	# Each keyframe is one sampled pose. Bones are identified by their channels' data paths.
	frames: set[float] = set()
	bones: set[str] = set()
	for fcurve in _FCurves(action):
		match = BONE_PATH.match(fcurve.data_path)
		if match is None:
			continue

		bones.add(match.group("name").replace('\\"', '"').replace("\\\\", "\\"))
		frames.update(point.co.x for point in fcurve.keyframe_points)

	return sorted(frames), bones


def DeriveLimits(armature_obj: bpy.types.Object, action: bpy.types.Action, padding: float, mirror: bool) -> tuple[int, int, list[str]]:
	"""Set the limit of each bone that moves in 'action' to contain every keyframed pose, widened by 'padding' radians.

	If 'mirror' is set, each sample also counts, mirrored, for the bone's mirror partner, so both sides get the same range of motion. Bones that the
	action does not move keep their limits. The armature's pose is restored afterwards. Returns the number of bones changed, the number of poses sampled,
	and, when mirroring, the side-named bones that could not be mirrored because they have no mirror partner.
	"""
	# Sample each keyed bone's rotation from its rest pose at every keyframe. 'matrix_basis' is that rotation in the bone's rest space.
	frames, bone_names = RangeOfMotionFrames(action)
	pose_bones = [pose_bone for pose_bone in armature_obj.pose.bones if pose_bone.name in bone_names]
	saved = {pose_bone.name: pose_bone.matrix_basis.copy() for pose_bone in armature_obj.pose.bones}
	selected = [pose_bone for pose_bone in armature_obj.pose.bones if pose_bone.select]
	samples: dict[str, list[SwingTwist]] = {pose_bone.name: [] for pose_bone in pose_bones}
	try:
		# 'apply_pose_from_action' poses only the selected bones when any are selected, so clear the selection while sampling
		for pose_bone in selected:
			pose_bone.select = False

		for frame in frames:
			armature_obj.pose.apply_pose_from_action(action, evaluation_time=frame)
			for pose_bone in pose_bones:
				samples[pose_bone.name].append(Decompose(pose_bone.matrix_basis.to_quaternion()))
	finally:
		# Leave the user's pose and selection as they were
		for pose_bone in armature_obj.pose.bones:
			pose_bone.matrix_basis = saved[pose_bone.name]
		for pose_bone in selected:
			pose_bone.select = True

	# Pool each bone's samples with its partner's mirrored samples. A partner that is not in the action still gets the mirrored samples.
	unmatched: list[str] = []
	if mirror:
		partners, unmatched = MirrorPartners(armature_obj.data)
		pooled = {name: list(values) for name, values in samples.items()}
		for name, values in samples.items():
			if name in partners:
				pooled.setdefault(partners[name], []).extend(MirrorSample(st) for st in values)

		samples = pooled
		unmatched = [name for name in unmatched if name in samples]

	# Fit each moving bone's ranges to its samples
	changed = 0
	for name, values in samples.items():
		if _FitLimit(armature_obj.data.bones[name].rylogic_rig, values, padding):
			changed += 1

	return changed, len(frames), unmatched


def _FitLimit(joint, samples: list[SwingTwist], padding: float) -> bool:
	"""Set a joint's ranges to contain 'samples' and the rest pose, widened by 'padding'. Returns False, leaving the joint unchanged, if no sample moves."""
	# A bone that never leaves its rest pose gives no information about its range
	tolerance = 1e-4
	if all(max(abs(s.swing_x), abs(s.swing_z), abs(s.twist)) <= tolerance for s in samples):
		return False

	# Each range spans the sampled extremes and the rest pose, plus the padding
	def Range(values: list[float]) -> tuple[float, float]:
		"""Return the padded range of 'values' and zero, within [-pi, pi]."""
		return max(-math.pi, min(0.0, *values) - padding), min(math.pi, max(0.0, *values) + padding)

	joint.use_limit = True
	joint.swing_x_min, joint.swing_x_max = Range([s.swing_x for s in samples])
	joint.swing_z_min, joint.swing_z_max = Range([s.swing_z for s in samples])
	joint.twist_min, joint.twist_max = Range([s.twist for s in samples])

	# The swing ellipse is inside the box of the ranges, so a diagonal sample can be outside it. Grow the swing ranges until every sample fits.
	growth = math.sqrt(max(1.0, *(SwingExcess(joint, s.swing_x, s.swing_z) for s in samples)))
	if growth > 1.0:
		joint.swing_x_min, joint.swing_x_max = max(-math.pi, joint.swing_x_min * growth), min(math.pi, joint.swing_x_max * growth)
		joint.swing_z_min, joint.swing_z_max = max(-math.pi, joint.swing_z_min * growth), min(math.pi, joint.swing_z_max * growth)

	return True


def _FCurves(action: bpy.types.Action):
	"""Yield every F-curve of 'action', whether it uses layered (slotted) or legacy storage."""
	# Blender 4.4 and later store channels in layers of strips, one channel bag per slot. Older versions store them on the action.
	layers = getattr(action, "layers", None)
	if layers:
		for layer in layers:
			for strip in layer.strips:
				for channelbag in strip.channelbags:
					yield from channelbag.fcurves
	elif hasattr(action, "fcurves"):
		yield from action.fcurves
