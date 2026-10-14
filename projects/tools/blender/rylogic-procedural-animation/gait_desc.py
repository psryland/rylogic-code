"""Build a '.gait' description from an armature's movement cycles, and write it to a file.

The '.gait' schema is owned by 'pr/animation/gait.h' in the Rylogic code library. A gait holds guidance for the runtime's procedural locomotion, not
animation to play back: per family, a stance; and per cycle, the speed, stride, swing height, footfall timing, and the first two harmonics of the body's
repeating motion. All values are in model space (see 'rig_desc.ModelSpace').

A movement cycle is a looping action that moves the armature's root bone forward by one stride. Its frame range covers exactly one cycle, so the pose at
the last frame is the pose at the first frame moved forward by the stride. Feet that stay still on the ground while the body moves are 'planted'.
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import bpy
from mathutils import Matrix, Quaternion, Vector

from .rig_desc import BonesParentFirst, ModelSpace

SCHEMA_VERSION = 3

# The number of harmonics in each bone motion
HARMONICS = 2

# A foot is planted while it moves slower than this fraction of the body's speed...
PLANTED_SPEED_FRACTION = 0.1

# ...and is near the lowest height at which it is still: within this fraction of its vertical travel. Measuring from the still heights, rather than
# the lowest of all heights, keeps a foot planted when its swing dips lower than where it stands.
PLANTED_HEIGHT_FRACTION = 0.25

# Motion smaller than these amplitudes is dropped: radians for rotations, and a fraction of the stride for positions
MIN_ROTATION_AMPLITUDE = math.radians(0.5)
MIN_POSITION_FRACTION = 0.01

# Stance rotations closer to the rest rotation than this, in radians, are left out
MIN_STANCE_ANGLE = math.radians(0.1)

# A cycle loops if every bone's pose at the last frame matches its pose at the first frame, after removing the travel, within these tolerances:
# radians for rotations, and a fraction of the stride for positions
LOOP_ROTATION_TOLERANCE = math.radians(1.0)
LOOP_POSITION_FRACTION = 0.01


@dataclass
class _Limb:
	"""A leg or arm chain that can walk, with the contact that it stands on."""

	role: str          # The '.gait' role: "leg" or "arm"
	side: str          # The '.gait' side
	bones: list[str]   # The chain's bones, base to end
	contact: Any       # The contact on the end bone (a 'RYLOGIC_PG_RigContact')


@dataclass
class _Samples:
	"""The model-space pose of every bone at each frame of a cycle. 'model[i]' is the pose at frame 'start + i', for i in [0, frames]."""

	frames: int                            # The number of frames in one cycle
	fps: float
	model: list[dict[str, Matrix]]


def WalkingLimbs(armature: bpy.types.Armature) -> list[_Limb]:
	"""Return the rig's leg and arm chains that have a contact to stand on: a sole on a leg's end bone, or a palm on an arm's end bone."""
	limbs: list[_Limb] = []
	for chain in armature.rylogic_rig.chains:
		# Only limbs with a contact on their end bone can be planted
		if chain.role not in ("LEG", "ARM") or not chain.bones:
			continue

		kind = "SOLE" if chain.role == "LEG" else "PALM"
		end = chain.bones[-1].name
		contact = next((c for c in armature.rylogic_rig.contacts if c.bone == end and c.kind == kind), None)
		if contact is not None:
			limbs.append(_Limb(role=chain.role.lower(), side=chain.side.lower(), bones=[ref.name for ref in chain.bones], contact=contact))

	return limbs


def BuildGaitDesc(armature_obj: bpy.types.Object, context: bpy.types.Context) -> dict[str, Any]:
	"""Return the '.gait' description of an armature's movement cycles as a JSON-compatible dictionary. Cycles with 'use_export' off are left out.

	Plays each cycle's action on the armature to measure it, then restores the armature's action and the scene's frame. Raises ValueError if a cycle
	cannot be measured. The description is not validated; see 'ValidateGaitDesc'.
	"""
	# Group the exported cycles by family, keeping the order in which families first appear. Unexported cycles are not measured, so they may be unfinished.
	space = ModelSpace(armature_obj)
	limbs = WalkingLimbs(armature_obj.data)
	families: dict[str, list[tuple[str, float, _Samples]]] = {}
	for cycle in armature_obj.data.rylogic_rig.cycles:
		if not cycle.use_export:
			continue

		if cycle.action is None:
			raise ValueError(f"Cycle '{cycle.name}' has no action")

		families.setdefault(cycle.family, []).append((cycle.name, cycle.hip_height, _SampleAction(armature_obj, context, space, cycle.action)))

	# Measure each cycle, then order each family's cycles by speed. The family's stance is the mean pose of its slowest cycle, the nearest to standing.
	out: list[dict[str, Any]] = []
	root = _RootBone(armature_obj.data)
	for family, cycles in families.items():
		measured = sorted((_MeasureCycle(armature_obj.data, space, root, limbs, name, hip_height, samples) for name, hip_height, samples in cycles), key=lambda c: c[0]["speed"])
		out.append({"name": family, "stance": measured[0][1], "cycles": [cycle for cycle, _ in measured]})

	return {"schema_version": SCHEMA_VERSION, "families": out}


def ValidateGaitDesc(desc: dict[str, Any]) -> list[str]:
	"""Check a '.gait' description for the rules that the runtime relies on. Returns a description of each problem, or an empty list if it is valid.

	The runtime checks the gait against the rig as well (see 'Validate' in 'pr/animation/gait.h').
	"""
	errors: list[str] = []
	if not desc["families"]:
		errors.append("There are no exported movement cycles")

	names = [family["name"] for family in desc["families"]]
	for family in desc["families"]:
		# Families are chosen by name, so names must be unique and not empty
		ctx = f"Family '{family['name']}'"
		if not family["name"] or names.count(family["name"]) != 1:
			errors.append(f"{ctx} needs a unique, non-empty name")

		cycles = family["cycles"]
		limbs = [(f["role"], f["side"]) for f in cycles[0]["footfalls"]] if cycles else []
		for i, cycle in enumerate(cycles):
			# Cycles blend by speed, so each must be faster than the one before, and all must walk on the same limbs
			cctx = f"{ctx} cycle '{cycle['name']}'"
			if i != 0 and not cycle["speed"] > cycles[i - 1]["speed"]:
				errors.append(f"{cctx} has the same speed as cycle '{cycles[i - 1]['name']}'")
			if not cycle["stride"] > 0:
				errors.append(f"{cctx} does not move the root bone forward")
			if not 0 < cycle["hip_height"] <= 1:
				errors.append(f"{cctx} needs a hip height in (0,1]")
			if not cycle["footfalls"]:
				errors.append(f"{cctx} has no planted feet")
			if [(f["role"], f["side"]) for f in cycle["footfalls"]] != limbs:
				errors.append(f"{cctx} plants different limbs to cycle '{cycles[0]['name']}'")

	return errors


def WriteGaitDesc(desc: dict[str, Any], path: Path) -> None:
	"""Write a '.gait' description to 'path' as deterministic JSON."""
	path = Path(path)
	path.parent.mkdir(parents=True, exist_ok=True)
	path.write_text(json.dumps(desc, indent=1), encoding="utf-8")


def _RootBone(armature: bpy.types.Armature) -> str:
	"""The bone that carries the body's travel: the first root bone. Raises ValueError if the armature has no bones."""
	bones = BonesParentFirst(armature)
	if not bones:
		raise ValueError(f"Armature '{armature.name}' has no bones")

	return bones[0].name


def _SampleAction(armature_obj: bpy.types.Object, context: bpy.types.Context, space: ModelSpace, action: bpy.types.Action) -> _Samples:
	"""Play 'action' on the armature and record the model-space pose of every bone at each frame of its range, including the last."""
	# The frame range covers one cycle: the last frame repeats the first, moved forward by one stride
	start, end = (int(round(f)) for f in action.frame_range)
	if end - start < 4:
		raise ValueError(f"Action '{action.name}' needs at least 4 frames in its frame range")

	scene = context.scene
	animation_data = armature_obj.animation_data or armature_obj.animation_data_create()
	saved_action, saved_frame = animation_data.action, scene.frame_current
	try:
		# Evaluate each frame, then read the posed bones
		animation_data.action = action
		model: list[dict[str, Matrix]] = []
		for frame in range(start, end + 1):
			scene.frame_set(frame)
			model.append({pose_bone.name: space.BonePose(pose_bone) for pose_bone in armature_obj.pose.bones})
	finally:
		# Leave the authoring state as it was
		animation_data.action = saved_action
		scene.frame_set(saved_frame)

	return _Samples(frames=end - start, fps=scene.render.fps / scene.render.fps_base, model=model)


def _MeasureCycle(armature: bpy.types.Armature, space: ModelSpace, root: str, limbs: list[_Limb], name: str, hip_height: float, samples: _Samples) -> tuple[dict[str, Any], list[dict[str, Any]]]:
	"""Measure one sampled cycle, which carries the hips at 'hip_height' (see the cycle property). Returns its '.gait' cycle and the stance of its mean pose.

	Raises ValueError if the root bone does not move forward, or if the cycle does not loop (see '_CheckLoops').
	"""
	# The body moves steadily by one stride per cycle. Body-relative values remove that travel.
	n = samples.frames
	travel = samples.model[n][root].to_translation() - samples.model[0][root].to_translation()
	stride = travel.length
	if stride < 1e-6:
		raise ValueError(f"Cycle '{name}' does not move the root bone '{root}' forward")

	_CheckLoops(name, samples, travel)

	speed = stride * samples.fps / n
	phases = [i / n for i in range(n)]

	# Find each limb's footfall. Limbs that are never planted, or never lifted, do not walk in this cycle.
	# The walking contacts' mean body-relative position is where the feet stand on average, which is the model origin by convention.
	footfalls: list[dict[str, Any]] = []
	walking: set[str] = set()
	swing_height = 0.0
	feet_centre = Vector()
	for limb in limbs:
		points = _ContactPoints(limb, samples, space.scale)
		footfall, lift = _Footfall(limb, points, samples, stride)
		if footfall is not None:
			footfalls.append(footfall)
			walking.update(limb.bones)
			swing_height = max(swing_height, lift)
			feet_centre += sum((points[i] - travel * phases[i] for i in range(n)), Vector()) / n

	if footfalls:
		feet_centre /= len(footfalls)

	# The local rotation of every bone at each frame, relative to its parent; root bones are relative to model space
	locals_: dict[str, list[Quaternion]] = {}
	for bone in BonesParentFirst(armature):
		rotations = []
		for i in range(n):
			model = samples.model[i]
			parent = model[bone.parent.name].to_quaternion() if bone.parent else Quaternion()
			rotations.append((parent.inverted() @ model[bone.name].to_quaternion()).normalized())

		locals_[bone.name] = rotations

	# The stance is the mean local rotation, and the root's mean body-relative position. Rotations near the rest pose are left out.
	stance: list[dict[str, Any]] = []
	motion: list[dict[str, Any]] = []
	for bone in BonesParentFirst(armature):
		mean = _MeanRotation(locals_[bone.name])
		entry: dict[str, Any] = {"bone": bone.name, "rotation": [mean.x, mean.y, mean.z, mean.w]}
		if bone.parent is None:
			positions = [samples.model[i][bone.name].to_translation() - travel * phases[i] for i in range(n)]
			mean_position = sum(positions, Vector()) / n

			# Place the root relative to where the feet stand, so the stance's feet are around the model origin wherever the cycle starts
			entry["position"] = _List(mean_position - Vector((feet_centre.x, feet_centre.y, 0.0)))

			# The root's repeating offset from its mean position, such as a bob or sway
			harmonics = _Harmonics([p - mean_position for p in positions])
			if _Amplitude(harmonics) >= MIN_POSITION_FRACTION * stride:
				motion.append({"bone": bone.name, "channel": "position", "harmonics": harmonics})

		if bone.parent is None or mean.rotation_difference(_RestLocal(space, bone)).angle > MIN_STANCE_ANGLE:
			stance.append(entry)

		# The repeating rotation of bones outside the walking limbs, as rotation vectors in the bone's frame relative to the stance
		if bone.name in walking:
			continue

		offsets = []
		for rotation in locals_[bone.name]:
			delta = mean.inverted() @ rotation
			offsets.append(_RotationVector(delta))

		harmonics = _Harmonics(offsets)
		if _Amplitude(harmonics) >= MIN_ROTATION_AMPLITUDE:
			motion.append({"bone": bone.name, "channel": "rotation", "harmonics": harmonics})

	cycle = {"name": name, "speed": speed, "stride": stride, "swing_height": swing_height, "hip_height": hip_height, "footfalls": footfalls, "motion": motion}
	return cycle, stance


def _CheckLoops(name: str, samples: _Samples, travel: Vector) -> None:
	"""Raise ValueError, naming the worst bones, if the pose at the last frame is not the pose at the first frame moved forward by 'travel'."""
	# Compare each bone's model-space rotation and travel-corrected position between the first and last frames
	first, last = samples.model[0], samples.model[samples.frames]
	position_tolerance = LOOP_POSITION_FRACTION * travel.length
	mismatches: list[tuple[float, str]] = []
	for bone, start in first.items():
		angle = start.to_quaternion().rotation_difference(last[bone].to_quaternion()).angle
		distance = (last[bone].to_translation() - travel - start.to_translation()).length
		if angle > LOOP_ROTATION_TOLERANCE or distance > position_tolerance:
			score = max(angle / LOOP_ROTATION_TOLERANCE, distance / position_tolerance)
			mismatches.append((score, f"'{bone}' ({math.degrees(angle):.1f}°, {distance:.3g} apart)"))

	if not mismatches:
		return

	# Report the worst few, because one wrong parent key moves every bone below it
	mismatches.sort(reverse=True)
	worst = ", ".join(text for _, text in mismatches[:3])
	more = f" and {len(mismatches) - 3} more" if len(mismatches) > 3 else ""
	raise ValueError(f"Cycle '{name}' does not loop: the last frame does not match the first frame moved forward by one stride. Worst bones: {worst}{more}")


def _ContactPoints(limb: _Limb, samples: _Samples, scale: float) -> list[Vector]:
	"""The model-space position of a limb's contact at each sampled frame, including the last."""
	# Bone transforms are rigid, so the bone-space offset carries the armature's uniform scale
	end = limb.bones[-1]
	offset = Vector(limb.contact.offset) * scale
	return [samples.model[i][end] @ offset for i in range(samples.frames + 1)]


def _Footfall(limb: _Limb, points: list[Vector], samples: _Samples, stride: float) -> tuple[dict[str, Any] | None, float]:
	"""Find when a limb's contact is planted from its sampled positions, in a cycle whose body moves 'stride' forward.
	Returns its footfall, or None if the limb does not walk, and the height that it lifts."""
	# A contact is planted from frame i to i+1 while it stays still and near the ground, so the planted frames count the time spent planted.
	# The ground is the lowest still height, which excludes a foot paused high in its swing and a swing that dips below the ground.
	n = samples.frames
	body_speed = stride * samples.fps / n
	heights = [p.z for p in points[:n]]
	low, high = min(heights), max(heights)
	still = [(points[i + 1] - points[i]).length * samples.fps < PLANTED_SPEED_FRACTION * body_speed for i in range(n)]
	if not any(still):
		return None, 0.0

	ground = min(heights[i] for i in range(n) if still[i]) + max(PLANTED_HEIGHT_FRACTION * (high - low), 1e-5)
	planted = [still[i] and heights[i] <= ground for i in range(n)]

	# The limb walks if it is planted for part of the cycle. Touchdown is the start of the longest planted run.
	count = sum(planted)
	if count == 0 or count == n:
		return None, 0.0

	best_start, best_length = 0, 0
	for i in range(n):
		if planted[i] and not planted[i - 1]:
			length = 0
			while length != n and planted[(i + length) % n]:
				length += 1
			if length > best_length:
				best_start, best_length = i, length

	footfall = {"role": limb.role, "side": limb.side, "touchdown": best_start / n, "duty": count / n}
	return footfall, high - low


def _RestLocal(space: ModelSpace, bone: bpy.types.Bone) -> Quaternion:
	"""The bone's rest rotation relative to its parent, as in the '.rig'; root bones are relative to model space."""
	rest = space.BoneRest(bone).to_quaternion()
	return rest if bone.parent is None else (space.BoneRest(bone.parent).to_quaternion().inverted() @ rest).normalized()


def _MeanRotation(rotations: list[Quaternion]) -> Quaternion:
	"""The normalised mean of rotations that are close together, with each aligned to the first's sign so that opposite signs do not cancel."""
	first = rotations[0]
	total = Vector((0.0, 0.0, 0.0, 0.0))
	for q in rotations:
		v = Vector((q.w, q.x, q.y, q.z))
		total += v if first.dot(q) >= 0 else -v

	return Quaternion(total).normalized()


def _RotationVector(q: Quaternion) -> Vector:
	"""The rotation vector (axis * angle) of 'q', taking the shorter way around."""
	if q.w < 0:
		q = -q

	axis, angle = q.to_axis_angle()
	return axis * angle


def _Harmonics(values: list[Vector]) -> list[dict[str, list[float]]]:
	"""The first harmonics of 'values', which are evenly spaced samples of one cycle, as '.gait' harmonics: {cos, sin} for k = 1 .. HARMONICS."""
	# Project onto each cosine and sine. With even samples over one cycle, these are the least-squares fit.
	n = len(values)
	out = []
	for k in range(1, HARMONICS + 1):
		c, s = Vector(), Vector()
		for i, value in enumerate(values):
			angle = 2 * math.pi * k * i / n
			c += value * math.cos(angle)
			s += value * math.sin(angle)

		out.append({"cos": _List(c * (2 / n)), "sin": _List(s * (2 / n))})

	return out


def _Amplitude(harmonics: list[dict[str, list[float]]]) -> float:
	"""The largest amplitude of any harmonic of a motion."""
	return max(math.sqrt(sum(c * c + s * s for c, s in zip(h["cos"], h["sin"]))) for h in harmonics)


def _List(vector: Vector) -> list[float]:
	"""Return a vector as a list of floats."""
	return [float(value) for value in vector]
