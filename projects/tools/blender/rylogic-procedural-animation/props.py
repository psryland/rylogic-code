"""Blender properties that hold a procedural-animation rig description.

Per-bone joint data lives on 'bpy.types.Bone.rylogic_rig'. Chains and contacts live on 'bpy.types.Armature.rylogic_rig'.
Bones are referred to by name. See 'rig_desc.py' for how these properties map to the '.rig' schema.
"""

import math
import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, FloatProperty, FloatVectorProperty, IntProperty, PointerProperty, StringProperty
from bpy.types import PropertyGroup

# Enum items. The identifiers are the lower-case '.rig' names in upper case, so the export is a simple mapping.
CHAIN_ROLES = [
	("SPINE", "Spine", "Pelvis to chest. Fitted as a smooth curve"),
	("NECK", "Neck", "Neck and head. Aimed at a look target"),
	("LEG", "Leg", "Upper leg, lower leg, foot. Solved with two-bone IK"),
	("ARM", "Arm", "Upper arm, forearm, hand. Solved with two-bone IK"),
	("TAIL", "Tail", "A flexible chain solved with FABRIK"),
	("DIGIT", "Digit", "A finger or toe. Posed but not solved"),
]
SIDES = [
	("CENTRE", "Centre", "On the centre line"),
	("LEFT", "Left", "On the left side"),
	("RIGHT", "Right", "On the right side"),
]
CONTACT_KINDS = [
	("SOLE", "Sole", "The underside of a foot"),
	("TOE", "Toe", "The tip of a toe"),
	("PALM", "Palm", "The underside of a hand"),
	("FINGER", "Finger", "The tip of a finger"),
	("TAIL", "Tail", "A point on the tail"),
]
FAMILY_UPS = [
	("SURFACE", "Surface", "The body stands up along the ground's normal, e.g. a quadruped on a slope"),
	("GRAVITY", "Gravity", "The body stands up against gravity, e.g. an upright biped"),
]
FAMILY_MEDIUMS = [
	("GROUND", "Ground", "The family walks on the ground"),
	("WATER", "Water", "The family swims at the water's surface. Its cycles must not plant any feet; the limbs hang from the body in the cycle's pose"),
]


def _MinAngle(name: str, default_degrees: float, description: str) -> FloatProperty:
	"""A range minimum. It is never positive, so every range contains the rest pose."""
	return FloatProperty(name=name, subtype="ANGLE", min=-math.pi, max=0.0, default=math.radians(default_degrees), description=description)


def _MaxAngle(name: str, default_degrees: float, description: str) -> FloatProperty:
	"""A range maximum. It is never negative, so every range contains the rest pose."""
	return FloatProperty(name=name, subtype="ANGLE", min=0.0, max=math.pi, default=math.radians(default_degrees), description=description)


class RYLOGIC_PG_RigBone(PropertyGroup):
	"""The joint limit of one bone.

	The limit is the bone's rotation range from its rest pose, split into a swing and a twist (see 'JointLimit' in 'pr/animation/rig.h'). The twist is
	the rotation about the bone's own +Y axis. The swing moves the bone's +Y axis, and is measured about the bone's rest +X and +Z axes, so bone roll
	chooses the swing axes. A zero-width swing range on one axis makes a hinge about the other axis.
	"""

	use_limit: BoolProperty(name="Limit Rotation", default=False, description="Limit the bone's rotation from its rest pose. Without a limit the joint can rotate freely")
	swing_x_min: _MinAngle("Swing X Min", -45, "The most negative swing about the bone's rest +X axis")
	swing_x_max: _MaxAngle("Swing X Max", 45, "The most positive swing about the bone's rest +X axis")
	swing_z_min: _MinAngle("Swing Z Min", -45, "The most negative swing about the bone's rest +Z axis")
	swing_z_max: _MaxAngle("Swing Z Max", 45, "The most positive swing about the bone's rest +Z axis")
	twist_min: _MinAngle("Twist Min", -180, "The most negative twist about the bone's +Y axis")
	twist_max: _MaxAngle("Twist Max", 180, "The most positive twist about the bone's +Y axis")


def SelectBones(armature_obj: bpy.types.Object, names: list[str]) -> None:
	"""Select exactly the named bones of 'armature_obj', making the first one active. Uses edit bones in Edit Mode and pose bones otherwise."""
	# Edit Mode keeps its own bone selection, which is written back to the pose bones on leaving it
	armature = armature_obj.data
	wanted = set(names)
	if armature_obj.mode == "EDIT":
		for edit_bone in armature.edit_bones:
			edit_bone.select = edit_bone.select_head = edit_bone.select_tail = edit_bone.name in wanted
		if names and names[0] in armature.edit_bones:
			armature.edit_bones.active = armature.edit_bones[names[0]]
	else:
		for pose_bone in armature_obj.pose.bones:
			pose_bone.select = pose_bone.name in wanted
		if names and names[0] in armature.bones:
			armature.bones.active = armature.bones[names[0]]


def SelectedBoneNames(armature_obj: bpy.types.Object) -> list[str]:
	"""Return the names of the selected bones of 'armature_obj'. Uses edit bones in Edit Mode and pose bones otherwise."""
	if armature_obj.mode == "EDIT":
		return [edit_bone.name for edit_bone in armature_obj.data.edit_bones if edit_bone.select]

	return [pose_bone.name for pose_bone in armature_obj.pose.bones if pose_bone.select]


def _OnActiveChainChanged(rig: "RYLOGIC_PG_Rig", context: bpy.types.Context) -> None:
	"""Select the bones of the newly active chain, so that the chain is visible in the viewport."""
	# Only act on the armature whose list was clicked
	armature_obj = context.active_object
	if armature_obj is None or armature_obj.data != rig.id_data or not 0 <= rig.active_chain < len(rig.chains):
		return

	SelectBones(armature_obj, [ref.name for ref in rig.chains[rig.active_chain].bones])


def _OnPinLiveChanged(rig: "RYLOGIC_PG_Rig", context: bpy.types.Context) -> None:
	"""Pin or release the selected bones of the armature whose toggle changed."""
	# Imported here because the pin module uses this module's selection helpers
	from . import pin

	armature_obj = context.active_object
	if armature_obj is not None and armature_obj.data == rig.id_data:
		pin.Sync(armature_obj)


class RYLOGIC_PG_RigBoneRef(PropertyGroup):
	"""A reference to a bone by name. The name is the built-in 'name' property."""


class RYLOGIC_PG_RigChain(PropertyGroup):
	"""A run of parent-child bones that is solved as one unit."""

	role: EnumProperty(name="Role", items=CHAIN_ROLES, default="SPINE")
	side: EnumProperty(name="Side", items=SIDES, default="CENTRE")
	bones: CollectionProperty(name="Bones", type=RYLOGIC_PG_RigBoneRef, description="Bones from base to end. Each bone is the parent of the next")
	active_bone: IntProperty(name="Active Bone", default=0)
	pole: FloatVectorProperty(name="Pole", size=3, default=(0.0, -1.0, 0.0), description="Leg and arm only. The rest-pose armature-space direction that the middle joint bends towards")
	forward: FloatVectorProperty(name="Forward", size=3, default=(0.0, -1.0, 0.0), description="Neck only. The rest-pose armature-space direction that the last bone faces")


class RYLOGIC_PG_RigContact(PropertyGroup):
	"""A named point on a bone that can be placed on a surface. The name is the built-in 'name' property."""

	bone: StringProperty(name="Bone", description="The bone that the contact is attached to")
	kind: EnumProperty(name="Kind", items=CONTACT_KINDS, default="SOLE")
	offset: FloatVectorProperty(name="Offset", size=3, subtype="TRANSLATION", default=(0.0, 0.0, 0.0), description="The contact point in the bone's rest space")
	normal: FloatVectorProperty(name="Normal", size=3, default=(0.0, 0.0, -1.0), description="The outward surface direction in the bone's rest space")


class RYLOGIC_PG_GaitFamily(PropertyGroup):
	"""A posture: a group of movement cycles that share walking limbs. The name is the built-in 'name' property, which cycles refer to.

	The order of the families is their order in the '.gait', which is the runtime's order of preference when a family cannot be used on the ground.
	"""

	up: EnumProperty(name="Up", items=FAMILY_UPS, default="SURFACE", description="The direction that the body stands up along")
	max_slope: FloatProperty(name="Max Slope", subtype="ANGLE", min=math.radians(0.1), max=math.pi, default=math.pi, description="The steepest ground, from level, on which the family can be used. 180 degrees allows any ground")
	max_speed: FloatProperty(name="Max Speed", unit="VELOCITY", min=0.01, default=1.0, description="The fastest that the character should move in this family. It may exceed the fastest cycle's speed")
	medium: EnumProperty(name="Medium", items=FAMILY_MEDIUMS, default="GROUND", description="Whether the family walks on the ground or swims in water")
	wave_amplitude: FloatProperty(name="Wave Amplitude", min=0.0, default=0.0, description="The peak bend of the body wave at the head end, in radians per body length. The runtime bends the spine and tails from side to side in a wave that travels from head to tail. Zero for no wave")
	wave_wavelength: FloatProperty(name="Wave Wavelength", min=0.01, default=1.0, description="The length of one body wave, in body lengths (spine plus longest tail)")
	wave_growth: FloatProperty(name="Wave Growth", min=0.0, default=1.0, description="How much the bend grows from the head end to the tail tip, e.g. 2 for a tail tip that bends twice as much as the chest")

class RYLOGIC_PG_GaitCycle(PropertyGroup):
	"""An example movement cycle: a looping action that moves the root bone forward by one stride. The name is the built-in 'name' property."""

	action: PointerProperty(name="Action", type=bpy.types.Action, description="The cycle's action. Its frame range covers exactly one cycle")
	family: StringProperty(name="Family", default="quadruped", description="The name of the cycle's family (see Families), e.g. 'bipedal' or 'quadruped'")
	use_export: BoolProperty(name="Export", default=True, description="Include the cycle in the '.gait'. Turn off to keep an unfinished cycle without validating or exporting it")
	hip_height: FloatProperty(name="Hip Height", default=0.5, min=0.05, max=1.0, description="The height at which the legs carry the hip joints above the ground, as a fraction of the legs' reach, e.g. 0.5 for a crouching quadruped or 0.9 for an upright biped. The runtime sets the body's height from this, not from the action's poses")
	spine_sway: FloatProperty(name="Spine Sway", default=0.25, min=0.0, max=1.5, subtype='ANGLE', description="The angle by which the hips and shoulders turn with the walking limbs, so that each limb's side moves forward as its foot lands, e.g. 0.1 rad for an upright biped or 0.3 rad for a lizard. The runtime adds this to the action's own spine motion")
	weight_shift: FloatProperty(name="Weight Shift", default=0.0, min=0.0, max=1.0, subtype='FACTOR', description="How far the body moves sideways over each planted foot, as a fraction of the foot's distance from the midline, most strongly at the middle of its stance, e.g. 0.4 for an upright biped or 0 for a crawler. The runtime adds this to the action's own body motion")
	arm_swing: FloatProperty(name="Arm Swing", default=0.0, min=0.0, max=1.5, subtype='ANGLE', description="The angle by which each hanging arm swings forward and back about its shoulder, opposite to the walking leg on its side, e.g. 0.4 rad for an upright biped. Arms that walk do not swing")
	leg_swing_out: FloatProperty(name="Leg Swing Out", default=0.0, min=0.0, subtype='DISTANCE', description="The distance by which each walking leg's foot moves out, away from the body's centre line, at the middle of its swing, so that the raised leg swings around rather than straight up and down, e.g. 0 for an upright biped or 0.06 m for a lizard")
	arm_swing_out: FloatProperty(name="Arm Swing Out", default=0.0, min=0.0, subtype='DISTANCE', description="The distance by which each walking arm's hand moves out, away from the body's centre line, at the middle of its swing (see Leg Swing Out). Arms that do not walk ignore it")

class RYLOGIC_PG_Rig(PropertyGroup):
	"""The chains, contacts, families and movement cycles of an armature."""

	chains: CollectionProperty(name="Chains", type=RYLOGIC_PG_RigChain)
	active_chain: IntProperty(name="Active Chain", default=0, update=_OnActiveChainChanged, description="The chain shown below. Selecting a chain selects its bones")
	contacts: CollectionProperty(name="Contacts", type=RYLOGIC_PG_RigContact)
	active_contact: IntProperty(name="Active Contact", default=0)
	body_axis_from: StringProperty(name="Body Axis From", description="The bone whose head is the rear end of the body axis, e.g. the hips. The collision capsule's axis passes through both bone heads")
	body_axis_to: StringProperty(name="Body Axis To", description="The bone whose head is the front end of the body axis, e.g. the neck. The collision capsule's axis passes through both bone heads")
	show_overlay: BoolProperty(name="Show Overlay", default=True, description="Draw joint limits, poles, facing directions and contacts in the viewport")
	show_limits: BoolProperty(name="Limits", default=True, description="Draw each joint's swing (orange) and twist (yellow) limits. A bone outside its limit is drawn in red even when this is off")
	show_poles: BoolProperty(name="Poles", default=True, description="Draw the outward bend direction (magenta) of each leg and arm chain")
	show_facing: BoolProperty(name="Facing", default=True, description="Draw the forward direction (sky blue) of each neck chain")
	show_contacts: BoolProperty(name="Contacts", default=True, description="Draw each contact point and its surface normal (green)")
	rom_action: PointerProperty(name="Range of Motion", type=bpy.types.Action, description="An action whose keyframes pose the armature at the extremes of each joint's range of motion")
	rom_mirror: BoolProperty(name="Mirror Samples", default=True, description="Derive the same, mirrored, limits for left and right bones from the poses of either side. Turn off for asymmetric rigs")
	rom_padding: FloatProperty(name="Padding", subtype="ANGLE", min=0.0, max=math.radians(45), default=math.radians(5), description="Extra range added beyond the posed extremes when deriving limits")
	families: CollectionProperty(name="Families", type=RYLOGIC_PG_GaitFamily)
	active_family: IntProperty(name="Active Family", default=0)
	cycles: CollectionProperty(name="Movement Cycles", type=RYLOGIC_PG_GaitCycle)
	active_cycle: IntProperty(name="Active Cycle", default=0)
	pin_live: BoolProperty(name="Live Pin", default=False, update=lambda rig, context: _OnPinLiveChanged(rig, context), description="In Pose Mode, hold the selected chain bones still as the frame changes, bending the chains above them with IK. With Auto Key on, each frame change keys the solved pose")


# The active bone of each armature when last seen, so that only a change of active bone moves the contact selection
_LAST_ACTIVE_BONE: dict[str, str] = {}


@persistent
def _OnDepsgraphUpdate(scene, depsgraph) -> None:
	"""Make a chain and a contact that contain the newly active bone the active ones, so that selecting a bone shows its chain and contact."""
	# Selection changes are only visible as scene updates, so react only when the active armature's active bone differs from last time
	armature_obj = bpy.context.view_layer.objects.active
	if armature_obj is None or armature_obj.type != "ARMATURE":
		return

	armature = armature_obj.data
	bone = armature.edit_bones.active if armature_obj.mode == "EDIT" else armature.bones.active
	name = bone.name if bone is not None else ""
	if _LAST_ACTIVE_BONE.get(armature.name_full) == name:
		return

	# Keep the active chain if it already contains this bone. Set the raw value, because the update callback would replace the user's bone selection.
	_LAST_ACTIVE_BONE[armature.name_full] = name
	rig = armature.rylogic_rig
	if not (0 <= rig.active_chain < len(rig.chains) and any(ref.name == name for ref in rig.chains[rig.active_chain].bones)):
		index = next((i for i, chain in enumerate(rig.chains) if any(ref.name == name for ref in chain.bones)), None)
		if index is not None:
			rig["active_chain"] = index

	# Keep the active contact if it is already on this bone, as a bone can have several contacts
	if not (0 <= rig.active_contact < len(rig.contacts) and rig.contacts[rig.active_contact].bone == name):
		index = next((i for i, contact in enumerate(rig.contacts) if contact.bone == name), None)
		if index is not None:
			rig.active_contact = index


CLASSES = (
	RYLOGIC_PG_RigBone,
	RYLOGIC_PG_RigBoneRef,
	RYLOGIC_PG_RigChain,
	RYLOGIC_PG_RigContact,
	RYLOGIC_PG_GaitFamily,
	RYLOGIC_PG_GaitCycle,
	RYLOGIC_PG_Rig,
)


def Register() -> None:
	"""Register the property groups and attach them to bones and armatures."""
	for cls in CLASSES:
		bpy.utils.register_class(cls)

	bpy.types.Bone.rylogic_rig = PointerProperty(type=RYLOGIC_PG_RigBone)
	bpy.types.Armature.rylogic_rig = PointerProperty(type=RYLOGIC_PG_Rig)
	bpy.app.handlers.depsgraph_update_post.append(_OnDepsgraphUpdate)


def Unregister() -> None:
	"""Detach and unregister the property groups. Stored values remain in the .blend file."""
	bpy.app.handlers.depsgraph_update_post.remove(_OnDepsgraphUpdate)
	del bpy.types.Armature.rylogic_rig
	del bpy.types.Bone.rylogic_rig
	for cls in reversed(CLASSES):
		bpy.utils.unregister_class(cls)
