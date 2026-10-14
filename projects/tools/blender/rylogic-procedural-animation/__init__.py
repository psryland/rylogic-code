"""Rylogic Procedural Animation: author the rig description used by the 'pr::animation' procedural animation system.

A rig description names the solvable chains of an armature (spine, neck, legs, arms, tail, digits), each bone's joint limit, and the contact points that
are placed on surfaces. It is written as a '.rig' JSON file next to the skinned model, and cooked by the p3d tool's '-Rig' command, which is the canonical
validation. The schema is owned by 'pr/animation/rig_json.h' in the Rylogic code library.

Joint limits bound each bone's rotation from its rest pose as a swing of the bone's direction and a twist about it (see 'limits.py'). The swing range is
set separately about the bone's rest +X and +Z axes, so roll the bone to align those axes with the joint. A zero-width range on one swing axis makes a
hinge about the other. Limits can be set by hand in the Bone properties, or derived from poses:
	1. In the Rylogic Rig panel, pose the armature at the limit of its range of motion and press 'Capture Pose'. This keys every bone's rotation into
	   the range-of-motion action, creating it if needed. Repeat for each limit, e.g. legs fully forward, back, out and twisted. Posing one rotation
	   axis at a time for all limbs (R X X, R Z Z, R Y Y rotate about a bone's local axes) is easier than posing whole extreme poses. The order of the
	   keyframes does not matter.
	2. Press 'Derive Limits'. Each bone that moves in the captured poses gets the smallest limit that contains them all, widened by 'Padding'.
	   Bones that never move keep their limits. With 'Mirror Samples' on, the poses of either side also count, mirrored, for the other side, so a
	   single capture can pose the left leg forward and the right leg back. Turn it off for asymmetric rigs.
	3. Review the limits in the viewport overlay and adjust them in the Bone properties. 'Symmetrize Limits' copies the selected bones' limits,
	   mirrored, to their left/right partners.
	4. Press 'Copy to IK Limits' to set Blender's own IK limits from the joint limits. Then dragging a foot or tail tip with Auto IK (Pose Options in
	   the Tool sidebar; Page Up/Down sets the chain length while dragging) keeps roughly within range. Blender limits each axis separately, so this is
	   an approximation; press it again after changing limits.
Mirroring needs left and right bones whose names differ only by side and whose rest transforms are mirror images, as Blender's Symmetrize makes them.

Contacts are points on a bone, with an outward surface normal, that are placed on surfaces, e.g. the soles of the feet. Point the normal out of the
part of the body that touches the surface. Add a contact in the Rylogic Rig panel, then press 'Pick on Surface' and click the skinned mesh. The
contact moves to the clicked point on the bone that most deforms that face, with the face's normal. This works on a posed mesh. Esc or right-click
cancels.

Selecting a chain in the panel selects its bones. Selecting a bone makes a chain and a contact that use it active, unless the active ones already do.

The body axis is the line from the head of its 'From' bone (e.g. the hips) to the head of its 'To' bone (e.g. the neck). The runtime's collision
capsule lies along this line as the body bends and rears up. Both bones are required.

The viewport overlay draws swing limits (orange), twist limits (yellow), limb poles (magenta), the neck's facing direction (sky blue) and contacts
(green). Limb poles and facing directions are fixed rest-pose model-space directions. The toggles under 'Show Overlay' hide each type. In Pose Mode
with bones selected, only the overlays of the selected bones are drawn. A bone whose current pose is outside its joint limit is drawn in red, with its
limit, whatever the selection and the 'Limits' toggle, so that animating past a limit is noticed while posing or playing an action. 'Check Limits'
plays the active movement cycle's action and lists each bone that leaves its limit, by how much, and on which frames. Both check the evaluated pose,
so constraints and IK count.

Movement cycles are example actions that guide the runtime's procedural locomotion. They are not played back: the runtime measures stride, speed,
footfall order and timing, and the body's sway from them, then places the feet itself. Each cycle belongs to a family (e.g. 'bipedal', 'quadruped',
'climb'), and game logic chooses the family. Within a family, the runtime blends the cycles by speed. To author a cycle:
	1. Under 'Movement Cycles', add a cycle from a template (the preset button), or add an empty cycle and assign an action. A template keys a
	   standard footfall pattern from the rig's leg chains and sole/palm contacts, as a starting point.
	2. Refine the action by hand. Its frame range is one loop: the pose at the last frame is the pose at the first frame, moved forward by one
	   stride. The root bone travels forward at a steady speed, and each foot is planted, still on the ground, for part of the loop. Export
	   rejects a cycle whose last frame does not match its first frame within 1° and 1% of the stride, and names the worst bones.
	   To plant feet, key the root's travel first, then turn on 'Live Pin' and Auto Key, and select the planted foot and hand bones. Each selected
	   chain bone holds still while the frame changes, the bones of its chain above it bend with IK to reach it, and each frame change keys the
	   result. Step through the frames one at a time, changing the selection as feet lift and land. 'Copy to IK Limits' first keeps the bending
	   within the joint limits.
	3. 'Validate Gait' measures every exported cycle and lists the footfalls it found. 'Export .gait' writes them to a '.gait' JSON file next to
	   the '.rig'. The schema is owned by 'pr/animation/gait.h'. Untick a cycle's checkbox in the list to keep it in the .blend but leave it out of
	   validation and export, e.g. while it is unfinished.
The walking limbs are the LEG chains ending on a SOLE contact and the ARM chains ending on a PALM contact.

Scripted use, e.g. from an asset exporter:
	addon = importlib.import_module("rylogic-procedural-animation")
	addon.register()  # the rig properties are read through registered property groups
	addon.WriteRig(armature_obj, rig_path)
	addon.WriteGait(armature_obj, gait_path)
"""

bl_info = {
	"name": "Rylogic Procedural Animation",
	"author": "Paul Ryland",
	"version": (2, 1, 0),
	"blender": (4, 2, 0),
	"description": "Author and export rig and gait descriptions for the Rylogic procedural animation system",
	"location": "Properties > Armature > Rylogic Rig (including Movement Cycles), and Properties > Bone > Rylogic Rig Joint",
	"category": "Animation",
}

import bpy

# Blender's 'Reload Scripts' re-runs this module but keeps its submodules cached, so reload them too for edits to take effect without a restart.
# Dependencies come before the modules that import names from them.
if "operators" in locals():
	import importlib
	for _name in ("limits", "rig_desc", "gait_desc", "gait_template", "validate", "props", "pin", "operators", "ui", "draw"):
		# A module added since the last load has not been imported yet, and is imported below
		if _name in locals():
			importlib.reload(locals()[_name])

from . import draw, operators, pin, props, ui
from .gait_desc import BuildGaitDesc, ValidateGaitDesc, WriteGaitDesc
from .operators import WriteGait, WriteRig
from .rig_desc import BuildRigDesc, WriteRigDesc
from .validate import ValidateRigDesc

__all__ = ["BuildGaitDesc", "BuildRigDesc", "ValidateGaitDesc", "ValidateRigDesc", "WriteGait", "WriteGaitDesc", "WriteRig", "WriteRigDesc", "register", "unregister"]


def register() -> None:
	"""Register the add-on's properties, operators, panels and viewport overlay."""
	# The overlay needs a viewport, so it is skipped when Blender runs without a user interface
	props.Register()
	pin.Register()
	operators.Register()
	ui.Register()
	if not bpy.app.background:
		draw.Register()


def unregister() -> None:
	"""Unregister everything that 'register' added."""
	draw.Unregister()
	ui.Unregister()
	operators.Unregister()
	pin.Unregister()
	props.Unregister()
