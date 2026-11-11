"""Properties-editor panels for the rig description."""

import math

import bpy
from bpy.types import Panel, UIList

from . import operators, pin


class RYLOGIC_UL_rig_chains(UIList):
	"""Lists an armature's chains."""

	def draw_item(self, context, layout, data, item, icon, active_data, active_property, index):
		# Summarise each chain by role, side and extent
		first = item.bones[0].name if item.bones else "-"
		last = item.bones[-1].name if item.bones else "-"
		layout.label(text=f"{item.role.title()} {item.side.title()}: {first} .. {last} ({len(item.bones)})", icon="LINKED")


class RYLOGIC_UL_rig_contacts(UIList):
	"""Lists an armature's contacts."""

	def draw_item(self, context, layout, data, item, icon, active_data, active_property, index):
		# Show the editable name with its kind and bone
		row = layout.row()
		row.prop(item, "name", text="", emboss=False, icon="SNAP_FACE")
		row.label(text=f"{item.kind.title()} on {item.bone}")


class RYLOGIC_UL_rig_families(UIList):
	"""Lists an armature's gait families."""

	def draw_item(self, context, layout, data, item, icon, active_data, active_property, index):
		# Show the editable name with its posture, or with its action for an action family
		row = layout.row()
		row.prop(item, "name", text="", emboss=False, icon="ARMATURE_DATA")
		if item.action_kind != "NONE":
			row.label(text=f"{item.action_kind.title()} from {item.action_base or '(no base)'}")
			return

		slope = "any slope" if item.max_slope >= math.pi else f"up to {math.degrees(item.max_slope):.0f}°"
		row.label(text=f"{item.medium.title()}, {item.up.title()} up, {slope}, {item.max_speed:.2f} m/s")


class RYLOGIC_UL_rig_punches(UIList):
	"""Lists an action family's punch sequence."""

	def draw_item(self, context, layout, data, item, icon, active_data, active_property, index):
		# Show each punch's hand, kind and duration in sequence order
		row = layout.row(align=True)
		row.label(text=f"{index + 1}.")
		row.prop(item, "hand", text="")
		row.prop(item, "kind", text="")
		row.prop(item, "duration", text="")


class RYLOGIC_UL_rig_cycles(UIList):
	"""Lists an armature's movement cycles."""

	def draw_item(self, context, layout, data, item, icon, active_data, active_property, index):
		# Show the export toggle and the editable name with its family and action. Unexported cycles are greyed out.
		row = layout.row()
		row.active = item.use_export
		row.prop(item, "use_export", text="")
		row.prop(item, "name", text="", emboss=False, icon="ANIM")
		row.label(text=f"{item.family}: {item.action.name if item.action else '(no action)'}")


class RYLOGIC_PT_rig(Panel):
	"""The armature's procedural-animation rig. Its sections are sub-panels, so each one can be collapsed and reordered."""

	bl_label = "Rylogic Rig"
	bl_space_type = "PROPERTIES"
	bl_region_type = "WINDOW"
	bl_context = "data"

	@classmethod
	def poll(cls, context):
		return operators.ActiveArmature(context) is not None

	def draw(self, context):
		# Tools that act on the whole rig
		layout = self.layout
		row = layout.row(align=True)
		row.operator("rylogic_rig.validate", icon="CHECKMARK")
		row.operator("rylogic_rig.validate_native", icon="CHECKMARK")
		layout.operator("rylogic_rig.export", icon="EXPORT")

		# The most recent validation result
		if operators.LAST_REPORT:
			box = layout.box()
			for line in operators.LAST_REPORT[:12]:
				box.label(text=line, icon="CHECKMARK" if line.startswith("Valid") else "ERROR")
			if len(operators.LAST_REPORT) > 12:
				box.label(text=f"... and {len(operators.LAST_REPORT) - 12} more (see the Info editor)")


class _RigSubPanel:
	"""Shared settings for the sections of the Rylogic Rig panel. Each section sets 'bl_parent_id' to the panel that contains it."""

	bl_space_type = "PROPERTIES"
	bl_region_type = "WINDOW"
	bl_context = "data"
	bl_parent_id = "RYLOGIC_PT_rig"

	@classmethod
	def poll(cls, context):
		return operators.ActiveArmature(context) is not None


class RYLOGIC_PT_rig_overlay(_RigSubPanel, Panel):
	"""Which parts of the rig the viewport overlay draws."""

	bl_label = "Overlay"

	def draw_header(self, context):
		# The whole overlay is switched on and off from the section's header
		self.layout.prop(context.active_object.data.rylogic_rig, "show_overlay", text="")

	def draw(self, context):
		# Visibility of each type of overlay, greyed out while the whole overlay is hidden
		rig = context.active_object.data.rylogic_rig
		row = self.layout.row(align=True)
		row.active = rig.show_overlay
		row.prop(rig, "show_limits", toggle=True)
		row.prop(rig, "show_poles", toggle=True)
		row.prop(rig, "show_facing", toggle=True)
		row.prop(rig, "show_contacts", toggle=True)


class RYLOGIC_PT_rig_skeleton(_RigSubPanel, Panel):
	"""The skeleton's structure: chains, the body axis and joint ranges of motion."""

	bl_label = "Skeleton"

	def draw(self, context):
		# The section's content is in its sub-panels
		pass


class RYLOGIC_PT_rig_chains(_RigSubPanel, Panel):
	"""The bone chains, with the active chain's details."""

	bl_label = "Chains"
	bl_parent_id = "RYLOGIC_PT_rig_skeleton"

	def draw(self, context):
		# The list of chains
		layout = self.layout
		rig = context.active_object.data.rylogic_rig
		row = layout.row()
		row.template_list("RYLOGIC_UL_rig_chains", "", rig, "chains", rig, "active_chain", rows=4)
		column = row.column(align=True)
		column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "CHAINS"
		column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "CHAINS"
		if not 0 <= rig.active_chain < len(rig.chains):
			return

		# The active chain, with the direction that suits its role
		chain = rig.chains[rig.active_chain]
		box = layout.box()
		box.prop(chain, "role")
		box.prop(chain, "side")
		match chain.role:
			case "LEG" | "ARM":
				box.prop(chain, "pole")
			case "NECK":
				box.prop(chain, "forward")
			case _:
				pass
		box.label(text="Bones: " + ", ".join(ref.name for ref in chain.bones))
		box.operator("rylogic_rig.chain_assign", icon="BONE_DATA")


class RYLOGIC_PT_rig_body_axis(_RigSubPanel, Panel):
	"""The body axis that the collision capsule follows."""

	bl_label = "Body Axis"
	bl_parent_id = "RYLOGIC_PT_rig_skeleton"

	def draw(self, context):
		# The bones that the axis runs between
		armature = context.active_object.data
		rig = armature.rylogic_rig
		column = self.layout.column()
		column.prop_search(rig, "body_axis_from", armature, "bones", text="From")
		column.prop_search(rig, "body_axis_to", armature, "bones", text="To")


class RYLOGIC_PT_rig_range_of_motion(_RigSubPanel, Panel):
	"""Capture extreme poses into an action, then derive the joint limits from them."""

	bl_label = "Range of Motion"
	bl_parent_id = "RYLOGIC_PT_rig_skeleton"
	bl_options = {"DEFAULT_CLOSED"}

	def draw(self, context):
		# The action that holds the poses, and the tools that use it
		layout = self.layout
		rig = context.active_object.data.rylogic_rig
		layout.prop(rig, "rom_action")
		row = layout.row(align=True)
		row.prop(rig, "rom_padding")
		row.prop(rig, "rom_mirror", toggle=True, icon="MOD_MIRROR")
		row = layout.row(align=True)
		row.operator("rylogic_rig.capture_pose", icon="KEYFRAME")
		row.operator("rylogic_rig.derive_limits", icon="CON_ROTLIMIT")
		row.operator("rylogic_rig.symmetrize_limits", icon="MOD_MIRROR")
		layout.operator("rylogic_rig.copy_ik_limits", icon="CON_KINEMATIC")


class RYLOGIC_PT_rig_contacts(_RigSubPanel, Panel):
	"""The contact points, with the active contact's details."""

	bl_label = "Contacts"

	def draw(self, context):
		# The list of contacts
		layout = self.layout
		armature = context.active_object.data
		rig = armature.rylogic_rig
		row = layout.row()
		row.template_list("RYLOGIC_UL_rig_contacts", "", rig, "contacts", rig, "active_contact", rows=4)
		column = row.column(align=True)
		column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "CONTACTS"
		column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "CONTACTS"
		if not 0 <= rig.active_contact < len(rig.contacts):
			return

		# The active contact. Placing it on the mesh sets its bone, offset and normal, which can then be fine-tuned by hand.
		contact = rig.contacts[rig.active_contact]
		box = layout.box()
		box.operator("rylogic_rig.contact_pick", icon="EYEDROPPER")
		box.prop(contact, "kind")
		box.prop_search(contact, "bone", armature, "bones")
		row = box.row()
		row.column().prop(contact, "offset")
		row.column().prop(contact, "normal")


class RYLOGIC_PT_rig_locomotion(_RigSubPanel, Panel):
	"""The postures and example movement cycles that guide the runtime's procedural locomotion."""

	bl_label = "Locomotion"

	def draw(self, context):
		# The section's content is in its sub-panels
		pass


class RYLOGIC_PT_rig_families(_RigSubPanel, Panel):
	"""The postures that cycles belong to, in the runtime's order of preference."""

	bl_label = "Families"
	bl_parent_id = "RYLOGIC_PT_rig_locomotion"

	def draw(self, context):
		# The list of families
		layout = self.layout
		rig = context.active_object.data.rylogic_rig
		row = layout.row()
		row.template_list("RYLOGIC_UL_rig_families", "", rig, "families", rig, "active_family", rows=2)
		column = row.column(align=True)
		column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "FAMILIES"
		column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "FAMILIES"
		if not 0 <= rig.active_family < len(rig.families):
			return

		# The active family. An action family uses only its action settings.
		family = rig.families[rig.active_family]
		box = layout.box()
		box.prop(family, "action_kind")
		if family.action_kind != "NONE":
			box.prop_search(family, "action_base", rig, "families")
			match family.action_kind:
				case "COLLAPSE":
					box.prop(family, "action_duration")
				case "PUNCHES":
					row = box.row()
					row.template_list("RYLOGIC_UL_rig_punches", "", family, "punches", family, "active_punch", rows=3)
					column = row.column(align=True)
					column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "PUNCHES"
					column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "PUNCHES"
				case _:
					raise ValueError(f"Unknown action kind '{family.action_kind}'")
			return

		box.prop(family, "up")
		box.prop(family, "max_slope")
		box.prop(family, "max_speed")
		box.prop(family, "medium")
		column = box.column(align=True)
		column.prop(family, "wave_amplitude")
		column.prop(family, "wave_wavelength")
		column.prop(family, "wave_growth")
		box.prop(family, "mirror_name")


class RYLOGIC_PT_rig_cycles(_RigSubPanel, Panel):
	"""Example actions that guide the runtime's procedural locomotion, exported as a '.gait', and the tools for authoring them."""

	bl_label = "Movement Cycles"
	bl_parent_id = "RYLOGIC_PT_rig_locomotion"

	def draw(self, context):
		# The list of cycles
		layout = self.layout
		armature = context.active_object.data
		rig = armature.rylogic_rig
		row = layout.row()
		row.template_list("RYLOGIC_UL_rig_cycles", "", rig, "cycles", rig, "active_cycle", rows=3)
		column = row.column(align=True)
		column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "CYCLES"
		column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "CYCLES"
		column.operator_menu_enum("rylogic_rig.gait_template", "template", icon="PRESET_NEW", text="")

		# The active cycle
		if 0 <= rig.active_cycle < len(rig.cycles):
			cycle = rig.cycles[rig.active_cycle]
			box = layout.box()
			box.prop_search(cycle, "family", rig, "families")
			box.prop(cycle, "action")
			box.prop(cycle, "hip_height")
			box.prop(cycle, "spine_sway")
			box.prop(cycle, "weight_shift")
			box.prop(cycle, "arm_swing")
			box.prop(cycle, "leg_swing_out")
			box.prop(cycle, "arm_swing_out")

		# Hold planted feet still while the body moves over them
		layout.prop(rig, "pin_live", icon="PINNED", toggle=True)
		pinned = pin.PinnedBones(context.active_object) if rig.pin_live else []
		if pinned:
			layout.label(text=f"Pinned: {', '.join(pinned)}")

		row = layout.row(align=True)
		row.operator("rylogic_rig.check_limits", icon="CON_ROTLIMIT")
		row.operator("rylogic_rig.gait_validate", icon="CHECKMARK")
		row.operator("rylogic_rig.gait_export", icon="EXPORT")


class RYLOGIC_PT_rig_bone(Panel):
	"""The active bone's joint limit."""

	bl_label = "Rylogic Rig Joint"
	bl_space_type = "PROPERTIES"
	bl_region_type = "WINDOW"
	bl_context = "bone"

	@classmethod
	def poll(cls, context):
		return context.bone is not None

	def draw(self, context):
		# Show each range as a min/max pair, only when the limit is used
		layout = self.layout
		joint = context.bone.rylogic_rig
		layout.prop(joint, "use_limit")
		column = layout.column(align=True)
		column.active = joint.use_limit
		for label, low, high in (("Swing X", "swing_x_min", "swing_x_max"), ("Swing Z", "swing_z_min", "swing_z_max"), ("Twist", "twist_min", "twist_max")):
			row = column.row(align=True)
			row.label(text=label)
			row.prop(joint, low, text="Min")
			row.prop(joint, high, text="Max")


CLASSES = (
	RYLOGIC_UL_rig_chains,
	RYLOGIC_UL_rig_contacts,
	RYLOGIC_UL_rig_families,
	RYLOGIC_UL_rig_punches,
	RYLOGIC_UL_rig_cycles,
	RYLOGIC_PT_rig,
	RYLOGIC_PT_rig_overlay,
	RYLOGIC_PT_rig_skeleton,
	RYLOGIC_PT_rig_chains,
	RYLOGIC_PT_rig_body_axis,
	RYLOGIC_PT_rig_range_of_motion,
	RYLOGIC_PT_rig_contacts,
	RYLOGIC_PT_rig_locomotion,
	RYLOGIC_PT_rig_families,
	RYLOGIC_PT_rig_cycles,
	RYLOGIC_PT_rig_bone,
)


def Register() -> None:
	"""Register the panels and lists."""
	for cls in CLASSES:
		bpy.utils.register_class(cls)


def Unregister() -> None:
	"""Unregister the panels and lists."""
	for cls in reversed(CLASSES):
		bpy.utils.unregister_class(cls)
