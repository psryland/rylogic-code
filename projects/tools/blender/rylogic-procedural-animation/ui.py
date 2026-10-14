"""Properties-editor panels for the rig description."""

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
	"""The armature's procedural-animation rig: chains, contacts and tools."""

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
		armature = context.active_object.data
		rig = armature.rylogic_rig
		row = layout.row(align=True)
		row.operator("rylogic_rig.validate", icon="CHECKMARK")
		row.operator("rylogic_rig.validate_native", icon="CHECKMARK")
		row = layout.row(align=True)
		row.operator("rylogic_rig.export", icon="EXPORT")
		row.prop(rig, "show_overlay", toggle=True, icon="OVERLAY")

		# Visibility of each type of overlay, greyed out while the whole overlay is hidden
		row = layout.row(align=True)
		row.active = rig.show_overlay
		row.prop(rig, "show_limits", toggle=True)
		row.prop(rig, "show_poles", toggle=True)
		row.prop(rig, "show_facing", toggle=True)
		row.prop(rig, "show_contacts", toggle=True)

		# The most recent validation result
		if operators.LAST_REPORT:
			box = layout.box()
			for line in operators.LAST_REPORT[:12]:
				box.label(text=line, icon="CHECKMARK" if line.startswith("Valid") else "ERROR")
			if len(operators.LAST_REPORT) > 12:
				box.label(text=f"... and {len(operators.LAST_REPORT) - 12} more (see the Info editor)")

		# Range of motion: capture extreme poses into an action, then derive the joint limits from them
		layout.label(text="Range of Motion")
		box = layout.box()
		box.prop(rig, "rom_action")
		row = box.row(align=True)
		row.prop(rig, "rom_padding")
		row.prop(rig, "rom_mirror", toggle=True, icon="MOD_MIRROR")
		row = box.row(align=True)
		row.operator("rylogic_rig.capture_pose", icon="KEYFRAME")
		row.operator("rylogic_rig.derive_limits", icon="CON_ROTLIMIT")
		row.operator("rylogic_rig.symmetrize_limits", icon="MOD_MIRROR")
		box.operator("rylogic_rig.copy_ik_limits", icon="CON_KINEMATIC")

		# Chains, with the active chain's details
		layout.label(text="Chains")
		row = layout.row()
		row.template_list("RYLOGIC_UL_rig_chains", "", rig, "chains", rig, "active_chain", rows=4)
		column = row.column(align=True)
		column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "CHAINS"
		column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "CHAINS"
		if 0 <= rig.active_chain < len(rig.chains):
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

		# Contacts, with the active contact's details
		layout.label(text="Contacts")
		row = layout.row()
		row.template_list("RYLOGIC_UL_rig_contacts", "", rig, "contacts", rig, "active_contact", rows=4)
		column = row.column(align=True)
		column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "CONTACTS"
		column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "CONTACTS"
		if 0 <= rig.active_contact < len(rig.contacts):
			contact = rig.contacts[rig.active_contact]
			box = layout.box()
			box.operator("rylogic_rig.contact_pick", icon="EYEDROPPER")
			box.prop_search(contact, "bone", armature, "bones")
			box.prop(contact, "kind")

		# The body axis that the collision capsule follows
		layout.label(text="Body Axis")
		box = layout.box()
		box.prop_search(rig, "body_axis_from", armature, "bones", text="From")
		box.prop_search(rig, "body_axis_to", armature, "bones", text="To")

		# Movement cycles: example actions that guide the runtime's procedural locomotion, exported as a '.gait'
		layout.label(text="Movement Cycles")
		row = layout.row()
		row.template_list("RYLOGIC_UL_rig_cycles", "", rig, "cycles", rig, "active_cycle", rows=3)
		column = row.column(align=True)
		column.operator("rylogic_rig.item_add", icon="ADD", text="").collection = "CYCLES"
		column.operator("rylogic_rig.item_remove", icon="REMOVE", text="").collection = "CYCLES"
		column.operator_menu_enum("rylogic_rig.gait_template", "template", icon="PRESET_NEW", text="")
		if 0 <= rig.active_cycle < len(rig.cycles):
			cycle = rig.cycles[rig.active_cycle]
			box = layout.box()
			box.prop(cycle, "family")
			box.prop(cycle, "action")
			box.prop(cycle, "hip_height")

		# Hold planted feet still while the body moves over them
		layout.prop(rig, "pin_live", icon="PINNED", toggle=True)
		pinned = pin.PinnedBones(context.active_object) if rig.pin_live and context.active_object.data == armature else []
		if pinned:
			layout.label(text=f"Pinned: {', '.join(pinned)}")

		row = layout.row(align=True)
		row.operator("rylogic_rig.check_limits", icon="CON_ROTLIMIT")
		row.operator("rylogic_rig.gait_validate", icon="CHECKMARK")
		row.operator("rylogic_rig.gait_export", icon="EXPORT")


class RYLOGIC_PT_rig_bone(Panel):
	"""The active bone's joint limit and stiffness."""

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

		layout.prop(joint, "stiffness")


CLASSES = (
	RYLOGIC_UL_rig_chains,
	RYLOGIC_UL_rig_contacts,
	RYLOGIC_UL_rig_cycles,
	RYLOGIC_PT_rig,
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
