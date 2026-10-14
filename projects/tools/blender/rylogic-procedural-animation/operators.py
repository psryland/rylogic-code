"""Operators for authoring, checking and exporting a rig description."""

import math
import subprocess
import tempfile
from pathlib import Path

import bpy
from bpy.props import EnumProperty, StringProperty
from bpy.types import Operator
from bpy_extras import view3d_utils
from bpy_extras.io_utils import ExportHelper
from mathutils import Vector
from mathutils.interpolate import poly_3d_calc

from . import gait_desc, gait_template, limits, props, rig_desc, validate

# The result of the most recent validation, shown in the Rig panel
LAST_REPORT: list[str] = []


def ActiveArmature(context: bpy.types.Context) -> bpy.types.Object | None:
	"""Return the active object if it is an armature."""
	obj = context.active_object
	return obj if obj is not None and obj.type == "ARMATURE" else None


def WriteRig(armature_obj: bpy.types.Object, path: Path) -> None:
	"""Build, check and write the '.rig' description of 'armature_obj'. Raises ValueError listing every problem if the description is invalid."""
	desc = rig_desc.BuildRigDesc(armature_obj)
	errors = validate.ValidateRigDesc(desc)
	if errors:
		raise ValueError(f"Rig '{armature_obj.name}' is invalid:\n  " + "\n  ".join(errors))

	rig_desc.WriteRigDesc(desc, path)


def WriteGait(armature_obj: bpy.types.Object, path: Path, context: bpy.types.Context | None = None) -> None:
	"""Measure, check and write the '.gait' description of the armature's movement cycles. Raises ValueError listing every problem if it is invalid."""
	desc = gait_desc.BuildGaitDesc(armature_obj, context or bpy.context)
	errors = gait_desc.ValidateGaitDesc(desc)
	if errors:
		raise ValueError(f"Gait of '{armature_obj.name}' is invalid:\n  " + "\n  ".join(errors))

	gait_desc.WriteGaitDesc(desc, path)


def _SetReport(lines: list[str]) -> None:
	"""Replace the report shown in the Rig panel."""
	LAST_REPORT.clear()
	LAST_REPORT.extend(lines)


class _ArmatureOperator(Operator):
	"""Base class for operators that act on the active armature."""

	@classmethod
	def poll(cls, context):
		return ActiveArmature(context) is not None


class RYLOGIC_OT_rig_validate(_ArmatureOperator):
	"""Check the rig description for the rules that the runtime relies on"""

	bl_idname = "rylogic_rig.validate"
	bl_label = "Validate"

	def execute(self, context):
		# Report each problem, or that the rig is valid
		try:
			errors = validate.ValidateRigDesc(rig_desc.BuildRigDesc(ActiveArmature(context)))
		except ValueError as error:
			errors = [str(error)]

		_SetReport(errors or ["Valid"])
		for error in errors:
			self.report({"WARNING"}, error)

		self.report({"ERROR"} if errors else {"INFO"}, f"{len(errors)} problems" if errors else "The rig is valid")
		return {"FINISHED"}


class RYLOGIC_OT_rig_validate_native(_ArmatureOperator):
	"""Export the armature's skinned meshes and rig to a temporary folder and check them with the p3d tool's canonical '-Rig' command"""

	bl_idname = "rylogic_rig.validate_native"
	bl_label = "Validate (native)"

	def execute(self, context):
		# The p3d tool is configured in the add-on preferences
		p3d_path = Path(context.preferences.addons[__package__].preferences.p3d_path)
		if not p3d_path.is_file():
			self.report({"ERROR"}, "Set the p3d tool path in the add-on preferences")
			return {"CANCELLED"}

		armature_obj = ActiveArmature(context)
		with tempfile.TemporaryDirectory(prefix="rylogic_rig_") as folder:
			# Export the armature and the meshes it deforms as a skinned model
			folder = Path(folder)
			try:
				_ExportSkinnedGlb(context, armature_obj, folder / "model.glb")
				rig_desc.WriteRigDesc(rig_desc.BuildRigDesc(armature_obj), folder / "in.rig")
			except (ValueError, RuntimeError) as error:
				self.report({"ERROR"}, str(error))
				return {"CANCELLED"}

			# Cook the rig against the model's skeleton
			result = subprocess.run(
				[str(p3d_path), "-fi", str(folder / "model.glb"), "-Rig", str(folder / "in.rig"), str(folder / "out.rig"), "-fo", str(folder / "model.p3d")],
				capture_output=True, text=True,
			)

		# Report the tool's output
		output = [line for line in (result.stdout + result.stderr).splitlines() if line.strip()]
		_SetReport(["Valid (native)"] if result.returncode == 0 else output or [f"p3d failed with exit code {result.returncode}"])
		for line in output if result.returncode != 0 else []:
			self.report({"WARNING"}, line)

		self.report({"INFO"} if result.returncode == 0 else {"ERROR"}, "The rig is valid (native)" if result.returncode == 0 else "Native validation failed")
		return {"FINISHED"}


class RYLOGIC_OT_rig_export(_ArmatureOperator, ExportHelper):
	"""Write the rig description to a '.rig' file"""

	bl_idname = "rylogic_rig.export"
	bl_label = "Export .rig"

	filename_ext = ".rig"
	filter_glob: StringProperty(default="*.rig", options={"HIDDEN"})

	def execute(self, context):
		# Refuse to write an invalid rig
		try:
			WriteRig(ActiveArmature(context), Path(self.filepath))
		except ValueError as error:
			self.report({"ERROR"}, str(error))
			return {"CANCELLED"}

		self.report({"INFO"}, f"Wrote {self.filepath}")
		return {"FINISHED"}


class RYLOGIC_OT_rig_capture_pose(_ArmatureOperator):
	"""Key the rotation of every bone into the range-of-motion action, as a new pose after the existing ones. Creates the action if there is none"""

	bl_idname = "rylogic_rig.capture_pose"
	bl_label = "Capture Pose"
	bl_options = {"REGISTER", "UNDO"}

	def execute(self, context):
		# Keys are inserted into the armature's active action, so make the range-of-motion action active
		armature_obj = ActiveArmature(context)
		rig = armature_obj.data.rylogic_rig
		if rig.rom_action is None:
			rig.rom_action = bpy.data.actions.new(f"{armature_obj.name}.RangeOfMotion")

		animation_data = armature_obj.animation_data or armature_obj.animation_data_create()
		if animation_data.action != rig.rom_action:
			animation_data.action = rig.rom_action

		# Key every bone, so that no bone's sample is interpolated from other poses
		frames, _ = limits.RangeOfMotionFrames(rig.rom_action)
		frame = int(frames[-1]) + 1 if frames else 1
		for pose_bone in armature_obj.pose.bones:
			pose_bone.keyframe_insert(gait_template.RotationPath(pose_bone), frame=frame, group=pose_bone.name)

		context.scene.frame_set(frame)
		self.report({"INFO"}, f"Captured pose {len(frames) + 1} at frame {frame} in '{rig.rom_action.name}'")
		return {"FINISHED"}


class RYLOGIC_OT_rig_derive_limits(_ArmatureOperator):
	"""Set the limit of each bone that moves in the range-of-motion action to contain every keyframed pose, plus the padding. Other bones are unchanged"""

	bl_idname = "rylogic_rig.derive_limits"
	bl_label = "Derive Limits"
	bl_options = {"REGISTER", "UNDO"}

	def execute(self, context):
		# The poses come from the armature's range-of-motion action
		armature_obj = ActiveArmature(context)
		rig = armature_obj.data.rylogic_rig
		if rig.rom_action is None:
			self.report({"ERROR"}, "Choose a range-of-motion action first")
			return {"CANCELLED"}

		changed, poses, unmatched = limits.DeriveLimits(armature_obj, rig.rom_action, rig.rom_padding, rig.rom_mirror)
		_ReportUnmatched(self, unmatched)
		self.report({"INFO"}, f"Derived limits for {changed} bones from {poses} poses")
		return {"FINISHED"}


class RYLOGIC_OT_rig_copy_ik_limits(_ArmatureOperator):
	"""Set Blender's IK limits of every bone from its joint limit, so that dragging with Auto IK or IK constraints stays roughly within range. Blender limits each axis separately, so this is an approximation"""

	bl_idname = "rylogic_rig.copy_ik_limits"
	bl_label = "Copy to IK Limits"
	bl_options = {"REGISTER", "UNDO"}

	def execute(self, context):
		# Overwrites any IK limits set by hand
		count = limits.CopyToIkLimits(ActiveArmature(context))
		self.report({"INFO"}, f"Set the IK limits of {count} bones")
		return {"FINISHED"}


class RYLOGIC_OT_rig_check_limits(_ArmatureOperator):
	"""Play the active movement cycle's action and list each bone that leaves its joint limit, with the frames where it does"""

	bl_idname = "rylogic_rig.check_limits"
	bl_label = "Check Limits"

	@classmethod
	def poll(cls, context):
		# Needs an active cycle with an action
		armature_obj = ActiveArmature(context)
		if armature_obj is None:
			return False

		rig = armature_obj.data.rylogic_rig
		return 0 <= rig.active_cycle < len(rig.cycles) and rig.cycles[rig.active_cycle].action is not None

	def execute(self, context):
		# Report each bone outside its limit, worst first, in the Rig panel and the Info editor
		armature_obj = ActiveArmature(context)
		rig = armature_obj.data.rylogic_rig
		action = rig.cycles[rig.active_cycle].action
		violations = limits.CheckAction(armature_obj, context.scene, action)
		lines = [
			f"{v.bone}: {math.degrees(v.worst_excess):.1f}° over at frame {v.worst_frame} (frames {limits.FrameRanges(v.frames)})"
			for v in violations
		]
		_SetReport(lines or [f"Valid: '{action.name}' stays within the joint limits"])
		for line in lines:
			self.report({"WARNING"}, line)

		self.report({"WARNING"} if lines else {"INFO"}, f"{len(lines)} bones leave their limits in '{action.name}'" if lines else f"'{action.name}' stays within the joint limits")
		return {"FINISHED"}


class RYLOGIC_OT_rig_symmetrize_limits(_ArmatureOperator):
	"""Copy the joint limit and stiffness of each selected bone, mirrored, to its left/right partner"""

	bl_idname = "rylogic_rig.symmetrize_limits"
	bl_label = "Symmetrize Limits"
	bl_options = {"REGISTER", "UNDO"}

	def execute(self, context):
		# A pair with both bones selected has no clear source, so it is skipped
		armature_obj = ActiveArmature(context)
		bones = armature_obj.data.bones
		selected = set(props.SelectedBoneNames(armature_obj))
		partners, unmatched = limits.MirrorPartners(armature_obj.data)
		copied, ambiguous = 0, []
		for name in sorted(selected):
			if name not in partners:
				continue
			if partners[name] in selected:
				ambiguous.append(name)
				continue

			limits.MirrorJoint(bones[name].rylogic_rig, bones[partners[name]].rylogic_rig)
			copied += 1

		# Explain anything that was not copied
		_ReportUnmatched(self, [name for name in unmatched if name in selected])
		if ambiguous:
			self.report({"WARNING"}, f"Skipped {len(ambiguous)} bones whose mirror partner is also selected: {', '.join(ambiguous)}")

		self.report({"INFO"}, f"Mirrored the limits of {copied} bones")
		return {"FINISHED"}


class RYLOGIC_OT_rig_contact_pick(_ArmatureOperator):
	"""Click on the armature's mesh to place the active contact there, facing out of the surface, on the bone that most deforms that spot"""

	bl_idname = "rylogic_rig.contact_pick"
	bl_label = "Pick on Surface"
	bl_options = {"REGISTER", "UNDO"}

	@classmethod
	def poll(cls, context):
		armature_obj = ActiveArmature(context)
		return armature_obj is not None and 0 <= armature_obj.data.rylogic_rig.active_contact < len(armature_obj.data.rylogic_rig.contacts)

	def invoke(self, context, event):
		# Wait for a click anywhere in the window, because the button lives in the Properties editor rather than the 3D view
		self.armature_name = ActiveArmature(context).name
		context.window.cursor_modal_set("EYEDROPPER")
		context.workspace.status_text_set("Click the mesh where the contact touches the ground.  Esc / Right-click: cancel")
		context.window_manager.modal_handler_add(self)
		return {"RUNNING_MODAL"}

	def modal(self, context, event):
		# Leave view navigation working while picking
		if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
			self._End(context)
			return {"CANCELLED"}
		if event.type != "LEFTMOUSE" or event.value != "PRESS":
			return {"PASS_THROUGH"}

		# A click away from a 3D view, or off the mesh, leaves the pick running so the user can try again
		armature_obj = bpy.data.objects.get(self.armature_name)
		ray = _MouseRay(context, event)
		if armature_obj is None or ray is None:
			return {"RUNNING_MODAL"}

		depsgraph = context.evaluated_depsgraph_get()
		hit, location, _, face, mesh_obj, _ = context.scene.ray_cast(depsgraph, *ray)
		if not hit or not _Deforms(mesh_obj, armature_obj):
			self.report({"WARNING"}, f"Click on a mesh deformed by '{armature_obj.name}'")
			return {"RUNNING_MODAL"}

		try:
			bone, offset, direction = ContactFromSurface(armature_obj, mesh_obj.evaluated_get(depsgraph), location, face)
		except ValueError as error:
			self.report({"WARNING"}, str(error))
			return {"RUNNING_MODAL"}

		# Place the active contact
		rig = armature_obj.data.rylogic_rig
		contact = rig.contacts[rig.active_contact]
		contact.bone, contact.offset, contact.normal = bone, offset, direction
		self.report({"INFO"}, f"Placed '{contact.name}' on '{bone}'")
		self._End(context)
		return {"FINISHED"}

	def _End(self, context) -> None:
		"""Restore the cursor and status bar, and redraw so that the panel shows the new values."""
		context.window.cursor_modal_restore()
		context.workspace.status_text_set(None)
		for area in context.window.screen.areas:
			area.tag_redraw()


def ContactFromSurface(armature_obj: bpy.types.Object, mesh_eval: bpy.types.Object, location: Vector, face: int) -> tuple[str, Vector, Vector]:
	"""Return the bone, bone-space offset and bone-space normal of a contact at a world-space surface point of an evaluated (posed) mesh.

	The normal is the shaded normal at the point (see ShadedNormal), so it follows the smooth surface that is rendered rather than the faceted mesh.

	The bone is the deforming bone with the most weight over the vertices of the hit face. Raises ValueError if no bone of the armature weights that face.
	"""
	# Total each bone's weight over the hit face's vertices
	bones = armature_obj.data.bones
	mesh = mesh_eval.data
	totals: dict[str, float] = {}
	for vertex in mesh.polygons[face].vertices:
		for element in mesh.vertices[vertex].groups:
			name = mesh_eval.vertex_groups[element.group].name
			if name in bones and bones[name].use_deform:
				totals[name] = totals.get(name, 0.0) + element.weight

	if not totals:
		raise ValueError("No bone deforms the picked face")

	# Express the point and direction in the bone's current frame, which has the same axes as its rest frame
	name = max(totals, key=totals.get)
	frame = armature_obj.matrix_world @ armature_obj.pose.bones[name].matrix
	offset = frame.inverted_safe() @ location
	direction = (frame.to_3x3().inverted_safe() @ ShadedNormal(mesh_eval, location, face)).normalized()
	return name, offset, direction


def ShadedNormal(mesh_eval: bpy.types.Object, location: Vector, face: int) -> Vector:
	"""Return the world-space normal used for shading at a world-space point on a face of an evaluated mesh.

	For a smooth-shaded face this blends the face's corner normals by the point's position within the face. For a flat-shaded face it is the face normal.
	"""
	# Weight each corner of the face by the point's position within it, in object space
	mesh = mesh_eval.data
	polygon = mesh.polygons[face]
	point = mesh_eval.matrix_world.inverted_safe() @ location
	weights = poly_3d_calc([mesh.vertices[vertex].co for vertex in polygon.vertices], point)
	normal = sum((weight * mesh.corner_normals[corner].vector for weight, corner in zip(weights, polygon.loop_indices)), Vector())

	# Normals transform by the inverse transpose so that they stay perpendicular to the surface under non-uniform scale
	return (mesh_eval.matrix_world.to_3x3().inverted_safe().transposed() @ normal).normalized()


def _MouseRay(context: bpy.types.Context, event: bpy.types.Event) -> tuple[Vector, Vector] | None:
	"""Return the world-space origin and direction of the view ray under the mouse, or None if the mouse is not over a 3D view."""
	# Window coordinates select the 3D view region under the mouse, then region coordinates give the ray
	for area in context.window.screen.areas:
		if area.type != "VIEW_3D":
			continue
		for region in area.regions:
			x, y = event.mouse_x - region.x, event.mouse_y - region.y
			if region.type == "WINDOW" and 0 <= x < region.width and 0 <= y < region.height:
				view = area.spaces.active.region_3d
				return view3d_utils.region_2d_to_origin_3d(region, view, (x, y)), view3d_utils.region_2d_to_vector_3d(region, view, (x, y))

	return None


def _Deforms(obj: bpy.types.Object | None, armature_obj: bpy.types.Object) -> bool:
	"""Return true if 'obj' is a mesh with an armature modifier that uses 'armature_obj'."""
	return obj is not None and obj.type == "MESH" and any(modifier.type == "ARMATURE" and modifier.object == armature_obj for modifier in obj.modifiers)


def _ReportUnmatched(operator: Operator, unmatched: list[str]) -> None:
	"""Warn about side-named bones that have no mirror partner, or whose partner's rest transform is not their mirror image."""
	if unmatched:
		operator.report({"WARNING"}, f"Not mirrored, as no bone is their exact mirror image: {', '.join(unmatched)}")


# The rig's editable lists
_COLLECTIONS = [("CHAINS", "Chains", ""), ("CONTACTS", "Contacts", ""), ("CYCLES", "Movement Cycles", "")]


class RYLOGIC_OT_gait_validate(_ArmatureOperator):
	"""Measure the exported movement cycles and check the gait for the rules that the runtime relies on"""

	bl_idname = "rylogic_rig.gait_validate"
	bl_label = "Validate Gait"

	def execute(self, context):
		# Measuring plays each cycle, which can fail as well as the checks
		try:
			desc = gait_desc.BuildGaitDesc(ActiveArmature(context), context)
			errors = gait_desc.ValidateGaitDesc(desc)
		except ValueError as error:
			desc, errors = None, [str(error)]

		# Summarise each measured cycle, so that its footfalls can be checked against the intended pattern
		lines = list(errors)
		if not errors:
			lines.append("Valid gait")
			for family in desc["families"]:
				for cycle in family["cycles"]:
					steps = ", ".join(f"{f['role']} {f['side']} @{f['touchdown']:.2f}/{f['duty']:.2f}" for f in cycle["footfalls"])
					lines.append(f"{family['name']}.{cycle['name']}: speed {cycle['speed']:.3g}, stride {cycle['stride']:.3g}; {steps}")

		_SetReport(lines)
		for error in errors:
			self.report({"WARNING"}, error)

		self.report({"ERROR"} if errors else {"INFO"}, f"{len(errors)} problems" if errors else "The gait is valid")
		return {"FINISHED"}


class RYLOGIC_OT_gait_export(_ArmatureOperator, ExportHelper):
	"""Write the exported movement cycles to a '.gait' file"""

	bl_idname = "rylogic_rig.gait_export"
	bl_label = "Export .gait"

	filename_ext = ".gait"
	filter_glob: StringProperty(default="*.gait", options={"HIDDEN"})

	def execute(self, context):
		# Refuse to write an invalid gait
		try:
			WriteGait(ActiveArmature(context), Path(self.filepath), context)
		except ValueError as error:
			self.report({"ERROR"}, str(error))
			return {"CANCELLED"}

		self.report({"INFO"}, f"Wrote {self.filepath}")
		return {"FINISHED"}


class RYLOGIC_OT_gait_template(_ArmatureOperator):
	"""Add a movement cycle keyed with a standard footfall pattern, as a starting point to refine by hand"""

	bl_idname = "rylogic_rig.gait_template"
	bl_label = "Add Template Cycle"
	bl_options = {"REGISTER", "UNDO"}

	template: EnumProperty(name="Template", items=gait_template.TEMPLATE_ITEMS)

	def execute(self, context):
		# The template needs the walking limbs' chains and contacts
		try:
			action = gait_template.GenerateTemplate(ActiveArmature(context), context, self.template)
		except ValueError as error:
			self.report({"ERROR"}, str(error))
			return {"CANCELLED"}

		self.report({"INFO"}, f"Added movement cycle '{action.name}'")
		return {"FINISHED"}


class RYLOGIC_OT_rig_item_add(_ArmatureOperator):
	"""Add a chain, contact or movement cycle"""

	bl_idname = "rylogic_rig.item_add"
	bl_label = "Add"
	bl_options = {"REGISTER", "UNDO"}

	collection: EnumProperty(items=_COLLECTIONS)

	def execute(self, context):
		# Append a new item and make it active
		rig = ActiveArmature(context).data.rylogic_rig
		match self.collection:
			case "CHAINS":
				rig.chains.add()
				rig.active_chain = len(rig.chains) - 1
			case "CONTACTS":
				contact = rig.contacts.add()
				contact.name = f"contact.{len(rig.contacts)}"
				rig.active_contact = len(rig.contacts) - 1
			case "CYCLES":
				cycle = rig.cycles.add()
				cycle.name = f"cycle.{len(rig.cycles)}"
				rig.active_cycle = len(rig.cycles) - 1
			case _:
				raise ValueError(f"Unknown collection '{self.collection}'")

		return {"FINISHED"}


class RYLOGIC_OT_rig_item_remove(_ArmatureOperator):
	"""Remove the active chain, contact or movement cycle"""

	bl_idname = "rylogic_rig.item_remove"
	bl_label = "Remove"
	bl_options = {"REGISTER", "UNDO"}

	collection: EnumProperty(items=_COLLECTIONS)

	def execute(self, context):
		# Remove the active item and keep the selection in range
		rig = ActiveArmature(context).data.rylogic_rig
		match self.collection:
			case "CHAINS":
				if 0 <= rig.active_chain < len(rig.chains):
					rig.chains.remove(rig.active_chain)
					rig.active_chain = max(0, min(rig.active_chain, len(rig.chains) - 1))
			case "CONTACTS":
				if 0 <= rig.active_contact < len(rig.contacts):
					rig.contacts.remove(rig.active_contact)
					rig.active_contact = max(0, min(rig.active_contact, len(rig.contacts) - 1))
			case "CYCLES":
				if 0 <= rig.active_cycle < len(rig.cycles):
					rig.cycles.remove(rig.active_cycle)
					rig.active_cycle = max(0, min(rig.active_cycle, len(rig.cycles) - 1))
			case _:
				raise ValueError(f"Unknown collection '{self.collection}'")

		return {"FINISHED"}


class RYLOGIC_OT_rig_chain_assign(_ArmatureOperator):
	"""Set the active chain's bones to the selected bones, ordered from base to end"""

	bl_idname = "rylogic_rig.chain_assign"
	bl_label = "Assign Selected Bones"
	bl_options = {"REGISTER", "UNDO"}

	def execute(self, context):
		# Order the selection by depth so that the chain runs from base to end; Validate reports a selection that is not a parent-child run
		armature = ActiveArmature(context).data
		rig = armature.rylogic_rig
		if not 0 <= rig.active_chain < len(rig.chains):
			self.report({"ERROR"}, "No active chain")
			return {"CANCELLED"}

		armature_obj = ActiveArmature(context)
		bones = armature.edit_bones if armature_obj.mode == "EDIT" else armature.bones
		selected = [bones[name] for name in props.SelectedBoneNames(armature_obj)]
		selected.sort(key=lambda bone: len(bone.parent_recursive))
		chain = rig.chains[rig.active_chain]
		chain.bones.clear()
		for bone in selected:
			chain.bones.add().name = bone.name

		return {"FINISHED"}


def _ExportSkinnedGlb(context: bpy.types.Context, armature_obj: bpy.types.Object, path: Path) -> None:
	"""Export 'armature_obj' and the meshes it deforms as a Z-up skinned GLB, restoring the selection afterwards."""
	# The export uses the selection, so remember it
	meshes = [obj for obj in context.scene.objects if obj.type == "MESH" and any(mod.type == "ARMATURE" and mod.object == armature_obj for mod in obj.modifiers)]
	if not meshes:
		raise ValueError(f"No mesh is deformed by armature '{armature_obj.name}'")

	view_layer = context.view_layer
	saved_selection = [obj for obj in view_layer.objects if obj.select_get()]
	saved_active = view_layer.objects.active
	try:
		# Match the skinned-model export used by asset pipelines: Blender's Z-up axes, with skins
		for obj in saved_selection:
			obj.select_set(False)
		for obj in [armature_obj, *meshes]:
			obj.select_set(True)

		view_layer.objects.active = armature_obj
		result = bpy.ops.export_scene.gltf(
			filepath=str(path), check_existing=False, export_format="GLB", use_selection=True,
			export_yup=False, export_skins=True, export_animations=False, export_materials="NONE", will_save_settings=False,
		)
		if result != {"FINISHED"}:
			raise RuntimeError(f"Blender glTF export did not finish: {result}")
	finally:
		# Restore the authoring selection even when the export fails
		for obj in view_layer.objects:
			obj.select_set(obj in saved_selection)

		view_layer.objects.active = saved_active


class RYLOGIC_AP_preferences(bpy.types.AddonPreferences):
	"""Add-on settings."""

	bl_idname = __package__

	p3d_path: StringProperty(name="p3d Tool", subtype="FILE_PATH", description="The p3d tool (p3d.exe), used by 'Validate (native)'")

	def draw(self, context):
		self.layout.prop(self, "p3d_path")


CLASSES = (
	RYLOGIC_OT_rig_validate,
	RYLOGIC_OT_rig_validate_native,
	RYLOGIC_OT_rig_export,
	RYLOGIC_OT_rig_capture_pose,
	RYLOGIC_OT_rig_derive_limits,
	RYLOGIC_OT_rig_symmetrize_limits,
	RYLOGIC_OT_rig_check_limits,
	RYLOGIC_OT_rig_copy_ik_limits,
	RYLOGIC_OT_rig_contact_pick,
	RYLOGIC_OT_gait_validate,
	RYLOGIC_OT_gait_export,
	RYLOGIC_OT_gait_template,
	RYLOGIC_OT_rig_item_add,
	RYLOGIC_OT_rig_item_remove,
	RYLOGIC_OT_rig_chain_assign,
	RYLOGIC_AP_preferences,
)


def Register() -> None:
	"""Register the operators and preferences."""
	for cls in CLASSES:
		bpy.utils.register_class(cls)


def Unregister() -> None:
	"""Unregister the operators and preferences."""
	for cls in reversed(CLASSES):
		bpy.utils.unregister_class(cls)
