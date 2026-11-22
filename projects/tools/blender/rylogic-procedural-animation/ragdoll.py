"""Generate the capsules of a ragdoll from an armature's bones.

A ragdoll is a set of linked rigid bodies that replaces an animated character when it goes limp. Each body is a capsule that follows one 'link' bone.
A capsule covers its link bone and the following bones that continue it end-to-end (each bone's head at the previous bone's tail) until the next link
bone, so that, for example, a hips link can also cover a short lower spine. Its radius fits the skin that those bones deform.

Each capsule is a mesh object named 'RAG_<bone>', parented to its bone, with a Rigid Body capsule shape and a 'physics_material' property. The asset
exporter reads them from the model's physics collection. A link's parent in the ragdoll is its nearest ancestor bone that is also a link, and its joint
limit combines the joint limits of the bones between them.
"""

import math

import bpy
from mathutils import Matrix, Vector

# The prefix that marks a mesh object as a ragdoll capsule
RAGDOLL_PREFIX = "RAG_"

# Bones whose ends are this close (in armature space) continue one another
_CONTINUE_TOLERANCE = 1e-4

# A vertex belongs to a capsule's skin when its weight to the covered bones is above this
_SKIN_WEIGHT = 0.5

# The number of segments around a capsule, and of rings in each end cap. The segments are a multiple of four so that the mesh bounds measure the radius exactly.
_SEGMENTS = 16
_CAP_RINGS = 4


def CapsuleName(bone_name: str) -> str:
	"""The name of the capsule object that follows 'bone_name'."""
	return RAGDOLL_PREFIX + bone_name


def CoveredBones(bone: bpy.types.Bone, link_names: set[str]) -> list[bpy.types.Bone]:
	"""Return the bones that a link bone's capsule covers: the link bone, then each single child that continues it end-to-end and is not a link."""
	# Stop at a fork, a gap, or the next link, because a capsule is one straight segment
	covered = [bone]
	while True:
		current = covered[-1]
		following = [child for child in current.children if child.name not in link_names and (child.head_local - current.tail_local).length < _CONTINUE_TOLERANCE]
		if len(following) != 1:
			return covered

		covered.append(following[0])


def _SegmentDistance(point: Vector, a: Vector, b: Vector) -> float:
	"""The distance from 'point' to the line segment from 'a' to 'b'."""
	# Clamp the projection onto the segment's line to the segment's ends
	ab = b - a
	length_sq = ab.length_squared
	t = 0.0 if length_sq == 0.0 else max(0.0, min(1.0, (point - a).dot(ab) / length_sq))
	return (point - (a + ab * t)).length


def SkinRadius(armature_obj: bpy.types.Object, covered: list[bpy.types.Bone], a: Vector, b: Vector, percentile: float) -> float | None:
	"""Return the radius about the world-space segment 'a'-'b' that encloses 'percentile' of the skin deformed by the covered bones, or None if there is no such skin."""
	# Measure the rest-pose vertices of every mesh that the armature deforms, keeping those weighted mostly to the covered bones
	names = {bone.name for bone in covered}
	distances: list[float] = []
	for obj in bpy.data.objects:
		if obj.type != "MESH" or not any(modifier.type == "ARMATURE" and modifier.object == armature_obj for modifier in obj.modifiers):
			continue

		groups = {group.index for group in obj.vertex_groups if group.name in names}
		if not groups:
			continue

		for vertex in obj.data.vertices:
			weight = sum(element.weight for element in vertex.groups if element.group in groups)
			if weight > _SKIN_WEIGHT:
				distances.append(_SegmentDistance(obj.matrix_world @ vertex.co, a, b))

	# A percentile ignores stray vertices, such as those of fingers weighted to the hand, that would make the capsule too fat
	if not distances:
		return None

	distances.sort()
	return distances[min(len(distances) - 1, int(percentile * len(distances)))]


def CapsuleMesh(name: str, radius: float, length: float) -> bpy.types.Mesh:
	"""Build a capsule mesh along local Z, centred on the origin, whose bounds are exactly 2*radius wide and length + 2*radius tall."""
	# Each end cap is a stack of rings from the equator towards its pole, finished by the pole vertex
	half = length / 2.0
	vertices: list[tuple[float, float, float]] = []
	for sign in (1.0, -1.0):
		for ring in range(_CAP_RINGS):
			angle = (math.pi / 2.0) * ring / _CAP_RINGS
			ring_radius = radius * math.cos(angle)
			z = sign * (half + radius * math.sin(angle))
			for segment in range(_SEGMENTS):
				around = 2.0 * math.pi * segment / _SEGMENTS
				vertices.append((ring_radius * math.cos(around), ring_radius * math.sin(around), z))

	vertices.append((0.0, 0.0, half + radius))
	vertices.append((0.0, 0.0, -(half + radius)))

	# Join neighbouring rings with quads, the two equators with the body's side, and each last ring to its pole with triangles
	def Index(cap: int, ring: int, segment: int) -> int:
		"""The index of a ring vertex. Cap 0 is the top (+Z) end and cap 1 the bottom."""
		# Rings are stored cap by cap, each with '_SEGMENTS' vertices
		return (cap * _CAP_RINGS + ring) * _SEGMENTS + segment % _SEGMENTS

	faces: list[tuple[int, ...]] = []
	top_pole = 2 * _CAP_RINGS * _SEGMENTS
	bottom_pole = top_pole + 1
	for segment in range(_SEGMENTS):
		faces.append((Index(1, 0, segment), Index(1, 0, segment + 1), Index(0, 0, segment + 1), Index(0, 0, segment)))
		for ring in range(_CAP_RINGS - 1):
			faces.append((Index(0, ring, segment), Index(0, ring, segment + 1), Index(0, ring + 1, segment + 1), Index(0, ring + 1, segment)))
			faces.append((Index(1, ring + 1, segment), Index(1, ring + 1, segment + 1), Index(1, ring, segment + 1), Index(1, ring, segment)))

		faces.append((Index(0, _CAP_RINGS - 1, segment), Index(0, _CAP_RINGS - 1, segment + 1), top_pole))
		faces.append((Index(1, _CAP_RINGS - 1, segment + 1), Index(1, _CAP_RINGS - 1, segment), bottom_pole))

	mesh = bpy.data.meshes.new(name)
	mesh.from_pydata(vertices, [], faces)
	mesh.update()
	return mesh


def CapsuleTransform(armature_obj: bpy.types.Object, bone: bpy.types.Bone, a: Vector, b: Vector) -> Matrix:
	"""Return the world transform of a capsule from 'a' to 'b' (world space) that lies along its local Z and stays as close as possible to the bone's rest orientation."""
	# Turn the bone's rest axes so that its +Y follows the segment, then turn +Y onto the capsule's local +Z
	bone_rotation = (armature_obj.matrix_world.to_3x3().normalized() @ bone.matrix_local.to_3x3()).normalized()
	bone_y = bone_rotation.col[1].normalized()
	direction = (b - a).normalized() if (b - a).length > 0.0 else bone_y
	rotation = bone_y.rotation_difference(direction).to_matrix() @ bone_rotation @ Matrix.Rotation(-math.pi / 2.0, 3, "X")
	return Matrix.Translation((a + b) / 2.0) @ rotation.to_4x4()


def GenerateRagdoll(armature_obj: bpy.types.Object, bone_names: list[str], collection: bpy.types.Collection, material: str, percentile: float = 0.6) -> list[bpy.types.Object]:
	"""Create or replace the ragdoll capsules of 'bone_names' in 'collection', returning them in the order given.

	The other bones that already have capsules count as links when deciding how far each new capsule reaches. 'collection' must be in the active
	scene so that the capsules can be given Rigid Body settings. Raises ValueError if a bone does not exist.
	"""
	# Capsules are made from the rest pose, so their parenting matches the bind pose whatever pose the document is in
	armature = armature_obj.data
	missing = [name for name in bone_names if name not in armature.bones]
	if missing:
		raise ValueError(f"Armature '{armature_obj.name}' has no bones named: {', '.join(missing)}")
	if collection not in bpy.context.scene.collection.children_recursive:
		raise ValueError(f"Collection '{collection.name}' is not in the active scene.")

	existing = {obj.parent_bone for obj in bpy.data.objects if obj.name.startswith(RAGDOLL_PREFIX) and obj.parent == armature_obj and obj.parent_type == "BONE"}
	link_names = existing | set(bone_names)
	saved_pose_position = armature.pose_position
	armature.pose_position = "REST"
	bpy.context.view_layer.update()
	try:
		created: list[bpy.types.Object] = []
		for name in bone_names:
			# Replace a previous capsule of the same bone, so that regenerating does not leave duplicates
			bone = armature.bones[name]
			old = bpy.data.objects.get(CapsuleName(name))
			if old is not None:
				old_mesh = old.data
				bpy.data.objects.remove(old)
				if old_mesh is not None and old_mesh.users == 0:
					bpy.data.meshes.remove(old_mesh)

			# Size the capsule to the covered bones' segment and skin. A bone with no skin gets a slim capsule.
			covered = CoveredBones(bone, link_names)
			a = armature_obj.matrix_world @ bone.head_local
			b = armature_obj.matrix_world @ covered[-1].tail_local
			length = (b - a).length
			radius = SkinRadius(armature_obj, covered, a, b, percentile) or max(0.01, 0.2 * length)
			obj = bpy.data.objects.new(CapsuleName(name), CapsuleMesh(CapsuleName(name), radius, length))
			collection.objects.link(obj)

			# Parent to the bone, then place in world space; Blender works out the offset from the bone
			obj.parent = armature_obj
			obj.parent_type = "BONE"
			obj.parent_bone = name
			bpy.context.view_layer.update()
			obj.matrix_world = CapsuleTransform(armature_obj, bone, a, b)
			obj.display_type = "WIRE"
			obj["physics_material"] = material

			# The capsule follows its bone, so it is a kinematic passive body that Blender's own simulation never moves
			with bpy.context.temp_override(object=obj, active_object=obj, selected_objects=[obj]):
				bpy.ops.rigidbody.object_add(type="PASSIVE")
			obj.rigid_body.collision_shape = "CAPSULE"
			obj.rigid_body.kinematic = True
			created.append(obj)

		return created
	finally:
		# Restore the authoring pose even if generation fails
		armature.pose_position = saved_pose_position
		bpy.context.view_layer.update()
