#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import struct
from ..gfx.colour import Colour32
from ..math.vector import Vec2, Vec3, Vec4, Mat4
from ..math.bbox import BBox
from ..math.variable_int import VariableInt
from typing import List, Optional, Tuple
from enum import Enum

# Binary packing helper
def _Pack(obj) -> bytes:
	if callable(obj):
		result = obj()
		if not isinstance(result, bytes): raise TypeError(f"Unsupported callable return type: {type(result)}")
		return result
	if isinstance(obj, bytes):
		return obj
	if hasattr(obj, '__pack__'):
		return obj.__pack__()
	if isinstance(obj, int):
		return struct.pack('<I', obj)
	if isinstance(obj, float):
		return struct.pack('<f', obj)
	if isinstance(obj, Enum):
		return struct.pack('<I', obj.value)
	if isinstance(obj, str):
		return obj.encode('utf-8')
	if isinstance(obj, list):
		return b''.join([_Pack(i) for i in obj])
	raise TypeError(f"Unsupported type: {type(obj)}")

# String packing helper
def _Str(obj) -> str:
	if callable(obj):
		result = obj()
		if not isinstance(result, str): raise TypeError(f"Unsupported callable return type: {type(result)}")
		return result
	if isinstance(obj, str):
		return obj
	if isinstance(obj, Enum) and not isinstance(obj, EKeyword):
		return obj.name
	if isinstance(obj, (bytes, bytearray)):
		return ''.join([f"{i:02X}" for i in obj])
	if isinstance(obj, list):
		return ' '.join([_Str(i) for i in obj])
	if hasattr(obj, '__str__'):
		return str(obj)
	raise TypeError(f"Unsupported type: {type(obj)}")

# Append a list of arguments to 'ldr'
def _Append(out: List[str]|bytearray, *args):
	if isinstance(out, bytearray):
		for a in args:
			abytes = bytes() if a is None else _Pack(a)
			if not abytes: continue
			out.extend(abytes)
	elif isinstance(out, list):
		for a in args:
			astr = '' if a is None else _Str(a)
			if not astr: continue
			last = out[-1][-1] if len(out) != 0 and len(out[-1]) != 0 else None
			out.append(' ' if last and not last.isspace() and last != '{' and astr[0] != '}' else '')
			out.append(astr)

# Write a ldraw block to 'out'
def _Write(out: List[str]|bytearray, keyword: 'EKeyword', *args):
	if isinstance(out, bytearray):
		ofs = len(out)
		_Append(out, keyword)
		_Append(out, 0)

		# if len(args) != 0 and isinstance(args[0], _LdrName):
		# 	if args[0].m_name: _Append(out, args[0].m_name)
		# 	args = args[1:]
		# if len(args) != 0 and isinstance(args[0], _LdrColour):
		# 	if args[0].m_colour.argb != 0xFFFFFFFF: _Append(out, args[0].m_colour)
		# 	args = args[1:]

		for a in args:
			if callable(a):
				a()
			else:
				_Append(out, a)

		size = len(out) - ofs - 8
		out[ofs+4:ofs+8] = struct.pack('<I', size)
	if isinstance(out, list):
		_Append(out, keyword)

		if len(args) != 0 and isinstance(args[0], _LdrName):
			if args[0].m_name: _Append(out, args[0].m_name)
			args = args[1:]
		if len(args) != 0 and isinstance(args[0], _LdrColour):
			if args[0].m_colour.argb != 0xFFFFFFFF: _Append(out, args[0].m_colour)
			args = args[1:]

		_Append(out, '{')
		for a in args:
			if callable(a):
				a()
			else:
				_Append(out, a)
		_Append(out, '}')

# Pretty format Ldraw script
def FormatScript(s: str, indent_str: str = '\t') -> str:
	out = []
	indent = 0
	for c in s:
		if c == '{':
			indent += 1
			out.append(c)
			out.append('\n')
			out.append(indent * indent_str)
		elif c == '}':
			indent -= 1
			out.append('\n')
			out.append(indent * indent_str)
			out.append(c)
			out.append('\n')
			out.append(indent * indent_str)
		else:
			out.append(c)

	return ''.join(out)

# Ldraw keyword values
class EKeyword(Enum):
	# AUTO-GENERATED-KEYWORDS-BEGIN
	Accel = 3784776339
	Addr = 1087856498
	Align = 1613521886
	Alpha = 1569418667
	Ambient = 479609067
	Anchor = 1122880180
	AngAccel = 801436173
	AngVelocity = 2226367268
	Animation = 3779456605
	AnimSource = 4254838773
	Arrow = 2220107398
	Aspect = 2929999953
	Axis = 1831579124
	AxisId = 1558262649
	BackColour = 1583172070
	BakeTransform = 3697187404
	BaseColour = 2533849228
	BaseColourTexture = 1020105447
	Billboard = 3842439780
	Billboard3D = 3185018435
	BinaryStream = 1110191492
	Box = 1892056626
	BoxList = 282663022
	CastShadow = 3890809582
	Chart = 1487494731
	Circle = 673280137
	Closed = 3958264005
	Colour = 33939709
	Colours = 3175120778
	Commands = 3062934995
	Cone = 3711978346
	ConvexHull = 1990998937
	CoordFrame = 3958982335
	CornerRadius = 2396916310
	CrossSection = 3755993582
	CString = 2719639595
	Custom = 542584942
	Cylinder = 2904473583
	Dashed = 4029489596
	Data = 3631407781
	DataPoints = 1656579750
	Depth = 4269121258
	Diffuse = 1416505917
	Dim = 3496118841
	Direction = 3748513642
	Divisions = 555458703
	Emissive = 2172449998
	EmissiveTexture = 2593342865
	EndCaps = 2864440763
	Equation = 2486886355
	Euler = 1180123250
	Faces = 455960701
	Faces32 = 1542536296
	Facets = 3463018577
	Far = 3170376174
	FilePath = 1962937316
	Filter = 3353438327
	Font = 659427984
	ForeColour = 3815865055
	Format = 3114108242
	Fov = 2968750556
	FovX = 862039340
	FovY = 878816959
	Frame = 3523899814
	FrameRange = 1562558803
	FrameRate = 2601589476
	FrustumFA = 3281884904
	FrustumWH = 3334630522
	GenerateNormals = 750341558
	Grid = 2944866961
	Group = 1605967500
	GroupTint = 381584201
	Hidden = 4128829753
	HideWhenNotAnimating = 2975106646
	Instance = 193386898
	Intensity = 2237916426
	Inverse = 2986472067
	Layers = 2411172191
	LeftHanded = 1992685208
	LightSource = 2597226090
	Line = 400234023
	LineBox = 3297263992
	LineList = 419493935
	Lines = 3789825596
	Lines32 = 4174700157
	LineStrip = 4082781759
	LookAt = 3951693683
	M3x3 = 1709156072
	M4x4 = 3279345952
	Material = 3538210912
	Mesh = 2701180604
	Metallic = 2024658632
	MetallicTexture = 1827337475
	Model = 2961925722
	Montage = 2939791094
	Name = 2369371622
	Near = 1425233679
	NewLine = 4281549323
	NoMaterials = 762077060
	NonAffine = 3876544483
	Normalise = 4066511049
	Normals = 247908339
	NormalTexture = 2206230309
	NoRotation = 4019988840
	NoTranslation = 864235717
	NoZTest = 329427844
	NoZWrite = 1339143375
	O2W = 2877203913
	Orthographic = 3824181163
	Orthonormalise = 2850748489
	Padding = 2157316278
	Param = 1309554226
	Parametrics = 4148475404
	Part = 2088252948
	Parts = 1480434725
	Period = 2580104964
	PerItemColour = 1734234667
	PerItemParametrics = 2079701142
	Pie = 1782644405
	Plane = 3435855957
	Point = 414084241
	PointDepth = 1069768758
	PointSize = 375054368
	PointStyle = 2082693170
	Polygon = 85768329
	Pos = 1412654217
	Position = 2471448074
	Quad = 1738228046
	Quat = 1469786142
	QuatPos = 1842422916
	Rand4x4 = 1326225514
	RandColour = 1796074266
	RandOri = 55550374
	RandPos = 4225427356
	Range = 4208725202
	Rect = 3940830471
	Reflectivity = 3111471187
	Resolution = 488725647
	Ribbon = 1119144745
	RootAnimation = 464566237
	Roughness = 3139841781
	RoughnessTexture = 1537835364
	Round = 1326178875
	Scale = 2190941297
	ScreenSpace = 3267318065
	Series = 3703783856
	Size = 597743964
	Smooth = 24442543
	Solid = 2973793012
	Source = 466561496
	Specular = 3195258592
	Sphere = 2950268184
	SphereList = 3185373280
	Square = 3031831110
	Step = 3343129103
	Stretch = 3542801962
	Strikeout = 3261692833
	Style = 2888859350
	Tetra = 1647597299
	Tetra32 = 2555339638
	TexCoords = 536531680
	Text = 3185987134
	TextLayout = 2881593448
	TextStream = 998584670
	Texture = 1013213428
	TimeBias = 2748914857
	TimeRange = 1138166793
	Transpose = 3224470464
	Triangle = 84037765
	TriList = 3668920810
	TriStrip = 1312470952
	Tube = 1747223167
	TwoSided = 3612706098
	Txfm = 2438414104
	Underline = 3850515583
	Unknown = 2608177081
	Up = 1128467232
	Velocity = 846470194
	Verts = 3167497763
	Video = 3472427884
	ViewPlaneZ = 458706800
	Wedges = 451463732
	Weight = 1352703673
	Width = 2508680735
	Wireframe = 305834533
	XAxis = 3274667154
	XColumn = 3953029077
	YAxis = 1077811589
	ZAxis = 3837765916
	# AUTO-GENERATED-KEYWORDS-END
	def __int__(self):
		return self.value
	def __str__(self):
		return f"*{self.name}"
	def __pack__(self):
		return struct.pack('<I', self.value)
class ECommandId(Enum):
	# AUTO-GENERATED-COMMANDS-BEGIN
	Invalid = 3419534640
	Clear = 1550717474
	ObjectToWorld = 1059927965
	ObjectColour = 886758798
	Render = 4009327117
	# AUTO-GENERATED-COMMANDS-END
	def __int__(self):
		return self.value
	def __str__(self):
		return self.name
	def __pack__(self):
		return struct.pack('<I', self.value)

# Primitive types ------------------------------------------
class EPointStyle(Enum):
	Square = 0
	Circle = 1
	Triangle = 2
	Star = 3
	Annulus = 4
class StringWithLength:
	def __init__(self, string: str=''):
		self.m_string = string
	def __repr__(self):
		return str(self)
	def __str__(self):
		return self.m_string
	def __pack__(self):
		strbytes = self.m_string.encode('utf-8')
		return VariableInt(len(strbytes)).__pack__() + strbytes

# Implementation ------------------------------------------
class _LdrName:
	def __init__(self, name: str = ''):
		super().__init__()
		self.m_name = name
	def __repr__(self):
		return self.m_name
	def __str__(self):
		return f'{EKeyword.Name} {{{self.m_name}}}' if self.m_name else ''
	def __pack__(self):
		return (struct.pack('<II', EKeyword.Name.value, len(self.m_name)) + self.m_name.encode('utf-8')) if self.m_name else bytes()
class _LdrColour:
	def __init__(self, colour: Colour32 = Colour32()):
		super().__init__()
		self.m_colour = colour
	def __repr__(self):
		return self.m_colour.__repr___()
	def __str__(self):
		return f'{EKeyword.Colour} {{{self.m_colour.argb:08X}}}' if self.m_colour.argb != 0xFFFFFFFF else ''
	def __pack__(self):
		return struct.pack('<III', EKeyword.Colour.value, 4, self.m_colour.argb) if self.m_colour.argb != 0xFFFFFFFF else bytes()
class _LdrSize:
	def __init__(self, size: float=0):
		super().__init__()
		self.m_size = size
	def __repr__(self):
		return self.m_size
	def __str__(self):
		return f'{EKeyword.Size} {{{self.m_size}}}' if self.m_size != 0 else ''
	def __pack__(self):
		return struct.pack('<III', EKeyword.Size.value, 4, self.m_size) if self.m_size != 0 else bytes()
class _LdrPerItemColour:
	def __init__(self, per_item_colour: bool=False):
		super().__init__()
		self.m_per_item_colour = per_item_colour
	def __bool__(self):
		return self.m_per_item_colour
	def __repr__(self):
		return self.m_per_item_colour
	def __str__(self):
		return f'{EKeyword.PerItemColour} {{}}' if self.m_per_item_colour else ''
	def __pack__(self):
		return struct.pack('<II', EKeyword.PerItemColour.value, 0) if self.m_per_item_colour else bytes()
class _LdrDepth:
	def __init__(self, depth: bool=False):
		super().__init__()
		self.m_depth = depth
	def __repr__(self):
		return self.m_depth
	def __str__(self):
		return f'{EKeyword.Depth} {{}}' if self.m_depth else ''
	def __pack__(self):
		return struct.pack('<II', EKeyword.Depth.value, 0) if self.m_depth else bytes()
class _LdrWireframe:
	def __init__(self, wire: bool=False):
		super().__init__()
		self.m_wire = wire
	def __repr__(self):
		return self.m_wire
	def __str__(self):
		return f'{EKeyword.Wireframe} {{}}' if self.m_wire else ''
	def __pack__(self):
		return struct.pack('<II', EKeyword.Wireframe.value, 0) if self.m_wire else bytes()
class _LdrSolid:
	def __init__(self, solid: bool=False):
		super().__init__()
		self.m_solid = solid
	def __repr__(self):
		return self.m_solid
	def __str__(self):
		return f'{EKeyword.Solid} {{}}' if self.m_solid else ''
	def __pack__(self):
		return struct.pack('<II', EKeyword.Solid.value, 0) if self.m_solid else bytes()
class _LdrAxisId:
	def __init__(self, axis_id: int=0):
		super().__init__()
		self.m_axis_id = axis_id
	def __repr__(self):
		return self.m_axis_id
	def __str__(self):
		return f'{EKeyword.AxisId} {{{self.m_axis_id}}}' if self.m_axis_id != 0 else ''
	def __pack__(self):
		return struct.pack('<III', EKeyword.AxisId.value, 4, self.m_axis_id) if self.m_axis_id != 0 else bytes()
class _LdrPos:
	def __init__(self, xyz: Optional[Vec3]=None):
		super().__init__()
		self.m_pos = xyz
	def __repr__(self):
		return self.m_pos
	def __str__(self):
		if not self.m_pos: return ''
		if self.m_pos == Vec3.Zero: return ''
		return f'{EKeyword.O2W} {{{EKeyword.Pos} {{{self.m_pos}}}}}'
	def __pack__(self):
		if not self.m_pos: return bytes()
		if self.m_pos == Vec3.Zero: return bytes()
		return struct.pack('<IIIIfff', EKeyword.O2W.value, 20, EKeyword.Pos.value, 12, self.m_pos.x, self.m_pos.y, self.m_pos.z)
class _LdrO2W:
	def __init__(self, o2w: Optional[Mat4]=None, pos:Optional[Vec3]=None):
		super().__init__()
		self.m_o2w = None
		if o2w: self.m_o2w = o2w
		if pos: self.m_o2w = Mat4(Vec4(1,0,0,0), Vec4(0,1,0,0), Vec4(0,0,1,0), pos.w1())
	def __repr__(self):
		return self.m_o2w
	def __str__(self):
		if not self.m_o2w: return ''
		if self.m_o2w.is_identity: return ''
		if self.m_o2w.is_translation: return f'{EKeyword.O2W} {{{EKeyword.Pos} {{{self.m_o2w.w.xyz}}}}}'
		return f'{EKeyword.O2W} {{{EKeyword.M4x4} {{{self.m_o2w}}}}}'
	def __pack__(self):
		if not self.m_o2w: return bytes()
		if self.m_o2w.is_identity: return bytes()
		if self.m_o2w.is_translation: return struct.pack('<IIIIfff', EKeyword.O2W.value, 20, EKeyword.Pos.value, 12, self.m_o2w.w.x, self.m_o2w.w.y, self.m_o2w.w.z)
		return struct.pack('<IIII', EKeyword.O2W.value, 72, EKeyword.M4x4.value, 64) + _Pack(self.m_o2w)

class _LdrObj:
	def __init__(self):
		self.m_objects :List['_LdrObj'] = []

	# Remove all children
	def reset(self):
		self.m_objects = []

	# Child objects
	def Group(self, name: str = '', colour: int = 0xFFFFFFFF):
		child = LdrGroup()
		self.m_objects.append(child)
		return child.name(name).col(Colour32(colour))
	def Points(self, name: str = '', colour: int = 0xFFFFFFFF):
		child = LdrPoint()
		self.m_objects.append(child)
		return child.name(name).col(Colour32(colour))
	def Box(self, name: str = '', colour: int = 0xFFFFFFFF):
		child = LdrBox()
		self.m_objects.append(child)
		return child.name(name).col(Colour32(colour))
	def Polygon(self, name: str = '', colour: int = 0xFFFFFFFF):
		child = LdrPolygon()
		self.m_objects.append(child)
		return child.name(name).col(Colour32(colour))
	def Command(self):
		child = LdrCommands()
		self.m_objects.append(child)
		return child

	# Serialise to a string
	def ToString(self) -> str:
		out = []
		self._WriteTo(out)
		return ''.join(out)

	# Serialise to a byte array
	def ToBytes(self) -> bytes:
		out = bytearray()
		self._WriteTo(out)
		return bytes(out)

	# Serialize this object to 'out'
	def _WriteTo(self, out: List[str]|bytearray):
		for s in self.m_objects:
			s._WriteTo(out)

class _LdrBase(_LdrObj):
	def __init__(self):
		super().__init__()
		self.m_name :_LdrName = _LdrName()
		self.m_o2w :_LdrO2W = _LdrO2W()
		self.m_axis_id :_LdrAxisId = _LdrAxisId()
		self.m_colour :_LdrColour = _LdrColour()
		self.m_wire :_LdrWireframe = _LdrWireframe()
		self.m_solid :_LdrSolid = _LdrSolid()

	def name(self, name: str):
		self.m_name = _LdrName(name)
		return self

	def col(self, colour: Colour32):
		self.m_colour = _LdrColour(colour)
		return self

	def pos(self, xyz: Vec3):
		self.m_o2w = _LdrO2W(pos=xyz)
		return self

	def o2w(self, o2w: Mat4):
		self.m_o2w = _LdrO2W(o2w=o2w)
		return self

	def scale(self, xyz: Vec3):
		self.m_o2w = _LdrO2W(o2w=Mat4(Vec4(xyz.x, 0, 0, 0), Vec4(0, xyz.y, 0, 0), Vec4(0, 0, xyz.z, 0), Vec4(0, 0, 0, 1)))
		return self

	def _WriteTo(self, out: List[str]|bytearray):
		_Append(out, self.m_axis_id, self.m_wire, self.m_solid, self.m_o2w)
		super()._WriteTo(out)

class LdrPoint(_LdrBase):
	def __init__(self):
		super().__init__()
		self.m_points: List[Vec3] = []
		self.m_colours: List[Colour32] = []
		self.m_per_item_colours: _LdrPerItemColour = _LdrPerItemColour()
		self.m_style: EPointStyle = EPointStyle.Square
		self.m_size: _LdrSize = _LdrSize()
		self.m_depth: _LdrDepth = _LdrDepth()

	def pt(self, xyz: Vec3, colour: Optional[int]=None):
		self.m_points.append(xyz)
		if colour is not None:
			self.m_colours.append(Colour32(colour))
		elif self.m_per_item_colours:
			self.m_colours.append(Colour32())
		return self

	def style(self, style: EPointStyle):
		self.m_style = style
		return self

	def size(self, size: float):
		self.m_size = _LdrSize(size)
		return self

	def depth(self, depth: bool=True):
		self.m_depth = _LdrDepth(depth)
		return self

	def _WriteTo(self, out: List[str]|bytearray):
		base = super()
		_Write(out, EKeyword.Point, self.m_name, self.m_colour, lambda: (
			_Write(out, EKeyword.Style, self.m_style),
			_Append(out, self.m_size, self.m_depth),
			_Write(out, EKeyword.Data, lambda: (
				_Append(out, zip(self.m_points, self.m_colours) if self.m_per_item_colours else self.m_points)
			)),
			base._WriteTo(out)
		))

class LdrBox(_LdrBase):
	def __init__(self):
		super().__init__()
		self.m_dim: Vec3 = Vec3(0, 0, 0)

	def dim(self, d: Vec3|float|int) -> 'LdrBox':
		self.m_dim = d if isinstance(d, Vec3) else Vec3(d, d, d)
		return self

	def radii(self, radii: Vec3|float|int) -> 'LdrBox':
		return self.dim(Vec3(radii.x * 2, radii.y * 2, radii.z * 2) if isinstance(radii, Vec3) else radii * 2)

	# Create from bounding box
	def bbox(self, bbox: BBox) -> 'LdrBox':
		if bbox == BBox.Reset(): return self
		return self.dim(2 * bbox.radius).pos(bbox.centre)

	def _WriteTo(self, out: List[str]|bytearray):
		base = super()
		_Write(out, EKeyword.Box, self.m_name, self.m_colour, lambda: (
			_Write(out, EKeyword.Data, self.m_dim),
			base._WriteTo(out)
		))

class LdrPolygon(_LdrBase):
	def __init__(self):
		super().__init__()
		self.m_points: List[Vec2] = []
		self.m_colours: List[Colour32] = []
		self.m_per_item_colour: _LdrPerItemColour = _LdrPerItemColour()

	def pt(self, xy: Vec2, colour: Optional[Colour32] = None):
		self.m_points.append(xy)
		if colour:
			self.m_colours.append(colour)
		elif self.m_per_item_colour:
			self.m_colours.append(Colour32())
		return self

	def solid(self, solid: bool=True):
		self.m_solid = _LdrSolid(solid)
		return self

	def _WriteTo(self, out: List[str]|bytearray):
		base = super()
		_Write(out, EKeyword.Polygon, self.m_name, self.m_colour, lambda: (
			_Write(out, EKeyword.Data, lambda: (
				_Append(out, zip(self.m_points, self.m_colours) if self.m_per_item_colour else self.m_points)
			)),
			base._WriteTo(out)
		))

class LdrGroup(_LdrBase):
	def __init__(self):
		super().__init__()

	def _WriteTo(self, out: List[str]|bytearray):
		base = super()
		_Write(out, EKeyword.Group, self.m_name, self.m_colour, lambda: base._WriteTo(out))

class LdrCommands(_LdrBase):
	def __init__(self):
		super().__init__()
		self.m_cmds :List[Tuple[ECommandId, List[int|float|StringWithLength|Vec2|Vec3|Vec4|Mat4|Mat4]]] = []

	def clear(self):
		self.m_cmds.append((ECommandId.Clear, []))
		return self

	def transform_object(self, object_name: str, o2w: Mat4):
		self.m_cmds.append((ECommandId.ObjectToWorld, [StringWithLength(object_name), o2w]))
		return self

	def object_colour(self, object_name: str, colour: int):
		self.m_cmds.append((ECommandId.ObjectColour, [StringWithLength(object_name), colour]))
		return self

	def render_frame(self):
		self.m_cmds.append((ECommandId.Render, []))
		return self

	def _WriteTo(self, out: List[str]|bytearray):
		def WriteCmds():
			for cmd in self.m_cmds:
				_Write(out, EKeyword.Data, cmd[0], *cmd[1])
		_Write(out, EKeyword.Commands, self.m_name, self.m_colour, WriteCmds)

class Builder(_LdrObj):
	def __init__(self):
		super().__init__()

