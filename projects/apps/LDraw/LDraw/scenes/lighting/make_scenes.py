#!/usr/bin/env python3
# Generates the View3D-12 lighting and shadow reference scenes (see README.md in this folder).
# Run from any directory: python make_scenes.py
import os
import math

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Distinct, saturated light colours used by the multi-light scenes.
PALETTE = [
	"FFFF4040", "FF40FF40", "FF4040FF", "FFFFFF40", "FFFF40FF", "FF40FFFF", "FFFF8020", "FF8020FF",
	"FF20FF80", "FFFFFFFF", "FFFF2080", "FF80FF20", "FF2080FF", "FFFFC080", "FF80C0FF", "FFC0FF80",
]

def write(name, text):
	# Write one scene file with LF line endings.
	path = os.path.join(OUT_DIR, name)
	with open(path, "w", newline="\n") as f:
		f.write(text)
	print(f"wrote {path}")

def fmt(v):
	# Format a 3-vector for an LDraw script.
	return f"{v[0]:g} {v[1]:g} {v[2]:g}"

def floor(name, size, colour="FF808080"):
	# A flat floor plane in the XY plane facing +Z.
	return f"*Plane {name} {colour}\n{{\n\t*Data {{{size} {size}}}\n\t*AxisId {{+3}}\n}}\n"

def box(name, colour, dim, pos):
	# An axis-aligned box centred at 'pos'.
	return f"*Box {name} {colour} {{ *Data {{{fmt(dim)}}} *o2w {{*pos {{{fmt(pos)}}}}} }}\n"

def sphere(name, colour, radius, pos):
	# A sphere centred at 'pos'.
	return f"*Sphere {name} {colour} {{ *Data {{{radius}}} *o2w {{*pos {{{fmt(pos)}}}}} }}\n"

def light_body(style, colour, shadow):
	# Properties shared by all light styles.
	s = f"\t*Style {{{style}}}\n\t*Diffuse {{{colour}}}\n\t*Specular {{FF404040 64}}\n"
	if shadow > 0:
		s += f"\t*CastShadow {{{shadow}}}\n"
	return s

def point_light(name, colour, pos, rng, falloff=0.0, shadow=0.0):
	# A point light with a limited range.
	return f"*LightSource {name} {colour}\n{{\n{light_body('Point', colour, shadow)}\t*Range {{{rng} {falloff}}}\n\t*o2w {{*pos {{{fmt(pos)}}}}}\n}}\n"

def spot_light(name, colour, pos, target, cone, rng, shadow=0.0):
	# A spot light at 'pos' aimed at 'target'. Spot lights shine down their -Z axis.
	return f"*LightSource {name} {colour}\n{{\n{light_body('Spot', colour, shadow)}\t*Range {{{rng} 0}}\n\t*Cone {{{cone[0]} {cone[1]}}}\n\t*o2w {{*pos {{{fmt(pos)}}} *LookAt {{{fmt(target)}}}}}\n}}\n"

def dir_light(name, colour, pos, shadow=0.0):
	# A directional light shining from 'pos' towards the origin.
	return f"*LightSource {name} {colour}\n{{\n{light_body('Directional', colour, shadow)}\t*o2w {{*pos {{{fmt(pos)}}} *LookAt {{0 0 0}}}}\n}}\n"

def marker(name, colour, pos):
	# A small sphere showing where a non-shadow-casting light is.
	return sphere(name, colour, 0.05, pos)

def scene_b0():
	# Lit only by the window's global light (light 0). Turn on shadows in the lighting settings to check the directional shadow.
	s = "// B0: Baseline. Lit only by the default global light. Turn on shadows in the lighting settings to check the directional shadow.\n"
	s += floor("floor", 12)
	s += box("red_box", "FFFF2020", (1, 1, 1), (-2.4, 0, 0.5))
	s += sphere("green_sphere", "FF20FF40", 0.7, (0, -0.8, 0.7))
	s += box("tall_box", "FF4080FF", (0.5, 0.5, 2.5), (2.2, 0.4, 1.25))
	s += box("back_box", "FFFFF020", (3, 0.4, 1.5), (0, 2.5, 0.75))
	return s

def scene_l1():
	# 16 coloured point lights over a grid of spheres. Tests the light loop and range cut-off.
	s = "// L1: 16 coloured point lights over a grid of spheres and a floor.\n"
	s += floor("floor", 20, "FFC0C0C0")
	for y in range(5):
		for x in range(5):
			s += sphere(f"s{x}{y}", "FFE0E0E0", 0.6, (-6 + 3 * x, -6 + 3 * y, 0.6))
	for i in range(16):
		x, y = i % 4, i // 4
		pos = (-4.5 + 3 * x, -4.5 + 3 * y, 1.5)
		s += point_light(f"light{i}", PALETTE[i], pos, 4.0)
		s += marker(f"marker{i}", PALETTE[i], pos)
	return s

def scene_l2():
	# 8 spot lights plus a directional light, with transparent objects to exercise the alpha K-buffer path.
	s = "// L2: 8 spot lights plus a directional light, with transparent objects.\n"
	s += floor("floor", 20, "FFA0A0A0")
	s += dir_light("sun", "FF404040", (-5, -8, 10))
	for i in range(8):
		ang = i * math.tau / 8
		pos = (6 * math.cos(ang), 6 * math.sin(ang), 5)
		target = (2.5 * math.cos(ang), 2.5 * math.sin(ang), 0)
		s += spot_light(f"spot{i}", PALETTE[i], pos, target, (20, 35), 15)
		s += marker(f"marker{i}", PALETTE[i], pos)
	for i in range(8):
		ang = i * math.tau / 8 + math.tau / 16
		s += sphere(f"glass{i}", "80E0E0FF", 0.7, (3 * math.cos(ang), 3 * math.sin(ang), 0.7))
	s += box("glass_wall", "60FFFFFF", (6, 0.1, 2), (0, 0, 1))
	s += box("solid_core", "FFE0E0E0", (1, 1, 2), (0, 1.5, 1))
	return s

def scene_s1():
	# A shadow-casting spot light over a field of boxes.
	s = "// S1: A shadow-casting spot light over a box field.\n"
	s += floor("floor", 20, "FFB0B0B0")
	for y in range(7):
		for x in range(7):
			h = 0.4 + ((x * 7 + y * 3) % 5) * 0.3
			s += box(f"b{x}{y}", "FFE0E0E0", (0.5, 0.5, h), (-4.5 + 1.5 * x, -4.5 + 1.5 * y, h / 2))
	s += spot_light("spot", "FFFFFFFF", (3, -3, 8), (0, 0, 0), (30, 45), 20, shadow=1)
	return s

def scene_s2():
	# A point light inside a room with pillars. Every wall and the floor should show pillar shadows.
	s = "// S2: A shadow-casting point light inside a room with pillars. The room has no ceiling so the camera can look in.\n"
	s += box("floor", "FFB0B0B0", (12, 12, 0.2), (0, 0, -0.1))
	s += box("wall_n", "FFD0D0D0", (12, 0.2, 4), (0, 6, 2))
	s += box("wall_s", "FFD0D0D0", (12, 0.2, 4), (0, -6, 2))
	s += box("wall_e", "FFD0D0D0", (0.2, 12, 4), (6, 0, 2))
	s += box("wall_w", "FFD0D0D0", (0.2, 12, 4), (-6, 0, 2))
	for i in range(8):
		ang = i * math.tau / 8
		s += box(f"pillar{i}", "FF8080FF", (0.4, 0.4, 3.5), (3 * math.cos(ang), 3 * math.sin(ang), 1.75))
	s += box("low_block", "FFFF8080", (0.8, 0.8, 0.8), (1.2, 0.5, 0.4))
	s += point_light("bulb", "FFFFFFFF", (0, 0, 2), 20, shadow=1)
	return s

def scene_s3():
	# About 2000 objects lit by 1 directional and 3 point shadow lights. Used for single-pass performance.
	s = "// S3: 1 directional + 3 point shadow lights over about 2000 boxes.\n"
	s += floor("floor", 60, "FFA0A0A0")
	n = 45
	for y in range(n):
		for x in range(n):
			h = 0.3 + ((x * 13 + y * 7) % 9) * 0.15
			s += box(f"b{x}_{y}", "FFE0E0E0", (0.4, 0.4, h), (-22 + x, -22 + y, h / 2))
	s += dir_light("sun", "FF808080", (-10, -20, 30), shadow=1)
	for i, (pos, col) in enumerate([((-8.5, -8.5, 4), "FFFF8080"), ((8.5, -4.5, 4), "FF80FF80"), ((0.5, 10.5, 4), "FF8080FF")]):
		s += point_light(f"lamp{i}", col, pos, 15, shadow=1)
	return s

def scene_s4():
	# A 1 km terrain-like plane with small objects near the origin. Used for cascade quality, shimmer, and pixelation.
	s = "// S4: A 1 km plane with small objects near the camera, lit by a shadow-casting directional light.\n"
	s += floor("ground", 1000, "FF70A070")
	for i in range(40):
		ang = i * 2.39996
		r = 0.5 + 0.35 * i
		h = 0.2 + (i % 5) * 0.2
		s += box(f"rock{i}", "FFC0B090", (0.3, 0.3, h), (r * math.cos(ang), r * math.sin(ang), h / 2))
	for i in range(20):
		ang = i * math.tau / 20
		s += box(f"far{i}", "FFB0B0B0", (2, 2, 6), (80 * math.cos(ang), 80 * math.sin(ang), 3))
	s += box("fence", "FF806040", (10, 0.05, 1), (0, -3, 0.5))
	s += dir_light("sun", "FFFFFFFF", (-30, -50, 60), shadow=1)
	return s

def scene_s5():
	# A static box field with one spinning caster, lit by a directional and a point shadow light. Used to check shadow view caching.
	s = "// S5: Static casters plus one spinning caster. Only views that see the spinning caster should be re-rendered.\n"
	s += floor("floor", 30, "FFA0A0A0")
	for y in range(8):
		for x in range(8):
			s += box(f"b{x}_{y}", "FFE0E0E0", (0.5, 0.5, 1.0), (-7 + 2 * x, -7 + 2 * y, 0.5))
	s += "*Box spinner FFFF8040\n{\n\t*Data {4 0.4 0.4}\n\t*RootAnimation {*Style {Continuous} *Period {1.6} *AngVelocity {0 0 1}}\n\t*o2w {*pos {0 0 3}}\n}\n"
	s += dir_light("sun", "FF808080", (-10, -20, 30), shadow=1)
	s += point_light("lamp", "FFFFE0C0", (6, 6, 5), 25, shadow=1)
	return s

write("b0_baseline.ldr", scene_b0())
write("l1_point_lights.ldr", scene_l1())
write("l2_spot_lights_alpha.ldr", scene_l2())
write("s1_spot_shadow.ldr", scene_s1())
write("s2_point_shadow_room.ldr", scene_s2())
write("s3_many_shadow_lights.ldr", scene_s3())
write("s4_large_terrain.ldr", scene_s4())
write("s5_shadow_cache.ldr", scene_s5())
