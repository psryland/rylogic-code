#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import unittest
from rylogic.ldraw.ldraw import *

class TestLDraw(unittest.TestCase):
	def test_ldraw(self):
		builder = Builder()
		ldr_group = builder.Group("MyGroup", 0xFF0000FF)
		ldr_points = ldr_group.Points("MyPoints", 0xFF00FF00)
		ldr_points.style(EPointStyle.Circle)
		ldr_points.pt(Vec3(0, 0, 0), 0xFF0000FF)
		ldr_points.pt(Vec3(1, 1, 1), 0xFF00FF00)
		ldr_points.pt(Vec3(2, 2, 2), 0xFFFF0000)
		ldr_group.pos(Vec3(1,1,1))
		ldr_cmd = builder.Command()
		ldr_cmd.transform_object('MyPoints', Mat4.Identity())
		s = builder.ToString()
		b = builder.ToBytes()

	def test_flipbook(self):
		# The flipbook modifier is written before the frames it controls
		builder = Builder()
		grp = builder.Group("fb").flipbook("PingPong", 4)
		grp.Box("f0").dim(1)
		grp.Box("f1").dim(2)
		self.assertEqual(builder.ToString(), "*Group fb {*Flipbook {*Style {PingPong} *FrameRate {4.0}} *Box f0 {*Data {1.0 1.0 1.0}} *Box f1 {*Data {2.0 2.0 2.0}}}")
		self.assertGreater(len(builder.ToBytes()), 0)
