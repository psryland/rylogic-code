using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Rylogic.LDraw;
using Rylogic.Maths;

namespace Rylogic.LDraw
{
	public static class Builder_
	{
		//public static void Mesh(this LdrBuilder ldr, string name, Colour32 colour, View3d.EGeom geom, IList<View3d.Vertex> verts, IList<ushort>? faces = null, IList<ushort>? lines = null, IList<ushort>? tetra = null, bool generate_normals = false, v4? position = null)
		//{
		//	ldr.Append("*Mesh ", name, " ", colour, " {\n");
		//	if ((geom & View3d.EGeom.Vert) != 0) ldr.Append("*Verts {").Append(verts.Select(x => Ldr.Vec3(x.m_pos))).Append("}\n");
		//	if ((geom & View3d.EGeom.Norm) != 0) ldr.Append("*Normals {").Append(verts.Select(x => Ldr.Vec3(x.m_norm))).Append("}\n");
		//	if ((geom & View3d.EGeom.Colr) != 0) ldr.Append("*Colours {").Append(verts.Select(x => Ldr.Col(x.m_col))).Append("}\n");
		//	if ((geom & View3d.EGeom.Tex0) != 0) ldr.Append("*TexCoords {").Append(verts.Select(x => Ldr.Vec2(x.m_uv))).Append("}\n");
		//	if (faces != null) { Debug.Assert(faces.All(i => i >= 0 && i < verts.Count)); ldr.Append("*Faces {").Append(faces).Append("}\n"); }
		//	if (lines != null) { Debug.Assert(lines.All(i => i >= 0 && i < verts.Count)); ldr.Append("*Lines {").Append(lines).Append("}\n"); }
		//	if (tetra != null) { Debug.Assert(tetra.All(i => i >= 0 && i < verts.Count)); ldr.Append("*Tetra {").Append(tetra).Append("}\n"); }
		//	if (generate_normals) ldr.Append("*GenerateNormals\n");
		//	if (position != null) ldr.Append(Ldr.Position(position.Value));
		//	ldr.Append("}\n");
		//}
	}
}
