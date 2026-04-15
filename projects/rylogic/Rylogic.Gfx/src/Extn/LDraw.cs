using System;
using System.Runtime.InteropServices;
using Rylogic.Collision;
using Rylogic.Common;
using Rylogic.Maths;

namespace Rylogic.LDraw
{
	public static class Builder_
	{
		public static LdrCollisionShape CollisionShape(this Builder builder, Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrCollisionShape();
			builder.Objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
	}

	public class LdrCollisionShape : LdrBase<LdrCollisionShape>
	{
		/// <summary>Add a collision shape from a contiguous byte buffer containing the shape data</summary>
		public LdrCollisionShape shape(byte[] data, int offset = 0)
		{
			using var pin = GCHandle_.Alloc(data, GCHandleType.Pinned);
			var base_ptr = pin.Handle.AddrOfPinnedObject() + offset;
			var header = Marshal.PtrToStructure<Shape>(base_ptr);

			switch (header.m_type)
			{
				case EShape.Sphere:
				{
					var s = Marshal.PtrToStructure<ShapeSphere>(base_ptr);
					Sphere().sphere(s.m_radius).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape.Box:
				{
					var s = Marshal.PtrToStructure<ShapeBox>(base_ptr);
					Box().box(2f * s.m_radius).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape.Triangle:
				{
					var s = Marshal.PtrToStructure<ShapeTriangle>(base_ptr);
					Triangle().tri(s.m_v.x, s.m_v.y, s.m_v.z).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape.Line:
				{
					var s = Marshal.PtrToStructure<ShapeLine>(base_ptr);
					Cylinder().cylinder(2 * s.m_radius, s.m_thickness).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape.Polytope:
				{
					var poly = new ShapePolytopeData(data);
					var tri = Triangle();
					for (int i = 0; i != poly.FaceCount; ++i)
					{
						var face = poly.Face(i);
						var a = poly.Vertex(face.m_i0);
						var b = poly.Vertex(face.m_i1);
						var c = poly.Vertex(face.m_i2);
						tri.tri(a, b, c);
					}
					tri.o2w(poly.Header.m_base.m_s2p);
					break;
				}
				case EShape.Array:
				{
					var arr = new ShapeArrayData(data);
					var grp = Group();
					for (int i = 0; i != arr.NumShapes; ++i)
					{
						var (child_offset, _) = arr.ChildShapeRange(i);
						var child = new LdrCollisionShape();
						grp.Objects.Add(child);
						child.shape(data, child_offset);
					}
					grp.o2w(arr.Header.m_base.m_s2p);
					break;
				}
				default:
				{
					throw new Exception($"Unknown shape type: {header.m_type}");
				}
			}
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			// Wrap child shapes in a group so name/colour/o2w are applied
			res.Write(EKeyword.Group, m_name, m_colour, () =>
			{
				base.WriteTo(res);
			});
		}
	}
}
