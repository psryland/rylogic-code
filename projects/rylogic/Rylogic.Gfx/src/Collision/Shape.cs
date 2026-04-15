namespace Rylogic.Collision;

using System;
using System.Runtime.InteropServices;
using Rylogic.Common;
using Rylogic.Maths;

public enum EShape : int
{
	// Special value to indicate the shape is a dummy object
	NoShape = -1,
	Sphere,
	Box,
	Line,
	Triangle,
	Polytope,
	Array,
}
public enum EFlags : int
{
	None = 0,
}

// Shape base. All shapes must have this as their first member.
// Native side has alignas(16) on v4/m4x4, so the struct is padded to 112 bytes.
[StructLayout(LayoutKind.Sequential)]
public struct Shape
{
	// Transform from shape space to parent shape space (or physics model space for root objects)
	public m4x4 m_s2p;        // offset 0, 64 bytes

	// A bounding box for the shape (and its children) (in shape space).
	public BBox m_bbox;        // offset 64, 32 bytes

	// The type of shape this is. One of EShape
	public EShape m_type;      // offset 96, 4 bytes

	// The physics material that this shape is made out of
	public uint m_material_id; // offset 100, 4 bytes

	// Flags for the shape. Bitwise OR of Shape::EFlags
	public EFlags m_flags;     // offset 104, 4 bytes

	// The size in bytes of this shape and its data
	public int m_size;         // offset 108, 4 bytes
	// Total: 112 bytes (16-byte aligned)
}

// A sphere collision shape
[StructLayout(LayoutKind.Sequential)]
struct ShapeSphere
{
	public Shape m_base;
	public float m_radius;
	public int m_hollow;
	public int pad0;
	public int pad1;
}

// A box collision shape
[StructLayout(LayoutKind.Sequential)]
public struct ShapeBox
{
	public Shape m_base;
	public v4 m_radius;
}

// A Line segment shape with optional thickness.
[StructLayout(LayoutKind.Sequential)]
public struct ShapeLine
{
	public Shape m_base;
	public float m_radius;    // Half-length along the Z axis. Length = 2 * m_radius.
	public float m_thickness; // Collision radius perpendicular to the Z axis. 0 = infinitely thin.
	private int pad0;
	private int pad1;
}

// A triangle collision shape.
[StructLayout(LayoutKind.Sequential)]
public struct ShapeTriangle
{
	public Shape m_base;
	public m4x4 m_v; // <x,y,z> = verts of the triangle, w = normal. Cross(w, y-x) should point toward the interior of the triangle
}

// A polytope is basically a triangle mesh with adjacency data. All polytopes are assumed to be convex.
[StructLayout(LayoutKind.Sequential)]
public struct ShapePolytope
{
	// Mesh structure types
	[StructLayout(LayoutKind.Sequential)]
	public struct Face
	{
		public byte m_i0, m_i1, m_i2;
		public byte pad;
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct Nbrs
	{
		// Neighbours are the vertices that share an edge connected to a vertex.
		// Neighbours also include an 'artificial' neighbour used to quickly link
		// to the other side of the polytope. The artificial neighbour is always
		// the first index in the list of neighbours.
		public ushort m_first; // Byte offset to the first neighbour
		public ushort m_count; // Number of neighbours
	}

	public Shape m_base;
	public int m_vert_count;
	public int m_face_count;
	public int pad0;
	public int pad1;

	// Memory layout. The following data is expected to follow this struct in memory, but is not actually part of the struct.
	// v4   m_vert[m_vert_count]
	// Face m_face[m_face_count]
	// Nbrs m_nbrs[m_vert_count]
	// byte m_nbr[sum(m_nbrs[i].m_count)]
	// byte padding[] to make the total size a multiple of 16 bytes
}

// Shape array
[StructLayout(LayoutKind.Sequential)]
public struct ShapeArray
{
	public Shape m_base;

	// The number of shapes in the array
	public int m_num_shapes;
	private int pad0, pad1, pad2; // Pad to 16 bytes

	// Followed by an array of other shape types (with different sizes):
	// ShapeBox s0;
	// ShapeSphere s1;
	// ...
}

/// <summary>
/// Typed access to a ShapePolytope and its trailing variable-length data.
/// The data is stored as a contiguous byte buffer matching the C++ memory layout.
/// </summary>
public class ShapePolytopeData
{
	private readonly byte[] m_data;
	public ShapePolytopeData(byte[] data)
	{
		m_data = data;
	}
	public ShapePolytopeData(IntPtr ptr, int size)
	{
		m_data = new byte[size];
		Marshal.Copy(ptr, m_data, 0, size);
	}
	public unsafe ShapePolytopeData(ShapePolytope* ptr)
	{
		// Create from a pointer to a ShapePolytope in unmanaged memory. Reads 'm_base.m_size' to determine the total byte count.
		m_data = new byte[ptr->m_base.m_size];
		Marshal.Copy((IntPtr)ptr, m_data, 0, m_data.Length);
	}

	/// <summary>The underlying data buffer</summary>
	public byte[] Data => m_data;

	/// <summary>The fixed-size header</summary>
	public ShapePolytope Header
	{
		get
		{
			var handle = GCHandle.Alloc(m_data, GCHandleType.Pinned);
			try { return Marshal.PtrToStructure<ShapePolytope>(handle.AddrOfPinnedObject()); }
			finally { handle.Free(); }
		}
	}

	/// <summary>Number of vertices</summary>
	public int VertCount => Header.m_vert_count;

	/// <summary>Number of faces</summary>
	public int FaceCount => Header.m_face_count;

	// Layout offsets
	private static readonly int HeaderSize = Marshal.SizeOf<ShapePolytope>();
	private static readonly int V4Size = Marshal.SizeOf<v4>();
	private static readonly int FaceSize = Marshal.SizeOf<ShapePolytope.Face>();
	private static readonly int NbrsSize = Marshal.SizeOf<ShapePolytope.Nbrs>();

	private int VertsOffset => HeaderSize;
	private int FacesOffset => VertsOffset + Header.m_vert_count * V4Size;
	private int NbrsOffset => FacesOffset + Header.m_face_count * FaceSize;
	private int NbrDataOffset => NbrsOffset + Header.m_vert_count * NbrsSize;

	/// <summary>Read vertex at the given index</summary>
	public v4 Vertex(int idx)
	{
		var ofs = VertsOffset + idx * V4Size;
		using var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
		return Marshal.PtrToStructure<v4>(pin.Handle.AddrOfPinnedObject() + ofs);
	}

	/// <summary>Read face at the given index</summary>
	public ShapePolytope.Face Face(int idx)
	{
		var ofs = FacesOffset + idx * FaceSize;
		using var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
		return Marshal.PtrToStructure<ShapePolytope.Face>(pin.Handle.AddrOfPinnedObject() + ofs);
	}

	/// <summary>Read neighbour info for a vertex</summary>
	public ShapePolytope.Nbrs NbrInfo(int vert_idx)
	{
		var ofs = NbrsOffset + vert_idx * NbrsSize;
		var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
		return Marshal.PtrToStructure<ShapePolytope.Nbrs>(pin.Handle.AddrOfPinnedObject() + ofs);
	}

	/// <summary>Read neighbour indices for a vertex</summary>
	public byte[] NbrIndices(int vert_idx)
	{
		var info = NbrInfo(vert_idx);

		// m_first is a byte offset relative to the Nbrs struct itself
		var nbrs_struct_ofs = NbrsOffset + vert_idx * NbrsSize;
		var data_ofs = nbrs_struct_ofs + info.m_first;

		var result = new byte[info.m_count];
		Array.Copy(m_data, data_ofs, result, 0, info.m_count);
		return result;
	}

	/// <summary>Copy all vertices into an array</summary>
	public v4[] CopyVerts()
	{
		var verts = new v4[Header.m_vert_count];
		using var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
		var ptr = pin.Handle.AddrOfPinnedObject() + VertsOffset;
		for (int i = 0; i != verts.Length; ++i)
		{
			verts[i] = Marshal.PtrToStructure<v4>(ptr + i * V4Size);
		}
		return verts;
	}

	/// <summary>Copy all faces into an array</summary>
	public ShapePolytope.Face[] CopyFaces()
	{
		var faces = new ShapePolytope.Face[Header.m_face_count];
		using var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
		var ptr = pin.Handle.AddrOfPinnedObject() + FacesOffset;
		for (int i = 0; i != faces.Length; ++i)
		{
			faces[i] = Marshal.PtrToStructure<ShapePolytope.Face>(ptr + i * FaceSize);
		}
		return faces;
	}
}

/// <summary>
/// Typed access to a ShapeArray and its trailing variable-length data.
/// The data is stored as a contiguous byte buffer matching the C++ memory layout.
/// </summary>
public class ShapeArrayData
{
	private readonly byte[] m_data;
	public ShapeArrayData(byte[] data)
	{
		m_data = data;
	}
	public ShapeArrayData(IntPtr ptr, int size)
	{
		m_data = new byte[size];
		Marshal.Copy(ptr, m_data, 0, size);
	}
	public unsafe ShapeArrayData(ShapeArray* ptr)
	{
		// Create from a pointer to a ShapeArray in unmanaged memory. Reads 'm_base.m_size' to determine the total byte count.
		m_data = new byte[ptr->m_base.m_size];
		Marshal.Copy((IntPtr)ptr, m_data, 0, m_data.Length);
	}

	/// <summary>The underlying data buffer</summary>
	public byte[] Data => m_data;

	/// <summary>The fixed-size header</summary>
	public ShapeArray Header
	{
		get
		{
			using var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
			return Marshal.PtrToStructure<ShapeArray>(pin.Handle.AddrOfPinnedObject());
		}
	}

	/// <summary>Number of shapes in the array</summary>
	public int NumShapes => Header.m_num_shapes;

	/// <summary>Access a shape header from the array by walking the variable-size chain</summary>
	public Shape this[int idx]
	{
		get
		{
			if (idx < 0 || idx >= Header.m_num_shapes)
				throw new IndexOutOfRangeException();

			using var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
			var base_ptr = pin.Handle.AddrOfPinnedObject();

			// First child shape starts after the ShapeArray header
			var ofs = Marshal.SizeOf<ShapeArray>();
			for (int i = 0; i != idx; ++i)
			{
				// Read the Shape header to get m_size, then advance by that many bytes
				var shape = Marshal.PtrToStructure<Shape>(base_ptr + ofs);
				ofs += shape.m_size;
			}
			return Marshal.PtrToStructure<Shape>(base_ptr + ofs);
		}
	}

	/// <summary>Get the byte offset and size of a child shape for typed access</summary>
	public (int offset, int size) ChildShapeRange(int idx)
	{
		if (idx < 0 || idx >= Header.m_num_shapes)
			throw new IndexOutOfRangeException();

		using var pin = GCHandle_.Alloc(m_data, GCHandleType.Pinned);
		var base_ptr = pin.Handle.AddrOfPinnedObject();

		var ofs = Marshal.SizeOf<ShapeArray>();
		for (int i = 0; i != idx; ++i)
		{
			var shape = Marshal.PtrToStructure<Shape>(base_ptr + ofs);
			ofs += shape.m_size;
		}
		var target = Marshal.PtrToStructure<Shape>(base_ptr + ofs);
		return (ofs, target.m_size);
	}
}