//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
//
// Perform a ray vs. model test on all verts, edges, faces in a model
//
// Algorithm:
//  For each Frame:
//     Constants { rays[16], ray_count, snap_mode, snap_distance, }
//     For each nugget:
//         Constants { o2w, instance_pointer }
//         Use a vertex shader to transform MS verts to world space.
//         Use geometry shaders to perform Ray Vs. Prim tests
//         Use a null pixel shader and set the stream output stage target to a buffer.
//         The format of the SO target buffer is:
//            float4 ws_intercept
//            float distance from ray origin to intercept
//            uint ray index
//            uint2 instance_pointer (x=hi,y=lo)
//  Use CopyStructureCount to read the number of intercepts written to the stream output buffer.
//  Use CopyResource to copy the intercepts buffer to a staging buffer

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/geometry.hlsli"
#include "pr/hlsl/intercept.hlsli"
#include "pr/hlsl/closest_point.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/ray_cast/ray_cast_cbuf.hlsli"

// Skinned Meshes
StructuredBuffer<Mat4x4> m_pose : reg(t4);
StructuredBuffer<Skinfluence> m_skin : reg(t5);

#include "view3d-12/src/shaders/hlsl/skinned/skinned.hlsli"

// Geometry shader input format
struct GSIn_RayCast
{
	float4 ws_vert : WSVertex;
};
struct GSOut_RayCast
{
	float4 ws_intercept : WSIntercept;
	float4 ws_normal : WSNormal;
	int snap_type : SnapType;
	int ray_index : RayIndex;
	uint2 inst_ptr : InstPtr;
};

// Additional geometry locations to snap to
struct SnapPoint
{
	float4 vert;
	int snap_type;
};
struct SnapEdge
{
	float4 vert;
	float4 edge;
	int snap_type;
};

// Default VS
GSIn_RayCast VSDefault(VSIn In)
{
	GSIn_RayCast Out = (GSIn_RayCast) 0;

	// Transform
	float4 os_vert = mul(In.vert, m_m2o);
	float4 os_norm = mul(In.norm, m_m2o);
	
	if (IsSkinned(m_flags))
	{
		os_vert = SkinVertex(m_pose, m_skin[In.idx0.x], os_vert);
		os_norm = SkinNormal(m_pose, m_skin[In.idx0.x], os_norm);
	}

	Out.ws_vert = mul(os_vert, m_o2w);

	return Out;
}

// Check if 'pt_ws' is within snap distance of 'target_ws'
bool DoesSnap(float4 pt_ws, float depth, float4 target_ws, int snap_mode, float snap_distance)
{
	float4 sep = pt_ws - target_ws;
	float distance = HasFlag(snap_mode, ESnapMode_Perspective) ? depth * snap_distance : snap_distance;
	return length(sep) < distance;
}

void RayCastFace(triangle GSIn_RayCast In[3], inout PointStream<GSOut_RayCast> OutStream)
{
	// Triangle verts
	float4 v0 = In[0].ws_vert;
	float4 v1 = In[1].ws_vert;
	float4 v2 = In[2].ws_vert;
	
	static const int SnapPointCount = 7;
	SnapPoint points[SnapPointCount] = {
		{v0, ESnapType_Vert},
		{v1, ESnapType_Vert},
		{v2, ESnapType_Vert},
		{(v0 + v1) / 2, ESnapType_EdgeMiddle},
		{(v1 + v2) / 2, ESnapType_EdgeMiddle},
		{(v2 + v0) / 2, ESnapType_EdgeMiddle},
		{(v0 + v1 + v2) / 3, ESnapType_FaceCentre},
	};
	static const int SnapEdgeCount = 3;
	SnapEdge edges[SnapEdgeCount] = {
		{v0, v1 - v0, ESnapType_Edge},
		{v1, v2 - v1, ESnapType_Edge},
		{v2, v0 - v2, ESnapType_Edge},
	};

	for (int i = 0; i != m_ray_count; ++i)
	{
		float4 origin       = m_rays[i].ws_origin;
		float4 direction    = m_rays[i].ws_direction;
		float snap_distance = m_rays[i].m_snap_distance;
		int snap_mode       = m_rays[i].m_snap_mode;
		
		// Find the closest point on the triangle to the ray
		bool face_intercept;
		float4 para = ClosestPoint_RayToTriangle(origin, direction, v0, v1, v2, face_intercept);

		// Find the closest point
		float4 pt = origin + para.w * direction;
		float depth = dot(pt - origin, direction);
		if (depth <= 0)
			continue; // behind the ray origin

		int j, jend;
		float4 intercept;
		float dist = float_max;
		int snap_type = ESnapType_None;
		
		// See if we should snap to verts before snapping to edges or faces
		jend = HasFlag(snap_mode, ESnapMode_Vert) && snap_distance != 0 ? SnapPointCount : 0;
		for (j = 0; j != jend; ++j)
		{
			float4 target = points[j].vert;
			float d = distance(pt.xyz, target.xyz);
			if (d < dist && DoesSnap(pt, depth, target, snap_mode, snap_distance))
			{
				intercept = target;
				snap_type = points[j].snap_type;
				dist = d;
			}
		}

		// If not snapped to a vert, try snapping to the edge
		jend = HasFlag(snap_mode, ESnapMode_Edge) && snap_distance != 0 && snap_type == ESnapType_None ? SnapEdgeCount : 0;
		for (j = 0; j != jend; ++j)
		{
			float4 target = edges[j].vert + saturate(ClosestPoint_PointVsRay(pt, edges[j].vert, edges[j].edge)) * edges[j].edge;
			float d = distance(pt.xyz, target.xyz);
			if (d < dist && DoesSnap(pt, depth, target, snap_mode, snap_distance))
			{
				intercept = target;
				snap_type = edges[j].snap_type;
				dist = d;
			}
		}

		// If not snapped to a vert or edge, try snapping to the face
		if (snap_type == ESnapType_None && HasFlag(snap_mode, ESnapMode_Face))
		{
			// If the point is within the triangle, snap to the face
			if (face_intercept)
			{
				intercept = pt;
				snap_type = ESnapType_Face;
			}
		}

		// Output an intercept
		if (snap_type != ESnapType_None)
		{
			float4 normal = NormaliseOrZero(float4(cross(v1.xyz - v0.xyz, v2.xyz - v0.xyz), 0));

			GSOut_RayCast Out = (GSOut_RayCast) 0;
			Out.ws_intercept = float4(intercept.xyz, depth);
			Out.ws_normal = snap_type == ESnapType_Face ? normal : float4(0, 0, 0, 0);
			Out.snap_type = snap_type;
			Out.ray_index = i;
			Out.inst_ptr = m_inst_ptr;
			OutStream.Append(Out);
		}
	}
}
void RayCastEdge(line GSIn_RayCast In[2], inout PointStream<GSOut_RayCast> OutStream)
{
	float4 v0 = In[0].ws_vert;
	float4 v1 = In[1].ws_vert;
	float4 edge = v1 - v0;

	static const int SnapPointCount = 3;
	SnapPoint points[SnapPointCount] = {
		{ v0, ESnapType_Vert },
		{ v1, ESnapType_Vert },
		{ (v0 + v1) / 2, ESnapType_EdgeMiddle },
	};

	for (int i = 0; i != m_ray_count; ++i)
	{
		float4 origin       = m_rays[i].ws_origin;
		float4 direction    = m_rays[i].ws_direction;
		float snap_distance = m_rays[i].m_snap_distance;
		int snap_mode       = m_rays[i].m_snap_mode;

		// Can't snap to points/edges when the snap distance is 0
		if (snap_distance == 0)
			continue;

		// Perform a ray vs. line segment test to get the closest point to the edge
		float2 para = ClosestPoint_RayToRay(v0, edge, origin, direction);
		para.x = saturate(para.x); // Clamp to edge segment

		// Find the closest point
		float4 pt = origin + para.y * direction;
		float depth = dot(pt - origin, direction);
		if (depth <= 0)
			continue; // behind the ray origin

		int j, jend;
		float4 intercept;
		float dist = float_max;
		int snap_type = ESnapType_None;

		// See if we should snap to verts before snapping to the edge
		jend = HasFlag(snap_mode, ESnapMode_Vert) ? SnapPointCount : 0;
		for (j = 0; j != jend; ++j)
		{
			float4 target = points[j].vert;
			float d = distance(pt.xyz, target.xyz);
			if (d < dist && DoesSnap(pt, depth, target, snap_mode, snap_distance) )
			{
				intercept = target;
				snap_type = points[j].snap_type;
				dist = d;
			}
		}

		// If not snapped to a vert, try snapping to the edge
		if (snap_type == ESnapType_None && HasFlag(snap_mode, ESnapMode_Edge))
		{
			float4 target = v0 + para.x * edge;
			if (DoesSnap(pt, depth, target, snap_mode, snap_distance))
			{
				intercept = target;
				snap_type = ESnapType_Edge;
			}
		}
		
		// Output an intercept
		if (snap_type != ESnapType_None)
		{
			GSOut_RayCast Out = (GSOut_RayCast) 0;
			Out.ws_intercept = float4(intercept.xyz, depth);
			Out.ws_normal = float4(0, 0, 0, 0);
			Out.snap_type = snap_type;
			Out.ray_index = i;
			Out.inst_ptr = m_inst_ptr;
			OutStream.Append(Out);
		}
	}
}
void RayCastVert(point GSIn_RayCast In[1], inout PointStream<GSOut_RayCast> OutStream)
{
	float4 v0 = In[0].ws_vert;

	for (int i = 0; i != m_ray_count; ++i)
	{
		float4 origin       = m_rays[i].ws_origin;
		float4 direction    = m_rays[i].ws_direction;
		float snap_distance = m_rays[i].m_snap_distance;
		int snap_mode       = m_rays[i].m_snap_mode;

		// Can't snap to points/edges when the snap distance is 0
		if (snap_distance == 0)
			continue;
		
		// Perform a ray vs. point test
		float para = ClosestPoint_PointVsRay(v0, origin, direction);

		// Find the closest point
		float4 pt = origin + para * direction;
		float depth = dot(pt - origin, direction);
		if (depth <= 0)
			continue; // behind the ray origin

		// Output an intercept
		if (DoesSnap(pt, depth, v0, snap_mode, snap_distance))
		{
			float4 intercept = v0;

			GSOut_RayCast Out = (GSOut_RayCast) 0;
			Out.ws_intercept = float4(intercept.xyz, depth);
			Out.ws_normal = float4(0, 0, 0, 0);
			Out.snap_type = ESnapType_Vert;
			Out.ray_index = i;
			Out.inst_ptr = m_inst_ptr;
			OutStream.Append(Out);
		}
	}
}

// Vertex shader
#ifdef PR_RDR_VSHADER_ray_cast
GSIn_RayCast main(VSIn In)
{
	GSIn_RayCast Out = VSDefault(In);
	return Out;
}
#endif

// Geometry shaders
#ifdef PR_RDR_GSHADER_ray_cast_face
[maxvertexcount(MaxRays)]
void main(triangle GSIn_RayCast In[3], inout PointStream<GSOut_RayCast> OutStream)
{
	RayCastFace(In, OutStream);
}
#endif
#ifdef PR_RDR_GSHADER_ray_cast_edge
[maxvertexcount(MaxRays)]
void main(line GSIn_RayCast In[2], inout PointStream<GSOut_RayCast> OutStream)
{
	RayCastEdge(In, OutStream);
}
#endif
#ifdef PR_RDR_GSHADER_ray_cast_vert
[maxvertexcount(MaxRays)]
void main(point GSIn_RayCast In[1], inout PointStream<GSOut_RayCast> OutStream)
{
	RayCastVert(In, OutStream);
}
#endif
