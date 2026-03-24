#ifndef PR_HLSL_BOUNDING_BOX_HLSLI
#define PR_HLSL_BOUNDING_BOX_HLSLI
#include "pr/hlsl/core.hlsli"

#ifdef __cplusplus
namespace pr::hlsl {
#endif

// Axis aligned bounding box (centre + half-extents)
struct BBox
{
	float4 centre;
	float4 radius;
};

// An empty bounding box (negative radius means no content)
inline BBox BBox_Reset()
{
	BBox bbox;
	bbox.centre = float4(0, 0, 0, 1);
	bbox.radius = float4(-1, -1, -1, 0);
	return bbox;
}

// Create a bounding box from centre and half-extents
inline BBox BBox_Create(float4 centre, float4 radius)
{
	BBox bbox;
	bbox.centre = centre;
	bbox.radius = radius;
	return bbox;
}

// Create a bounding box from min and max corners (w components preserved from mn)
inline BBox BBox_FromMinMax(float4 mn, float4 mx)
{
	BBox bbox;
	bbox.centre = (mn + mx) * 0.5f;
	bbox.centre.w = 1;
	bbox.radius = (mx - mn) * 0.5f;
	bbox.radius.w = 0;
	return bbox;
}

// True if the bounding box has no content
inline bool BBox_IsEmpty(BBox bbox)
{
	return bbox.radius.x < 0 || bbox.radius.y < 0 || bbox.radius.z < 0;
}

// Lower corner (w = 1)
inline float4 BBox_Min(BBox bbox)
{
	return float4((bbox.centre - bbox.radius).xyz, 1);
}

// Upper corner (w = 1)
inline float4 BBox_Max(BBox bbox)
{
	return float4((bbox.centre + bbox.radius).xyz, 1);
}

// Expand a bounding box to include a point
inline BBox BBox_Grow(BBox bbox, float4 pt)
{
	if (BBox_IsEmpty(bbox))
	{
		bbox.centre = float4(pt.xyz, 1);
		bbox.radius = float4(0, 0, 0, 0);
		return bbox;
	}

	float4 mn = min(bbox.centre - bbox.radius, pt);
	float4 mx = max(bbox.centre + bbox.radius, pt);
	bbox.centre = (mn + mx) * 0.5f;
	bbox.centre.w = 1;
	bbox.radius = (mx - mn) * 0.5f;
	bbox.radius.w = 0;
	return bbox;
}

// Union of two bounding boxes
inline BBox BBox_Union(BBox a, BBox b)
{
	if (BBox_IsEmpty(a)) return b;
	if (BBox_IsEmpty(b)) return a;

	float4 mn = min(a.centre - a.radius, b.centre - b.radius);
	float4 mx = max(a.centre + a.radius, b.centre + b.radius);
	BBox bbox;
	bbox.centre = (mn + mx) * 0.5f;
	bbox.centre.w = 1;
	bbox.radius = (mx - mn) * 0.5f;
	bbox.radius.w = 0;
	return bbox;
}

// True if two bounding boxes overlap
inline bool BBox_IsIntersection(BBox a, BBox b)
{
	float4 d = abs(a.centre - b.centre);
	float4 r = a.radius + b.radius;
	return d.x <= r.x && d.y <= r.y && d.z <= r.z;
}

// Transform a bounding box by a row-major float4x4 (row-vector convention).
// The radius is recomputed by projecting the rotated half-extents onto each target axis.
inline BBox BBox_Transform(BBox bbox, float4x4 a2b)
{
	// Transpose so that absM[j] contains the j-th column of the abs rotation.
	// Then dot(absM[j], radius) gives the new half-extent along target axis j.
	float3x3 absM = transpose(float3x3(abs(a2b[0].xyz), abs(a2b[1].xyz), abs(a2b[2].xyz)));

	BBox result;
	result.centre = mul(bbox.centre, a2b);
	result.radius = float4(
		dot(absM[0], bbox.radius.xyz),
		dot(absM[1], bbox.radius.xyz),
		dot(absM[2], bbox.radius.xyz),
		0);

	return result;
}

#ifdef __cplusplus
}
#endif
#endif
