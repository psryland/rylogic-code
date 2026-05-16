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

	// True if the bounding box has no content
	bool IsEmpty()
	{
		return radius.x < 0 || radius.y < 0 || radius.z < 0;
	}

	// Lower corner (w = 1)
	float4 Min()
	{
		return float4((centre - radius).xyz, 1);
	}

	// Upper corner (w = 1)
	float4 Max()
	{
		return float4((centre + radius).xyz, 1);
	}

	// Return a bounding box expanded to include a point
	BBox Grow(float4 pt)
	{
		BBox bbox;
		if (IsEmpty())
		{
			bbox.centre = float4(pt.xyz, 1);
			bbox.radius = float4(0, 0, 0, 0);
			return bbox;
		}

		float4 mn = min(centre - radius, pt);
		float4 mx = max(centre + radius, pt);
		bbox.centre = (mn + mx) * 0.5f;
		bbox.centre.w = 1;
		bbox.radius = (mx - mn) * 0.5f;
		bbox.radius.w = 0;
		return bbox;
	}

	// Return the union of this bounding box and another
	BBox Union(BBox b)
	{
		if (IsEmpty())
			return b;

		BBox bbox;
		bbox.centre = centre;
		bbox.radius = radius;
		if (b.IsEmpty())
			return bbox;

		float4 mn = min(centre - radius, b.centre - b.radius);
		float4 mx = max(centre + radius, b.centre + b.radius);
		bbox.centre = (mn + mx) * 0.5f;
		bbox.centre.w = 1;
		bbox.radius = (mx - mn) * 0.5f;
		bbox.radius.w = 0;
		return bbox;
	}

	// True if this bounding box overlaps another
	bool IsIntersection(BBox b)
	{
		float4 d = abs(centre - b.centre);
		float4 r = radius + b.radius;
		return d.x <= r.x && d.y <= r.y && d.z <= r.z;
	}

	// Transform this bounding box by a row-major float4x4 (row-vector convention).
	// The radius is recomputed by projecting the rotated half-extents onto each target axis.
	BBox Transform(float4x4 a2b)
	{
		// Transpose so that absM[j] contains the j-th column of the abs rotation.
		// Then dot(absM[j], radius) gives the new half-extent along target axis j.
		float3x3 absM = transpose(float3x3(abs(a2b[0].xyz), abs(a2b[1].xyz), abs(a2b[2].xyz)));

		BBox result;
		result.centre = mul(centre, a2b);
		result.radius = float4(
			dot(absM[0], radius.xyz),
			dot(absM[1], radius.xyz),
			dot(absM[2], radius.xyz),
			0);

		return result;
	}
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

#ifdef __cplusplus
}
#endif
#endif
