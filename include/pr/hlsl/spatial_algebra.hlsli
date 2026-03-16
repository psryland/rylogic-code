//*********************************************
// HLSL Spatial Algebra
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
// Spatial algebra utilities for physics compute shaders.
// Provides the core operations needed for rigid body integration:
//   - Rodrigues rotation (exponential map from axis-angle to rotation matrix)
//   - Symmetric 3x3 inverse inertia tensor reconstruction and rotation
//   - Gram-Schmidt orthonormalization for rotation matrices
//   - Spatial vector operations (InertiaInv × momentum → velocity)
//
// Matrix convention (row-vector / DirectX-style):
//   - C++ m4x4 stores columns contiguously as x, y, z, w members.
//   - HLSL 'row_major float4x4' maps each row to one C++ column.
//   - Therefore HLSL row i = C++ column i = the i-th basis vector.
//   - For vector transforms: mul(v, M) = M_math * v  (row-vector multiply)
//   - For matrix compose:   mul(B, A) = (A_math * B_math)^T = (A * B) in row convention
//   - All float3x3 rotation matrices here follow this same convention:
//     rows are basis vectors (the transpose of the mathematical rotation matrix).
#ifndef PR_HLSL_SPATIAL_ALGEBRA_HLSLI
#define PR_HLSL_SPATIAL_ALGEBRA_HLSLI

// Rodrigues' rotation formula: exponential map from axis-angle vector to rotation matrix.
// 'axis_angle' encodes both the rotation axis (direction) and angle (magnitude).
// Returns the rotation matrix in row-vector convention (rows = basis vectors = R^T).
// This matches the C++ m3x4::Rotation() layout where x/y/z members are column vectors.
//
// To apply the rotation to a vector: mul(v, R) — row-vector convention.
// To compose rotations: mul(R_old, R_new) — applies R_new after R_old.
//
// The formula is:
//   R_math = I + (sin θ / θ) [v]× + ((1 - cos θ) / θ²) [v]×²
// where θ = |axis_angle|, [v]× is the skew-symmetric cross-product matrix.
// We return transpose(R_math) for our row-vector convention.
float3x3 rodrigues_rotation(float3 axis_angle)
{
	float theta_sq = dot(axis_angle, axis_angle);

	// For small angles, use Taylor expansion to avoid numerical issues.
	// sin(θ)/θ → 1 - θ²/6 + ... and (1 - cos(θ))/θ² → 1/2 - θ²/24 + ...
	float s, c;
	if (theta_sq < 1e-8f)
	{
		s = 1.0f - theta_sq / 6.0f;  // sin(θ)/θ
		c = 0.5f - theta_sq / 24.0f; // (1 - cos(θ))/θ²
	}
	else
	{
		float theta = sqrt(theta_sq);
		s = sin(theta) / theta;
		c = (1.0f - cos(theta)) / theta_sq;
	}

	float x = axis_angle.x;
	float y = axis_angle.y;
	float z = axis_angle.z;

	// Build the standard rotation matrix R_math, then transpose it.
	// R_math = I + s*K + c*K² where K is the skew-symmetric cross-product matrix.
	float3x3 R;
	R[0][0] = 1.0f - c * (y * y + z * z);
	R[0][1] = s * (-z) + c * (x * y);
	R[0][2] = s * ( y) + c * (x * z);
	R[1][0] = s * ( z) + c * (x * y);
	R[1][1] = 1.0f - c * (x * x + z * z);
	R[1][2] = s * (-x) + c * (y * z);
	R[2][0] = s * (-y) + c * (x * z);
	R[2][1] = s * ( x) + c * (y * z);
	R[2][2] = 1.0f - c * (x * x + y * y);

	// Return R^T: rows become basis vectors, matching our row-vector convention
	return transpose(R);
}

// Build a symmetric 3x3 matrix from diagonal and off-diagonal (product) terms.
// This reconstructs the inverse inertia tensor from its compact storage form.
//   diag = {Ixx, Iyy, Izz}
//   prod = {Ixy, Ixz, Iyz}
float3x3 build_symmetric_3x3(float3 diag, float3 prod)
{
	return float3x3(
		diag.x, prod.x, prod.y,
		prod.x, diag.y, prod.z,
		prod.y, prod.z, diag.z
	);
}

// Rotate an inverse inertia tensor from one frame to another.
// Given I⁻¹ in frame A and rotation 'rot' (A→B, in row-vector convention: rows = basis vectors),
// compute I⁻¹ in frame B:
//   I⁻¹_B = R * I⁻¹_A * Rᵀ
//
// Since 'rot' stores R^T (rows = basis vectors), the formula becomes:
//   I⁻¹_B = transpose(rot) * I⁻¹_A * rot
// The result is symmetrized to counteract floating-point drift after repeated rotations.
float3x3 rotate_inertia_inv(float3x3 iinv, float3x3 rot)
{
	// rot = R^T in row-vector convention, so:
	//   transpose(rot) = R
	//   transpose(rot) * iinv * rot = R * I⁻¹ * R^T ✓
	float3x3 temp = mul(iinv, rot);             // I⁻¹_A * R^T
	float3x3 result = mul(transpose(rot), temp); // R * (I⁻¹_A * R^T)

	// Symmetrize to fix floating-point drift
	result[0][1] = result[1][0] = 0.5f * (result[0][1] + result[1][0]);
	result[0][2] = result[2][0] = 0.5f * (result[0][2] + result[2][0]);
	result[1][2] = result[2][1] = 0.5f * (result[1][2] + result[2][1]);
	return result;
}

// Gram-Schmidt orthonormalization of a 3x3 rotation matrix.
// After repeated small rotations, floating-point drift causes the matrix to lose
// orthonormality. This function restores it by re-orthogonalizing the basis vectors
// and renormalizing them. The X axis is taken as the reference direction.
float3x3 orthonorm3x3(float3x3 m)
{
	// Start from the X axis (row 0), normalize it
	float3 x = normalize(m[0]);

	// Make Y orthogonal to X, then normalize
	float3 y = m[1] - dot(m[1], x) * x;
	y = normalize(y);

	// Z = X × Y (guaranteed orthogonal to both)
	float3 z = cross(x, y);

	return float3x3(x, y, z);
}

// Multiply an inverse inertia by a spatial force vector to get a spatial motion vector.
// This implements: velocity = I⁻¹ × momentum
//
// When the centre of mass (com) is at the model origin (com ≈ 0), the spatial inverse
// inertia is block-diagonal:
//   [I⁻¹_c   0  ] [τ]   [I⁻¹_c * τ    ]   [ω]
//   [0     1/m  ] [f] = [(1/m) * f     ] = [v]
//
// When com ≠ 0, the full 6×6 spatial inverse inertia has off-diagonal coupling terms
// from the parallel axis theorem:
//   Io⁻¹ = [Ic⁻¹          , -Ic⁻¹ * cx           ]
//          [cx * Ic⁻¹     , 1/m - cx * Ic⁻¹ * cx  ]
// where cx is the cross-product matrix of the com offset: cx * v = cross(com, v).
//
// Applied to a spatial force (τ, f):
//   ω = Ic⁻¹ * τ - Ic⁻¹ * (com × f)
//   v = com × (Ic⁻¹ * τ) + f/m - com × (Ic⁻¹ * (com × f))
//
// Parameters:
//   iinv_3x3   — The world-space 3×3 inverse inertia (at CoM, scaled by inv_mass)
//   inv_mass   — 1/mass
//   com        — Centre of mass offset from model origin (in current frame)
//   torque     — Angular component of the spatial force (momentum.ang)
//   force      — Linear component of the spatial force (momentum.lin)
//   out_ang    — [out] Angular velocity
//   out_lin    — [out] Linear velocity
void spatial_multiply_inertia_inv(
	float3x3 iinv_3x3, float inv_mass, float3 com,
	float3 torque, float3 force,
	out float3 out_ang, out float3 out_lin)
{
	// Check if com is effectively zero (block-diagonal case)
	if (dot(com, com) < 1e-12f)
	{
		// Simple case: no coupling between angular and linear
		out_ang = mul(iinv_3x3, torque);
		out_lin = inv_mass * force;
	}
	else
	{
		// Full 6×6 spatial inverse inertia multiply.
		// Compute two intermediates to share work between ω and v:
		float3 Ic_inv_tau = mul(iinv_3x3, torque);            // Ic⁻¹ * τ
		float3 Ic_inv_cxf = mul(iinv_3x3, cross(com, force)); // Ic⁻¹ * (com × f)

		// ω = Ic⁻¹ * τ - Ic⁻¹ * (com × f) = Ic⁻¹ * (τ - com × f)
		out_ang = Ic_inv_tau - Ic_inv_cxf;

		// v = com × (Ic⁻¹ * τ) + f/m - com × (Ic⁻¹ * (com × f))
		out_lin = cross(com, Ic_inv_tau) + inv_mass * force - cross(com, Ic_inv_cxf);
	}
}

// Compute the dot product of two spatial vectors (angular + linear parts).
// Used for kinetic energy: KE = 0.5 * dot(velocity, momentum)
float spatial_dot(float3 ang_a, float3 lin_a, float3 ang_b, float3 lin_b)
{
	return dot(ang_a, ang_b) + dot(lin_a, lin_b);
}

#endif
