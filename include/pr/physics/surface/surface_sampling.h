//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/surface/forward.h"
#include "pr/physics/surface/surface_sampling.hlsli"

namespace pr::physics::surface
{
	static_assert(sizeof(SurfacePatch) == 96);

	// Leave room for the GPU's final grid-stride increment in its signed sample index.
	inline constexpr uint32_t MaxSampleCount = 0x7fff0000u;

	// An immutable primitive-local plan, independent of fluids and interior tetrahedralisation.
	struct Plan
	{
		std::vector<SurfacePatch> m_patches;
		uint32_t m_count = 0;
	};

	// Reject invalid spacing even for empty or zero-area shapes.
	inline void ValidateSpacing(float spacing)
	{
		if (!std::isfinite(spacing) || spacing <= 0.0f)
			throw std::runtime_error("Surface sample spacing must be finite and positive");
	}

	namespace impl
	{
		// Derive subdivisions without truncating coverage or overflowing float grid coordinates.
		inline uint32_t Divisions(double length, double spacing)
		{
			auto const count = std::max(1.0, std::ceil(length / spacing));
			if (!std::isfinite(count) || count > 1048576.0)
				throw std::runtime_error("Surface sampling grid exceeds representable subdivision count");

			return static_cast<uint32_t>(count);
		}

		// Append endpoint-inclusive segment coverage with zero normal and area, including one sample for a collapsed point.
		inline void Segment(Plan& plan, v4 a, v4 b, float spacing)
		{
			auto const delta = b - a;
			if (!IsFinite(a) || !IsFinite(b) || !IsFinite(delta))
				throw std::runtime_error("Surface segment coordinates are not representable");

			// Reject counts or steps that cannot preserve the requested coverage in the plan's representation.
			auto const length = std::hypot(double(delta.x), double(delta.y), double(delta.z));
			auto const n = Divisions(length, spacing);
			auto const count = length == 0 ? 1u : n + 1;
			auto const step = delta / static_cast<float>(n);
			if (length != 0 && (All(a + step == a) || All(b - step == b)))
				throw std::runtime_error("Surface segment spacing is below shape-local float resolution");
			if (count > MaxSampleCount - plan.m_count)
				throw std::runtime_error("Surface segment exceeds representable sample count");

			// Retain the full segment as one compact patch rather than allocating its sample positions.
			plan.m_count += count;
			plan.m_patches.push_back(SurfacePatch{
				.m_origin = a, .m_u = delta, .m_v = {}, .m_normal = {}, .m_nu = n, .m_nv = 1,
				.m_kind = 5, .m_sample_end = plan.m_count, .m_taper = 0, .m_measure = 0, .m_pad0 = 0, .m_pad1 = 0,
			});
		}

		// Append only valid positive-area patches after checking cumulative sample/resource bounds.
		inline void Append(Plan& plan, SurfacePatch patch)
		{
			if (!IsFinite(patch.m_origin) || !IsFinite(patch.m_u) || !IsFinite(patch.m_v) || !IsFinite(patch.m_normal) ||
				!std::isfinite(patch.m_measure) || patch.m_measure <= 0.0f)
				throw std::runtime_error("Surface sampling geometry or area is not representable");

			// Reject weights that GPU float arithmetic would flush to zero, including boundary and tapered-tip weights.
			auto const minimum_weight = double(patch.m_measure) / (12.0 * patch.m_nu * patch.m_nv);
			if (minimum_weight < std::numeric_limits<float>::min())
				throw std::runtime_error("Surface sample area weight is below normal float range");

			// A grid step must remain distinguishable at the supplied shape-local coordinate magnitude.
			auto const far_corner = patch.m_origin + patch.m_u + patch.m_v;
			auto const du = patch.m_u / static_cast<float>(patch.m_nu);
			auto const dv = patch.m_v / static_cast<float>(patch.m_nv);
			if (All(patch.m_origin + du == patch.m_origin) || All(patch.m_origin + dv == patch.m_origin) ||
				All(far_corner - du == far_corner) || All(far_corner - dv == far_corner))
				throw std::runtime_error("Surface sampling spacing is below shape-local float resolution");

			auto count = (uint64_t{patch.m_nu} + 1) * (uint64_t{patch.m_nv} + 1);
			if (patch.m_kind == 1 && patch.m_taper == 1.0f)
				count -= patch.m_nv;

			if (count > MaxSampleCount - plan.m_count)
				throw std::runtime_error("Surface sampling plan exceeds representable sample count");

			if (plan.m_patches.size() >= static_cast<size_t>(std::numeric_limits<int>::max()) / sizeof(SurfacePatch))
				throw std::runtime_error("Surface sampling patches exceed addressable buffer size");

			plan.m_count += static_cast<uint32_t>(count);
			patch.m_sample_end = plan.m_count;
			plan.m_patches.push_back(patch);
		}

		// Cover a right triangle with near-uniform tapered strips rather than an aspect-ratio-squared lattice.
		inline void RightTriangle(Plan& plan, v4 origin, v4 u, v4 v, v4 normal, float spacing)
		{
			auto lu = Length(u);
			auto lv = Length(v);
			if (lu == 0.0f || lv == 0.0f)
				return;

			if (lu < lv)
			{
				std::swap(u, v);
				std::swap(lu, lv);
			}
			auto const n = Divisions(2.0 * std::hypot(double(lu), double(lv)), spacing);

			// Check the complete strip population before allocating O(longest edge / spacing) records.
			auto count = uint64_t{0};
			for (uint32_t i = 0; i != n; ++i)
			{
				auto const height = lv * (static_cast<float>(n - i) / n);
				auto const m = Divisions(2.0 * height, spacing);
				count += 2 * (uint64_t{m} + 1) - (i + 1 == n ? m : 0);
			}
			if (count > MaxSampleCount - plan.m_count)
				throw std::runtime_error("Surface triangle exceeds representable sample count");

			for (uint32_t i = 0; i != n; ++i)
			{
				auto const remaining = static_cast<float>(n - i);
				auto const height = lv * (remaining / n);
				Append(plan, SurfacePatch{
					.m_origin = origin + u * (static_cast<float>(i) / n),
					.m_u = u / static_cast<float>(n),
					.m_v = v * (remaining / n),
					.m_normal = normal,
					.m_nu = 1,
					.m_nv = Divisions(2.0 * height, spacing),
					.m_kind = 1,
					.m_sample_end = 0,
					.m_taper = 1.0f / remaining,
					.m_measure = (lu / n) * height,
					.m_pad0 = 0,
					.m_pad1 = 0,
				});
			}
		}

		// Split at the longest edge's altitude, preserving original corners and each face's outward normal.
		inline void Triangle(Plan& plan, v4 a, v4 b, v4 c, v4 normal, float spacing)
		{
			if (!IsFinite(a) || !IsFinite(b) || !IsFinite(c))
				throw std::runtime_error("Surface triangle vertices must be finite");

			// The longest edge contains the perpendicular foot, including for obtuse and slender triangles.
			if (LengthSq(c - b) > LengthSq(b - a) && LengthSq(c - b) >= LengthSq(a - c))
			{
				auto old_a = a;
				a = b; b = c; c = old_a;
			}
			else if (LengthSq(a - c) > LengthSq(b - a))
			{
				auto old_b = b;
				b = a; a = c; c = old_b;
			}
			auto const edge = b - a;
			auto const length_sq = LengthSq(edge);
			auto const area = Length(Cross(edge, c - a));
			if (!std::isfinite(length_sq) || !std::isfinite(area))
				throw std::runtime_error("Surface triangle geometry exceeds float range");

			// Distinguish exact degeneracy from area/normal information lost in single-precision arithmetic.
			if (area == 0.0f)
			{
				auto const ax = double(b.x) - a.x, ay = double(b.y) - a.y, az = double(b.z) - a.z;
				auto const bx = double(c.x) - a.x, by = double(c.y) - a.y, bz = double(c.z) - a.z;
				if (ay * bz - az * by != 0.0 || az * bx - ax * bz != 0.0 || ax * by - ay * bx != 0.0)
					throw std::runtime_error("Surface triangle area is below representable float resolution");

				// Preserve longest-edge contact coverage even though this triangle has no represented area.
				Segment(plan, a, b, spacing);
				return;
			}
			if (!IsFinite(normal) || LengthSq(normal.w0()) == 0.0f)
				throw std::runtime_error("Surface triangle has positive area but no representable face normal");

			// Both halves share the face normal even if their local grid bases have opposite winding.
			auto const foot = a + edge * std::clamp(Dot3(c - a, edge) / length_sq, 0.0f, 1.0f);
			RightTriangle(plan, foot, a - foot, c - foot, normal.w0(), spacing);
			RightTriangle(plan, foot, b - foot, c - foot, normal.w0(), spacing);
		}
	}

	// Build a deterministic feature-preserving plan for one primitive; no volume geometry is required.
	// Every generated surface cell has diameter <= spacing (sphere: geodesic distance), up to float rounding.
	inline Plan BuildPlan(collision::Shape const& shape, float spacing = DefaultSpacing)
	{
		using namespace collision;
		ValidateSpacing(spacing);
		auto plan = Plan{};
		switch (shape.m_type)
		{
			case EShape::Line:
			{
				auto const& line = shape_cast<ShapeLine>(shape);
				if (!std::isfinite(line.m_hlength) || !std::isfinite(line.m_radius) || line.m_hlength < 0 || line.m_radius < 0)
					throw std::runtime_error("Surface line dimensions must be finite and non-negative");

				// A thin line retains positional coverage without inventing a unique outward normal.
				if (line.m_radius == 0)
				{
					impl::Segment(plan, v4(0, 0, -line.m_hlength, 1), v4(0, 0, line.m_hlength, 1), spacing);
					break;
				}

				// Split sphere-face patches at the equator and translate each hemisphere to its segment endpoint.
				auto const sphere = ShapeSphere(line.m_radius);
				auto const caps = BuildPlan(sphere, spacing);
				for (auto patch : caps.m_patches)
				{
					auto const side_face = patch.m_normal.z == 0;
					for (int side = 0; side != (side_face ? 2 : 1); ++side)
					{
						auto cap = patch;
						auto const positive = side_face ? side == 1 : patch.m_normal.z > 0;
						if (side_face)
						{
							auto& edge = cap.m_u.z != 0 ? cap.m_u : cap.m_v;
							edge *= 0.5f;
							cap.m_origin.z = positive ? 0.0f : -1.0f;
						}
						cap.m_kind = 3;
						cap.m_normal = v4(0, 0, positive ? line.m_hlength : -line.m_hlength, 0);
						impl::Append(plan, cap);
					}
				}

				// Fill the cylindrical side with axial and arc-length subdivisions that bound each cell diameter. The periodic angular rule
				// needs at least three subdivisions: with fewer, the normals do not cancel and do not average to the cylinder's circular second
				// moment, so a uniform pressure produces a net force and a linear pressure field produces the wrong force.
				if (line.m_hlength != 0)
				{
					impl::Append(plan, SurfacePatch{
						.m_origin = v4(0, 0, -line.m_hlength, 1), .m_u = v4(0, 0, 2 * line.m_hlength, 0),
						.m_v = v4(line.m_radius, 0, 0, 0), .m_normal = {},
						.m_nu = impl::Divisions(2.0 * std::sqrt(2.0) * line.m_hlength, spacing),
						.m_nv = std::max(3u, impl::Divisions(std::sqrt(2.0) * math::constants<double>::tau * line.m_radius, spacing)),
						.m_kind = 4, .m_sample_end = 0, .m_taper = line.m_radius,
						.m_measure = static_cast<float>(math::constants<double>::tau * line.m_radius * 2 * line.m_hlength),
						.m_pad0 = 0, .m_pad1 = 0,
					});
				}
				break;
			}
			case EShape::Cylinder:
			{
				auto const& cyl = shape_cast<ShapeCylinder>(shape);
				if (!std::isfinite(cyl.m_hlength) || !std::isfinite(cyl.m_radius) || cyl.m_hlength < 0 || !(cyl.m_radius > 0))
					throw std::runtime_error("Surface cylinder dimensions must be finite, with a positive radius and a non-negative half length");

				// Cover each flat end cap with a polar disc patch. The cap at -Z uses a mirrored tangent so both discs keep a right-handed basis about their outward normal.
				auto const r = cyl.m_radius;
				for (int side = 0; side != 2; ++side)
				{
					auto const sign = side == 0 ? -1.0f : +1.0f;
					impl::Append(plan, SurfacePatch{
						.m_origin = v4(0, 0, sign * cyl.m_hlength, 1), .m_u = v4(r, 0, 0, 0), .m_v = v4(0, sign * r, 0, 0), .m_normal = v4(0, 0, sign, 0),
						.m_nu = impl::Divisions(std::sqrt(2.0) * r, spacing),
						.m_nv = std::max(3u, impl::Divisions(std::sqrt(2.0) * math::constants<double>::tau * r, spacing)),
						.m_kind = 6, .m_sample_end = 0, .m_taper = 0,
						.m_measure = static_cast<float>(0.5 * math::constants<double>::tau * r * r),
						.m_pad0 = 0, .m_pad1 = 0,
					});
				}

				// Fill the side wall like the capsule side. See the 'Line' case for why at least three angular subdivisions are needed.
				if (cyl.m_hlength != 0)
				{
					impl::Append(plan, SurfacePatch{
						.m_origin = v4(0, 0, -cyl.m_hlength, 1), .m_u = v4(0, 0, 2 * cyl.m_hlength, 0),
						.m_v = v4(r, 0, 0, 0), .m_normal = {},
						.m_nu = impl::Divisions(2.0 * std::sqrt(2.0) * cyl.m_hlength, spacing),
						.m_nv = std::max(3u, impl::Divisions(std::sqrt(2.0) * math::constants<double>::tau * r, spacing)),
						.m_kind = 4, .m_sample_end = 0, .m_taper = r,
						.m_measure = static_cast<float>(math::constants<double>::tau * r * 2 * cyl.m_hlength),
						.m_pad0 = 0, .m_pad1 = 0,
					});
				}
				break;
			}
			case EShape::Box:
			case EShape::Sphere:
			{
				auto sphere = false;
				auto radius = v4::Zero();
				switch (shape.m_type)
				{
					case EShape::Box: { radius = shape_cast<ShapeBox>(shape).m_radius.w0(); break; }
					case EShape::Sphere: { sphere = true; radius = v4(shape_cast<ShapeSphere>(shape).m_radius).w0(); break; }
					default: { throw std::runtime_error("Unexpected surface primitive"); }
				}
				if (!IsFinite(radius) || MinElement(radius.xyz) < 0.0f)
					throw std::runtime_error("Surface primitive dimensions must be finite and non-negative");

				if (!sphere && MinElement(radius.xyz) == 0.0f)
					throw std::runtime_error("Surface box dimensions must be positive");

				// A zero-radius sphere retains one position for geometry queries, with no surface-force contribution.
				if (sphere && radius.x == 0)
				{
					impl::Segment(plan, v4::Origin(), v4::Origin(), spacing);
					break;
				}

				// Each face retains its own boundary nodes; normals are never averaged at sharp features.
				for (int axis = 0; axis != 3; ++axis)
				{
					auto const j = (axis + 1) % 3;
					auto const k = (axis + 2) % 3;
					if (radius[j] == 0.0f || radius[k] == 0.0f)
						continue;

					for (int side = 0; side != 2; ++side)
					{
						auto normal = v4::Zero();
						normal[axis] = side == 0 ? 1.0f : -1.0f;
						auto const rj = sphere ? 1.0f : radius[j];
						auto const rk = sphere ? 1.0f : radius[k];
						auto origin = v4::Origin();
						origin[axis] = normal[axis] * (sphere ? 1.0f : radius[axis]);
						origin[j] = -rj;
						origin[k] = -rk;
						auto u = v4::Zero(); u[j] = 2.0f * rj;
						auto v = v4::Zero(); v[k] = 2.0f * rk;
						impl::Append(plan, SurfacePatch{
							.m_origin = origin, .m_u = u, .m_v = v, .m_normal = normal,
							.m_nu = impl::Divisions(2.0 * std::sqrt(2.0) * radius[j], spacing),
							.m_nv = impl::Divisions(2.0 * std::sqrt(2.0) * radius[k], spacing),
							.m_kind = sphere ? 2u : 0u, .m_sample_end = 0,
							.m_taper = sphere ? radius.x : 0.0f,
							.m_measure = sphere ? radius.x * radius.x : 4.0f * rj * rk,
							.m_pad0 = 0, .m_pad1 = 0,
						});
					}
				}
				break;
			}
			case EShape::Triangle:
			{
				auto const& triangle = shape_cast<ShapeTriangle>(shape);
				impl::Triangle(plan, triangle.m_v.x, triangle.m_v.y, triangle.m_v.z, triangle.m_v.w, spacing);
				break;
			}
			case EShape::Polytope:
			{
				auto const& poly = shape_cast<ShapePolytope>(shape);
				for (int i = 0; i != poly.m_face_count; ++i)
				{
					auto const& face = poly.face(i);
					impl::Triangle(plan, poly.vertex(face.m_index[0]), poly.vertex(face.m_index[1]), poly.vertex(face.m_index[2]), Normalise(face.m_plane.direction()), spacing);
				}
				break;
			}
			default:
			{
				throw std::runtime_error("Surface sampling requires a box, sphere, line, cylinder, triangle, or polytope primitive");
			}
		}
		return plan;
	}

	// An upper bound on the area weight of any sample that 'patch' emits, in O(1). The bound is exact for flat, strip and cylinder
	// patches, and tight at the face centre for sphere patches.
	inline float MaxSampleArea(SurfacePatch const& patch)
	{
		switch (patch.m_kind)
		{
			case 0:
			case 4:
			{
				// Interior nodes carry the full grid-cell weight and boundary nodes carry less.
				return patch.m_measure / (static_cast<float>(patch.m_nu) * static_cast<float>(patch.m_nv));
			}
			case 1:
			{
				// The strip weights depend only on the taper (see EmitSurfaceSample), and the collapsed tip of a full taper has its own weight.
				auto const t = patch.m_taper;
				auto weight = patch.m_measure * std::max(std::abs(3.0f - t), std::abs(3.0f - 2.0f * t)) / (6.0f * static_cast<float>(patch.m_nv));
				if (t == 1.0f)
					weight = std::max(weight, patch.m_measure / 6.0f);

				return weight;
			}
			case 2:
			case 3:
			{
				// A node owns a quarter of each of up to four incident cells. A cell of a plane at distance 'h' from the sphere centre
				// subtends a solid angle no greater than its planar area divided by h^2, because every point of the cell is at least 'h' away.
				auto const cross = Cross(patch.m_u.w0(), patch.m_v.w0());
				auto const cross_len = Length(cross);
				auto const h = std::abs(Dot3(patch.m_origin, cross)) / cross_len;
				assert(h > 0.0f && "Sphere patches must lie on a plane that does not pass through the sphere centre");
				auto const cell_area = cross_len / (static_cast<float>(patch.m_nu) * static_cast<float>(patch.m_nv));
				return patch.m_measure * cell_area / (h * h);
			}
			case 5:
			{
				// Segments carry position only.
				return 0.0f;
			}
			case 6:
			{
				// A ring node owns at most '2/nu' of the disc's radius-squared fraction and '1/nv' of the angle.
				return patch.m_measure * 2.0f / (static_cast<float>(patch.m_nu) * static_cast<float>(patch.m_nv));
			}
			default:
			{
				throw std::runtime_error("Unknown surface patch kind");
			}
		}
	}

	// Emit a stable primitive-local identity in O(log patch count), with no per-sample allocation.
	inline SurfaceSample EmitSurfaceSample(Plan const& plan, uint32_t index)
	{
		if (index >= plan.m_count)
			throw std::runtime_error("Surface sample ordinal is outside its plan");

		auto const iter = std::upper_bound(plan.m_patches.begin(), plan.m_patches.end(), index, [](uint32_t value, SurfacePatch const& patch) { return value < patch.m_sample_end; });
		auto const begin = iter == plan.m_patches.begin() ? 0u : (iter - 1)->m_sample_end;
		return EmitSurfaceSample(*iter, index - begin);
	}
}
