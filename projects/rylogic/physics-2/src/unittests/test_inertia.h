//************************************
// Physics-2 Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics-2/shape/inertia.h"
#include "pr/physics-2/shape/shape_mass.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_box.h"

namespace pr::physics
{
	PRUnitTestClass(InertiaTests)
	{
		PRUnitTestMethod(InertiaConstruction)
		{
			auto mass = 5.0f;
			constexpr auto moment = (1.0f/6.0f) * Sqr(2.0f);

			auto I0 = Inertia{moment, mass};
			PR_EXPECT(FEql(I0, Inertia{I0.To3x3(1), I0.Mass()}));
			PR_EXPECT(FEql(I0, Inertia{I0.To6x6()}));

			auto I1 = Transform(I0, m4x4::Transform(RotationRad<m3x4>(constants<float>::tau_by_4, constants<float>::tau_by_4, 0), v4{1,2,3,0}), ETranslateInertia::AwayFromCoM);
			PR_EXPECT(FEql(I1, Inertia{I1.To3x3(1), I1.Mass()}));
			PR_EXPECT(FEql(I1, Inertia{I1.To6x6()}));

			// Note: about Inertia{3x3, com} vs. Translate,
			//  Inertia{3x3, com} says "'3x3' is the inertia over there at 'com'"
			//  Translate(3x3, ofs) says "'3x3' is the inertia here at the CoM, now measure it over there at 'ofs'"
			auto I2 = Inertia{I1, -v4{3,2,1,0}};
			PR_EXPECT(FEql(I2, Inertia{I2.To6x6()}));
		}
		PRUnitTestMethod(InertiaInvConstruction)
		{
			auto mass = 5.0f;
			constexpr auto moment = (1.0f/6.0f) * Sqr(2.0f);

			auto I0_inv = Invert(Inertia{moment, mass});
			PR_EXPECT(FEql(I0_inv, InertiaInv{I0_inv.To3x3(1), I0_inv.InvMass()}));
			PR_EXPECT(FEql(I0_inv, InertiaInv{I0_inv.To6x6()}));

			auto I1_inv = Transform(I0_inv, m4x4::Transform(RotationRad<m3x4>(constants<float>::tau_by_4, constants<float>::tau_by_4, 0), v4{1,2,3,0}), ETranslateInertia::AwayFromCoM);
			PR_EXPECT(FEql(I1_inv, InertiaInv{I1_inv.To3x3(1), I1_inv.InvMass()}));
			PR_EXPECT(FEql(I1_inv, InertiaInv{I1_inv.To6x6()}));

			auto I2_inv = InertiaInv{I1_inv, -v4{3,2,1,0}};
			PR_EXPECT(FEql(I2_inv, InertiaInv{I2_inv.To6x6()}));
		}
		PRUnitTestMethod(Infinite)
		{
			auto inf_inv = Invert(Inertia::Infinite());
			PR_EXPECT(inf_inv == InertiaInv::Zero());
			auto inf = Invert(inf_inv);
			PR_EXPECT(inf == Inertia::Infinite());
		}
		PRUnitTestMethod(TranslateAndRotate)
		{
			auto mass = 5.0f;
			constexpr auto moment = (1.0f/6.0f) * Sqr(2.0f);
			auto Ic0 = Inertia{moment, mass};
			auto Ic1 = Ic0;

			Ic1 = Translate(Ic1, v4{1,0,0,0}, ETranslateInertia::AwayFromCoM);
			Ic1 = Rotate(Ic1, RotationRad<m3x4>(constants<float>::tau_by_4, 0, 0));
			Ic1 = Rotate(Ic1, RotationRad<m3x4>(0, constants<float>::tau_by_4, 0));
			Ic1 = Translate(Ic1, v4{0,0,1,0}, ETranslateInertia::TowardCoM);

			PR_EXPECT(FEqlRelative(Ic0, Ic1, 0.0001f));
		}
		PRUnitTestMethod(Transform)
		{
			auto mass = 5.0f;
			constexpr auto moment = (1.0f/6.0f) * Sqr(2.0f);
			auto a2b = m4x4::Transform(RotationRad<m3x4>(constants<float>::tau_by_4, constants<float>::tau_by_4, 0), v4{0,0,1,1});
			auto Ic0 = Inertia{moment, mass};
			auto Ic1 = Translate(Rotate(Ic0, a2b.rot), a2b.pos, ETranslateInertia::AwayFromCoM);
			auto Ic2 = Transform(Ic0, a2b, ETranslateInertia::AwayFromCoM);
			PR_EXPECT(FEql(Ic1, Ic2));
		}
		PRUnitTestMethod(TranslateInverse)
		{
			auto mass = 5.0f;
			auto a2b = m4x4::Transform(RotationRad<m3x4>(constants<float>::tau_by_4, constants<float>::tau_by_4, 0), v4{0,0,1,1});
			auto Ic0 = Rotate(Inertia::Box(v4{1,2,3,0}, mass, v4{1,1,1,0}), a2b.rot);
			auto Ic0_inv = Invert(Ic0);

			// Translate by invert-translate-invert
			auto Ic1 = Invert(Ic0_inv);
			auto Io1 = Translate(Ic1, +a2b.pos, ETranslateInertia::AwayFromCoM);
			auto Io1_inv = Invert(Io1);

			auto Io2 = Invert(Io1_inv);
			auto Ic2 = Translate(Io2, -a2b.pos, ETranslateInertia::TowardCoM);
			auto Ic2_inv = Invert(Ic2);

			// Directly translate the inverse inertia
			auto IC0_inv = Ic0_inv;
			auto IO1_inv = Translate(IC0_inv, +a2b.pos, ETranslateInertia::AwayFromCoM);
			auto IC2_inv = Translate(IO1_inv, -a2b.pos, ETranslateInertia::TowardCoM);

			PR_EXPECT(FEql(Ic0_inv, IC0_inv));
			PR_EXPECT(FEql(Io1_inv, IO1_inv));
			PR_EXPECT(FEql(Ic2_inv, IC2_inv));
		}
		PRUnitTestMethod(SixBySixVsThreeByThreeNoOffset)
		{
			auto mass = 5.0f;
			auto avel = v4{0, 0, 1, 0}; //v4(-1,-2,-3,0);
			auto lvel = v4{0, 1, 0, 0}; //v4(+1,+2,+3,0);
			auto vel = v8motion{avel, lvel};

			// Inertia of a sphere with radius 1, positioned at (0,0,0), measured at (0,0,0) (2/5 m r²)
			auto Ic = (2.0f/5.0f) * m3x4::Identity();

			// Traditional momentum calculation
			auto amom = mass * Ic * avel; // I.w
			auto lmom = mass * lvel;      // M.v

			// Spatial inertia for the same sphere, expressed at (0,0,0)
			auto sIc = Inertia(Ic, mass);
			auto mom = sIc * vel;
			PR_EXPECT(FEql(mom.ang, amom));
			PR_EXPECT(FEql(mom.lin, lmom));

			// Full spatial matrix multiply
			auto sIc2 = sIc.To6x6();
			auto mom2 = sIc2 * vel;
			PR_EXPECT(FEql(mom, mom2));
		}
		PRUnitTestMethod(SixBySixWithOffset)
		{
			auto mass = 5.0f;
			auto avel = v4{0, 0, 1, 0};
			auto lvel = v4{0, 1, 0, 0};
			auto vel = v8motion{avel, lvel};

			// Inertia of a sphere with radius 0.5, positioned at (0,0,0), measured at (0,0,0) (2/5 m r²)
			auto Ic = Inertia::Sphere(0.5f, 1);

			// Calculate the momentum at 'r'
			auto r = v4{1,0,0,0};
			auto amom = mass * (Ic.To3x3() * avel - Cross(r, lvel));
			auto lmom = mass * lvel;

			// Inertia of a sphere with radius 0.5, positioned at (0,0,0) and measured at (0,0,0) expressed at 'r'
			auto vel_r = Shift(vel, r);
			auto sI_r = Inertia(Ic.To3x3(1), mass, -r);
			auto mom = sI_r * vel_r;
			PR_EXPECT(FEql(mom.ang, amom));
			PR_EXPECT(FEql(mom.lin, lmom));

			// Expressed at another point
			r = v4{2,0,0,0};
			amom = mass * (Ic.To3x3() * avel - Cross(r, lvel));
			lmom = mass * lvel;

			// Inertia of a sphere with radius 0.5, positioned at (0,0,0) and measured at (0,0,0) expressed at 'r'
			vel_r = Shift(vel, r);
			sI_r = Inertia(Ic.To3x3(1), mass, -r);
			mom = sI_r * vel_r;
			PR_EXPECT(FEql(mom.ang, amom));
			PR_EXPECT(FEql(mom.lin, lmom));

			// Expressed at another point
			r = v4{1,2,3,0};
			amom = mass * (Ic.To3x3() * avel - Cross(r, lvel));
			lmom = mass * lvel;

			// Inertia of a sphere with radius 0.5, positioned at (0,0,0) and measured at (0,0,0) expressed at 'r'
			vel_r = Shift(vel, r);
			sI_r = Inertia(Ic.To3x3(1), mass, -r);
			mom = sI_r * vel_r;
			PR_EXPECT(FEql(mom.ang, amom));
			PR_EXPECT(FEql(mom.lin, lmom));
		}
		PRUnitTestMethod(AdditionSubtractionInertia)
		{
			auto mass = 5.0f;
			auto sph0 = Inertia::Sphere(0.5f, mass);
			auto sph1 = Inertia::Sphere(0.5f, mass);

			// Simple addition/subtraction of inertia in CoM frame
			auto sph2 = Inertia::Sphere(0.5f, 2*mass);
			auto SPH2 = Join(sph0, sph1);
			auto SPH3 = Split(sph2, sph1);
			PR_EXPECT(FEql(sph2, SPH2));
			PR_EXPECT(FEql(sph0, SPH3));

			// Addition/Subtraction of translated inertias
			auto sph4 = Translate(sph0, v4{-1,0,0,0}, ETranslateInertia::AwayFromCoM);
			auto sph5 = Translate(sph1, v4{+1,0,0,0}, ETranslateInertia::AwayFromCoM);
			auto sph6 = Inertia{v4{0.1f,1.1f,1.1f,0}, v4{}, 2*mass};
			auto SPH6 = Join(sph4, sph5);
			auto SPH7 = Split(sph6, sph4);
			PR_EXPECT(FEql(sph6, SPH6));
			PR_EXPECT(FEql(sph5, SPH7));

			// Addition/Subtraction of inertias with offsets
			auto sph8 = Inertia{sph0, v4{1,2,3,0}};
			auto sph9 = Inertia{sph1, v4{1,2,3,0}};
			auto sph10 = Inertia{sph2, v4{1,2,3,0}};
			auto SPH10 = Join(sph8, sph9);
			auto SPH11 = Split(sph10, sph9);
			PR_EXPECT(FEql(sph10, SPH10));
			PR_EXPECT(FEql(sph8, SPH11));
		}
		PRUnitTestMethod(AdditionSubtractionInverseInertia)
		{
			auto mass = 5.0f;
			auto sph0 = Inertia::Sphere(0.5f, mass);
			auto sph1 = Inertia::Sphere(0.5f, mass);
			auto sph2 = Inertia::Sphere(0.5f, 2*mass);

			// Simple addition/subtraction of inertia in CoM frame
			auto SPH2 = Join(Invert(sph0), Invert(sph1));
			auto SPH3 = Split(Invert(sph2), Invert(sph1));
			PR_EXPECT(FEql(Invert(sph2), SPH2));
			PR_EXPECT(FEql(Invert(sph0), SPH3));

			// Addition/Subtraction of translated inertias
			auto sph4 = Translate(sph0, v4{-1,0,0,0}, ETranslateInertia::AwayFromCoM);
			auto sph5 = Translate(sph1, v4{+1,0,0,0}, ETranslateInertia::AwayFromCoM);
			auto sph6 = Inertia{v4{0.1f,1.1f,1.1f,0}, v4{}, 2*mass};
			auto SPH6 = Join(Invert(sph4), Invert(sph5));
			auto SPH7 = Split(Invert(sph6), Invert(sph4));
			PR_EXPECT(FEql(Invert(sph6), SPH6));
			PR_EXPECT(FEql(Invert(sph5), SPH7));

			// Addition/Subtraction of inertias with offsets
			auto sph8 = Inertia{sph0, v4{1,2,3,0}};
			auto sph9 = Inertia{sph1, v4{1,2,3,0}};
			auto sph10 = Inertia{sph2, v4{1,2,3,0}};
			auto SPH10 = Join(Invert(sph8), Invert(sph9));
			auto SPH11 = Split(Invert(sph10), Invert(sph9));
			PR_EXPECT(FEql(Invert(sph10), SPH10));
			PR_EXPECT(FEql(Invert(sph8), SPH11));
		}
		PRUnitTestMethod(InvertingSixBySixInertia)
		{
			auto mass = 5.0f;
			auto Ic = Inertia::Sphere(0.5f, 1);
			auto r = v4{1,2,3,0};

			auto a = Inertia{Ic.Ic3x3(1), mass, -r};
			auto b = Invert(a);
			auto c = Invert(b);
			auto a6x6 = a.To6x6();
			auto b6x6 = b.To6x6();
			auto c6x6 = c.To6x6();

			auto A = a.To6x6();
			auto B = Invert(A);
			auto C = Invert(B);

			PR_EXPECT(FEqlRelative(a6x6, A, 0.001f));
			PR_EXPECT(FEqlRelative(b6x6, B, 0.001f));
			PR_EXPECT(FEqlRelative(c6x6, C, 0.001f));
		}
		PRUnitTestMethod(AccelerationFromForce)
		{
			auto mass = 5.0f;
			auto F = 2.0f;
			auto L = 1.0f;
			auto I = (1.0f/12.0f) * mass * Sqr(L); // I = 1/12 m L²

			// Create a vertical rod inertia
			auto Ic = Inertia::Box(v4{0.0001f, 0.5f*L, 0.0001f,0}, mass);
			auto Ic_inv = Invert(Ic);

			// Apply a force at the CoM
			auto f0 = v8force{0, 0, 0, F, 0, 0};
			auto a0 = Ic_inv * f0;
			PR_EXPECT(FEql(a0, v8motion{0, 0, 0, F/mass, 0, 0}));

			// Apply a force at the top
			// a = F/m, A = F.d/I
			auto r = v4{0, 0.5f*L, 0, 0};
			auto f1 = Shift(f0, -r);
			auto a1 = Ic_inv * f1;
			PR_EXPECT(FEql(a1, v8motion{0, 0, -F*r.y/I, F/mass, 0, 0}));

			// Apply a force at an arbitrary point
			r = v4{3,2,0,0};
			auto f2 = Shift(f0, -r);
			auto a2 = Ic_inv * f2;
			auto a = (1.0f/mass)*f0.lin;
			auto A = Ic_inv.To3x3() * Cross(r, f0.lin);
			PR_EXPECT(FEql(a2, v8motion{A, a}));
		}
		PRUnitTestMethod(KineticEnergy)
		{
			auto mass = 5.0f;

			// Sphere travelling at 'vel'
			auto avel = v4{0, 0, 1, 0}; //v4(-1,-2,-3,0);
			auto lvel = v4{0, 1, 0, 0}; //v4(+1,+2,+3,0);
			auto vel = v8motion{avel, lvel};

			// Inertia of a sphere with radius 1, positioned at (0,0,0), measured at (0,0,0) (2/5 m r²)
			auto Ic = (2.0f/5.0f) * m3x4::Identity();

			// Calculate kinetic energy
			auto ke_lin = 0.5f * mass * Dot(lvel,lvel);
			auto ke_ang = 0.5f * mass * Dot(avel, Ic * avel);
			auto ke = ke_lin + ke_ang;

			auto sIc = Inertia(Ic, mass);
			auto mom = sIc * vel;
			auto KE = 0.5f * Dot(vel, mom); // 0.5 v.I.v

			PR_EXPECT(FEql(KE, ke));
		}
		PRUnitTestMethod(PolytopeVsBoxInertia_UnitCube)
		{
			// Validate that a cube-shaped polytope has the same inertia as the analytic ShapeBox formula.
			// This catches errors in the polytope face integration (wrong divisors, sign errors, etc.)
			using namespace pr::collision;

			// Unit cube: half-extents = 1 in each axis
			auto s = 1.0f;
			v4 pts[] = {
				v4{-s,-s,-s,1}, v4{+s,-s,-s,1}, v4{+s,+s,-s,1}, v4{-s,+s,-s,1},
				v4{-s,-s,+s,1}, v4{+s,-s,+s,1}, v4{+s,+s,+s,1}, v4{-s,+s,+s,1},
			};
			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			// ShapeBox stores half-extents in m_radius, takes full dimensions
			ShapeBox box(v4{2 * s, 2 * s, 2 * s, 0});

			auto density = 1.0f;
			auto mp_poly = CalcMassProperties(poly, density);
			auto mp_box = CalcMassProperties(box, density);

			// Volume should match: (2s)^3 = 8
			PR_EXPECT(FEqlRelative(mp_poly.m_mass, mp_box.m_mass, 0.001f));

			// CoM should be at origin for a symmetric cube
			PR_EXPECT(FEql(mp_poly.m_centre_of_mass, v4{}));

			// Unit inertia diagonals should match: (1/3)(s² + s²) = 2s²/3
			auto& Ip = mp_poly.m_os_unit_inertia;
			auto& Ib = mp_box.m_os_unit_inertia;
			PR_EXPECT(FEqlRelative(Ip.x.x, Ib.x.x, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.y.y, Ib.y.y, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.z.z, Ib.z.z, 0.001f));

			// Off-diagonals should be zero for a symmetric cube at origin
			PR_EXPECT(FEql(Ip.x.y, 0.0f));
			PR_EXPECT(FEql(Ip.x.z, 0.0f));
			PR_EXPECT(FEql(Ip.y.z, 0.0f));
		}
		PRUnitTestMethod(PolytopeVsBoxInertia_AsymmetricBox)
		{
			// Validate polytope inertia for an asymmetric box (different dimensions per axis)
			using namespace pr::collision;

			// Asymmetric box: half-extents 0.5, 1.0, 1.5
			auto hx = 0.5f, hy = 1.0f, hz = 1.5f;
			v4 pts[] = {
				v4{-hx,-hy,-hz,1}, v4{+hx,-hy,-hz,1}, v4{+hx,+hy,-hz,1}, v4{-hx,+hy,-hz,1},
				v4{-hx,-hy,+hz,1}, v4{+hx,-hy,+hz,1}, v4{+hx,+hy,+hz,1}, v4{-hx,+hy,+hz,1},
			};
			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();
			ShapeBox box(v4{2 * hx, 2 * hy, 2 * hz, 0});

			auto density = 2.5f;
			auto mp_poly = CalcMassProperties(poly, density);
			auto mp_box = CalcMassProperties(box, density);

			// Volume = 1×2×3 = 6, mass = 6 × 2.5 = 15
			PR_EXPECT(FEqlRelative(mp_poly.m_mass, mp_box.m_mass, 0.001f));
			PR_EXPECT(FEql(mp_poly.m_centre_of_mass, v4{}));

			// Each diagonal should match the box formula: (1/3)(hy² + hz²), etc.
			auto& Ip = mp_poly.m_os_unit_inertia;
			auto& Ib = mp_box.m_os_unit_inertia;
			PR_EXPECT(FEqlRelative(Ip.x.x, Ib.x.x, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.y.y, Ib.y.y, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.z.z, Ib.z.z, 0.001f));
			PR_EXPECT(FEql(Ip.x.y, 0.0f));
			PR_EXPECT(FEql(Ip.x.z, 0.0f));
			PR_EXPECT(FEql(Ip.y.z, 0.0f));
		}
		PRUnitTestMethod(PolytopeVsBoxInertia_OffsetCube)
		{
			// Validate polytope inertia for an offset cube (CoM not at origin).
			// The unit inertia should be measured at the model origin, not the centroid.
			using namespace pr::collision;

			auto s = 1.0f;
			auto ox = 2.0f, oy = 3.0f, oz = 4.0f;
			v4 pts[] = {
				v4{ox-s,oy-s,oz-s,1}, v4{ox+s,oy-s,oz-s,1}, v4{ox+s,oy+s,oz-s,1}, v4{ox-s,oy+s,oz-s,1},
				v4{ox-s,oy-s,oz+s,1}, v4{ox+s,oy-s,oz+s,1}, v4{ox+s,oy+s,oz+s,1}, v4{ox-s,oy+s,oz+s,1},
			};
			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			auto density = 1.0f;
			auto mp_poly = CalcMassProperties(poly, density);

			// CoM should be at (ox, oy, oz)
			PR_EXPECT(FEqlRelative(mp_poly.m_centre_of_mass.x, ox, 0.001f));
			PR_EXPECT(FEqlRelative(mp_poly.m_centre_of_mass.y, oy, 0.001f));
			PR_EXPECT(FEqlRelative(mp_poly.m_centre_of_mass.z, oz, 0.001f));

			// The unit inertia is measured at the model ORIGIN (parallel axis theorem).
			// Io = Ic + (d·d)I - d⊗d  where Ic is the centroidal inertia, d is the offset
			auto Ic_diag = (1.0f / 3.0f) * (s * s + s * s); // centroidal unit cube
			auto Io_xx = Ic_diag + (oy * oy + oz * oz);
			auto Io_yy = Ic_diag + (ox * ox + oz * oz);
			auto Io_zz = Ic_diag + (ox * ox + oy * oy);
			auto Io_xy = -(ox * oy);
			auto Io_xz = -(ox * oz);
			auto Io_yz = -(oy * oz);

			auto& Ip = mp_poly.m_os_unit_inertia;
			PR_EXPECT(FEqlRelative(Ip.x.x, Io_xx, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.y.y, Io_yy, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.z.z, Io_zz, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.x.y, Io_xy, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.x.z, Io_xz, 0.001f));
			PR_EXPECT(FEqlRelative(Ip.y.z, Io_yz, 0.001f));
		}
		PRUnitTestMethod(PolytopeInertia_RegularTetrahedron)
		{
			// Validate that a regular tetrahedron polytope has the correct inertia.
			// Analytic formula: for a regular tetrahedron with edge length a,
			// I_diag = (1/20) * a² (per unit mass, about CoM)
			using namespace pr::collision;

			// Regular tetrahedron with edge length a = 2
			auto a = 2.0f;
			auto h = a * Sqrt(2.0f / 3.0f); // height from base to apex
			auto r = a / Sqrt(3.0f);         // circumradius of base triangle
			v4 pts[] = {
				v4{ r,          0,       -h / 4.0f, 1}, // base vertex 0
				v4{-r / 2.0f,  a / 2.0f, -h / 4.0f, 1}, // base vertex 1
				v4{-r / 2.0f, -a / 2.0f, -h / 4.0f, 1}, // base vertex 2
				v4{ 0,          0,        3.0f * h / 4.0f, 1}, // apex
			};
			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			auto density = 1.0f;
			auto mp = CalcMassProperties(poly, density);

			// Volume of regular tetrahedron: a³/(6√2)
			auto expected_volume = (a * a * a) / (6.0f * Sqrt(2.0f));
			PR_EXPECT(FEqlRelative(mp.m_mass / density, expected_volume, 0.01f));

			// CoM should be at the centroid: (0, 0, 0) by construction
			// (we placed the base at -h/4 and apex at +3h/4 to centre vertically)
			PR_EXPECT(FEqlRelative(mp.m_centre_of_mass.x, 0.0f, 0.01f));
			PR_EXPECT(FEqlRelative(mp.m_centre_of_mass.y, 0.0f, 0.01f));
			PR_EXPECT(FEqlRelative(mp.m_centre_of_mass.z, 0.0f, 0.01f));

			// For a regular tetrahedron centred at origin, the centroidal unit inertia
			// is isotropic: I_xx = I_yy = I_zz = (1/20) * a² (per unit mass)
			auto expected_I = (1.0f / 20.0f) * a * a;
			auto& I = mp.m_os_unit_inertia;

			// Since CoM ≈ origin, the origin-measured inertia ≈ centroidal inertia
			PR_EXPECT(FEqlRelative(I.x.x, expected_I, 0.02f));
			PR_EXPECT(FEqlRelative(I.y.y, expected_I, 0.02f));
			PR_EXPECT(FEqlRelative(I.z.z, expected_I, 0.02f));
		}
		PRUnitTestMethod(LineInertia)
		{
			auto mass = 3.0f;
			auto half_length = 2.0f;

			// Line along Z-axis: Ixx = Iyy = (1/3)*h², Izz = 0, products = 0
			auto I = Inertia::Line(half_length, mass);
			auto expected = (1.0f / 3.0f) * Sqr(half_length);
			PR_EXPECT(FEql(I.m_diagonal.x, expected));
			PR_EXPECT(FEql(I.m_diagonal.y, expected));
			PR_EXPECT(FEql(I.m_diagonal.z, 0.0f));
			PR_EXPECT(FEql(I.m_products, v4{}));
			PR_EXPECT(FEql(I.Mass(), mass));
			PR_EXPECT(FEql(I.CoM(), v4{}));
		}
		PRUnitTestMethod(LineInertia_MatchesDegenerateBox)
		{
			// A line along Z should match a box with zero X/Y half-extents
			auto mass = 5.0f;
			auto h = 1.5f;
			auto line = Inertia::Line(h, mass);
			auto box = Inertia::Box(v4{0, 0, h, 0}, mass);
			PR_EXPECT(FEql(line, box));
		}
		PRUnitTestMethod(LineInertia_WithOffset)
		{
			auto mass = 4.0f;
			auto h = 1.0f;
			auto offset = v4{1, 2, 3, 0};
			auto I = Inertia::Line(h, mass, offset);

			// Build the same thing manually: create at origin, then translate
			auto I_manual = Inertia::Line(h, mass);
			I_manual = Translate(I_manual, offset, ETranslateInertia::AwayFromCoM);
			PR_EXPECT(FEql(I, I_manual));
		}
		PRUnitTestMethod(LineInertia_Roundtrip6x6)
		{
			auto mass = 2.0f;
			auto h = 3.0f;
			auto offset = v4{1, 0, 0, 0};
			auto I = Inertia::Line(h, mass, offset);

			// Roundtrip through 6x6 construction
			PR_EXPECT(FEql(I, Inertia{I.To6x6()}));
		}
		PRUnitTestMethod(TriangleInertia_RightTriangleXY)
		{
			// Right triangle in XY plane: (0,0,0), (3,0,0), (0,4,0)
			auto mass = 6.0f;
			auto a = v4{0, 0, 0, 1};
			auto b = v4{3, 0, 0, 1};
			auto c = v4{0, 4, 0, 1};
			auto I = Inertia::Triangle(a, b, c, mass);

			// CoM = (1, 4/3, 0)
			PR_EXPECT(FEqlRelative(I.CoM().x, 1.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.CoM().y, 4.0f / 3.0f, 0.0001f));
			PR_EXPECT(FEql(I.CoM().z, 0.0f));
			PR_EXPECT(FEql(I.Mass(), mass));

			// Unit inertia about CoM (derived from barycentric integrals)
			// Ixx = 8/9, Iyy = 1/2, Izz = 25/18
			PR_EXPECT(FEqlRelative(I.m_diagonal.x, 8.0f / 9.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_diagonal.y, 1.0f / 2.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_diagonal.z, 25.0f / 18.0f, 0.0001f));

			// Products: Ixy = 1/3, Ixz = Iyz = 0
			PR_EXPECT(FEqlRelative(I.m_products.x, 1.0f / 3.0f, 0.0001f));
			PR_EXPECT(FEql(I.m_products.y, 0.0f));
			PR_EXPECT(FEql(I.m_products.z, 0.0f));
		}
		PRUnitTestMethod(TriangleInertia_PerpendicularAxisTheorem)
		{
			// For any planar lamina in the XY plane: Izz = Ixx + Iyy
			auto a = v4{1, 0, 0, 1};
			auto b = v4{-2, 3, 0, 1};
			auto c = v4{0.5f, -1, 0, 1};
			auto I = Inertia::Triangle(a, b, c, 1.0f);
			PR_EXPECT(FEqlRelative(I.m_diagonal.z, I.m_diagonal.x + I.m_diagonal.y, 0.0001f));
		}
		PRUnitTestMethod(TriangleInertia_EquilateralSymmetry)
		{
			// Equilateral triangle centred at origin in XY plane.
			// Vertices on the unit circle: (0,1,0), (-√3/2,-1/2,0), (√3/2,-1/2,0)
			// Should have Ixx = Iyy and all products = 0 (3-fold symmetry)
			auto s3_2 = Sqrt(3.0f) / 2.0f;
			auto a = v4{0, 1, 0, 1};
			auto b = v4{-s3_2, -0.5f, 0, 1};
			auto c = v4{s3_2, -0.5f, 0, 1};
			auto I = Inertia::Triangle(a, b, c, 1.0f);

			// CoM at origin
			PR_EXPECT(FEqlRelative(I.CoM().x, 0.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.CoM().y, 0.0f, 0.0001f));

			// Ixx = Iyy = 1/8 (circumradius = 1, side = √3)
			PR_EXPECT(FEqlRelative(I.m_diagonal.x, 1.0f / 8.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_diagonal.y, 1.0f / 8.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_diagonal.z, 1.0f / 4.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_products.x, 0.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_products.y, 0.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_products.z, 0.0f, 0.0001f));
		}
		PRUnitTestMethod(TriangleInertia_3DIsotropic)
		{
			// Triangle with vertices (1,0,0), (0,1,0), (0,0,1) — symmetric under
			// cyclic permutation of axes. Ixx = Iyy = Izz = 1/9, Ixy = Ixz = Iyz = 1/36.
			auto a = v4{1, 0, 0, 1};
			auto b = v4{0, 1, 0, 1};
			auto c = v4{0, 0, 1, 1};
			auto I = Inertia::Triangle(a, b, c, 1.0f);

			// CoM = (1/3, 1/3, 1/3)
			PR_EXPECT(FEqlRelative(I.CoM().x, 1.0f / 3.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.CoM().y, 1.0f / 3.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.CoM().z, 1.0f / 3.0f, 0.0001f));

			// All diagonals equal, all products equal (cyclic symmetry)
			PR_EXPECT(FEqlRelative(I.m_diagonal.x, 1.0f / 9.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_diagonal.y, 1.0f / 9.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_diagonal.z, 1.0f / 9.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_products.x, 1.0f / 36.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_products.y, 1.0f / 36.0f, 0.0001f));
			PR_EXPECT(FEqlRelative(I.m_products.z, 1.0f / 36.0f, 0.0001f));
		}
		PRUnitTestMethod(TriangleInertia_Roundtrip6x6)
		{
			// Roundtrip through 6x6 construction
			auto a = v4{1, 0, 0, 1};
			auto b = v4{0, 1, 0, 1};
			auto c = v4{0, 0, 1, 1};
			auto I = Inertia::Triangle(a, b, c, 5.0f);
			PR_EXPECT(FEql(I, Inertia{I.To6x6()}));
		}
	};
}
#endif
