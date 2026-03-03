//************************************
// Physics-2 Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics-2/shape/inertia.h"

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

			auto I1 = Transform(I0, m4x4::Transform(float(maths::tau_by_4), float(maths::tau_by_4), 0, v4{1,2,3,0}), ETranslateInertia::AwayFromCoM);
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

			auto I1_inv = Transform(I0_inv, m4x4::Transform(float(maths::tau_by_4), float(maths::tau_by_4), 0, v4{1,2,3,0}), ETranslateInertia::AwayFromCoM);
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
			Ic1 = Rotate(Ic1, m3x4::Rotation(float(maths::tau_by_4), 0, 0));
			Ic1 = Rotate(Ic1, m3x4::Rotation(0, float(maths::tau_by_4), 0));
			Ic1 = Translate(Ic1, v4{0,0,1,0}, ETranslateInertia::TowardCoM);

			PR_EXPECT(FEqlRelative(Ic0, Ic1, 0.0001f));
		}

		PRUnitTestMethod(Transform)
		{
			auto mass = 5.0f;
			constexpr auto moment = (1.0f/6.0f) * Sqr(2.0f);
			auto a2b = m4x4::Transform(float(maths::tau_by_4), float(maths::tau_by_4), 0, v4{0,0,1,1});
			auto Ic0 = Inertia{moment, mass};
			auto Ic1 = Translate(Rotate(Ic0, a2b.rot), a2b.pos, ETranslateInertia::AwayFromCoM);
			auto Ic2 = Transform(Ic0, a2b, ETranslateInertia::AwayFromCoM);
			PR_EXPECT(FEql(Ic1, Ic2));
		}

		PRUnitTestMethod(TranslateInverse)
		{
			auto mass = 5.0f;
			auto a2b = m4x4::Transform(float(maths::tau_by_4), float(maths::tau_by_4), 0, v4{0,0,1,1});
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
			auto Ic = (2.0f/5.0f) * m3x4Identity;

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
			auto Ic = (2.0f/5.0f) * m3x4Identity;

			// Calculate kinetic energy
			auto ke_lin = 0.5f * mass * Dot(lvel,lvel);
			auto ke_ang = 0.5f * mass * Dot(avel, Ic * avel);
			auto ke = ke_lin + ke_ang;

			auto sIc = Inertia(Ic, mass);
			auto mom = sIc * vel;
			auto KE = 0.5f * Dot(vel, mom); // 0.5 v.I.v

			PR_EXPECT(FEql(KE, ke));
		}
	};
}
#endif
