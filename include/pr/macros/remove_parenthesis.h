//*************************************
// Remove parenthesis from macro parameters
// Copyright (c) Rylogic Ltd 2014
//*************************************
#pragma once

// Credit to Steve Robb for this:
#define PR_JOIN_AND_EXPAND3(...) __VA_ARGS__
#define PR_JOIN_AND_EXPAND2(a, ...) PR_JOIN_AND_EXPAND3(a##__VA_ARGS__)
#define PR_JOIN_AND_EXPAND(a, ...) PR_JOIN_AND_EXPAND2(a, __VA_ARGS__)
#define PR_RPIMPLPR_RPIMPL
#define PR_RPIMPL(...) PR_RPIMPL __VA_ARGS__

// Use with 0 or 1 arguments only
#define PR_REMOVE_PARENS(...) PR_JOIN_AND_EXPAND(PR_RPIMPL,PR_RPIMPL __VA_ARGS__)

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::macros
{
	PRUnitTestClass(RemoveParensTest)
	{
		struct Thing1
		{
			int val;
			Thing1() { val = 45; }
		};
		template <typename A, typename B> struct Thing2
		{
			A a;
			B b;
		};

		#define DECLARE_VAR(type, name, init) PR_REMOVE_PARENS(type) name PR_REMOVE_PARENS(init)
		DECLARE_VAR(float, pi, = 3.14f);
		DECLARE_VAR(Thing1, t1, );
		DECLARE_VAR((Thing2<int, float>), t2, (= { 10, 5.99f }));
		#undef DECLARE_VAR

		PRUnitTestMethod(General, Quick)
		{
			PR_EXPECT(pi == 3.14f);
			PR_EXPECT(t1.val == 45);
			PR_EXPECT(t2.a == 10);
			PR_EXPECT(t2.b == 5.99f);
		}
	};
}
#endif
