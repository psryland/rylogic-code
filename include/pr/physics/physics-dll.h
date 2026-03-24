//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// This is the dll API header.
#pragma once

#ifdef PHYSICS_EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

// ** No dependencies except standard includes and Windows headers **
#include <cstdint>
#include <type_traits>
#include <windows.h>

namespace pr::physics
{
	struct RigidBody;

	using DllHandle = unsigned char const*;

	#pragma region Callbacks
	template <typename FuncType>
	struct Callback
	{
		// Usage: { &thing, [](void* ctx, args...){ return type_ptr<Thing>(ctx)->Do(args); } }
		using FuncCB = FuncType;
		using CtxPtr = union { void const* cp; void* p; };

		CtxPtr m_ctx = {};
		FuncCB m_cb = {};

		template <typename... Args>
		auto operator()(Args&&... args) const
		{
			return m_cb(m_ctx.p, std::forward<Args>(args)...);
		}
		explicit operator bool() const
		{
			return m_cb != nullptr;
		}
		friend bool operator == (Callback lhs, Callback rhs)
		{
			return lhs.m_cb == rhs.m_cb && lhs.m_ctx.cp == rhs.m_ctx.cp;
		}
	};
	using ReportErrorCB = Callback<void(__stdcall*)(void* ctx, char const* msg, char const* filepath, int line, int64_t pos)>;
	#pragma endregion
}

// C linkage for API functions
extern "C"
{
	// Dll Context ****************************
	
	// Initialise calls are reference counted and must be matched with Shutdown calls
	// 'initialise_error_cb' is used to report dll initialisation errors only (i.e. it isn't stored)
	// Note: this function is not thread safe, avoid race calls
	PHYSICS_API pr::physics::DllHandle __stdcall Physics_Initialise(pr::physics::ReportErrorCB global_error_cb);
	PHYSICS_API void __stdcall Physics_Shutdown(pr::physics::DllHandle context);
}

namespace pr::physics
{
}