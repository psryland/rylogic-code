//**********************************************************************************
// Pointer to a reference counted object
//  Copyright (c) Rylogic Ltd 2011
//**********************************************************************************
// Use this pointer to point to objects with 'AddRef' and 'Release' functions
// Note: types that inherit pr::RefCount<> can be used
//
// Use the following registry key to prevent the debugger stepping into this type:
//  [HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\VisualStudio\9.0\NativeDE\StepOver]
//  "pr::RefPtr"="pr\\:\\:RefPtr.*\\:\\:.*=NoStepInto"
// Inheritors use the IncRef/DecRef methods, so that stack tracing works
#pragma once
#include <cassert>
#include <type_traits>
#include "pr/common/refcount.h"

#define PR_REFPTR_TRACE 0
#if PR_REFPTR_TRACE == 1
	#include <format>
	#include <string_view>
	#include "pr/common/assert.h"
	#include "pr/win32/stackdump.h"
	//#define PR_REFPTR_TRACE_TYPE IDXGISwapChain
#endif

namespace pr
{
	#if PR_REFPTR_TRACE == 1
	template <RefCountedType T> long PtrRefCount(T*);
	template <RefCountedType T> inline void RefPtrTrace(bool add, T* ptr)
	{
		#if defined(PR_REFPTR_TRACE_TYPE)
		if constexpr (std::is_convertible_v<T, PR_REFPTR_TRACE_TYPE>)
		#endif
		{
			OutputDebugStringA(std::format("[{}] - [{:016x}] - Count = {}\n", (add ? "AddRef" : "Release"), uintptr_t(ptr), PtrRefCount(ptr)).c_str());
			DumpStack([](std::string_view name, std::string_view file, int line) { OutputDebugStringA(std::format("{}({}): {}\n", file, line, name).c_str()); }, 3, 5);
		}
	}
	#endif

	// A ptr wrapper to a reference counting object. 'T' should have methods 'AddRef' and 'Release'
	// Not the same as std::shared_ptr<> because it assumes the pointed-to object has AddRef()/Release() methods
	template <typename T>
	struct RefPtr
	{
		// Notes:
		//  - Can't use a concept for 'T' because that means RefPtr can't be defined using forward declared types. 

		using element_type = T; // Same as std::unique_ptr<>::element_type
		mutable T* m_ptr;

		// Default Construct
		RefPtr()
			:m_ptr(nullptr)
		{}

		// Construct from nullptr
		RefPtr(nullptr_t)
			:m_ptr(nullptr)
		{}

		// Move construct
		RefPtr(RefPtr&& rhs) noexcept
			:m_ptr(rhs.m_ptr)
		{
			rhs.m_ptr = nullptr;
		}

		// Copy construct
		RefPtr(RefPtr const& rhs)
			:m_ptr(rhs.m_ptr)
		{
			if (m_ptr)
				IncRef(m_ptr);
		}

		// Move construct from convertible pointer
		template<class U, class = typename std::enable_if<std::is_convertible<U*,T*>::value>::type>
		RefPtr(RefPtr<U>&& rhs)
			:m_ptr(static_cast<T*>(rhs.m_ptr))
		{
			rhs.m_ptr = nullptr;
		}

		// Copy construct from convertible pointer
		template <typename U, class = typename std::enable_if<std::is_convertible<U*,T*>::value>::type>
		RefPtr(RefPtr<U> const& rhs)
			:m_ptr(static_cast<T*>(rhs.m_ptr))
		{
			if (m_ptr)
				IncRef(m_ptr);
		}

		// Construct from a raw pointer that is convertible to 'T'
		template <typename U, class = typename std::enable_if<std::is_convertible<U*,T*>::value>::type>
		RefPtr(U* t, bool add_ref)
			:m_ptr(t)
		{
			if (!m_ptr)
				return;

			// There are two common cases:
			//  - the object is created with an initial ref count of zero (COM case).
			//  - the object is created with an initial ref count of one (DirectX case)
			if (add_ref)
				IncRef(m_ptr);
		}

		// Destructor
		~RefPtr()
		{
			if (m_ptr)
				DecRef(m_ptr);
		}

		// Assignment to null
		RefPtr& operator = (nullptr_t)
		{
			if (m_ptr) DecRef(m_ptr);
			m_ptr = nullptr;
			return *this;
		}

		// Move assign
		RefPtr& operator = (RefPtr&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			auto ptr = m_ptr;
			m_ptr = rhs.m_ptr;
			rhs.release();
			if (ptr) DecRef(ptr);
			return *this;
		}

		// Copy assign
		RefPtr& operator = (RefPtr const& rhs)
		{
			if (this == &rhs) return *this;
			auto ptr = m_ptr;
			m_ptr = rhs.m_ptr;
			if (m_ptr) IncRef(m_ptr);
			if (ptr) DecRef(ptr);
			return *this;
		}

		// Move assign to convertible pointer type
		template <typename U, class = typename std::enable_if<std::is_convertible<U*,T*>::value>::type>
		RefPtr& operator = (RefPtr<U>&& rhs)
		{
			auto ptr = m_ptr;
			m_ptr = static_cast<T*>(rhs.m_ptr);
			rhs.release();
			if (ptr) DecRef(ptr);
			return *this;
		}

		// Copy assign to convertible pointer type
		template <typename U, class = typename std::enable_if<std::is_convertible<U*,T*>::value>::type>
		RefPtr& operator = (RefPtr<U> const& rhs)
		{
			auto ptr = m_ptr;
			m_ptr = static_cast<T*>(rhs.m_ptr);
			if (m_ptr) IncRef(m_ptr);
			if (ptr) DecRef(ptr);
			return *this;
		}

		// Conversion to bool
		explicit operator bool() const
		{
			return m_ptr != nullptr;
		}

		// Note: there is deliberately no 'operator RefPtr<U>()' conversion operator here.
		// The templated copy/move constructors above already provide the only implicit
		// conversion that is safe: from RefPtr<T> to RefPtr<U> whenever 'T*' converts to
		// 'U*' (i.e. 'U' is a base of 'T', or 'T' itself), since that direction can't slice
		// or misrepresent the pointee's actual type. A member conversion operator enabled
		// for that same direction would be redundant with those constructors and would make
		// the conversion ambiguous, because both become equally-good user-defined conversions
		// for the same source/target pair. Enabling it for the opposite direction instead
		// would let a RefPtr<Base> silently, implicitly "convert" to a RefPtr<Derived> - an
		// unchecked downcast that the compiler can't verify. Use an explicit static_cast or
		// dynamic_cast on the raw pointer if a downcast is genuinely needed.

		// Pointer de-reference
		T* operator -> () const
		{
			return m_ptr;
		}

		// The current reference count
		long RefCount() const
		{
			if (!m_ptr) return 0;
			auto count = m_ptr->AddRef() - 1;
			m_ptr->Release();
			return count;
		}

		// std::unique_ptr<> interface
		// Don't implement 'reset' because it's ambiguous whether to AddRef
		// the provided pointer. Use a RefPtr constructor then assign.
		T const* get() const
		{
			return m_ptr;
		}
		T* get()
		{
			return m_ptr;
		}
		T* release()
		{
			auto ptr = m_ptr;
			m_ptr = nullptr;
			return ptr;
		}

		// Similar to the ComPtr::GetAddressOf() method
		T** address_of()
		{
			return &m_ptr;
		}

	protected:

		long IncRef(T* ptr) const
		{
			#if PR_REFPTR_TRACE == 1
			RefPtrTrace(true, ptr);
			#endif
			return ptr->AddRef();
		}
		void DecRef(T* ptr) const
		{
			#if PR_REFPTR_TRACE == 1
			RefPtrTrace(false, ptr);
			#endif

			// Before releasing the reference, check that there is at least one reference to release.
			// This test will probably crash rather than assert, but hey, good enough.
			// If this fails, check that two or more RefPtrs haven't been created from the same raw pointer.
			// e.g.
			//   ID3DInterface* raw (ref count = 1)
			//   D3DPtr p0(raw) (ref count = 1 still because the following DecRef)
			//   D3DPtr p1(raw) (ref count = 1 still because the following DecRef)
			//   p1->~D3DPtr()  (ref count = 0)
			//   p0->~D3DPtr()  "app.exe has triggered a break point" (i.e. crashed)
			#if PR_REFPTR_TRACE == 1
			assert(pr::PtrRefCount(ptr) > 0 && "Pointer reference count is 0");
			#endif

			// If you get 'use of undefined type' here. You need to include the header for type 'T'.
			// RefPtr can't use forward declared types because it needs to call AddRef()/Release().
			// 'using MyPtr = RefPtr<struct MyType>' is find, but if you use MyPtr in a class it needs to be fully defined.
			ptr->Release();
		}

	public:

		friend bool operator == (RefPtr const& lhs, RefPtr const& rhs) { return lhs.m_ptr == rhs.m_ptr; }
		friend bool operator != (RefPtr const& lhs, RefPtr const& rhs) { return lhs.m_ptr != rhs.m_ptr; }
		friend bool operator <  (RefPtr const& lhs, RefPtr const& rhs) { return lhs.m_ptr < rhs.m_ptr; }
		friend bool operator >  (RefPtr const& lhs, RefPtr const& rhs) { return lhs.m_ptr > rhs.m_ptr; }
		friend bool operator <= (RefPtr const& lhs, RefPtr const& rhs) { return lhs.m_ptr <= rhs.m_ptr; }
		friend bool operator >= (RefPtr const& lhs, RefPtr const& rhs) { return lhs.m_ptr >= rhs.m_ptr; }
		friend bool operator == (RefPtr const& lhs, T const* rhs) { return lhs.m_ptr == rhs; }
		friend bool operator != (RefPtr const& lhs, T const* rhs) { return lhs.m_ptr != rhs; }
		friend bool operator == (T const* lhs, RefPtr const& rhs) { return lhs == rhs.m_ptr; }
		friend bool operator != (T const* lhs, RefPtr const& rhs) { return lhs != rhs.m_ptr; }
		friend bool operator == (RefPtr const& lhs, nullptr_t) { return lhs.m_ptr == nullptr; }
		friend bool operator != (RefPtr const& lhs, nullptr_t) { return lhs.m_ptr != nullptr; }

		friend void swap(RefPtr& left, RefPtr& right) noexcept
		{
			std::swap(left.m_ptr, right.m_ptr);
		}
	};

	// Implementation
	namespace impl
	{
		template <RefCountedType T>
		inline long RefCount(T* ptr, decltype(&T::m_ref_count)*)
		{
			return ptr->m_ref_count;
		}
		template <RefCountedType T>
		inline long RefCount(T* ptr, ...)
		{
			auto count = ptr->AddRef();
			ptr->Release();
			return count - 1;
		}

		// The interface required by RefPtr.
		// Note, the 'T' in RefPtr<T> doesn't actually need to inherit this interface
		// due to template duck-typing, but it can be useful for RefPtr<IRefCounted>
		// pointers to different typed objects.
		struct RefPtrStaticCheckType
		{
			virtual ~RefPtrStaticCheckType() = default;
			virtual long AddRef() const = 0;
			virtual void Release() const = 0;
		};

		// Check RefPtr size
		static_assert(sizeof(RefPtr<RefPtrStaticCheckType>) == sizeof(void*), "Must be the same size as a raw pointer so arrays of RefPtrs can be cast to arrays of raw pointers");
	}

	// Return the current ref count for a ref pointer
	template <RefCountedType T>
	inline long PtrRefCount(T* ptr)
	{
		if (!ptr)
			return 0;

		// A crash here indicates that 'ptr' has already been released.
		// If ptr is a D3DPtr, check that two or more D3DPtrs haven't
		// been created from the same raw pointer. e.g.
		// ID3DInterface* raw (ref count = 1)
		// D3DPtr p0(raw) (ref count = 1 still because the DecRef in the constructor)
		// D3DPtr p1(raw) (ref count = 1 still because the DecRef in the constructor)
		// p1->~D3DPtr()  (ref count = 0)
		// p0->~D3DPtr()  "app.exe has triggered a break point" (i.e. crashed)
		//
		// Watch out for:
		//   D3DPtr<IBlah> p = CorrectlyCreatedBlah();
		//   D3DPtr<IBlahBase> b = p.m_ptr; -- this is wrong, it should be: b = p;
		return impl::RefCount(ptr, nullptr);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTestClass(TestRefPtr)
	{
		struct Thing : RefCount<Thing>
		{
			virtual ~Thing() = default;
			virtual int Kind() const { return 0; }
		};
		struct Derived : Thing
		{
			int Kind() const override { return 1; }
		};
		struct Unrelated : RefCount<Unrelated>
		{
		};

		PRUnitTestMethod(ConversionDirection, Quick)
		{
			// The safe upcast (derived -> base) must be implicit.
			static_assert(std::is_convertible_v<RefPtr<Derived>, RefPtr<Thing>>, "RefPtr<Derived> should implicitly convert to RefPtr<Thing> (its base)");

			// The unsafe downcast (base -> derived) must not be implicit; it can't be checked at compile time.
			static_assert(!std::is_convertible_v<RefPtr<Thing>, RefPtr<Derived>>, "RefPtr<Thing> must not implicitly convert to RefPtr<Derived>");

			// Unrelated types must not be convertible in either direction.
			static_assert(!std::is_convertible_v<RefPtr<Unrelated>, RefPtr<Thing>>, "RefPtr<Unrelated> must not implicitly convert to RefPtr<Thing>");
			static_assert(!std::is_convertible_v<RefPtr<Thing>, RefPtr<Unrelated>>, "RefPtr<Thing> must not implicitly convert to RefPtr<Unrelated>");

			// Exercise the allowed upcast at runtime, not just via the type trait.
			RefPtr<Derived> d(new Derived(), true);
			RefPtr<Thing> b = d;
			PR_EXPECT(b->Kind() == 1);
			PR_EXPECT(b.RefCount() == 2);
		}
	};
}
#endif
