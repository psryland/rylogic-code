//**************************************************************
// Lambda based RAII object
//  Copyright (C) Rylogic Ltd 2014
//**************************************************************
// Use:
//  auto s = pr::Scope<void>(
//     [&]{ m_flag = true; },
//     [&]{ m_flag = false; });
//
#pragma once
#include <functional>
#include <type_traits>
#include <concepts>

namespace pr
{
	template <typename State>
	struct Scope
	{
		// Notes:
		//  - It is possible to implement this with templates and not std::function but it makes the return
		//    impossible to name.
		using doit_t = std::function<State()>;
		using undo_t = std::function<void(State const&)>;
		using state_t = State;

		state_t m_state;
		undo_t m_undo;
		bool m_dont;

		Scope(state_t const& state, undo_t undo)
			:m_state(state)
			,m_undo(undo)
			,m_dont(false)
		{
		}
		Scope(doit_t doit, undo_t undo)
			:m_state(doit())
			,m_undo(undo)
			,m_dont(false)
		{
		}
		Scope(Scope&& rhs) noexcept
			:m_state(std::move(rhs.m_state))
			, m_undo(std::move(rhs.m_undo))
			, m_dont(rhs.m_dont)
		{
			rhs.m_dont = true;
		}
		Scope(Scope const&) = delete;
		Scope& operator = (Scope&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			std::swap(m_state, rhs.m_state);
			std::swap(m_undo, rhs.m_undo);
			std::swap(m_dont, rhs.m_dont);
			return *this;
		}
		Scope& operator = (Scope const&) = delete;
		~Scope()
		{
			if (m_dont) return;
			try { m_undo(m_state); }
			catch (...) {}
		}
	};

	template <>
	struct Scope<void>
	{
		using doit_t = std::function<void()>;
		using undo_t = std::function<void()>;

		undo_t m_undo;
		bool m_dont;

		explicit Scope(undo_t undo)
			:m_undo(undo)
			,m_dont(false)
		{}
		Scope(doit_t doit, undo_t undo)
			:m_undo(undo)
			,m_dont(false)
		{
			doit();
		}
		Scope(Scope&& rhs) noexcept
			:m_undo(std::move(rhs.m_undo))
			,m_dont(rhs.m_dont)
		{
			rhs.m_dont = true;
		}
		Scope(Scope const&) = delete;
		Scope& operator = (Scope&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			std::swap(m_undo, rhs.m_undo);
			std::swap(m_dont, rhs.m_dont);
			return *this;
		}
		Scope& operator = (Scope const&) = delete;
		~Scope()
		{
			if (m_dont) return;
			try { m_undo(); }
			catch (...) {}
		}
	};

	// Use: pr::Scope<bool&> guard(m_flag);
	template <>
	struct Scope<bool&>
	{
		bool* m_flag;
		Scope(bool& flag)
			: m_flag(&flag)
		{
			if (m_flag)
				*m_flag = true;
		}
		Scope(Scope&& rhs) noexcept
			: m_flag()
		{
			std::swap(m_flag, rhs.m_flag);
		}
		Scope(Scope const&) = delete;
		Scope& operator = (Scope&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			std::swap(m_flag, rhs.m_flag);
			rhs.m_flag = nullptr;
			return *this;
		}
		Scope& operator = (Scope const&) = delete;
		~Scope()
		{
			if (m_flag)
				*m_flag = false;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(ScopeTests, Quick)
	{
		bool flag = false;
		{
			auto s = Scope<void>(
				[&]{ flag = true;  },
				[&]{ flag = false; });

			PR_EXPECT(flag);
		}
		PR_EXPECT(!flag);

		int value = 1;
		{
			auto s = Scope<int>(
				[=]{ return value; },
				[&](int i){ value = i; });
			PR_EXPECT(s.m_state == 1);

			++value;
			PR_EXPECT(value == 2);
		}
		PR_EXPECT(value == 1);
	}
}
#endif