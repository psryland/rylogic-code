//************************************************************************
// Cancellation token
// //  Copyright (c) Rylogic Ltd 2024
//************************************************************************
#pragma once
#include <atomic>
#include <memory>
#include <chrono>
#include <vector>
#include <span>
#include <mutex>
#include <condition_variable>
#include <stdexcept>

// Note:
//  For simple use, there is std::stop_source
//  You can't create linked stop sources though, or wait on them.

namespace pr
{
	// Cancellation exception
	struct operation_cancelled : std::runtime_error
	{
		operation_cancelled()
			: std::runtime_error("Operation cancelled")
		{}
		const char* what() const noexcept override
		{
			return "Operation cancelled";
		}
	};

	// Cancellation token state
	namespace cancellation_token
	{
		struct State
		{
			std::atomic<bool> m_cancelled;
			std::mutex m_mutex_cancelled;
			std::condition_variable m_cv_cancelled;
			std::vector<std::weak_ptr<State>> m_notify; // Tokens to notify when cancelled (downstream). Weak, so linked sources don't keep each other alive.

			State() = default;
			State(State&&) = delete;
			State(State const&) = delete;
			State& operator=(State&&) = delete;
			State& operator=(State const&) = delete;

			// True if cancel has been requested on the token
			[[nodiscard]] bool IsCancelRequested() const
			{
				return m_cancelled;
			}

			// Throw an operation cancelled exception if cancel has been requested
			void ThrowIfCancelRequested() const
			{
				if (IsCancelRequested())
					throw operation_cancelled();
			}

			// Cancel the token
			void Cancel()
			{
				// Set the flag and take the downstream list under the lock, so it can't race with 'Link'. Only signal once.
				std::vector<std::weak_ptr<State>> notify;
				{
					std::unique_lock<std::mutex> lock(m_mutex_cancelled);
					if (m_cancelled)
						return;

					m_cancelled = true;
					notify.swap(m_notify);
					m_cv_cancelled.notify_all();
				}

				// Cancel downstream tokens that still exist
				for (auto& downstream : notify)
				{
					if (auto state = downstream.lock())
						state->Cancel();
				}
			}

			// Cancel 'downstream' when this token is cancelled. 'downstream' is cancelled immediately if this token is already cancelled.
			void Link(std::shared_ptr<State> const& downstream)
			{
				// Register 'downstream' under the lock, so a concurrent 'Cancel' either sees it or has already set the flag
				{
					std::unique_lock<std::mutex> lock(m_mutex_cancelled);
					if (!m_cancelled)
					{
						// Drop links to tokens that no longer exist, so long-lived sources don't grow without bound
						std::erase_if(m_notify, [](auto const& w) { return w.expired(); });
						m_notify.push_back(downstream);
						return;
					}
				}
				downstream->Cancel();
			}

			// Wait for the token to be cancelled
			void Wait()
			{
				std::unique_lock<std::mutex> lock(m_mutex_cancelled);
				m_cv_cancelled.wait(lock, [this] { return m_cancelled.load(); });
			}
			bool Wait(std::chrono::milliseconds wait_time)
			{
				std::unique_lock<std::mutex> lock(m_mutex_cancelled);
				return m_cv_cancelled.wait_for(lock, wait_time, [this] { return m_cancelled.load(); });
			}
		};
	}

	// A cancel token is a reference to a token source, and only has read access to the token state.
	struct CancelToken
	{
	private:
		using State = cancellation_token::State;
		friend struct CancelTokenSource;

		std::shared_ptr<State> m_state;
		CancelToken(std::shared_ptr<State> m_state)
			: m_state(m_state)
		{}

	public:

		// A null token, that can't be cancelled
		static CancelToken const& None()
		{
			// Create a token with no source, which can't be cancelled
			static CancelToken none(std::shared_ptr<State>(new State));
			return none;
		}

		// True if cancel has been requested on the token
		[[nodiscard]] bool IsCancelRequested() const
		{
			return m_state->IsCancelRequested();
		}

		// Throw an operation cancelled exception if cancel has been requested
		void ThrowIfCancelRequested() const
		{
			return m_state->ThrowIfCancelRequested();
		}

		// Wait for the token to be cancelled
		void Wait() const
		{
			m_state->Wait();
		}
		bool Wait(std::chrono::milliseconds wait_time) const
		{
			return m_state->Wait(wait_time);
		}
	};

	// A source is used to create references to a common token, and can cancel those tokens
	struct CancelTokenSource
	{
	private:
		using State = cancellation_token::State;
		std::shared_ptr<State> m_state;

	public:

		CancelTokenSource()
			: m_state(new State)
		{}

		// Create a reference to this token source
		CancelToken Token() const
		{
			return CancelToken(m_state);
		}

		// Create a cancellation token source based on existing tokens
		static CancelTokenSource CreateLinked(CancelTokenSource& linked)
		{
			return CreateLinked(std::span<CancelTokenSource>{ &linked, 1 });
		}
		static CancelTokenSource CreateLinked(std::span<CancelTokenSource> linked)
		{
			CancelTokenSource lhs;
			for (auto& rhs : linked)
			{
				// Silently ignore linking to the null token
				if (rhs.m_state == nullptr)
					continue;

				rhs.m_state->Link(lhs.m_state);
			}
			return lhs;
		}

		// True if cancel has been requested on the token
		[[nodiscard]] bool IsCancelRequested() const
		{
			return m_state->IsCancelRequested();
		}

		// Throw an operation cancelled exception if cancel has been requested
		void ThrowIfCancelRequested() const
		{
			return m_state->ThrowIfCancelRequested();;
		}

		// Wait for the token to be cancelled
		void Wait() const
		{
			m_state->Wait();
		}
		bool Wait(std::chrono::milliseconds wait_time) const
		{
			return m_state->Wait(wait_time);
		}

		// Cancel the token
		void Cancel()
		{
			m_state->Cancel();
		}
	};
}

#if PR_UNITTESTS
#include <thread>
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(CancellationTokenTests, Quick)
	{
		auto cts1 = CancelTokenSource();
		auto token1 = cts1.Token();
		{
			auto cts2 = CancelTokenSource::CreateLinked(cts1);
			auto token2 = cts2.Token();

			// Start a thread to wait on the token
			auto thrd = std::thread([token2]
			{
				token2.Wait();
				PR_EXPECT(token2.IsCancelRequested());

			});

			PR_EXPECT(!token1.IsCancelRequested());
			PR_EXPECT(!token2.IsCancelRequested());

			cts1.Cancel();

			PR_EXPECT(token1.IsCancelRequested());
			PR_EXPECT(token2.IsCancelRequested());

			if (thrd.joinable())
				thrd.join();
		}

		PR_EXPECT(token1.IsCancelRequested());

		token1.Wait(); // Should return immediately
		PR_EXPECT(token1.IsCancelRequested());

		// Linking to an already cancelled source gives a cancelled source
		auto cts3 = CancelTokenSource::CreateLinked(cts1);
		PR_EXPECT(cts3.IsCancelRequested());
	}
}
#endif