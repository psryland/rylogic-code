//************************************************************************
// Cancellation token
//  Copyright (c) Rylogic Ltd 2024
//************************************************************************
#pragma once
#include <cassert>
#include <chrono>
#include <memory>
#include <atomic>
#include <vector>
#include <span>
#include <mutex>
#include <condition_variable>
#include <stdexcept>
#include <algorithm>

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
			// Notes:
			// - Links point downstream only, from a state to the states that should be cancelled with it.
			//   They are weak references, so a linked state is owned only by its own sources and tokens,
			//   and links can't form ownership cycles. Expired links are removed when new links are added.
			// - A state is cancelled at most once. After that, its links are no longer needed and are released.

			std::atomic<bool> m_cancelled;
			std::mutex m_mutex;
			std::condition_variable m_cv_cancelled;
			std::vector<std::weak_ptr<State>> m_downstream; // States to cancel when this state is cancelled

			State() = default;
			State(State&&) = delete;
			State(State const&) = delete;
			State& operator=(State&&) = delete;
			State& operator=(State const&) = delete;

			// True if cancel has been requested on the token
			[[nodiscard]] bool IsCancelRequested() const
			{
				// Lock free, the flag only ever changes from false to true
				return m_cancelled.load(std::memory_order_acquire);
			}

			// Throw an operation cancelled exception if cancel has been requested
			void ThrowIfCancelRequested() const
			{
				// Report cancellation as an exception
				if (IsCancelRequested())
					throw operation_cancelled();
			}

			// Cancel this state and all states linked downstream of it
			void Cancel()
			{
				// Set the flag and take the downstream links. Only the first call does anything.
				std::vector<std::weak_ptr<State>> downstream;
				{
					std::lock_guard lock(m_mutex);
					if (m_cancelled)
						return;

					m_cancelled.store(true, std::memory_order_release);
					downstream.swap(m_downstream);
				}
				m_cv_cancelled.notify_all();

				// Propagate without holding this state's lock, so no lock is held while taking another
				for (auto& link : downstream)
				{
					if (auto state = link.lock())
						state->Cancel();
				}
			}

			// Arrange for 'downstream' to be cancelled when this state is cancelled
			void Link(std::shared_ptr<State> const& downstream)
			{
				// Either register the link before this state is cancelled, or see that it already has been.
				// Checking under the lock means a concurrent 'Cancel' can't miss the new link.
				{
					std::lock_guard lock(m_mutex);
					if (!m_cancelled)
					{
						std::erase_if(m_downstream, [](auto const& link) { return link.expired(); });
						m_downstream.push_back(downstream);
						return;
					}
				}

				// Already cancelled, so cancel 'downstream' now
				downstream->Cancel();
			}

			// Wait for the token to be cancelled
			void Wait()
			{
				// Block until the flag is set
				std::unique_lock lock(m_mutex);
				m_cv_cancelled.wait(lock, [this] { return m_cancelled.load(); });
			}

			// Wait for the token to be cancelled, or for 'wait_time' to pass. Returns true if cancelled.
			bool Wait(std::chrono::milliseconds wait_time)
			{
				// Block until the flag is set or the timeout expires
				std::unique_lock lock(m_mutex);
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
		explicit CancelToken(std::shared_ptr<State> state)
			: m_state(std::move(state))
		{}

	public:

		// A null token, that can't be cancelled
		static CancelToken const& None()
		{
			// A token with no source can never be cancelled
			static CancelToken none(std::make_shared<State>());
			return none;
		}

		// True if cancel has been requested on the token
		[[nodiscard]] bool IsCancelRequested() const
		{
			// Forward to the shared state
			return m_state->IsCancelRequested();
		}

		// Throw an operation cancelled exception if cancel has been requested
		void ThrowIfCancelRequested() const
		{
			// Forward to the shared state
			m_state->ThrowIfCancelRequested();
		}

		// Wait for the token to be cancelled
		void Wait() const
		{
			// Forward to the shared state
			m_state->Wait();
		}

		// Wait for the token to be cancelled, or for 'wait_time' to pass. Returns true if cancelled.
		bool Wait(std::chrono::milliseconds wait_time) const
		{
			// Forward to the shared state
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
			: m_state(std::make_shared<State>())
		{}

		// Create a reference to this token source
		CancelToken Token() const
		{
			// Tokens share the source's state
			return CancelToken(m_state);
		}

		// Create a source that is cancelled when 'linked' is cancelled, or when it is cancelled itself.
		// The new source is cancelled immediately if 'linked' has already been cancelled.
		static CancelTokenSource CreateLinked(CancelToken const& linked)
		{
			// Forward to the multi-token version
			return CreateLinked(std::span<CancelToken const>{ &linked, 1 });
		}

		// Create a source that is cancelled when any of 'linked' is cancelled, or when it is cancelled itself.
		// The new source is cancelled immediately if any of 'linked' has already been cancelled.
		static CancelTokenSource CreateLinked(std::span<CancelToken const> linked)
		{
			// Register the new state downstream of each linked state. Links don't keep the new state alive.
			CancelTokenSource source;
			for (auto const& token : linked)
			{
				assert(token.m_state != nullptr && "Linking to a moved-from token");
				token.m_state->Link(source.m_state);
			}
			return source;
		}

		// True if cancel has been requested on the token
		[[nodiscard]] bool IsCancelRequested() const
		{
			// Forward to the shared state
			return m_state->IsCancelRequested();
		}

		// Throw an operation cancelled exception if cancel has been requested
		void ThrowIfCancelRequested() const
		{
			// Forward to the shared state
			m_state->ThrowIfCancelRequested();
		}

		// Wait for the token to be cancelled
		void Wait() const
		{
			// Forward to the shared state
			m_state->Wait();
		}

		// Wait for the token to be cancelled, or for 'wait_time' to pass. Returns true if cancelled.
		bool Wait(std::chrono::milliseconds wait_time) const
		{
			// Forward to the shared state
			return m_state->Wait(wait_time);
		}

		// Cancel the token, and all sources linked to it
		void Cancel()
		{
			// Forward to the shared state
			m_state->Cancel();
		}
	};
}

#if PR_UNITTESTS
#include <thread>
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTestClass(CancellationTokenTests)
	{
		PRUnitTestMethod(CancelAndWait, Quick)
		{
			using namespace std::chrono_literals;

			// A waiting thread is released by 'Cancel'
			auto cts = CancelTokenSource();
			auto token = cts.Token();
			auto thrd = std::thread([token]
			{
				token.Wait();
				PR_EXPECT(token.IsCancelRequested());
			});

			PR_EXPECT(!token.IsCancelRequested());
			PR_EXPECT(!token.Wait(1ms));
			cts.Cancel();
			thrd.join();

			// Cancelled tokens return immediately and throw on request. Cancelling again does nothing.
			PR_EXPECT(token.IsCancelRequested());
			PR_EXPECT(token.Wait(0ms));
			token.Wait();
			PR_THROWS(token.ThrowIfCancelRequested(), operation_cancelled);
			cts.Cancel();
			PR_EXPECT(cts.IsCancelRequested());

			// The 'None' token is never cancelled
			PR_EXPECT(!CancelToken::None().IsCancelRequested());
			PR_EXPECT(!CancelToken::None().Wait(1ms));
		}
		PRUnitTestMethod(LinkedSources, Quick)
		{
			// Cancellation propagates downstream through chains of links, from any upstream token
			{
				auto cts1 = CancelTokenSource();
				auto cts2 = CancelTokenSource();
				CancelToken upstream[] = { cts1.Token(), cts2.Token() };
				auto linked = CancelTokenSource::CreateLinked(upstream);
				auto chained = CancelTokenSource::CreateLinked(linked.Token());

				// Waiting threads on the end of the chain are released
				auto thrd = std::thread([token = chained.Token()]
				{
					token.Wait();
					PR_EXPECT(token.IsCancelRequested());
				});

				PR_EXPECT(!linked.IsCancelRequested());
				PR_EXPECT(!chained.IsCancelRequested());

				cts2.Cancel();
				thrd.join();

				PR_EXPECT(!cts1.IsCancelRequested());
				PR_EXPECT(linked.IsCancelRequested());
				PR_EXPECT(chained.IsCancelRequested());
			}

			// Cancellation doesn't propagate upstream
			{
				auto cts = CancelTokenSource();
				auto linked = CancelTokenSource::CreateLinked(cts.Token());
				linked.Cancel();
				PR_EXPECT(linked.IsCancelRequested());
				PR_EXPECT(!cts.IsCancelRequested());
			}

			// Linking to a cancelled token gives a cancelled source
			{
				auto cts = CancelTokenSource();
				cts.Cancel();
				auto linked = CancelTokenSource::CreateLinked(cts.Token());
				PR_EXPECT(linked.IsCancelRequested());
			}

			// A linked source can be destroyed before its upstream source is cancelled
			{
				auto cts = CancelTokenSource();
				for (int i = 0; i != 10; ++i)
					auto linked = CancelTokenSource::CreateLinked(cts.Token());

				cts.Cancel();
				PR_EXPECT(cts.IsCancelRequested());
			}

			// A linked source outlives its upstream sources, and remains cancellable itself
			{
				auto linked = CancelTokenSource::CreateLinked(CancelTokenSource().Token());
				PR_EXPECT(!linked.IsCancelRequested());
				linked.Cancel();
				PR_EXPECT(linked.IsCancelRequested());
			}
		}
	};
}
#endif
