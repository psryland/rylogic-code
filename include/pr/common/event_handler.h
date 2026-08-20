//******************************************
// Multicast Event
//  Copyright (c) Oct 2011 Rylogic Ltd
//******************************************
#pragma once

#include <list>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <type_traits>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace pr
{
	// Notes:
	//  - EventHandler is for notification where the context pointer is the thing doing the notifying.
	//  - MultiCast is for callbacks where the context pointer is subscriber provided.
	//  - EventHandler requires all handlers to have a signature: Handler(Thing& sender, EventArgs const& args);
	//  - Unlike C# however, EventArgs can be anything and doesn't have to be const.

	// Map place-holders into pr::
	inline constexpr auto _1 = std::placeholders::_1;
	inline constexpr auto _2 = std::placeholders::_2;
	inline constexpr auto _3 = std::placeholders::_3;
	inline constexpr auto _4 = std::placeholders::_4;
	inline constexpr auto _5 = std::placeholders::_5;
	inline constexpr auto _6 = std::placeholders::_6;
	inline constexpr auto _7 = std::placeholders::_7;
	inline constexpr auto _8 = std::placeholders::_8;
	inline constexpr auto _9 = std::placeholders::_9;

	// Implementation detail for EventHandler
	namespace multicast
	{
		using Id = unsigned long long;
		struct Sub;

		// Non-template interface for all event handlers and multicasts
		struct IMultiCast
		{
			virtual ~IMultiCast() = default;
			virtual void unsubscribe(Sub& sub) = 0;
		};

		// A reference to an event handler subscription. Used for unsubscribing.
		struct Sub
		{
			IMultiCast* m_mc;
			Id m_id;

			Sub()
				:m_mc()
				,m_id()
			{}
			Sub(IMultiCast* mc, Id id)
				:m_mc(mc)
				,m_id(id)
			{}
			static Sub Make(IMultiCast* evt)
			{
				static std::atomic_uint64_t s_id = {};
				auto id = s_id.load();
				for (; !s_id.compare_exchange_weak(id, id + 1); id = s_id.load()) {}
				return Sub(evt, Id(id) + 1);
			}

			// Boolean test for 'subscribed'
			explicit operator bool() const
			{
				return m_mc != nullptr;
			}
		};

		// An RAII event un-subscriber
		struct AutoSub
		{
			Sub m_sub;

			AutoSub()
				:m_sub()
			{}
			AutoSub(Sub sub)
				:m_sub(sub)
			{}
			AutoSub(AutoSub&& rhs) noexcept
				:m_sub(rhs.m_sub)
			{
				rhs.m_sub = Sub();
			}
			AutoSub(AutoSub const& rhs) = delete;
			AutoSub& operator =(AutoSub&& rhs) noexcept
			{
				if (this == &rhs)
					return *this;

				reset();
				m_sub = rhs.m_sub;
				rhs.m_sub = Sub();
				return *this;
			}
			AutoSub& operator = (AutoSub const& rhs) = delete;
			~AutoSub()
			{
				reset();
			}

			// Unsubscribe the current subscription, if any.
			void reset()
			{
				if (m_sub.m_mc != nullptr)
					m_sub.m_mc->unsubscribe(m_sub);
			}
		};
	
		// Null proxy for 'std::mutex' when threadsafety isn't needed
		struct NoCS
		{
			std::thread::id m_id;
			NoCS() :m_id(std::this_thread::get_id()) {}
		};

		// Null proxy for 'std::lock_guard<std::mutex>' when threadsafety isn't needed
		struct NoLockGuard
		{
			NoLockGuard(NoCS cs)
			{
				if (cs.m_id == std::this_thread::get_id()) return;
				throw std::runtime_error("Cross-thread access to non-threadsafe function");
			}
		};

		// Handler storage using reusable published snapshots.
		template <typename Handler, bool ThreadSafe>
		struct Handlers
		{
			// This type prevents the need to lock a mutex for each event invocation.
			using HandlerCont = std::vector<Handler>;
			using LockGuard = std::conditional_t<ThreadSafe, std::lock_guard<std::mutex>, NoLockGuard>;
			using CS = std::conditional_t<ThreadSafe, std::mutex, NoCS>;

			// A published snapshot of the handler list.
			struct Buffer
			{
				HandlerCont m_handlers;
				mutable std::atomic_uint m_readers;

				Buffer()
					:m_handlers()
					,m_readers()
				{}
				Buffer(Buffer const&) = delete;
				Buffer& operator =(Buffer const&) = delete;
			};

			// Pins the active snapshot while it is being read.
			struct Pin
			{
				Buffer* m_buffer;

				Pin(Handlers const& owner)
					:m_buffer()
				{
					for (;;)
					{
						auto* buffer = owner.m_active.load(std::memory_order_acquire);
						buffer->m_readers.fetch_add(1, std::memory_order_acq_rel);
						if (buffer == owner.m_active.load(std::memory_order_acquire))
						{
							m_buffer = buffer;
							break;
						}
						buffer->m_readers.fetch_sub(1, std::memory_order_release);
					}
				}
				Pin(Pin const&) = delete;
				Pin& operator =(Pin const&) = delete;
				~Pin()
				{
					if (m_buffer != nullptr)
						m_buffer->m_readers.fetch_sub(1, std::memory_order_release);
				}

				// Return the pinned handler snapshot.
				HandlerCont const& handlers() const
				{
					return m_buffer->m_handlers;
				}
			};

			HandlerCont m_master;
			std::vector<std::unique_ptr<Buffer>> m_buffer;
			std::atomic<Buffer*> m_active;
			mutable CS m_cs;

			Handlers()
				:m_master()
				,m_buffer()
				,m_active()
				,m_cs()
			{
				m_buffer.reserve(2);
				m_buffer.push_back(std::make_unique<Buffer>());
				m_buffer.push_back(std::make_unique<Buffer>());
				m_active.store(m_buffer[0].get(), std::memory_order_relaxed);
			}
			Handlers(Handlers&& rhs)
				:Handlers()
			{
				initialise(take(rhs));
			}
			Handlers(Handlers const& rhs)
				:Handlers()
			{
				initialise(copy(rhs));
			}
			Handlers& operator =(Handlers&& rhs)
			{
				if (this == &rhs) return *this;
				assign(take(rhs));
				return *this;
			}
			Handlers& operator =(Handlers const& rhs)
			{
				if (this == &rhs) return *this;
				assign(copy(rhs));
				return *this;
			}

			// Invoke 'func' for the current published snapshot without copying the handler list.
			template <typename Func> void for_each(Func func) const
			{
				Pin pin(*this);
				for (auto const& h : pin.handlers())
					func(h);
			}

			// True if there are no subscribed handlers.
			bool empty() const
			{
				Pin pin(*this);
				return pin.handlers().empty();
			}

			// Return the number of subscribed handlers.
			size_t size() const
			{
				Pin pin(*this);
				return pin.handlers().size();
			}

			// Remove all subscribed handlers.
			void clear()
			{
				LockGuard lock(m_cs);
				m_master.clear();
				publish_locked();
			}

			// Append a new subscribed handler.
			void push_back(Handler handler)
			{
				LockGuard lock(m_cs);
				m_master.push_back(std::move(handler));
				publish_locked();
			}

			// Remove handlers that match 'pred'.
			template <typename Pred> void remove_if(Pred pred)
			{
				LockGuard lock(m_cs);
				auto const size = m_master.size();
				m_master.erase(std::remove_if(std::begin(m_master), std::end(m_master), pred), std::end(m_master));
				if (m_master.size() != size)
					publish_locked();
			}

		private:

			// Copy the authoritative handler list from 'rhs'.
			static HandlerCont copy(Handlers const& rhs)
			{
				LockGuard lock(rhs.m_cs);
				return rhs.m_master;
			}

			// Move the authoritative handler list from 'rhs' and publish an empty list there.
			static HandlerCont take(Handlers& rhs)
			{
				LockGuard lock(rhs.m_cs);
				auto handlers = std::move(rhs.m_master);
				rhs.m_master.clear();
				rhs.publish_locked();
				return handlers;
			}

			// Initialise this handler list without taking the thread guard.
			void initialise(HandlerCont handlers)
			{
				m_master = std::move(handlers);
				m_buffer[0]->m_handlers = m_master;
				m_active.store(m_buffer[0].get(), std::memory_order_release);
			}

			// Replace this handler list and publish the new snapshot.
			void assign(HandlerCont handlers)
			{
				LockGuard lock(m_cs);
				m_master = std::move(handlers);
				publish_locked();
			}

			// Publish the authoritative handler list to an inactive reusable buffer.
			void publish_locked()
			{
				auto* active = m_active.load(std::memory_order_acquire);
				auto* buffer = next_buffer_locked(active);
				buffer->m_handlers = m_master;
				m_active.store(buffer, std::memory_order_release);
			}

			// Return an inactive buffer that is not being read, growing the reusable pool only if needed.
			Buffer* next_buffer_locked(Buffer* active)
			{
				for (auto& buffer : m_buffer)
				{
					if (buffer.get() != active && buffer->m_readers.load(std::memory_order_acquire) == 0)
						return buffer.get();
				}

				m_buffer.push_back(std::make_unique<Buffer>());
				return m_buffer.back().get();
			}
		};
	}

	// EventHandler<Sender, Args>
	// Use:
	//   btn.Click += [&](Button&,EmptyArgs const) {...}
	//   btn.Click += std::bind(&MyDlg::HandleBtn, this, _1, _2);
	//   btn.Click += &MyDlg::HandleBtn;
	template <typename Sender, typename Args, bool ThreadSafe = false>
	struct EventHandler :multicast::IMultiCast
	{
		// The signature of the event handling function
		using Delegate = std::function<void(Sender, Args)>;
		using AutoSub = multicast::AutoSub;
		using Sub = multicast::Sub;
		using Id = multicast::Id;

		// Wraps a handler function
		struct Handler
		{
			Delegate m_delegate;
			Id m_id;

			Handler(Delegate delegate, Id id)
				:m_delegate(delegate)
				,m_id(id)
			{}
			friend bool operator == (Handler const& lhs, Sub const& rhs)
			{
				return lhs.m_id == rhs.m_id;
			}
			template <typename FuncType> friend bool operator == (Handler const& lhs, FuncType rhs)
			{
				auto f = lhs.m_delegate.target<FuncType>();
				return f != nullptr && *f == rhs;
			}
		};
		using HandlerCont = std::vector<Handler>;
		using HandlerList = multicast::Handlers<Handler, ThreadSafe>;

		// Subscribed handlers
		HandlerList m_handlers;

		// Construct
		EventHandler()
			:m_handlers()
		{}
		EventHandler(EventHandler&& rhs)
			:m_handlers()
		{
			m_handlers = std::move(rhs.m_handlers);
		}
		EventHandler(EventHandler const& rhs)
			:m_handlers()
		{
			m_handlers = rhs.m_handlers;
		}
		EventHandler& operator=(EventHandler&& rhs)
		{
			if (this == &rhs) return *this;
			m_handlers = std::move(rhs.m_handlers);
			return *this;
		}
		EventHandler& operator=(EventHandler const& rhs)
		{
			if (this == &rhs) return *this;
			m_handlers = rhs.m_handlers;
			return *this;
		}

		// Raise the event notifying subscribed observers
		void operator()(Sender s, Args a) const
		{
			// Note:
			//  - 's' is not 'Sender&' here because it can be a smart pointer or something else.
			//  - Callers should declare the template as EventHandler<MyType&, EmptyArgs const&> if references are wanted

			m_handlers.for_each([&](auto const& h)
			{
				h.m_delegate(s, a); // Can't std::forward rvalues in a loop.
			});
		}
		void operator()(Sender s) const
		{
			(*this)(s, Args{});
		}

		// Boolean test for no assigned handlers
		explicit operator bool() const
		{
			return !m_handlers.empty();
		}

		// Detach all handlers. NOTE: this invalidates all associated Handler's
		void reset()
		{
			m_handlers.clear();
		}

		// Number of attached handlers
		size_t count() const
		{
			return m_handlers.size();
		}

		// Assign/Attach/Detach handlers
		Sub operator = (Delegate func)
		{
			reset();
			return func ? *this += func : Sub{};
		}
		Sub operator += (Delegate func)
		{
			if (!func) throw std::runtime_error("Handle cannot be null");
			auto sub = Sub::Make(this);
			m_handlers.push_back(Handler(func, sub.m_id));
			return sub;
		}
		void operator -= (Sub& sub)
		{
			if (!sub)
				return;

			remove_handlers([=](auto& h) { return h == sub; });
			sub = Sub();
		}
		template <typename FuncType, typename = std::enable_if_t<std::is_same_v<decltype(std::declval<FuncType&>() == std::declval<FuncType&>()), bool>>>
		void operator -= (FuncType func)
		{
			// Notes:
			//  - This operator will only work if 'FuncType' is equality compareable
			//    In general, std::function<> does not allow operator ==.
			if (!func)
				return;

			remove_handlers([=](auto& h) { return h == func; });
		}
		template <typename Pred> void remove_handlers(Pred pred)
		{
			m_handlers.remove_if(pred);
		}

		// IMultiCast interface
		void multicast::IMultiCast::unsubscribe(Sub& sub)
		{
			*this -= sub;
		}
	};

	// MultiCast<std::function<void(int)>>
	// Use:
	//   thg.OnError += [&](int) {...}
	//   thg.OnError += pr::StaticCallback(func_ptr, ctx)
	template <typename FuncType, bool ThreadSafe = false>
	struct MultiCast :multicast::IMultiCast
	{
		// The signature of the event handling function
		using AutoSub = multicast::AutoSub;
		using Sub = multicast::Sub;
		using Id = multicast::Id;

		template <class Lambda, class... Ts>
		static constexpr auto test_sfinae(Lambda lambda, Ts&&...) -> decltype(lambda(std::declval<Ts>()...), bool{}) { return true; }
		static constexpr bool test_sfinae(...) { return false; }

		// Wraps a handler function
		struct Handler
		{
			FuncType m_delegate;
			Id m_id;

			Handler(FuncType delegate, Id id)
				:m_delegate(delegate)
				,m_id(id)
			{}
			friend bool operator == (Handler const& lhs, Sub const& rhs)
			{
				return lhs.m_id == rhs.m_id;
			}
			template <typename Func, typename = decltype(std::declval<FuncType>() == std::declval<FuncType>())>
			friend bool operator == (Handler const& lhs, Func rhs)
			{
				return lhs.m_delegate == rhs;
			}
			//friend bool operator == (Handler const& lhs, Func rhs)
			//{
			//	auto f = lhs.m_delegate.target<FuncType>();
			//	return f != nullptr && *f == rhs;
			//}
		};
		using HandlerCont = std::vector<Handler>;
		using HandlerList = multicast::Handlers<Handler, ThreadSafe>;

		// Subscribed handlers
		HandlerList m_handlers;

		// Construct
		MultiCast()
			:m_handlers()
		{}
		MultiCast(MultiCast&& rhs)
			:m_handlers()
		{
			m_handlers = std::move(rhs.m_handlers);
		}
		MultiCast(MultiCast const& rhs)
			:m_handlers()
		{
			m_handlers = rhs.m_handlers;
		}
		MultiCast& operator=(MultiCast&& rhs)
		{
			if (this == &rhs) return *this;
			m_handlers = std::move(rhs.m_handlers);
			return *this;
		}
		MultiCast& operator=(MultiCast const& rhs)
		{
			if (this == &rhs) return *this;
			m_handlers = rhs.m_handlers;
			return *this;
		}

		// Raise the event notifying subscribed observers
		template <typename... Args> void operator()(Args&&... args)
		{
			m_handlers.for_each([&](auto const& h)
			{
				h.m_delegate(args...); // Can't std::forward rvalues in a loop.
			});
		}

		// Boolean test for no assigned handlers
		explicit operator bool() const
		{
			return !m_handlers.empty();
		}

		// Detach all handlers. NOTE: this invalidates all associated Handler's
		void reset()
		{
			m_handlers.clear();
		}

		// Number of attached handlers
		size_t count() const
		{
			return m_handlers.size();
		}

		// Assign/Attach/Detach handlers
		Sub operator = (FuncType func)
		{
			reset();
			return *this += func;
		}
		Sub operator += (FuncType func)
		{
			auto sub = Sub::Make(this);
			m_handlers.push_back(Handler(func, sub.m_id));
			return sub;
		}
		void operator -= (Sub& sub)
		{
			if (!sub)
				return;

			remove_handlers([=](auto& h) { return h == sub; });
			sub = Sub();
		}
		void operator -= (FuncType func) requires (requires (FuncType f) { f == f; })
		{
			// Notes:
			//  - This operator will only work if 'FuncType' is equality compareable
			//    In general, std::function<> does not allow operator ==.
			if (!func)
				return;

			remove_handlers([=](auto& h) { return h == func; });
		}
		template <typename Pred> void remove_handlers(Pred pred)
		{
			m_handlers.remove_if(pred);
		}

		// IMultiCast interface
		void multicast::IMultiCast::unsubscribe(Sub& sub)
		{
			*this -= sub;
		}
	};

	// A subscription
	using Sub = multicast::Sub;

	// An RAII un-subscriber
	using AutoSub = multicast::AutoSub;

	// Place-holder for events that take no arguments. (Makes the templating consistent)
	struct EmptyArgs
	{};

	// Event args used in cancel-able operations
	struct CancelEventArgs :EmptyArgs
	{
		bool m_cancel;
		CancelEventArgs(bool cancel = false)
			:m_cancel(cancel)
		{}
	};

	// Event args used to report an error code and error message
	struct ErrorEventArgs :EmptyArgs
	{
		std::wstring m_msg;
		int m_code;

		ErrorEventArgs(std::wstring msg = L"", int code = 0)
			:m_msg(msg)
			,m_code(code)
		{}
	};

	// Event args used to report a property has changed
	struct PropertyChangedEventArgs :EmptyArgs
	{
		char const* m_property_name;
		PropertyChangedEventArgs(char const* prop_name)
			:m_property_name(prop_name)
		{}
	};

	// Event args used to report a change of some value
	template <typename Type> struct ChangeEventArgs :EmptyArgs
	{
		// If 'm_before' is true, this is the old value just prior to being changed.
		// If 'm_before' is false, this is the new value just after being changed.
		Type m_value;

		// True if before the change, false if after
		bool m_before;

		ChangeEventArgs(Type value, bool before)
			:m_value(value)
			,m_before(before)
		{}
		bool before() const { return m_before; }
		bool after() const { return !m_before; }
	};

	// Event for WndProc handlers translated to events
	struct WindowEventArgs
	{
		void* hwnd;
		uint32_t msg;
		uint64_t wparam;
		int64_t lparam;
		bool handled;
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/common/static_callback.h"
namespace pr::common
{
	PRUnitTestClass(EventHandlerTests)
	{
		struct Thing
		{
			int m_count;

			Thing() :m_count() {}
			Thing(Thing const&) = delete;
			Thing& operator=(Thing const&) = delete;

			void Call1() { Event1(*this, EmptyArgs{}); }
			EventHandler<Thing&, EmptyArgs const&> Event1;

			void Call2() { Event2(*this, EmptyArgs{}); }
			EventHandler<Thing&, EmptyArgs const&, true> Event2;

			void Call3() { Action1(&m_count); }
			MultiCast<std::function<void(int*)>> Action1;

			void Call4() { Action2(&m_count); }
			MultiCast<std::function<void(int*)>, true> Action2;

			void Call5() { Action3(&m_count); }
			MultiCast<StaticCB<void, int*>, true> Action3;

			// Tests classes providing handlers that can be attached to an event
			// This method can be bound to 'Event1' using &Thing::Handler because the
			// '__thiscall' 'this' parameter matches the Event Sender
			void Handler(EmptyArgs const&)
			{
				++m_count;
			}
		};
		struct Observer
		{
			int m_count;
			Observer() :m_count() {}

			// This method can be bound to 'Thing::Event1' using std::bind(&Observer::Handler, &obs, _1, _2)
			// However std::binder does not define operator == so -= has to use 'Sub'
			void Handler(Thing& thg, EmptyArgs const&)
			{
				++m_count;
				++thg.m_count;
			}
		};

		PRUnitTestMethod(EventHander, Quick)
		{
			{// lambda handler
				Thing thg;
				int count = 0;

				auto sub = thg.Event1 += [&](Thing&, EmptyArgs const&) { ++count; };
				thg.Call1();
				PR_EXPECT(count == 1);
				thg.Event1 -= sub;
				thg.Call1();
				PR_EXPECT(count == 1);
			}
			{// static function handler
				Thing thg;
				struct L { static void Handler(Thing& thing, EmptyArgs const&) { ++thing.m_count; }; };
				thg.Event1 += &L::Handler;

				PR_EXPECT(thg.m_count == 0);
				thg.Call1();
				PR_EXPECT(thg.m_count == 1);
				thg.Event1 -= &L::Handler;
				thg.Call1();
				PR_EXPECT(thg.m_count == 1);
			}
			{// Class member handler
				Thing thg;
				Observer obs;

				thg.Event1 += &Thing::Handler;
				auto sub = thg.Event1 += std::bind(&Observer::Handler, &obs, _1, _2);

				thg.Call1();
				PR_EXPECT(thg.m_count == 2);
				PR_EXPECT(obs.m_count == 1);
				thg.Event1 -= &Thing::Handler;
				thg.Event1 -= sub;
				thg.Call1();
				PR_EXPECT(thg.m_count == 2);
				PR_EXPECT(obs.m_count == 1);
			}
			{ // pr::StaticCB
				Thing thg;
				struct L { static void __stdcall Handler(void* ctx, Thing&, EmptyArgs const&) { ++*static_cast<int*>(ctx); }; };
				int count = 0, other = 0;

				thg.Event1 += StaticCB{ &count, &L::Handler };
				thg.Call1();
				PR_EXPECT(count == 1);
				thg.Event1 -= StaticCB{ &other, &L::Handler }; // should not remove anything because ctx is different
				thg.Call1();
				PR_EXPECT(count == 2);
				thg.Event1 -= StaticCB{ &count, &L::Handler }; // should remove the callback
				thg.Call1();
				PR_EXPECT(count == 2);
			}
			{// static function handler using std::bind
				Thing thg;
				auto Handler = [](void* ctx, Thing& thing, EmptyArgs const&) { thing.m_count += int(ctx == nullptr); };
				{
					AutoSub sub = thg.Event1 += std::bind(Handler, nullptr, _1, _2);
					thg.Call1();
					PR_EXPECT(thg.m_count == 1);
				}
				thg.Call1();
				PR_EXPECT(thg.m_count == 1);
			}
			{// Multiple handlers
				Thing thg;
				auto& count = thg.m_count;

				struct L { static void Handler(Thing& thing, EmptyArgs const&) { ++thing.m_count; } };
				auto const Lambda = [](Thing&, EmptyArgs const&, int* c) { ++(*c); };

				auto sub0 = thg.Event1 += [&](Thing&, EmptyArgs const&) { ++count; };
				auto sub1 = thg.Event1 += &L::Handler;
				auto sub2 = thg.Event1 += std::bind(Lambda, _1, _2, &count);

				thg.Call1();
				PR_EXPECT(count == 3);
				thg.Event1 -= &L::Handler;
				thg.Call1();
				PR_EXPECT(count == 5);
				thg.Event1 -= sub0;
				thg.Call1();
				PR_EXPECT(count == 6);
				thg.Event1 -= sub2;
				thg.Call1();
				PR_EXPECT(count == 6);
			}
			{ // Thread safety
				Thing thg;
				std::atomic_int c0 = 0, c1 = 0;
				auto t0 = std::thread([&]
				{
					for (; c0 == 0;)
					{
						AutoSub sub = thg.Event2 += [&](Thing&, EmptyArgs const&) { ++c0; };
						std::this_thread::yield();
					}
				});
				auto t1 = std::thread([&]
				{
					for (; c1 == 0;)
					{
						AutoSub sub = thg.Event2 += [&](Thing&, EmptyArgs const&) { ++c1; };
						std::this_thread::yield();
					}
				});

				int c2 = 0;
				{
					AutoSub sub = thg.Event2 += [&](Thing& t, EmptyArgs const&) { ++c2; ++t.m_count; };
					for (int i = 0; c0 == 0 || c1 == 0; ++i)
					{
						thg.Call2();
						std::this_thread::yield();
						PR_EXPECT(i < 100000);
					}
				}

				t0.join();
				t1.join();

				PR_EXPECT(thg.m_count == c2);
				PR_EXPECT(c0 > 0 && c0 <= thg.m_count);
				PR_EXPECT(c1 > 0 && c1 <= thg.m_count);
			}
			{ // Reentrant handler changes on the thread-safe path
				Thing thg;
				int c0 = 0, c1 = 0;
				Sub sub1;
				auto sub0 = thg.Event2 += [&](Thing&, EmptyArgs const&)
				{
					++c0;
					if (!sub1)
						sub1 = thg.Event2 += [&](Thing&, EmptyArgs const&) { ++c1; };
				};

				thg.Call2();
				PR_EXPECT(c0 == 1);
				PR_EXPECT(c1 == 0);

				thg.Call2();
				PR_EXPECT(c0 == 2);
				PR_EXPECT(c1 == 1);

				thg.Event2 -= sub0;
				thg.Event2 -= sub1;
			}
		}
		PRUnitTestMethod(MultiCast, Quick)
		{
			{// lambda handler
				Thing thg;
				int count = 0;

				auto sub = thg.Action1 += [&](int* c) { *c = ++count; };
				thg.Call3();
				PR_EXPECT(count == 1);
				thg.Action1 -= sub;
				thg.Call3();
				PR_EXPECT(count == 1);
			}
			{// static function handler
				Thing thg;
				struct L { static void __stdcall Handler(int* count) { ++(*count); }; };
				auto sub = thg.Action1 += &L::Handler;

				PR_EXPECT(thg.m_count == 0);
				thg.Call3();
				PR_EXPECT(thg.m_count == 1);
				thg.Action1 -= sub;// &L::Handler;
				thg.Call3();
				PR_EXPECT(thg.m_count == 1);
			}
			{ // pr::StaticCB
				Thing thg;
				struct L { static void __stdcall Handler(void* ctx, int*) { ++*static_cast<int*>(ctx); }; };
				int count = 0, other = 0;

				thg.Action3 += StaticCB{ &count, &L::Handler};
				thg.Call5();
				PR_EXPECT(count == 1);
				thg.Action3 -= StaticCB{ &other, &L::Handler }; // should not remove anything because ctx is different
				thg.Call5();
				PR_EXPECT(count == 2);
				thg.Action3 -= StaticCB{ &count, &L::Handler}; // should remove the callback
				thg.Call5();
				PR_EXPECT(count == 2);
			}
			{// static function handler using std::bind
				Thing thg;
				auto Handler = [](void* ctx, int* c) { *c += int(ctx == nullptr); };
				{
					AutoSub sub = thg.Action1 += std::bind(Handler, nullptr, _1);
					thg.Call3();
					PR_EXPECT(thg.m_count == 1);
				}
				thg.Call3();
				PR_EXPECT(thg.m_count == 1);
			}
			{// Multiple handers
				Thing thg;
				auto& count = thg.m_count;

				struct L { static void __stdcall Handler(int* c) { ++(*c); } };
				auto const Lambda = [](void*, int* c) { ++(*c); };

				auto sub0 = thg.Action1 += [&](int*) { ++count; };
				auto sub1 = thg.Action1 += &L::Handler;
				auto sub2 = thg.Action1 += std::bind(Lambda, nullptr, _1);

				thg.Call3();
				PR_EXPECT(count == 3);
				thg.Action1 -= sub1;// &L::Handler;
				thg.Call3();
				PR_EXPECT(count == 5);
				thg.Action1 -= sub0;
				thg.Call3();
				PR_EXPECT(count == 6);
				thg.Action1 -= sub2;
				thg.Call3();
				PR_EXPECT(count == 6);
			}
			{// thread safety
				Thing thg;
				std::atomic_int c0 = 0, c1 = 0;
				auto t0 = std::thread([&]
					{
						for (; c0 == 0;)
						{
							AutoSub sub = thg.Action2 += [&](int*) { ++c0; };
							std::this_thread::yield();
						}
					});
				auto t1 = std::thread([&]
					{
						for (; c1 == 0;)
						{
							AutoSub sub = thg.Action2 += [&](int*) { ++c1; };
							std::this_thread::yield();
						}
					});

				int c2 = 0;
				{
					AutoSub sub = thg.Action2 += [&](int* c) { ++c2; ++(*c); };
					for (int i = 0; c0 == 0 || c1 == 0; ++i)
					{
						thg.Call4();
						std::this_thread::yield();
						PR_EXPECT(i < 100000);
					}
				}

				t0.join();
				t1.join();

				PR_EXPECT(thg.m_count == c2);
				PR_EXPECT(c0 > 0 && c0 <= thg.m_count);
				PR_EXPECT(c1 > 0 && c1 <= thg.m_count);
			}
		}
	};
}
#endif
