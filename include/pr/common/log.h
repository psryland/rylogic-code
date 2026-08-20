//*****************************************************************************************
// Log
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************
#pragma once
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <functional>
#include <format>
#include <memory>
#include <chrono>
#include <atomic>
#include <mutex>
#include <thread>
#include <concepts>
#include <cassert>
#include <cstdint>
#include <concurrent_queue.h>
#include <libloaderapi.h>
#include <debugapi.h>

#include "pr/gui/terminal_colours.h"

// Notes:
//  If you create a log function like this:
//     inline Logger& Log() { static Logger s_log; return s_log; }
//  be careful about async access. Multiple threads calling the Log() function
//  is a race condition, you need to instantiate the static s_log object first.

namespace pr::log
{
	enum class ELevel :uint8_t
	{
		Debug,
		Info,
		Warn,
		Error,
	};
	constexpr char const* ToString(ELevel level)
	{
		switch (level)
		{
			case ELevel::Debug: return "Debug";
			case ELevel::Info:  return "Info";
			case ELevel::Warn:  return "Warn";
			case ELevel::Error: return "Error";
			default: return "Unknown";
		}
	}

	// Event types
	enum class EEventType :uint8_t
	{
		Normal,
		Fence,
		TerminationSentinel,
	};

	// Log message behaviour
	enum class EMode
	{
		Async,
		Immediate,
	};

	// Timer
	using RTC = std::chrono::high_resolution_clock;
	inline std::string ToString(RTC::duration ts)
	{
		using namespace std::chrono;
		auto hours = duration_cast<std::chrono::hours>(ts); ts -= hours;
		auto mins = duration_cast<minutes>(ts); ts -= mins;
		auto secs = duration_cast<seconds>(ts); ts -= secs;
		auto ms = duration_cast<milliseconds>(ts);
		return std::format("{:02}:{:02}:{:02}:{:03}", hours.count(), mins.count(), secs.count(), ms.count());
	}

	// An individual log event
	struct Event
	{
		using path = std::filesystem::path;

		ELevel           m_level;       // Debug, Info, Warn, Error
		EEventType       m_event_type;  // Normal, Fence, TerminationSentinel
		uint16_t         m_event_data;  // Data specific to the event type
		path             m_file;        // Source file
		int              m_line;        // Line number in the source file
		int              m_occurrences; // When event type != normal, this is the fence count
		RTC::duration    m_timestamp;   // Time since logging started
		std::string_view m_context;     // Context
		std::string      m_msg;         // Message

		Event()
			: m_level()
			, m_event_type()
			, m_event_data()
			, m_file()
			, m_line()
			, m_occurrences()
			, m_timestamp()
			, m_context()
			, m_msg()
		{}
		Event(ELevel level, RTC::time_point tzero, std::string_view ctx, std::string_view msg, path const& file, int line)
			: m_level(level)
			, m_event_type()
			, m_event_data()
			, m_file(file)
			, m_line(line)
			, m_occurrences(1)
			, m_timestamp(RTC::now() - tzero)
			, m_context(ctx)
			, m_msg(msg)
		{}
		Event(ELevel level, RTC::time_point tzero, std::string_view ctx, std::string&& msg, path const& file, int line)
			: m_level(level)
			, m_event_type()
			, m_event_data()
			, m_file(file)
			, m_line(line)
			, m_occurrences(1)
			, m_timestamp(RTC::now() - tzero)
			, m_context(ctx)
			, m_msg(std::move(msg))
		{}
		explicit Event(EEventType event_type)
			: Event()
		{
			static std::atomic_int s_fence = 0;
			m_event_type = event_type;
			m_event_data = static_cast<decltype(m_event_data)>(++s_fence);
		}

		// Compare two event for equality
		static bool Same(Event const& lhs, Event const& rhs)
		{
			return
				lhs.m_level == rhs.m_level &&
				lhs.m_context == rhs.m_context &&
				lhs.m_file == rhs.m_file &&
				lhs.m_line == rhs.m_line &&
				lhs.m_msg == rhs.m_msg &&
				lhs.m_event_type == EEventType::Normal &&
				rhs.m_event_type == EEventType::Normal;
		}

		// Write 'ev' as a string to 'stream'
		friend std::ostream& operator << (std::ostream& stream, Event const& ev)
		{
			char const* delim = "";
			char const* colour =
				ev.m_level == ELevel::Debug ? gui::TermColours::Gray :
				ev.m_level == ELevel::Info ? gui::TermColours::White :
				ev.m_level == ELevel::Warn ? gui::TermColours::Yellow :
				ev.m_level == ELevel::Error ? gui::TermColours::Red :
				"";

			if (!ev.m_file.empty()) { stream << ev.m_file; delim = " "; }
			if (ev.m_line != -1) { stream << "(" << ev.m_line << "):"; delim = " "; }
			stream << std::format("{}{:8}|{}{}{}|{}|{}\n", delim, ev.m_context, colour, ToString(ev.m_level), gui::TermColours::Reset, ToString(ev.m_timestamp), ev.m_msg);
			return stream;
		}
		friend std::string ToString(Event const& ev)
		{
			std::stringstream s;
			s << ev;
			return s.str();
		}
	};

	// Write log output to stdout
	struct ToStdout
	{
		ELevel m_level = ELevel::Debug;
		void operator ()(Event const& ev)
		{
			if (ev.m_level < m_level) return;
			std::cout << ev;
		}
	};

	// Write log output to stderr
	struct ToStderr
	{
		ELevel m_level = ELevel::Debug;
		void operator ()(Event const& ev)
		{
			if (ev.m_level < m_level) return;
			std::cerr << ev;
		}
	};

	// Write log output to the debug output window
	struct ToOutputDebugString
	{
		ELevel m_level = ELevel::Debug;
		void operator ()(Event const& ev)
		{
			if (ev.m_level < m_level) return;
			OutputDebugStringA(ToString(ev).c_str());
		}
	};

	// Helper object for writing log output to a file
	struct ToFile
	{
		std::filesystem::path m_filepath;
		std::shared_ptr<std::ofstream> m_outf;

		ToFile(std::filesystem::path const& filepath, std::ios_base::openmode mode = std::ios_base::out)
			: m_filepath(filepath)
			, m_outf(std::make_shared<std::ofstream>(filepath, mode))
		{}
		void operator ()(Event const& ev)
		{
			auto& fp = *m_outf;
			fp.seekp(0, std::ios_base::end);
			fp << ev;
			fp.flush();
		}
	};

	// Interface for a shared log context
	struct IContext
	{
		// The time point when logging started
		log::RTC::time_point const m_time_zero;

		IContext()
			: m_time_zero(log::RTC::now())
		{
		}

		// Enable/Disable immediate mode.
		// In immediate mode, log events are written to 'log_cb' instead of being queued for processing
		// by the background thread. Useful when you want the log to be written in sync with debugging.
		virtual void WriteMode(EMode mode) = 0;

		// Queue a log event for writing to the log callback function
		virtual void Enqueue(log::Event&& ev) = 0;

		// Pull an event from the queue of log events
		virtual bool Dequeue(log::Event& ev) = 0;

		// Wait for all log events to be flushed (all at the time of calling)
		virtual void Flush() {}
	};

	// Logger context. A single Context is shared by many instances of Logger
	template <std::invocable<Event> OutputCB>
	class Context : public IContext
	{
		friend struct Logger;
			
		// Producer/Consumer queue for log events
		using log_queue_t = concurrency::concurrent_queue<log::Event>;

		// Queue of log events to report
		log_queue_t m_queue;

		// A signal for when a log event is added to the queue
		std::condition_variable m_cv_queue;
		std::mutex m_mutex;
			
		// A signal for when a fence message is reached in the log
		std::condition_variable m_cv_fence;
		std::atomic_int m_fence;

		// A callback function used in immediate mode
		OutputCB m_output_cb;
		EMode m_mode;

		// The worker thread that forwards log events to the callback function
		std::thread m_thread;

		// Workaround for "this used in initializer list" warning
		Context& this_() { return *this; }

		// Thread entry point for consuming log events
		template <std::invocable<Event> OutputCB>
		static void LogConsumerThread(Context& ctx, OutputCB log_cb, int const occurrences_batch_size = 0)
		{
			using namespace std::chrono;
			try
			{
				SetThreadName(L"pr::Logger::LogConsumerThread");

				log::Event ev, prev;
				for (;;)
				{
					// Wait for an event to arrive
					{
						std::unique_lock<std::mutex> lock(ctx.m_mutex);
						ctx.m_cv_queue.wait(lock, [&] { return ctx.Dequeue(ev); });
					}

					// Is it the same as the prevent event
					auto is_same = log::Event::Same(ev, prev);

					// Same event as last time? add it to the batch
					if (is_same && prev.m_occurrences < occurrences_batch_size)
					{
						++prev.m_occurrences;
						prev.m_timestamp = ev.m_timestamp;
						continue;
					}

					// Have events been batched? Report them now
					if (prev.m_occurrences != 0)
					{
						log_cb(prev);
						prev.m_occurrences = 0;
					}

					// Start of the next batch (and batching is enabled)? Add it to the batch
					if (is_same && occurrences_batch_size != 0)
					{
						prev.m_occurrences = 1;
						prev.m_timestamp = ev.m_timestamp;
						continue;
					}

					// Control event?
					switch (ev.m_event_type)
					{
						case log::EEventType::TerminationSentinel:
						{
							ctx.m_fence = ev.m_event_data;
							ctx.m_cv_fence.notify_all();
							return;
						}
						case log::EEventType::Fence:
						{
							ctx.m_fence = ev.m_event_data;
							ctx.m_cv_fence.notify_all();
							continue;
						}
						case log::EEventType::Normal:
						{
							// Log the event to the output
							log_cb(ev);
							prev = ev;
							prev.m_occurrences = 0;
							break;
						}
						default:
						{
							throw std::runtime_error("Unknown control event type");
						}
					}
				}
			}
			catch (...)
			{
				assert(!"Unknown exception in log thread");
			}
		}

		// Set the name of the current thread
		static void SetThreadName(wchar_t const* name)
		{
			// Call 'SetThreadDescription'. 'SetThreadDescription' only exists on >= Win10
			if (auto kernal32 = ::LoadLibraryW(L"kernel32.dll"))
			{
				using GetCurrentThreadFn = HANDLE(__stdcall*)();
				using SetThreadDescriptionFn = HRESULT(__stdcall*)(HANDLE hThread, PCWSTR lpThreadDescription);

				auto SetThreadDescription = reinterpret_cast<SetThreadDescriptionFn>(GetProcAddress(kernal32, "SetThreadDescription"));
				auto GetCurrentThread = reinterpret_cast<GetCurrentThreadFn>(GetProcAddress(kernal32, "GetCurrentThread"));
				if (SetThreadDescription && GetCurrentThread)
					SetThreadDescription(GetCurrentThread(), &name[0]);

				::FreeLibrary(kernal32);
				return;
			}
		}

	public:

		Context(OutputCB log_cb, EMode mode, int occurrences_batch_size)
			: IContext()
			, m_queue()
			, m_cv_queue()
			, m_cv_fence()
			, m_mutex()
			, m_fence()
			, m_output_cb(log_cb)
			, m_mode(mode)
			, m_thread(LogConsumerThread<OutputCB>, std::ref(this_()), log_cb, occurrences_batch_size)
		{}
		~Context()
		{
			m_mode = EMode::Async;
			Enqueue(log::Event(log::EEventType::TerminationSentinel));
			if (m_thread.joinable())
				m_thread.join();
		}

		// Enable/Disable immediate mode.
		// In immediate mode, log events are written to 'log_cb' instead of being queued for processing
		// by the background thread. Useful when you want the log to be written in sync with debugging.
		void WriteMode(EMode mode) override
		{
			m_mode = mode;
		}

		// Queue a log event for writing to the log callback function
		void Enqueue(log::Event&& ev) override
		{
			// Immediate mode?
			if (m_mode == EMode::Immediate)
			{
				m_output_cb(ev);
				return;
			}

			// Otherwise queue the log event for the background thread
			m_queue.push(std::move(ev));
			m_cv_queue.notify_all();
		}

		// Pull an event from the queue of log events
		bool Dequeue(log::Event& ev) override
		{
			return m_queue.try_pop(ev);
		}

		// Wait for all log events to be flushed (all at the time of calling)
		void Flush() override
		{
			auto fence = log::Event(log::EEventType::Fence);
			m_queue.push(fence);

			std::unique_lock<std::mutex> lock(m_mutex);
			m_cv_fence.wait(lock, [&]{ return m_fence >= fence.m_event_data; });
		}
	};

	// Null context for disabled logging
	class NullContext : public IContext
	{
	public:

		// Default for disabled logging
		inline static std::shared_ptr<NullContext> const Instance = {};

		// Enable/Disable immediate mode.
		// In immediate mode, log events are written to 'log_cb' instead of being queued for processing
		// by the background thread. Useful when you want the log to be written in sync with debugging.
		void WriteMode(EMode) override
		{
		}

		// Queue a log event for writing to the log callback function
		void Enqueue(log::Event&&) override
		{
		}

		// Pull an event from the queue of log events
		bool Dequeue(log::Event&) override
		{
			return false;
		}
	};

	// Provides logging support
	struct Logger
	{
		using OutputCB = std::function<void(log::Event const&)>;

		// The shared Context that this instance references
		std::shared_ptr<IContext> m_context;

		// An id to use in log messages
		std::string Tag;

		// On/Off switch for logging
		std::atomic_bool Enabled;

		Logger()
			: m_context(NullContext::Instance)
			, Tag()
			, Enabled()
		{
		}
		Logger(std::string_view tag, OutputCB log_cb, EMode mode, int occurrences_batch_size = 0)
			: m_context(new Context(log_cb, mode, occurrences_batch_size))
			, Tag(tag)
			, Enabled()
		{
			Enabled = true;
		}
		Logger(std::string_view tag, Logger const& rhs)
			:m_context(rhs.m_context)
			,Tag(tag)
			,Enabled()
		{
			Enabled = rhs.Enabled.load();
		}
		Logger(Logger&&) = default;
		Logger(Logger const&) = delete;
		Logger& operator =(Logger&&) = default;
		Logger& operator =(Logger const&) = delete;

		// Access to the shared logger context
		IContext& SharedContext() const
		{
			return *m_context;
		}

		// Log a message
		void Write(log::ELevel level, std::string_view msg, std::filesystem::path const& file = "", int line = -1) const
		{
			if (!Enabled) return;
			log::Event evt(level, m_context->m_time_zero, Tag, msg, file, line);
			m_context->Enqueue(std::move(evt));
		}

		// Log an exception with message 'msg'
		void Write(log::ELevel level, std::exception const& ex, std::string_view msg, std::filesystem::path const& file = "", int line = -1) const
		{
			if (!Enabled) return;
			auto message = std::format("{} - Exception: {}", msg, ex.what());
			log::Event evt(level, m_context->m_time_zero, Tag, message, file, line);
			m_context->Enqueue(std::move(evt));
		}

		// Block the caller until the logger is idle
		void Flush() const
		{
			if (!Enabled) return;
			m_context->Flush();
		}
	};
}

#if PR_LOGGING == 1
	#define PR_LOG(logger, level, message)          do { logger.Write(pr::log::ELevel::level,         (message), __FILE__, __LINE__); } while (0)
	#define PR_LOGE(logger, level, except, message) do { logger.Write(pr::log::ELevel::level, except, (message), __FILE__, __LINE__); } while (0)
#else
	#define PR_LOG(logger, level, message)          do {} while (0)
	#define PR_LOGE(logger, level, except, message) do { (void)(except); } while (0)
#endif

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::log
{
	PRUnitTest(LogTests, Quick)
	{
		std::string str;

		{// Null log instance
			Logger log1;
			Logger log2("test", log1);

			log2.Write(ELevel::Debug, "event 1");
			log2.Flush();
		}
		{// Single instance
			str.resize(0);
			Logger log("test", [&](Event const& ev)
			{
				std::stringstream s;
				s << log::ToString(ev.m_level) << "," << ev.m_context << ": " << ev.m_msg << "," << ev.m_occurrences << std::endl;
				str += s.str();
			}, EMode::Async, 0);
			log.Write(ELevel::Debug, "event 1");
			log.Flush();
			PR_EXPECT(str == "Debug,test: event 1,1\n");
		}
		{// Copied instances
			str.resize(0);
			Logger log1("log1", [&](Event const& ev)
			{
				std::stringstream s;
				s << log::ToString(ev.m_level) << "," << ev.m_context << ": " << ev.m_msg << "," << ev.m_occurrences << std::endl;
				str += s.str();
			}, EMode::Async, 0);
			Logger log2("log2", log1);

			log1.Write(ELevel::Info, "event 1");
			log2.Write(ELevel::Debug, "event 2");
			log1.Write(ELevel::Info, "event 3");
			log1.Flush();
			PR_EXPECT(str ==
				"Info,log1: event 1,1\n"
				"Debug,log2: event 2,1\n"
				"Info,log1: event 3,1\n"
				);
		}
	}
}
#endif
