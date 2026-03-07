//**********************************
// Pipe
//  Copyright (C) Rylogic Ltd 2007
//**********************************
#pragma once
#include <vector>
#include <unordered_map>
#include <string>
#include <utility>
#include <format>
#include <stdexcept>
#include <cstdint>
#include <chrono>
#include <ranges>
#include <mutex>
#include <stop_token>
#include <windows.h>

#include "pr/common/cast.h"
#include "pr/common/hresult.h"
#include "pr/common/event_handler.h"
#include "pr/common/log.h"

namespace pr::pipe
{
	// Pipe mode
	enum class EMode
	{
		Server,
		Client,
	};

	class Pipe
	{
		// Notes:
		//  - This implementation uses an IO Completion Port which is supposed
		//    to handle IO operations more efficiently than with separate threads.

		// State of the pipe being communicated over
		enum class EState
		{
			// The pipe has been created, but no connection has been made
			Disconnected,

			// A request has been made to connect to another client, but no client has yet connected
			ConnectPending,

			// A client has connected to the other side of the pipe
			Connected,

			// The pipe has been broken, likely because the client on the other end disconnected
			Broken,

			// Clean up and exit
			Shutdown,
		};

		struct handle_closer { void operator()(HANDLE h) { if (h) CloseHandle(h); } };
		using ScopedHandle = std::unique_ptr<void, handle_closer>;
	
		ScopedHandle SafeHandle(HANDLE h) { return ScopedHandle((h != INVALID_HANDLE_VALUE) ? h : nullptr); }

		// A type for a buffer of bytes
		using buffer_t = std::vector<uint8_t>;

		// Different types of async operations that the IPC pipe queues
		enum class EAsyncOp
		{
			None = 0,
			Connect,
			Read,
			Send,
			Reconnect,
			Shutdown,
		};

		// Overlapped structure passed through the IPC IOCP for use on completion of async ops
		struct Overlapped : OVERLAPPED
		{
			// Which operation does this OVERLAPPED represent
			EAsyncOp m_op;
			EMode m_owner;

			// Message being built up over one or more async read operations
			// 'm_len' is the length of data read so far. 'm_data' can be larger than this.
			int64_t m_len;
			buffer_t m_data;

			bool m_used;

			Overlapped()
				: OVERLAPPED()
				, m_op()
				, m_owner()
				, m_len()
				, m_data(PipeBufferSize)
				, m_used()
			{}
			Overlapped(Overlapped&&) = delete;
			Overlapped(Overlapped const&) = delete;

			std::span<uint8_t const> data() const
			{
				return { m_data.data(), static_cast<size_t>(m_len) };
			}
			std::span<uint8_t> buf()
			{
				return m_data;
			}
			//uint8_t const* data() const
			//{
			//	return m_data.data();
			//}
			//uint8_t* data()
			//{
			//	return m_data.data();
			//}
			//int64_t data_len() const
			//{
			//	return m_len;
			//}
			//size_t buf_size() const
			//{
			//	return m_data.size();
			//}
			void append(std::span<uint8_t const> data)
			{
				if (m_len + data.size() > m_data.size())
					grow(s_cast<size_t>(m_len + data.size()));

				memcpy(m_data.data() + m_len, data.data(), data.size());
				m_len += isize(data);
			}
			void append(std::string_view message)
			{
				append(std::span<uint8_t const>(reinterpret_cast<uint8_t const*>(message.data()), message.size()));
			}
			void grow(size_t min_new_size)
			{
				auto new_size = std::max(min_new_size, m_data.size() * 2);
				m_data.resize(new_size);
			}
			void grow()
			{
				grow(m_data.size() * 2);
			}
			void shrink()
			{
				m_data.resize(s_cast<size_t>(m_len));
			}
		};

		// Helper for automatically returning an overlapped object to the free pool
		struct OverlappedReturner
		{
			Pipe& m_pipe;
			Overlapped& m_overlapped;
			bool m_retain;

			OverlappedReturner(Overlapped& overlapped, Pipe& pipe, bool retain = false)
				: m_pipe(pipe)
				, m_overlapped(overlapped)
				, m_retain(retain)
			{
			}
			OverlappedReturner(OverlappedReturner&&) = delete;
			OverlappedReturner(OverlappedReturner const&) = delete;
			OverlappedReturner& operator = (OverlappedReturner&&) = delete;
			OverlappedReturner& operator = (OverlappedReturner const&) = delete;
			~OverlappedReturner()
			{
				if (m_retain) return;
				m_pipe.Return(m_overlapped);
			}
		};

		using OverlappedPtr = std::shared_ptr<Overlapped>;
		using OverlappedCont = std::vector<OverlappedPtr>;

	private:

		// The mode this object is in (server or client)
		EMode m_mode;

		// Name of the pipe the IPC is communicating over
		std::string m_pipe_name;

		// Named pipe used for the IPC
		ScopedHandle m_pipe;

		// IO completion port monitoring the pipe
		ScopedHandle m_iocp;

		// A container of allocated Overlapped objects and the number in the container that are used.
		// The first half of the container up to 'm_used' are in-flight operations.
		OverlappedCont m_pool;
		int64_t m_used;

		// Mutex guarding access to Ops
		std::mutex m_mutex_pool;

		// A buffer of messages received before the pipe is connected. Send upon successful connection.
		std::vector<std::vector<uint8_t>> m_saved_messages;
		std::mutex m_mutex_saved_messages;

		log::Logger m_log;

		// Read/Write buffer size
		static constexpr int PipeBufferSize = 4096;

	public:

		struct Options
		{
			std::chrono::milliseconds ProcessIOWaitTime{ 10 };
			std::chrono::milliseconds SleepWhileDisconectedTime{ 10 };

		} m_options;

		Pipe(EMode mode, std::string_view pipe_name, Options const& options = {}, log::Logger const& log_parent = {})
			: m_mode(mode)
			, m_pipe_name(pipe_name)
			, m_pipe()
			, m_iocp()
			, m_pool()
			, m_used()
			, m_mutex_pool()
			, m_saved_messages()
			, m_mutex_saved_messages()
			, m_log(mode == EMode::Server ? "Server" : "Client", log_parent)
			, m_options(options)
			, MessageReceived()
		{
			CreatePipe();
		}

		// Take over this thread to process incoming and outgoing IPC communication
		void Run(std::stop_token shutdown) // Worker thread context
		{
			auto new_connection = true;
			auto state = EState::Disconnected;
			for (; !shutdown.stop_requested() && state != EState::Shutdown;)
			{
				try
				{
					// Run the state machine
					switch (state)
					{
						// While not connected, attempt to connect.
						case EState::Disconnected:
						{
							// Attempt to connect the pipe
							if (m_mode == EMode::Server)
								state = ConnectServerPipe();
							if (m_mode == EMode::Client)
								state = ConnectClientPipe();
							
							new_connection = true;
							break;
						}
						case EState::ConnectPending:
						{
							// While connecting, just wait
							state = ProcessIO(m_options.ProcessIOWaitTime, state);
							new_connection = true;
							break;
						}
						case EState::Connected:
						{
							// If just connected, send buffer messages and start a read operation
							if (new_connection)
							{
								new_connection = false;
								SendSavedMessages();
								QueueRead();
							}

							// While connected, read from the pipe and consume queue IOCP notifications.
							state = ProcessIO(m_options.ProcessIOWaitTime, state);
							break;
						}
						case EState::Broken:
						{
							// While broken, disconnect then reconnect
							Disconnect();
							CreatePipe();
							state = EState::Disconnected;
							break;
						}
						case EState::Shutdown:
						{
							// Thread shutdown requested
							break;
						}
						default:
						{
							throw std::runtime_error(std::format("Unknown IpcChannel state: {}", int(state)));
						}
					}
				}
				catch (std::exception const& ex)
				{
					state = EState::Broken;
					OutputDebugStringA(ex.what());
					std::this_thread::sleep_for(m_options.SleepWhileDisconectedTime);
					continue;
				}
			}

			// Exit 'Run' in the disconnected state and ready to be run again
			Disconnect();
		}

		// Send a message from this IPC object to the remote IPC object asynchronously.
		// When this function returns, the message is queued but it may not yet be delivered.
		void Write(std::string_view message)
		{
			if (m_mode == EMode::Server && message == "Message To Server")
				assert(false);

			return Write(std::span<uint8_t const>(reinterpret_cast<uint8_t const*>(message.data()), message.size()));
		}
		void Write(std::span<uint8_t const> data)
		{
			// Get an OVERLAPPED structure for the operation and append the message
			auto& overlapped = GetOverlapped(EAsyncOp::Send);
			OverlappedReturner cleaner(overlapped, *this);
			overlapped.append(data);

			// Attempt to send the message over the pipe
			auto message = overlapped.data();
			auto r = WriteFile(m_pipe.get(), message.data(), s_cast<DWORD>(message.size()), nullptr, &overlapped);
			auto error = r ? ERROR_SUCCESS : GetLastError();
			switch (error)
			{
				case ERROR_SUCCESS: // The send was completed.
				{
					m_log.Write(log::ELevel::Debug, std::format("Send complete immediate: {}", Summary(data)));
					break;
				}
				case ERROR_IO_PENDING: // The send was started but is not complete.
				{
					// Message send is in progress, preserve 'overlapped'
					m_log.Write(log::ELevel::Debug, std::format("Send started: {}", Summary(data)));
					cleaner.m_retain = true;
					break;
				}
				case ERROR_PIPE_LISTENING: // The pipe is not connected.
				case ERROR_INVALID_HANDLE: // The pipe has not been created
				{
					SaveMessage(data);
					break;
				}
				case ERROR_NO_DATA: // The client has closed their end of the pipe.
				{
					SaveMessage(data);
					QueueSignal(EAsyncOp::Reconnect); // Queue a message to reconnect the pipe
					break;
				}
				default:
				{
					Check(HRESULT_FROM_WIN32(error), "WriteFile failed");
					return;
				}
			}
		}

		// Message received event
		MultiCast<std::function<void(std::span<uint8_t const> data)>, true> MessageReceived;

	private:
		
		// Grab an overlapped object from the pool
		Overlapped& GetOverlapped(EAsyncOp op)
		{
			std::scoped_lock<std::mutex> lock(m_mutex_pool);
			if (m_used == isize(m_pool))
				m_pool.push_back(OverlappedPtr{ new Overlapped() });

			Overlapped& overlapped = *m_pool[s_cast<size_t>(m_used++)];
			overlapped.m_op = op;
			overlapped.m_owner = m_mode;
			overlapped.m_data.resize(PipeBufferSize);
			overlapped.m_len = 0;
			overlapped.m_used = true;
			return overlapped;
		}

		// Return an overlapped object to the free pool
		void Return(Overlapped& overlapped)
		{
			std::scoped_lock<std::mutex> lock(m_mutex_pool);
			overlapped.m_used = false;
			//overlapped.m_op = EAsyncOp::None;

			auto beg = m_pool.data();
			auto end = beg + m_used;
			for (; beg != end; ++beg)
			{
				if (beg->get() != &overlapped)
					continue;

				if (beg != end - 1)
					std::swap(*beg, *(end - 1));

				--m_used;
				break;
			}
		}

		// Return all overlapped objects to the free pool
		void ReturnAll()
		{
			std::scoped_lock<std::mutex> lock(m_mutex_pool);
			for (int i = 0; i != m_used != 0; ++i)
			{
				auto& overlapped = *m_pool[i];
				overlapped.m_op = EAsyncOp::None;
			}
			m_used = 0;
		}

		// Create/Recreate the pipe
		void CreatePipe()
		{
			m_iocp = nullptr;
			m_pipe = nullptr;

			// The server creates the pipe, the client connects to it
			if (m_mode != EMode::Server)
				return;

			// Create and configure the named pipe as well as configure the IOCP for this handle
			auto pipe = SafeHandle(CreateNamedPipeA(
				m_pipe_name.c_str(),
				PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
				PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT | PIPE_ACCEPT_REMOTE_CLIENTS,
				PIPE_UNLIMITED_INSTANCES,
				static_cast<DWORD>(PipeBufferSize),
				static_cast<DWORD>(PipeBufferSize),
				NMPWAIT_USE_DEFAULT_WAIT,
				nullptr));

			// The pipe couldn't be created
			if (pipe == nullptr)
			{
				auto error = GetLastError();
				Check(HRESULT_FROM_WIN32(error), "Failed to create named pipe (as server)");
				return;
			}

			// Create an IO completion port so we can receive notification of completed IO operations.
			auto iocp = SafeHandle(CreateIoCompletionPort(pipe.get(), nullptr, 0, 0));
			if (iocp == nullptr)
			{
				auto error = GetLastError();
				Check(HRESULT_FROM_WIN32(error), "Failed to create completion IO port (as server)");
				return;
			}

			m_pipe = std::move(pipe);
			m_iocp = std::move(iocp);
		}

		// Attempt to connect a client IPC object to the pipe
		EState ConnectClientPipe()
		{
			// Client connection involves opening a file to the named pipe that was created by the server,
			// configuring that file to match the settings of the named pipe, and allocating an IOCP for it.
			auto pipe = SafeHandle(CreateFileA(
				m_pipe_name.c_str(),
				GENERIC_READ | GENERIC_WRITE,
				FILE_SHARE_READ | FILE_SHARE_WRITE,
				nullptr,
				OPEN_EXISTING,
				FILE_FLAG_OVERLAPPED,
				nullptr));

			// Deal with rejection
			if (pipe == nullptr)
			{
				auto error = GetLastError();
				switch (error)
				{
					// Server is up but busy, wait for availability
					case ERROR_PIPE_BUSY:
					{
						// Wait for the server to be not-busy
						if (WaitNamedPipeA(m_pipe_name.c_str(), NMPWAIT_USE_DEFAULT_WAIT))
							return EState::Connected;

						// 'ERROR_BAD_PATHNAME' can indicate that the pipe is currently being setup from the server - loop until it can connect
						auto wait_error = GetLastError();
						if (wait_error == ERROR_BAD_PATHNAME)
							return EState::Disconnected;

						// Busy for some other reason
						Check(HRESULT_FROM_WIN32(wait_error), "Pipe Client: WaitNamedPipe failed");
						return EState::Disconnected;
					}
					case ERROR_FILE_NOT_FOUND:
					{
						// Pipe name doesn't exist yet
						return EState::Disconnected;
					}
					default:
					{
						// Some other error
						Check(HRESULT_FROM_WIN32(error), "Pipe Client: Failed to open pipe handle");
						return EState::Disconnected;
					}
				}
			}

			// Set the read mode of the pipe
			DWORD mode = PIPE_READMODE_MESSAGE | PIPE_WAIT;
			if (!SetNamedPipeHandleState(pipe.get(), &mode, nullptr, nullptr))
			{
				auto error = GetLastError();
				Check(HRESULT_FROM_WIN32(error), "SetNamedPipeHandleState failed");
				return EState::Disconnected;
			}

			// Create an IO completion port to queue async IO operations
			auto iocp = SafeHandle(CreateIoCompletionPort(pipe.get(), nullptr, 0, 0));
			if (iocp == nullptr)
			{
				auto error = GetLastError();
				Check(HRESULT_FROM_WIN32(error), "CreateIoCompletionPort failed");
				return EState::Disconnected;
			}

			m_log.Write(log::ELevel::Info, "Connect complete");

			m_pipe = std::move(pipe);
			m_iocp = std::move(iocp);
			return EState::Connected;
		}

		// Attempt to connect a server IPC object to the pipe
		EState ConnectServerPipe()
		{
			// The pipe should've been created at construction time
			if (!m_pipe)
				throw std::runtime_error("Pipe handle doesn't exist");

			// Get an overlapped object from the pool
			auto& overlapped = GetOverlapped(EAsyncOp::Connect);
			OverlappedReturner cleaner(overlapped, *this);

			// Connect the pipe
			if (ConnectNamedPipe(m_pipe.get(), &overlapped))
			{
				// Async connect is always expected to return FALSE even if the pipe is already connected
				auto error = GetLastError();
				Check(HRESULT_FROM_WIN32(error), "Unexpected TRUE result from async ConnectNamedPipe");
				return EState::Broken;
			}

			// Check for errors
			auto error = GetLastError();
			switch (error)
			{
				case ERROR_PIPE_CONNECTED:
				{
					// Pipe has connected immediately.
					m_log.Write(log::ELevel::Info, "Connect completed immediate");
					return EState::Connected;
				}
				case ERROR_PIPE_LISTENING:
				case ERROR_IO_PENDING:
				{
					// Pipe is ready, waiting for the client to connect or a connection is in progress.
					cleaner.m_retain = true;
					m_log.Write(log::ELevel::Info, "Connecting in progress");
					return EState::ConnectPending;
				}
				case ERROR_NO_DATA:
				{
					// The previous client has closed the pipe handle, but the server has not yet disconnected.
					m_log.Write(log::ELevel::Info, "Client closed pipe");
					return EState::Broken;
				}
				case ERROR_INVALID_HANDLE:
				{
					// There's something wrong with the pipe handle, create a new one.
					CreatePipe();
					return EState::Disconnected;
				}
				default:
				{
					Check(HRESULT_FROM_WIN32(error), "ConnectNamedPipe failed");
					return EState::Broken;
				}
			}
		}

		// Disconnect from the IPC pipe
		void Disconnect()
		{
			if (m_pipe == nullptr)
				return;

			// Mark existing IO operations as cancel-pending.
			if (!CancelIo(m_pipe.get()))
			{
				// If this fails, the documentation doesn't say what to do with in-flight IO operations.
				auto error = GetLastError();
				Check(HRESULT_FROM_WIN32(error), "CancelIo failed");
				return;
			}

			// Process any remaining queued IO operations
			ProcessIO(std::chrono::milliseconds{ 0 }, EState::Disconnected);

			// Disconnect the pipe
			if (m_mode == EMode::Server && !DisconnectNamedPipe(m_pipe.get()))
			{
				auto error = GetLastError();
				if (error != ERROR_PIPE_NOT_CONNECTED)
				{
					Check(HRESULT_FROM_WIN32(error), "DisconnectNamedPipe failed");
					return;
				}
			}
			m_log.Write(log::ELevel::Info, "Pipe diconnected");
					
			// Release any leftover overlapped objects. Ideally there shouldn't be any
			// but that can't be guaranteed depending on how the pipe connection is lost.
			ReturnAll();

			m_pipe = nullptr;
			m_iocp = nullptr;
		}

		// Pump the queue of completed async IO operations
		EState ProcessIO(std::chrono::milliseconds wait_time, EState current_state)
		{
			// Process completed IO operations
			for (;;)
			{
				ULONG_PTR key = 0;
				DWORD bytes_transferred = 0;
				OVERLAPPED* completion_overlapped = nullptr;
				bool more_data = false;

				// Wait up to 'wait_time_ms' for an IO completion event
				if (!GetQueuedCompletionStatus(m_iocp.get(), &bytes_transferred, &key, &completion_overlapped, static_cast<DWORD>(wait_time.count())))
				{
					auto error = GetLastError();
					switch (error)
					{
						case WAIT_TIMEOUT: // No more queued IO operations
						{
							return current_state;
						}
						case ERROR_MORE_DATA: // Incomplete read
						{
							more_data = true;
							break;
						}
						case ERROR_BROKEN_PIPE:
						case ERROR_OPERATION_ABORTED:
						case ERROR_PIPE_NOT_CONNECTED:
						case ERROR_ABANDONED_WAIT_0:
						case ERROR_INVALID_HANDLE:
						{
							// The pipe has been disconnected
							OverlappedReturner cleaner(*static_cast<Overlapped*>(completion_overlapped), *this);
							return EState::Broken;
						}
						default:
						{
							assert(completion_overlapped == nullptr);
							Check(HRESULT_FROM_WIN32(error), "GetQueuedCompletionStatus failed");
							return EState::Broken;
						}
					}
				}

				// The completed IO operation does not have an associated overlapped object, on to the next...
				if (completion_overlapped == nullptr)
					continue;

				// The completed overlapped IO operation data
				auto& overlapped = *static_cast<Overlapped*>(completion_overlapped);
				OverlappedReturner cleaner(overlapped, *this);

				// Process the completed IO operation. Remove the overlapped object from the collection of pending IO operations.
				switch (overlapped.m_op)
				{
					case EAsyncOp::Connect:
					{
						// Connection completed
						current_state = EState::Connected;
						m_log.Write(log::ELevel::Info, "Connect completed");
						break;
					}
					case EAsyncOp::Send:
					{
						// A Send completed
						m_log.Write(log::ELevel::Info, std::format("Send completed: {}", Summary(overlapped.data())));
						break;
					}
					case EAsyncOp::Read:
					{
						HandleReadComplete(overlapped, bytes_transferred, more_data);
						cleaner.m_retain = more_data;
						break;
					}
					case EAsyncOp::Reconnect:
					{
						// This message signals that the connection should be dropped and re-connected
						return EState::Broken;
					}
					case EAsyncOp::Shutdown:
					{
						// A terminate request was sent
						m_log.Write(log::ELevel::Info, "Shutdown received");
						return EState::Shutdown;
					}
					case EAsyncOp::None:
					{
						throw std::runtime_error("Overlapped operation completed using a freed overlapped object");
					}
					default:
					{
						throw std::runtime_error(std::format("Unknown state share async operation : {}", static_cast<int>(overlapped.m_op)));
					}
				}
			}
		}

		// Handle the result
		void HandleReadComplete(Overlapped& overlapped, int64_t bytes_transferred, bool more_data)
		{
			// A Read has completed, but maybe only partially.
			overlapped.m_len += bytes_transferred;

			if (more_data)
			{
				m_log.Write(log::ELevel::Info, std::format("Read partial: {}", Summary(overlapped.data())));
	
				// If the message isn't complete, read again. Retain the overlapped object.
				overlapped.grow();
				QueueRead(overlapped);
			}
			else
			{
				m_log.Write(log::ELevel::Info, std::format("Read complete: {}", Summary(overlapped.data())));

				// Otherwise, the full message has been received. Pass the message to observers and start a new read
				overlapped.shrink();
				MessageReceived(overlapped.data());
				QueueRead();
			}
		}

		// Begin an async read on the pipe
		void QueueRead()
		{
			QueueRead(GetOverlapped(EAsyncOp::Read));
		}

		// Continue an async read on the pipe
		void QueueRead(Overlapped& overlapped)
		{
			OverlappedReturner cleaner(overlapped, *this);

			// Read into 'overlapped.data'
			DWORD bytes_read = 0;
			auto buf = overlapped.buf().subspan(static_cast<size_t>(overlapped.m_len));
			auto r = ReadFile(m_pipe.get(), buf.data(), s_cast<DWORD>(buf.size()), &bytes_read, &overlapped);
			auto error = r ? ERROR_SUCCESS : GetLastError();
			switch (error)
			{
				case ERROR_SUCCESS: // Read was completed
				{
					HandleReadComplete(overlapped, bytes_read, false);
					return;
				}
				case ERROR_IO_PENDING: // Read was started
				{
					// Read started
					cleaner.m_retain = true;
					return;
				}
				case ERROR_BROKEN_PIPE:
				case ERROR_PIPE_NOT_CONNECTED:
				{
					// Queue a message to reconnect
					QueueSignal(EAsyncOp::Reconnect);
					return;
				}
				default:
				{
					Check(HRESULT_FROM_WIN32(error), "ReadFile failed");
					return;
				}
			}
		}

		// Queue a signal to the worker thread
		void QueueSignal(EAsyncOp op)
		{
			auto& overlapped = GetOverlapped(op);
			OverlappedReturner cleaner(overlapped, *this, true);

			if (!PostQueuedCompletionStatus(m_iocp.get(), 0, 0, &overlapped))
			{
				cleaner.m_retain = false;
				auto error = GetLastError();
				Check(HRESULT_FROM_WIN32(error), "PostQueuedCompletionStatus failed");
			}
		}

		// Save a message to be sent once a connection is established
		void SaveMessage(std::span<uint8_t const> data)
		{
			std::scoped_lock<std::mutex> lock(m_mutex_saved_messages);
			m_saved_messages.push_back(std::vector<uint8_t>{data.begin(), data.end()});
		}

		// Send any messages that were saved to be sent once we connect
		void SendSavedMessages()
		{
			std::vector<std::vector<uint8_t>> messages;
			{
				std::scoped_lock<std::mutex> lock(m_mutex_saved_messages);
				swap(messages, m_saved_messages);
				if (messages.empty())
					return;
			}

			m_log.Write(log::ELevel::Debug, "Sending saved messages...");
			for (auto const& message : messages)
			{
				Write(message);
			}
			m_log.Write(log::ELevel::Debug, "Sending saved messages...done");
		}
	
		// Summaries 'data'
		std::string Summary(std::span<uint8_t const> data)
		{
			return std::string(reinterpret_cast<char const*>(data.data()), std::min(50, isize(data)));
		}
	};
}

#if PR_UNITTESTS && 0 // nearly working!
#include "pr/common/unittests.h"
#include "pr/threads/name_thread.h"
#include <condition_variable>
namespace pr::pipe
{
	PRUnitTest(PipeSimpleTest)
	{
		// Start a server and a client.
		// Send one message from the client to the server
		char const UnitTestPipeName[] = "\\\\.\\pipe\\Pipe_UnitTest";
		int ServerMsgsSent = 0;
		int ServerMsgsRecv = 0;
		int ClientMsgsSent = 0;
		int ClientMsgsRecv = 0;

		log::Logger log("", log::ToOutputDebugString{}, log::EMode::Immediate);

		// Event for signalling message received
		std::condition_variable cv_signal;
		std::stop_source shutdown;
		std::mutex mutex;

		// Create the pipe server and client
		Pipe ipc_server(EMode::Server, UnitTestPipeName, {}, log);
		Pipe ipc_client(EMode::Client, UnitTestPipeName, {}, log);

		// Attach message handlers
		ipc_server.MessageReceived += [&ServerMsgsRecv, &cv_signal](std::span<uint8_t const> data)
		{
			std::string_view msg(reinterpret_cast<char const*>(data.data()), data.size());
			PR_EXPECT(msg == "Message To Server");

			++ServerMsgsRecv;
			cv_signal.notify_one();
		};
		ipc_client.MessageReceived += [&ClientMsgsRecv, &cv_signal](std::span<uint8_t const> data)
		{
			std::string_view msg(reinterpret_cast<char const*>(data.data()), data.size());
			PR_EXPECT(msg == "Message To Client");

			++ClientMsgsRecv;
			cv_signal.notify_one();
		};

		// Start the server and client
		std::thread ipc_server_thread([&ipc_server, &shutdown]
			{
				threads::SetCurrentThreadName("IPC Server");
				ipc_server.Run(shutdown.get_token());
			});
		std::thread ipc_client_thread([&ipc_client, &shutdown]
			{
				threads::SetCurrentThreadName("IPC Client");
				ipc_client.Run(shutdown.get_token());
			});

		// One message from server to client
		ipc_server.Write("Message To Client");
		++ServerMsgsSent;

		// Wait till received
		{
			std::unique_lock<std::mutex> lock(mutex);
			cv_signal.wait(lock, [&ClientMsgsRecv, &ServerMsgsSent] { return ClientMsgsRecv == ServerMsgsSent; });
		}

		// One message from client to server
		ipc_client.Write("Message To Server");
		++ClientMsgsSent;

		// Wait till received
		{
			std::unique_lock<std::mutex> lock(mutex);
			cv_signal.wait(lock, [&ServerMsgsRecv, &ClientMsgsSent] { return ServerMsgsRecv == ClientMsgsSent; });
		}

		PR_EXPECT(ClientMsgsRecv == ClientMsgsSent);
		PR_EXPECT(ServerMsgsRecv == ServerMsgsSent);

		// Shutdown
		shutdown.request_stop();
		ipc_server_thread.join();
		ipc_client_thread.join();
	}
	PRUnitTest(PipeTest)
	{
		char const UnitTestPipeName[] = "\\\\.\\pipe\\Pipe_UnitTest";
		int ServerMsgsSent = 0;
		int ServerMsgsRecv = 0;
		int ClientMsgsSent = 0;
		int ClientMsgsRecv = 0;

		log::Logger log("", log::ToOutputDebugString{}, log::EMode::Immediate);


		// Event for signalling message received
		std::condition_variable cv_signal;
		std::stop_source shutdown;
		std::mutex mutex;

		// Create the pipe server and client
		Pipe ipc_server(EMode::Server, UnitTestPipeName, {}, log);
		Pipe ipc_client(EMode::Client, UnitTestPipeName, {}, log);

		// Attach message handlers
		ipc_server.MessageReceived += [&ServerMsgsRecv, &cv_signal](std::span<uint8_t const> data)
		{
			std::string_view msg(reinterpret_cast<char const*>(data.data()), data.size());
			PR_EXPECT(msg == "Message To Server");

			++ServerMsgsRecv;
			cv_signal.notify_one();
		};
		ipc_client.MessageReceived += [&ClientMsgsRecv, &cv_signal](std::span<uint8_t const> data)
		{
			std::string_view msg(reinterpret_cast<char const*>(data.data()), data.size());
			PR_EXPECT(msg == "Message To Client");

			++ClientMsgsRecv;
			cv_signal.notify_one();
		};

		// Send messages before starting the client or server (these should get buffered and sent on connection)
		ipc_server.Write("Message To Client");
		ipc_client.Write("Message To Server");
		++ServerMsgsSent;
		++ClientMsgsSent;

		// Start the server and client
		std::thread ipc_server_thread([&ipc_server, &shutdown]
			{
				threads::SetCurrentThreadName("IPC Server");
				ipc_server.Run(shutdown.get_token());
			});
		std::thread ipc_client_thread([&ipc_client, &shutdown]
			{
				threads::SetCurrentThreadName("IPC Client");
				ipc_client.Run(shutdown.get_token());
			});

		// Send more messages, potentially while the connection is still being established
		for (int i = 0; i != 10; ++i)
		{
			ipc_server.Write("Message To Client");
			std::this_thread::yield();
			++ClientMsgsSent;
			ipc_client.Write("Message To Server");
			std::this_thread::yield();
			++ServerMsgsSent;
		}

		// Send a message from server to client and confirm all messages received
		{
			std::unique_lock<std::mutex> lock(mutex);
			ipc_server.Write("Message To Client");
			++ClientMsgsSent;

			cv_signal.wait(lock, [&ClientMsgsRecv, ClientMsgsSent] { return ClientMsgsRecv == ClientMsgsSent; });
		}

		// Send a message from client to server and confirm all messages received
		{
			std::unique_lock<std::mutex> lock(mutex);
			ipc_client.Write("Message To Server");
			++ServerMsgsSent;

			cv_signal.wait(lock, [&ServerMsgsRecv, ServerMsgsSent] { return ServerMsgsRecv == ServerMsgsSent; });
		}
		PR_EXPECT(ClientMsgsRecv == ClientMsgsSent);
		PR_EXPECT(ServerMsgsRecv == ServerMsgsSent);

		// Shutdown
		shutdown.request_stop();
		ipc_server_thread.join();
		ipc_client_thread.join();
	}
	PRUnitTest(PipeTest_ClientOnly)
	{
		// Create a client with no server
		char const UnitTestPipeName[] = "\\\\.\\pipe\\Pipe_UnitTest";
		int ClientMsgsRecv = 0;

		log::Logger log("", log::ToOutputDebugString{}, log::EMode::Immediate);

		// Event for signalling message received
		std::condition_variable cv_signal;
		std::stop_source shutdown;
		std::mutex mutex;

		// Create the pipe client
		Pipe ipc_client(EMode::Client, UnitTestPipeName, {}, log);

		// Attach message handlers
		ipc_client.MessageReceived += [&ClientMsgsRecv, &cv_signal](std::span<uint8_t const> data)
		{
			std::string_view msg(reinterpret_cast<char const*>(data.data()), data.size());
			PR_EXPECT(msg == "Message To Client");

			++ClientMsgsRecv;
			cv_signal.notify_one();
		};

		// Send messages before starting (these will be buffered and never sent because we're not starting a server)
		ipc_client.Write("Message To Server");

		// Start client
		std::thread ipc_client_thread([&ipc_client, &shutdown]
		{
			threads::SetCurrentThreadName("IPC Client");
			ipc_client.Run(shutdown.get_token());
		});

		// Send more messages (again, no server, they'll be buffered)
		for (int i = 0; i != 10; ++i)
		{
			ipc_client.Write("Message To Server");
			std::this_thread::yield();
		}
		PR_EXPECT(ClientMsgsRecv == 0);

		// Shutdown
		shutdown.request_stop();
		ipc_client_thread.join();
	}
	PRUnitTest(PipeTest_ServerOnly)
	{
		char const UnitTestPipeName[] = "\\\\.\\pipe\\Pipe_UnitTest";
		int ServerMsgsRecv = 0;

		log::Logger log("", log::ToOutputDebugString{}, log::EMode::Immediate);

		// Event for signalling message received
		std::condition_variable cv_signal;
		std::stop_source shutdown;
		std::mutex mutex;

		// Create the pipe server and client
		Pipe ipc_server(EMode::Server, UnitTestPipeName, {}, log);

		// Attach message handlers
		ipc_server.MessageReceived += [&ServerMsgsRecv, &cv_signal](std::span<uint8_t const> data)
		{
			std::string_view msg(reinterpret_cast<char const*>(data.data()), data.size());
			PR_EXPECT(msg == "Message To Server");

			++ServerMsgsRecv;
			cv_signal.notify_one();
		};

		// Send messages before starting (these will be buffered and never sent because we're not starting a client)
		ipc_server.Write("Message To Client");

		// Start the server
		std::thread ipc_server_thread([&ipc_server, &shutdown]
		{
			threads::SetCurrentThreadName("IPC Server");
			ipc_server.Run(shutdown.get_token());
		});
	
		// Send more messages (again, no client, they'll be buffered)
		for (int i = 0; i != 10; ++i)
		{
			ipc_server.Write("Message To Client");
			std::this_thread::yield();
		}

		PR_EXPECT(ServerMsgsRecv == 0);

		// Shutdown
		shutdown.request_stop();
		ipc_server_thread.join();
	}
}
#endif