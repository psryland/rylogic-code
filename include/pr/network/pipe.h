//**********************************
// Pipe
//  Copyright (C) Rylogic Ltd 2007
//**********************************
#pragma once
#include <vector>
#include <string>
#include <string_view>
#include <span>
#include <chrono>
#include <stdexcept>
#include <cassert>
#include <format>
#include <condition_variable>
#include <algorithm>
#include <type_traits>
#include <concepts>
#include <windows.h>
#include <stop_token>

#include "pr/common/event_handler.h"
#include "pr/common/hresult.h"

namespace pr
{
	// Named Pipe IO
	class Pipe
	{
		struct handle_closer { void operator()(HANDLE h) { if (h) CloseHandle(h); } };
		using ScopedHandle = std::unique_ptr<void, handle_closer>;
	
		ScopedHandle SafeHandle(HANDLE h) { return ScopedHandle((h != INVALID_HANDLE_VALUE) ? h : nullptr); }

		std::string   m_pipe_name;       // The unique name for the pipe
		ScopedHandle  m_pipe;            // The handle to the pipe for outgoing data
		ScopedHandle  m_evt_read;        // Manual reset event for overlapped read operations
		ScopedHandle  m_evt_write;       // Manual reset event for overlapped write operations
		mutable DWORD m_last_error;      // The last error returned from a call to ::GetLastError()

	public:

		struct Options
		{
			int BufferSize{ 4096 };
			bool Overlapped{ true };
			std::chrono::milliseconds ConnectTimeout { 10 };
			std::chrono::milliseconds ReadTimeout{ 10 };
			std::chrono::milliseconds WriteTimeout{ 0 };
			std::chrono::milliseconds WaitForServerAvailabilityTimeout{ 5000 };
			std::function<void(std::exception const&, DWORD)> OnPipeError{};
		} m_options;

		static std::string MakeName(std::string_view name)
		{
			if (name.starts_with("\\\\.\\pipe\\")) return std::string(name);
			return std::string("\\\\.\\pipe\\").append(name);
		}

		explicit Pipe(std::string_view pipe_name, Options const& options = {})
			: m_pipe_name(MakeName(pipe_name))
			, m_pipe()
			, m_evt_read(CreateEventA(nullptr, true, false, nullptr))
			, m_evt_write(CreateEventA(nullptr, true, false, nullptr))
			, m_last_error()
			, m_options(options)
			, MessageReceived()
		{
			// Don't connect on construction, callers may want to connect in a different thread
		}
		~Pipe()
		{
			Disconnect();
		}

		// The global name of the pipe
		std::string_view GetPipeName() const
		{
			return m_pipe_name;
		}

		// Take over the calling thread and run the pipe until shutdown is signalled
		void Run(std::stop_token shutdown, std::condition_variable& cv_notify) // Worker thread context
		{
			enum class EState { Disconnected, Connected, } state = EState::Disconnected;
			std::vector<uint8_t> ibuf(m_options.BufferSize);

			for (; !shutdown.stop_requested();)
			{
				try
				{
					switch (state)
					{
						case EState::Disconnected:
						{
							Connect();
							state = EState::Connected;
							cv_notify.notify_all();
							break;
						}
						case EState::Connected:
						{
							size_t bytes_read;
							if (Read(ibuf, bytes_read) && bytes_read != 0)
							{
								MessageReceived(std::span{ ibuf.data(), bytes_read });
							}
							else
							{
								state = EState::Disconnected;
								cv_notify.notify_all();
							}
							break;
						}
						default:
						{
							throw std::runtime_error("Unknown state");
						}
					}
				}
				catch (std::exception const& ex)
				{
					m_last_error = GetLastError();
					if (m_options.OnPipeError)
						m_options.OnPipeError(ex, m_last_error);

					state = EState::Disconnected;
					cv_notify.notify_all();
				}
			}

			// Leave in a state ready for Run to be called again
			Disconnect();
			cv_notify.notify_all();
		}
		
		// Message received event
		pr::MultiCast<std::function<void(std::span<uint8_t const> data)>, true> MessageReceived;

		// True if the pipe is connected and someone is listening
		bool IsConnected() const
		{
			return m_pipe != nullptr;
		}

		// Attempt to connect to someone listening
		void Connect()
		{
			// A problem with this dual server/client connect is there's a race condition if the two processes
			// start at the same time.
			Disconnect();

			// Try to connect as a client
			for (;;)
			{
				// Try to open the named pipe (as a client assuming the server exists)
				m_pipe = SafeHandle(CreateFileA(
					m_pipe_name.c_str(),
					GENERIC_WRITE | GENERIC_READ,
					FILE_SHARE_READ | FILE_SHARE_WRITE,
					nullptr,
					OPEN_EXISTING,
					m_options.Overlapped ? FILE_FLAG_OVERLAPPED : FILE_ATTRIBUTE_NORMAL,
					nullptr));

				// The pipe will exist if there is a server listening.
				// If this fails, we'll try and be the server and create the pipe
				if (m_pipe != nullptr)
				{
					DWORD mode = PIPE_READMODE_MESSAGE;
					if (!SetNamedPipeHandleState(m_pipe.get(), &mode, nullptr, nullptr))
					{
						m_last_error = GetLastError();
						Check(HRESULT_FROM_WIN32(m_last_error), "SetNamedPipeHandleState failed");
						return;
					}
					return;
				}

				m_last_error = GetLastError();

				// If the pipe doesn't exist, try to be the server and create it
				if (m_last_error == ERROR_FILE_NOT_FOUND)
					break;

				// Server is up but busy, wait for availability
				if (m_last_error == ERROR_PIPE_BUSY)
				{
					if (!WaitNamedPipeA(m_pipe_name.c_str(), static_cast<DWORD>(m_options.WaitForServerAvailabilityTimeout.count())))
					{
						m_last_error = GetLastError();
						Check(HRESULT_FROM_WIN32(m_last_error), "WaitNamedPipe failed");
						return;
					}
				}

				Check(HRESULT_FROM_WIN32(m_last_error), "Server found but failed to connect as a client");
				return;
			}

			// Try to be the server
			for (;;)
			{
				// Create the named pipe
				m_pipe = SafeHandle(CreateNamedPipeA(
					m_pipe_name.c_str(),
					PIPE_ACCESS_DUPLEX	| FILE_FLAG_FIRST_PIPE_INSTANCE | (m_options.Overlapped ? FILE_FLAG_OVERLAPPED : 0),
					PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
					PIPE_UNLIMITED_INSTANCES,
					DWORD(m_options.BufferSize),
					DWORD(m_options.BufferSize),
					NMPWAIT_USE_DEFAULT_WAIT,
					nullptr));

				// The pipe couldn't be created
				if (m_pipe == nullptr)
				{
					m_last_error = GetLastError();
					Check(HRESULT_FROM_WIN32(m_last_error), "Failed to create named pipe (as server)");
					return;
				}

				// Wait for a client connection to the pipe
				auto ovrlap = OVERLAPPED{ .hEvent = m_evt_read.get() };
				if (!ConnectNamedPipe(m_pipe.get(), m_options.Overlapped ? &ovrlap : nullptr))
				{
					m_last_error = GetLastError();

					// The client is connected
					if (m_last_error == ERROR_PIPE_CONNECTED)
						return;
			
					// The connection is in progress, wait for it to complete
					if (m_last_error == ERROR_IO_PENDING)
					{
						auto r = ::WaitForSingleObject(ovrlap.hEvent, INFINITE);
						switch (r)
						{
							case WAIT_OBJECT_0:
							{
								DWORD xferd;
								if (::GetOverlappedResult(m_pipe.get(), &ovrlap, &xferd, FALSE) != TRUE)
								{
									m_last_error = GetLastError();
									Check(HRESULT_FROM_WIN32(m_last_error), "GetOverlappedResult failed waiting for client to complete connection");
									return;
								}
								return;
							}
							case WAIT_TIMEOUT:
							{
								::CancelIo(m_pipe.get());
								return;
							}
							case WAIT_ABANDONED:
							{
								return;
							}
						}
					}

					// Some other problem
					Check(HRESULT_FROM_WIN32(m_last_error), "ConnectNamedPipe connect failed");
					return;
				}

				// Client connected
				return;
			}
		}

		// Close the connection
		void Disconnect()
		{
			 m_pipe = nullptr;
		}

		// Send data on the pipe
		// Returns true if some data was sent.
		// Note: timeouts are only supported for overlapped IO
		bool Write(std::span<uint8_t const> data, size_t& bytes_sent)
		{
			bytes_sent = 0;
			if (!IsConnected())
				throw std::runtime_error("Pipe is not connected");


			// Sanity check
			if (data.size() > DWORD(~0))
				throw std::runtime_error("Too much data to send on named pipe");

			DWORD sent;
			bool res;

			// Write the data to the pipe
			if (!m_options.Overlapped)
			{
				res = ::WriteFile(m_pipe.get(), data.data(), DWORD(data.size()), &sent, nullptr) != 0;
			}
			else
			{
				// Write the data and wait for the overlapped operation to complete
				auto ovrlap = OVERLAPPED{ .hEvent = m_evt_write.get() };
				if (!::WriteFile(m_pipe.get(), data.data(), DWORD(data.size()), nullptr, &ovrlap) &&
					(m_last_error = ::GetLastError()) != ERROR_IO_PENDING)
					return false;

				// Wait for the write to complete
				auto r = ::WaitForSingleObject(ovrlap.hEvent, static_cast<DWORD>(m_options.WriteTimeout.count()));
				switch (r)
				{
					case WAIT_OBJECT_0:
					{
						res = ::GetOverlappedResult(m_pipe.get(), &ovrlap, &sent, FALSE) != 0;
						break;
					}
					case WAIT_TIMEOUT:
					{
						::CancelIo(m_pipe.get());
						return false;
					}
					case WAIT_ABANDONED:
					{
						return false;
					}
					default:
					{
						Check(HRESULT_FROM_WIN32(m_last_error), "Named pipe Write command failed");
						return false;
					}
				}
			}

			// Condition here to allow break points when 'sent' != 0
			if (sent != 0)
				bytes_sent = sent;

			return res;
		}

		// Write all of 'data' to the i/o connection
		bool Write(std::span<uint8_t const> data)
		{
			for (size_t writ, bytes_writ = 0; bytes_writ != data.size(); bytes_writ += writ)
			{
				if (!Write(data.subspan(bytes_writ, data.size() - bytes_writ), writ))
					return false;
			}
			return true;
		}
		bool Write(std::string_view message)
		{
			return Write(std::span<uint8_t const>(reinterpret_cast<uint8_t const*>(message.data()), message.size()));
		}

		// Read data from the IO connection.
		// Returns true if data was read, false if the timeout was reached
		// Note: timeouts are only supported for overlapped IO
		bool Read(std::span<uint8_t> buffer, size_t& bytes_read)
		{
			bytes_read = 0;
			if (!IsConnected())
				throw std::runtime_error("Pipe is not connected");

			// Sanity check
			if (buffer.size() > DWORD(~0))
				throw std::runtime_error("Too much data to send on named pipe");

			DWORD read;
			bool res;

			if (!m_options.Overlapped)
			{
				// Sync read
				res = ::ReadFile(m_pipe.get(), buffer.data(), static_cast<DWORD>(buffer.size()), &read, nullptr) != 0;
			}
			else
			{
				// Read data and wait for the overlapped operation to complete
				auto ovrlap = OVERLAPPED{ .hEvent = m_evt_read.get() };
				if (!::ReadFile(m_pipe.get(), buffer.data(), static_cast<DWORD>(buffer.size()), 0, &ovrlap) &&
					(m_last_error = GetLastError()) != ERROR_IO_PENDING)
					return false;

				// Wait for the read to complete
				auto r = ::WaitForSingleObject(ovrlap.hEvent, static_cast<DWORD>(m_options.ReadTimeout.count()));
				switch (r)
				{
				case WAIT_OBJECT_0:
					res = ::GetOverlappedResult(m_pipe.get(), &ovrlap, &read, FALSE) != 0;
					break;
				case WAIT_TIMEOUT:
					::CancelIo(m_pipe.get());
					return false;
				case WAIT_ABANDONED:
					return false;
				default:
					Check(HRESULT_FROM_WIN32(LastError()), "Named pipe Read failed");
					throw std::runtime_error(std::format("Unknown return code (0x{:08X}) during Serial port Read command", r));
				}
			}

			// Condition here to allow break points when 'read' != 0
			if (read != 0)
				bytes_read = read;

			return res;
		}

		// Read all of 'size' into buffer or timeout
		bool Read(std::span<uint8_t> buffer)
		{
			for (size_t read, bytes_read = 0; bytes_read != buffer.size(); bytes_read += read)
			{
				if (!Read(buffer.subspan(bytes_read, buffer.size() - bytes_read), read))
					return false;
			}
			return true;
		}

		// Read a POD object from the i/o connection. (Careful with padding!)
		template <typename Type> requires std::is_standard_layout_v<Type>
		bool Read(Type& type)
		{
			return Read({ &type, sizeof(type) });
		}

		// Flush any buffered data
		void Flush()
		{
			auto res = ::FlushFileBuffers(m_pipe.get());
			if (res == 0)
			{
				m_last_error = GetLastError();
				if (m_last_error == ERROR_NOT_SUPPORTED) return;
				if (m_last_error == ERROR_INVALID_FUNCTION) return;
				Check(res, "Failed to flush write buffer");
			}
		}

		// Look at, but don't remove, data from the pipe
		bool Peek(std::span<uint8_t> buffer, size_t& bytes_read, size_t& available, size_t& message_bytes_left)
		{
			bytes_read = 0;
			available = 0;
			message_bytes_left = 0;
			if (!IsConnected())
			{
				assert(!"Pipe not connected");
				return false;
			}

			// Sanity check
			if (buffer.size() > DWORD(~0))
				throw std::runtime_error("Too much data to send on named pipe");

			DWORD read, avail, msg_left;
			if (!::PeekNamedPipe(m_pipe.get(), buffer.data(), static_cast<DWORD>(buffer.size()), &read, &avail, &msg_left))
			{
				m_last_error = GetLastError();
				return false;
			}

			// Condition here to allow break points when 'read' != 0
			if (read != 0)
			{
				bytes_read = read;
				available = avail;
				message_bytes_left = msg_left;
			}
			return true;
		}

		// GetLastError
		DWORD LastError() const
		{
			return m_last_error;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/threads/name_thread.h"
#include <condition_variable>
#include <thread>
namespace pr::network
{
	PRUnitTest(PipeSimpleTest)
	{
		// Start a server and a client.
		// Send one message from the client to the server
		char const UnitTestPipeName[] = "Pipe_UnitTest";
		int ServerMsgsSent = 0;
		int ServerMsgsRecv = 0;
		int ClientMsgsSent = 0;
		int ClientMsgsRecv = 0;

		// Event for signalling message received
		std::condition_variable cv_signal;
		std::stop_source shutdown;
		std::mutex mutex;

		// Create the pipe server and client
		Pipe ipc_server(UnitTestPipeName, {});
		Pipe ipc_client(UnitTestPipeName, {});

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
		std::thread ipc_server_thread([&ipc_server, &shutdown, &cv_signal]
		{
			threads::SetCurrentThreadName("IPC Server");
			ipc_server.Run(shutdown.get_token(), cv_signal);
		});
		std::thread ipc_client_thread([&ipc_client, &shutdown, &cv_signal]
		{
			threads::SetCurrentThreadName("IPC Client");
			ipc_client.Run(shutdown.get_token(), cv_signal);
		});

		// Wait till the threads are running
		{
			std::unique_lock<std::mutex> lock(mutex);
			cv_signal.wait(lock, [&] { return ipc_server.IsConnected() && ipc_client.IsConnected(); });
		}

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
}
#endif