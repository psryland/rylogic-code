//**********************************************************
// Process
//  Copyright (c) Rylogic Ltd 2011
//**********************************************************
// Usage:
//  pr::Process proc;
//  proc.Start("c:\program files\program_name.exe", "-args");
//  <do stuff>
//  int exit_code = proc.BlockTillExit();
//  <or>
//  int exit_code = proc.Stop();

#pragma once

#include <string>
#include <cassert>
#include <windows.h>
#include <io.h>
#include "pr/common/hresult.h"
#include "pr/str/string_util.h"

namespace pr
{
	class Process
	{
		struct AttributeList
		{
			std::unique_ptr<BYTE[]> m_list;
			explicit AttributeList(int attribute_count)
				:m_list()
			{
				SIZE_T size;
				InitializeProcThreadAttributeList(nullptr, attribute_count, 0, &size);
				m_list = std::unique_ptr<BYTE[]>(new BYTE[size]);
				Check(InitializeProcThreadAttributeList(*this, attribute_count, 0, &size));
			}
			~AttributeList()
			{
				DeleteProcThreadAttributeList(*this);
			}
			void AddHandleList(std::vector<HANDLE> const& handles)
			{
				Check(UpdateProcThreadAttribute(*this, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST, (void*)handles.data(), sizeof(HANDLE)*handles.size(), nullptr, nullptr));
			}
			operator LPPROC_THREAD_ATTRIBUTE_LIST() const
			{
				return reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(m_list.get());
			}
		};

	public:

		PROCESS_INFORMATION ProcessInfo;
		STARTUPINFOEXW StartupInfo;
		SECURITY_ATTRIBUTES SecurityAttributes;

		Process()
			:ProcessInfo()
			,StartupInfo(STARTUPINFOEXW{sizeof(STARTUPINFOEXW)})
			,SecurityAttributes(SECURITY_ATTRIBUTES{sizeof(SECURITY_ATTRIBUTES)})
		{}
		~Process()
		{
			Stop();
		}

		// Start the process
		bool Start(wchar_t const* exe_path, wchar_t const* args, wchar_t const* startdir)
		{
			// Stop the process if currently running
			if (ProcessInfo.hProcess != nullptr)
				Stop();

			// Construct the new command line. 'args' is optional
			auto cmdline = str::Quotes<std::wstring>(exe_path, true);
			if (args != nullptr)
				cmdline.append(L" ").append(args);

			// Create the child process
			auto res = CreateProcessW(exe_path, &cmdline[0], &SecurityAttributes, nullptr, TRUE, EXTENDED_STARTUPINFO_PRESENT, nullptr, startdir, &StartupInfo.StartupInfo, &ProcessInfo);
			return Succeeded(res);
		}
		bool Start(wchar_t const* exe_path, wchar_t const* args)
		{
			return Start(exe_path, args, nullptr);
		}
		bool Start(wchar_t const* exe_path)
		{
			return Start(exe_path, nullptr);
		}

		// Shutdown the process
		int Stop()
		{
			int exit_code = -1;
			if (ProcessInfo.hProcess != nullptr)
			{
				PostThreadMessageW(ProcessInfo.dwThreadId, WM_QUIT, 0, 0);
				exit_code = BlockTillExit();
			}
			
			if (ProcessInfo.hProcess != nullptr)
			{
				CloseHandle(ProcessInfo.hProcess);
				ProcessInfo.hProcess = nullptr;
			}
			if (ProcessInfo.hThread != nullptr)
			{
				CloseHandle(ProcessInfo.hThread);
				ProcessInfo.hThread = nullptr;
			}

			return exit_code;
		}

		// Block the calling thread until the child process exits
		int BlockTillExit()
		{
			if (ProcessInfo.hProcess == nullptr)
				throw std::runtime_error("Process not running");

			WaitForSingleObject(ProcessInfo.hProcess, INFINITE);
			
			DWORD exit_code;
			Check(GetExitCodeProcess(ProcessInfo.hProcess, &exit_code), "Process exit code not available");
			return static_cast<int>(exit_code);
		}

		// Returns true if the process is running
		bool IsActive() const
		{
			assert((ProcessInfo.hProcess == nullptr) == (ProcessInfo.hThread == nullptr));
			return ProcessInfo.hProcess != nullptr;
		}
	};
}


// Broken attempt to create unbuffered pipes
//// Create requested pipes
//std::vector<HANDLE> handles;
//if (flags.m_stdin_pipe)
//{
//	Throw(CreatePipe(&m_stdin.m_read.m_handle, &m_stdin.m_write.m_handle, &sa, flags.m_stdin_pipe_bufsize));
//	handles.push_back(m_stdin.m_read);
//}
//if (flags.m_stdout_pipe)
//{
//	Throw(CreatePipe(&m_stdout.m_read.m_handle, &m_stdout.m_write.m_handle, &sa, flags.m_stdout_pipe_bufsize));
//	handles.push_back(m_stdout.m_write);
//}
//if (flags.m_stderr_pipe)
//{
//	Throw(CreatePipe(&m_stderr.m_read.m_handle, &m_stderr.m_write.m_handle, &sa, flags.m_stderr_pipe_bufsize));
//	handles.push_back(m_stderr.m_write);
//}

//// Pass the pipe handles to the child process
//ByteData<> buffer;
//AttributeList attr_list(1);
//if (!handles.empty())
//{
//	attr_list.AddHandleList(handles);

//	BYTE const FOPEN = 0x01;
//	BYTE const FDEV = 0x40;

//	// Construct a byte buffer to pass in the startup info.
//	buffer.push_back(int(handles.size())); // cfi_len, number of handles
//	if (flags.m_stdin_pipe)  buffer.push_back(FOPEN | FDEV); // file info
//	if (flags.m_stdout_pipe) buffer.push_back(FOPEN | FDEV); // file info
//	if (flags.m_stderr_pipe) buffer.push_back(FOPEN | FDEV); // file info
//	if (flags.m_stdin_pipe)  buffer.push_back<void*>(m_stdin.m_read.m_handle); // os handle
//	if (flags.m_stdout_pipe) buffer.push_back<void*>(m_stdout.m_write.m_handle); // os handle
//	if (flags.m_stderr_pipe) buffer.push_back<void*>(m_stderr.m_write.m_handle); // os handle

//	si.lpAttributeList = attr_list;
//	si.StartupInfo.cbReserved2 = static_cast<WORD>(buffer.size());
//	si.StartupInfo.lpReserved2 = buffer.data();
//}
