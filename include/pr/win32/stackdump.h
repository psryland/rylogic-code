//***********************************************************************
//  Stack Dump
//***********************************************************************
// Note: This implementation is based on an unreleased Boost library,
// which can be found here: http://groups.yahoo.com/group/boost/files/StackTraces/
// This implementation requires the dbghelp.dll.
// It should be placed somewhere where it can be found by the executable.
//
// Usage:
//  Use StackDump() to produce the addresses at the time StackDump() is called
//  Use GetCallSource() to convert addresses into file/line numbers
//  If you only get addresses printed out you need to make "DbgHelp.Dll" available for the exe
//   get the debugging tools from: http://www.microsoft.com/whdc/devtools/debugging/installx86.mspx
// e.g.
//   void SomeHelperTraceFunc()
//   {
//       // 1 = skip this function, and output the next 5 on the stack
//       pr::DumpStack([](std::string const& sym, std::string const& filepath, int line)
//       {
//          printf(..);
//       }, 1, 5);
//   }

#pragma once

#include <malloc.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <fstream>
#include <functional>
#include <filesystem>
#include <mutex>
#include <windows.h>

static_assert(_WIN32_WINNT >= _WIN32_WINNT_VISTA, "_WIN32_WINNT version 0x0600 or greater required");

// <dbghelp.h> has conflicts with <imagehlp.h>, so it cannot be included
// <imagehlp.h> contains everything we would need from <dbghelp.h> anyway
#pragma warning(push)
#pragma warning(disable:4091)
#include <imagehlp.h>
#include <tlhelp32.h>
#include <psapi.h>
#pragma warning(pop)

#pragma comment(lib, "dbghelp.lib")

namespace pr
{
	struct CallAddress
	{
		void* m_address;
	};
	struct CallSource
	{
		std::filesystem::path m_filepath;
		unsigned int m_line = 0;
		std::string m_sym_name;
	};

	namespace impl
	{
		// Win32/64 helper macros (defined by system header files or the compiler):
		//  _WIN64 64-bit
		//  _WIN32 64-bit or 32-bit
		//
		//  _M_IX86 Intel 32-bit
		//  _M_IA64 Intel 64-bit
		//  _M_AMD64 AMD 64-bit
		//
		// Occasionally, a reinterpret_cast is used in these functions.  These casts
		//  are safe; just keep in mind the following:
		//   void * - 32 bits on Win32, 64 bits on Win64
		//   DWORD64 - 64 bits on Win32, 64 bits on Win64
		//   DWORD - 32 bits on Win32, unspecified on Win64 (probably 32 bits)
		//   SIZE_T - 32 bits on Win32, and 64 bits on Win64 (unsigned)

		// Simple class that closes a handle on scope exit
		struct ScopedHandle
		{
			// Note: this isn't for a file, it's a handle return from one of the dbg dll functions
			HANDLE m_handle;

			ScopedHandle(HANDLE handle) :m_handle(handle) {}
			ScopedHandle(ScopedHandle&&) = delete;
			ScopedHandle(ScopedHandle const&) = delete;
			ScopedHandle& operator=(ScopedHandle&&) = delete;
			ScopedHandle& operator=(ScopedHandle const&) = delete;
			~ScopedHandle() { CloseHandle(m_handle); }
		};

		// A global mutex to sync access to 'GetCallSource()'
		class StackdumpLock
		{
			std::lock_guard<std::recursive_mutex> m_lock;
			static std::recursive_mutex& cs()
			{
				static std::recursive_mutex s_cs;
				return s_cs;
			}

		public:

			StackdumpLock()
				:m_lock(cs())
			{}
			StackdumpLock(StackdumpLock&&) = delete;
			StackdumpLock(StackdumpLock const&) = delete;
			StackdumpLock& operator=(StackdumpLock&&) = delete;
			StackdumpLock& operator=(StackdumpLock const&) = delete;
		};

		// Wraps and initialises a SYMBOL_INFO object
		struct SymInfo :SYMBOL_INFO
		{
			wchar_t sym_name[1024];
			SymInfo()
				:SYMBOL_INFO()
				,sym_name()
			{
				SizeOfStruct = sizeof(SYMBOL_INFO);
				MaxNameLen = _countof(sym_name);
			}
		};

		// Wraps and initialises a IMAGEHLP_LINE64 object
		struct Line64 :IMAGEHLP_LINE64
		{
			Line64()
				:IMAGEHLP_LINE64()
			{
				SizeOfStruct = sizeof(IMAGEHLP_LINE64);
			}
		};

		// DLL functions and classes **********************************************************************

		// This is a class that represents a DLL that has been Loaded into memory
		// Loads and unloads the named DLL into memory
		// If the DLL is not successfully Loaded, 'm_module' will be nullptr.
		struct DllBase
		{
			HMODULE m_module;

			// Only the file name is needed, not the full path
			explicit DllBase(wchar_t const* dll_filename)
				:m_module(LoadLibraryW(dll_filename))
			{}
			~DllBase()
			{
				if (m_module == nullptr) return;
				FreeLibrary(m_module);
			}

			// Returns the address of the specified procedure, or 0 if not found.
			void* ProcAddress(const char* proc_name)
			{
				return ::GetProcAddress(m_module, proc_name);
			}
		};

		// This class represents kernel32.dll and the tool help functions from it
		struct ToolHelp_Dll :DllBase
		{
			typedef HANDLE(__stdcall* CreateToolhelp32Snapshot_type)(DWORD , DWORD);
			typedef BOOL  (__stdcall* Module32First_type           )(HANDLE, tagMODULEENTRY32*);
			typedef BOOL  (__stdcall* Module32Next_type            )(HANDLE, tagMODULEENTRY32*);

			CreateToolhelp32Snapshot_type CreateToolhelp32Snapshot;
			Module32First_type            FirstModule32;
			Module32Next_type             NextModule32;

			ToolHelp_Dll()
				:DllBase(L"kernel32.dll")
				,CreateToolhelp32Snapshot((CreateToolhelp32Snapshot_type) ProcAddress("CreateToolhelp32Snapshot"))
				,FirstModule32           ((Module32First_type           ) ProcAddress("Module32First"))
				,NextModule32            ((Module32Next_type            ) ProcAddress("Module32Next"))
			{}
			ToolHelp_Dll(ToolHelp_Dll&&) = delete;
			ToolHelp_Dll(ToolHelp_Dll const&) = delete;
			ToolHelp_Dll& operator=(ToolHelp_Dll&&) = delete;
			ToolHelp_Dll& operator=(ToolHelp_Dll const&) = delete;
	
			// Returns true if all required functions were found
			bool Loaded() const
			{
				return CreateToolhelp32Snapshot != nullptr && FirstModule32 != nullptr && NextModule32 != nullptr;
			}
		};

		// This class represents psapi.dll and the functions that we use from it
		struct PSAPI_Dll :DllBase
		{
			typedef BOOL (__stdcall* EnumProcessModules_type  )(HANDLE, HMODULE*, DWORD, LPDWORD);
			typedef BOOL (__stdcall* GetModuleInformation_type)(HANDLE, HMODULE, LPMODULEINFO, DWORD);

			EnumProcessModules_type   EnumProcModules;
			GetModuleInformation_type GetModuleInfo;

			PSAPI_Dll()
				:DllBase(L"psapi.dll")
				,EnumProcModules((EnumProcessModules_type  ) ProcAddress("EnumProcessModules"))
				,GetModuleInfo  ((GetModuleInformation_type) ProcAddress("GetModuleInformation"))
			{}
			PSAPI_Dll(PSAPI_Dll&&) = delete;
			PSAPI_Dll(PSAPI_Dll const&) = delete;
			PSAPI_Dll& operator=(PSAPI_Dll&&) = delete;
			PSAPI_Dll& operator=(PSAPI_Dll const&) = delete;

			// Returns true if all required functions were found
			bool Loaded() const
			{
				return EnumProcModules != nullptr && GetModuleInfo != nullptr;
			}
		};

		// This class represents dbghelp.dll and the functions that we use from it.
		// It also keeps track of the dll's initialized state and will call Clean up if necessary before it unloads
		struct DbgHelp_Dll :DllBase
		{
			typedef BOOL   (WINAPI* SymInitialize_type         )(HANDLE, PSTR, BOOL);
			typedef BOOL   (WINAPI* SymCleanup_type            )(HANDLE);
			typedef DWORD  (WINAPI* SymSetOptions_type         )(DWORD);
			typedef PVOID  (WINAPI* SymFunctionTableAccess_type)(HANDLE, DWORD64);
			typedef DWORD64(WINAPI* SymGetModuleBase_type      )(HANDLE, DWORD64);
			typedef BOOL   (WINAPI* StackWalk_type             )(DWORD, HANDLE, HANDLE, LPSTACKFRAME64, PVOID, PREAD_PROCESS_MEMORY_ROUTINE64, PFUNCTION_TABLE_ACCESS_ROUTINE64, PGET_MODULE_BASE_ROUTINE64, PTRANSLATE_ADDRESS_ROUTINE64);
			typedef DWORD64(WINAPI* SymLoadModule_type         )(HANDLE, HANDLE, PSTR, PSTR, DWORD64, DWORD);
			typedef BOOL   (WINAPI* SymGetLineFromAddr_type    )(HANDLE, DWORD64, PDWORD, PIMAGEHLP_LINE64);
			typedef BOOL   (WINAPI* SymGetSearchPath_type      )(HANDLE, PSTR, DWORD);
			typedef BOOL   (WINAPI* SymFromAddr_type           )(HANDLE, DWORD64, PDWORD64, PSYMBOL_INFO);

			// This is true if the DLL was found, loaded, all required functions found, and initialised without error
			bool m_loaded;

			// The following are functions that we absolutely required in order to do stack traces
			SymInitialize_type          SymInitialize;
			SymCleanup_type             SymCleanup;
			SymSetOptions_type          SymSetOptions;
			SymFunctionTableAccess_type SymFunctionTableAccess64;
			SymGetModuleBase_type       SymGetModuleBase64;
			StackWalk_type              StackWalk64;

			// The following are functions that we may be able to do without, but they are always
			// provided if the functions above are provided (so we assume they will be present)
			SymLoadModule_type      SymLoadModule64;
			SymGetLineFromAddr_type SymGetLineFromAddr64;
			SymGetSearchPath_type   SymGetSearchPath;
			SymFromAddr_type        SymFromAddr;

			DbgHelp_Dll()
				:DllBase(L"dbghelp.dll")
				,m_loaded(false)
				,SymInitialize            ((SymInitialize_type         ) ProcAddress("SymInitialize"           ))
				,SymCleanup               ((SymCleanup_type            ) ProcAddress("SymCleanup"              ))
				,SymSetOptions            ((SymSetOptions_type         ) ProcAddress("SymSetOptions"           ))
				,SymFunctionTableAccess64 ((SymFunctionTableAccess_type) ProcAddress("SymFunctionTableAccess64"))
				,SymGetModuleBase64       ((SymGetModuleBase_type      ) ProcAddress("SymGetModuleBase64"      ))
				,StackWalk64              ((StackWalk_type             ) ProcAddress("StackWalk64"             ))
				,SymLoadModule64          ((SymLoadModule_type         ) ProcAddress("SymLoadModule64"         ))
				,SymGetLineFromAddr64     ((SymGetLineFromAddr_type    ) ProcAddress("SymGetLineFromAddr64"    ))
				,SymGetSearchPath         ((SymGetSearchPath_type      ) ProcAddress("SymGetSearchPath"        ))
				,SymFromAddr              ((SymFromAddr_type           ) ProcAddress("SymFromAddr"             ))
			{
				// If any critical functions (or the DLL itself) are missing, give up
				if (SymInitialize            == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymCleanup               == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymSetOptions            == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymFunctionTableAccess64 == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymGetModuleBase64       == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (StackWalk64              == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymLoadModule64          == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymGetLineFromAddr64     == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymGetSearchPath         == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");
				if (SymFromAddr              == nullptr) throw std::exception("failed to load dbghelp.dll for stack dump");

				SymSetOptions(SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);
				if (!SymInitialize(Process(), 0, FALSE)) // Initialise 'dbghelp'
					throw std::exception("failed to initialise dbghelp.dll for stack dump");

				LoadModules();   // Enumerate and load all Process modules
			}
			DbgHelp_Dll(DbgHelp_Dll&&) = delete;
			DbgHelp_Dll(DbgHelp_Dll const&) = delete;
			DbgHelp_Dll& operator=(DbgHelp_Dll&&) = delete;
			DbgHelp_Dll& operator=(DbgHelp_Dll const&) = delete;
			~DbgHelp_Dll()
			{
				SymCleanup(Process());
			}

			// Singleton accessor
			static DbgHelp_Dll& get()
			{
				static DbgHelp_Dll me;
				return me;
			}

			// The value used to initialize dbghelp.dll. We always use the Process id here, so it will work on 95/98/Me
			static HANDLE Process()
			{
				return GetCurrentProcess();
			}

			// Helper function for getting the full path and file name for a module. Returns std::string() on error
			static std::filesystem::path GetModuleFilename(HMODULE module)
			{
				wchar_t result[_MAX_PATH + 1] = {};
				if (!::GetModuleFileNameW(module, result, _countof(result)))
					result[0] = 0;

				return result;
			}

			// Load all the modules in this Process, using the PSAPI
			void LoadModules(PSAPI_Dll const& psapi)
			{
				DWORD bytes_needed;
				psapi.EnumProcModules(GetCurrentProcess(), (HMODULE*) &bytes_needed, 0, &bytes_needed);
				SetLastError(0);

				auto modules = reinterpret_cast<HMODULE*>(_alloca(bytes_needed));
				if (!psapi.EnumProcModules(GetCurrentProcess(), modules, bytes_needed, &bytes_needed))
					return;

				for (auto i = modules, end = modules + bytes_needed / sizeof(HMODULE); i != end;)
				{
					auto module = *i++;

					MODULEINFO info;
					if (psapi.GetModuleInfo(GetCurrentProcess(), module, &info, sizeof(info)))
					{
						auto filename = GetModuleFilename(module).string();
						SymLoadModule64(Process(), 0, filename.data(), 0, reinterpret_cast<DWORD64>(info.lpBaseOfDll), info.SizeOfImage);
					}
				}
			}

			// Load all the modules in this Process, using the ToolHelp API
			void LoadModules(ToolHelp_Dll const& toolhelp)
			{
				const HANDLE snapshot = toolhelp.CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
				if (snapshot == INVALID_HANDLE_VALUE)
					return;

				ScopedHandle snapshot_guard(snapshot);
				tagMODULEENTRY32 module = {};
				module.dwSize = sizeof(module);
				for (BOOL ok = toolhelp.FirstModule32(snapshot, &module); ok; ok = toolhelp.NextModule32(snapshot, &module))
				{
					auto exe_path = module.szExePath;
					auto mod_name = module.szModule;
					SymLoadModule64(Process(), 0, exe_path, mod_name, reinterpret_cast<DWORD64>(module.modBaseAddr), module.modBaseSize);
				}
			}

			// This function will try anything to enumerate and load all of this Process' modules
			void LoadModules()
			{
				// Try the ToolHelp API first
				const ToolHelp_Dll toolhelp;
				if (toolhelp.Loaded())
				{
					LoadModules(toolhelp);
					return;
				}

				// Try PSAPI if ToolHelp isn't present
				const PSAPI_Dll psapi;
				if (psapi.Loaded())
				{
					LoadModules(psapi);
					return;
				}

				// One of the two above should work.  The only possible case where they wouldn't is
				//  if an NT user deleted psapi.dll...
			}

			// Retrieves the current search path, or std::string() on error
			std::filesystem::path GetSearchPath()
			{
				char result[_MAX_PATH + 1] = {};
				if (!SymGetSearchPath(Process(), result, _MAX_PATH+1))
					result[0] = 0;
				return result;
			}
		};

		template <typename = void>
		struct StackDumpImpl
		{
			static CallSource GetCallSource(CallAddress p)
			{
				StackdumpLock lock;                        // Acquire the thread lock
				DbgHelp_Dll& dbghelp = DbgHelp_Dll::get(); // Ensure dbghelp.dll is Loaded

				CallSource result;

				// 'SymGetLineFromAddr64' fails if the address is not found in the PDB
				Line64 line;
				DWORD displacement;
				if (dbghelp.SymGetLineFromAddr64(dbghelp.Process(), reinterpret_cast<DWORD64>(p.m_address), &displacement, &line))
				{
					result.m_filepath = line.FileName;
					result.m_line     = line.LineNumber;
				}
				else
				{
					[[maybe_unused]] DWORD e = GetLastError();
					char s_filename[MAX_PATH + 1] = {};
					_snprintf(s_filename, MAX_PATH, "0x%p", p.m_address);
					result.m_filepath = s_filename;
					result.m_line     = 0;
				}

				SymInfo info;
				DWORD64 displacement64;
				if (dbghelp.SymFromAddr(dbghelp.Process(), reinterpret_cast<DWORD64>(p.m_address), &displacement64, &info))
				{
					result.m_sym_name = std::string(&info.Name[0], &info.Name[info.NameLen]);
				}
				else
				{
				}
				return result;
			}

			// This is used as the callback that StackWalk uses to read Process memory
			// We pass it through to ReadProcessMemory, after making the call 64-bit safe, and make a note of any errors returned.
			static BOOL CALLBACK DoReadProcessMemory(HANDLE, DWORD64 address, void* buffer, DWORD size, DWORD* bytes_read)
			{
				// On Win32, only 32 bits of 'address' is used
				// We use a separate SIZE_T variable for the bytes_read result value, because it is 64 bits on Win64
				SIZE_T bytes_read_;
				const BOOL ret = ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<const void*>(address), buffer, size, &bytes_read_);
				*bytes_read = static_cast<DWORD>(bytes_read_);
				return ret;
			}
		};
	}

	// Return the call source from an arbitrary memory address
	inline CallSource GetCallSource(void* address)
	{
		return impl::StackDumpImpl<>::GetCallSource(reinterpret_cast<CallAddress const&>(address));
	}

	// The stack dump function
	// 'out' is a call back function called for each stack frame
	// 'skip' is the number of initial stack frames to not call 'func' for
	// 'count' is the number of stack frames to call 'func' for before stopping
	// 'TOut(std::string const& sym, std::filesystem::path const& file, int line)'
	template <typename TOut> void DumpStack(TOut out, size_t skip, size_t count)
	{
		size_t const max_frames = 256;
		CallAddress frames[max_frames];
		if (count > max_frames) count = max_frames;
		count = CaptureStackBackTrace(DWORD(skip), DWORD(count), reinterpret_cast<void**>(&frames[0]), nullptr);

		// Get symbol info and output the frames
		for (size_t i = 0; i != count; ++i)
		{
			CallSource src = impl::StackDumpImpl<>::GetCallSource(frames[i]);
			out(src.m_sym_name, src.m_filepath.string(), src.m_line);
		}
	}

	// Return the stack dump as a string
	inline std::string DumpStack(size_t skip, size_t count)
	{
		std::stringstream out;
		pr::DumpStack([&](std::string const& sym, std::filesystem::path const& file, int line)
		{
			out << file << "(" << line << "): " << sym << std::endl;
		}, skip, count);
		return out.str();
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::common
{
	namespace unittests::stackdump
	{
		struct StackDumpTest
		{
			template <typename TOut> static __declspec(noinline) void Func1(TOut out) { Func2(out); }
			template <typename TOut> static __declspec(noinline) void Func2(TOut out) { Func3(out); }
			template <typename TOut> static __declspec(noinline) void Func3(TOut out) { DumpStack(out); }
		};
	}
	PRUnitTest(StackDumpTests)
	{
		#if 0 // not working under VS2017... don't know why yet
		std::stringstream out;
		size_t fcount = 0;
		StackDumpTest::Func1([&](std::string sym, std::string file, int line)
		{
			out << file << "(" << line << "): " << sym << std::endl;
			++fcount;
		});

		PR_EXPECT(fcount > 3);

		// Requires debug symbols...
		#if !defined(NDEBUG)
		auto s = out.str();
		std::string::size_type ofs = 0U;
		PR_EXPECT((ofs = s.find("unittests::StackDumpTest::Func3", ofs)) != std::string::npos);
		PR_EXPECT((ofs = s.find("unittests::StackDumpTest::Func2", ofs)) != std::string::npos);
		PR_EXPECT((ofs = s.find("unittests::StackDumpTest::Func1", ofs)) != std::string::npos);
		#endif
		#endif
	}
}
#endif


	#if 0 // Ye Olde Way...
	template <typename TOut> void StackDump2(TOut out, int count = 100, int skip = 0)
	{
		impl::StackdumpLock lock;                              // Acquire the thread lock
		impl::DbgHelp_Dll& dbghelp = impl::DbgHelp_Dll::get(); // Ensure dbghelp.dll is Loaded

		// Get the context record directly
		// You're really not supposed to do this...
		CONTEXT ctx;
		RtlCaptureContext(&ctx);

		// Ignore the msdn docs, see: http://www.codeproject.com/Articles/11132/Walking-the-callstack
		STACKFRAME64 frame = {};
		#if defined(_M_IX86)
		const DWORD image_file_type = IMAGE_FILE_MACHINE_I386;
		frame.AddrPC.Mode           = AddrModeFlat;
		frame.AddrPC.Offset         = ctx.Eip;
		frame.AddrFrame.Mode        = AddrModeFlat;
		frame.AddrFrame.Offset      = ctx.Ebp;
		frame.AddrStack.Mode        = AddrModeFlat;
		frame.AddrStack.Offset      = ctx.Esp;
		#elif defined(_M_IA64)
		const DWORD image_file_type = IMAGE_FILE_MACHINE_IA64;
		frame.AddrPC.Mode           = AddrModeFlat;
		frame.AddrPC                = ctx.StIIP;
		frame.AddrFrame.Mode        = AddrModeFlat;
		frame.AddrFrame.Offset      = ctx.RsBSP;
		frame.AddrStack.Mode        = AddrModeFlat;
		frame.AddrStack.Offset      = ctx.IntSp;
		frame.AddrBStore.Mode       = AddrModeFlat;
		frame.AddrBStore            = ctx.RsBSP;
		#elif defined(_M_AMD64)
		const DWORD image_file_type = IMAGE_FILE_MACHINE_AMD64;
		frame.AddrPC.Mode           = AddrModeFlat;
		frame.AddrPC.Offset         = ctx.Rip;
		frame.AddrFrame.Mode        = AddrModeFlat;
		frame.AddrFrame.Offset      = ctx.Rsp;
		frame.AddrStack.Mode        = AddrModeFlat;
		frame.AddrStack.Offset      = ctx.Rsp;
		#else
		#error Unknown machine type!
		#endif

		size_t fcount = 0;
		size_t const max_fcount = 100;
		CallAddress frames[max_fcount];
		while (dbghelp.StackWalk64(
			image_file_type,
			dbghelp.Process(),
			GetCurrentThread(),
			&frame,
			&ctx,
			&impl::StackDumpImpl<>::DoReadProcessMemory,
			dbghelp.SymFunctionTableAccess64,
			dbghelp.SymGetModuleBase64,
			0))
		{
			// Sometimes StackWalk will leave the last error set, which causes us to report an (incorrect) error after this loop.
			SetLastError(0);
			if (skip > 0) --skip;
			else if (count-- == 0) break;
			else if (fcount == max_fcount) break;
			else frames[fcount++] = reinterpret_cast<CallAddress const&>(frame.AddrPC.Offset);
			if (frame.AddrReturn.Offset == 0)
				break;
		}

		// Get symbol info and output the frames
		for (size_t i = 0; i != fcount; ++i)
		{
			CallSource src = impl::StackDumpImpl<>::GetCallSource(frames[i]);
			out(src.m_sym_name, src.m_filepath, src.m_line);
		}
	}
	#endif