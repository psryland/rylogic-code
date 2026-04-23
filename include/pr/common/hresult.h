//********************************
// HRESULT error codes
//  Copyright (c) Rylogic Ltd 2011
//********************************
#pragma once
#include <vector>
#include <string>
#include <string_view>
#include <sstream>
#include <iostream>
#include <type_traits>
#include <cassert>
#include <cerrno>
#include <format>
#include <stdexcept>
#include <windows.h>

// Support d3d errors if the header has been included before now.
#if defined(PR_SUPPORT_D3D_HRESULTS) || defined(_D3D9_H_)
	#ifndef PR_SUPPORT_D3D9_ERRORS
	#define PR_SUPPORT_D3D9_ERRORS
	#endif
#endif
#ifdef PR_SUPPORT_D3D9_ERRORS
	// Dxerr9.h requires the DXSDK to be installed
	#include <d3d9.h>
	#include <dxerr9.h>
	#pragma comment(lib, "dxerr.lib")
#endif
#if defined(PR_SUPPORT_D3D_HRESULTS) || defined(__d3d11_h__)
	#ifdef PR_SUPPORT_D3D11_ERRORS
	#define PR_SUPPORT_D3D11_ERRORS 1
	#endif
#endif
#ifdef PR_SUPPORT_D3D11_ERRORS
	#include <d3d11.h>
	#include <dxgitype.h>
#endif

namespace pr
{
	// Map HRESULT to an enum type
	enum class HResult :long long
	{
		Ok   = HRESULT(S_OK),
		Fail = HRESULT(E_FAIL),
	};

	// Convert an HRESULT into an error message string
	inline std::string HrMsg(HRESULT result)
	{
		// If it's a d3d9 error code
		#ifdef PR_SUPPORT_D3D9_ERRORS
		if (HRESULT_FACILITY(result) == _FACD3D)
		{
			std::stringstream ss;
			ss << "D3D9 Error: "  << DXGetErrorString(result) << std::endl;
			ss << "Description: " << DXGetErrorDescription(result) << std::endl;
			return ss.str();
		}
		#endif
		#ifdef PR_SUPPORT_D3D11_ERRORS
		if (HRESULT_FACILITY(result) == _FACDXGI)
		{
			char const *code, *desc;
			switch (result)
			{
			default: code = "Unknown DXGI error"; desc = ""; break;
			case DXGI_ERROR_DEVICE_HUNG:                 code = "DXGI_ERROR_DEVICE_HUNG";                  desc = "The application's device failed due to badly formed commands sent by the application. This is an design-time issue that should be investigated and fixed."; break;
			case DXGI_ERROR_DEVICE_REMOVED:              code = "DXGI_ERROR_DEVICE_REMOVED";               desc = "The video card has been physically removed from the system, or a driver upgrade for the video card has occurred. The application should destroy and recreate the device. For help debugging the problem, call ID3D10Device::GetDeviceRemovedReason."; break;
			case DXGI_ERROR_DEVICE_RESET:                code = "DXGI_ERROR_DEVICE_RESET";                 desc = "The device failed due to a badly formed command. This is a run-time issue; The application should destroy and recreate the device."; break;
			case DXGI_ERROR_DRIVER_INTERNAL_ERROR:       code = "DXGI_ERROR_DRIVER_INTERNAL_ERROR";        desc = "The driver encountered a problem and was put into the device removed state."; break;
			case DXGI_ERROR_FRAME_STATISTICS_DISJOINT:   code = "DXGI_ERROR_FRAME_STATISTICS_DISJOINT";    desc = "An event (for example, a power cycle) interrupted the gathering of presentation statistics."; break;
			case DXGI_ERROR_GRAPHICS_VIDPN_SOURCE_IN_USE:code = "DXGI_ERROR_GRAPHICS_VIDPN_SOURCE_IN_USE"; desc = "The application attempted to acquire exclusive ownership of an output, but failed because some other application (or device within the application) already acquired ownership."; break;
			case DXGI_ERROR_INVALID_CALL:                code = "DXGI_ERROR_INVALID_CALL";                 desc = "The application provided invalid parameter data; this must be debugged and fixed before the application is released."; break;
			case DXGI_ERROR_MORE_DATA:                   code = "DXGI_ERROR_MORE_DATA";                    desc = "The buffer supplied by the application is not big enough to hold the requested data."; break;
			case DXGI_ERROR_NONEXCLUSIVE:                code = "DXGI_ERROR_NONEXCLUSIVE";                 desc = "A global counter resource is in use, and the Direct3D device can't currently use the counter resource."; break;
			case DXGI_ERROR_NOT_CURRENTLY_AVAILABLE:     code = "DXGI_ERROR_NOT_CURRENTLY_AVAILABLE";      desc = "The resource or request is not currently available, but it might become available later."; break;
			case DXGI_ERROR_NOT_FOUND:                   code = "DXGI_ERROR_NOT_FOUND";                    desc = "When calling IDXGIObject::GetPrivateData, the GUID passed in is not recognized as one previously passed to IDXGIObject::SetPrivateData or IDXGIObject::SetPrivateDataInterface. When calling IDXGIFactory::EnumAdapters or IDXGIAdapter::EnumOutputs, the enumerated ordinal is out of range."; break;
			case DXGI_ERROR_REMOTE_CLIENT_DISCONNECTED:  code = "DXGI_ERROR_REMOTE_CLIENT_DISCONNECTED";   desc = "Reserved"; break;
			case DXGI_ERROR_REMOTE_OUTOFMEMORY:          code = "DXGI_ERROR_REMOTE_OUTOFMEMORY";           desc = "Reserved"; break;
			case DXGI_ERROR_WAS_STILL_DRAWING:           code = "DXGI_ERROR_WAS_STILL_DRAWING";            desc = "The GPU was busy at the moment when a call was made to perform an operation, and did not execute or schedule the operation."; break;
			case DXGI_ERROR_UNSUPPORTED:                 code = "DXGI_ERROR_UNSUPPORTED";                  desc = "The requested functionality is not supported by the device or the driver."; break;
			case DXGI_ERROR_ACCESS_LOST:                 code = "DXGI_ERROR_ACCESS_LOST";                  desc = "The desktop duplication interface is invalid. The desktop duplication interface typically becomes invalid when a different type of image is displayed on the desktop."; break;
			case DXGI_ERROR_WAIT_TIMEOUT:                code = "DXGI_ERROR_WAIT_TIMEOUT";                 desc = "The time-out interval elapsed before the next desktop frame was available."; break;
			case DXGI_ERROR_SESSION_DISCONNECTED:        code = "DXGI_ERROR_SESSION_DISCONNECTED";         desc = "The Remote Desktop Services session is currently disconnected."; break;
			case DXGI_ERROR_RESTRICT_TO_OUTPUT_STALE:    code = "DXGI_ERROR_RESTRICT_TO_OUTPUT_STALE";     desc = "The DXGI output (monitor) to which the swap chain content was restricted is now disconnected or changed."; break;
			case DXGI_ERROR_CANNOT_PROTECT_CONTENT:      code = "DXGI_ERROR_CANNOT_PROTECT_CONTENT";       desc = "DXGI can't provide content protection on the swap chain. This error is typically caused by an older driver, or when you use a swap chain that is incompatible with content protection."; break;
			case DXGI_ERROR_ACCESS_DENIED:               code = "DXGI_ERROR_ACCESS_DENIED";                desc = "You tried to use a resource to which you did not have the required access privileges. This error is most typically caused when you write to a shared resource with read-only access."; break;
			case DXGI_ERROR_NAME_ALREADY_EXISTS:         code = "DXGI_ERROR_NAME_ALREADY_EXISTS";          desc = "The supplied name of a resource in a call to IDXGIResource1::CreateSharedHandle is already associated with some other resource."; break;
			case S_OK:                                   code = "S_OK";                                    desc = "The method succeeded without an error."; break;
			}
			std::stringstream ss;
			ss << "D3D11 Error: " << code << std::endl;
			ss << "Description: " << desc << std::endl;
			return ss.str();
		}
		#endif

		// Convert win32 error codes to HRESULTS (leaves HRESULTS unchanged)
		result = HRESULT_FROM_WIN32(result);

		// else ask windows
		std::string msg(4096, '\0');
		auto len = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM|FORMAT_MESSAGE_IGNORE_INSERTS, NULL, result, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), msg.data(), DWORD(msg.size()), nullptr);
		if (len != 0) msg.resize(len); else msg = std::format("Unknown error code: 0x{:08X}", static_cast<unsigned>(result));
		return msg;
	}

	// Forward declare the ToString function
	inline std::string ToString(HResult result)
	{
		return HrMsg(static_cast<HRESULT>(result));
	}

	// A string to put the last error message into
	inline std::string& Reason()
	{
		static std::string s_reason;
		return s_reason;
	}

	// Success / Failure / Verify
	template <typename Result> inline bool Succeeded(Result result)
	{
		static_assert(std::is_enum<Result>::value, "Only enum result codes should be used as ToString() for other types has a different meaning");

		if (static_cast<long long>(result) >= 0) return true;
		Reason() = ToString(result);
		// std::cerr << Reason(); <- Caller's choice to output.
		return false;
	}
	template <> inline bool Succeeded(__int64 result)
	{
		return Succeeded(static_cast<HResult>(result));
	}
	template <> inline bool Succeeded(HRESULT result)
	{
		return Succeeded(static_cast<HResult>(result));
	}
	template <> inline bool Succeeded(BOOL result)
	{
		return Succeeded(result != 0 ? HResult::Ok : HResult::Fail);
	}
	template <> inline bool Succeeded(bool result)
	{
		if (result) return true;
		Reason() = "false returned";
		// std::cerr << Reason(); <- Caller's choice to output.
		return false;
	}
	template <typename Result> inline bool Failed(Result result)
	{
		return !Succeeded(result);
	}
	template <typename Result> inline void Verify(Result result)
	{
		assert(Succeeded(result) && Reason().c_str());
		(void)result;
	}
	template <typename Result> inline void Check(Result result, std::string_view msg = {})
	{
		if (Succeeded(result)) return;
		throw std::runtime_error(std::string(msg).append(!msg.empty()?" ":"").append(Reason()));
	}

	// Check the 'errno' value, and throw if non-zero
	template <typename T> inline T const& CheckErrno(T const& r)
	{
		auto err = errno;
		switch (err)
		{
			case EPERM:        throw std::runtime_error("Operation not permitted");
			case ENOENT:       throw std::runtime_error("No such file or directory");
			case ESRCH:        throw std::runtime_error("No such process");
			case EINTR:        throw std::runtime_error("Interrupted function");
			case EIO:          throw std::runtime_error("I/O error");
			case ENXIO:        throw std::runtime_error("No such device or address");
			case E2BIG:        throw std::runtime_error("Argument list too long");
			case ENOEXEC:      throw std::runtime_error("Exec format error");
			case EBADF:        throw std::runtime_error("Bad file number");
			case ECHILD:       throw std::runtime_error("No spawned processes");
			case EAGAIN:       throw std::runtime_error("No more processes or not enough memory or maximum nesting level reached");
			case ENOMEM:       throw std::runtime_error("Not enough memory");
			case EACCES:       throw std::runtime_error("Permission denied");
			case EFAULT:       throw std::runtime_error("Bad address");
			case EBUSY:        throw std::runtime_error("Device or resource busy");
			case EEXIST:       throw std::runtime_error("File exists");
			case EXDEV:        throw std::runtime_error("Cross-device link");
			case ENODEV:       throw std::runtime_error("No such device");
			case ENOTDIR:      throw std::runtime_error("Not a directory");
			case EISDIR:       throw std::runtime_error("Is a directory");
			case EINVAL:       throw std::runtime_error("Invalid argument");
			case ENFILE:       throw std::runtime_error("Too many files open in system");
			case EMFILE:       throw std::runtime_error("Too many open files");
			case ENOTTY:       throw std::runtime_error("Inappropriate I/O control operation");
			case EFBIG:        throw std::runtime_error("File too large");
			case ENOSPC:       throw std::runtime_error("No space left on device");
			case ESPIPE:       throw std::runtime_error("Invalid seek");
			case EROFS:        throw std::runtime_error("Read-only file system");
			case EMLINK:       throw std::runtime_error("Too many links");
			case EPIPE:        throw std::runtime_error("Broken pipe");
			case EDOM:         throw std::runtime_error("Maths argument");
			case ERANGE:       throw std::runtime_error("Result too large (overflow or underflow)");
			case EDEADLK:      throw std::runtime_error("Resource deadlock would occur");
			case ENAMETOOLONG: throw std::runtime_error("Filename too long");
			case ENOLCK:       throw std::runtime_error("No locks available");
			case ENOSYS:       throw std::runtime_error("Function not supported");
			case ENOTEMPTY:    throw std::runtime_error("Directory not empty");
			case EILSEQ:       throw std::runtime_error("Illegal byte sequence");
			case STRUNCATE:    throw std::runtime_error("String was truncated");
			default:           return r;
		}
	}
}

#undef PR_SUPPORT_D3D9_ERRORS
#undef PR_SUPPORT_D3D11_ERRORS
