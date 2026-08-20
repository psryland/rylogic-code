//*******************************************************
// PE File
//  Copyright (c) Rylogic Ltd 2015
//*******************************************************
// Originally written by Robert Simpson (robert@blackcastlesoft.com)

#pragma once
#include <winnt.h>
#include <format>
#include "pr/common/cast.h"
#include "pr/common/hresult.h"

namespace pr
{
	// Opens and maps out an executable file
	class PEFile
	{
		enum class EImageSig :DWORD
		{
			DOS    = IMAGE_DOS_SIGNATURE   , // MZ
			OS2    = IMAGE_OS2_SIGNATURE   , // NE
			OS2_LE = IMAGE_OS2_SIGNATURE_LE, // LE
			VXD    = IMAGE_VXD_SIGNATURE   , // LE
			NT     = IMAGE_NT_SIGNATURE    , // PE00
		};

		enum class EOptionalHeaderType :WORD
		{
			Hdr32 = IMAGE_NT_OPTIONAL_HDR32_MAGIC,
			Hdr64 = IMAGE_NT_OPTIONAL_HDR64_MAGIC,
			ROM   = IMAGE_ROM_OPTIONAL_HDR_MAGIC ,
		};

		// The shared parts of IMAGE_NT_HEADERS32 and IMAGE_NT_HEADERS64
		struct IMAGE_NT_HEADERSCMN
		{
			EImageSig Signature;
			IMAGE_FILE_HEADER FileHeader;
			struct { EOptionalHeaderType Magic; } OptionalHeader;
		};

		HANDLE              m_hmap;
		HANDLE              m_hfile;
		IMAGE_DOS_HEADER*   m_base;
		union {
		IMAGE_NT_HEADERSCMN* m_nt_hdr;
		IMAGE_NT_HEADERS32*  m_nt_hdr32;
		IMAGE_NT_HEADERS64*  m_nt_hdr64;
		};
		BOOL                m_64bit;

	public:
		PEFile()
			:m_hmap()
			,m_hfile(INVALID_HANDLE_VALUE)
			,m_base()
			,m_nt_hdr()
			,m_64bit()
		{}
		PEFile(TCHAR const* file, bool read_only = true) :PEFile()
		{
			pr::Check(Open(file, read_only), std::format("Failed to open binary PE file '{}'", file));
		}
		~PEFile()
		{
			Close();
		}

		// Open an executeable file
		HRESULT Open(TCHAR const* file, bool read_only = true)
		{
			Close();

			HRESULT hr = S_OK;
			m_hfile = CreateFile(file, GENERIC_READ | (!read_only ? GENERIC_WRITE : 0), FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
			if (m_hfile == INVALID_HANDLE_VALUE)
			{
				hr = HRESULT_FROM_WIN32(GetLastError());
			}
			else
			{
				m_hmap = CreateFileMapping(m_hfile, NULL, read_only ? PAGE_READONLY : PAGE_READWRITE, 0, 0, NULL);
				if (!m_hmap)
				{
					hr = HRESULT_FROM_WIN32(GetLastError());
				}
				else
				{
					m_base = (IMAGE_DOS_HEADER*)MapViewOfFile(m_hmap, FILE_MAP_READ | (!read_only ? FILE_MAP_WRITE : 0), 0, 0, 0);
					if (!m_base)
					{
						hr = HRESULT_FROM_WIN32(GetLastError());
					}
				}
			}

			if (SUCCEEDED(hr))
			{
				if (m_base->e_magic != IMAGE_DOS_SIGNATURE)
					hr = HRESULT_FROM_WIN32(ERROR_BAD_FORMAT);

				m_nt_hdr = reinterpret_cast<IMAGE_NT_HEADERSCMN*>((DWORD_PTR)m_base + (DWORD_PTR)m_base->e_lfanew);
				if (IsBadReadPtr(m_nt_hdr, sizeof(m_nt_hdr->Signature)))
				{
					hr = HRESULT_FROM_WIN32(ERROR_BAD_FORMAT);
				}
				else
				{
					if (m_nt_hdr->Signature != EImageSig::NT)
					{
						hr = HRESULT_FROM_WIN32(ERROR_BAD_FORMAT);
					}
				}
				m_64bit = m_nt_hdr->OptionalHeader.Magic == EOptionalHeaderType::Hdr64;
			}

			if (FAILED(hr))
				Close();

			return hr;
		}
		void Close()
		{
			if (m_base)
				UnmapViewOfFile(m_base);

			if (m_hmap)
				CloseHandle(m_hmap);

			if (m_hfile != INVALID_HANDLE_VALUE)
				CloseHandle(m_hfile);

			m_hmap = NULL;
			m_hfile = INVALID_HANDLE_VALUE;
			m_base = NULL;
		}

		IMAGE_SECTION_HEADER* EnclosingSectionHeader(DWORD rva) const
		{
			auto section = m_64bit
				? IMAGE_FIRST_SECTION(m_nt_hdr64)
				: IMAGE_FIRST_SECTION(m_nt_hdr32);

			for (UINT i = 0; i < m_nt_hdr->FileHeader.NumberOfSections; ++i, ++section)
			{
				auto size = section->Misc.VirtualSize;
				if (!size) size = section->SizeOfRawData;

				if (rva >= section->VirtualAddress &&
					rva <  section->VirtualAddress + size)
					return section;
			}
			return nullptr;
		}
		IMAGE_SECTION_HEADER* SectionHeader(char const* name) const
		{
			auto section = m_64bit
				? IMAGE_FIRST_SECTION(m_nt_hdr64)
				: IMAGE_FIRST_SECTION(m_nt_hdr32);

			for (UINT i = 0; i != m_nt_hdr->FileHeader.NumberOfSections; ++i, ++section)
			{
				if (_strnicmp(reinterpret_cast<char const*>(section->Name), name, IMAGE_SIZEOF_SHORT_NAME) == 0)
					return section;
			}

			return nullptr;
		}

		void* PtrFromRVA(DWORD rva) const
		{
			auto section_hdr = EnclosingSectionHeader(rva);
			if (!section_hdr)
				return nullptr;

			auto delta = INT(section_hdr->VirtualAddress - section_hdr->PointerToRawData);
			return pr::byte_ptr(m_base) + rva - delta;
		}

		operator IMAGE_DOS_HEADER*() const
		{
			return m_base;
		}
		operator IMAGE_NT_HEADERS32*() const
		{
			if (!m_base || m_64bit) return nullptr;
			return m_nt_hdr32;
		}
		operator IMAGE_NT_HEADERS64*() const
		{
			if (!m_base || !m_64bit) return nullptr;
			return m_nt_hdr64;
		}
		operator IMAGE_COR20_HEADER*() const
		{
			if (!m_base)
				return nullptr;

			auto rva = m_64bit
				? m_nt_hdr64->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_COM_DESCRIPTOR].VirtualAddress
				: m_nt_hdr32->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_COM_DESCRIPTOR].VirtualAddress;
			if (!rva)
				return nullptr;

			return static_cast<IMAGE_COR20_HEADER*>(PtrFromRVA(rva));
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(PeFileTests, Quick)
	{
		TCHAR name[256];
		GetModuleFileName(nullptr, name, _countof(name));
		PEFile file(name, true);
		auto hdr = file.SectionHeader(".text");
		PR_EXPECT(_stricmp((char const*)hdr->Name, ".text") == 0);
	}
}
#endif