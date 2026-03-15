//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	struct ResourceIncludeHandler : IDxcIncludeHandler
	{
		// Notes:
		// - Include handler that loads files from embedded resources

		// IUnknown methods
		ULONG STDMETHODCALLTYPE AddRef() override
		{
			return InterlockedIncrement(&m_ref_count);
		}
		ULONG STDMETHODCALLTYPE Release() override
		{
			ULONG ref_count = InterlockedDecrement(&m_ref_count);
			if (ref_count == 0) { delete this; }
			return ref_count;
		}
		HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject) override
		{
			if (riid == __uuidof(IUnknown) || riid == __uuidof(IDxcIncludeHandler))
			{
				*ppvObject = static_cast<IDxcIncludeHandler*>(this);
				AddRef();
				return S_OK;
			}
			*ppvObject = nullptr;
			return E_NOINTERFACE;
		}

		/// <inheritdoc/>
		HRESULT STDMETHODCALLTYPE LoadSource(LPCWSTR pFilename, IDxcBlob** ppIncludeSource) override
		{
			std::wstring name = resource::Name(pFilename);

			// Start with the full path, and trim off parent directories until the resource is resolved
			for (;;)
			{
				if (resource::Find(name, L"TEXT"))
					break;

				// Trim upto and including the next '/\\'
				if (auto i = name.find_first_of(L"./\\"); i != std::wstring::npos)
					name = name.substr(i + 1);
				else
					throw std::runtime_error(std::format("Resource for '{}' not found", Narrow(pFilename)));
			}

			// Read the file from the resources
			auto source = resource::Read<char>(name, L"TEXT");
		
			D3DPtr<IDxcLibrary> library;
			Check(DxcCreateInstance(CLSID_DxcLibrary, __uuidof(IDxcLibrary), (void**)library.address_of()));

			D3DPtr<IDxcBlobEncoding> blob;
			Check(library->CreateBlobWithEncodingOnHeapCopy(source.m_data, static_cast<UINT32>(source.m_len), CP_UTF8, blob.address_of()));

			*ppIncludeSource = blob.release();
			return S_OK;
		}

	private:
		ULONG m_ref_count = 1;
	};
}
