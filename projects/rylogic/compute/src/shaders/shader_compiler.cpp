//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/compute/shaders/shader_compiler.h"
#pragma comment(lib, "dxcompiler.lib")

namespace pr::compute
{
	using namespace shader_cache;

	namespace
	{
		// DXC include handler that routes include requests through an 'ISourceResolver' so include contents and dependency metadata are collected together.
		struct ResolverIncludeHandler : IDxcIncludeHandler
		{
			ISourceResolver const* m_resolver;
			std::vector<Dependency>* m_dependencies;
			D3DPtr<IDxcLibrary> m_library;
			ULONG m_ref_count;

			// Create an include handler that records every resolved include as a cache dependency.
			ResolverIncludeHandler(ISourceResolver const& resolver, std::vector<Dependency>& dependencies)
				: m_resolver(&resolver)
				, m_dependencies(&dependencies)
				, m_library()
				, m_ref_count(1)
			{
				Check(DxcCreateInstance(CLSID_DxcLibrary, __uuidof(IDxcLibrary), (void**)m_library.address_of()));
			}

			// Increment the COM reference count.
			ULONG STDMETHODCALLTYPE AddRef() override
			{
				return InterlockedIncrement(&m_ref_count);
			}

			// Decrement the COM reference count.
			ULONG STDMETHODCALLTYPE Release() override
			{
				auto ref_count = InterlockedDecrement(&m_ref_count);
				if (ref_count == 0)
					delete this;

				return ref_count;
			}

			// Return the requested COM interface.
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

			// Load a DXC include through the cache-aware resolver.
			HRESULT STDMETHODCALLTYPE LoadSource(LPCWSTR pFilename, IDxcBlob** ppIncludeSource) override
			{
				try
				{
					// Resolve text as UTF-8 and remember its dependency data so cached bytecode can be invalidated when any include changes.
					auto source = m_resolver->Resolve(Narrow(pFilename));
					if (source.m_code.size() > (std::numeric_limits<UINT32>::max)())
						return E_FAIL;

					m_dependencies->push_back(std::move(source.m_dependency));

					D3DPtr<IDxcBlobEncoding> blob;
					Check(m_library->CreateBlobWithEncodingOnHeapCopy(source.m_code.data(), static_cast<UINT32>(source.m_code.size()), CP_UTF8, blob.address_of()));
					*ppIncludeSource = blob.release();
					return S_OK;
				}
				catch (...)
				{
					*ppIncludeSource = nullptr;
					return E_FAIL;
				}
			}
		};

		// Return metadata that invalidates cached output when DXC changes.
		std::wstring DxcCompilerKey()
		{
			static auto const key = []()
			{
				if (auto module = ::GetModuleHandleW(L"dxcompiler.dll"))
				{
					auto path = CanonicalPath(win32::ModuleFileName(module));
					auto dep = FileDependency(path);

					// The compiler DLL path, size, and write time are enough to separate cache entries compiled by different DXC binaries.
					return std::format(L"{};{};{}", dep.m_path.wstring(), dep.m_size, dep.m_last_write);
				}
				return std::wstring(L"dxcompiler.dll;unknown");
			}();
			return key;
		}
	}

	// Create a DXC compiler wrapper
	ShaderCompiler::ShaderCompiler()
		: m_result()
		, m_compiler()
		, m_source_blob()
		, m_includes()
		, m_cache()
		, m_include_resolver()
		, m_owned_resolver()
		, m_pdb_dir()
		, m_pdb_name_hint()
		, m_pdb_arg()
		, m_source_code()
		, m_source()
		, m_source_dependency()
		, m_source_identity()
		, m_defines()
		, m_ep()
		, m_sm()
		, m_hlsl_version(EHlslVersion::Hlsl2021)
		, m_optimise()
		, m_debug_info()
		, m_extras()
		, m_args()
		, m_canonical_args()
	{
		Check(DxcCreateInstance(CLSID_DxcCompiler, __uuidof(IDxcCompiler3), (void**)m_compiler.address_of()));

		// Keep DXC's default include handler available for callers that compile raw source and do not need cache dependency tracking.
		D3DPtr<IDxcUtils> utils;
		Check(DxcCreateInstance(CLSID_DxcUtils, __uuidof(IDxcUtils), (void**)utils.address_of()));
		Check(utils->CreateDefaultIncludeHandler(m_includes.address_of()));
	}

	// Move a compiler and repair internal views that point into owned storage.
	ShaderCompiler::ShaderCompiler(ShaderCompiler&& rhs)
		: m_result(std::move(rhs.m_result))
		, m_compiler(std::move(rhs.m_compiler))
		, m_source_blob(std::move(rhs.m_source_blob))
		, m_includes(std::move(rhs.m_includes))
		, m_cache(rhs.m_cache)
		, m_include_resolver(rhs.m_include_resolver)
		, m_owned_resolver(std::move(rhs.m_owned_resolver))
		, m_pdb_dir(std::move(rhs.m_pdb_dir))
		, m_pdb_name_hint(std::move(rhs.m_pdb_name_hint))
		, m_pdb_arg(std::move(rhs.m_pdb_arg))
		, m_source_code(std::move(rhs.m_source_code))
		, m_source()
		, m_source_dependency(std::move(rhs.m_source_dependency))
		, m_source_identity(std::move(rhs.m_source_identity))
		, m_defines(std::move(rhs.m_defines))
		, m_ep(std::move(rhs.m_ep))
		, m_sm(std::move(rhs.m_sm))
		, m_hlsl_version(rhs.m_hlsl_version)
		, m_optimise(rhs.m_optimise)
		, m_debug_info(rhs.m_debug_info)
		, m_extras(std::move(rhs.m_extras))
		, m_args()
		, m_canonical_args(std::move(rhs.m_canonical_args))
	{
		RebindSource();
		rhs.m_cache = nullptr;
		rhs.m_include_resolver = nullptr;
		rhs.m_args.clear();
		rhs.RebindSource();
	}

	// Move assign a compiler and repair internal views that point into owned storage.
	ShaderCompiler& ShaderCompiler::operator = (ShaderCompiler&& rhs)
	{
		if (this == &rhs)
			return *this;

		m_result = std::move(rhs.m_result);
		m_compiler = std::move(rhs.m_compiler);
		m_source_blob = std::move(rhs.m_source_blob);
		m_includes = std::move(rhs.m_includes);
		m_cache = rhs.m_cache;
		m_include_resolver = rhs.m_include_resolver;
		m_owned_resolver = std::move(rhs.m_owned_resolver);
		m_pdb_dir = std::move(rhs.m_pdb_dir);
		m_pdb_name_hint = std::move(rhs.m_pdb_name_hint);
		m_pdb_arg = std::move(rhs.m_pdb_arg);
		m_source_code = std::move(rhs.m_source_code);
		m_source_dependency = std::move(rhs.m_source_dependency);
		m_source_identity = std::move(rhs.m_source_identity);
		m_defines = std::move(rhs.m_defines);
		m_ep = std::move(rhs.m_ep);
		m_sm = std::move(rhs.m_sm);
		m_hlsl_version = rhs.m_hlsl_version;
		m_optimise = rhs.m_optimise;
		m_debug_info = rhs.m_debug_info;
		m_extras = std::move(rhs.m_extras);
		m_args.clear();
		m_canonical_args = std::move(rhs.m_canonical_args);

		RebindSource();
		rhs.m_cache = nullptr;
		rhs.m_include_resolver = nullptr;
		rhs.m_args.clear();
		rhs.RebindSource();
		return *this;
	}

	// Replace the root source and update the DxcBuffer to point at owned UTF-8 storage.
	void ShaderCompiler::SetSource(shader_cache::Source source)
	{
		m_source_blob = nullptr;
		m_source_code = std::move(source.m_code);
		m_source_identity = std::move(source.m_identity);
		m_source_dependency = std::move(source.m_dependency);
		RebindSource();
	}

	// Point the DXC source buffer at the current owned UTF-8 source text.
	void ShaderCompiler::RebindSource()
	{
		m_source = DxcBuffer{
			.Ptr = m_source_code.data(),
			.Size = m_source_code.size(),
			.Encoding = DXC_CP_UTF8,
		};
	}

	// Build the DXC argument pointer list and a canonical copy suitable for cache keys.
	void ShaderCompiler::BuildArgs()
	{
		m_args.clear();

		// Core compile options are always first so cache keys remain stable across repeated BuildArgs() calls.
		m_args.push_back(m_ep.c_str());
		m_args.push_back(m_sm.c_str());
		m_args.push_back(m_optimise ? L"-O3" : L"-Od");

		// DXC defaults can change over time, so explicitly requested HLSL versions become part of the argument list.
		switch (m_hlsl_version)
		{
			case EHlslVersion::DxcDefault:
			{
				break;
			}
			case EHlslVersion::Hlsl2021:
			{
				m_args.push_back(L"-HV");
				m_args.push_back(L"2021");
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown HLSL version");
			}
		}

		// Debug information only adds compiler switches here. PDB extraction is handled after compilation succeeds.
		if (m_debug_info)
		{
			m_args.push_back(L"-Zi");
		}

		// Definitions are stored in a map, giving deterministic argument ordering for cache keys.
		for (auto& def : m_defines)
		{
			m_args.push_back(def.second.c_str());
		}

		// DXC writes PDBs to the path following '-Fd'. The real path can be machine-specific, so canonical args use only the name hint.
		if (!m_pdb_dir.empty())
		{
			std::filesystem::create_directories(m_pdb_dir);
			m_pdb_arg = !m_pdb_name_hint.empty()
				? (m_pdb_dir / m_pdb_name_hint).wstring()
				: m_pdb_dir.wstring();

			m_args.push_back(L"-Fd");
			m_args.push_back(m_pdb_arg.c_str());
		}
		for (auto& extra : m_extras)
		{
			m_args.push_back(extra.c_str());
		}

		// Copy arguments into owning strings for the cache key, replacing volatile PDB paths with the stable PDB name hint.
		m_canonical_args.clear();
		for (auto i = size_t{}; i != m_args.size(); ++i)
		{
			auto arg = std::wstring_view(m_args[i]);
			if (arg == L"-Fd" && i + 1 != m_args.size())
			{
				m_canonical_args.push_back(std::wstring(arg));
				m_canonical_args.push_back(m_pdb_name_hint);
				++i;
				continue;
			}

			m_canonical_args.push_back(std::wstring(arg));
		}
	}

	// Return true when all inputs needed for deterministic cache validation are known.
	bool ShaderCompiler::Cacheable() const
	{
		return m_cache != nullptr && m_include_resolver != nullptr && !m_source_identity.empty() && m_source_dependency.IsValid();
	}

	// Return the cache request that describes the current source, compiler, resolver, arguments, and optional PDB output.
	CompileRequest ShaderCompiler::Request() const
	{
		return CompileRequest{
			.m_source_identity = m_source_identity,
			.m_source_dependency = m_source_dependency,
			.m_resolver_key = m_include_resolver != nullptr ? m_include_resolver->CacheKey() : std::string{},
			.m_compiler_key = DxcCompilerKey(),
			.m_args = m_canonical_args,
			.m_pdb_name_hint = m_pdb_name_hint,
			.m_pdb_output_dir = m_pdb_dir,
		};
	}

	// Compile the current source directly with DXC and return bytecode, optional PDB data, and all discovered dependencies.
	CompileResult ShaderCompiler::CompileUncached()
	{
		auto dependencies = std::vector<Dependency>{};
		if (m_source_dependency.IsValid())
			dependencies.push_back(m_source_dependency);

		// Use the resolver-backed include handler when available so every include contributes a dependency to the cache entry.
		auto include_handler = m_includes;
		if (m_include_resolver != nullptr)
			include_handler = D3DPtr<IDxcIncludeHandler>(new ResolverIncludeHandler(*m_include_resolver, dependencies), false);

		// Compile the shader code.
		m_result = nullptr;
		auto hr = m_compiler->Compile(&m_source, m_args.data(), s_cast<uint32_t>(m_args.size()), include_handler.get(), __uuidof(IDxcResult), (void**)m_result.address_of());
		if (SUCCEEDED(hr))
		{
			Check(m_result->GetStatus(&hr));
		}
		if (FAILED(hr))
		{
			// Surface DXC's diagnostic buffer as the exception message because HRESULTs alone do not identify HLSL syntax or include failures.
			std::string message = "Compile Failed";
			if (m_result)
			{
				D3DPtr<IDxcBlobEncoding> errors_blob;
				if (SUCCEEDED(m_result->GetErrorBuffer(errors_blob.address_of())))
				{
					std::string error(static_cast<char const*>(errors_blob->GetBufferPointer()), errors_blob->GetBufferSize());
					message.append(": ").append(error);
				}
			}
			Check(hr, message.c_str());
		}

		// Copy the compiled bytecode out of DXC-owned memory so callers and cache entries do not depend on IDxcResult lifetime.
		D3DPtr<IDxcBlob> shader;
		Check(m_result->GetResult(shader.address_of()));
		std::vector<uint8_t> byte_code(shader->GetBufferSize());
		memcpy(byte_code.data(), shader->GetBufferPointer(), shader->GetBufferSize());

		// Copy optional PDB output to memory for the cache and also write it to the requested directory for PIX/debugger symbol lookup.
		auto pdb_data = std::vector<uint8_t>{};
		auto pdb_filename = std::wstring{};
		if (!m_pdb_dir.empty())
		{
			D3DPtr<IDxcBlob> pdb;
			D3DPtr<IDxcBlobUtf16> pdb_name;
			Check(m_result->GetOutput(DXC_OUT_PDB, __uuidof(IDxcBlob), (void**)pdb.address_of(), pdb_name.address_of()));
			pdb_filename = pdb_name->GetStringPointer();
			pdb_data.resize(pdb->GetBufferSize());
			memcpy(pdb_data.data(), pdb->GetBufferPointer(), pdb->GetBufferSize());

			auto pdb_path = m_pdb_dir / pdb_filename;
			std::ofstream file(pdb_path, std::ios::binary);
			file.write(reinterpret_cast<char const*>(pdb_data.data()), static_cast<std::streamsize>(pdb_data.size()));
		}

		// Return both the compiled payload and the dependency list discovered during source/include resolution.
		return CompileResult{
			.m_output = CompileOutput{
				.m_byte_code = std::move(byte_code),
				.m_pdb = std::move(pdb_data),
				.m_pdb_name = std::move(pdb_filename),
			},
			.m_dependencies = std::move(dependencies),
		};
	}

	// Compile the current shader configuration, using the shader cache when all cache requirements are satisfied.
	std::vector<uint8_t> ShaderCompiler::Compile()
	{
		#if PR_PIX_ENABLED && PR_COMPUTE_SHADER_DEBUG
		// Force shader symbols and warnings-as-errors for explicit shader debugging builds.
		DebugInfo().Arg(L"-WX").Optimise(false).PDBOutput(std::filesystem::temp_directory_path() / L"Rylogic" / L"Symbols");
		#pragma message(PR_LINK "WARNING: ************************************************** Debug Shader Compiling enabled")
		#endif

		BuildArgs();

		// Keep the actual compile operation in one callable so the cache can run it only after acquiring any required cache lock.
		auto compile = [&]
		{
			return CompileUncached();
		};

		// Cache misses and invalidated hits compile through the same uncached path, preserving dependency collection and PDB materialisation.
		if (Cacheable())
			return m_cache->GetOrCompile(Request(), compile).m_byte_code;

		return compile().m_output.m_byte_code;
	}
}

