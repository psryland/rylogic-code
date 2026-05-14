//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/shaders/compiler/shader_compiler.h"
#pragma comment(lib, "dxcompiler.lib")

namespace pr::rdr12
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

	// Create a DXC compiler wrapper and the default include handler used when no cache-aware resolver is supplied.
	ShaderCompiler::ShaderCompiler(IShaderCache* cache)
		: m_result()
		, m_compiler()
		, m_source_blob()
		, m_includes()
		, m_cache(cache)
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

	// Set the shader cache that should be consulted by Compile().
	ShaderCompiler& ShaderCompiler::Cache(IShaderCache* cache)
	{
		m_cache = cache;
		return *this;
	}

	// Use a file as the root source and resolve relative includes from the file's directory.
	ShaderCompiler& ShaderCompiler::File(std::filesystem::path file)
	{
		auto path = CanonicalPath(std::move(file));
		m_owned_resolver = std::make_unique<FileSourceResolver>(path.parent_path());

		// FileSourceResolver returns both the source text and the file metadata needed to validate cached output later.
		SetSource(m_owned_resolver->Resolve(Narrow(path.wstring())));
		m_include_resolver = m_owned_resolver.get();
		m_includes = nullptr;
		return *this;
	}

	// Use anonymous UTF-8 source text. Anonymous source is compilable but not cacheable because it has no stable identity.
	ShaderCompiler& ShaderCompiler::Source(std::string_view code)
	{
		m_source_blob = nullptr;
		m_source_code.assign(code);
		m_source_identity.clear();
		m_source_dependency = {};
		m_source = DxcBuffer {
			.Ptr = m_source_code.data(),
			.Size = m_source_code.size(),
			.Encoding = DXC_CP_UTF8,
		};
		return *this;
	}

	// Use UTF-8 source text with a stable cache identity.
	ShaderCompiler& ShaderCompiler::Source(std::string_view code, std::string_view identity)
	{
		Source(code);
		if (!identity.empty())
		{
			// Hash the current source text so cache entries are invalidated when the caller supplies changed text under the same identity.
			m_source_identity = identity;
			m_source_dependency = SourceTextDependency(m_source_identity, m_source_code);
		}
		return *this;
	}

	// Use a caller-provided resolver for the root source and all includes.
	ShaderCompiler& ShaderCompiler::Source(std::string_view source_name, ISourceResolver const& resolver)
	{
		SetSource(resolver.Resolve(source_name));
		m_include_resolver = &resolver;
		m_includes = nullptr;
		return *this;
	}

	// Use a raw DXC include handler. This disables include dependency tracking because the handler has no cache-key contract.
	ShaderCompiler& ShaderCompiler::Includes(D3DPtr<IDxcIncludeHandler> handler)
	{
		m_includes = handler;
		m_include_resolver = nullptr;
		return *this;
	}

	// Use a cache-aware include resolver for include lookup.
	ShaderCompiler& ShaderCompiler::Includes(ISourceResolver const& resolver)
	{
		m_include_resolver = &resolver;
		m_includes = nullptr;
		return *this;
	}

	// Set the HLSL entry point passed to DXC.
	ShaderCompiler& ShaderCompiler::EntryPoint(std::wstring_view ep)
	{
		m_ep.clear();
		m_ep.append(L"-E").append(ep);
		return *this;
	}

	// Set the shader model target passed to DXC.
	ShaderCompiler& ShaderCompiler::ShaderModel(std::wstring_view sm)
	{
		m_sm.clear();
		m_sm.append(L"-T").append(sm);
		return *this;
	}

	// Select the HLSL language version requested from DXC.
	ShaderCompiler& ShaderCompiler::HlslVersion(EHlslVersion version)
	{
		m_hlsl_version = version;
		return *this;
	}

	// Enable or disable DXC optimisation.
	ShaderCompiler& ShaderCompiler::Optimise(bool opt)
	{
		m_optimise = opt;
		return *this;
	}

	// Enable or disable PDB/debug-info emission in the compiled shader.
	ShaderCompiler& ShaderCompiler::DebugInfo(bool dbg)
	{
		m_debug_info = dbg;
		return *this;
	}

	// Add or replace a preprocessor definition passed to DXC.
	ShaderCompiler& ShaderCompiler::Define(std::wstring_view sym, std::wstring_view value)
	{
		m_defines[std::wstring(sym)] = !value.empty()
			? std::format(L"-D{}={}", sym, value)
			: std::format(L"-D{}", sym);
		return *this;
	}

	// Request PDB output in 'dir', optionally using 'filename' as the cache-stable name hint.
	ShaderCompiler& ShaderCompiler::PDBOutput(std::filesystem::path dir, std::string_view filename)
	{
		m_pdb_dir = std::move(dir);
		m_pdb_name_hint = Widen(filename);
		return *this;
	}

	// Append a raw DXC argument after the generated arguments.
	ShaderCompiler& ShaderCompiler::Arg(std::wstring_view arg)
	{
		m_extras.push_back(std::wstring(arg));
		return *this;
	}

	// Replace the root source and update the DxcBuffer to point at owned UTF-8 storage.
	void ShaderCompiler::SetSource(shader_cache::Source source)
	{
		m_source_blob = nullptr;
		m_source_code = std::move(source.m_code);
		m_source_identity = std::move(source.m_identity);
		m_source_dependency = std::move(source.m_dependency);
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
