//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "view3d-12/src/shaders/common.h"
#pragma comment(lib, "dxcompiler.lib")

namespace pr::rdr12
{
	Shader::Shader(Renderer& rdr)
		: RefCounted<Shader>()
		, m_rdr(&rdr)
		, m_code()
		, m_signature()
	{
	}

	// Renderer access
	Renderer const& Shader::rdr() const
	{
		return *m_rdr;
	}
	Renderer& Shader::rdr()
	{
		return *m_rdr;
	}
		
	// Sort id for the shader
	SortKeyId Shader::SortId() const
	{
		// Hash all of the ByteCode pointers together for the sort id.
		return SortKeyId(hash::HashBytes32(&m_code, &m_code + 1) % SortKey::MaxShaderId);
	}

	// Ref counting clean up function
	void Shader::RefCountZero(RefCounted<Shader>* doomed)
	{
		auto shdr = static_cast<Shader*>(doomed);
		shdr->rdr().DeferRelease(shdr->m_signature);
		shdr->Delete();
	}
	void Shader::Delete()
	{
		rdr12::Delete<Shader>(this);
	}

	// Runtime shader compiler
	ShaderCompiler::ShaderCompiler()
		: m_result()
		, m_compiler()
		, m_source_blob()
		, m_includes()
		, m_pdb_path()
		, m_source()
		, m_defines()
		, m_ep()
		, m_sm()
		, m_optimise()
		, m_debug_info()
		, m_extras()
		, m_args()
	{
		Check(DxcCreateInstance(CLSID_DxcCompiler, __uuidof(IDxcCompiler3), (void**)m_compiler.address_of()));

		D3DPtr<IDxcUtils> utils;
		Check(DxcCreateInstance(CLSID_DxcUtils, __uuidof(IDxcUtils), (void**)utils.address_of()));
		Check(utils->CreateDefaultIncludeHandler(m_includes.address_of()));
	}
	ShaderCompiler& ShaderCompiler::File(std::filesystem::path file)
	{		
		D3DPtr<IDxcUtils> utils;
		Check(DxcCreateInstance(CLSID_DxcUtils, __uuidof(IDxcUtils), (void**)utils.address_of()));

		m_source_blob = nullptr;
		uint32_t code_page = DXC_CP_UTF8;
		Check(utils->LoadFile(file.wstring().c_str(), &code_page, m_source_blob.address_of()));

		Source({ static_cast<char const*>(m_source_blob->GetBufferPointer()), m_source_blob->GetBufferSize() });
		return *this;
	}
	ShaderCompiler& ShaderCompiler::Source(std::string_view code)
	{
		m_source = DxcBuffer {
			.Ptr = code.data(),
			.Size = code.size(),
			.Encoding = DXC_CP_UTF8,
		};
		return *this;
	}
	ShaderCompiler& ShaderCompiler::Includes(D3DPtr<IDxcIncludeHandler> handler)
	{
		m_includes = handler;
		return *this;
	}
	ShaderCompiler& ShaderCompiler::EntryPoint(std::wstring_view ep)
	{
		m_ep.clear();
		m_ep.append(L"-E").append(ep);
		return *this;
	}
	ShaderCompiler& ShaderCompiler::ShaderModel(std::wstring_view sm)
	{
		m_sm.clear();
		m_sm.append(L"-T").append(sm);
		return *this;
	}
	ShaderCompiler& ShaderCompiler::Optimise(bool opt)
	{
		m_optimise = opt;
		return *this;
	}
	ShaderCompiler& ShaderCompiler::DebugInfo(bool dbg)
	{
		m_debug_info = dbg;
		return *this;
	}
	ShaderCompiler& ShaderCompiler::Define(std::wstring_view sym, std::wstring_view value)
	{
		m_defines[std::wstring(sym)] = !value.empty()
			? std::format(L"-D{}={}", sym, value)
			: std::format(L"-D{}", sym);
		return *this;
	}
	ShaderCompiler& ShaderCompiler::PDBOutput(std::filesystem::path dir, std::string_view filename)
	{
		m_pdb_path = dir / filename;
		return *this;
	}
	ShaderCompiler& ShaderCompiler::Arg(std::wstring_view arg)
	{
		m_extras.push_back(std::wstring(arg));
		return *this;
	}
	std::vector<uint8_t> ShaderCompiler::Compile()
	{
		#if _DEBUG
		DebugInfo().Arg(L"-WX").Optimise(false).PDBOutput(L"E:\\Dump\\Symbols");
		#pragma message(PR_LINK "WARNING: ************************************************** Debug Shader Compiling enabled")
		#endif

		m_args.clear();
		m_args.push_back(m_ep.c_str());
		m_args.push_back(m_sm.c_str());
		m_args.push_back(m_optimise ? L"-O3" : L"-Od");
		if (m_debug_info)
		{
			m_args.push_back(L"-Zi");
		}
		for (auto& def : m_defines)
		{
			m_args.push_back(def.second.c_str());
		}
		if (!m_pdb_path.empty())
		{
			m_args.push_back(L"-Fd");
			m_args.push_back(m_pdb_path.c_str());
		}
		for (auto& extra : m_extras)
		{
			m_args.push_back(extra.c_str());
		}

		// Compile the shader code
		m_result = nullptr;
		auto hr = m_compiler->Compile(&m_source, m_args.data(), s_cast<uint32_t>(m_args.size()), m_includes.get(), __uuidof(IDxcResult), (void**)m_result.address_of());
		if (SUCCEEDED(hr))
		{
			Check(m_result->GetStatus(&hr));
		}
		if (FAILED(hr))
		{
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

		// Get the compiled shader code
		D3DPtr<IDxcBlob> shader;
		Check(m_result->GetResult(shader.address_of()));
		std::vector<uint8_t> byte_code(shader->GetBufferSize());
		memcpy(byte_code.data(), shader->GetBufferPointer(), shader->GetBufferSize());

		// Output the pdb file
		if (!m_pdb_path.empty())
		{
			D3DPtr<IDxcBlob> pdb;
			D3DPtr<IDxcBlobUtf16> pdb_name;
			Check(m_result->GetOutput(DXC_OUT_PDB, __uuidof(IDxcBlob), (void**)pdb.address_of(), pdb_name.address_of()));
			auto pdb_path = m_pdb_path / pdb_name->GetStringPointer();

			std::ofstream file(pdb_path, std::ios::binary);
			file.write(static_cast<char const*>(pdb->GetBufferPointer()), pdb->GetBufferSize());
		}

		return byte_code;
	}

	// Compiled shader byte code
	namespace shader_code
	{
		// Not a shader
		ByteCode const none;

		// Forward rendering shaders
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(forward_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_radial_fade_ps.h)
		}
		ByteCode const forward_vs(compiled::forward_vs);
		ByteCode const forward_ps(compiled::forward_ps);
		ByteCode const forward_radial_fade_ps(compiled::forward_radial_fade_ps);

		// Deferred rendering
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(gbuffer_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(gbuffer_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(dslighting_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(dslighting_ps.h)
		}
		ByteCode const gbuffer_vs(compiled::gbuffer_vs);
		ByteCode const gbuffer_ps(compiled::gbuffer_ps);
		ByteCode const dslighting_vs(compiled::dslighting_vs);
		ByteCode const dslighting_ps(compiled::dslighting_ps);

		// Shadows
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(shadow_map_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(shadow_map_ps.h)
		}
		ByteCode const shadow_map_vs(compiled::shadow_map_vs);
		ByteCode const shadow_map_ps(compiled::shadow_map_ps);

		// Screen Space
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(point_sprites_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(thick_line_list_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(thick_line_strip_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(arrow_head_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(show_normals_gs.h)
		}
		ByteCode const point_sprites_gs(compiled::point_sprites_gs);
		ByteCode const thick_line_list_gs(compiled::thick_line_list_gs);
		ByteCode const thick_line_strip_gs(compiled::thick_line_strip_gs);
		ByteCode const arrow_head_gs(compiled::arrow_head_gs);
		ByteCode const show_normals_gs(compiled::show_normals_gs);

		// Ray cast
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_vert_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_edge_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_face_gs.h)
		}
		ByteCode const ray_cast_vs(compiled::ray_cast_vs);
		ByteCode const ray_cast_vert_gs(compiled::ray_cast_vert_gs);
		ByteCode const ray_cast_edge_gs(compiled::ray_cast_edge_gs);
		ByteCode const ray_cast_face_gs(compiled::ray_cast_face_gs);

		// MipMap generation
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(mipmap_generator_cs.h)
		}
		ByteCode const mipmap_generator_cs(compiled::mipmap_generator_cs);
	}
}