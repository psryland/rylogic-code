//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "view3d-12/src/shaders/common.h"

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
			#include PR_RDR_SHADER_COMPILED_DIR(forward_alpha_collect_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(kbuffer_resolve_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(kbuffer_alpha_resolve_ps.h)
		}
		ByteCode const forward_vs(compiled::forward_vs);
		ByteCode const forward_ps(compiled::forward_ps);
		ByteCode const forward_radial_fade_ps(compiled::forward_radial_fade_ps);
		ByteCode const forward_alpha_collect_ps(compiled::forward_alpha_collect_ps);
		ByteCode const kbuffer_resolve_vs(compiled::kbuffer_resolve_vs);
		ByteCode const kbuffer_alpha_resolve_ps(compiled::kbuffer_alpha_resolve_ps);

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
