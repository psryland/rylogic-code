//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/descriptor.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12
{
	inline constexpr auto RayTracingReflectionAttributeFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;

	// Owns the raster side-buffer that identifies the visible opaque surface for RT reflections.
	struct RayTracingReflectionBuffer
	{
	private:

		D3DPtr<ID3D12Resource> m_attributes;
		Descriptor m_srv;
		Descriptor m_rtv;
		iv2 m_size;
		MultiSamp m_multisamp;

	public:

		RayTracingReflectionBuffer();
		RayTracingReflectionBuffer(RayTracingReflectionBuffer&& rhs) = delete;
		RayTracingReflectionBuffer(RayTracingReflectionBuffer const&) = delete;
		RayTracingReflectionBuffer& operator =(RayTracingReflectionBuffer&& rhs) = delete;
		RayTracingReflectionBuffer& operator =(RayTracingReflectionBuffer const&) = delete;
		~RayTracingReflectionBuffer();

		// Release GPU resources after deferring GPU lifetime management through the renderer.
		void DeferRelease(Renderer& rdr);

		// Create or resize the reflection attribute buffer for the current render target.
		bool Prepare(ResourceFactory& factory, iv2 size, MultiSamp multisamp);

		// Clear the reflection attributes ready for the next opaque pass.
		void Clear(GfxCmdList& cmd_list) const;

		// True if the buffer exists and matches the requested render target shape.
		bool Matches(iv2 size, MultiSamp multisamp) const;

		// True if the buffer can be used by the forward and RT passes.
		explicit operator bool() const;

		// Access the reflection attribute resource.
		ID3D12Resource* Attributes() const;

		// Access the CPU descriptor for rendering attributes in the forward opaque pass.
		D3D12_CPU_DESCRIPTOR_HANDLE RTV() const;

		// Access the CPU descriptor for binding attributes in the RT pass.
		Descriptor const& SRV() const;
	};
}
