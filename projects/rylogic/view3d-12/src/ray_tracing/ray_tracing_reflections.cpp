//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_reflections.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_resource.h"
#include "pr/view3d-12/resource/resource_store.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	// Create an empty reflection attribute buffer.
	RayTracingReflectionBuffer::RayTracingReflectionBuffer()
		: m_attributes()
		, m_srv()
		, m_rtv()
		, m_size(iv2::Zero())
		, m_multisamp()
	{}

	// Destroy reflection resources owned by this wrapper.
	RayTracingReflectionBuffer::~RayTracingReflectionBuffer() = default;

	// Release GPU resources after deferring GPU lifetime management through the renderer.
	void RayTracingReflectionBuffer::DeferRelease(Renderer& rdr)
	{
		if (m_srv || m_rtv)
		{
			ResourceStore::Access store(rdr);
			if (m_srv)
				store.Descriptors().Release(m_srv);
			if (m_rtv)
				store.Descriptors().Release(m_rtv);
		}

		rdr.DeferRelease(m_attributes);
		m_srv = {};
		m_rtv = {};
		m_size = iv2::Zero();
		m_multisamp = MultiSamp();
	}

	// Create or resize the reflection attribute buffer for the current render target.
	bool RayTracingReflectionBuffer::Prepare(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, iv2 size, MultiSamp multisamp)
	{
		if (size.x <= 0 || size.y <= 0)
			return false;
		if (Matches(size, multisamp))
			return true;

		DeferRelease(rdr);

		auto clear = ClearValue(RayTracingReflectionAttributeFormat, Colour(0.0f, 0.0f, 0.0f, 0.0f));
		auto desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, RayTracingReflectionAttributeFormat }, 1U, EUsage::RenderTarget)
			.multisamp(multisamp)
			.clear(clear)
			.def_state(D3D12_RESOURCE_STATE_RENDER_TARGET);

		m_attributes = CreateRayTracingResource(rdr, cmd_list, upload, desc, "RT-ReflectionAttributes");
		m_size = size;
		m_multisamp = multisamp;

		auto srv_desc = D3D12_SHADER_RESOURCE_VIEW_DESC{
			.Format = RayTracingReflectionAttributeFormat,
			.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DMS,
			.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
		};
		auto rtv_desc = D3D12_RENDER_TARGET_VIEW_DESC{
			.Format = RayTracingReflectionAttributeFormat,
			.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2DMS,
		};

		ResourceStore::Access store(rdr);
		m_srv = store.Descriptors().Create(m_attributes.get(), srv_desc);
		m_rtv = store.Descriptors().Create(m_attributes.get(), rtv_desc);
		return true;
	}

	// Clear the reflection attributes ready for the next opaque pass.
	void RayTracingReflectionBuffer::Clear(GfxCmdList& cmd_list) const
	{
		if (!*this)
			return;

		auto clear = float4_t{ 0, 0, 0, 0 };
		cmd_list.ClearRenderTargetView(m_rtv.m_cpu, clear);
	}

	// True if the buffer exists and matches the requested render target shape.
	bool RayTracingReflectionBuffer::Matches(iv2 size, MultiSamp multisamp) const
	{
		return m_attributes != nullptr && All(m_size == size) && m_multisamp == multisamp;
	}

	// True if the buffer can be used by the forward and RT passes.
	RayTracingReflectionBuffer::operator bool() const
	{
		return m_attributes != nullptr && m_srv && m_rtv;
	}

	// Access the reflection attribute resource.
	ID3D12Resource* RayTracingReflectionBuffer::Attributes() const
	{
		return const_cast<ID3D12Resource*>(m_attributes.get());
	}

	// Access the CPU descriptor for rendering attributes in the forward opaque pass.
	D3D12_CPU_DESCRIPTOR_HANDLE RayTracingReflectionBuffer::RTV() const
	{
		return m_rtv.m_cpu;
	}

	// Access the CPU descriptor for binding attributes in the RT pass.
	Descriptor const& RayTracingReflectionBuffer::SRV() const
	{
		return m_srv;
	}
}
