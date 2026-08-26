//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "renderer.h"

// D3DCompile (runtime HLSL compilation) is only needed here, so it is linked exactly where it is
// used rather than from every DLL translation unit; see text_shaper.cpp for the same precedent
// with dwrite.lib.
#include <d3dcompiler.h>
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "d3dcompiler.lib")

namespace pr::view3d::ui
{
	namespace
	{
		// One square atlas page holds 512*512 = 256KiB of single-channel coverage; 512 bytes/row
		// is already a multiple of D3D12_TEXTURE_DATA_PITCH_ALIGNMENT (256), so no per-row upload
		// padding is ever needed.
		constexpr std::uint32_t kAtlasPageDim = 512;

		// Every root-constants payload is exactly eight float4 groups (32 DWORDs), deliberately
		// avoiding any scalar/float2 field: HLSL cbuffer packing pads scalars/float2s to avoid
		// straddling 16-byte boundaries, and reasoning about that padding by hand is error-prone,
		// whereas float4-only layouts are always 16-byte aligned/sized on both sides with zero
		// ambiguity. Box draws populate g0..g3 (+g4.x); glyph draws populate g0/g1.xy/g2/g4/g5;
		// g6/g7 carry the per-group world-root depth and occlusion-fade parameters and are read
		// only by the occlusion-faded pixel shaders. Which fields apply is determined entirely by
		// which PSO is bound, not by any field here.
		//
		// g0 carries the D3D viewport extent, never the render-target extent, and every rect
		// position is local to that viewport. The render-target extent is deliberately absent:
		// the only stage that needs it is the resolved-depth lookup, and that addresses the
		// texture through SV_POSITION, which the hardware has already expressed in target space.
		struct RootConstants
		{
			float g0[4]; // viewport_w_px, viewport_h_px, rect_x_px, rect_y_px (all viewport-local)
			float g1[4]; // rect_w_px, rect_h_px, border_thickness_px, corner_radius_px
			float g2[4]; // fill_r, fill_g, fill_b, fill_a
			float g3[4]; // border_r, border_g, border_b, border_a
			float g4[4]; // opacity, uv0_x, uv0_y, uv1_x
			float g5[4]; // uv1_y, page, clip_depth, unused
			float g6[4]; // ui_view_depth, occlusion_min_opacity, occlusion_fade_depth, occlusion_depth_bias
			float g7[4]; // near_plane, far_plane, orthographic, unused
		};
		constexpr UINT kRootConstantCount = 32;
		static_assert(sizeof(RootConstants) == kRootConstantCount * sizeof(float));

		// Text-decoration geometry. A caret is one DIP wide so it stays visible without covering
		// the glyph it precedes; a composition underline is two DIPs so it reads as provisional
		// text at typical UI sizes. 'kMaxDecorationRects' bounds the rectangles one selection or
		// composition range can produce - a bidirectional range is split into several visually
		// contiguous runs, so the count is not simply one.
		constexpr float kCaretWidthDip = 1.0f;
		constexpr float kCompositionUnderlineDip = 2.0f;
		constexpr std::uint32_t kMaxDecorationRects = 32;

		// Selection highlight colour, fixed rather than style-driven: the style vocabulary
		// (types.h) has no selection channel, and inventing one would be a public ABI change.
		constexpr Colour kSelectionFill{ 0.24f, 0.47f, 0.85f, 0.45f };

		// Vertex shader emits a unit quad from SV_VertexID alone (no vertex/index buffer is ever
		// bound); both pixel shaders consume only the six float4 groups their own draw populated.
		// PSBox renders solid/rounded/border boxes via two nested rounded-box SDF evaluations (one
		// for the outer anti-aliased silhouette, one for the fill-vs-border boundary); PSGlyph
		// samples one page of the glyph atlas and modulates it by the item's fill colour/opacity.
		// PSBoxFaded/PSGlyphFaded add the occlusion fade: they read the host's resolved single-
		// sample depth at their own pixel, convert it to a linear view depth, and scale opacity
		// down towards the group's floor as scene geometry moves in front of the root's anchor.
		// All outputs are premultiplied (colour already multiplied by coverage*alpha*opacity) to
		// match the renderer's fixed premultiplied source-over blend state.
		constexpr char const* kShaderSource = R"hlsl(
cbuffer RootConstants : register(b0)
{
	float4 g0;
	float4 g1;
	float4 g2;
	float4 g3;
	float4 g4;
	float4 g5;
	float4 g6;
	float4 g7;
};

Texture2DArray<float> glyph_atlas : register(t0);
Texture2D<float> scene_depth : register(t1);
SamplerState glyph_sampler : register(s0);

struct VSOut
{
	float4 pos : SV_POSITION;
	float2 local : TEXCOORD0;
};

VSOut VSMain(uint vid : SV_VertexID)
{
	// 'rect_pos' is local to the D3D viewport, so the divisor is the viewport extent. The
	// viewport transform applied after this shader adds the viewport's origin and extent once.
	float2 viewport_size = g0.xy;
	float2 rect_pos = g0.zw;
	float2 rect_size = g1.xy;
	float clip_depth = g5.z;

	float2 corner = float2(float(vid & 1u), float((vid >> 1u) & 1u));
	float2 px = rect_pos + corner * rect_size;
	float2 ndc = float2(px.x / viewport_size.x * 2.0 - 1.0, 1.0 - px.y / viewport_size.y * 2.0);

	VSOut o;
	o.pos = float4(ndc, clip_depth, 1.0);
	o.local = corner;
	return o;
}

float RoundedBoxSdf(float2 p, float2 half_extent, float radius)
{
	float2 q = abs(p) - half_extent + radius;
	return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - radius;
}

float BoxCoverage(VSOut i, out float4 straight)
{
	float2 rect_size = g1.xy;
	float border_thickness = g1.z;
	float corner_radius = g1.w;
	float4 fill_colour = g2;
	float4 border_colour = g3;

	float2 half_extent = rect_size * 0.5;
	float2 p = i.local * rect_size - half_extent;
	float r = min(corner_radius, min(half_extent.x, half_extent.y));

	// A ~1px-wide anti-aliased band straddling the true SDF boundary (d == 0), rather than a hard
	// hi/lo cut, so box edges do not alias at arbitrary DPI scales.
	float d_outer = RoundedBoxSdf(p, half_extent, r);
	float outer_coverage = saturate(0.5 - d_outer);

	float2 inner_extent = max(half_extent - border_thickness, 0.0);
	float inner_r = max(r - border_thickness, 0.0);
	float d_inner = RoundedBoxSdf(p, inner_extent, inner_r);
	float fill_coverage = saturate(0.5 - d_inner);

	straight = border_thickness > 0.0 ? lerp(border_colour, fill_colour, fill_coverage) : fill_colour;
	return outer_coverage;
}

// Convert a normalised device depth back to a linear distance in front of the camera, matching
// View3D's right-handed projection. A sample still at the far plane means nothing was drawn there,
// which must never be treated as an occluder.
float LinearViewDepth(float ndc_z)
{
	float zn = g7.x;
	float zf = g7.y;
	bool orthographic = g7.z != 0.0;
	if (orthographic)
		return zn + ndc_z * (zf - zn);

	float denom = zf - ndc_z * (zf - zn);
	return denom > 0.0 ? (zn * zf) / denom : zf;
}

// Opacity multiplier for one pixel of an occlusion-faded root: 1 while the nearest scene sample is
// still behind the root's anchor, falling linearly to the group's floor as geometry moves in front
// of it by 'fade_depth' world units. 'pixel_xy' is SV_POSITION, which the viewport transform has
// already expressed in render-target pixels, so the resolved depth is addressed directly by texel
// and must span the whole target; it is deliberately not renormalised by the viewport extent.
float OcclusionFade(float2 pixel_xy)
{
	float ui_view_depth = g6.x;
	float min_opacity = g6.y;
	float fade_depth = g6.z;
	float depth_bias = g6.w;

	int3 texel = int3((int)pixel_xy.x, (int)pixel_xy.y, 0);
	float scene_view_depth = LinearViewDepth(scene_depth.Load(texel));

	// Positive when the scene is behind the root, negative when it occludes it.
	float behind = scene_view_depth - (ui_view_depth - depth_bias);
	float t = fade_depth > 0.0 ? saturate(behind / fade_depth + 1.0) : (behind >= 0.0 ? 1.0 : 0.0);
	return lerp(min_opacity, 1.0, t);
}

float4 PSBox(VSOut i) : SV_TARGET
{
	float4 straight;
	float coverage = BoxCoverage(i, straight) * g4.x;
	return float4(straight.rgb * straight.a * coverage, straight.a * coverage);
}

float4 PSBoxFaded(VSOut i) : SV_TARGET
{
	float4 straight;
	float coverage = BoxCoverage(i, straight) * g4.x * OcclusionFade(i.pos.xy);
	return float4(straight.rgb * straight.a * coverage, straight.a * coverage);
}

float GlyphCoverage(VSOut i)
{
	float2 uv0 = g4.yz;
	float2 uv1 = float2(g4.w, g5.x);
	float page = g5.y;

	float2 uv = lerp(uv0, uv1, i.local);
	return glyph_atlas.Sample(glyph_sampler, float3(uv, page));
}

float4 PSGlyph(VSOut i) : SV_TARGET
{
	float4 fill_colour = g2;
	float alpha = GlyphCoverage(i) * fill_colour.a * g4.x;
	return float4(fill_colour.rgb * alpha, alpha);
}

float4 PSGlyphFaded(VSOut i) : SV_TARGET
{
	float4 fill_colour = g2;
	float alpha = GlyphCoverage(i) * fill_colour.a * g4.x * OcclusionFade(i.pos.xy);
	return float4(fill_colour.rgb * alpha, alpha);
}
)hlsl";

		// Returns a raw void** suitable for a COM creation method's final out-parameter, without
		// requiring the IID_PPV_ARGS macro (not guaranteed available under this module's
		// dependency-minimal <unknwn.h>-only COM include set; see text_shaper.cpp for the same
		// manual-QueryInterface precedent).
		template <typename T>
		void** Ppv(Microsoft::WRL::ComPtr<T>& ptr)
		{
			return reinterpret_cast<void**>(ptr.ReleaseAndGetAddressOf());
		}

		// Compiles one HLSL entry point from kShaderSource, throwing with the compiler's own
		// diagnostic text on failure so a shader authoring mistake is immediately actionable.
		Microsoft::WRL::ComPtr<ID3DBlob> CompileShader(char const* entry_point, char const* target)
		{
			Microsoft::WRL::ComPtr<ID3DBlob> code;
			Microsoft::WRL::ComPtr<ID3DBlob> errors;
			auto hr = ::D3DCompile(kShaderSource, std::strlen(kShaderSource), nullptr, nullptr, nullptr, entry_point, target, 0, 0, code.GetAddressOf(), errors.GetAddressOf());
			if (FAILED(hr))
			{
				auto message = std::string("Renderer: HLSL compile failed for ") + entry_point;
				if (errors != nullptr)
					message += std::string(": ") + static_cast<char const*>(errors->GetBufferPointer());
				throw EngineException(EStatus::InternalError, message);
			}
			return code;
		}
	}

	Renderer::Renderer(IUnknown* device, Config const& config) noexcept
		: m_device(device)
		, m_config(config)
	{
	}

	Renderer::~Renderer()
	{
		// Map()/Unmap() must be paired; every persistent upload buffer was mapped once in
		// EnsureGlyphUploadBuffer and is unmapped here, immediately before its ComPtr releases the
		// underlying resource.
		for (auto& upload : m_glyph_uploads)
		{
			if (upload != nullptr)
				upload->Unmap(0, nullptr);
		}
	}

	bool Renderer::CheckDeviceLost()
	{
		if (m_d3d_device->GetDeviceRemovedReason() != S_OK)
			m_device_lost = true;
		return m_device_lost;
	}

	void Renderer::EnsureDeviceResources(DXGI_FORMAT colour_format)
	{
		if (m_d3d_device != nullptr)
			return; // already realised by an earlier Prepare call this session

		// Every fallible step below operates on purely local variables; nothing is assigned to
		// *this until every single one has succeeded (see the declaration comment in renderer.h).
		Microsoft::WRL::ComPtr<ID3D12Device> device;
		auto hr = m_device->QueryInterface(__uuidof(ID3D12Device), Ppv(device));
		if (FAILED(hr))
			throw EngineException(EStatus::InternalError, "Renderer: QueryInterface<ID3D12Device> failed");

		auto text_shaper = std::make_unique<TextShaper>();

		auto const atlas_dim_px = kAtlasPageDim;
		auto glyph_cache = std::make_unique<GlyphCache>(atlas_dim_px, m_config.max_glyph_cache_pages, static_cast<std::uint64_t>(m_config.max_glyph_cache_bytes));
		auto const pages_capacity = std::max<std::uint32_t>(1, glyph_cache->MaxPages());

		auto root_signature = CreateRootSignature(device.Get());
		auto shaders = CompileShaders();

		// The two single-sample, depth-free variants cover every overlay stage and are created up
		// front; the depth-tested variant needs a scene target description no Prepare pass carries.
		auto pso_overlay = CreatePipelineVariant(device.Get(), root_signature.Get(), shaders, colour_format, DXGI_FORMAT_UNKNOWN, 1, 0, false);
		auto pso_faded = CreatePipelineVariant(device.Get(), root_signature.Get(), shaders, colour_format, DXGI_FORMAT_UNKNOWN, 1, 0, true);

		Microsoft::WRL::ComPtr<ID3D12Resource> glyph_atlas;
		Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> srv_heap;
		CreateGlyphAtlasResources(device.Get(), atlas_dim_px, pages_capacity, glyph_atlas, srv_heap);

		// Every step above succeeded; commit as one atomic group of moves/copies, and only now
		// does 'm_d3d_device != nullptr' become true for every later EnsureDeviceResources/Draw
		// early-return check.
		m_text_shaper = std::move(text_shaper);
		m_glyph_cache = std::move(glyph_cache);
		m_glyph_atlas_dim_px = atlas_dim_px;
		m_glyph_atlas_pages_capacity = pages_capacity;
		m_root_signature = std::move(root_signature);
		m_shaders = std::move(shaders);
		m_pso_overlay = std::move(pso_overlay);
		m_pso_faded = std::move(pso_faded);
		m_colour_format = colour_format;
		m_glyph_atlas = std::move(glyph_atlas);
		m_srv_heap = std::move(srv_heap);
		m_glyph_uploads.resize(pages_capacity);
		m_glyph_upload_mapped.assign(pages_capacity, nullptr);
		m_d3d_device = std::move(device);
	}

	Microsoft::WRL::ComPtr<ID3D12RootSignature> Renderer::CreateRootSignature(ID3D12Device* device)
	{
		// One descriptor table (the glyph atlas SRV at t0 and the host's resolved scene depth at
		// t1) plus one inline 32-bit-constants parameter (32 DWORDs, well within the 64-DWORD
		// root-argument budget) and one static linear-clamp sampler; no descriptor heap management
		// is needed for the sampler itself.
		D3D12_DESCRIPTOR_RANGE srv_ranges[2]{};
		srv_ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		srv_ranges[0].NumDescriptors = 1;
		srv_ranges[0].BaseShaderRegister = 0;
		srv_ranges[0].RegisterSpace = 0;
		srv_ranges[0].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		srv_ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		srv_ranges[1].NumDescriptors = 1;
		srv_ranges[1].BaseShaderRegister = 1;
		srv_ranges[1].RegisterSpace = 0;
		srv_ranges[1].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		D3D12_ROOT_PARAMETER params[2]{};
		params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
		params[0].Constants.ShaderRegister = 0;
		params[0].Constants.RegisterSpace = 0;
		params[0].Constants.Num32BitValues = kRootConstantCount;
		params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
		params[1].DescriptorTable.NumDescriptorRanges = 2;
		params[1].DescriptorTable.pDescriptorRanges = srv_ranges;
		params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

		D3D12_STATIC_SAMPLER_DESC sampler{};
		sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
		sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
		sampler.ShaderRegister = 0;
		sampler.RegisterSpace = 0;
		sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

		D3D12_ROOT_SIGNATURE_DESC desc{};
		desc.NumParameters = 2;
		desc.pParameters = params;
		desc.NumStaticSamplers = 1;
		desc.pStaticSamplers = &sampler;
		desc.Flags = D3D12_ROOT_SIGNATURE_FLAG_NONE; // no input assembler stage at all (SV_VertexID only)

		Microsoft::WRL::ComPtr<ID3DBlob> signature;
		Microsoft::WRL::ComPtr<ID3DBlob> error;
		auto hr = ::D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, signature.GetAddressOf(), error.GetAddressOf());
		if (FAILED(hr))
		{
			auto message = std::string("Renderer: D3D12SerializeRootSignature failed");
			if (error != nullptr)
				message += std::string(": ") + static_cast<char const*>(error->GetBufferPointer());
			throw EngineException(EStatus::InternalError, message);
		}

		Microsoft::WRL::ComPtr<ID3D12RootSignature> root_signature;
		hr = device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), __uuidof(ID3D12RootSignature), Ppv(root_signature));
		if (FAILED(hr))
			throw EngineException(EStatus::InternalError, "Renderer: CreateRootSignature failed");

		return root_signature;
	}

	// Compile every entry point once; each pipeline variant is only a different fixed-function
	// configuration over the same shader bytecode.
	ShaderBlobs Renderer::CompileShaders()
	{
		return ShaderBlobs{
			.vs = CompileShader("VSMain", "vs_5_0"),
			.ps_box = CompileShader("PSBox", "ps_5_0"),
			.ps_glyph = CompileShader("PSGlyph", "ps_5_0"),
			.ps_box_faded = CompileShader("PSBoxFaded", "ps_5_0"),
			.ps_glyph_faded = CompileShader("PSGlyphFaded", "ps_5_0"),
		};
	}

	PipelineVariant Renderer::CreatePipelineVariant(ID3D12Device* device, ID3D12RootSignature* root_signature, ShaderBlobs const& shaders, DXGI_FORMAT colour_format, DXGI_FORMAT depth_format, std::uint32_t sample_count, std::uint32_t sample_quality, bool faded)
	{
		// Premultiplied source-over for every variant: the pixel shaders already output colour
		// pre-multiplied by their own coverage*alpha*opacity, so the fixed-function blend only
		// needs to add the destination's remainder.
		D3D12_BLEND_DESC blend{};
		blend.RenderTarget[0].BlendEnable = TRUE;
		blend.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
		blend.RenderTarget[0].DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
		blend.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
		blend.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
		blend.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA;
		blend.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
		blend.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

		D3D12_RASTERIZER_DESC rasterizer{};
		rasterizer.FillMode = D3D12_FILL_MODE_SOLID;
		rasterizer.CullMode = D3D12_CULL_MODE_NONE;
		rasterizer.DepthClipEnable = TRUE;
		rasterizer.MultisampleEnable = sample_count > 1 ? TRUE : FALSE;

		// Depth-tested world UI reads the scene depth buffer but never writes it, so scene geometry
		// occludes it while overlapping UI still composites in the packet's deterministic order
		// rather than by depth.
		D3D12_DEPTH_STENCIL_DESC depth_stencil{};
		if (depth_format != DXGI_FORMAT_UNKNOWN)
		{
			depth_stencil.DepthEnable = TRUE;
			depth_stencil.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
			depth_stencil.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
		}

		D3D12_GRAPHICS_PIPELINE_STATE_DESC desc{};
		desc.pRootSignature = root_signature;
		desc.VS = { shaders.vs->GetBufferPointer(), shaders.vs->GetBufferSize() };
		desc.BlendState = blend;
		desc.SampleMask = UINT_MAX;
		desc.RasterizerState = rasterizer;
		desc.DepthStencilState = depth_stencil;
		desc.DSVFormat = depth_format;
		desc.InputLayout = { nullptr, 0 }; // no vertex buffer at all; geometry comes purely from SV_VertexID
		desc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
		desc.NumRenderTargets = 1;
		desc.RTVFormats[0] = colour_format;
		desc.SampleDesc.Count = sample_count;
		desc.SampleDesc.Quality = sample_quality;

		auto const* ps_box = faded ? shaders.ps_box_faded.Get() : shaders.ps_box.Get();
		auto const* ps_glyph = faded ? shaders.ps_glyph_faded.Get() : shaders.ps_glyph.Get();

		PipelineVariant variant;
		desc.PS = { const_cast<ID3DBlob*>(ps_box)->GetBufferPointer(), const_cast<ID3DBlob*>(ps_box)->GetBufferSize() };
		if (FAILED(device->CreateGraphicsPipelineState(&desc, __uuidof(ID3D12PipelineState), Ppv(variant.box))))
			throw EngineException(EStatus::InternalError, "Renderer: CreateGraphicsPipelineState(box) failed");

		desc.PS = { const_cast<ID3DBlob*>(ps_glyph)->GetBufferPointer(), const_cast<ID3DBlob*>(ps_glyph)->GetBufferSize() };
		if (FAILED(device->CreateGraphicsPipelineState(&desc, __uuidof(ID3D12PipelineState), Ppv(variant.glyph))))
			throw EngineException(EStatus::InternalError, "Renderer: CreateGraphicsPipelineState(glyph) failed");

		return variant;
	}

	PipelineVariant const& Renderer::EnsureDepthPipeline(Pass const& pass)
	{
		// Reuse the existing pair unless the host's scene target description changed, which only
		// happens across a multi-sampling or depth-format reconfiguration.
		if (m_pso_depth.box != nullptr && m_pso_depth_format == pass.m_depth_format && m_pso_depth_sample_count == pass.m_sample_count && m_pso_depth_sample_quality == pass.m_sample_quality)
			return m_pso_depth;

		m_pso_depth = CreatePipelineVariant(m_d3d_device.Get(), m_root_signature.Get(), m_shaders, m_colour_format, pass.m_depth_format, pass.m_sample_count, pass.m_sample_quality, false);
		m_pso_depth_format = pass.m_depth_format;
		m_pso_depth_sample_count = pass.m_sample_count;
		m_pso_depth_sample_quality = pass.m_sample_quality;
		return m_pso_depth;
	}

	bool Renderer::EnsureResolvedDepthView(Pass const& pass)
	{
		auto const& resolved = pass.m_resolved_depth;
		if (resolved.m_resource == nullptr || resolved.m_srv_format == DXGI_FORMAT_UNKNOWN)
			return false;

		// The fade shaders address the resolved depth with SV_POSITION, which is a render-target
		// pixel coordinate, so the lookup is only defined when the resolved copy spans the whole
		// target. A differently sized buffer is refused rather than sampled at the wrong texels.
		if (resolved.m_width != pass.m_width || resolved.m_height != pass.m_height)
			return false;

		if (m_resolved_depth == resolved.m_resource && m_resolved_depth_format == resolved.m_srv_format && m_resolved_depth_width == resolved.m_width && m_resolved_depth_height == resolved.m_height)
			return true;

		// Invalidate the key before creating the view so a throwing or partially completed
		// recreation can never leave the cache claiming a descriptor it did not write.
		m_resolved_depth = nullptr;
		m_resolved_depth_format = DXGI_FORMAT_UNKNOWN;
		m_resolved_depth_width = 0;
		m_resolved_depth_height = 0;

		// Creating a view is not a resource transition and does not take a reference the host
		// relies on, so the provider may point its own heap at the host's resource. The host
		// guarantees the resource outlives the callback.
		auto const increment = m_d3d_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
		auto handle = m_srv_heap->GetCPUDescriptorHandleForHeapStart();
		handle.ptr += increment;

		D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
		srv.Format = resolved.m_srv_format;
		srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
		srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
		srv.Texture2D.MipLevels = 1;
		m_d3d_device->CreateShaderResourceView(resolved.m_resource, &srv, handle);

		m_resolved_depth = resolved.m_resource;
		m_resolved_depth_format = resolved.m_srv_format;
		m_resolved_depth_width = resolved.m_width;
		m_resolved_depth_height = resolved.m_height;
		++m_resolved_depth_view_epoch;
		return true;
	}

	std::uint64_t Renderer::ResolvedDepthViewEpoch() const
	{
		return m_resolved_depth_view_epoch;
	}

	Vec2 Renderer::NdcDivisorPx(Pass const& pass)
	{
		auto w = pass.m_viewport.Width;
		auto h = pass.m_viewport.Height;
		if (!(w > 0.0f) || !(h > 0.0f))
			return Vec2{ static_cast<float>(pass.m_width), static_cast<float>(pass.m_height) };

		return Vec2{ w, h };
	}

	void Renderer::CreateGlyphAtlasResources(ID3D12Device* device, std::uint32_t atlas_dim_px, std::uint32_t pages_capacity, Microsoft::WRL::ComPtr<ID3D12Resource>& out_atlas, Microsoft::WRL::ComPtr<ID3D12DescriptorHeap>& out_srv_heap)
	{
		// One committed default-heap texture array, one slice per page, sized to this Renderer's
		// fixed page capacity up front; individual pages are only ever written into (never added
		// or resized) as glyphs are first placed by GlyphCache::Acquire.
		D3D12_HEAP_PROPERTIES heap_props{};
		heap_props.Type = D3D12_HEAP_TYPE_DEFAULT;

		D3D12_RESOURCE_DESC atlas_desc{};
		atlas_desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
		atlas_desc.Width = atlas_dim_px;
		atlas_desc.Height = atlas_dim_px;
		atlas_desc.DepthOrArraySize = static_cast<UINT16>(pages_capacity);
		atlas_desc.MipLevels = 1;
		atlas_desc.Format = DXGI_FORMAT_R8_UNORM;
		atlas_desc.SampleDesc.Count = 1;
		atlas_desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;

		auto hr = device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &atlas_desc, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, nullptr, __uuidof(ID3D12Resource), Ppv(out_atlas));
		if (FAILED(hr))
			throw EngineException(EStatus::InternalError, "Renderer: glyph atlas CreateCommittedResource failed");

		D3D12_DESCRIPTOR_HEAP_DESC heap_desc{};
		heap_desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
		heap_desc.NumDescriptors = 2; // slot 0: glyph atlas, slot 1: the host's resolved scene depth
		heap_desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
		if (FAILED(device->CreateDescriptorHeap(&heap_desc, __uuidof(ID3D12DescriptorHeap), Ppv(out_srv_heap))))
			throw EngineException(EStatus::InternalError, "Renderer: glyph atlas SRV heap creation failed");

		auto const increment = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
		auto handle = out_srv_heap->GetCPUDescriptorHandleForHeapStart();

		D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
		srv.Format = DXGI_FORMAT_R8_UNORM;
		srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
		srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
		srv.Texture2DArray.MipLevels = 1;
		srv.Texture2DArray.ArraySize = pages_capacity;
		device->CreateShaderResourceView(out_atlas.Get(), &srv, handle);

		// Slot 1 is initialised as a null descriptor so the whole table is always well-defined,
		// even on a frame where no occlusion-faded root exists and the host offers no resolved
		// depth. A null Texture2D reads as zero, which the fade shader treats as "nothing there".
		handle.ptr += increment;
		D3D12_SHADER_RESOURCE_VIEW_DESC depth_srv{};
		depth_srv.Format = DXGI_FORMAT_R32_FLOAT;
		depth_srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
		depth_srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
		depth_srv.Texture2D.MipLevels = 1;
		device->CreateShaderResourceView(nullptr, &depth_srv, handle);
	}

	void Renderer::EnsureGlyphUploadBuffer(std::uint32_t page)
	{
		if (m_glyph_uploads[page] != nullptr)
			return; // already created and mapped by an earlier glyph placed on this page

		D3D12_HEAP_PROPERTIES heap_props{};
		heap_props.Type = D3D12_HEAP_TYPE_UPLOAD;

		auto const buffer_bytes = static_cast<std::uint64_t>(m_glyph_atlas_dim_px) * m_glyph_atlas_dim_px;
		D3D12_RESOURCE_DESC desc{};
		desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
		desc.Width = buffer_bytes;
		desc.Height = 1;
		desc.DepthOrArraySize = 1;
		desc.MipLevels = 1;
		desc.Format = DXGI_FORMAT_UNKNOWN;
		desc.SampleDesc.Count = 1;
		desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

		Microsoft::WRL::ComPtr<ID3D12Resource> upload;
		auto hr = m_d3d_device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, __uuidof(ID3D12Resource), Ppv(upload));
		if (FAILED(hr))
			throw EngineException(EStatus::InternalError, "Renderer: glyph upload buffer CreateCommittedResource failed");

		void* mapped = nullptr;
		D3D12_RANGE const no_read{ 0, 0 }; // this buffer is never read back on the CPU side
		if (FAILED(upload->Map(0, &no_read, &mapped)))
			throw EngineException(EStatus::InternalError, "Renderer: glyph upload buffer Map failed");

		// Zero the whole page up front so any byte never explicitly written by a placed glyph
		// (the gaps between shelf-packed rectangles) samples as fully transparent rather than
		// whatever the allocator happened to hand back, since every CopyTextureRegion below reads
		// the buffer as one contiguous atlas_dim x atlas_dim footprint.
		std::memset(mapped, 0, static_cast<std::size_t>(buffer_bytes));

		m_glyph_uploads[page] = upload;
		m_glyph_upload_mapped[page] = static_cast<std::uint8_t*>(mapped);
	}

	void Renderer::UploadGlyphBitmap(GlyphSlot const& slot, GlyphBitmap const& bitmap, ID3D12GraphicsCommandList* command_list)
	{
		// Write this glyph's coverage bytes into its permanently-assigned rectangle within the
		// page's 1:1 CPU mirror; every other byte in the mirror was already zeroed once, in
		// EnsureGlyphUploadBuffer, and is never touched again.
		auto* mapped = m_glyph_upload_mapped[slot.page];
		for (auto row = std::uint32_t{}; row != slot.pixel_h; ++row)
		{
			auto* dst = mapped + (static_cast<std::size_t>(slot.pixel_y) + row) * m_glyph_atlas_dim_px + slot.pixel_x;
			auto const* src = bitmap.alpha.data() + static_cast<std::size_t>(row) * slot.pixel_w;
			std::memcpy(dst, src, slot.pixel_w);
		}

		D3D12_TEXTURE_COPY_LOCATION dst_loc{};
		dst_loc.pResource = m_glyph_atlas.Get();
		dst_loc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
		dst_loc.SubresourceIndex = slot.page; // one mip level, so array slice == subresource index

		D3D12_TEXTURE_COPY_LOCATION src_loc{};
		src_loc.pResource = m_glyph_uploads[slot.page].Get();
		src_loc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
		src_loc.PlacedFootprint.Offset = 0;
		src_loc.PlacedFootprint.Footprint.Format = DXGI_FORMAT_R8_UNORM;
		src_loc.PlacedFootprint.Footprint.Width = m_glyph_atlas_dim_px;
		src_loc.PlacedFootprint.Footprint.Height = m_glyph_atlas_dim_px;
		src_loc.PlacedFootprint.Footprint.Depth = 1;
		src_loc.PlacedFootprint.Footprint.RowPitch = m_glyph_atlas_dim_px; // 1 byte/pixel; already 256-byte aligned by kAtlasPageDim

		D3D12_BOX const src_box{ slot.pixel_x, slot.pixel_y, 0, slot.pixel_x + slot.pixel_w, slot.pixel_y + slot.pixel_h, 1 };

		// The atlas otherwise rests in PIXEL_SHADER_RESOURCE; only this Renderer's own resource is
		// transitioned here, and only around its own copy, never a resource owned by 'pass'.
		D3D12_RESOURCE_BARRIER to_copy_dest{};
		to_copy_dest.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
		to_copy_dest.Transition.pResource = m_glyph_atlas.Get();
		to_copy_dest.Transition.Subresource = slot.page;
		to_copy_dest.Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
		to_copy_dest.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
		command_list->ResourceBarrier(1, &to_copy_dest);

		command_list->CopyTextureRegion(&dst_loc, slot.pixel_x, slot.pixel_y, 0, &src_loc, &src_box);

		auto to_shader_resource = to_copy_dest;
		std::swap(to_shader_resource.Transition.StateBefore, to_shader_resource.Transition.StateAfter);
		command_list->ResourceBarrier(1, &to_shader_resource);
	}

	void Renderer::PrepareText(ID3D12GraphicsCommandList* command_list, DrawPacket const& packet, bool& out_hit_limit, bool& out_missing_asset)
	{
		out_hit_limit = false;
		out_missing_asset = false;
		m_missing_fonts.clear();
		auto const dpi_scale = packet.viewport_dpi / 96.0f;

		// Shapes/rasterizes every not-yet-resident glyph across every text item this frame, so
		// Draw() later never touches DirectWrite or issues an upload; a glyph the cache cannot
		// place under its bounded capacity is simply omitted from this frame rather than aborting
		// the rest of the packet (out_hit_limit reports this to the caller as ResourceLimit).
		std::vector<ShapedGlyph> glyphs;
		for (auto const& item : packet.items)
		{
			if (item.primitive != EVisualPrimitive::TextPresenter || item.text.empty())
				continue;

			// A font resource naming a family this machine does not have is a diagnosable missing
			// asset, not a licence to draw in some other face: record it, skip the item for the
			// rest of the frame, and keep rendering everything else.
			auto const request_key = TextShaper::FontKey(item.font_family, item.font_size);
			if (m_missing_fonts.contains(request_key))
				continue;

			try
			{
				m_text_shaper->Shape(item.font_family, item.font_size, dpi_scale, item.text, glyphs);
			}
			catch (EngineException const& ex)
			{
				if (ex.Status() != EStatus::MissingAsset)
					throw;

				m_missing_fonts.insert(request_key);
				out_missing_asset = true;
				continue;
			}

			for (auto const& glyph : glyphs)
			{
				// Keyed by the run's *resolved* face, because DirectWrite may shape one string
				// across several faces and a glyph index is only meaningful within its own face.
				auto const key = GlyphKey{ glyph.font_key, glyph.glyph_index };
				if (m_glyph_cache->Find(key) != nullptr)
					continue; // already resident from an earlier frame

				auto const bitmap = m_text_shaper->Rasterize(glyph.font_key, dpi_scale, glyph.glyph_index);
				if (bitmap.width_px == 0 || bitmap.height_px == 0)
					continue; // a whitespace glyph has no coverage pixels and is never placed

				GlyphSlot slot{};
				auto new_page = false;
				auto const status = m_glyph_cache->Acquire(key, bitmap.width_px, bitmap.height_px, bitmap.origin_x_px, bitmap.origin_y_px, slot, new_page);
				if (status == EStatus::ResourceLimit)
				{
					out_hit_limit = true;
					continue;
				}

				EnsureGlyphUploadBuffer(slot.page);
				UploadGlyphBitmap(slot, bitmap, command_list);
			}
		}
	}

	float Renderer::TextRunStartXDip(DrawItem const& item, float total_advance_dip)
	{
		switch (item.text_align)
		{
			case ETextAlign::Left: return item.bounds.x + item.text_inset_dip;
			case ETextAlign::Center: return item.bounds.x + (item.bounds.w - total_advance_dip) * 0.5f;
			case ETextAlign::Count:
			default:
			{
				throw EngineException(EStatus::InvalidArgument, "unknown text alignment");
			}
		}
	}

	// Fill the per-group half of the root constants; every draw within a root's subtree shares it.
	namespace
	{
		void ApplyGroupState(RootConstants& rc, GroupState const& group)
		{
			rc.g5[2] = group.clip_depth;
			rc.g6[0] = group.view_depth;
			rc.g6[1] = group.min_opacity;
			rc.g6[2] = group.fade_depth;
			rc.g6[3] = group.depth_bias;
			rc.g7[0] = group.near_plane;
			rc.g7[1] = group.far_plane;
			rc.g7[2] = group.orthographic;
		}
	}

	void Renderer::DrawBoxItem(Pass const& pass, GroupState const& group, DrawItem const& item, float dpi_scale, float viewport_w, float viewport_h)
	{
		RootConstants rc{};
		rc.g0[0] = viewport_w;
		rc.g0[1] = viewport_h;
		rc.g0[2] = item.bounds.x * dpi_scale;
		rc.g0[3] = item.bounds.y * dpi_scale;
		rc.g1[0] = item.bounds.w * dpi_scale;
		rc.g1[1] = item.bounds.h * dpi_scale;
		rc.g1[2] = item.border_thickness * dpi_scale;
		rc.g1[3] = item.corner_radius * dpi_scale;
		rc.g2[0] = item.fill.r;
		rc.g2[1] = item.fill.g;
		rc.g2[2] = item.fill.b;
		rc.g2[3] = item.fill.a;
		rc.g3[0] = item.border_colour.r;
		rc.g3[1] = item.border_colour.g;
		rc.g3[2] = item.border_colour.b;
		rc.g3[3] = item.border_colour.a;
		rc.g4[0] = item.opacity;
		// g4[1..3]/g5[0..1] are read only by the glyph shaders; left zero-initialised for a box draw.
		ApplyGroupState(rc, group);

		pass.m_command_list->SetGraphicsRoot32BitConstants(0, kRootConstantCount, &rc, 0);
		pass.m_command_list->DrawInstanced(4, 1, 0, 0);
	}

	// Fill the per-item half of the root constants for a solid, untextured quad in DIP-space
	// bounds. Used for a text item's selection highlight, composition underline and caret, all of
	// which are plain rectangles rather than glyphs.
	namespace
	{
		void FillDecorationConstants(RootConstants& rc, Rect const& bounds_dip, Colour const& fill, float opacity, float dpi_scale, float viewport_w, float viewport_h)
		{
			rc.g0[0] = viewport_w;
			rc.g0[1] = viewport_h;
			rc.g0[2] = bounds_dip.x * dpi_scale;
			rc.g0[3] = bounds_dip.y * dpi_scale;
			rc.g1[0] = bounds_dip.w * dpi_scale;
			rc.g1[1] = bounds_dip.h * dpi_scale;
			rc.g2[0] = fill.r;
			rc.g2[1] = fill.g;
			rc.g2[2] = fill.b;
			rc.g2[3] = fill.a;
			rc.g4[0] = opacity;
		}
	}

	EVisualPrimitive Renderer::DrawTextItem(Pass const& pass, GroupState const& group, DrawItem const& item, PipelineVariant const& variant, EVisualPrimitive bound, float dpi_scale, float viewport_w, float viewport_h)
	{
		// A focused empty field still has to paint its caret, so only an item with neither text nor
		// a caret can be skipped outright.
		if (item.text.empty() && item.caret_visible == 0)
			return bound;

		// PrepareText already reported this family as missing for the whole frame; drawing it in
		// some substitute face would contradict the module's font-resolution policy.
		if (m_missing_fonts.contains(TextShaper::FontKey(item.font_family, item.font_size)))
			return bound;

		// Re-shape purely to recover this frame's pen positions; PrepareText already guaranteed
		// every non-whitespace glyph here is either resident or was omitted under ResourceLimit,
		// so this call never rasterizes anything (cached font face, no CreateGlyphRunAnalysis).
		std::vector<ShapedGlyph> glyphs;
		auto const total_advance_dip = m_text_shaper->Shape(item.font_family, item.font_size, dpi_scale, item.text, glyphs);

		float ascent_dip = 0.0f, descent_dip = 0.0f;
		m_text_shaper->Metrics(item.font_family, item.font_size, ascent_dip, descent_dip);

		// The whole layout box is placed once, and every glyph, selection rectangle and caret is
		// then expressed relative to it, so the decorations can never drift away from the glyphs.
		auto const origin_x_dip = TextRunStartXDip(item, total_advance_dip);
		auto const origin_y_dip = TextOriginYDip(item.bounds.y, item.bounds.h, ascent_dip, descent_dip);

		// Selection and composition rectangles paint behind the glyphs, so the glyph coverage is
		// composited over them in a single pass without needing a blend-state change.
		auto has_selection = item.selection_end > item.selection_start;
		auto has_composition = item.composition_length != 0;
		if (has_selection || has_composition)
		{
			if (bound != EVisualPrimitive::SolidBox)
			{
				pass.m_command_list->SetPipelineState(variant.box.Get());
				bound = EVisualPrimitive::SolidBox;
			}

			if (has_selection)
			{
				auto const rects = m_text_shaper->RangeRects(item.font_family, item.font_size, item.text, item.selection_start, item.selection_end, kMaxDecorationRects);
				for (auto const& r : rects)
				{
					RootConstants rc{};
					FillDecorationConstants(rc, Rect{ origin_x_dip + r.x, origin_y_dip + r.y, r.w, r.h }, kSelectionFill, item.opacity, dpi_scale, viewport_w, viewport_h);
					ApplyGroupState(rc, group);
					pass.m_command_list->SetGraphicsRoot32BitConstants(0, kRootConstantCount, &rc, 0);
					pass.m_command_list->DrawInstanced(4, 1, 0, 0);
				}
			}

			// Composition text is underlined rather than filled, so the text under an active IME
			// conversion reads as provisional and stays visually distinct from committed text.
			if (has_composition)
			{
				auto const rects = m_text_shaper->RangeRects(item.font_family, item.font_size, item.text, item.composition_start, item.composition_start + item.composition_length, kMaxDecorationRects);
				for (auto const& r : rects)
				{
					auto const underline = Rect{ origin_x_dip + r.x, origin_y_dip + r.y + r.h - kCompositionUnderlineDip, r.w, kCompositionUnderlineDip };
					RootConstants rc{};
					FillDecorationConstants(rc, underline, item.fill, item.opacity, dpi_scale, viewport_w, viewport_h);
					ApplyGroupState(rc, group);
					pass.m_command_list->SetGraphicsRoot32BitConstants(0, kRootConstantCount, &rc, 0);
					pass.m_command_list->DrawInstanced(4, 1, 0, 0);
				}
			}
		}

		if (!glyphs.empty())
		{
			if (bound != EVisualPrimitive::TextPresenter)
			{
				pass.m_command_list->SetPipelineState(variant.glyph.Get());
				bound = EVisualPrimitive::TextPresenter;
			}

			for (auto const& glyph : glyphs)
			{
				auto const key = GlyphKey{ glyph.font_key, glyph.glyph_index };
				auto const* slot = m_glyph_cache->Find(key);
				if (slot == nullptr)
					continue; // whitespace, or this glyph was omitted this frame under ResourceLimit

				auto const baseline_px_x = (origin_x_dip + glyph.origin_x) * dpi_scale;
				auto const baseline_px_y = (origin_y_dip + glyph.origin_y) * dpi_scale;

				RootConstants rc{};
				rc.g0[0] = viewport_w;
				rc.g0[1] = viewport_h;
				rc.g0[2] = baseline_px_x + static_cast<float>(slot->origin_x_px);
				rc.g0[3] = baseline_px_y + static_cast<float>(slot->origin_y_px);
				rc.g1[0] = static_cast<float>(slot->pixel_w);
				rc.g1[1] = static_cast<float>(slot->pixel_h);
				// g1[2..3]/g3[*] are read only by the box shaders; left zero-initialised for a glyph draw.
				rc.g2[0] = item.fill.r;
				rc.g2[1] = item.fill.g;
				rc.g2[2] = item.fill.b;
				rc.g2[3] = item.fill.a;
				rc.g4[0] = item.opacity;
				rc.g4[1] = slot->uv0_x;
				rc.g4[2] = slot->uv0_y;
				rc.g4[3] = slot->uv1_x;
				rc.g5[0] = slot->uv1_y;
				rc.g5[1] = static_cast<float>(slot->page);
				ApplyGroupState(rc, group);

				pass.m_command_list->SetGraphicsRoot32BitConstants(0, kRootConstantCount, &rc, 0);
				pass.m_command_list->DrawInstanced(4, 1, 0, 0);
			}
		}

		// The caret paints last so it stays visible over both the glyphs and any highlight.
		if (item.caret_visible != 0)
		{
			if (bound != EVisualPrimitive::SolidBox)
			{
				pass.m_command_list->SetPipelineState(variant.box.Get());
				bound = EVisualPrimitive::SolidBox;
			}

			auto const caret = m_text_shaper->CaretAt(item.font_family, item.font_size, item.text, item.caret_offset);
			RootConstants rc{};
			FillDecorationConstants(rc, Rect{ origin_x_dip + caret.x, origin_y_dip + caret.y, kCaretWidthDip, caret.height }, item.fill, item.opacity, dpi_scale, viewport_w, viewport_h);
			ApplyGroupState(rc, group);
			pass.m_command_list->SetGraphicsRoot32BitConstants(0, kRootConstantCount, &rc, 0);
			pass.m_command_list->DrawInstanced(4, 1, 0, 0);
		}

		return bound;
	}

	void Renderer::Draw(Pass const& pass, DrawPacket const& packet, ERootPolicy policy, PipelineVariant const& variant, bool bind_depth)
	{
		auto const dpi_scale = packet.viewport_dpi / 96.0f;

		// Every rect in the packet is local to the host's viewport, so the shader divides by the
		// viewport extent; RSSetViewports below then applies the viewport origin and extent once.
		auto const divisor = NdcDivisorPx(pass);
		auto const viewport_w = divisor.x;
		auto const viewport_h = divisor.y;

		// A pass that owns no group this frame records nothing at all, so an unused host stage
		// costs exactly one predicate rather than a redundant state block.
		auto const has_work = std::any_of(packet.groups.begin(), packet.groups.end(), [policy](DrawGroup const& g) { return g.policy == policy && g.item_count != 0; });
		if (!has_work)
			return;

		// Pipeline/root-signature/heap/topology/viewport/scissor/render-target state is set once
		// per pass, not per item; only SetPipelineState toggles (box vs glyph) and the per-item
		// root constants vary within the loop below.
		pass.m_command_list->SetGraphicsRootSignature(m_root_signature.Get());

		ID3D12DescriptorHeap* heaps[] = { m_srv_heap.Get() };
		pass.m_command_list->SetDescriptorHeaps(1, heaps);
		pass.m_command_list->SetGraphicsRootDescriptorTable(1, m_srv_heap->GetGPUDescriptorHandleForHeapStart());

		pass.m_command_list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
		pass.m_command_list->RSSetViewports(1, &pass.m_viewport);
		pass.m_command_list->RSSetScissorRects(1, &pass.m_scissor);

		auto const rtv = pass.m_rtv;
		auto const dsv = pass.m_dsv;
		pass.m_command_list->OMSetRenderTargets(1, &rtv, FALSE, bind_depth ? &dsv : nullptr);

		// The camera's clip planes are shared by every group; the fade shaders need them to turn a
		// sampled device depth back into a linear distance.
		auto const near_plane = pass.m_camera.m_near_plane;
		auto const far_plane = pass.m_camera.m_far_plane;
		auto const orthographic = pass.m_camera.m_orthographic != 0 ? 1.0f : 0.0f;

		// DrawPacket::groups is in tree root order and partitions DrawPacket::items, which is
		// itself a stable pre-order traversal (draw_packet.h), so recording draw calls in this
		// order is what makes the frame's visual stacking deterministic.
		for (auto const& group : packet.groups)
		{
			if (group.policy != policy || group.item_count == 0)
				continue;

			auto const state = GroupState{
				.clip_depth = group.clip_depth,
				.view_depth = group.view_depth,
				.min_opacity = group.occlusion_min_opacity,
				.fade_depth = group.occlusion_fade_depth,
				.depth_bias = group.occlusion_depth_bias,
				.near_plane = near_plane,
				.far_plane = far_plane,
				.orthographic = orthographic,
			};

			auto bound = EVisualPrimitive::Count;
			for (auto i = group.first_item, end = group.first_item + group.item_count; i != end; ++i)
			{
				auto const& item = packet.items[i];
				switch (item.primitive)
				{
					case EVisualPrimitive::SolidBox:
					case EVisualPrimitive::RoundedBox:
					case EVisualPrimitive::Border:
					{
						if (bound != EVisualPrimitive::SolidBox)
						{
							pass.m_command_list->SetPipelineState(variant.box.Get());
							bound = EVisualPrimitive::SolidBox;
						}
						DrawBoxItem(pass, state, item, dpi_scale, viewport_w, viewport_h);
						break;
					}
					case EVisualPrimitive::TextPresenter:
					{
						// DrawTextItem binds its own pipelines because one text item can mix box
						// quads (selection, composition underline, caret) with glyph quads.
						bound = DrawTextItem(pass, state, item, variant, bound, dpi_scale, viewport_w, viewport_h);
						break;
					}
					case EVisualPrimitive::ContentPresenter:
					case EVisualPrimitive::Clip:
					case EVisualPrimitive::TransformOpacityGroup:
					case EVisualPrimitive::Count:
					default:
						// Reserved for a future milestone (draw_packet_builder.cpp does not emit these
						// yet); reaching here means the builder started emitting a primitive this
						// renderer was never updated to draw, which is a bug, not a runtime condition.
						throw EngineException(EStatus::InternalError, "Renderer: visual primitive not implemented");
				}
			}
		}
	}

	ERootPolicy Renderer::PolicyForPass(EPass pass)
	{
		switch (pass)
		{
			case EPass::DepthTested: return ERootPolicy::DepthTested;
			case EPass::OcclusionFaded: return ERootPolicy::OcclusionFaded;
			case EPass::Overlay: return ERootPolicy::Overlay;
			case EPass::FinalOverlay: return ERootPolicy::Screen;
			case EPass::Prepare:
			default:
			{
				throw EngineException(EStatus::InvalidArgument, "Renderer: pass does not draw any root policy");
			}
		}
	}

	EStatus Renderer::Record(Pass const& pass, DrawPacket const& packet)
	{
		if (m_device_lost)
			return EStatus::DeviceLost;

		try
		{
			switch (pass.m_pass)
			{
				case EPass::Prepare:
				{
					EnsureDeviceResources(pass.m_colour_format);
					if (CheckDeviceLost())
						return EStatus::DeviceLost;

					auto hit_limit = false;
					auto missing_asset = false;
					PrepareText(pass.m_command_list, packet, hit_limit, missing_asset);

					// A missing font is the more actionable diagnosis, so it wins over a glyph
					// budget that was merely exhausted; both leave the rest of the frame drawable.
					if (missing_asset)
						return EStatus::MissingAsset;

					return hit_limit ? EStatus::ResourceLimit : EStatus::Success;
				}
				case EPass::DepthTested:
				{
					if (m_d3d_device == nullptr)
						return EStatus::Success; // Prepare has not yet realised device resources this session; nothing to draw

					if (CheckDeviceLost())
						return EStatus::DeviceLost;

					// Without a host depth-stencil view there is nothing to test against, so the
					// policy degrades to drawing nothing rather than silently becoming an overlay.
					if (pass.m_dsv.ptr == 0 || pass.m_depth_format == DXGI_FORMAT_UNKNOWN)
						return EStatus::Success;

					Draw(pass, packet, ERootPolicy::DepthTested, EnsureDepthPipeline(pass), true);
					return EStatus::Success;
				}
				case EPass::OcclusionFaded:
				{
					if (m_d3d_device == nullptr)
						return EStatus::Success;

					if (CheckDeviceLost())
						return EStatus::DeviceLost;

					// The overlay pipelines are built once against a single-sample target, which is
					// what the host's post-alpha swap target always is. A multi-sampled target here
					// would be a silent pipeline mismatch, so refuse it rather than risk the device.
					if (pass.m_sample_count != 1)
						return EStatus::InvalidArgument;

					// The fade is defined only against the host's resolved single-sample depth; if
					// the host offered none this frame, nothing is drawn rather than falling back
					// to a differently-defined depth source.
					if (!EnsureResolvedDepthView(pass))
						return EStatus::Success;

					Draw(pass, packet, ERootPolicy::OcclusionFaded, m_pso_faded, false);
					return EStatus::Success;
				}
				case EPass::Overlay:
				{
					if (m_d3d_device == nullptr)
						return EStatus::Success;

					if (CheckDeviceLost())
						return EStatus::DeviceLost;

					if (pass.m_sample_count != 1)
						return EStatus::InvalidArgument; // see EPass::OcclusionFaded

					Draw(pass, packet, ERootPolicy::Overlay, m_pso_overlay, false);
					return EStatus::Success;
				}
				case EPass::FinalOverlay:
				{
					if (m_d3d_device == nullptr)
						return EStatus::Success; // Prepare has not yet realised device resources this session; nothing to draw

					if (CheckDeviceLost())
						return EStatus::DeviceLost;

					if (pass.m_sample_count != 1)
						return EStatus::InvalidArgument; // see EPass::OcclusionFaded

					Draw(pass, packet, ERootPolicy::Screen, m_pso_overlay, false);
					return EStatus::Success;
				}
				default:
					throw EngineException(EStatus::InvalidArgument, "Renderer: pass.m_pass out of range");
			}
		}
		catch (EngineException const& ex)
		{
			return ex.Status();
		}
	}
}
