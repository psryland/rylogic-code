//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"

namespace pr::rdr12
{
	namespace
	{
		// One pipeline state stream subobject: its type tag followed by its value, aligned as the runtime parses them. The padding is intended.
		#pragma warning(push)
		#pragma warning(disable: 4324)
		template <D3D12_PIPELINE_STATE_SUBOBJECT_TYPE Type, typename T>
		struct alignas(void*) StreamSubobject
		{
			D3D12_PIPELINE_STATE_SUBOBJECT_TYPE m_type = Type;
			T m_value;
		};
		#pragma warning(pop)

		// The graphics pipeline description fields, plus view instancing, as a pipeline state stream
		struct GraphicsStream
		{
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_ROOT_SIGNATURE, ID3D12RootSignature*> m_root_sig;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_VS, D3D12_SHADER_BYTECODE> m_vs;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_PS, D3D12_SHADER_BYTECODE> m_ps;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DS, D3D12_SHADER_BYTECODE> m_ds;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_HS, D3D12_SHADER_BYTECODE> m_hs;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_GS, D3D12_SHADER_BYTECODE> m_gs;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_STREAM_OUTPUT, D3D12_STREAM_OUTPUT_DESC> m_stream_output;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_BLEND, D3D12_BLEND_DESC> m_blend;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_MASK, UINT> m_sample_mask;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RASTERIZER, D3D12_RASTERIZER_DESC> m_rasterizer;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL, D3D12_DEPTH_STENCIL_DESC> m_depth_stencil;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_INPUT_LAYOUT, D3D12_INPUT_LAYOUT_DESC> m_input_layout;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_IB_STRIP_CUT_VALUE, D3D12_INDEX_BUFFER_STRIP_CUT_VALUE> m_strip_cut;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_PRIMITIVE_TOPOLOGY, D3D12_PRIMITIVE_TOPOLOGY_TYPE> m_topology;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RENDER_TARGET_FORMATS, D3D12_RT_FORMAT_ARRAY> m_rtv_formats;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL_FORMAT, DXGI_FORMAT> m_dsv_format;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_DESC, DXGI_SAMPLE_DESC> m_sample_desc;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_NODE_MASK, UINT> m_node_mask;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_CACHED_PSO, D3D12_CACHED_PIPELINE_STATE> m_cached_pso;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_FLAGS, D3D12_PIPELINE_STATE_FLAGS> m_flags;
			StreamSubobject<D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_VIEW_INSTANCING, D3D12_VIEW_INSTANCING_DESC> m_view_instancing;
		};

		// Create a graphics pipeline state that rasterises each draw once per view, with view 'i' going to viewport and scissor 'i'
		D3DPtr<ID3D12PipelineState> CreateViewInstancedPSO(ID3D12Device4* device, D3D12_GRAPHICS_PIPELINE_STATE_DESC const& desc, int view_count)
		{
			// The view locations must outlive the creation call
			D3D12_VIEW_INSTANCE_LOCATION locations[D3D12_MAX_VIEW_INSTANCE_COUNT] = {};
			for (int i = 0; i != view_count; ++i)
				locations[i] = D3D12_VIEW_INSTANCE_LOCATION{ .ViewportArrayIndex = s_cast<UINT>(i), .RenderTargetArrayIndex = 0 };

			// Copy the description into the stream
			auto rtv_formats = D3D12_RT_FORMAT_ARRAY{ .NumRenderTargets = desc.NumRenderTargets };
			for (int i = 0; i != _countof(rtv_formats.RTFormats); ++i)
				rtv_formats.RTFormats[i] = desc.RTVFormats[i];

			GraphicsStream stream = {};
			stream.m_root_sig.m_value = desc.pRootSignature;
			stream.m_vs.m_value = desc.VS;
			stream.m_ps.m_value = desc.PS;
			stream.m_ds.m_value = desc.DS;
			stream.m_hs.m_value = desc.HS;
			stream.m_gs.m_value = desc.GS;
			stream.m_stream_output.m_value = desc.StreamOutput;
			stream.m_blend.m_value = desc.BlendState;
			stream.m_sample_mask.m_value = desc.SampleMask;
			stream.m_rasterizer.m_value = desc.RasterizerState;
			stream.m_depth_stencil.m_value = desc.DepthStencilState;
			stream.m_input_layout.m_value = desc.InputLayout;
			stream.m_strip_cut.m_value = desc.IBStripCutValue;
			stream.m_topology.m_value = desc.PrimitiveTopologyType;
			stream.m_rtv_formats.m_value = rtv_formats;
			stream.m_dsv_format.m_value = desc.DSVFormat;
			stream.m_sample_desc.m_value = desc.SampleDesc;
			stream.m_node_mask.m_value = desc.NodeMask;
			stream.m_cached_pso.m_value = desc.CachedPSO;
			stream.m_flags.m_value = desc.Flags;
			stream.m_view_instancing.m_value = D3D12_VIEW_INSTANCING_DESC{ .ViewInstanceCount = s_cast<UINT>(view_count), .pViewInstanceLocations = &locations[0], .Flags = D3D12_VIEW_INSTANCING_FLAG_NONE };

			D3DPtr<ID3D12PipelineState> pso;
			auto stream_desc = D3D12_PIPELINE_STATE_STREAM_DESC{ .SizeInBytes = sizeof(stream), .pPipelineStateSubobjectStream = &stream };
			Check(device->CreatePipelineState(&stream_desc, __uuidof(ID3D12PipelineState), (void**)pso.address_of()));
			return pso;
		}
	}

	PipeStatePool::PipeStatePool(Window& wnd)
		:m_wnd(&wnd)
		,m_pool()
	{}

	// Return a pipeline state instance for the given description
	ID3D12PipelineState* PipeStatePool::Get(PipeStateDesc const& desc)
	{
		auto frame_number = m_wnd->FrameNumber();

		// See if a pipeline state object already exists
		auto iter = pr::find_if(m_pool, [=](auto& pso) { return pso.m_hash == desc.hash(); });
		if (iter == m_pool.end())
		{
			// If it doesn't exists, create it now
			auto device = m_wnd->rdr().D3DDevice();

			// Delete any states that haven't been used for a while
			constexpr size_t MaxFrameAge = 30;
			constexpr size_t DrainPoolCount = 100;
			auto pool_size = m_pool.size();
			if (pool_size > DrainPoolCount)
			{
				erase_if_unstable(m_pool, [=](auto& item) { return frame_number - item.m_frame_number > MaxFrameAge; });

				#if PR_DBG_RDR
				if (m_pool.size() == pool_size)
					throw std::runtime_error("Too many unique pipeline states");
				#endif
			}

			// Create the pipeline state instance. Append it, because draining the pool invalidates 'iter'.
			D3DPtr<ID3D12PipelineState> pso;
			if (desc.ViewCount() != 0)
				pso = CreateViewInstancedPSO(device, *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc), desc.ViewCount());
			else
				Check(device->CreateGraphicsPipelineState(desc, __uuidof(ID3D12PipelineState), (void**)pso.address_of()));

			m_pool.emplace_back(pso, frame_number, desc.hash());
			iter = m_pool.end() - 1;
		}

		auto& pso = *iter;
		pso.m_frame_number = frame_number;
		return pso.m_pso.get();
	}
}