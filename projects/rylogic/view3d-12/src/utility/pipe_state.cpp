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

	// Return a hash of the final state, for lookup only
	uint64_t PipeStateDesc::Hash() const
	{
		// Mix the description one word at a time. It is hashed on every pipeline change while drawing, so this must be cheap.
		static_assert(sizeof(m_desc) % sizeof(uint64_t) == 0);
		auto hash = s_cast<uint64_t>(m_view_count) * 0x9E3779B97F4A7C15ULL;
		auto const* words = reinterpret_cast<uint64_t const*>(&m_desc);
		for (size_t i = 0, iend = sizeof(m_desc) / sizeof(uint64_t); i != iend; ++i)
		{
			// Each step spreads the word's bits over the whole hash
			hash = (hash ^ words[i]) * 0xFF51AFD7ED558CCDULL;
			hash ^= hash >> 32;
		}
		return hash;
	}

	PipeStateCache::PipeStateCache(Renderer& rdr)
		:m_rdr(&rdr)
		,m_entries()
		,m_mutex()
	{}

	// Return the pipeline state object for 'desc', creating it if needed
	ID3D12PipelineState* PipeStateCache::Get(PipeStateDesc const& desc)
	{
		// Find an existing pipeline with an identical final state. The hash only narrows the search.
		auto const hash = desc.Hash();
		auto const now = clock_t::now();
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto [beg, end] = m_entries.equal_range(hash); beg != end; ++beg)
		{
			auto& entry = beg->second;
			if (!(entry.m_desc == desc))
				continue;

			entry.m_last_used = now;
			return entry.m_pso.get();
		}

		// Release idle pipelines before adding another, so the cache does not grow without bound
		if (m_entries.size() >= TrimCount)
			Trim(now);

		// Create the pipeline state instance
		auto device = m_rdr->D3DDevice();
		auto const& d3d_desc = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
		D3DPtr<ID3D12PipelineState> pso;
		if (desc.ViewCount() != 0)
			pso = CreateViewInstancedPSO(device, d3d_desc, desc.ViewCount());
		else
			Check(device->CreateGraphicsPipelineState(&d3d_desc, __uuidof(ID3D12PipelineState), (void**)pso.address_of()));

		// Hold the root signature so that its address keeps identifying this entry
		auto& entry = m_entries.emplace(hash, Entry{
			.m_desc = desc,
			.m_signature = D3DPtr<ID3D12RootSignature>(d3d_desc.pRootSignature, true),
			.m_pso = pso,
			.m_last_used = now,
		})->second;
		return entry.m_pso.get();
	}

	// Release every pipeline that uses 'signature' or any shader stage starting at an address in 'bytecode'
	void PipeStateCache::Forget(ID3D12RootSignature const* signature, std::span<void const* const> bytecode)
	{
		// True if 'code' is one of the stages being forgotten
		auto forgotten = [=](D3D12_SHADER_BYTECODE const& code)
		{
			return code.pShaderBytecode != nullptr && std::ranges::find(bytecode, code.pShaderBytecode) != bytecode.end();
		};

		// Remove the entries that refer to the forgotten objects
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto iter = m_entries.begin(); iter != m_entries.end();)
		{
			auto& entry = iter->second;
			auto const& d = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(entry.m_desc);
			auto doomed =
				(signature != nullptr && d.pRootSignature == signature) ||
				forgotten(d.VS) || forgotten(d.PS) || forgotten(d.DS) || forgotten(d.HS) || forgotten(d.GS);
			if (!doomed)
			{
				++iter;
				continue;
			}

			Release(entry);
			iter = m_entries.erase(iter);
		}
	}

	// The number of cached pipeline state objects
	size_t PipeStateCache::Count() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_entries.size();
	}

	// Release entries that have been idle for too long
	void PipeStateCache::Trim(clock_t::time_point now)
	{
		// Recently used entries are kept even if the cache stays large, because their states are still being drawn
		for (auto iter = m_entries.begin(); iter != m_entries.end();)
		{
			auto& entry = iter->second;
			if (now - entry.m_last_used <= MaxIdleTime)
			{
				++iter;
				continue;
			}

			Release(entry);
			iter = m_entries.erase(iter);
		}
	}

	// Release one entry's pipeline once the GPU is no longer using it
	void PipeStateCache::Release(Entry& entry)
	{
		// Command lists that are recorded but not yet finished may still reference the pipeline and its root signature
		m_rdr->DeferRelease(entry.m_pso);
		m_rdr->DeferRelease(entry.m_signature);
	}
}