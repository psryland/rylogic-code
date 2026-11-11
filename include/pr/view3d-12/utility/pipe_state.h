//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Notes:
	//  - The enum values 'EPipeState' are encoded with (offset, size) pairs, corresponding to the
	//    parts of the PSO description that can be changed. Add as required.

	// IDs for the pipeline state description fields
	enum class EPipeState :uint32_t
	{
		// Parts of the pipeline state that can be changed.
		#define PR_RDR_PIPE_STATE_FIELDS(x)/*(EPipeState::<name>, PipelineStateDesc::<field>) */\
			x(RootSignature         , pRootSignature                        )\
			x(VS                    , VS                                    )\
			x(PS                    , PS                                    )\
			x(DS                    , DS                                    )\
			x(HS                    , HS                                    )\
			x(GS                    , GS                                    )\
			x(TopologyType          , PrimitiveTopologyType                 )\
			x(FillMode              , RasterizerState.FillMode              )\
			x(CullMode              , RasterizerState.CullMode              )\
			x(DepthClipEnable       , RasterizerState.DepthClipEnable       )\
			x(DepthEnable           , DepthStencilState.DepthEnable         )\
			x(DepthWriteMask        , DepthStencilState.DepthWriteMask      )\
			x(DepthFunc             , DepthStencilState.DepthFunc           )\
			x(BlendState0           , BlendState.RenderTarget[0]            )\
			x(RTVFormats            , RTVFormats                            )\
			x(DSVFormat             , DSVFormat                             )\
			x(SampleDesc            , SampleDesc                            )\
			x(IBStripCutValue       , IBStripCutValue                       )\
			x(InputLayout           , InputLayout                           )\
			//x(StreamOutput          , StreamOutput                          )\
			//x(BlendEnable0          , BlendState.RenderTarget[0].BlendEnable)\
			//x(BlendEnable1          , BlendState.RenderTarget[1].BlendEnable)\
			//x(BlendEnable2          , BlendState.RenderTarget[2].BlendEnable)\
			//x(BlendEnable3          , BlendState.RenderTarget[3].BlendEnable)\
			//x(BlendEnable4          , BlendState.RenderTarget[4].BlendEnable)\
			//x(BlendEnable5          , BlendState.RenderTarget[5].BlendEnable)\
			//x(BlendEnable6          , BlendState.RenderTarget[6].BlendEnable)\
			//x(BlendEnable7          , BlendState.RenderTarget[7].BlendEnable)\
			//x(SampleMask            , SampleMask                            )\
			//x(RasterizerState       , RasterizerState                       )\
			//x(DepthStencilState     , DepthStencilState                     )\
			//x(PrimitiveTopologyType , PrimitiveTopologyType                 )\
			//x(NumRenderTargets      , NumRenderTargets                      )\
			//x(NodeMask              , NodeMask                              )\
			//x(CachedPSO             , CachedPSO                             )\
			//x(Flags                 , Flags                                 )

		// Encode the byte offset and size into the enum value
		#define PR_RDR_PIPE_STATE_FIELD(name, member) name = \
			(((0xFFFF & offsetof(D3D12_GRAPHICS_PIPELINE_STATE_DESC, member)) << 16) | \
			 ((0xFFFF & sizeof(std::declval<D3D12_GRAPHICS_PIPELINE_STATE_DESC>().member)))),
		PR_RDR_PIPE_STATE_FIELDS(PR_RDR_PIPE_STATE_FIELD)
		#undef PR_RDR_PIPE_STATE_FIELD
	};

	// Convert 'EPipeState' to the corresponding type
	template <EPipeState pls> struct pipe_state_field { using type = void; };
	#define PR_RDR_PIPE_STATE_FIELD(name, member)\
	template <> struct pipe_state_field<EPipeState::name> { using type = std::decay_t<decltype(std::declval<D3D12_GRAPHICS_PIPELINE_STATE_DESC>().member)>; };
	PR_RDR_PIPE_STATE_FIELDS(PR_RDR_PIPE_STATE_FIELD)
	#undef PR_RDR_PIPE_STATE_FIELD
	template <EPipeState PS> using pipe_state_field_t = typename pipe_state_field<PS>::type;

	#undef PR_RDR_PIPE_STATE_FILES

	// Represents a change to the pipeline state description
	struct PipeState
	{
		using field_t = struct { uint16_t size, ofs; };

		// The largest changeable PSO description field is a render target blend description. Every field is stored inline,
		// so pipe state changes are trivially copyable and never allocate.
		static constexpr size_t MaxFieldSize = sizeof(D3D12_RENDER_TARGET_BLEND_DESC);
		using state_t = struct alignas(8) { std::byte bytes[MaxFieldSize]; };

		state_t m_value;  // The data that replaces the PSO description field
		union {
		EPipeState m_id;  // Identifies the offset and size of the field in the PSO description.
		field_t m_field;  // Ofs/Size into the PSO description. (this is how 'EPipeState' is encoded)
		};

		PipeState() = default;

		template <typename TState> PipeState(EPipeState ps, TState const& value)
			: m_value()
			, m_id(ps)
		{
			static_assert(std::is_trivially_copyable_v<TState>, "TState must be trivially copyable");
			static_assert(sizeof(TState) <= sizeof(state_t) && alignof(TState) <= alignof(state_t), "TState is too large for a pipe state field");
			assert(m_field.size == sizeof(TState) && "Value type does not match the pipe state field");
			memcpy(&m_value, &value, sizeof(TState));
		}

		// Returns a pointer into 'desc' for this pipe state field
		void const* ptr(D3D12_GRAPHICS_PIPELINE_STATE_DESC const& desc) const
		{
			return byte_ptr(&desc) + m_field.ofs;
		}
		void* ptr(D3D12_GRAPHICS_PIPELINE_STATE_DESC& desc) const // Note: the returned pointer is into 'desc' not '*this'
		{
			return byte_ptr(&desc) + m_field.ofs;
		}
		void const* value() const
		{
			return &m_value;
		}
		int size() const
		{
			return m_field.size;
		}
	};

	// Create a PipeState object
	template <EPipeState PS> inline PipeState PSO(pipe_state_field_t<PS> const& data)
	{
		return PipeState(PS, data);
	}

	// True for strip primitive topologies. Only strip topologies honour primitive restart (strip-cut),
	// so callers gate the IBStripCutValue PSO override on this to leave list/point geometry's pipeline
	// state (and its PSO cache key) unchanged.
	constexpr bool IsStripTopo(ETopo topo)
	{
		switch (topo)
		{
			case ETopo::LineStrip:
			case ETopo::TriStrip:
			case ETopo::LineStripAdj:
			case ETopo::TriStripAdj:
				return true;
			case ETopo::Undefined:
			case ETopo::PointList:
			case ETopo::LineList:
			case ETopo::TriList:
			case ETopo::LineListAdj:
			case ETopo::TriListAdj:
				return false;
			default:
				throw std::runtime_error("Unknown topology");
		}
	}

	// The primitive-restart (strip-cut) sentinel that matches an index buffer's element format. The
	// engine never assigns the sentinel value as a real index (16-bit buffers cap vertex counts below
	// 0xFFFF, 32-bit below 0xFFFFFFFF), so enabling restart for strips can't corrupt existing geometry.
	constexpr D3D12_INDEX_BUFFER_STRIP_CUT_VALUE StripCutValue(DXGI_FORMAT index_format)
	{
		switch (index_format)
		{
			case DXGI_FORMAT_R16_UINT: return D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_0xFFFF;
			case DXGI_FORMAT_R32_UINT: return D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_0xFFFFFFFF;
			default: return D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED;
		}
	}

	// A collection of pipe state changes
	struct PipeStates
	{
		pr::vector<PipeState, 4> m_states;
		int m_fixed; // The first modifiable state. Below this index cannot be cleared/set.

		PipeStates()
			: m_states()
			, m_fixed()
		{}

		// The number of overrides
		int count() const
		{
			return s_cast<int>(m_states.size());
		}

		// Iteration access
		PipeState const* begin() const
		{
			return m_states.data();
		}
		PipeState const* end() const
		{
			return begin() + m_states.size();
		}

		// Remove a pipe state override
		template <EPipeState PS> void Clear()
		{
			pr::erase_if(m_states, [&](auto& ps) { return &ps - begin() >= m_fixed && ps.m_id == PS; });
		}

		// Set the pipeline state PS to 'data'
		template <EPipeState PS> void Set(pipe_state_field_t<PS> const& data)
		{
			Clear<PS>();
			m_states.push_back(PipeState(PS, data));
		}

		// See if the given pipe state is in the set of overrides
		template <EPipeState PS> pipe_state_field_t<PS> const* Find() const
		{
			for (auto iter = end(); iter-- != begin();)
			{
				auto& state = *iter;
				if (state.m_id != PS) continue;
				return type_ptr<pipe_state_field_t<PS>>(state.value());
			}
			return nullptr;
		}
	};

	// Pipe state object description. Two descriptions are equal when their final states are equal, whatever order changes were applied in.
	// Shader stages and the input layout are identified by address and length, so their data must outlive any pipeline created from them.
	struct PipeStateDesc
	{
	private:

		D3D12_GRAPHICS_PIPELINE_STATE_DESC m_desc;
		int m_view_count;

	public:

		PipeStateDesc()
			:m_desc()
			,m_view_count()
		{}
		PipeStateDesc(D3D12_GRAPHICS_PIPELINE_STATE_DESC const& rhs)
			:m_desc(rhs)
			,m_view_count()
		{}

		// Return a hash of the final state, for lookup only. Equal descriptions have equal hashes.
		uint64_t Hash() const;

		// True if the final states are identical
		friend bool operator == (PipeStateDesc const& lhs, PipeStateDesc const& rhs)
		{
			// Compare the bytes, because the description is plain data with no owned pointers
			return lhs.m_view_count == rhs.m_view_count && memcmp(&lhs.m_desc, &rhs.m_desc, sizeof(lhs.m_desc)) == 0;
		}

		// The number of view instances. Zero disables view instancing. Otherwise each draw is rasterised once per view, and view 'i' goes to
		// viewport and scissor 'i'. Requires device support for view instancing (see D3D12_FEATURE_DATA_D3D12_OPTIONS3::ViewInstancingTier).
		int ViewCount() const
		{
			return m_view_count;
		}
		void ViewCount(int count)
		{
			// View instancing has a device limit on the number of views
			assert(count >= 0 && count <= D3D12_MAX_VIEW_INSTANCE_COUNT);
			m_view_count = count;
		}

		// Apply changes to the pipeline state description
		void Apply(PipeState const& ps)
		{
			// Set the value of a pipeline state description field
			memcpy(ps.ptr(m_desc), ps.value(), ps.size());
		}

		// Return the current value of a pipeline state member
		template <EPipeState PS> pipe_state_field_t<PS> const& Get() const
		{
			// The field's byte offset is encoded in the upper half of the enum value
			auto const ofs = s_cast<uint32_t>(PS) >> 16;
			return *reinterpret_cast<pipe_state_field_t<PS> const*>(byte_ptr(&m_desc) + ofs);
		}

		// Convert to a pointer for passing to functions
		operator D3D12_GRAPHICS_PIPELINE_STATE_DESC const* () const
		{
			return &m_desc;
		}
	};

	// The graphics pipeline state objects of one renderer, shared by all of its windows and render steps.
	// Pipelines are found by their final description, so equal states share one pipeline. Pipelines that have not been used for 'MaxIdleTime' are
	// released when the cache grows beyond 'TrimCount'. Pipelines can also be created ahead of use on worker threads, see 'Prewarm'.
	// The cache is thread safe.
	struct PipeStateCache
	{
		using clock_t = std::chrono::steady_clock;
		static constexpr size_t TrimCount = 256;
		static constexpr auto MaxIdleTime = std::chrono::seconds(10);

		// Counters of pipeline creation, for measuring how much creation happens while drawing
		struct Stats
		{
			int64_t m_created;        // Pipelines created by 'Get', i.e. while recording draws
			double m_created_ms;      // The total time 'Get' spent creating pipelines
			double m_created_max_ms;  // The longest single creation by 'Get'
			int64_t m_prewarmed;      // Pipelines created by the prewarm workers
			double m_prewarmed_ms;    // The total worker time spent creating pipelines
			int64_t m_failed;         // Prewarm creations that failed. 'Get' reports the same failure if the state is drawn.
			int64_t m_pending;        // Prewarm requests that are queued or being created
			int64_t m_count;          // The number of cached pipelines
		};

	private:

		struct Entry
		{
			PipeStateDesc m_desc;                    // The description that 'm_pso' was created from
			D3DPtr<ID3D12RootSignature> m_signature; // Holds the root signature, so its address cannot be reused while it identifies this entry
			D3DPtr<ID3D12PipelineState> m_pso;       // The pipeline state object
			clock_t::time_point m_last_used;         // When 'Get' last returned this entry
			bool m_pinned;                           // Prewarmed and not yet used. Trimming keeps pinned entries, so prewarmed states survive until drawn.
		};
		struct Request
		{
			PipeStateDesc m_desc;                    // The description to create a pipeline for
			D3DPtr<ID3D12RootSignature> m_signature; // Holds the root signature until the pipeline is created
			uint64_t m_hash;                         // 'm_desc.Hash()'
		};

		Renderer* m_rdr;
		std::unordered_multimap<uint64_t, Entry> m_entries;
		std::deque<Request> m_requests;
		int m_in_flight;
		Stats m_stats;
		mutable std::mutex m_mutex;
		std::condition_variable_any m_cv;

		// Held shared by a worker while it creates a pipeline, and exclusively by 'Forget'. Lock order: 'm_create_lock' then 'm_mutex'.
		std::shared_mutex m_create_lock;

		// Declared last so the workers stop before any other member is destroyed
		std::vector<std::jthread> m_workers;

		// Find the entry for 'desc'. Requires 'm_mutex' to be held.
		Entry* Find(PipeStateDesc const& desc, uint64_t hash);

		// Create prewarm requests until stopped
		void Worker(std::stop_token stop);

		// Release entries that have been idle for too long. Requires 'm_mutex' to be held.
		void Trim(clock_t::time_point now);

		// Release one entry's pipeline once the GPU is no longer using it. Requires 'm_mutex' to be held.
		void Release(Entry& entry);

	public:

		explicit PipeStateCache(Renderer& rdr);
		PipeStateCache(PipeStateCache const&) = delete;
		PipeStateCache& operator=(PipeStateCache const&) = delete;

		// Return the pipeline state object for 'desc', creating it if needed. The returned pointer is valid until the GPU finishes the work submitted
		// after the next 'Forget' or trim that removes it, so it is safe to record into a command list that is submitted as usual.
		ID3D12PipelineState* Get(PipeStateDesc const& desc);

		// Create pipelines for 'descs' on worker threads, so that later 'Get' calls find them. States that are cached or already requested are ignored.
		// The root signatures and shader bytecode of 'descs' must remain valid until created or forgotten (see 'Forget'). Returns the number of new requests.
		int Prewarm(std::span<PipeStateDesc const> descs);

		// Release every pipeline that uses 'signature' or any shader stage whose bytecode starts at an address in 'bytecode', and drop matching prewarm requests.
		// Call this before the storage of the bytecode or the signature is released, so that a later allocation at the same address cannot match.
		void Forget(ID3D12RootSignature const* signature, std::span<void const* const> bytecode);

		// The number of cached pipeline state objects
		size_t Count() const;

		// The creation counters, and the number of prewarm requests not yet complete
		Stats GetStats() const;
	};
}
