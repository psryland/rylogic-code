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
		using state_t = union { uint64_t local; void* heap; };
		
		state_t m_value;  // The data that replaces the PSO description field
		uint16_t m_size;  // The size of the data in 'm_value'
		uint16_t m_align; // The alignment of the data in 'm_value'
		union {
		EPipeState m_id;  // Identifies the offset and size of the field in the PSO description.
		field_t m_field;  // Ofs/Size into the PSO description. (this is how 'EPipeState' is encoded)
		};

		PipeState() = default;

		template <typename TState> PipeState(EPipeState ps, TState const& value)
			: m_value()
			, m_size(sizeof(value))
			, m_align(alignof(TState))
			, m_id(ps)
		{
			static_assert(std::is_trivially_copyable_v<TState>, "TState must be trivially copyable");
			static_assert(sizeof(value) <=  std::numeric_limits<uint16_t>::max(), "TState is too large");
			if constexpr (sizeof(TState) <= sizeof(state_t) && alignof(TState) <= alignof(state_t))
			{
				memcpy(&m_value.local, &value, sizeof(TState));
			}
			else
			{
				m_value.heap = _aligned_malloc(m_size, m_align);
				memcpy(m_value.heap, &value, sizeof(TState));
			}
		}
		PipeState(PipeState&& rhs) noexcept
			: m_value(rhs.m_value)
			, m_size(rhs.m_size)
			, m_align(rhs.m_align)
			, m_id(rhs.m_id)
		{
			rhs.m_value.local = 0;
			rhs.m_size = 0;
		}
		PipeState(PipeState const& rhs)
			: m_value(rhs.m_value)
			, m_size(rhs.m_size)
			, m_align(rhs.m_align)
			, m_id(rhs.m_id)
		{
			if (rhs.is_local()) return;
			m_value.heap = _aligned_malloc(m_size, m_align);
			if (m_value.heap == nullptr)
				throw std::runtime_error("_aligned_malloc failed");
			
			memcpy(m_value.heap, rhs.m_value.heap, m_size);
		}
		PipeState& operator=(PipeState&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			std::swap(m_value, rhs.m_value);
			std::swap(m_size, rhs.m_size);
			std::swap(m_align, rhs.m_align);
			std::swap(m_id, rhs.m_id);
			return *this;
		}
		PipeState& operator=(PipeState const& rhs)
		{
			if (this == &rhs) return *this;
			this->~PipeState();
			new (this) PipeState(rhs);
			return *this;
		}
		~PipeState()
		{
			if (is_local()) return;
			_aligned_free(m_value.heap);
		}

		// Is the data stored locally?
		constexpr bool is_local() const
		{
			return m_size <= sizeof(state_t) && m_align <= alignof(state_t);
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
			return is_local() ? &m_value.local : m_value.heap;
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

	// Pipe state object description
	struct PipeStateDesc
	{
	private:

		// Note:
		//  - Don't set members of 'm_desc' directly otherwise
		//    the hash won't be updated and pooling won't work.
		D3D12_GRAPHICS_PIPELINE_STATE_DESC m_desc;
		int m_hash;

	public:

		PipeStateDesc()
			:m_desc()
			,m_hash(hash::HashBytes32(&m_desc, &m_desc + 1))
		{}
		PipeStateDesc(D3D12_GRAPHICS_PIPELINE_STATE_DESC const& rhs)
			:m_desc(rhs)
			,m_hash(hash::HashBytes32(&m_desc, &m_desc + 1))
		{}

		// The current hash value for this pipe state description
		int hash() const
		{
			return m_hash;
		}

		// Apply changes to the pipeline state description
		void Apply(PipeState const& ps)
		{
			// Record the change in the hash
			m_hash = hash::HashBytes32(byte_ptr(ps.value()), byte_ptr(ps.value()) + ps.size(), m_hash);

			// Set the value of a pipeline state description field
			memcpy(ps.ptr(m_desc), ps.value(), ps.size());
		}

		// Return the current value of a pipeline state member
		template <EPipeState PS> pipe_state_field_t<PS> const& Get() const
		{
			PipeState ps(PS, pipe_state_field_t<PS>{});
			return *static_cast<pipe_state_field_t<PS> const*>(ps.ptr(m_desc));
		}

		// Convert to a pointer for passing to functions
		operator D3D12_GRAPHICS_PIPELINE_STATE_DESC const* () const
		{
			return &m_desc;
		}
	};

	// A pipe state object instance
	struct PipeStateObject
	{
		D3DPtr<ID3D12PipelineState> m_pso; // The pipeline state object
		int64_t m_frame_number;            // The frame number when last used
		int m_hash;                        // Hash of the pipeline state data used to create 'm_pso'

		PipeStateObject(D3DPtr<ID3D12PipelineState> pso, int64_t frame_number, int hash)
			: m_pso(pso)
			, m_frame_number(frame_number)
			, m_hash(hash)
		{}

		// Access the allocator
		ID3D12PipelineState const* operator ->() const
		{
			return m_pso.get();
		}
		ID3D12PipelineState* operator ->()
		{
			return m_pso.get();
		}

		// Convert to the allocator pointer
		operator ID3D12PipelineState const* () const
		{
			return m_pso.get();
		}
		operator ID3D12PipelineState* ()
		{
			return m_pso.get();
		}
	};

	// A pool of pipe states
	struct PipeStatePool
	{
		using pool_t = pr::vector<PipeStateObject, 16, false>;
		Window* m_wnd;
		pool_t m_pool;

		explicit PipeStatePool(Window& wnd);
		PipeStatePool(PipeStatePool&&) = default;
		PipeStatePool(PipeStatePool const&) = delete;
		PipeStatePool& operator=(PipeStatePool&&) = default;
		PipeStatePool& operator=(PipeStatePool const&) = delete;

		// Return a pipeline state instance for the given description
		ID3D12PipelineState* Get(PipeStateDesc const& desc);
	};
}
