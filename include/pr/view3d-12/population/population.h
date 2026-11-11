//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Native mirror of the population contract in "pr/view3d-12/shaders/population.hlsli". See that file for the meaning of each field.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"

namespace pr::rdr12
{
	// The maximum number of LODs in a population prototype. Matches VIEW3D_POPULATION_MAX_LODS.
	inline constexpr int PopulationMaxLods = 8;

	// Flags of a population instance. Matches VIEW3D_POPULATION_FLAG_*.
	// A 'Faded' instance holds its own unorm16 coverage in bits 16-31 of the flags word (see PopulationFadeFlags).
	enum class EPopulationInstanceFlags : uint32_t
	{
		None = 0,
		Enabled = 1 << 0,
		Faded = 1 << 1,
		FadeComplement = 1 << 2,
		_flags_enum = 0,
	};

	// The bit position of the coverage of a 'Faded' instance in its flags word. Matches VIEW3D_POPULATION_FLAG_FADE_SHIFT.
	inline constexpr int PopulationFadeShift = 16;

	// Return the flags of an enabled instance that is dithered by 'coverage' in [0,1] instead of by its prototype's fade bands.
	// With 'complement', the instance keeps exactly the pixels that an object with the dither word from 'PopulationDitherWord' for the same
	// coverage and origin discards.
	inline EPopulationInstanceFlags PopulationFadeFlags(float coverage, bool complement)
	{
		// Quantise the coverage to unorm16 in the upper half of the flags word
		auto word = static_cast<uint32_t>(std::clamp(coverage, 0.0f, 1.0f) * 65535.0f + 0.5f) << PopulationFadeShift;
		auto flags = EPopulationInstanceFlags::Enabled | EPopulationInstanceFlags::Faded;
		if (complement)
			flags = flags | EPopulationInstanceFlags::FadeComplement;

		return static_cast<EPopulationInstanceFlags>(static_cast<uint32_t>(flags) | word);
	}

	// One placement of one of a population's prototypes. Matches 'View3DPopulationInstance'.
	struct PopulationInstance
	{
		// Rows of the affine instance-to-world transform: 'world[i] = Dot(o2w[i], p.w1())'.
		v4 o2w[3];

		// Multiplies the vertex colour.
		Colour32 tint;

		// See EPopulationInstanceFlags.
		EPopulationInstanceFlags flags;

		// Caller payload index, interpreted only by caller shaders.
		uint32_t payload;

		// The index of the population prototype that this instance places. Instances with an index outside the population's prototypes are not drawn.
		uint32_t prototype;
	};
	static_assert(sizeof(PopulationInstance) == 64, "Must match View3DPopulationInstance");

	// Make an instance record from an affine instance-to-world transform. 'o2w.w' must be (0,0,0,1) in its 'w' components.
	inline PopulationInstance MakePopulationInstance(m4x4 const& o2w, Colour32 tint, EPopulationInstanceFlags flags, uint32_t payload, uint32_t prototype)
	{
		// The record holds the rows of the transform, so the shader can use dot products.
		assert(o2w.x.w == 0 && o2w.y.w == 0 && o2w.z.w == 0 && o2w.w.w == 1 && "Population instance transforms must be affine");
		return PopulationInstance{
			.o2w = {
				v4{ o2w.x.x, o2w.y.x, o2w.z.x, o2w.w.x },
				v4{ o2w.x.y, o2w.y.y, o2w.z.y, o2w.w.y },
				v4{ o2w.x.z, o2w.y.z, o2w.z.z, o2w.w.z },
			},
			.tint = tint,
			.flags = flags,
			.payload = payload,
			.prototype = prototype,
		};
	}

	// Dither word fields. Match VIEW3D_POPULATION_DITHER_*.
	inline constexpr uint32_t PopulationDitherCoverageMask = 0x0000FFFFu;
	inline constexpr int PopulationDitherSeedShift = 16;
	inline constexpr uint32_t PopulationDitherSeedMask = 0x3FFFu;
	inline constexpr uint32_t PopulationDitherActive = 0x40000000u;
	inline constexpr uint32_t PopulationDitherComplement = 0x80000000u;

	// Return the dither word that an ordinary object at 'origin' (its object-to-world translation) uses to draw 'coverage' in [0,1] of its pixels.
	// The word uses the same seed as a population instance at the same origin, so the object and a 'Faded' instance with the same coverage and the
	// complement flag together cover each pixel exactly once. 'origin' must have exactly the float values of the instance translation.
	uint32_t PopulationDitherWord(v4 const& origin, float coverage, bool complement);

	// Layout of the per-view list header, for a population with 'list_count' visible lists. Match VIEW3D_POPULATION_HEADER_*.
	inline constexpr int PopulationHeaderDiag = 0;
	inline constexpr int PopulationHeaderCounts = 16;
	inline constexpr int PopulationHeaderOffsets(int list_count)
	{
		return PopulationHeaderCounts + 4 * list_count;
	}
	inline constexpr int PopulationHeaderCursors(int list_count)
	{
		return PopulationHeaderCounts + 8 * list_count;
	}
	inline constexpr int PopulationHeaderSize(int list_count)
	{
		return (PopulationHeaderCounts + 12 * list_count + 15) & ~15;
	}

	// One entry of a per-view visible list. Matches 'View3DPopulationEntry'.
	struct PopulationEntry
	{
		uint32_t source;
		uint32_t dither;
	};
	static_assert(sizeof(PopulationEntry) == 8, "Must match View3DPopulationEntry");

	// Per-draw constants of a population draw. Matches 'View3DPopulationDraw'.
	struct PopulationDraw
	{
		uint32_t list;
		uint32_t list_base;
		uint32_t offsets;
		uint32_t pad;
	};
	static_assert(sizeof(PopulationDraw) == 16, "Must match View3DPopulationDraw");

	// One level of detail of a population prototype.
	struct PopulationLod
	{
		// The geometry and materials of this LOD. Every nugget is drawn for each instance at this LOD.
		ModelPtr m_model;

		// Optional. When set, this material replaces the material of every nugget of 'm_model', in the same way as an instance material override.
		MaterialPtr m_material;

		// The minimum projected radius, in pixels, of the prototype bounding sphere at which this LOD is drawn.
		// LODs are ordered from most to least detailed, so thresholds must strictly decrease. Instances smaller than the last LOD's threshold are not drawn.
		float m_min_size;

		// The width of the transition band above 'm_min_size', as a fraction of 'm_min_size'. Zero means an abrupt switch.
		// Within [m_min_size, m_min_size * (1 + m_transition)), this LOD fades out with a dither while the next LOD (or nothing, for the last LOD) fades in.
		// Bands must not overlap the previous LOD's band: 'm_min_size * (1 + m_transition) <= previous m_min_size'.
		float m_transition;
	};

	// A distance band [m_start, m_start + m_width] in which instances fade in or out. A band with zero start and zero width is unused.
	struct PopulationFade
	{
		// The distance at which the band begins.
		float m_start;

		// The width of the band. Zero means an abrupt switch at 'm_start'.
		float m_width;

		// True if the band has any effect.
		bool used() const
		{
			return m_start + m_width > 0;
		}

		// The distance at which the band ends.
		float end() const
		{
			return m_start + m_width;
		}
	};

	// Reusable geometry, materials, and LOD policy shared by populations. Immutable after creation.
	struct PopulationPrototype : RefCounted<PopulationPrototype>
	{
		// The LODs, from most to least detailed.
		vector<PopulationLod> m_lods;

		// Instances whose bounding sphere is further than this from the scene camera are not drawn. Zero means unlimited.
		// Shadow views measure the distance from the scene camera too, so instances cast shadows only while they are within the draw distance.
		float m_draw_distance;

		// Distance bands, measured from the scene camera to each instance's origin, in which instances fade in or out with a screen-door dither.
		// Instances nearer than the start of a used 'm_fade_in' and further than the end of a used 'm_fade_out' are not drawn. A fade-in band and a
		// fade-out band with the same start and width give exactly complementary masks to instances with the same origin, so two prototypes can hand
		// instances over without gaps or double coverage. Within a band, the distance fade replaces any LOD transition dither. When either band is
		// used, the instance origin must lie inside the prototype bounding sphere, so that block bounds also bound the instance origins.
		PopulationFade m_fade_in;
		PopulationFade m_fade_out;

		// The instance-space bounding sphere of all LODs (xyz = centre, w = radius). Derived from the LOD model bounding boxes.
		v4 m_bound;

		// Validate 'lods' and create a prototype. Throws std::invalid_argument for invalid descriptions.
		PopulationPrototype(std::span<PopulationLod const> lods, float draw_distance, PopulationFade fade_in, PopulationFade fade_out);

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<PopulationPrototype>* doomed);
	};

	// A caller-owned group of instance slots with a conservative world-space bound.
	struct PopulationBlock
	{
		// World-space bounds that contain every live instance's bounding sphere, including deformation.
		BBox m_bbox;

		// The instance slots [m_first, m_first + m_capacity) owned by this block. Slots beyond the block's live count are not drawn.
		int m_first;
		int m_capacity;

		// True if the block takes part in rendering.
		bool m_active;
	};

	// Counters of one population classification. GPU counters come from the classification's list header, read back without waiting.
	struct PopulationViewStats
	{
		int64_t m_blocks_rejected;                   // Active blocks skipped by the CPU block test (frustum or draw distance).
		int64_t m_blocks_overflow;                   // Visible blocks dropped because the view exceeded 'PopulationCuller::MaxVisibleBlocks'.
		int64_t m_instances_tested;                  // Live instance slots tested by the GPU.
		int64_t m_entries_emitted;                   // Visible list entries written. An instance in a LOD transition writes two.
		int64_t m_entries_per_lod[PopulationMaxLods]; // Visible list entries of each LOD, summed over prototypes.
		int64_t m_indirect_draws;                    // Indirect draw records ((prototype, LOD, nugget) triples) of the view.
		int64_t m_live_count_violations;             // Visible blocks whose GPU live count exceeded their capacity.
	};

	// Diagnostics of a population. CPU counters are current. GPU counters are the sum, over the views that have classified the population, of each view's
	// latest classification whose results the GPU has completed, so they lag the CPU by the frames in flight.
	struct PopulationStats
	{
		int64_t m_instance_capacity;   // The number of instance slots.
		int64_t m_block_capacity;      // The number of block slots.
		int64_t m_active_blocks;       // Blocks defined by 'SetBlock' and not removed.
		int64_t m_admitted_instances;  // The sum of the active blocks' capacities: the most instances that can be live.
		int64_t m_resident_bytes;      // The size of the instance, live count, payload, and prototype buffers.
		int64_t m_scratch_bytes;       // The size of all per-view classification buffers.
		int64_t m_scratch_high_water;  // The largest value 'm_scratch_bytes' has had.
		int64_t m_upload_bytes;        // The total bytes copied to the GPU by prototype, instance, block, and payload updates.
		int64_t m_producer_dispatches; // The total number of producer dispatches recorded.
		int64_t m_views;               // The number of views with completed GPU counters.
		PopulationViewStats m_classified; // The latest completed classification counters, summed over views.
	};

	// A retained set of placements of one or more prototypes. Each instance names its prototype. Instances live in GPU buffers; the caller updates
	// ranges, not individual objects. All prototypes are classified together, so one classification per view covers every prototype.
	struct Population : RefCounted<Population>
	{
		using GfxCmdList = ::pr::compute::GfxCmdList;
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;
		using GpuViewHeap = ::pr::compute::GpuDescriptorHeap<D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV>;
		using GpuSync = ::pr::compute::GpuSync;

		// An identity-transform instance that supplies element constants for a LOD's nuggets. Population draws place vertices with the instance records.
		// 'm_material' is the LOD's material override, or null to draw each nugget with its own material.
		struct Proxy
		{
			#define PR_RDR_INST(x)\
			x(m4x4       , m_i2w     , EInstComp::I2WTransform)\
			x(ModelPtr   , m_model   , EInstComp::ModelPtr)\
			x(MaterialPtr, m_material, EInstComp::MaterialPtr)
			PR_RDR12_INSTANCE_MEMBERS(Proxy, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		// One visible list of a classification: the instances of one prototype drawn at one LOD.
		struct List
		{
			int m_prototype; // The index of the prototype in 'm_prototypes'.
			int m_lod;       // The LOD of the prototype.
		};

		// A slot of a view's diagnostics readback ring: a copy of one classification's list header, valid once 'm_fence' reaches 'm_sync_point'.
		struct ViewReadback
		{
			D3DPtr<ID3D12Fence> m_fence;    // The fence of the queue that executes the copy. Null when the slot holds no pending copy.
			uint64_t m_sync_point = 0;      // The fence value that marks the copy complete.
			int64_t m_sequence = 0;         // The view's classification number, so newer results replace older ones.
			PopulationViewStats m_cpu = {}; // The CPU counters of the classification. GPU counters are added when the copy completes.
		};
		inline static constexpr int ReadbackSlots = 3;

		// The per-view classification output: the list header, indirect arguments, and visible lists. See 'population_cull.hlsl'.
		struct ViewScratch
		{
			D3DPtr<ID3D12Resource> m_res;
			int64_t m_size = 0;
			D3DPtr<ID3D12Resource> m_readback;            // Readback buffer of 'ReadbackSlots' list headers.
			std::byte const* m_mapped = nullptr;          // Persistent CPU mapping of 'm_readback'.
			std::array<ViewReadback, ReadbackSlots> m_slots;
			int64_t m_sequence = 0;                       // The number of classifications recorded for this view.
			int64_t m_latest_sequence = 0;                // The classification number of 'm_latest'. Zero when no counters have completed.
			PopulationViewStats m_latest = {};            // The counters of the latest completed classification.
		};

		// Content work waiting for the next frame's preparation commands: a range copy, or a producer dispatch when 'm_pso' is not null.
		struct PendingWork
		{
			ID3D12Resource* m_dst;              // The copy destination. Unused by dispatches.
			int64_t m_ofs;                      // The copy destination byte offset. Unused by dispatches.
			std::vector<std::byte> m_data;      // The bytes to copy, or the producer constants.
			D3DPtr<ID3D12PipelineState> m_pso;  // The producer pipeline, or null for a copy.
			iv3 m_groups;                       // The producer thread-group counts.
		};

		Renderer* m_rdr;
		vector<PopulationPrototypePtr> m_prototypes; // Indexed by the instance prototype index.
		vector<List> m_lists;                        // The visible lists, grouped by prototype, then ordered by LOD.
		float m_block_near;                          // Blocks whose furthest point is nearer than this hold no visible instances: the nearest fade-in start over all prototypes.
		float m_block_far;                           // Blocks whose closest point is further than this hold no visible instances: the furthest draw limit over all prototypes. Infinite means unlimited.
		ESortGroup m_sort_group;                     // The sort group of every draw of the population. Replaces the group of the LOD nuggets' sort keys.
		int m_instance_capacity;                     // The number of instance slots.
		int m_payload_size;                          // The size of the caller payload buffer in bytes. Zero means no payload.
		float m_coverage;                            // The fraction of each instance's pixels that are drawn, in [0,1]. See 'SetCoverage'.
		D3DPtr<ID3D12Resource> m_instances;          // StructuredBuffer<View3DPopulationInstance>[m_instance_capacity]
		D3DPtr<ID3D12Resource> m_live_counts;        // StructuredBuffer<uint>[block capacity]
		D3DPtr<ID3D12Resource> m_payload;            // ByteAddressBuffer[m_payload_size], or null
		D3DPtr<ID3D12Resource> m_prototype_table;    // The classification constants of each prototype. See 'PrototypeConstants' in 'population_cull.hlsl'.
		std::vector<PopulationBlock> m_blocks;       // Indexed by block index. The size is the block capacity.
		std::vector<Proxy> m_proxies;                // One per list. Sized once, so instance addresses are stable.
		std::unordered_map<uint64_t, ViewScratch> m_views; // Classification scratch, keyed by consuming view.
		std::vector<PendingWork> m_pending;          // Uploads and producer dispatches recorded by the next 'Flush', in submission order.
		std::vector<uint8_t> m_produce_cs;           // The bytecode of the cached producer pipeline.
		D3DPtr<ID3D12RootSignature> m_produce_sig;   // The producer root signature. See 'population.hlsli' for the bindings.
		D3DPtr<ID3D12PipelineState> m_produce_pso;   // The pipeline for the most recent producer bytecode.
		uint64_t m_revision;                         // Incremented by every content change, for cache invalidation.
		int64_t m_scratch_bytes;                     // The size of all view buffers.
		int64_t m_scratch_high_water;                // The largest value of 'm_scratch_bytes'.
		int64_t m_upload_bytes;                      // The total bytes copied by 'Flush'.
		int64_t m_producer_dispatches;               // The total dispatches recorded by 'Flush'.
		std::mutex m_mutex;

		// Create a population of 'prototypes' with fixed instance, block, and payload capacities. Instances select a prototype by its index in
		// 'prototypes'. Resources are created up front, so admission fails here, not mid-frame. 'sort_group' orders the population's draws relative to
		// the scene's other draws, as for objects: opaque groups are drawn in increasing order, and draws in a lower group can hide the pixels of later ones.
		Population(Renderer& rdr, std::span<PopulationPrototypePtr const> prototypes, int instance_capacity, int block_capacity, int payload_size, ESortGroup sort_group);
		Population(Population const&) = delete;
		Population& operator =(Population const&) = delete;
		~Population();

		// The number of block slots.
		int BlockCapacity() const;

		// The size of the list header of a classification.
		int HeaderSize() const;

		// Define block 'index'. 'live_count' instances starting at 'first' are drawn. Throws std::out_of_range for invalid ranges.
		void SetBlock(int index, BBox const& bbox, int first, int capacity, int live_count);

		// Stop drawing block 'index'.
		void RemoveBlock(int index);

		// Draw a fraction 'coverage', in [0,1], of the pixels of every instance with a screen-door dither, in every view. One draws instances in full,
		// and zero draws nothing. Within LOD transitions and fade bands, each entry keeps 'coverage' of its own pixels, so complementary entries still
		// never cover a pixel twice. Throws std::invalid_argument for values outside [0,1].
		void SetCoverage(float coverage);

		// Call 'fn' with the bounds of each active block, and return the content revision consistent with those blocks.
		uint64_t ActiveBlocks(std::function<void(BBox const&)> fn);

		// Copy 'instances' into slots [first, first + instances.size()). The data is copied during this call.
		void UpdateInstances(int first, std::span<PopulationInstance const> instances);

		// Copy 'data' into the payload buffer at byte offset 'ofs'. The data is copied during this call.
		void UpdatePayload(int64_t ofs, std::span<uint8_t const> data);

		// Queue a dispatch of the compute shader 'cs' that writes instances, block live counts, and payload on the GPU. 'constants' (1 to 65536 bytes)
		// are copied during this call. The dispatch runs in the next frame, in order with other updates, before any consumer reads the population.
		// See 'population.hlsli' for the bindings. Writes outside the buffers are discarded.
		void Produce(std::span<uint8_t const> cs, std::span<std::byte const> constants, iv3 groups);

		// Record the pending uploads and producer dispatches into 'cmd_list'. Call before any consumer of this frame reads the population.
		// Producer dispatches bind their buffers through descriptors in 'heap', which this call sets on 'cmd_list'.
		void Flush(GfxCmdList& cmd_list, GpuUploadBuffer& upload, GpuViewHeap& heap);

		// Return the classification scratch buffer for 'view_key', growing it to at least 'size' bytes.
		ID3D12Resource* ViewBuffer(uint64_t view_key, int64_t size);

		// Create the classification scratch buffers of 'view_keys', of at least 'size' bytes, and their diagnostics readback rings, before the
		// classifications that use them. Creating these resources is slow, so a caller can do it at a time it chooses. Existing resources are kept.
		void ReserveViews(std::span<uint64_t const> view_keys, int64_t size);

		// Record a copy of the list header of 'view_key's classification into the view's readback ring. 'scratch' must be in the copy source state.
		// 'gsync' is the timeline that will execute 'cmd_list', and 'cpu' holds the CPU counters of the classification. Nothing waits for the GPU.
		void ReadBackDiagnostics(GfxCmdList& cmd_list, GpuSync const& gsync, uint64_t view_key, PopulationViewStats const& cpu);

		// Return the current diagnostics. GPU counters include only classifications that the GPU has already completed.
		PopulationStats Stats();

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<Population>* doomed);

	private:

		// Queue a copy of 'data' to 'dst' at byte offset 'ofs'.
		void Enqueue(ID3D12Resource* dst, int64_t ofs, std::span<std::byte const> data);

		// Publish readback slots whose copies have completed. Callers hold 'm_mutex'.
		void PollDiagnostics();

		// Return the scratch buffer of 'view', growing it to at least 'size' bytes. Callers hold 'm_mutex'.
		ID3D12Resource* GrowViewBuffer(ViewScratch& view, int64_t size);

		// Create the diagnostics readback ring of 'view' if it has none. Callers hold 'm_mutex'.
		void CreateReadback(ViewScratch& view);
	};

	// The consuming view of a population classification.
	struct PopulationViewDesc
	{
		// A key that identifies the classification's scratch buffer across frames, so the buffer is reused.
		// Classifications of one population that are recorded before their draws need different keys.
		uint64_t m_key;

		// World to screen (clip) transform. Supplies the frustum side planes.
		m4x4 m_w2s;

		// The world-space projection origin, used for projected sizes. Unused by orthographic projections.
		v4 m_eye;

		// The world-space point that the prototype draw distance is measured from.
		v4 m_range_origin;

		// Projected radius in pixels = radius * size_scale / distance (perspective), or radius * size_scale (orthographic).
		float m_size_scale;

		// True for orthographic projections.
		bool m_orthographic;
	};

	// One indirect draw of a classified population: a (prototype, LOD, nugget) triple.
	// Each draw has an indexed record over the nugget's index range and a non-indexed record over its vertex range, so any fill mode can be drawn.
	struct PopulationViewDraw
	{
		Population::Proxy const* m_proxy; // The proxy instance of the list.
		Nugget const* m_nugget;           // The nugget to draw.
		int m_prototype;                  // The prototype of the draw.
		int m_lod;                        // The LOD of the draw.
		int m_list;                       // The visible list that the draw renders.
		int64_t m_args_indexed;           // Byte offset of the indexed argument record in 'm_scratch'.
		int64_t m_args_vertex;            // Byte offset of the non-indexed argument record in 'm_scratch'.
	};

	// The result of classifying a population for one view.
	struct PopulationViewLists
	{
		Population const* m_population;
		ID3D12Resource* m_scratch;  // The view buffer. In INDIRECT_ARGUMENT | shader resource state after classification.
		uint32_t m_list_base;       // Byte offset of the first list entry in 'm_scratch'.
		uint32_t m_offsets;         // Byte offset of the list offsets array of the header in 'm_scratch'.
		vector<PopulationViewDraw> m_draws;
	};

	// Records GPU visibility and LOD classification of populations. Owned by the renderer.
	struct PopulationCuller
	{
		using GfxCmdList = ::pr::compute::GfxCmdList;
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;
		using GpuSync = ::pr::compute::GpuSync;

		// The size of an indirect argument record. Non-indexed records are padded to the indexed size.
		inline static constexpr int ArgStride = 20;

		// Limits of a single classification (the dispatch grid is 'block capacity' x 'visible blocks').
		inline static constexpr int MaxVisibleBlocks = 65535;
		inline static constexpr int MaxBlockCapacity = 65535 * 64;

	private:

		D3DPtr<ID3D12RootSignature> m_signature;
		D3DPtr<ID3D12PipelineState> m_pso_clear;
		D3DPtr<ID3D12PipelineState> m_pso_count;
		D3DPtr<ID3D12PipelineState> m_pso_prefix;
		D3DPtr<ID3D12PipelineState> m_pso_scatter;
		D3DPtr<ID3D12CommandSignature> m_draw_indexed;
		D3DPtr<ID3D12CommandSignature> m_draw;

	public:

		explicit PopulationCuller(Renderer& rdr);

		// Classify 'pop' for 'view', recording compute work into 'cmd_list'. 'step' selects the nuggets that have a material pass for that render step.
		// 'gsync' is the timeline that executes 'cmd_list'; it dates the diagnostics readback. Leaves the compute root signature and PSO changed.
		PopulationViewLists Cull(GfxCmdList& cmd_list, GpuUploadBuffer& upload, GpuSync const& gsync, Population& pop, PopulationViewDesc const& view, ERenderStep step);

		// The size of the classification scratch buffer that 'Cull' uses for 'pop' and 'step'.
		int64_t ViewSize(Population const& pop, ERenderStep step) const;

		// The command signature for an indexed or non-indexed draw.
		ID3D12CommandSignature* DrawSignature(bool indexed);
	};

	// Throw if 'prototype' or 'population' is null.
	void Validate(PopulationPrototype const* prototype);
	void Validate(Population const* population);

	// Return the nuggets of a LOD that draw for 'step': the default dependents of each top-level nugget, that are not hidden and have a material pass.
	// 'material' is the LOD's material override, or null to use each nugget's own material. 'fn' is called with each nugget and the material it draws with.
	// This is a template so callers' capturing lambdas are not wrapped in an allocating 'std::function'.
	template <typename Fn> void PopulationNuggets(Model const& model, Material const* material, ERenderStep step, Fn&& fn)
	{
		// Populations draw the default variant of each nugget, in the same way as an instance without overrides.
		// Nuggets that do not cast shadows are skipped in the shadow pass, as they are for ordinary instances.
		for (auto& nug : Enumerate(model.m_nuggets))
		{
			for (auto const& dep : nug.Dependents())
			{
				if (dep.m_variant != DefaultNugget)
					continue;
				if (AllSet(dep.m_nflags, ENuggetFlag::Hidden))
					continue;
				if (step == ERenderStep::ShadowMap && AllSet(dep.m_nflags, ENuggetFlag::ShadowCastExclude))
					continue;

				auto const& mat = material != nullptr ? *material : dep.mat();
				if (mat.Pass(step) == nullptr)
					continue;

				fn(dep, mat);
			}
		}
	}
	template <typename Fn> void PopulationNuggets(PopulationLod const& lod, ERenderStep step, Fn&& fn)
	{
		// A LOD draws its model with its optional material override
		PopulationNuggets(*lod.m_model.get(), lod.m_material.get(), step, std::forward<Fn>(fn));
	}
	template <typename Fn> void PopulationNuggets(Population::Proxy const& proxy, ERenderStep step, Fn&& fn)
	{
		// A proxy draws its LOD's model with the LOD's optional material override
		PopulationNuggets(*proxy.m_model.get(), proxy.m_material.get(), step, std::forward<Fn>(fn));
	}

	// Return the overlay whose vertex shader 'material' applies for 'step', or null if the stock vertex shader is used.
	// Population admission only accepts procedural overlays here, and their vertex shaders place population vertices themselves (see population.hlsli).
	Shader const* PopulationVertexOverlay(Material const& material, ERenderStep step);

	// Describe the view of 'cam' rendered into a viewport 'viewport_height' pixels high.
	PopulationViewDesc PopulationCameraView(uint64_t key, Camera const& cam, float viewport_height);

	// Describe a view with world to clip transform 'w2s' (a perspective or orthographic projection of an orthonormal view) rendered into a viewport
	// 'viewport_height' pixels high. The draw distance is measured from 'range_origin'.
	PopulationViewDesc PopulationProjectionView(uint64_t key, m4x4 const& w2s, float viewport_height, v4 range_origin);
}
