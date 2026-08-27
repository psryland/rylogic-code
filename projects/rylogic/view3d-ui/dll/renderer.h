//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// DLL-internal D3D12 screen-space retained renderer for one attached UI context (implementation-
// plan.md section 4.4/9). One instance is owned per ContextSlot (context.h), constructed from the
// AddRef'd external ID3D12Device supplied to View3DUI_ContextCreate and destroyed before that
// device reference is released. This header (and renderer.cpp) is, alongside host_bridge.h/.cpp,
// one of only two pairs of files in this module permitted to include the private view3d-12 UI
// host bridge (pr/view3d-12/view3d-ui-bridge.h) and therefore <d3d12.h>; every other view3d-ui
// header/source stays dependency-minimal per implementation-plan.md section 2.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/engine.h"
#include "pr/view3d-12/view3d-ui-bridge.h"
#include "../src/glyph_cache.h"
#include "../src/text_shaper.h"
#include <d3d12.h>

namespace pr::view3d::ui
{
	// The compiled HLSL for every pipeline variant, retained so a variant first needed mid-frame
	// (the depth-tested pair, whose sample count is only known once a host pass supplies it) is
	// created without recompiling shaders.
	struct ShaderBlobs
	{
		Microsoft::WRL::ComPtr<ID3DBlob> vs;
		Microsoft::WRL::ComPtr<ID3DBlob> ps_box;
		Microsoft::WRL::ComPtr<ID3DBlob> ps_glyph;
		Microsoft::WRL::ComPtr<ID3DBlob> ps_box_faded;
		Microsoft::WRL::ComPtr<ID3DBlob> ps_glyph_faded;
	};

	// One box/glyph pipeline pair sharing a render-target, depth and multi-sample configuration.
	// A pass selects exactly one variant; nothing else about a draw depends on the host stage.
	struct PipelineVariant
	{
		Microsoft::WRL::ComPtr<ID3D12PipelineState> box;
		Microsoft::WRL::ComPtr<ID3D12PipelineState> glyph;
	};

	// The values every draw within one root's subtree shares. Screen roots leave all of these at
	// zero, which is exactly the state the M0-M5 screen pipelines already assumed.
	struct GroupState
	{
		float clip_depth;
		float view_depth;
		float min_opacity;
		float fade_depth;
		float depth_bias;
		float near_plane;
		float far_plane;
		float orthographic;
	};

	// Screen-space retained renderer that turns one context's immutable DrawPacket into D3D12
	// draw calls recorded into the host's own command list, entirely within the host-bridge pass
	// callbacks it is given (host_bridge.cpp's RecordThunk). Every device/pipeline/atlas
	// resource this class owns is created lazily, on the first real EPass::Prepare call,
	// never in the constructor: the constructor must be provably non-throwing so it can
	// participate in View3DUI_ContextCreate's all-or-nothing rollback ordering (view3d-ui.cpp),
	// and a headless context (one never attached to a window, e.g. every ABI/logic unit test)
	// never drives a single Record call, so its Renderer's device/pipeline state is simply never
	// realised at all.
	class Renderer
	{
		// Non-owning: 'm_device' is the same borrowed IUnknown* the owning ContextSlot AddRef'd
		// once at context creation. ContextSlot destroys its Renderer strictly before releasing
		// that reference (context.h), so this class never needs its own AddRef/Release pair.
		IUnknown* m_device;
		Config m_config;

		// Realised lazily by EnsureDeviceResources() on the first Prepare call; every field below
		// stays default/empty for a Renderer that is constructed but never actually driven by the
		// host bridge (the common case in every unit test, which never supplies a window).
		bool m_device_lost = false;
		Microsoft::WRL::ComPtr<ID3D12Device> m_d3d_device;
		Microsoft::WRL::ComPtr<ID3D12RootSignature> m_root_signature;
		ShaderBlobs m_shaders;
		PipelineVariant m_pso_overlay;
		PipelineVariant m_pso_faded;
		Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srv_heap;
		std::unique_ptr<TextShaper> m_text_shaper;
		std::unique_ptr<GlyphCache> m_glyph_cache;

		// The depth-tested variant is created on demand because its sample count and depth format
		// are properties of the host's scene target, which is only described to this renderer once
		// a depth-tested pass actually runs, and can change across a device/format reconfiguration.
		PipelineVariant m_pso_depth;
		DXGI_FORMAT m_pso_depth_format = DXGI_FORMAT_UNKNOWN;
		std::uint32_t m_pso_depth_sample_count = 0;
		std::uint32_t m_pso_depth_sample_quality = 0;

		// The colour format every pipeline variant is built against, captured from the first
		// Prepare pass so a later lazily-created variant matches the variants created with it.
		DXGI_FORMAT m_colour_format = DXGI_FORMAT_UNKNOWN;

		// The host-owned resolved depth buffer the occlusion-faded shaders sample, plus the view
		// description the current SRV in heap slot 1 was created from. A view is not a resource
		// transition, so creating it here respects the bridge's rule that a provider never
		// transitions or takes ownership of a host resource. The extents are part of the key, not
		// just diagnostics: the host releases and recreates this resource across a resize and the
		// COM allocator may return the same address, so a pointer-and-format key alone can leave
		// slot 1 describing a destroyed resource.
		ID3D12Resource* m_resolved_depth = nullptr;
		DXGI_FORMAT m_resolved_depth_format = DXGI_FORMAT_UNKNOWN;
		std::uint32_t m_resolved_depth_width = 0;
		std::uint32_t m_resolved_depth_height = 0;
		std::uint64_t m_resolved_depth_view_epoch = 0;

		// The glyph atlas texture array (one array slice per GlyphCache page) and one persistently-
		// mapped upload buffer per slice, kept as a 1:1 CPU-side mirror of that slice's committed
		// pixels for the Renderer's entire lifetime (see renderer.cpp's EnsureGlyphPage for why
		// this makes re-uploading always safe without a ring allocator). Both vectors are indexed
		// by page/slice and never shrink; their length is bounded by Config::max_glyph_cache_pages.
		Microsoft::WRL::ComPtr<ID3D12Resource> m_glyph_atlas;
		std::vector<Microsoft::WRL::ComPtr<ID3D12Resource>> m_glyph_uploads;
		std::vector<std::uint8_t*> m_glyph_upload_mapped;
		std::uint32_t m_glyph_atlas_dim_px = 0;
		std::uint32_t m_glyph_atlas_pages_capacity = 0;

		// TextShaper::FontKey of every (family, size) request PrepareText found unresolvable this
		// frame. Rebuilt from scratch each Prepare, so a font installed between frames is picked
		// up naturally; bounded by the number of distinct font requests in one packet.
		std::unordered_set<std::uint64_t> m_missing_fonts;

	public:
		// Stores the borrowed device pointer and a copy of 'config''s bounded capacities only;
		// never queries an interface, creates a device object, or touches DirectWrite, so this
		// constructor can never throw or fail (see the class comment above).
		Renderer(IUnknown* device, Config const& config) noexcept;
		~Renderer();

		Renderer(Renderer const&) = delete;
		Renderer& operator=(Renderer const&) = delete;
		Renderer(Renderer&&) = delete;
		Renderer& operator=(Renderer&&) = delete;

		// Handle one host-bridge Provider::m_record call for 'pass' against the context's current
		// 'packet'. Prepare creates resources and uploads glyphs; each drawing pass records only
		// the packet groups whose root policy that pass owns, so a root is drawn exactly once per
		// frame, in the host stage its policy names. Returns EStatus::DeviceLost once
		// ID3D12Device::GetDeviceRemovedReason first reports a removal, and every subsequent call
		// thereafter without attempting further GPU work.
		EStatus Record(Pass const& pass, DrawPacket const& packet);

		// The root policy whose groups 'pass' is responsible for drawing. Pure/static so the pass
		// selection rule is directly testable without a device.
		static ERootPolicy PolicyForPass(EPass pass);

		// Resolves the DIP x-coordinate a TextPresenter glyph run must start at within 'item's own
		// bounds, given the run's already-shaped 'total_advance_dip' (TextShaper::Shape's return
		// value): ETextAlign::Left starts at bounds.x + text_inset_dip; ETextAlign::Center centers
		// the run within bounds.w and ignores text_inset_dip. Pure/static so it is directly testable
		// without a live TextShaper or D3D12 device.
		static float TextRunStartXDip(DrawItem const& item, float total_advance_dip);

		// The pixel extent the vertex shader divides viewport-local rect positions by to reach
		// normalised device coordinates. This is deliberately the D3D viewport rather than the
		// render target: every rect reaching the renderer is already local to the viewport, and
		// RSSetViewports then applies the viewport's origin and extent exactly once. Dividing by
		// the target instead would apply the viewport scale a second time. A degenerate viewport
		// falls back to the target extent so a division by zero can never reach the shader.
		// Pure/static so it is directly testable without a D3D12 device.
		static Vec2 NdcDivisorPx(Pass const& pass);

		// Number of times an occlusion-faded pass has (re)created the resolved-depth SRV in heap
		// slot 1. Exposed so a test can prove the descriptor is rebuilt when the host recreates the
		// resource and reused when it does not; it carries no rendering meaning.
		std::uint64_t ResolvedDepthViewEpoch() const;

	private:
		// Creates the device/root-signature/PSOs/glyph atlas on first use only, sized against
		// 'colour_format' (the render target format the host's Prepare pass reports); a no-op on
		// every later call once realised. Every fallible step is built into a purely local
		// variable and committed to this instance's members only once every step has succeeded,
		// so a failure partway through (e.g. shader compilation failing after the root signature
		// already succeeded) can never leave this Renderer half-initialised - the next Prepare
		// call simply retries EnsureDeviceResources from scratch. Throws pr::view3d::ui::EngineException
		// on an unrecoverable setup failure, which Record translates to an EStatus rather than
		// letting it escape.
		void EnsureDeviceResources(DXGI_FORMAT colour_format);

		// Each of the following builds one independent group of resources purely from its
		// arguments (never reading or writing *this' members), so EnsureDeviceResources can freely
		// discard every local on a later failure without any member state having been touched yet.
		static Microsoft::WRL::ComPtr<ID3D12RootSignature> CreateRootSignature(ID3D12Device* device);
		static ShaderBlobs CompileShaders();
		static PipelineVariant CreatePipelineVariant(ID3D12Device* device, ID3D12RootSignature* root_signature, ShaderBlobs const& shaders, DXGI_FORMAT colour_format, DXGI_FORMAT depth_format, std::uint32_t sample_count, std::uint32_t sample_quality, bool faded);
		static void CreateGlyphAtlasResources(ID3D12Device* device, std::uint32_t atlas_dim_px, std::uint32_t pages_capacity, Microsoft::WRL::ComPtr<ID3D12Resource>& out_atlas, Microsoft::WRL::ComPtr<ID3D12DescriptorHeap>& out_srv_heap);

		// Creates (or recreates after a target reconfiguration) the depth-tested pipeline variant
		// matching 'pass's scene target, and returns it.
		PipelineVariant const& EnsureDepthPipeline(Pass const& pass);

		// Points heap slot 1 at 'pass's host-owned resolved depth buffer, recreating the view only
		// when the resource or its format changes. Returns false when the host offered no resolved
		// depth this frame, in which case the occlusion-faded pass records nothing rather than
		// sampling an undefined resource.
		bool EnsureResolvedDepthView(Pass const& pass);

		// Ensures every text DrawItem's glyphs are shaped, rasterized, and resident in the glyph
		// atlas (uploading any newly-placed glyph's coverage bitmap via 'command_list'), so the
		// later Draw call for the same frame never needs to touch DirectWrite or issue a texture
		// upload. Sets 'out_hit_limit' true if any glyph could not be placed because every atlas
		// page and the configured budget are both already full; that glyph is simply omitted from
		// this frame's Draw call rather than aborting the rest of the frame. Sets
		// 'out_missing_asset' true if any item requested a font family that is not installed;
		// those items are recorded in m_missing_fonts and skipped for the rest of the frame rather
		// than being drawn in a substitute face. Only called from Prepare.
		void PrepareText(ID3D12GraphicsCommandList* command_list, DrawPacket const& packet, bool& out_hit_limit, bool& out_missing_asset);

		// Records the deterministic pre-order draw call sequence for every group in 'packet' whose
		// root policy is 'policy', using 'variant's pipelines and the glyph placements PrepareText
		// already resolved for this same frame. 'bind_depth' binds the host's depth-stencil view so
		// scene geometry can reject depth-tested world UI. Never creates or uploads a resource.
		void Draw(Pass const& pass, DrawPacket const& packet, ERootPolicy policy, PipelineVariant const& variant, bool bind_depth);

		// Lazily creates and permanently maps 'page''s persistent upload buffer the first time
		// that page is touched (a no-op on every later call for the same page); every subsequent
		// glyph upload into this page reuses that same mapped pointer for the Renderer's whole
		// lifetime, which is what makes uploading a glyph safe without waiting for the GPU to have
		// executed any earlier copy out of it (see the class comment on m_glyph_uploads).
		void EnsureGlyphUploadBuffer(std::uint32_t page);

		// Writes 'bitmap''s coverage bytes into 'slot''s permanently-assigned offset inside its
		// page's upload buffer, then records a CopyTextureRegion from that offset into the glyph
		// atlas texture at the same slot. Only called once per glyph, the first time it is placed.
		void UploadGlyphBitmap(GlyphSlot const& slot, GlyphBitmap const& bitmap, ID3D12GraphicsCommandList* command_list);

		// Records one solid/rounded/border box quad (item.primitive is one of those three closed
		// values) via the box PSO and root constants only; never allocates a vertex/index buffer.
		void DrawBoxItem(Pass const& pass, GroupState const& group, DrawItem const& item, float dpi_scale, float viewport_w, float viewport_h);

		// Re-shapes 'item.text' (cheap: cached font face, no rasterization) purely to recover each
		// glyph's pen position for this frame, then records the item's selection highlight and
		// composition underline, one glyph-atlas-sampling quad per already-resident glyph, and its
		// caret - in that order, so the caret and glyphs always paint over the highlight. Every
		// rectangle is derived from the same shaped layout as the glyphs, so a caret or selection
		// can never disagree with what is drawn, including in bidirectional text. A glyph
		// PrepareText could not place this frame (ResourceLimit) is simply skipped, matching the
		// bounded-degradation contract, and an item whose font family is missing draws nothing.
		// 'bound' is the primitive whose pipeline is currently set; the primitive left bound on
		// exit is returned, because one text item may bind both the box and glyph pipelines.
		EVisualPrimitive DrawTextItem(Pass const& pass, GroupState const& group, DrawItem const& item, PipelineVariant const& variant, EVisualPrimitive bound, float dpi_scale, float viewport_w, float viewport_h);

		// True once ID3D12Device::GetDeviceRemovedReason first reports a non-S_OK removal; latches
		// m_device_lost so every later Record call short-circuits without touching the GPU again.
		bool CheckDeviceLost();
	};
}
