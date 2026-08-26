//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Private, versioned host bridge that lets an out-of-tree renderer module (currently only
// view3d-ui.dll) attach one render-callback provider to a pr::rdr12::V3dWindow without either
// module linking against the other's static/import library. Resolved dynamically via
// GetProcAddress against the four named exports below (see the Export name constants); this
// header is not part of view3d-12's public API and is never included from view3d-dll.h.
#pragma once
#include <windows.h>
#include <d3d12.h>
#include <cstdint>
#include <type_traits>

namespace pr::view3d::ui
{
	// Bridge ABI version, independent of view3d-12's own public View3D_ApiVersion. Bumped whenever
	// any exported function signature or struct layout in this header changes.
	constexpr std::uint32_t HostApiVersion = 0x00020000U;

	// Struct schema version stamped into every HostStructHeader below.
	constexpr std::uint32_t HostStructVersion = 2U;

	// Outcome of one bridge attach/detach/record operation.
	enum class EHostStatus : std::int32_t
	{
		Success = 0,
		InvalidArgument,
		InvalidStruct,
		AlreadyAttached,
		NotAttached,
		WrongThread,
		ProviderFailed,
	};

	// Identifies one of this header's versioned structures, for View3D_UIHostStructSize.
	enum class EHostStructId : std::uint32_t
	{
		Provider = 1,
		Pass = 2,
	};

	// Render stage a Provider::m_record call is being invoked for. Each value names one host
	// command list recorded at a fixed point in the frame; the host owns which list that is, every
	// barrier around it, and the resource states the provider observes.
	//   Prepare        - before any scene work, for the provider's own upload/transition-free setup.
	//   DepthTested    - scene-adjacent, into the multi-sampled scene target with the scene depth
	//                    buffer bound and depth testing available, so world UI is occluded by geometry.
	//   OcclusionFaded - after alpha resolution, with a single-sample resolved copy of the scene
	//                    depth buffer available for read-only sampling, so world UI can fade where
	//                    it is occluded without being clipped by it.
	//   Overlay        - after alpha resolution, unoccluded world-anchored UI.
	//   FinalOverlay   - the last stage before present; screen-space UI only.
	enum class EPass : std::uint32_t
	{
		Prepare = 0,
		DepthTested = 1,
		OcclusionFaded = 2,
		Overlay = 3,
		FinalOverlay = 4,
	};

	// Common header stamped at the front of every versioned bridge structure.
	struct HostStructHeader
	{
		std::uint32_t m_size;
		std::uint32_t m_version;
	};

	// The host camera for the frame being recorded, in the same right-handed convention as
	// pr::math::ProjectionPerspective/ProjectionOrthographic with righthanded == true: the camera
	// looks along m_forward (camera-space -z), and normalised device depth runs 0 at the near
	// plane to 1 at the far plane. 'm_valid' is 0 when the host has no camera this frame, in which
	// case every other field is zero and the provider must not project anything.
	// Supplied so a provider can reconstruct the host's exact projection during recording without
	// calling back into any View3D API.
	struct Camera
	{
		float m_position[3];
		float m_right[3];       // Camera-space +x in world space
		float m_up[3];          // Camera-space +y in world space
		float m_forward[3];     // Unit look direction; camera-space -z in world space
		float m_near_plane;
		float m_far_plane;
		float m_fov_y_rad;      // Vertical field of view; only meaningful when m_orthographic == 0
		float m_ortho_height;   // World units spanned vertically; only meaningful when m_orthographic != 0
		std::uint32_t m_orthographic;
		std::uint32_t m_valid;
	};

	// A single-sample, read-only copy of the host's scene depth buffer, created and owned by the
	// host and valid only for the duration of an EPass::OcclusionFaded record call. The host has
	// already transitioned it to a pixel-shader-readable state and is responsible for resolving or
	// copying the scene depth into it; the provider may create a shader resource view of
	// 'm_resource' in its own descriptor heap but must not transition, write to, or retain it.
	// 'm_resource' is null on every other pass and whenever no depth buffer exists.
	// 'm_width'/'m_height' always equal the pass's render-target extent, which lets a provider
	// address the copy directly by SV_POSITION; a provider must treat any other extent as
	// unusable rather than sampling it at the wrong texels. They also make the resource identity
	// observable across a host resize, where the same address may be reused by a new resource.
	struct ResolvedDepth
	{
		ID3D12Resource* m_resource;
		DXGI_FORMAT m_srv_format;  // The single-sample R-typed format an SRV must be created with
		std::uint32_t m_width;
		std::uint32_t m_height;
	};

	// GPU recording context for one attached pass, valid only for the duration of one
	// Provider::m_record call. The provider may set its own pipeline state, root signature,
	// viewport/scissor, vertex/index buffers, and descriptor heaps; it must not transition any
	// host resource, close or execute 'm_command_list', signal a fence, submit to a queue,
	// present, or retain any pointer from this structure beyond the call.
	// 'm_dsv' is a writable depth-stencil view only on EPass::DepthTested; on every other pass it
	// is a null handle and the provider must render without depth.
	struct Pass
	{
		HostStructHeader m_header;
		EPass m_pass;
		std::uint32_t m_reserved;
		ID3D12GraphicsCommandList* m_command_list;
		ID3D12Resource const* m_colour_target;
		ID3D12Resource const* m_depth_target;
		D3D12_CPU_DESCRIPTOR_HANDLE m_rtv;
		D3D12_CPU_DESCRIPTOR_HANDLE m_dsv;
		std::uint32_t m_width;
		std::uint32_t m_height;
		DXGI_FORMAT m_colour_format;
		DXGI_FORMAT m_depth_format;
		std::uint32_t m_sample_count;
		std::uint32_t m_sample_quality;
		std::uint64_t m_frame_number;
		float m_dpi_x;
		float m_dpi_y;
		D3D12_VIEWPORT m_viewport;
		D3D12_RECT m_scissor;
		Camera m_camera;
		ResolvedDepth m_resolved_depth;
	};

	// Invoked once per attached pass, per frame, on the window owner/render thread that attached
	// the provider. A non-Success return is surfaced by the host and cannot roll back recorded work.
	using RecordFn = EHostStatus(__stdcall*)(void* context, Pass const* pass);

	// Invoked if the host force-detaches this provider (e.g. window destruction) without the
	// attaching module having called View3D_UIHostDetach first. 'context' is the same token
	// supplied in the originating Provider; the attaching module must not call
	// View3D_UIHostDetach afterwards for this window/context pair.
	using DetachedFn = void(__stdcall*)(void* context);

	// One attached render-callback provider. 'm_context' is an opaque token the attaching module
	// supplies and receives back unmodified in every callback.
	struct Provider
	{
		HostStructHeader m_header;
		void* m_context;
		RecordFn m_record;
		DetachedFn m_detached;
	};

	// --- Exported function signatures, resolved dynamically by name from view3d-12.dll ----------
	using ApiVersionFn = std::uint32_t(__stdcall*)();
	using StructSizeFn = EHostStatus(__stdcall*)(EHostStructId struct_id, std::uint32_t* size);
	using AttachFn = EHostStatus(__stdcall*)(void* window, Provider const* provider);
	using DetachFn = EHostStatus(__stdcall*)(void* window, void* provider_context);

	// --- Export names, resolved via GetProcAddress against the already-loaded view3d-12.dll ------
	inline constexpr char const* ApiVersionExport = "View3D_UIHostApiVersion";
	inline constexpr char const* StructSizeExport = "View3D_UIHostStructSize";
	inline constexpr char const* AttachExport = "View3D_UIHostAttach";
	inline constexpr char const* DetachExport = "View3D_UIHostDetach";

	static_assert(std::is_standard_layout_v<HostStructHeader>);
	static_assert(std::is_standard_layout_v<Camera>);
	static_assert(std::is_standard_layout_v<ResolvedDepth>);
	static_assert(std::is_standard_layout_v<Pass>);
	static_assert(std::is_standard_layout_v<Provider>);
	static_assert(sizeof(HostStructHeader) == 8);
	static_assert(sizeof(Camera) == 72);
	static_assert(sizeof(ResolvedDepth) == 24);
}
