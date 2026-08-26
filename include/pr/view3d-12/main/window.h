//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/frame.h"
#include "pr/view3d-12/openxr/openxr.h"
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/render/alpha_kbuffer.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/utility/diagnostics.h"

namespace pr::rdr12
{
	struct WindowBase
	{
		// Notes:
		//  - This class is used to ensure types in 'Window' are destructed before types in here
		//    This allows reference counting to check that there are no outstanding references without
		//    having to early release everything.
		Renderer*                    m_rdr;              // The owning renderer
		HWND                         m_hwnd;             // The window handle this window is bound to
		DummyWindow                  m_hwnd_dummy;       // A dummy window handle for debug and message queues
		DXGI_SWAP_CHAIN_FLAG         m_swap_chain_flags; // Options to allow GDI and DX together (see DXGI_SWAP_CHAIN_FLAG)
		D3DPtr<IDXGISwapChain>       m_swap_chain_dbg;   // A swap chain bound to the dummy window handle for debugging
		D3DPtr<IDXGISwapChain3>      m_swap_chain;       // The swap chain bound to the window handle
		D3DPtr<ID3D12DescriptorHeap> m_rtv_heap;         // Render target view descriptors for the swap-chain
		D3DPtr<ID3D12DescriptorHeap> m_dsv_heap;         // Depth stencil view descriptor for the swap-chain
		D3DPtr<ID2D1DeviceContext>   m_d2d_dc;           // The device context for D2D

		WindowBase(Renderer& rdr, WndSettings const& settings);
		~WindowBase();
	};
	struct Window : WindowBase
	{
		// Notes:
		//  - A window wraps an HWND and contains a SwapChain.
		//  - The stuff visible in a window is governed by one or more scenes.
		//  - A window where HWND = nullptr, is used for rendering to off-screen render targets only.
		//    So parallel command list building requires multiple command allocators.
		//  - Command allocators can only be reset when they are not used by the GPU any more.
		//  - The swap chain does not have a depth stencil resource, it's managed by the window.

		using ClearValue = ::pr::compute::ClearValue;
		using MultiSamp = ::pr::compute::MultiSamp;
		using RTProps = ClearValue;
		using DSProps = ClearValue;
		using BackBuffers = pr::vector<BackBuffer, 4, false>;
		using CmdLists = pr::vector<ID3D12CommandList*, 4, false>;
		using GpuViewHeap = ::pr::compute::GpuDescriptorHeap<D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV>;
		using GpuSampHeap = ::pr::compute::GpuDescriptorHeap<D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER>;
		using GpuSync = ::pr::compute::GpuSync;
		using GfxCmdAllocPool = ::pr::compute::GfxCmdAllocPool;
		using GfxCmdListPool = ::pr::compute::GfxCmdListPool;
		using GfxCmdList = ::pr::compute::GfxCmdList;
		using ResStateStore = ::pr::compute::ResStateStore;
		using OpenXRPtr = std::unique_ptr<openxr::OpenXR>;

		GpuSync                      m_gsync;            // GPU fence for frames
		BackBuffers                  m_swap_bb;          // Back buffer render targets from the swap chain.
		BackBuffer                   m_msaa_bb;          // The MSAA back buffer render target
		OpenXRPtr                    m_open_xr;          // OpenXR support (if not null)
		int                          m_bb_index;         // The current back buffer index
		RTProps                      m_rt_props;         // The properties of the MSAA back buffer
		DSProps                      m_ds_props;         // The properties of the depth stencil buffer
		GfxCmdAllocPool              m_cmd_alloc_pool;   // A pool of command allocators
		GfxCmdListPool               m_cmd_list_pool;    // A pool of command lists
		GpuViewHeap                  m_heap_view;        // Shader visible heap for CBV/SRV/UAV
		GpuSampHeap                  m_heap_samp;        // Shader visible heap for Samplers
		ResStateStore                m_res_state;        // Tracks the resource state of render targets and depth stencil resources
		AlphaKBuffer                 m_alpha_kbuffer;    // 1x K-buffer resources used for alpha sorting
		Frame                        m_frame;            // Class for managing the rendering of a frame (for reuse)
		DiagState                    m_diag;             // Diagnostic variables
		int64_t                      m_frame_number;     // The number of times 'RenderFrame' has been called.
		UINT                         m_vsync;            // Present SyncInterval value
		bool                         m_idle;             // True while the window is occluded
		string32                     m_name;             // A debugging name for the window

		Window(Renderer& rdr, WndSettings const& settings);

		// Access the renderer manager classes
		ID3D12Device4* d3d() const;
		Renderer& rdr() const;

		// Return the current DPI for this window. Use DIPtoPhysical(pt, Dpi()) for converting points
		v2 Dpi() const;

		// The current back buffer index
		int BBIndex() const;
		int BBCount() const;

		// The number of times 'RenderFrame' has been called
		int64_t FrameNumber() const;

		// Get/Set the window background colour / clear value
		Colour BkgdColour() const;
		void BkgdColour(Colour const& colour);

		// Get/Set the size of the back buffer
		iv2 BackBufferSize() const;
		void BackBufferSize(iv2 size, bool force, MultiSamp const* multisamp = nullptr);

		// Get/Set whether the swap chain owns an output in DXGI exclusive fullscreen mode
		bool FullScreen() const;
		void FullScreen(bool fullscreen);

		// Get/Set the multi sampling used. Changing the multi-sampling is like resizing the MSAA back buffer only.
		MultiSamp MultiSampling() const;
		void MultiSampling(MultiSamp ms);
		
		// Replace the swap chain buffers with new ones
		void CustomSwapChain(std::span<BackBuffer> back_buffers);
		void CustomSwapChain(std::span<Texture2D*> back_buffers);

		// Start rendering a new frame. Returns an object that scenes can render into
		Frame& NewFrame();

		// Present the frame to the display
		void Present(Frame& frame, EGpuFlush flush = EGpuFlush::Async);

		// Wait for the GPU to finish rendering the current frame. Use before shutdown
		void WaitForGpu();

		// Create an MSAA render target and depth stencil
		BackBuffer CreateRenderTarget(iv2 size, MultiSamp ms, ClearValue rt_clear, ClearValue ds_clear);

		// A single-sample, read-only copy of 'bb's depth buffer, owned by this window and recreated
		// on demand when the back buffer size or depth format changes. Returns null when 'bb' has no
		// depth buffer. The returned resource is only meaningful after RecordDepthResolve has
		// recorded the resolve/copy for the current frame.
		ID3D12Resource* ResolvedDepth(BackBuffer const& bb);

		// The single-sample R-typed format a shader resource view of ResolvedDepth must use.
		DXGI_FORMAT ResolvedDepthSrvFormat() const;

		// Record the multi-sample resolve (or plain copy) of 'bb's depth buffer into the window's
		// resolved-depth resource, leaving it in a pixel-shader-readable state. The window owns
		// every barrier involved; callers only choose when in the frame this happens.
		void RecordDepthResolve(GfxCmdList& cmd_list, BackBuffer const& bb);

	private:

		D3DPtr<ID3D12Resource> m_resolved_depth; // Single-sample copy of the scene depth buffer, created on demand
		iv2 m_resolved_depth_size;               // Dimensions 'm_resolved_depth' was created with
		DXGI_FORMAT m_resolved_depth_format;     // Depth format 'm_resolved_depth' was created from

		// Create the swap chain back buffers
		void CreateSwapChain(iv2 size);
	};
}

#if 0 // todo
		// Draw text directly to the back buffer
		void DrawString(wchar_t const* text, float x, float y);

		// Set the viewport to all of the render target
		void RestoreFullViewport();

		// Get/Set full screen mode
		// Don't use the automatic alt-enter system, it's too uncontrollable
		// Handle WM_SYSKEYDOWN for VK_RETURN, then call FullScreenMode
		bool FullScreenMode() const;
		void FullScreenMode(bool on, DisplayMode mode);

		// The display mode of the main render target
		DXGI_FORMAT DisplayFormat() const;

		// Release all references to the swap chain to allow it to be created or resized.
		void RebuildRT(std::function<void(ID3D11Device*)> work);

		// Signal the start and end of a frame.
		// A frame can be any number of scenes rendered into the back buffer.
		void FrameBeg();
		void FrameEnd();
		auto FrameScope()
		{
			return Scope(
			[this] { FrameBeg(); },
			[this] { FrameEnd(); });
		}

		// Rendering:
		//  For each scene to be rendered:
		//     Build/Update the draw list for that scene
		//     Set the scene viewport
		//     Render the drawlist
		//
		// Rendering a Drawlist:
		//   Deferred using: http://www.catalinzima.com/tutorials/deferred-rendering-in-xna/creating-the-g-buffer/
		//
		// Drawlist Order:
		//   opaques
		//   sky box
		//   alphas
		//
		// Observations:
		//   Only immediate context needed for normal rendering,
		//   Deferred context might be useful for generating shadow data (dunno yet)
		//
		// Call Present() to present the scene to the display.
		//   From DirectX docs: To enable maximal parallelism between the CPU and the graphics
		//   accelerator, it is advantageous to call RenderEnd() as far ahead of calling Present()
		//   as possible. 'BltBackBuffer()' can be used to redraw the display from the last back
		//   buffer but this only works for D3DSWAPEFFECT_COPY.
		void Present();
		#endif
