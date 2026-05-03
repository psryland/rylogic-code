//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/settings.h"
#include "pr/view3d-12/resource/resource_store.h"
#include "pr/view3d-12/utility/features.h"
#include "pr/view3d-12/utility/eventargs.h"
#include "pr/view3d-12/utility/cmd_list_collection.h"
#include "pr/view3d-12/utility/gpu_sync.h"
#include "pr/view3d-12/utility/keep_alive.h"

namespace pr::rdr12
{
	// Registered windows message for BeginInvoke/Tasks
	inline static constexpr UINT WM_BeginInvoke = WM_USER + 0x1976;
	inline static constexpr UINT WM_PollTasks = WM_BeginInvoke + 1;

	// Render main object
	struct Renderer
	{
	private:

		struct PollCallback
		{
			StaticCB<void> m_cb; // The callback
			seconds_t m_period; // The period to call the callback
			time_point_t m_last; // The time when the callback last completed
			void operator()()
			{
				if (!m_cb) return;
				auto now = std::chrono::system_clock::now();
				if (now - m_last >= m_period)
				{
					m_cb();
					m_last += std::chrono::duration_cast<decltype(m_last)::duration>(m_period);
					if (now - m_last > m_period)
						m_last = now;
				}
			}
		};

		using TaskQueue = std::vector<std::future<void>>;
		using PollCBList = std::vector<PollCallback>;
		using AllocationsTracker = AllocationsTracker<void>;
		using TimerHandle = std::shared_ptr<void>;

		// Renderer state
		struct RdrState
		{
			// This is needed so that the Dx12 device is created before the managers are constructed.
			RdrSettings                 m_settings;
			FeatureSupport              m_features;
			D3DPtr<ID3D12Device4>       m_d3d_device;
			D3DPtr<ID3D12CommandQueue>  m_gfx_queue;
			D3DPtr<ID3D12CommandQueue>  m_com_queue;
			D3DPtr<ID3D12CommandQueue>  m_cpy_queue;
			D3DPtr<ID3D11On12Device>    m_dx11_device;
			D3DPtr<ID3D11DeviceContext> m_dx11_dc;
			D3DPtr<ID2D1Factory2>       m_d2dfactory;
			D3DPtr<ID2D1Device>         m_d2d_device;
			DWORD                       m_main_thread_id;

			RdrState(RdrSettings const& settings);
			~RdrState();
		};

		// Members
		RdrState             m_state;
		std::recursive_mutex m_d3d_mutex;
		std::mutex           m_mutex_task_queue;
		TaskQueue            m_task_queue;
		bool                 m_last_task;
		PollCBList           m_poll_callbacks;
		HWND                 m_dummy_hwnd;
		TimerHandle          m_poll_timer;
		std::atomic_int      m_id32_src;
		AllocationsTracker   m_mem_tracker;
		GpuSync              m_gsync;
		KeepAlive            m_keep_alive;

		// Storage of resources
		ResourceStore m_res_store;

	public:

		explicit Renderer(RdrSettings const& settings);
		Renderer(Renderer const&) = delete;
		Renderer& operator=(Renderer const&) = delete;
		~Renderer();

		// Access the renderer manager classes
		ID3D12Device4 const* d3d() const;
		ID3D12Device4* d3d();
		Renderer& rdr();
		ResourceStore& store();
		AllocationsTracker& mem_tracker();

		// Access the adapter that the device was created on
		IDXGIAdapter const* Adapter() const
		{
			return m_state.m_settings.m_adapter.ptr.get();
		}
		IDXGIAdapter* Adapter()
		{
			return const_call(Adapter());
		}

		// Access the device
		ID3D12Device4 const* D3DDevice() const
		{
			// The D3D device is free-threaded in DX12, no need to synchronise access to it.
			return m_state.m_d3d_device.get();
		}
		ID3D12Device4* D3DDevice()
		{
			return const_call(D3DDevice());
		}

		// Return the graphics command queue
		ID3D12CommandQueue const* GfxQueue() const
		{
			// The D3D command queue is free-threaded in DX12, no need to synchronise access to it.
			return m_state.m_gfx_queue.get();
		}
		ID3D12CommandQueue* GfxQueue()
		{
			return const_call(GfxQueue());
		}

		// Return the compute command queue
		ID3D12CommandQueue const* ComQueue() const
		{
			// The D3D command queue is free-threaded in DX12, no need to synchronise access to it.
			return m_state.m_com_queue.get();
		}
		ID3D12CommandQueue* ComQueue()
		{
			return const_call(ComQueue());
		}

		// Return the copy command queue
		ID3D12CommandQueue const* CpyQueue() const
		{
			// The D3D command queue is free-threaded in DX12, no need to synchronise access to it.
			return m_state.m_cpy_queue.get();
		}
		ID3D12CommandQueue* CpyQueue()
		{
			return const_call(CpyQueue());
		}

		// Return the Dx11 device
		ID3D11On12Device const* Dx11Device() const
		{
			return m_state.m_dx11_device.get();
		}
		ID3D11On12Device* Dx11Device()
		{
			return const_call(Dx11Device());
		}

		// Return the Dx11 device context
		ID3D11DeviceContext const* Dx11DeviceContext() const
		{
			return m_state.m_dx11_dc.get();
		}
		ID3D11DeviceContext* Dx11DeviceContext()
		{
			return const_call(Dx11DeviceContext());
		}

		// Return the direct2d factory
		ID2D1Factory const* D2DFactory() const
		{
			return m_state.m_d2dfactory.get();
		}
		ID2D1Factory* D2DFactory()
		{
			return const_call(D2DFactory());
		}

		// Return the d2d device
		ID2D1Device const* D2DDevice() const
		{
			return m_state.m_d2d_device.get();
		}
		ID2D1Device* D2DDevice()
		{
			return const_call(D2DDevice());
		}

		// Read access to the initialisation settings
		RdrSettings const& Settings() const;

		// Device supported features
		FeatureSupport const& Features() const;

		// Return the associated HWND. Note: this is not associated with any particular window. 'Window' objects have an hwnd.
		HWND DummyHwnd() const;

		// Return the current desktop DPI (Fall back if window DPI not available)
		v2 SystemDpi() const;

		// Return info about the available video memory
		DXGI_QUERY_VIDEO_MEMORY_INFO GPUMemoryInfo() const;

		// Generate a unique Id on each call
		int NewId32();

		// Use the 'ResolveFilepath' event to resolve a filepath
		std::filesystem::path ResolvePath(std::string_view path) const;

		// Raised when a window resizes it's back buffer.
		// This is provided on the renderer so that other managers can receive
		// notification without having to sign up to every window that gets created.
		EventHandler<Window&, BackBufferSizeChangedEventArgs> BackBufferSizeChanged;

		// An event that is called to resolve file paths.
		EventHandler<Renderer const&, ResolvePathArgs&, true> ResolveFilepath;

		// Execute a list of graphics command lists. Allows syntax: ExecuteCommandLists({list, list_array, ...})
		void ExecuteGfxCommandLists(GfxCmdListCollection cmd_lists)
		{
			GfxQueue()->ExecuteCommandLists(cmd_lists.count(), cmd_lists.data());
		}
		void ExecuteComCommandLists(ComCmdListCollection cmd_lists)
		{
			ComQueue()->ExecuteCommandLists(cmd_lists.count(), cmd_lists.data());
		}

		// Defer release of a D3D12 resource until the GPU has finished using it.
		// Call this before destroying a C++ wrapper that owns a GPU resource.
		void DeferRelease(D3DPtr<ID3D12Resource> const& res)
		{
			if (!res) return;
			m_keep_alive.Add(D3DPtr<ID3D12Resource>(res));
		}
		void DeferRelease(D3DPtr<ID3D12RootSignature> const& res)
		{
			if (!res) return;
			m_keep_alive.Add(D3DPtr<ID3D12RootSignature>(res));
		}

		// Signal the deferred-deletion fence on the gfx queue.
		// Call this after submitting GPU work that may reference deferred resources.
		void AddDeferredSyncPoint()
		{
			m_gsync.AddSyncPoint(GfxQueue());
		}
		void FlushDeferredReleases()
		{
			AddDeferredSyncPoint();
			m_gsync.Wait();
			m_gsync.Poll();
		}

		// Run the given function on the Main/GUI thread
		// 'policy = std::launch::deferred' means the function is executed by the main thread during 'RunTasks'
		// 'policy = std::launch::async' means the function is run at any time in a worker thread. The result is collected in 'RunTasks'
		// 'policy' can be a bitwise OR of both deferred and async
		// WARNING: be careful with shutdown. Although functions are called on the main thread, they can still be called after
		// referenced data has been destructed.
		template <typename Func, typename... Args>
		void RunOnMainThread(std::launch policy, Func&& func, Args&&... args)
		{
			{
				std::lock_guard<std::mutex> lock(m_mutex_task_queue);
				if (m_last_task) return; // Don't add further tasks after 'LastTask' has been called.
				m_task_queue.emplace_back(std::async(policy, std::move(func), std::forward<Args>(args)...));
			}

			// Post a message to notify of the new task
			for (; !::PostMessageW(m_dummy_hwnd, WM_BeginInvoke, WPARAM(this), LPARAM());)
			{
				auto err = GetLastError();
				if (err == ERROR_NOT_ENOUGH_QUOTA)
				{
					// The message queue is full, just wait a bit. This is probably a deadlock though
					std::this_thread::yield();
					continue;
				}

				auto msg = pr::HrMsg(err);
				throw std::runtime_error(msg.c_str());
			}
		}
		template <typename Func, typename... Args>
		void RunOnMainThread(Func&& func, Args&&... args)
		{
			static_assert(noexcept(func(std::forward<Args>(args)...)), "func should be noexcept");
			RunOnMainThread(std::launch::deferred, std::move(func), std::forward<Args>(args)...);
		}

		// Execute any pending tasks in the task queue
		void RunTasks();

		// Call this during shutdown to flush the task queue and prevent any further tasks from being added.
		void LastTask();

		// Add/Remove a callback function that will be polled as fast as the windows message queue will allow
		void AddPollCB(StaticCB<void> cb, seconds_t period_ms);
		void RemovePollCB(StaticCB<void> cb);

		// Call all registered poll event callbacks
		void Poll();

		// For when I'm searching for the d3d device on this object
		enum { D3DDevice_IsAccessedViaTheRendererLock };
		bool AssertMainThread() const;
	};
}

