//**********************************
// Compute
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/utility/utility.h"

namespace pr::compute
{
	template <D3D12_COMMAND_LIST_TYPE QueueType> struct GpuJob;

	// Diagnostics options for a device created by 'Gpu'. They have no effect on a borrowed device.
	enum class EGpuOptions
	{
		None = 0,

		// Enable the D3D12 debug layer. This is process-wide and is not reverted when the 'Gpu' is destroyed.
		DeviceDebug = 1 << 0,

		// Also enable GPU-based validation. This is very slow.
		DeviceGPUDebug = 1 << 1 | DeviceDebug,

		// Break into the debugger on D3D12 corruption, error, and warning messages.
		BreakOnErrors = 1 << 2 | DeviceDebug,

		_flags_enum = 0,
	};

	// A DirectX 12 device and command queue for running compute shaders. Work is recorded and submitted by 'GpuJob's, which own their fences.
	template <D3D12_COMMAND_LIST_TYPE ListType>
	class Gpu
	{
		// Notes:
		//  - Use this type when you want to run compute shaders on the GPU but don't
		//    want to create a full 'Renderer' instance.

		// D3D Device and command queue
		D3DPtr<ID3D12Device4> m_device;
		D3DPtr<ID3D12CommandQueue> m_cmd_queue;

	public:

		// Create a device on the default adapter, with the diagnostics in 'options', and a command queue with 'priority'.
		explicit Gpu(EGpuOptions options = EGpuOptions::None, D3D12_COMMAND_QUEUE_PRIORITY priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL);

		// Borrow 'existing_device', which must not be null. Borrow 'existing_queue' when provided, otherwise create a queue with 'priority'.
		explicit Gpu(ID3D12Device4* existing_device, ID3D12CommandQueue* existing_queue = nullptr, D3D12_COMMAND_QUEUE_PRIORITY priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL);

	private:

		// Create the command queue if one was not borrowed, and check the device supports the compute shaders.
		void InitQueue(D3D12_COMMAND_QUEUE_PRIORITY priority);

	public:

		ID3D12Device4 const* device() const
		{
			return m_device.get();
		}
		ID3D12Device4* device()
		{
			return m_device.get();
		}

		ID3D12CommandQueue const* queue() const
		{
			return m_cmd_queue.get();
		}
		ID3D12CommandQueue* queue()
		{
			return m_cmd_queue.get();
		}

		// Allow use as a device
		ID3D12Device4 const* operator -> () const
		{
			return device();
		}
		ID3D12Device4* operator ->()
		{
			return device();
		}
		operator ID3D12Device4 const* () const
		{
			return device();
		}
		operator ID3D12Device4* ()
		{
			return device();
		}

		// Allocate a DX resource. Initial data is staged in 'job's upload buffer and copied by 'job's command list,
		// so the resource is only initialised once 'job' has been submitted and completed.
		D3DPtr<ID3D12Resource> CreateResource(ResDesc const& desc, GpuJob<ListType>& job, std::string_view name);
	};

	using GfxGpu = Gpu<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ComGpu = Gpu<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
}

#if PR_UNITTESTS
namespace pr::compute
{
	// Diagnostics for devices created by unit tests. Debug builds validate GPU usage and stop at the first D3D12 error.
	inline constexpr EGpuOptions TestGpuOptions = IsDebug ? EGpuOptions::BreakOnErrors : EGpuOptions::None;
}
#endif

