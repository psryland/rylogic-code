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

	// Device-local diagnostics; borrowed devices keep their owner's settings. Process validation is configured before device creation.
	// D3D12 shares one device per adapter, so message settings affect all users of that device, not only this wrapper.
	enum class EGpuOptions
	{
		None = 0,

		// Break on corruption, errors and warnings. Requires ConfigureProcessDiagnostics at process startup.
		BreakOnErrors = 1 << 0,

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

		// Create a device and queue with device-local 'options'. This never changes process diagnostics.
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
	// Test runners configure TestProcessDiagnostics at startup; Debug fixtures stop on device validation errors.
	inline constexpr EGpuOptions TestGpuOptions = IsDebug ? EGpuOptions::BreakOnErrors : EGpuOptions::None;
}
#endif

