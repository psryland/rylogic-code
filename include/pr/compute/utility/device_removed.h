//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr::compute
{
	// Configure process-wide D3D12 device-removal diagnostics before creating a device.
	inline bool EnableDeviceRemovedDiagnostics()
	{
		D3DPtr<ID3D12DeviceRemovedExtendedDataSettings1> settings;
		if (FAILED(D3D12GetDebugInterface(__uuidof(ID3D12DeviceRemovedExtendedDataSettings1), (void**)settings.address_of())))
			return false;

		settings->SetAutoBreadcrumbsEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
		settings->SetPageFaultEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
		settings->SetBreadcrumbContextEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
		return true;
	}

	// Return a bounded UTF-8 rendering of a DRED debug name.
	inline std::string DredName(char const* name_a, wchar_t const* name_w)
	{
		static constexpr auto MaxNameLength = std::size_t{512};
		if (name_a != nullptr)
			return std::string(name_a, strnlen_s(name_a, MaxNameLength));
		if (name_w != nullptr)
			return Narrow(std::wstring_view(name_w, wcsnlen_s(name_w, MaxNameLength)));
		return "<unnamed>";
	}

	// Return a readable name for the DRED operations most useful around a fault.
	inline std::string_view DredOperationName(D3D12_AUTO_BREADCRUMB_OP operation)
	{
		switch (operation)
		{
			case D3D12_AUTO_BREADCRUMB_OP_SETMARKER: { return "SetMarker"; }
			case D3D12_AUTO_BREADCRUMB_OP_BEGINEVENT: { return "BeginEvent"; }
			case D3D12_AUTO_BREADCRUMB_OP_ENDEVENT: { return "EndEvent"; }
			case D3D12_AUTO_BREADCRUMB_OP_DRAWINSTANCED: { return "DrawInstanced"; }
			case D3D12_AUTO_BREADCRUMB_OP_DRAWINDEXEDINSTANCED: { return "DrawIndexedInstanced"; }
			case D3D12_AUTO_BREADCRUMB_OP_EXECUTEINDIRECT: { return "ExecuteIndirect"; }
			case D3D12_AUTO_BREADCRUMB_OP_DISPATCH: { return "Dispatch"; }
			case D3D12_AUTO_BREADCRUMB_OP_COPYBUFFERREGION: { return "CopyBufferRegion"; }
			case D3D12_AUTO_BREADCRUMB_OP_COPYTEXTUREREGION: { return "CopyTextureRegion"; }
			case D3D12_AUTO_BREADCRUMB_OP_COPYRESOURCE: { return "CopyResource"; }
			case D3D12_AUTO_BREADCRUMB_OP_RESOLVESUBRESOURCE: { return "ResolveSubresource"; }
			case D3D12_AUTO_BREADCRUMB_OP_RESOURCEBARRIER: { return "ResourceBarrier"; }
			case D3D12_AUTO_BREADCRUMB_OP_EXECUTEBUNDLE: { return "ExecuteBundle"; }
			case D3D12_AUTO_BREADCRUMB_OP_PRESENT: { return "Present"; }
			case D3D12_AUTO_BREADCRUMB_OP_DISPATCHRAYS: { return "DispatchRays"; }
			case D3D12_AUTO_BREADCRUMB_OP_BUILDRAYTRACINGACCELERATIONSTRUCTURE: { return "BuildRaytracingAccelerationStructure"; }
			default: { return "Other"; }
		}
	}

	// Append a bounded list of allocations associated with a page fault.
	inline void AppendDredAllocations(std::ostringstream& report, std::string_view title, D3D12_DRED_ALLOCATION_NODE1 const* allocation)
	{
		static constexpr auto MaxAllocations = 24;
		report << title << ":\n";
		auto count = 0;
		for (; allocation != nullptr && count != MaxAllocations; allocation = allocation->pNext, ++count)
		{
			report
				<< "  [" << count << "] "
				<< DredName(allocation->ObjectNameA, allocation->ObjectNameW)
				<< " type=" << static_cast<unsigned>(allocation->AllocationType)
				<< '\n';
		}
		if (allocation != nullptr)
			report << "  ... additional allocations omitted\n";
	}

	// Capture bounded DRED breadcrumbs and page-fault evidence after a D3D12 device is removed.
	inline std::string DeviceRemovedReport(ID3D12Device* device, HRESULT reason)
	{
		static constexpr auto MaxBreadcrumbNodes = 64;
		static constexpr auto MaxOperationsPerNode = 16U;
		static constexpr auto MaxContextsPerNode = 16U;
		static constexpr auto MaxReportLength = std::size_t{64 * 1024};

		auto report = std::ostringstream{};
		report << "D3D12 device removed: 0x" << std::format("{:08X}", static_cast<std::uint32_t>(reason)) << '\n';

		// DRED data is exposed only after removal and may be unavailable when the OS controlled settings disabled collection.
		D3DPtr<ID3D12DeviceRemovedExtendedData1> dred;
		auto query_result = device != nullptr
			? device->QueryInterface(__uuidof(ID3D12DeviceRemovedExtendedData1), (void**)dred.address_of())
			: E_POINTER;
		if (FAILED(query_result))
		{
			report << "DRED unavailable: 0x" << std::format("{:08X}", static_cast<std::uint32_t>(query_result)) << '\n';
			return report.str();
		}

		// Breadcrumbs identify the command list and operations nearest each queue's last completed command.
		auto breadcrumbs = D3D12_DRED_AUTO_BREADCRUMBS_OUTPUT1{};
		auto breadcrumbs_result = dred->GetAutoBreadcrumbsOutput1(&breadcrumbs);
		if (SUCCEEDED(breadcrumbs_result))
		{
			auto node_index = 0;
			for (auto node = breadcrumbs.pHeadAutoBreadcrumbNode; node != nullptr && node_index != MaxBreadcrumbNodes; node = node->pNext, ++node_index)
			{
				auto completed = node->pLastBreadcrumbValue != nullptr ? std::min(*node->pLastBreadcrumbValue, node->BreadcrumbCount) : 0U;
				report
					<< "Breadcrumb[" << node_index << "] queue=" << DredName(node->pCommandQueueDebugNameA, node->pCommandQueueDebugNameW)
					<< " list=" << DredName(node->pCommandListDebugNameA, node->pCommandListDebugNameW)
					<< " completed=" << completed << '/' << node->BreadcrumbCount
					<< '\n';

				auto begin = completed > MaxOperationsPerNode / 2 ? completed - MaxOperationsPerNode / 2 : 0U;
				auto end = std::min(node->BreadcrumbCount, begin + MaxOperationsPerNode);
				for (auto index = begin; node->pCommandHistory != nullptr && index != end; ++index)
				{
					auto operation = node->pCommandHistory[index];
					report
						<< "  " << (index < completed ? "done" : index == completed ? "next" : "pending")
						<< '[' << index << "] " << DredOperationName(operation)
						<< " (" << static_cast<unsigned>(operation) << ")\n";
				}

				auto context_count = std::min(node->BreadcrumbContextsCount, MaxContextsPerNode);
				for (auto index = 0U; node->pBreadcrumbContexts != nullptr && index != context_count; ++index)
				{
					auto const& context = node->pBreadcrumbContexts[index];
					report
						<< "  context[" << context.BreadcrumbIndex << "] "
						<< (context.pContextString != nullptr ? DredName(nullptr, context.pContextString) : "<unnamed>")
						<< '\n';
				}
			}
			if (node_index == MaxBreadcrumbNodes)
				report << "Additional breadcrumb nodes omitted\n";
		}
		else
		{
			report << "DRED breadcrumbs unavailable: 0x" << std::format("{:08X}", static_cast<std::uint32_t>(breadcrumbs_result)) << '\n';
		}

		// Page-fault allocation names identify the resource owning the faulting virtual address.
		auto page_fault = D3D12_DRED_PAGE_FAULT_OUTPUT1{};
		auto page_fault_result = dred->GetPageFaultAllocationOutput1(&page_fault);
		if (SUCCEEDED(page_fault_result))
		{
			report << "Page fault virtual address: 0x" << std::format("{:016X}", page_fault.PageFaultVA) << '\n';
			AppendDredAllocations(report, "Existing allocations", page_fault.pHeadExistingAllocationNode);
			AppendDredAllocations(report, "Recently freed allocations", page_fault.pHeadRecentFreedAllocationNode);
		}
		else
		{
			report << "DRED page-fault data unavailable: 0x" << std::format("{:08X}", static_cast<std::uint32_t>(page_fault_result)) << '\n';
		}

		auto result = report.str();
		if (result.size() > MaxReportLength)
			result.resize(MaxReportLength);
		return result;
	}

	// A typed D3D12 removal failure that preserves the native HRESULT and bounded DRED evidence.
	struct DeviceRemovedException :std::runtime_error
	{
		HRESULT m_reason;

		DeviceRemovedException(ID3D12Device* device, HRESULT reason)
			: std::runtime_error(DeviceRemovedReport(device, reason))
			, m_reason(reason)
		{
		}
	};
}
