//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_diagnostic.h"
#include "pr/view3d-12/main/frame.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_scene.h"
#include "pr/view3d-12/resource/gpu_descriptor_heap.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/root_signature.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	namespace
	{
		constexpr auto OutputFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
		constexpr wchar_t RayGenShader[] = L"RayGen";
		constexpr wchar_t MissShader[] = L"Miss";
		constexpr wchar_t ClosestHitShader[] = L"ClosestHit";
		constexpr wchar_t HitGroup[] = L"HitGroup";

		enum class ETraceRootParam
		{
			CBufFrame,
			Scene,
			InputColour,
			Depth,
			Output,
		};
		enum class EPresentRootParam
		{
			Output,
		};

		// Copy a DXR shader identifier into a shader-table record.
		void CopyShaderIdentifier(ID3D12StateObjectProperties* props, wchar_t const* export_name, std::span<uint8_t> record)
		{
			auto const* identifier = props->GetShaderIdentifier(export_name);
			if (identifier == nullptr)
				throw std::runtime_error("DXR shader export was not found");

			memcpy(record.data(), identifier, D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES);
		}

		struct DxilLibraryState
		{
			std::array<D3D12_EXPORT_DESC, 3> m_exports;
			D3D12_DXIL_LIBRARY_DESC m_library;

			DxilLibraryState()
				: m_exports{
					D3D12_EXPORT_DESC{ .Name = RayGenShader, .ExportToRename = nullptr, .Flags = D3D12_EXPORT_FLAG_NONE },
					D3D12_EXPORT_DESC{ .Name = MissShader, .ExportToRename = nullptr, .Flags = D3D12_EXPORT_FLAG_NONE },
					D3D12_EXPORT_DESC{ .Name = ClosestHitShader, .ExportToRename = nullptr, .Flags = D3D12_EXPORT_FLAG_NONE },
				}
				, m_library{
					.DXILLibrary = shader_code::ray_trace_lib,
					.NumExports = s_cast<UINT>(m_exports.size()),
					.pExports = m_exports.data(),
				}
			{}
		};
	}

	struct RayTracingDiagnostic::Data
	{
		D3DPtr<ID3D12RootSignature> m_trace_signature;
		D3DPtr<ID3D12RootSignature> m_present_signature;
		D3DPtr<ID3D12StateObject> m_trace_state;
		D3DPtr<ID3D12StateObjectProperties> m_trace_props;
		D3DPtr<ID3D12PipelineState> m_present_pso;
		D3DPtr<ID3D12Resource> m_shader_table;
		D3DPtr<ID3D12Resource> m_output;
		D3D12_DISPATCH_RAYS_DESC m_dispatch_desc;
		DXGI_FORMAT m_present_format;
		iv2 m_output_size;

		Data()
			: m_trace_signature()
			, m_present_signature()
			, m_trace_state()
			, m_trace_props()
			, m_present_pso()
			, m_shader_table()
			, m_output()
			, m_dispatch_desc()
			, m_present_format(DXGI_FORMAT_UNKNOWN)
			, m_output_size(iv2::Zero())
		{}
	};

	// Create empty diagnostic ray tracing state.
	RayTracingDiagnostic::RayTracingDiagnostic()
		: m_data()
	{}

	// Move diagnostic ray tracing state without copying GPU resource ownership.
	RayTracingDiagnostic::RayTracingDiagnostic(RayTracingDiagnostic&& rhs) noexcept = default;

	// Move diagnostic ray tracing state without copying GPU resource ownership.
	RayTracingDiagnostic& RayTracingDiagnostic::operator =(RayTracingDiagnostic&& rhs) noexcept = default;

	// Destroy the diagnostic ray tracing state.
	RayTracingDiagnostic::~RayTracingDiagnostic() = default;

	// Release GPU resources after deferring GPU lifetime management through the renderer.
	void RayTracingDiagnostic::DeferRelease(Renderer& rdr)
	{
		if (m_data == nullptr)
			return;

		rdr.DeferRelease(m_data->m_output);
		rdr.DeferRelease(m_data->m_shader_table);
		rdr.DeferRelease(m_data->m_trace_signature);
		rdr.DeferRelease(m_data->m_present_signature);
		m_data = nullptr;
	}

	// Create the global DXR root signature.
	static void EnsureTraceSignature(Renderer& rdr, RayTracingDiagnostic::Data& data)
	{
		if (data.m_trace_signature != nullptr)
			return;

		data.m_trace_signature = RootSig(ERootSigFlags::None)
			.CBuf(hlsl::ECBufReg::b0)
			.SRV(hlsl::ESRVReg::t0)
			.SRV(hlsl::ESRVReg::t1, 1)
			.SRV(hlsl::ESRVReg::t2, 1)
			.UAV(hlsl::EUAVReg::u0, 1)
			.Create(rdr.D3DDevice(), "RT-DiagnosticTraceSig");
	}

	// Create the fullscreen-present root signature.
	static void EnsurePresentSignature(Renderer& rdr, RayTracingDiagnostic::Data& data)
	{
		if (data.m_present_signature != nullptr)
			return;

		data.m_present_signature = RootSig(ERootSigFlags::GraphicsOnly)
			.SRV(hlsl::ESRVReg::t0, 1, D3D12_SHADER_VISIBILITY_PIXEL)
			.Create(rdr.D3DDevice(), "RT-DiagnosticPresentSig");
	}

	// Create the minimal DXR state object.
	static void EnsureTraceState(Renderer& rdr, RayTracingDiagnostic::Data& data)
	{
		if (data.m_trace_state != nullptr)
			return;

		D3DPtr<ID3D12Device5> device;
		Check(rdr.D3DDevice()->QueryInterface(__uuidof(ID3D12Device5), (void**)device.address_of()));

		auto library_state = DxilLibraryState{};
		auto hit_group = D3D12_HIT_GROUP_DESC{
			.HitGroupExport = HitGroup,
			.Type = D3D12_HIT_GROUP_TYPE_TRIANGLES,
			.AnyHitShaderImport = nullptr,
			.ClosestHitShaderImport = ClosestHitShader,
			.IntersectionShaderImport = nullptr,
		};
		auto shader_config = D3D12_RAYTRACING_SHADER_CONFIG{
			.MaxPayloadSizeInBytes = sizeof(v4) + sizeof(float) + 3 * sizeof(uint32_t),
			.MaxAttributeSizeInBytes = 2 * sizeof(float),
		};
		auto global_signature = D3D12_GLOBAL_ROOT_SIGNATURE{
			.pGlobalRootSignature = data.m_trace_signature.get(),
		};
		auto pipeline_config = D3D12_RAYTRACING_PIPELINE_CONFIG{
			.MaxTraceRecursionDepth = 1,
		};
		auto subobjects = std::array{
			D3D12_STATE_SUBOBJECT{ .Type = D3D12_STATE_SUBOBJECT_TYPE_DXIL_LIBRARY, .pDesc = &library_state.m_library },
			D3D12_STATE_SUBOBJECT{ .Type = D3D12_STATE_SUBOBJECT_TYPE_HIT_GROUP, .pDesc = &hit_group },
			D3D12_STATE_SUBOBJECT{ .Type = D3D12_STATE_SUBOBJECT_TYPE_RAYTRACING_SHADER_CONFIG, .pDesc = &shader_config },
			D3D12_STATE_SUBOBJECT{ .Type = D3D12_STATE_SUBOBJECT_TYPE_GLOBAL_ROOT_SIGNATURE, .pDesc = &global_signature },
			D3D12_STATE_SUBOBJECT{ .Type = D3D12_STATE_SUBOBJECT_TYPE_RAYTRACING_PIPELINE_CONFIG, .pDesc = &pipeline_config },
		};
		auto state_desc = D3D12_STATE_OBJECT_DESC{
			.Type = D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE,
			.NumSubobjects = s_cast<UINT>(subobjects.size()),
			.pSubobjects = subobjects.data(),
		};

		Check(device->CreateStateObject(&state_desc, __uuidof(ID3D12StateObject), (void**)data.m_trace_state.address_of()));
		DebugName(data.m_trace_state, "RT-DiagnosticState");
		Check(data.m_trace_state->QueryInterface(__uuidof(ID3D12StateObjectProperties), (void**)data.m_trace_props.address_of()));
	}

	// Create the fullscreen-present PSO for the current back-buffer format.
	static void EnsurePresentPso(Renderer& rdr, RayTracingDiagnostic::Data& data, DXGI_FORMAT present_format)
	{
		if (data.m_present_pso != nullptr && data.m_present_format == present_format)
			return;

		data.m_present_format = present_format;
		auto desc = D3D12_GRAPHICS_PIPELINE_STATE_DESC{
			.pRootSignature = data.m_present_signature.get(),
			.VS = shader_code::ray_trace_present_vs,
			.PS = shader_code::ray_trace_present_ps,
			.DS = shader_code::none,
			.HS = shader_code::none,
			.GS = shader_code::none,
			.StreamOutput = StreamOutputDesc{},
			.BlendState = BlendStateDesc{},
			.SampleMask = UINT_MAX,
			.RasterizerState = RasterStateDesc{},
			.DepthStencilState = DepthStateDesc{}.Enabled(false),
			.InputLayout = {},
			.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
			.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE,
			.NumRenderTargets = 1U,
			.RTVFormats = { present_format },
			.DSVFormat = DXGI_FORMAT_UNKNOWN,
			.SampleDesc = MultiSamp(1, 0),
			.NodeMask = 0U,
			.CachedPSO = {},
			.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
		};

		data.m_present_pso = nullptr;
		Check(rdr.D3DDevice()->CreateGraphicsPipelineState(&desc, __uuidof(ID3D12PipelineState), (void**)data.m_present_pso.address_of()));
		DebugName(data.m_present_pso, "RT-DiagnosticPresentPSO");
	}

	// Create the shader table for the diagnostic state object.
	static void EnsureShaderTable(ResourceFactory& factory, RayTracingDiagnostic::Data& data)
	{
		if (data.m_shader_table != nullptr)
			return;

		constexpr auto record_stride = PadTo<size_t>(D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES, D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT);
		auto records = std::array<uint8_t, 3 * record_stride>{};

		CopyShaderIdentifier(data.m_trace_props.get(), RayGenShader, std::span<uint8_t>{ records.data() + 0 * record_stride, record_stride });
		CopyShaderIdentifier(data.m_trace_props.get(), MissShader, std::span<uint8_t>{ records.data() + 1 * record_stride, record_stride });
		CopyShaderIdentifier(data.m_trace_props.get(), HitGroup, std::span<uint8_t>{ records.data() + 2 * record_stride, record_stride });

		auto desc = ResDesc::Buf<uint8_t>(isize(records), std::span<uint8_t const>{ records.data(), records.size() })
			.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		data.m_shader_table = factory.CreateResource(desc, "RT-DiagnosticShaderTable");

		auto base_address = data.m_shader_table->GetGPUVirtualAddress();
		data.m_dispatch_desc.RayGenerationShaderRecord = {
			.StartAddress = base_address + 0 * record_stride,
			.SizeInBytes = record_stride,
		};
		data.m_dispatch_desc.MissShaderTable = {
			.StartAddress = base_address + 1 * record_stride,
			.SizeInBytes = record_stride,
			.StrideInBytes = record_stride,
		};
		data.m_dispatch_desc.HitGroupTable = {
			.StartAddress = base_address + 2 * record_stride,
			.SizeInBytes = record_stride,
			.StrideInBytes = record_stride,
		};
	}

	// Create or resize the intermediate UAV output texture.
	static void EnsureOutput(ResourceFactory& factory, RayTracingDiagnostic::Data& data, iv2 output_size)
	{
		if (data.m_output != nullptr && All(data.m_output_size == output_size))
			return;

		factory.rdr().DeferRelease(data.m_output);
		data.m_output_size = output_size;

		auto desc = ResDesc::Tex2D(Image{ output_size.x, output_size.y, nullptr, OutputFormat }, 1U, EUsage::UnorderedAccess)
			.def_state(D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		data.m_output = factory.CreateResource(desc, "RT-DiagnosticOutput");
	}

	// Create or resize GPU resources needed for the diagnostic pass.
	bool RayTracingDiagnostic::Prepare(ResourceFactory& factory, iv2 output_size, DXGI_FORMAT present_format)
	{
		if (output_size.x <= 0 || output_size.y <= 0 || present_format == DXGI_FORMAT_UNKNOWN)
			return false;

		if (m_data == nullptr)
			m_data = std::make_unique<Data>();

		auto& data = *m_data;
		EnsureTraceSignature(factory.rdr(), data);
		EnsurePresentSignature(factory.rdr(), data);
		EnsureTraceState(factory.rdr(), data);
		EnsurePresentPso(factory.rdr(), data, present_format);
		EnsureShaderTable(factory, data);
		EnsureOutput(factory, data, output_size);
		return true;
	}

	// Record the ray dispatch and presentation commands for the selected screen-space pass.
	void RayTracingDiagnostic::Record(GfxCmdList& cmd_list, Frame& frame, Scene const& scene, RayTracingScene const& ray_tracing_scene, ERayTracingScreenPass pass, bool restore_present_state)
	{
		if (m_data == nullptr || m_data->m_output == nullptr || !ray_tracing_scene.Built())
			return;

		auto& data = *m_data;
		D3DPtr<ID3D12GraphicsCommandList4> dxr_cmd_list;
		Check(cmd_list.get()->QueryInterface(__uuidof(ID3D12GraphicsCommandList4), (void**)dxr_cmd_list.address_of()));

		auto const output_size = data.m_output_size;
		auto const output = data.m_output.get();
		auto const input = const_cast<ID3D12Resource*>(frame.bb_post().m_render_target.get());
		auto const depth = const_cast<ID3D12Resource*>(frame.bb_main().m_depth_stencil.get());
		auto const tlas_address = ray_tracing_scene.AccelerationStructureAddress();
		if (pass == ERayTracingScreenPass::HardShadows && depth == nullptr)
			return;

		// Dispatch rays into an intermediate UAV. The fullscreen present pass that follows keeps the UAV format independent of the swap-chain format.
		{
			BarrierBatch barriers(cmd_list);
			barriers.Transition(input, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			if (depth != nullptr)
				barriers.Transition(depth, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Transition(output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			barriers.Commit();

			auto input_desc = input->GetDesc();
			auto input_srv_desc = D3D12_SHADER_RESOURCE_VIEW_DESC{
				.Format = input_desc.Format,
				.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
				.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				.Texture2D = {
					.MostDetailedMip = 0U,
					.MipLevels = 1U,
					.PlaneSlice = 0U,
					.ResourceMinLODClamp = 0.0f,
				},
			};
			auto input_srv = frame.bb_post().wnd().m_heap_view.Add(input, input_srv_desc);
			auto depth_srv = frame.bb_main().m_depth_srv
				? frame.bb_post().wnd().m_heap_view.Add(frame.bb_main().m_depth_srv)
				: frame.bb_post().wnd().m_heap_view.Add(nullptr, D3D12_SHADER_RESOURCE_VIEW_DESC{
					.Format = DXGI_FORMAT_R32_FLOAT,
					.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DMS,
					.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				});

			auto uav_desc = D3D12_UNORDERED_ACCESS_VIEW_DESC{
				.Format = OutputFormat,
				.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D,
				.Texture2D = {
					.MipSlice = 0U,
					.PlaneSlice = 0U,
				},
			};
			auto output_uav = frame.bb_post().wnd().m_heap_view.Add(output, uav_desc);

			auto cb = shaders::rt::CBufFrame{};
			SetViewConstants(cb.cam, scene.m_cam);
			cb.s2w = Invert(cb.cam.w2s);
			SetLightingConstants(cb.global_light, scene.m_global_light, scene.m_cam);
			cb.camera = v4(
				s_cast<float>(scene.m_cam.Aspect()),
				s_cast<float>(scene.m_cam.FovY()),
				s_cast<float>(scene.m_cam.FocusDist()),
				scene.m_cam.Orthographic() ? 1.0f : 0.0f);
			cb.clip = v4(
				s_cast<float>(scene.m_cam.Near(false)),
				s_cast<float>(scene.m_cam.Far(false)),
				0.0f,
				0.0f);
			cb.shadow = v4(0.55f, 0.01f, 0.0f, 0.0f);
			cb.options = iv4(
				pass == ERayTracingScreenPass::Diagnostic ? shaders::rt::RayTracingMode_Diagnostic :
				pass == ERayTracingScreenPass::HardShadows ? shaders::rt::RayTracingMode_HardShadows :
				throw std::runtime_error("Unknown ray tracing screen pass"),
				0, 0, 0);

			cmd_list.SetComputeRootSignature(data.m_trace_signature.get());
			cmd_list.SetComputeRootConstantBufferView(ETraceRootParam::CBufFrame, frame.m_upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false));
			cmd_list.SetComputeRootShaderResourceView(ETraceRootParam::Scene, tlas_address);
			cmd_list.SetComputeRootDescriptorTable(ETraceRootParam::InputColour, input_srv);
			cmd_list.SetComputeRootDescriptorTable(ETraceRootParam::Depth, depth_srv);
			cmd_list.SetComputeRootDescriptorTable(ETraceRootParam::Output, output_uav);
			dxr_cmd_list->SetPipelineState1(data.m_trace_state.get());

			auto dispatch_desc = data.m_dispatch_desc;
			dispatch_desc.Width = s_cast<UINT>(output_size.x);
			dispatch_desc.Height = s_cast<UINT>(output_size.y);
			dispatch_desc.Depth = 1U;
			dxr_cmd_list->DispatchRays(&dispatch_desc);
		}

		// Present the diagnostic texture over the post-resolve back buffer so the path works with MSAA and BGRA swap-chain formats.
		{
			BarrierBatch barriers(cmd_list);
			barriers.UAV(output);
			barriers.Transition(output, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
			barriers.Transition(frame.bb_post().m_render_target.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
			if (depth != nullptr)
				barriers.Transition(depth, D3D12_RESOURCE_STATE_DEPTH_WRITE);
			barriers.Commit();

			auto srv_desc = D3D12_SHADER_RESOURCE_VIEW_DESC{
				.Format = OutputFormat,
				.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
				.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				.Texture2D = {
					.MostDetailedMip = 0U,
					.MipLevels = 1U,
					.PlaneSlice = 0U,
					.ResourceMinLODClamp = 0.0f,
				},
			};
			auto output_srv = frame.bb_post().wnd().m_heap_view.Add(output, srv_desc);
			auto viewport = Viewport(output_size);

			cmd_list.SetGraphicsRootSignature(data.m_present_signature.get());
			cmd_list.SetGraphicsRootDescriptorTable(EPresentRootParam::Output, output_srv);
			cmd_list.SetPipelineState(data.m_present_pso.get());
			cmd_list.RSSetViewports({ &viewport, 1 });
			cmd_list.RSSetScissorRects(viewport.m_clip);
			cmd_list.OMSetRenderTargets({ &frame.bb_post().m_rtv, 1 }, false, nullptr);
			cmd_list.IASetPrimitiveTopology(ETopo::TriList);
			cmd_list.DrawInstanced(3, 1, 0, 0);

			if (restore_present_state)
			{
				BarrierBatch bb(cmd_list);
				bb.Transition(frame.bb_post().m_render_target.get(), D3D12_RESOURCE_STATE_PRESENT);
				bb.Commit();
			}
		}
	}
}
