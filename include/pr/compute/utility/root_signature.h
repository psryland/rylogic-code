//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr::compute
{
	struct RootSig
	{
		// Notes:
		//  - This type helps build a root signature
		//  - EParam and ESamp are really just indexes. They are indexes into the array of
		//    root parameters or static samplers which don't need to be the same as registers
		//    declared in a shader.
		pr::vector<D3D12_ROOT_PARAMETER1, 16, false> m_root_params;
		pr::vector<D3D12_STATIC_SAMPLER_DESC, 8, false> m_static_samplers;
		pr::deque<D3D12_DESCRIPTOR_RANGE1, 16> m_des_range;
		ERootSigFlags m_flags;

		explicit RootSig(ERootSigFlags flags)
			:m_root_params()
			,m_des_range()
			,m_static_samplers()
			,m_flags(flags)
		{}

		// Add a u32 constant parameter
		RootSig& U32(hlsl::ECBufReg reg, int num_values, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL)
		{
			param() = D3D12_ROOT_PARAMETER1{
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS,
				.Constants = {.ShaderRegister = s_cast<UINT>(reg), .RegisterSpace = 0U, .Num32BitValues = s_cast<UINT>(num_values)},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}
		template <typename CBufType>
		RootSig& U32(hlsl::ECBufReg reg, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL)
		{
			static_assert(sizeof(CBufType) % sizeof(uint32_t) == 0);
			auto count = s_cast<int>(sizeof(CBufType) / sizeof(uint32_t));
			return U32(reg, count, shader_visibility);
		}

		// Add a constant buffer root parameter
		RootSig& CBuf(hlsl::ECBufReg reg, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL, D3D12_ROOT_DESCRIPTOR_FLAGS flags = D3D12_ROOT_DESCRIPTOR_FLAG_NONE)
		{
			param() = D3D12_ROOT_PARAMETER1{
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV,
				.Descriptor = {.ShaderRegister = s_cast<UINT>(reg), .RegisterSpace = 0U, .Flags = flags},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}

		// Add a constant buffer range parameter
		RootSig& CBuf(hlsl::ECBufReg reg, int count, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAGS flags = D3D12_DESCRIPTOR_RANGE_FLAG_NONE)
		{
			m_des_range.push_back(D3D12_DESCRIPTOR_RANGE1 {
				.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_CBV,
				.NumDescriptors = s_cast<UINT>(count),
				.BaseShaderRegister = s_cast<UINT>(reg),
				.RegisterSpace = 0U,
				.Flags = flags,
				.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND,
			});

			param() = D3D12_ROOT_PARAMETER1{
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
				.DescriptorTable = {.NumDescriptorRanges = 1U, .pDescriptorRanges = &m_des_range.back()},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}

		// Add a SRV/texture descriptor root parameter
		RootSig& SRV(hlsl::ESRVReg reg, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL, D3D12_ROOT_DESCRIPTOR_FLAGS flags = D3D12_ROOT_DESCRIPTOR_FLAG_NONE)
		{
			param() = D3D12_ROOT_PARAMETER1 {
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV,
				.Descriptor = {.ShaderRegister = s_cast<UINT>(reg), .RegisterSpace = 0, .Flags = flags},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}

		// Add a SRV/texture descriptor range parameter
		RootSig& SRV(hlsl::ESRVReg reg, int count, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAGS flags = D3D12_DESCRIPTOR_RANGE_FLAG_NONE)
		{
			m_des_range.push_back(D3D12_DESCRIPTOR_RANGE1 {
				.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV,
				.NumDescriptors = s_cast<UINT>(count),
				.BaseShaderRegister = s_cast<UINT>(reg),
				.RegisterSpace = 0U,
				.Flags = flags,
				.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND,
			});
			param() = D3D12_ROOT_PARAMETER1 {
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
				.DescriptorTable = {.NumDescriptorRanges = 1U, .pDescriptorRanges = &m_des_range.back()},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}

		// Add an Unordered access view root descriptor parameter
		RootSig& UAV(hlsl::EUAVReg reg, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL, D3D12_ROOT_DESCRIPTOR_FLAGS flags = D3D12_ROOT_DESCRIPTOR_FLAG_NONE)
		{
			param() = D3D12_ROOT_PARAMETER1{
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV,
				.Descriptor = {.ShaderRegister = s_cast<UINT>(reg), .RegisterSpace = 0, .Flags = flags},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}

		// Add an Unordered access view descriptor range parameter
		RootSig& UAV(hlsl::EUAVReg reg, int count, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAGS flags = D3D12_DESCRIPTOR_RANGE_FLAG_NONE)
		{
			m_des_range.push_back(D3D12_DESCRIPTOR_RANGE1{
				.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV,
				.NumDescriptors = s_cast<UINT>(count),
				.BaseShaderRegister = s_cast<UINT>(reg),
				.RegisterSpace = 0U,
				.Flags = flags,
				.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND,
			});
			param() = D3D12_ROOT_PARAMETER1{
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
				.DescriptorTable = {.NumDescriptorRanges = 1U, .pDescriptorRanges = &m_des_range.back()},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}

		// Add a sampler descriptor range parameter
		RootSig& Samp(hlsl::ESamReg reg, int count, D3D12_SHADER_VISIBILITY shader_visibility = D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAGS flags = D3D12_DESCRIPTOR_RANGE_FLAG_NONE)
		{
			m_des_range.push_back(D3D12_DESCRIPTOR_RANGE1{
				.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SAMPLER,
				.NumDescriptors = s_cast<UINT>(count),
				.BaseShaderRegister = s_cast<UINT>(reg),
				.RegisterSpace = 0U,
				.Flags = flags,
				.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND,
			});
			param() = D3D12_ROOT_PARAMETER1{
				.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
				.DescriptorTable = {.NumDescriptorRanges =  1U, .pDescriptorRanges = &m_des_range.back()},
				.ShaderVisibility = shader_visibility,
			};
			return *this;
		}

		// Add a static sampler
		RootSig& Samp(D3D12_STATIC_SAMPLER_DESC const& desc)
		{
			samp() = desc;
			return *this;
		}

		// Compile the shader signature
		D3DPtr<ID3D12RootSignature> Create(ID3D12Device* device, char const* name)
		{
			// Create the root signature
			D3D12_VERSIONED_ROOT_SIGNATURE_DESC rs_desc = {
				.Version = D3D_ROOT_SIGNATURE_VERSION_1_1,
				.Desc_1_1 = {
					.NumParameters = s_cast<UINT>(m_root_params.size()),
					.pParameters = m_root_params.data(),
					.NumStaticSamplers = s_cast<UINT>(m_static_samplers.size()),
					.pStaticSamplers = m_static_samplers.data(),
					.Flags = s_cast<D3D12_ROOT_SIGNATURE_FLAGS>(m_flags),
				},
			};
			
			D3DPtr<ID3DBlob> signature, error;
			auto hr = D3D12SerializeVersionedRootSignature(&rs_desc, signature.address_of(), error.address_of());
			if (error != nullptr) Check(hr, { static_cast<char const*>(error->GetBufferPointer()), error->GetBufferSize() });
			Check(hr, "Create root signature failed");
			
			D3DPtr<ID3D12RootSignature> shader_sig;
			Check(device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), __uuidof(ID3D12RootSignature), (void**)shader_sig.address_of()));
			DebugName(shader_sig, name);
			return shader_sig;
		}

	private:

		D3D12_ROOT_PARAMETER1& param()
		{
			m_root_params.push_back({});
			return m_root_params.back();
		}

		D3D12_STATIC_SAMPLER_DESC& samp()
		{
			m_static_samplers.push_back({});
			return m_static_samplers.back();
		}
	};
}

